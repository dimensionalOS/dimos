# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""A simulated robot body: joint commands in as world_actions, joint states and its IMU out from world_observations, and mounts its sensors ride on."""

import asyncio
from collections.abc import AsyncIterator, Mapping, Sequence
from functools import cached_property
import time
from typing import Any, ClassVar

from dimos_lcm.world_msgs import WorldAction, WorldObservations
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.frames import usd_frame_pose
from dimos.robot.assets.usd_utils.lfs import pull_model_assets
from dimos.robot.assets.usd_utils.pxr import Prim, layer_from, load_pxr
from dimos.sim2.mujoco.drives import usd_drives
from dimos.sim2.sim import (
    JointCommand,
    SimulatedRobot,
    SimulatedWorldSpec,
    WorldMirror,
    world_action,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Isaac Sim's Robot Schema marks a frame other things attach to (a sensor, a tool) as a site.
ISAAC_SITE_API = "IsaacSiteAPI"

# Standard gravity in world axes (z up): an accelerometer at rest reads its negative.
_GRAVITY = np.array([0.0, 0.0, -9.81])


def with_sites(usd: UsdModel, sites: Mapping[str, tuple[str, str]]) -> UsdModel:
    """``usd`` with each frame of ``sites`` (frame path -> (description, forward axis)) tagged as an Isaac Sim site."""
    # Before its layer is opened, which opens every mesh layer it references.
    pull_model_assets(usd)
    pxr = load_pxr()
    layer = layer_from(pxr, usd.usd, "a robot", usd.asset_root)
    stage = pxr.Usd.Stage.Open(layer)
    for frame, (description, forward_axis) in sites.items():
        prim = stage.GetPrimAtPath(f"/{frame}")
        if not prim.IsValid():
            raise KeyError(f"no prim /{frame} to make a site of")
        prim.AddAppliedSchema(ISAAC_SITE_API)
        prim.CreateAttribute("isaac:description", pxr.Sdf.ValueTypeNames.String).Set(description)
        forward = prim.CreateAttribute(
            "isaac:forwardAxis",
            pxr.Sdf.ValueTypeNames.Token,
            variability=pxr.Sdf.VariabilityUniform,
        )
        forward.Set(forward_axis)
    return UsdModel.from_layer(str(layer.ExportToString()), usd.source_path)


def is_site(prim: Prim) -> bool:
    """Whether ``prim`` applies IsaacSiteAPI, read off its apiSchemas: Isaac Sim's schemas are not registered here."""
    listed = prim.GetMetadata("apiSchemas")
    return listed is not None and ISAAC_SITE_API in listed.ApplyOperations([])


def imu_reading(
    pose: NDArray[np.float64],
    velocity: NDArray[np.float64],
    acceleration: NDArray[np.float64],
    imu_in_body: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64], NDArray[np.float64]]:
    """The gyro, accelerometer (specific force) and x, y, z, w orientation of an IMU fixed at ``imu_in_body`` (a 4x4) on a body.

    ``pose`` is the body's x, y, z, qx, qy, qz, qw; ``velocity`` and ``acceleration`` are linear then
    angular, world axes, at the body's origin, without gravity: a BodyState as world_observations carries it.
    """
    world_from_body = Rotation.from_quat(pose[3:7]).as_matrix()
    world_from_imu = world_from_body @ imu_in_body[:3, :3]
    angular_velocity, linear_acceleration = velocity[3:], acceleration[:3]
    lever = world_from_body @ imu_in_body[:3, 3]
    # A point fixed on the body: tangential and centripetal acceleration on top of the origin's.
    at_imu = (
        linear_acceleration
        + np.cross(acceleration[3:], lever)
        + np.cross(angular_velocity, np.cross(angular_velocity, lever))
    )
    gyro = world_from_imu.T @ angular_velocity
    accelerometer = world_from_imu.T @ (at_imu - _GRAVITY)
    return gyro, accelerometer, Rotation.from_matrix(world_from_imu).as_quat()


class SimBodyConfig(HardwareModuleConfig):
    # Its joints as joint_state and position_command name them (<hardware>/<joint>), in order; the USD calls each <joint>_joint.
    joint_names: tuple[str, ...]
    # Each joint's stiffness and damping, in joint_names order; None is its USD drive's.
    kp: tuple[float, ...] | None = None
    kd: tuple[float, ...] | None = None
    # The IMU's site on its root link (a prim path of its USD), or None for a body without one.
    imu_site: str | None = None
    # How often its odometry and moving joints go out on tf.
    tf_rate_hz: float = 50.0
    # The frame its odometry is in, under its namespace, which its mount holds; an OdomTf beside it publishes odom -> body.
    odom_frame: str = "odom"


class SimBody(HardwareModule):
    """A body the world moves; mount it on a frame of the scene, which is where it starts.

    It drives each joint of a position_command to its target with its gains. Its odometry (where its
    root link is, from the mount it started on) is an estimate, which a HardwareModule never puts on
    tf: it publishes ``odom``, and an ``OdomTf`` beside it turns that into odom -> body.
    """

    config: SimBodyConfig
    # Its joint states are what a policy balances on: they share no GIL with the physics or a sensor, which would make them late.
    dedicated_worker: ClassVar[bool] = True
    world: SimulatedWorldSpec
    position_command: In[JointState]
    joint_state: Out[JointState]
    imu: Out[Imu]
    odom: Out[Odometry]
    world_observations: In[WorldObservations]
    world_actions: Out[WorldAction]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._world = WorldMirror()
        self._gains = self._joint_gains()
        self._mount_inverse: NDArray[np.float64] | None = None

    def _joint_gains(self) -> dict[str, tuple[float, float]]:
        """Each joint's (kp, kd): the config's, or else its USD drive's."""
        config = self.config
        names = config.joint_names
        if config.kp is None or config.kd is None:
            usd = config.usd
            drives = usd_drives(usd.usd, usd.asset_root)
            missing = [name for name in names if self.usd_joint(name) not in drives]
            if missing:
                raise ValueError(f"no kp and kd given, and the USD drives none of {missing}")
            usd_gains = [drives[self.usd_joint(name)][:2] for name in names]
        kp = config.kp if config.kp is not None else tuple(kp for kp, _ in usd_gains)
        kd = config.kd if config.kd is not None else tuple(kd for _, kd in usd_gains)
        if not len(names) == len(kp) == len(kd):
            raise ValueError(
                f"{len(names)} joints need as many kp and kd, not {len(kp)} and {len(kd)}"
            )
        return dict(zip(names, zip(kp, kd, strict=True), strict=True))

    @staticmethod
    def usd_joint(name: str) -> str:
        """The USD's name for a joint as its streams name it: g1/left_knee -> left_knee_joint."""
        return f"{name.partition('/')[2]}_joint"

    @property
    def attach_frame(self) -> str:
        """Its odometry frame: the mount it started on holds odom, and odom holds its body."""
        odom = self.config.odom_frame
        return f"{self.namespace}/{odom}" if self.namespace else odom

    @cached_property
    def body_frame(self) -> str:
        """Its root link's tf frame, which its odometry holds; worked out once, as reading the root link opens its USD."""
        return self.tf_frames[self.config.usd.root_link]

    @cached_property
    def _static_edges(self) -> list[Transform]:
        """The USD's static edges below its body, and the mount: odometry holds the body, so the prims above it go out on no edge."""
        below = f"{self.config.usd.root_link}/"
        edges = [e for e in self.config.usd.static_transforms() if e.child.startswith(below)]
        mount = (self.namespace, self.attach_frame)
        return [
            *self._usd_edges_as_namespaced_transforms(edges, None),
            *(t for t in super()._static_edges if (t.frame_id, t.child_frame_id) == mount),
        ]

    @cached_property
    def _body_on_mount(self) -> NDArray[np.float64]:
        """Where its root link stands on its mount, as a 4x4: odometry starts there, its standing height above odom."""
        usd = self.config.usd
        return usd_frame_pose(usd.usd, usd.root_link, usd.asset_root)

    @cached_property
    def _imu_in_body(self) -> NDArray[np.float64]:
        """Where its IMU site sits on its root link, the site's parent prim, as a 4x4."""
        pxr = load_pxr()
        usd = self.config.usd
        stage = pxr.Usd.Stage.Open(layer_from(pxr, usd.usd, "a robot", usd.asset_root))
        site = stage.GetPrimAtPath(f"/{self.config.imu_site}")
        if not site.IsValid():
            raise KeyError(f"the body's USD has no IMU site at /{self.config.imu_site}")
        # Gf matrices are row-vector, so transposed.
        return np.array(pxr.UsdGeom.Xformable(site).GetLocalTransformation(), dtype=np.float64).T

    @rpc
    def simulated_robot(self) -> SimulatedRobot:
        return SimulatedRobot(
            self.namespace, self.config.usd, held_until_commanded=True, odom_frame=self.attach_frame
        )

    @rpc
    def start(self) -> None:
        # Before the world asks for its USD, which opens a stage of every mesh layer it references.
        pull_model_assets(self.config.usd)
        super().start()

    async def main(self) -> AsyncIterator[None]:
        imu_in_body = (
            None
            if self.config.imu_site is None
            else await asyncio.to_thread(lambda: self._imu_in_body)
        )
        # Read on every frame of the world as it comes, so no timer (which macOS lets a background process oversleep) makes a reading late.
        self._world.on_frame(lambda _: self._publish_reading(imu_in_body))
        self._world.subscribe(self.world_observations, self.world.snapshot)
        posing = asyncio.create_task(self._pose())
        yield
        posing.cancel()

    async def handle_position_command(self, command: JointState) -> None:
        # Until the world shows this robot, it has no joints to command.
        if self._world.entity(self.namespace) is None:
            return
        unknown = set(command.name) - self._gains.keys()
        if unknown:
            logger.warning(
                f"{self.namespace}: a position_command for joints it has not: {sorted(unknown)}"
            )
            return
        gains = [self._gains[name] for name in command.name]
        zeros = (0.0,) * len(gains)
        self.world_actions.publish(
            world_action(
                self._world,
                JointCommand(
                    self.namespace,
                    tuple(self.usd_joint(name) for name in command.name),
                    position=tuple(command.position),
                    velocity=zeros,
                    effort=zeros,
                    kp=tuple(kp for kp, _ in gains),
                    kd=tuple(kd for _, kd in gains),
                ),
            )
        )

    def _publish_reading(self, imu_in_body: NDArray[np.float64] | None) -> None:
        """Its joint states, and what its IMU reads, from the world as its latest frame has it."""
        world = self._world
        with world.lock:
            names, rows = world.joints(self.namespace)
            base = world.entity(self.namespace)
            state = None if base is None else (base.pose, base.velocity, base.acceleration)
        stamp = time.time()
        by_name = dict(zip(names, rows, strict=True))
        wanted = [self.usd_joint(name) for name in self.config.joint_names]
        if not all(joint in by_name for joint in wanted):
            return
        ordered = np.array([by_name[joint] for joint in wanted])
        self.joint_state.publish(
            JointState(
                ts=stamp,
                frame_id=self.body_frame,
                name=list(self.config.joint_names),
                position=ordered[:, 0].tolist(),
                velocity=ordered[:, 1].tolist(),
                effort=ordered[:, 2].tolist(),
            )
        )
        if imu_in_body is None or state is None or any(part is None for part in state):
            return
        pose, velocity, acceleration = state
        gyro, accelerometer, (x, y, z, w) = imu_reading(pose, velocity, acceleration, imu_in_body)
        self.imu.publish(
            Imu(
                angular_velocity=Vector3(*gyro),
                linear_acceleration=Vector3(*accelerometer),
                orientation=Quaternion(x, y, z, w),
                # A site is no tf frame: the IMU's is its body's, which it reads unturned on.
                frame_id=self.body_frame,
                ts=stamp,
            )
        )

    async def _pose(self) -> None:
        async for _ in self._world.ticks(self.config.tf_rate_hz):
            self._publish_pose()

    def _publish_pose(self) -> None:
        """Its odometry, and its joints' edges on tf."""
        world = self._world
        with world.lock:
            names, rows = world.joints(self.namespace)
            base = world.entity(self.namespace)
            state = None if base is None else (base.pose, base.velocity)
        if state is None or state[0] is None or state[1] is None:
            return
        stamp = time.time()
        self.odom.publish(self.odometry(state[0], state[1], stamp))
        positions = {joint: float(row[0]) for joint, row in zip(names, rows, strict=True)}
        transforms = self.moving_transforms(positions, ts=stamp)
        if transforms:
            self.tfbuffer.publish(*transforms)

    def odometry(
        self,
        pose: Sequence[float] | NDArray[np.float64],
        velocity: Sequence[float] | NDArray[np.float64],
        stamp: float,
    ) -> Odometry:
        """Its root link in its odom frame (the mount it started on is odom's origin) and its twist in its own frame, given its world pose (x, y, z, qx, qy, qz, qw) and velocity (linear then angular, world axes)."""
        here = Transform(
            translation=Vector3(*pose[:3]), rotation=Quaternion(*pose[3:7])
        ).to_matrix()
        if self._mount_inverse is None:
            self._mount_inverse = np.linalg.inv(here @ np.linalg.inv(self._body_on_mount))
        relative = Transform.from_matrix(self._mount_inverse @ here)
        body_from_world = here[:3, :3].T
        linear, angular = (
            body_from_world @ np.asarray(part) for part in (velocity[:3], velocity[3:6])
        )
        return Odometry(
            ts=stamp,
            frame_id=self.attach_frame,
            child_frame_id=self.body_frame,
            pose=Pose(relative.translation, relative.rotation),
            twist=Twist(Vector3(*linear), Vector3(*angular)),
        )
