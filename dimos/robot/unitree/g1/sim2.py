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

"""A simulated G1: the real G1 connection's streams, against a G1 the world moves."""

import asyncio
from collections.abc import AsyncIterator
from functools import cached_property
import math
from pathlib import Path
import time
from typing import Any

from dimos_lcm.world_msgs import WorldAction, WorldObservations
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.control.components import make_humanoid_joints
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModule, HardwareModuleConfig, Mounts
from dimos.core.stream import In, Out
from dimos.hardware.sensors.lidar.pointlio.module import PointLioConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel, UsdModel
from dimos.robot.assets.usd_utils.frames import usd_frame_pose
from dimos.robot.assets.usd_utils.lfs import pull_model_assets
from dimos.robot.unitree.g1.config import G1_JOINT_MODEL, G1_JOINT_NAMES
from dimos.robot.unitree.g1.g1_tf_publisher import (
    _TORSO_MID360_XYZ,
    MID360_PITCH,
    G1TfPublisherConfig,
)
from dimos.sim2.actions import JointCommand, world_action
from dimos.sim2.mount_odometry import MountOdometry
from dimos.sim2.mujoco.sensor_world import imu_reading
from dimos.sim2.sim import SimulatedWorldSpec, until_it_stops
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world_observations import OnWorldTime, WorldMirror

# The G1's root link, its body: the pelvis's prim path in its USD, named here so building a blueprint needs no USD read.
G1_BODY = "g1_29dof_rev_1_0/Geometry/pelvis"
G1_TORSO = f"{G1_BODY}/waist_yaw_link/waist_roll_link/torso_link"

G1_USD = UsdModel.from_file(Path(__file__).parent / "assets" / "g1.usda").with_frames(
    {"lidar": (G1_TORSO, _TORSO_MID360_XYZ, (math.pi, MID360_PITCH, 0.0))}
)
# The pelvis IMU: a site fixed on the root link, so no frame of the tf.
IMU_FRAME = f"{G1_BODY}/imu_in_pelvis"

# The frame a real G1 reports its odometry in: Point-LIO's, which it namespaces itself.
G1_ODOM_FRAME: str = PointLioConfig.model_fields["frame_id"].default

# The frames of the odometry tf a real G1 publishes, for the OdomTf beside a simulated one.
G1_ODOM_TF: dict[str, dict[str, str]] = {
    "tf_frames_local": {"odom": G1_ODOM_FRAME, "body": G1_BODY},
}


def usd_joint(name: str) -> str:
    """The G1 USD's name for a control joint: g1/left_knee -> left_knee_joint."""
    return f"{name.split('/', 1)[1]}_joint"


class G1SimConfig(HardwareModuleConfig, ConnectionConfig):
    usd: UsdModel = G1_USD
    source: str = "g1"
    model: RobotModel | None = G1_JOINT_MODEL
    joint_commands: dict[str, tuple[Interface, ...]] = {
        name: (Interface.POSITION,) for name in G1_JOINT_NAMES
    }
    # GR00T's ankle and hip targets can overshoot the joint range; the joint stops at its limit either way.
    clamp_position: bool = True
    kp: tuple[float, ...] = tuple(G1_GROOT_KP)
    kd: tuple[float, ...] = tuple(G1_GROOT_KD)
    # Stiffness ramp-in, as on the real G1 (G1WholeBodyConnectionConfig); off by default since a sim G1 starts standing, not hanging.
    soft_start_seconds: float = Field(default=0.0, ge=0.0)
    publish_rate_hz: float = 500.0
    # The real G1's tf goes out at its mount publisher's rate, not at lowstate's 500 Hz.
    tf_rate_hz: float = G1TfPublisherConfig.model_fields["publish_hz"].default
    tf_frames_local: dict[str, str] = {"body": G1_BODY}
    # The frame its odometry is in, under its namespace, which its mount holds; its OdomTf publishes odom -> body.
    odom_frame: str = G1_ODOM_FRAME
    # Sim seconds between its controller's ticks, which the world holds for; None runs the world free of it.
    control_period: float | None = None


# ConnectionModule.frame(name) and TfModule.frame (by role) share a name; G1Sim uses neither, only tf_frames.
class G1Sim(OnWorldTime, ConnectionModule, HardwareModule):  # type: ignore[misc]
    """A G1 in a simulated world; mount it on a frame of the scene, which is where it starts.

    Its odometry is an estimate, which a HardwareModule never puts on tf: it publishes ``odom``,
    and an ``OdomTf`` beside it (``G1_ODOM_TF``) turns that into odom -> body.
    """

    config: G1SimConfig
    mounts = Mounts(f"{G1_TORSO}/lidar")
    world: SimulatedWorldSpec
    position_command: In[JointState]
    joint_state: Out[JointState]
    imu: Out[Imu]
    odom: Out[PoseStamped]
    world_observations: In[WorldObservations]
    world_actions: Out[WorldAction]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._soft_start: float | None = None
        self._odometry: MountOdometry | None = None

    @property
    def joint_names(self) -> list[str]:
        return make_humanoid_joints(self.config.source)

    @property
    def attach_frame(self) -> str:
        """Its odometry frame: the mount it started on holds odom, and odom holds its body."""
        odom = self.config.odom_frame
        return f"{self.namespace}/{odom}" if self.namespace else odom

    @cached_property
    def _static_edges(self) -> list[Transform]:
        """The USD's static edges below its body, and the mount: odometry holds the body, so the prims above it go out on no edge."""
        body = f"{self.tf_frames[self.config.usd.root_link]}/"
        return [t for t in super()._static_edges if not body.startswith(f"{t.child_frame_id}/")]

    @cached_property
    def _imu_in_body(self) -> np.ndarray:
        """Where the pelvis IMU sits on the root link, as a 4x4; fixed, as the site is."""
        usd = self.config.usd
        root = usd.source_path.parent if usd.source_path is not None else None
        body = usd_frame_pose(usd.usd, usd.root_link, root)
        return np.linalg.inv(body) @ usd_frame_pose(usd.usd, IMU_FRAME, root)

    @rpc
    def simulated_robot(self) -> SimulatedRobot:
        return SimulatedRobot(
            self.namespace,
            self.config.usd,
            held_until_commanded=True,
            odom_frame=self.attach_frame,
            control_period=self.config.control_period,
        )

    @rpc
    def start(self) -> None:
        # Before the world asks for its USD, which opens a stage of every mesh layer it references.
        pull_model_assets(self.config.usd)
        super().start()
        self.register_disposable(
            Disposable(
                self.position_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.POSITION,)))
                )
            )
        )

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_observations, self.world.snapshot)
        publishing = asyncio.create_task(until_it_stops("g1 sim", self._publish()))
        posing = asyncio.create_task(until_it_stops("g1 sim tf", self._publish_tf()))
        yield
        publishing.cancel()
        posing.cancel()

    def connect(self) -> None:
        self._soft_start = None

    def write(self, values: dict[str, float]) -> None:
        now = time.monotonic()
        if self._soft_start is None:
            self._soft_start = now
        duration = self.config.soft_start_seconds
        scale = min(1.0, (now - self._soft_start) / duration) if duration > 0 else 1.0
        joints = self.joint_names
        names = tuple(usd_joint(name) for name in joints)
        self.world_actions.publish(
            world_action(
                self._world,
                JointCommand(
                    self.namespace,
                    names,
                    position=tuple(values[f"{name}/position"] for name in joints),
                    velocity=(0.0,) * len(names),
                    effort=(0.0,) * len(names),
                    kp=tuple(kp * scale for kp in self.config.kp),
                    kd=self.config.kd,
                ),
            )
        )

    def halt(self) -> None:
        """Damping only, as the real G1's halt: no stiffness, so nothing is driven toward a stale target."""
        self._soft_start = None
        # In lockstep the world holds for its next command, so no stale target ever drives it.
        if self.config.control_period is not None:
            return
        if not hasattr(self, "_world") or not self._world.ids:
            return
        names = tuple(usd_joint(name) for name in self.joint_names)
        self.world_actions.publish(
            world_action(
                self._world,
                JointCommand(
                    self.namespace,
                    names,
                    position=(0.0,) * len(names),
                    velocity=(0.0,) * len(names),
                    effort=(0.0,) * len(names),
                    kp=(0.0,) * len(names),
                    kd=self.config.kd,
                ),
            )
        )

    def _odom(self, stamp: float) -> PoseStamped | None:
        base = self._world.entity(self.namespace)
        if base is None or base.pose is None:
            return None
        if self._odometry is None:
            usd = self.config.usd
            self._odometry = MountOdometry.of(usd, usd.root_link)
        relative = self._odometry(base.pose)
        return PoseStamped(
            ts=stamp,
            frame_id=self.attach_frame,
            position=[relative.translation.x, relative.translation.y, relative.translation.z],
            orientation=[
                relative.rotation.x,
                relative.rotation.y,
                relative.rotation.z,
                relative.rotation.w,
            ],
        )

    async def _publish(self) -> None:
        imu_in_body = await asyncio.to_thread(lambda: self._imu_in_body)
        async for _ in self._world.ticks(self.config.publish_rate_hz):
            # Off the event loop: it is the loop that keeps the connection's deadman fed.
            await asyncio.to_thread(self._publish_tick, imu_in_body)

    def _publish_tick(self, imu_in_body: np.ndarray) -> None:
        world = self._world
        with world.lock:
            names, rows = world.joints(self.namespace)
            stamp = world.stamp()
            base = world.entity(self.namespace)
            pose, velocity, acceleration = (
                (base.pose, base.velocity, base.acceleration)
                if base is not None
                else (None, None, None)
            )
        by_name = dict(zip(names, rows, strict=True))
        wanted = [usd_joint(name) for name in self.joint_names]
        if not all(joint in by_name for joint in wanted):
            return
        ordered = np.array([by_name[joint] for joint in wanted])
        self.joint_state.publish(
            JointState(
                ts=stamp,
                frame_id=self.tf_frames["body"],
                name=self.joint_names,
                position=ordered[:, 0].tolist(),
                velocity=ordered[:, 1].tolist(),
                effort=ordered[:, 2].tolist(),
            )
        )
        if pose is None or velocity is None or acceleration is None:
            return
        gyro, accelerometer, (x, y, z, w) = imu_reading(pose, velocity, acceleration, imu_in_body)
        self.imu.publish(
            Imu(
                angular_velocity=Vector3(*gyro),
                linear_acceleration=Vector3(*accelerometer),
                orientation=Quaternion(x, y, z, w),
                frame_id=self.tf_frames["body"],
                ts=stamp,
            )
        )

    async def _publish_tf(self) -> None:
        async for _ in self._world.ticks(self.config.tf_rate_hz):
            await asyncio.to_thread(self._publish_tf_tick)

    def _publish_tf_tick(self) -> None:
        world = self._world
        with world.lock:
            names, rows = world.joints(self.namespace)
            stamp = world.stamp()
        odom = self._odom(stamp)
        if odom is None:
            return
        self.odom.publish(odom)
        positions = {joint: float(row[0]) for joint, row in zip(names, rows, strict=True)}
        transforms = self.moving_transforms(positions, ts=stamp)
        if transforms:
            self.tfbuffer.publish(*transforms)
