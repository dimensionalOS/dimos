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
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import G1_GROOT_KD, G1_GROOT_KP
from dimos.core.core import rpc
from dimos.core.hardware_module import HardwareModuleConfig, Mounts
from dimos.core.stream import In, Out
from dimos.hardware.sensors.lidar.pointlio.module import PointLioConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import UsdModel
from dimos.robot.assets.usd_utils.lfs import pull_model_assets
from dimos.robot.assets.usd_utils.pxr import layer_from, load_pxr
from dimos.robot.unitree.g1.config import G1_JOINT_NAMES
from dimos.robot.unitree.g1.g1_tf_publisher import (
    _TORSO_MID360_XYZ,
    MID360_PITCH,
    G1TfPublisherConfig,
)
from dimos.sim2.actions import JointCommand, world_action
from dimos.sim2.sim import SimulatedWorldSpec
from dimos.sim2.sim_robot import SimRobot
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.world_observations import WorldMirror

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


# Standard gravity in world axes (z up): an accelerometer at rest reads its negative.
_GRAVITY = np.array([0.0, 0.0, -9.81])


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


def usd_joint(name: str) -> str:
    """The G1 USD's name for a control joint: g1/left_knee -> left_knee_joint."""
    return f"{name.split('/', 1)[1]}_joint"


class G1SimConfig(HardwareModuleConfig):
    usd: UsdModel = G1_USD
    # Each joint's gains, in G1_JOINT_NAMES order: GR00T's, as the real G1 runs it.
    kp: tuple[float, ...] = tuple(G1_GROOT_KP)
    kd: tuple[float, ...] = tuple(G1_GROOT_KD)
    publish_rate_hz: float = 500.0
    # The real G1's tf goes out at its mount publisher's rate, not at lowstate's 500 Hz.
    tf_rate_hz: float = G1TfPublisherConfig.model_fields["publish_hz"].default
    tf_frames_local: dict[str, str] = {"body": G1_BODY}
    # The frame its odometry is in, under its namespace, which its mount holds; its OdomTf publishes odom -> body.
    odom_frame: str = G1_ODOM_FRAME


class G1Sim(SimRobot):
    """A G1 in a simulated world; mount it on a frame of the scene, which is where it starts.

    It drives each joint of a position_command to its target with its gains; MuJoCo stops a joint at its limit.
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
        self._world = WorldMirror()
        self._gains = dict(
            zip(G1_JOINT_NAMES, zip(self.config.kp, self.config.kd, strict=True), strict=True)
        )

    @property
    def attach_frame(self) -> str:
        """Its odometry frame: the mount it started on holds odom, and odom holds its body."""
        odom = self.config.odom_frame
        return f"{self.namespace}/{odom}" if self.namespace else odom

    @cached_property
    def _imu_in_body(self) -> np.ndarray:
        """Where the pelvis IMU site sits on the root link, its parent prim, as a 4x4."""
        pxr = load_pxr()
        usd = self.config.usd
        stage = pxr.Usd.Stage.Open(layer_from(pxr, usd.usd, "a robot", usd.asset_root))
        site = stage.GetPrimAtPath(f"/{IMU_FRAME}")
        if not site.IsValid():
            raise KeyError(f"the G1 USD has no IMU site at /{IMU_FRAME}")
        # Gf matrices are row-vector, so transposed.
        return np.array(pxr.UsdGeom.Xformable(site).GetLocalTransformation(), dtype=np.float64).T

    @rpc
    def simulated_robot(self) -> SimulatedRobot:
        return SimulatedRobot(
            self.namespace,
            self.config.usd,
            held_until_commanded=True,
            odom_frame=self.attach_frame,
        )

    @rpc
    def start(self) -> None:
        # Before the world asks for its USD, which opens a stage of every mesh layer it references.
        pull_model_assets(self.config.usd)
        super().start()

    async def main(self) -> AsyncIterator[None]:
        self._world.subscribe(self.world_observations, self.world.snapshot)
        publishing = asyncio.create_task(self._publish())
        posing = asyncio.create_task(self._publish_tf())
        yield
        publishing.cancel()
        posing.cancel()

    async def handle_position_command(self, command: JointState) -> None:
        # Until the world shows this robot, it has no joints to command.
        if self._world.entity(self.namespace) is None:
            return
        gains = [self._gains[name] for name in command.name]
        zeros = (0.0,) * len(gains)
        self.world_actions.publish(
            world_action(
                self._world,
                JointCommand(
                    self.namespace,
                    tuple(usd_joint(name) for name in command.name),
                    position=tuple(command.position),
                    velocity=zeros,
                    effort=zeros,
                    kp=tuple(kp for kp, _ in gains),
                    kd=tuple(kd for _, kd in gains),
                ),
            )
        )

    def _odom(self, stamp: float) -> PoseStamped | None:
        base = self._world.entity(self.namespace)
        if base is None or base.pose is None:
            return None
        return self.odometry(base.pose, stamp, self.attach_frame)

    async def _publish(self) -> None:
        imu_in_body = await asyncio.to_thread(lambda: self._imu_in_body)
        async for _ in self._world.ticks(self.config.publish_rate_hz):
            # Off the event loop, which takes the position commands.
            await asyncio.to_thread(self._publish_tick, imu_in_body)

    def _publish_tick(self, imu_in_body: np.ndarray) -> None:
        world = self._world
        with world.lock:
            names, rows = world.joints(self.namespace)
            base = world.entity(self.namespace)
            pose, velocity, acceleration = (
                (base.pose, base.velocity, base.acceleration)
                if base is not None
                else (None, None, None)
            )
        stamp = time.time()
        by_name = dict(zip(names, rows, strict=True))
        wanted = [usd_joint(name) for name in G1_JOINT_NAMES]
        if not all(joint in by_name for joint in wanted):
            return
        ordered = np.array([by_name[joint] for joint in wanted])
        self.joint_state.publish(
            JointState(
                ts=stamp,
                frame_id=self.tf_frames["body"],
                name=G1_JOINT_NAMES,
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
        stamp = time.time()
        odom = self._odom(stamp)
        if odom is None:
            return
        self.odom.publish(odom)
        positions = {joint: float(row[0]) for joint, row in zip(names, rows, strict=True)}
        transforms = self.moving_transforms(positions, ts=stamp)
        if transforms:
            self.tfbuffer.publish(*transforms)
