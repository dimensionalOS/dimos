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
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel, UsdModel
from dimos.robot.unitree.g1.config import G1_JOINT_MODEL, G1_JOINT_NAMES
from dimos.robot.unitree.g1.g1_tf_publisher import _TORSO_MID360_XYZ, MID360_PITCH
from dimos.sim2.actions import JointCommand, world_action
from dimos.sim2.mount_odometry import MountOdometry
from dimos.sim2.mujoco.sensor_world import open_view, robot_ids
from dimos.sim2.mujoco.world import SensorMount
from dimos.sim2.sim import SimulatedWorldSpec, until_it_stops
from dimos.sim2.spec import SimulatedRobot
from dimos.sim2.usd_layer import layer_file
from dimos.sim2.world_observations import WorldMirror

G1_USD = UsdModel.from_file(Path(__file__).parent / "assets" / "g1.usda").with_frames(
    {"lidar": ("torso_link", _TORSO_MID360_XYZ, (math.pi, MID360_PITCH, 0.0))}
)
IMU_FRAME = "imu_in_pelvis"


def _usd_joint(name: str) -> str:
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
    tf_frames_local: dict[str, str] = {"odom": "odom", "body": "pelvis"}
    tf_edges: list[tuple[str, str]] = [("odom", "body")]


# ConnectionModule.frame(name) and TfModule.frame (by role) share a name; G1Sim uses neither, only tf_frames.
class G1Sim(ConnectionModule, HardwareModule):  # type: ignore[misc]
    """A G1 in a simulated world; mount it on a frame of the scene, which is where it starts."""

    config: G1SimConfig
    mounts = Mounts("lidar")
    world: SimulatedWorldSpec
    position_command: In[JointState]
    joint_state: Out[JointState]
    imu: Out[Imu]
    odom: Out[PoseStamped]
    world_observations: In[WorldObservations]
    actions: Out[WorldAction]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._soft_start: float | None = None
        self._odometry: MountOdometry | None = None

    @property
    def joint_names(self) -> list[str]:
        return make_humanoid_joints(self.config.source)

    @property
    def mounted_frame(self) -> str:
        return self.tf_frames["odom"]

    @rpc
    def simulated_robot(self) -> SimulatedRobot:
        return SimulatedRobot(self.namespace, self.config.usd, held_until_commanded=True)

    @rpc
    def start(self) -> None:
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
        yield
        publishing.cancel()

    def connect(self) -> None:
        self._soft_start = None

    def write(self, values: dict[str, float]) -> None:
        now = time.monotonic()
        if self._soft_start is None:
            self._soft_start = now
        duration = self.config.soft_start_seconds
        scale = min(1.0, (now - self._soft_start) / duration) if duration > 0 else 1.0
        joints = self.joint_names
        names = tuple(_usd_joint(name) for name in joints)
        self.actions.publish(
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
        if not hasattr(self, "_world") or not self._world.ids:
            return
        names = tuple(_usd_joint(name) for name in self.joint_names)
        self.actions.publish(
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
            frame_id=self.tf_frames["odom"],
            position=[relative.translation.x, relative.translation.y, relative.translation.z],
            orientation=[
                relative.rotation.x,
                relative.rotation.y,
                relative.rotation.z,
                relative.rotation.w,
            ],
        )

    async def _publish(self) -> None:
        imu_frame = self.usd_frame(IMU_FRAME)
        view = None
        async for _ in self._world.ticks(self.config.publish_rate_hz):
            if view is None:
                layer = await asyncio.to_thread(
                    lambda: layer_file(self.world.get_world_usd(), "world")
                )
                ids = robot_ids(self._world)
                view = await asyncio.to_thread(
                    open_view, layer, ids, SensorMount("imu", imu_frame, imu_frame)
                )
            # Off the event loop: it is the loop that keeps the connection's deadman fed.
            await asyncio.to_thread(self._publish_tick, view, imu_frame)

    def _publish_tick(self, view: Any, imu_frame: str) -> None:
        world = self._world
        with world.lock:
            names, rows = world.joints(self.namespace)
            stamp = world.stamp()
        by_name = dict(zip(names, rows, strict=True))
        wanted = [_usd_joint(name) for name in self.joint_names]
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
        view.update(world)
        gyro, accelerometer, (w, x, y, z) = view.imu(imu_frame)
        self.imu.publish(
            Imu(
                angular_velocity=Vector3(*gyro),
                linear_acceleration=Vector3(*accelerometer),
                orientation=Quaternion(x, y, z, w),
                frame_id=self.tf_frames["body"],
                ts=stamp,
            )
        )
        odom = self._odom(stamp)
        if odom is None:
            return
        self.odom.publish(odom)
        positions = {joint: float(row[0]) for joint, row in by_name.items()}
        self.tfbuffer.publish(
            Transform.from_pose(self.tf_frames["body"], odom),
            *self.usd_transforms(positions, ts=stamp),
        )
