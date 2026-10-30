# Copyright 2025-2026 Dimensional Inc.
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

from abc import ABC, abstractmethod
import math
from typing import Any, ClassVar

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.control.components import make_humanoid_joints
from dimos.control.feedback import ControlFeedback, MotorControl, VelocityControllerSpec
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.config import connection_type
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.robot.unitree.g1.dds_device import G1DDSConfig, G1DDSDevice, G1LowStateSnapshot
from dimos.robot.unitree.g1.model import WALKING_DRIVES
from dimos.sim2.client import SimRobotClient
from dimos.sim2.types import Frame, JointTargets, Layout
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1Config(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)
    connection_type: str = Field(
        default_factory=lambda m: connection_type(
            replay=m["g"].replay, simulation=m["g"].simulation
        )
    )
    # Per-device AES-128 key (G1 fw >=1.5.1).
    aes_128_key: str | None = Field(default=None, repr=False)
    sim_prefix: str = "g1"
    control_hz: float = Field(default=50.0, gt=0, allow_inf_nan=False)
    dds: G1DDSConfig = Field(default_factory=G1DDSConfig)


class G1ConnectionBase(Module, ABC):
    """Abstract base for G1 connections (real hardware and simulation).

    Modules that depend on G1 connection RPC methods should reference this
    base class so the blueprint wiring works regardless of which concrete
    connection is deployed.
    """

    config: ModuleConfig

    @rpc
    @abstractmethod
    def start(self) -> None:
        super().start()

    @rpc
    @abstractmethod
    def stop(self) -> None:
        super().stop()

    @rpc
    @abstractmethod
    def move(self, twist: Twist, duration: float = 0.0) -> None: ...

    @rpc
    @abstractmethod
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]: ...


class G1Connection(G1ConnectionBase):
    config: G1Config
    cmd_vel: In[Twist]
    dedicated_worker: ClassVar[bool] = True
    world_layout: In[Layout]
    world_frame: In[Frame]
    targets: Out[JointTargets]
    odom: Out[Odometry]
    control_feedback: Out[ControlFeedback]
    motor_control: In[MotorControl]
    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]
    imu: Out[Imu]
    _dds_device: G1DDSDevice | None = None
    _velocity_controller: VelocityControllerSpec | None = None
    connection: UnitreeWebRTCConnection | None = None
    _sim_client: SimRobotClient | None = None
    _sim_root: int = 0

    @rpc
    def start(self) -> None:
        super().start()

        match self.config.connection_type:
            case "webrtc":
                self.connection = UnitreeWebRTCConnection(
                    self.config.ip, aes_128_key=self.config.aes_128_key
                )
            case "dds":
                self._dds_device = G1DDSDevice(self.config.dds, self._publish_dds_feedback)
                self._dds_device.start()
            case "sim2":
                names = tuple(self.config.sim_prefix + "/" + d.joint for d in WALKING_DRIVES)
                self._sim_client = SimRobotClient(names, self.targets.publish)
            case "replay":
                raise ValueError("Replay connection not implemented for G1 robot")
            case "mujoco":
                raise ValueError(
                    "This module does not support simulation, use G1SimConnection instead"
                )
            case _:
                raise ValueError(f"Unknown connection type: {self.config.connection_type}")

        if self.connection is not None:
            self.connection.start()
        if self._dds_device is None:
            self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

    @rpc
    def stop(self) -> None:
        if self.connection is not None:
            self.connection.stop()
        if self._dds_device is not None:
            self._dds_device.stop()
        super().stop()

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> None:
        if self._sim_client is not None:
            if (
                not all(
                    math.isfinite(v)
                    for v in (twist.linear.x, twist.linear.y, twist.angular.z, duration)
                )
                or duration < 0
            ):
                raise ValueError("velocity/duration must be finite; duration must be nonnegative")
            if self._velocity_controller is None:
                raise ValueError(
                    "velocity control is not configured; attach a controller or send motor_control"
                )
            # Preserve the existing velocity RPC; the coordinator owns its latch/deadman.
            command = Twist(
                linear=Vector3(
                    max(-0.5, min(0.5, twist.linear.x)), max(-0.3, min(0.3, twist.linear.y)), 0
                ),
                angular=Vector3(0, 0, max(-0.6, min(0.6, twist.angular.z))),
            )
            self._velocity_controller.set_velocity_command(command, duration)
        elif self._dds_device is not None:
            raise ValueError("DDS motor control owns the robot; send velocity to its controller")
        else:
            assert self.connection is not None
            self.connection.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        logger.info(f"Publishing request to topic: {topic} with data: {data}")
        if self._sim_client is not None or self._dds_device is not None:
            raise NotImplementedError("firmware RPCs require the stock WebRTC connection")
        assert self.connection is not None
        return self.connection.publish_request(topic, data)  # type: ignore[no-any-return]

    async def handle_world_layout(self, layout: Layout) -> None:
        if self._sim_client is not None:
            self._sim_client.bind(layout)
            self._sim_root = layout.bodies.index(self.config.sim_prefix + "/pelvis")

    async def handle_world_frame(self, frame: Frame) -> None:
        if self._sim_client is not None:
            feedback = self._sim_client.feedback(
                frame,
                self.config.sim_prefix,
                self.config.sim_prefix + "/imu",
                1 / self.config.control_hz,
            )
            if feedback is None:
                return
            self.control_feedback.publish(feedback)
            state = frame.state
            pose = state.poses[self._sim_root]
            inverse = Quaternion(*pose[3:]).conjugate()
            velocity = state.twists[self._sim_root]
            self.odom.publish(
                Odometry(
                    ts=state.wall_time,
                    frame_id="world",
                    child_frame_id=self.config.sim_prefix + "/pelvis",
                    pose=Pose(position=Vector3(*pose[:3]), orientation=Quaternion(*pose[3:])),
                    twist=Twist(
                        linear=inverse.rotate_vector(Vector3(*velocity[:3])),
                        angular=inverse.rotate_vector(Vector3(*velocity[3:])),
                    ),
                )
            )

    async def handle_motor_control(self, command: MotorControl) -> None:
        if self._sim_client is not None:
            self._sim_client.write_motors(command)

    async def handle_motor_command(self, command: MotorCommandArray) -> None:
        if self._dds_device is None:
            raise ValueError("raw motor_command requires DDS; sim2 requires an epoch/tick reply")
        self._dds_device.write(command)

    def _publish_dds_feedback(self, ts: float, sample: G1LowStateSnapshot) -> None:
        # Retain the established DDS wire order and names for existing hardware blueprints.
        self.motor_states.publish(
            JointState(
                ts=ts,
                frame_id=self.config.dds.frame_id,
                name=make_humanoid_joints(self.config.sim_prefix),
                position=sample.positions,
                velocity=sample.velocities,
                effort=sample.efforts,
            )
        )
        w, x, y, z = sample.quaternion
        self.imu.publish(
            Imu(
                ts=ts,
                frame_id=self.config.dds.frame_id,
                orientation=Quaternion(x, y, z, w),
                angular_velocity=Vector3(*sample.gyroscope),
                linear_acceleration=Vector3(*sample.accelerometer),
            )
        )

    @rpc
    def control_status(self) -> dict[str, Any]:
        return {
            "controller": "ControlCoordinator"
            if self._sim_client or self._dds_device
            else "Unitree firmware"
        }
