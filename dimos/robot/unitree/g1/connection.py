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
from typing import Any, ClassVar

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.control.components import make_humanoid_joints
from dimos.control.feedback import ControlFeedback, MotorControl, VelocityControllerSpec
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.config import connection_type
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.robot.unitree.g1.dds_device import G1DDSConfig, G1DDSDevice
from dimos.robot.unitree.g1.model import G1_MOTOR_JOINTS
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
    robot_name: str = "g1"
    legacy_motor_streams: bool = False  # older hardware blueprints using the LCM adapter
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
    _motor_device: SimRobotClient | G1DDSDevice | None = None

    @rpc
    def start(self) -> None:
        if self.config.legacy_motor_streams and self.config.connection_type != "dds":
            raise ValueError("legacy motor streams require DDS")
        super().start()

        match self.config.connection_type:
            case "webrtc":
                self.connection = UnitreeWebRTCConnection(
                    self.config.ip, aes_128_key=self.config.aes_128_key
                )
            case "dds":
                self._dds_device = G1DDSDevice(
                    self.config.dds, self._publish_feedback, prefix=self.config.robot_name
                )
                self._motor_device = self._dds_device
                self._dds_device.start()
            case "sim2":
                names = tuple(self.config.robot_name + "/" + name for name in G1_MOTOR_JOINTS)
                self._sim_client = SimRobotClient(
                    names,
                    self.targets.publish,
                    imu_frame=self.config.robot_name + "/imu",
                    root_frame=self.config.robot_name + "/pelvis",
                )
                self._motor_device = self._sim_client
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
        if not self.config.legacy_motor_streams:
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
        if self._motor_device is not None:
            if self._velocity_controller is None:
                raise ValueError(
                    "velocity control is not configured; attach a controller or send motor_control"
                )
            self._velocity_controller.set_velocity_command(twist, duration)
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

    async def handle_world_frame(self, frame: Frame) -> None:
        if self._sim_client is not None:
            feedback = self._sim_client.feedback(frame)
            if feedback is not None:
                self._publish_feedback(feedback)
                odom = self._sim_client.odometry()
                if odom is not None:
                    self.odom.publish(odom)

    async def handle_motor_control(self, command: MotorControl) -> None:
        if self._motor_device is not None and not self.config.legacy_motor_streams:
            self._motor_device.write_motors(command)

    async def handle_motor_command(self, command: MotorCommandArray) -> None:
        if self._dds_device is None or not self.config.legacy_motor_streams:
            raise ValueError("raw motor_command requires the legacy DDS stream configuration")
        self._dds_device.write(command)

    def _publish_feedback(self, sample: ControlFeedback) -> None:
        if self.config.legacy_motor_streams:
            # Preserve the old adapter's names only at its compatibility boundary.
            joints = sample.joints
            self.motor_states.publish(
                JointState(
                    ts=joints.ts,
                    frame_id=joints.frame_id,
                    name=make_humanoid_joints(self.config.robot_name),
                    position=joints.position,
                    velocity=joints.velocity,
                    effort=joints.effort,
                )
            )
            if sample.imu is not None:
                self.imu.publish(sample.imu)
        else:
            self.control_feedback.publish(sample)

    @rpc
    def control_status(self) -> dict[str, Any]:
        return {
            "controller": "external motor controller"
            if self._sim_client or self._dds_device
            else "Unitree firmware"
        }
