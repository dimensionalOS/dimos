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

"""One G1 connection for stock firmware commands or external motor control."""

from __future__ import annotations

import asyncio
from functools import partial
from importlib import import_module
import math
import time
from typing import TYPE_CHECKING, Any, ClassVar, Literal, cast

from pydantic import Field, model_validator
from reactivex.disposable import Disposable

from dimos.control.components import make_humanoid_joints
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.connection.device import ConnectionDevice
from dimos.control.contract.convert import (
    TWIST_INTERFACES,
    motor_command_to_values,
    twist_to_values,
)
from dimos.control.contract.keys import Interface, is_valid_segment
from dimos.control.contract.limits import PD_INTERFACES, Limits, limits as config_limits
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.g1.effectors.high_level.high_level_spec import G1StockDevice
from dimos.robot.unitree.g1.motor_device import G1MotorDevice

if TYPE_CHECKING:
    from dimos.sim2.spec import RobotInstanceSpec


class G1ConnectionConfig(ConnectionConfig):
    source: str = "g1"
    control_mode: Literal["stock", "motors"] = "stock"
    stock_transport: Literal["webrtc", "dds"] = "webrtc"
    ip: str = Field(default_factory=lambda values: values["g"].robot_ip)
    aes_128_key: str | None = Field(default=None, repr=False)
    connection_mode: str = "ai"
    network_interface: str = ""
    ai_standup: bool = True
    motion_switcher_timeout: float = Field(default=5.0, gt=0, allow_inf_nan=False)
    loco_client_timeout: float = Field(default=10.0, gt=0, allow_inf_nan=False)
    min_effective_linear_velocity: float = Field(default=0.05, ge=0, allow_inf_nan=False)
    min_effective_angular_velocity: float = Field(default=0.2, ge=0, allow_inf_nan=False)
    deadman_timeout_s: float = Field(default=0.2, gt=0, allow_inf_nan=False)
    halt_damping: float = Field(default=5.0, ge=0, allow_inf_nan=False)
    release_sport_mode: bool = True
    publish_rate_hz: float = Field(default=500.0, gt=0, allow_inf_nan=False)
    frame_id: str = "g1_pelvis"
    mode_machine: int = 5
    soft_start_seconds: float = Field(default=3.0, ge=0, allow_inf_nan=False)
    initial_qpos: tuple[float, ...] = ()
    startup_kp: tuple[float, ...] = ()
    startup_kd: tuple[float, ...] = ()
    wait_for_first_command: bool = False
    sim_spawn: str = "default"
    sim_position: tuple[float, float, float] | None = None
    sim_orientation_rpy: tuple[float, float, float] | None = None

    @model_validator(mode="after")
    def resolve_command_mode(self) -> G1ConnectionConfig:
        if not is_valid_segment(self.source):
            raise ValueError("source must contain only letters, digits and underscores")
        self.pd = self.control_mode == "motors"
        return self


class G1Connection(ConnectionModule):
    """Own one G1 device; the external controller is independent of the connection.

    Stock mode accepts body velocities and vendor requests. Motor mode accepts
    complete PD commands and never enables the stock firmware controller.
    Simulation replaces the device while retaining this module and its checks.
    """

    config: G1ConnectionConfig
    default_sim_scene: ClassVar[str] = "logistics.xml"
    cmd_vel: In[Twist]
    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]
    imu: Out[Imu]
    odom: Out[PoseStamped]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._joint_names = tuple(make_humanoid_joints(self.config.source))
        self._velocity_generation = 0
        self._velocity_timer: asyncio.TimerHandle | None = None

    @staticmethod
    def describe_simulated_robot(config: G1ConnectionConfig) -> RobotInstanceSpec:
        # Model adapters are optional simulation dependencies, loaded only for a sim build.
        model = import_module("dimos.robot.unitree.g1.model")
        return cast("RobotInstanceSpec", model.describe_simulated_robot(config))

    @property
    def limits(self) -> dict[str, Limits]:
        if self._limits is None:
            if self.config.control_mode == "motors":
                # A PD position is a virtual spring target; its controller owns joint limits.
                self._limits = {
                    f"{joint}/{interface}": (
                        Limits(lo=0.0) if interface in (Interface.KP, Interface.KD) else Limits()
                    )
                    for joint in make_humanoid_joints(self.config.source)
                    for interface in PD_INTERFACES
                }
            elif self.config.base_range is not None:
                self._limits = config_limits(self.config)
            else:
                self._limits = {
                    f"{self.config.source}/base/{axis}": Limits() for axis in TWIST_INTERFACES
                }
        return self._limits

    def create_hardware_device(self) -> ConnectionDevice:
        if self.config.control_mode == "motors":
            return G1MotorDevice(self.config, self.publish_feedback)
        if self.config.stock_transport == "dds":
            # DDS is optional on machines only using WebRTC or simulation.
            backend = import_module("dimos.robot.unitree.g1.effectors.high_level.dds_sdk")
            return cast("ConnectionDevice", backend.G1DdsDevice(self.config))
        backend = import_module("dimos.robot.unitree.g1.effectors.high_level.webrtc")
        return cast("ConnectionDevice", backend.G1WebRtcDevice(self.config))

    @rpc
    def start(self) -> None:
        if self.config.g.replay:
            raise ValueError("G1 replay is not implemented; refusing to open a live device")
        super().start()
        try:
            # Direct RPC callers need no input transport; blueprints wire one.
            if self.config.control_mode == "stock":
                if self.cmd_vel.transport is not None:
                    self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
            elif self.motor_command.transport is not None:
                self.register_disposable(
                    Disposable(self.motor_command.subscribe(self._on_motor_command))
                )
        except BaseException:
            self.stop()
            raise

    @rpc
    def stop(self) -> None:
        self._cancel_velocity()
        super().stop()

    def halt(self) -> None:
        self._cancel_velocity()
        super().halt()

    def _cancel_velocity(self) -> None:
        with self._hardware_lock:
            self._velocity_generation += 1
            self._on_loop(self._cancel_velocity_timer)

    def _cancel_velocity_timer(self) -> None:
        with self._hardware_lock:
            if self._velocity_timer is not None:
                self._velocity_timer.cancel()
                self._velocity_timer = None

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a stock body velocity, optionally refreshing it for a bounded duration."""
        if self.config.control_mode != "stock":
            self._report("stock velocity commands require control_mode='stock'", rejection=True)
            return False
        if not math.isfinite(duration) or duration < 0:
            self._report("movement duration must be finite and nonnegative", rejection=True)
            return False
        values = twist_to_values(twist, f"{self.config.source}/base")
        with self._hardware_lock:
            if not self.command(values):
                return False
            self._velocity_generation += 1
            generation = self._velocity_generation
            self._on_loop(self._cancel_velocity_timer)
            if duration > 0:
                self._on_loop(
                    partial(
                        self._schedule_velocity, generation, values, time.monotonic() + duration
                    )
                )
            return True

    def _schedule_velocity(
        self, generation: int, values: dict[str, float], deadline: float
    ) -> None:
        with self._hardware_lock:
            if generation != self._velocity_generation:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.command(twist_to_values(Twist(), f"{self.config.source}/base"))
                self._velocity_timer = None
                return
            delay = min(remaining, self.config.deadman_timeout_s / 2)
            self._velocity_timer = asyncio.get_running_loop().call_later(
                delay, self._refresh_velocity, generation, values, deadline
            )

    def _refresh_velocity(self, generation: int, values: dict[str, float], deadline: float) -> None:
        with self._hardware_lock:
            if generation != self._velocity_generation:
                return
            if time.monotonic() >= deadline:
                self._schedule_velocity(generation, values, deadline)
            elif self.command(values):
                self._schedule_velocity(generation, values, deadline)

    @rpc
    def stop_movement(self) -> None:
        """Cancel a timed move and halt the selected device."""
        self._cancel_velocity()
        with self._hardware_lock:
            self._call_halt()

    def _on_motor_command(self, message: MotorCommandArray) -> None:
        if self.config.control_mode != "motors":
            self._report("motor commands require control_mode='motors'", rejection=True)
            return
        columns = (message.q, message.dq, message.kp, message.kd, message.tau)
        if any(len(column) != len(self._joint_names) for column in columns):
            self._report(
                "G1 motor commands need one value per motor in every column", rejection=True
            )
            return
        try:
            values = motor_command_to_values(message, self._joint_names)
        except (ValueError, TypeError) as error:
            self._report(str(error), rejection=True)
            return
        self.command(values)

    def _stock(self) -> G1StockDevice:
        if self.config.control_mode != "stock":
            raise RuntimeError("stock firmware requests require control_mode='stock'")
        if self.config.g.simulation:
            raise NotImplementedError(
                "the stock simulation models locomotion, not Unitree firmware gestures or state transitions"
            )
        if not isinstance(self._device, G1StockDevice):
            raise RuntimeError("G1 stock device is not connected")
        return self._device

    @rpc
    def disconnect(self) -> None:
        self.stop()

    @rpc
    def get_state(self) -> str:
        if not self.status().connected:
            return "Not connected"
        if self.config.control_mode == "motors":
            return "External motor control"
        if self.config.g.simulation:
            return "Simulated stock locomotion"
        with self._hardware_lock:
            return self._stock().get_state()

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[str, Any]:
        if self.config.control_mode != "stock":
            raise RuntimeError("stock firmware requests require control_mode='stock'")
        if topic in ("rt/api/sport/request", "rt/api/loco/request") and data.get("api_id") == 7105:
            parameter = data["parameter"]
            vx, vy, wz = parameter["velocity"]
            accepted = self.move(
                Twist(linear=Vector3(vx, vy, 0), angular=Vector3(0, 0, wz)),
                duration=float(parameter.get("duration", 0.0)),
            )
            if not accepted:
                raise ValueError("G1 rejected the velocity request; see connection status")
            return {"code": 0}
        with self._hardware_lock:
            return self._stock().publish_request(topic, data)

    @rpc
    def stand_up(self) -> bool:
        with self._hardware_lock:
            stock = self._stock()
            self.stop_movement()
            return stock.stand_up()

    @rpc
    def lie_down(self) -> bool:
        with self._hardware_lock:
            stock = self._stock()
            self.stop_movement()
            return stock.lie_down()
