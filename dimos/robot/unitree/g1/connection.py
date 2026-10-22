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

"""G1's public robot interface; SDK and simulated motor IO are internal devices."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import threading
import time
from typing import Any, ClassVar, Literal, Protocol, cast

import numpy as np
from pydantic import BaseModel, Field, model_validator
from typing_extensions import Self

from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.contract.convert import JOINT_STATE_FIELDS, imu_to_values, joint_state_to_values
from dimos.control.contract.description import Limits
from dimos.control.contract.ports import MotorPort, TwistPort
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.control_msgs.ControlValues import ControlValues
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import RobotModel
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.robot.unitree.g1.controllers.groot.config import G1_GROOT_HOME, G1_GROOT_KD, G1_GROOT_KP
from dimos.robot.unitree.g1.devices.motors import G1MotorConfig, G1MotorDevice
from dimos.robot.unitree.g1.model.config import G1_URDF_PATH
from dimos.robot.unitree.g1.model.joints import g1_joints
from dimos.robot.unitree.g1.model.simulation import g1_definition
from dimos.sim2.client import SimRobotClient
from dimos.sim2.messages import StreamDescriptor
from dimos.sim2.module import SimulationSpec
from dimos.sim2.spec import RobotInstance
from dimos.sim2.world.scene import scene_robot
from dimos.spec.utils import Spec
from dimos.utils.data import LfsPath

_JOINTS = [name.split("/", 1)[1] for name in g1_joints]
_MODEL = RobotModel.from_file(
    G1_URDF_PATH, package_paths={"g1": LfsPath("g1_urdf")}
).with_renamed_joints({name + "_joint": name for name in _JOINTS})


class G1LocomotionSpec(Spec, Protocol):
    def set_velocity_command(self, vx: float, vy: float, yaw_rate: float) -> None: ...
    def set_estop(self, estopped: bool) -> Any: ...


class G1Config(ConnectionConfig):
    source: str = "g1"
    model: RobotModel | None = _MODEL
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)
    controller: Literal["stock", "groot"] = Field(
        default_factory=lambda m: "groot" if m["g"].simulation == "mujoco" else "stock"
    )
    aes_128_key: str | None = Field(default=None, repr=False)
    motors: G1MotorConfig = Field(
        default_factory=lambda: G1MotorConfig(command_timeout_seconds=0.25)
    )
    cmd_vel: TwistPort | None = None
    motor_command: MotorPort | None = None
    spawn: str = "default"
    frame_id: str = "g1_pelvis"
    deadman_timeout_s: float = 0.25

    @model_validator(mode="after")
    def _control_ports(self) -> Self:
        if self.controller == "stock":
            self.cmd_vel = TwistPort(range=Twist(Vector3(1.0, 0.5, 0), Vector3(0, 0, 1.5)))
            self.motor_command = None
        else:
            self.cmd_vel = None
            # Policy PD targets are virtual spring equilibria, not kinematic
            # position requests. Preserve vendor motor semantics; measured
            # overspeed, finite commands and actuator torque limits still apply.
            self.motor_command = MotorPort(
                joints=_JOINTS,
                limit_overrides={f"{joint}/position": Limits() for joint in _JOINTS},
            )
        return self


class G1Connection(ConnectionModule):
    """One public connection for stock firmware or GR00T.

    ``move`` / ``cmd_vel`` always express walking intent. A custom controller
    produces motor commands; only this module owns the SDK or SimRobotClient.
    Full world odometry is an observation, not the planar control contract.
    """

    config: G1Config
    default_sim_scene: ClassVar[str] = "logistics.xml"
    _simulation: SimulationSpec | None
    _locomotion: G1LocomotionSpec | None

    cmd_vel: In[Twist]
    motor_command: In[MotorCommandArray]
    controller_fault: In[String]
    motor_states: Out[JointState]
    imu: Out[Imu]
    odom: Out[PoseStamped]
    tf: Out[TFMessage]
    g1_fault: Out[String]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._simulation = None
        self._locomotion = None
        self._sim: SimRobotClient | None = None
        self._motors: G1MotorDevice | None = None
        self._stock: UnitreeWebRTCConnection | None = None
        self._device_lock = threading.RLock()
        self._feedback: dict[str, float] = {}
        self._feedback_at = 0.0
        self._imu_at = 0.0
        self._sequence = -1
        self._estopped = False
        self._move_timer: threading.Timer | None = None
        self._names = [f"{self.config.source}/{name}" for name in _JOINTS]

    @staticmethod
    def describe_simulated_robot(config: BaseModel, scene: Path) -> RobotInstance:
        config = cast("G1Config", config)
        if config.controller == "stock":
            raise ValueError("MuJoCo requires groot; stock Unitree firmware is not emulated")
        home, kp, kd = G1_GROOT_HOME, G1_GROOT_KP, G1_GROOT_KD
        instance = scene_robot(
            scene,
            g1_definition(config.source),
            config.spawn,
            default=(0.0, 0.0, 0.0),
        )
        names = [joint.name for joint in instance.config.joints]
        return replace(
            instance,
            initial_positions=dict(zip(names, map(float, home), strict=True)),
            hold_gains={
                name: (float(p), float(d)) for name, p, d in zip(names, kp, kd, strict=True)
            },
            control_imu="imu",
            wait_for_first_command=True,
            controller={
                "name": config.controller,
                "startup_configuration": G1Connection.describe_simulated_robot,
            },
        )

    def control_readings(self) -> dict[type, str]:
        return {} if self.config.controller == "stock" else {JointState: "motor_states", Imu: "imu"}

    def connect(self) -> None:
        if self.config.g.simulation == "mujoco":
            if self._simulation is None:
                raise RuntimeError("G1 simulated device requires the collected SimulationModule")
            robot_id = self.config.instance_name or self.name
            descriptor = self._simulation.describe()["robots"][robot_id]
            self._sim = SimRobotClient(StreamDescriptor.from_dict(descriptor), writable=True)
            self._sim.connect()
        elif self.config.g.simulation:
            raise ValueError("G1Connection only supports real hardware or MuJoCo")
        elif self.config.controller == "stock":
            self._stock = UnitreeWebRTCConnection(
                self.config.ip, aes_128_key=self.config.aes_128_key
            )
            self._stock.start()
        else:
            self._motors = G1MotorDevice(
                self.config.motors,
                publish_joints=self._receive_joints,
                publish_imu=self._receive_imu,
                publish_fault=self.g1_fault.publish,
            )
            self._motors.start()

    def _receive_joints(self, msg: JointState) -> None:
        msg.name = self._names
        with self._device_lock:
            self._feedback.update(joint_state_to_values(msg, JOINT_STATE_FIELDS))
            self._feedback_at = msg.ts

    def _receive_imu(self, msg: Imu) -> None:
        with self._device_lock:
            self._feedback.update(imu_to_values(msg, f"{self.config.source}/imu"))
            self._imu_at = msg.ts

    def read_state(self) -> dict[str, float] | ControlValues:
        with self._device_lock:
            if self._sim is None:
                acquired = min(self._feedback_at, self._imu_at)
                if time.time() - acquired > self.config.motors.feedback_timeout_seconds:
                    return {}
                return ControlValues(
                    self.config.source,
                    acquired,
                    interface_names=list(self._feedback),
                    values=list(self._feedback.values()),
                )
            frame = self._sim.read()
            if frame.metadata.sequence == self._sequence:
                return {}
            self._sequence = frame.metadata.sequence
            v = frame.values
            ts = float(v["wall_time"][0])
            joints = JointState(
                name=self._names,
                position=v["position"].tolist(),
                velocity=v["velocity"].tolist(),
                effort=v["effort"].tolist(),
                ts=ts,
            )
            w, x, y, z = v["imu_quaternion"]
            imu = Imu(
                orientation=Quaternion(x, y, z, w),
                angular_velocity=Vector3(*v["imu_gyroscope"]),
                linear_acceleration=Vector3(*v["imu_accelerometer"]),
                frame_id=self.frame_id,
                ts=ts,
            )
            w, x, y, z = v["root_quaternion"]
            pose = PoseStamped(
                position=Vector3(*v["root_position"]),
                orientation=Quaternion(x, y, z, w),
                frame_id="world",
                ts=ts,
            )
            self.odom.publish(pose)
            self.tf.publish(
                TFMessage(
                    Transform(
                        translation=pose.position,
                        rotation=pose.orientation,
                        frame_id="world",
                        child_frame_id=self.frame_id,
                        ts=ts,
                    )
                )
            )
            values = {
                **joint_state_to_values(joints, JOINT_STATE_FIELDS),
                **imu_to_values(imu, f"{self.config.source}/imu"),
            }
            return ControlValues(
                self.config.source,
                ts,
                frame.metadata.episode_id,
                frame.metadata.sequence,
                list(values),
                list(values.values()),
            )

    def write(self, values: dict[str, float], *, source_ts: float | None = None) -> None:
        if self.config.controller == "stock":
            self.move(
                Twist(
                    Vector3(
                        values[f"{self.config.source}/base/vx"],
                        values[f"{self.config.source}/base/vy"],
                        0,
                    ),
                    Vector3(0, 0, values[f"{self.config.source}/base/wz"]),
                )
            )
            return
        fields = [
            [values[f"{name}/{field}"] for name in self._names]
            for field in ("position", "velocity", "kp", "kd", "effort")
        ]
        command = MotorCommandArray(
            q=fields[0],
            dq=fields[1],
            kp=fields[2],
            kd=fields[3],
            tau=fields[4],
            timestamp=source_ts,
        )
        with self._device_lock:
            if self._estopped:
                raise RuntimeError("G1 emergency stop is latched; restart to recover")
            if self._sim is not None:
                self._sim.write_motors(command, observed=self._sim.read())
            elif self._motors is not None:
                self._motors.write(command)
            else:
                raise RuntimeError("G1 motor device is disconnected")

    async def handle_cmd_vel(self, msg: Twist) -> None:
        # Stock commands are handled by ConnectionModule's configured TwistPort.
        if self.config.controller != "stock":
            self.move(msg)

    async def handle_controller_fault(self, msg: String) -> None:
        self.set_estop(True)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> None:
        if (
            duration < 0
            or not np.isfinite([twist.linear.x, twist.linear.y, twist.angular.z, duration]).all()
        ):
            raise ValueError("movement and duration must be finite; duration must be nonnegative")
        with self._device_lock:
            if self._estopped:
                raise RuntimeError("G1 emergency stop is latched")
            if self._move_timer is not None:
                self._move_timer.cancel()
                self._move_timer = None
            if self._stock is not None:
                self._stock.move(twist, duration)
            elif self._locomotion is not None:
                self._locomotion.set_velocity_command(
                    float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)
                )
                if duration:
                    self._move_timer = threading.Timer(duration, self.move, args=(Twist(),))
                    self._move_timer.daemon = True
                    self._move_timer.start()
            else:
                raise RuntimeError(f"G1 {self.config.controller} locomotion controller is missing")

    def halt(self) -> None:
        with self._device_lock:
            if self._sim is not None:
                self._sim.disable(observed=self._sim.read())
            elif self._motors is not None:
                self._motors.set_estop(True)
            elif self._stock is not None:
                self._stock.move(Twist())

    @rpc
    def set_estop(self, estopped: bool) -> bool:
        with self._device_lock:
            if not estopped and self._estopped:
                raise RuntimeError("G1 emergency stop is latched; restart to recover")
            self._estopped = estopped
            if estopped:
                self.halt()
        if estopped and self._locomotion is not None:
            self._locomotion.set_estop(True)
        return True

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        if self._stock is None:
            raise NotImplementedError(
                "stock SDK actions are not provided by a custom locomotion policy"
            )
        return cast("dict[Any, Any]", self._stock.publish_request(topic, data))

    @rpc
    def stop(self) -> None:
        try:
            super().stop()
        finally:
            with self._device_lock:
                if self._move_timer is not None:
                    self._move_timer.cancel()
                if self._sim is not None:
                    self._sim.close()
                    self._sim = None
            if self._motors is not None:
                self._motors.stop()
            if self._stock is not None:
                self._stock.stop()
