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

"""A motor device attached to the shared world through ordinary transports.

The connection keeps its public ports, command validation and lifecycle. This
device carries its checked motor commands and feedback on private world topics;
it never starts physics or imports a robot SDK.
"""

from contextlib import ExitStack
from dataclasses import dataclass
import functools
import threading
from typing import Any

from dimos.control.connection.device import Feedback
from dimos.control.contract.convert import motor_command_from_values, twist_from_values
from dimos.core.global_config import GlobalConfig
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.spec import RobotInstanceSpec

FEEDBACK_TYPES: dict[str, type] = {
    "motor_states": JointState,
    "imu": Imu,
    "odom": PoseStamped,
    "tf": TFMessage,
}


@dataclass(frozen=True)
class SimulatedDeviceFactory:
    """Resolved by blueprint composition, instantiated by ConnectionModule.

    Nothing is opened when this description is created or sent to a worker.
    ``topics`` are the exact same private topics configured on the shared world.
    """

    robot: RobotInstanceSpec
    topics: dict[str, str]
    g: GlobalConfig
    feedback_ports: tuple[str, ...]
    halt_damping: float = 1.0

    def __call__(self, publish: Feedback) -> "SimRobotClient":
        return SimRobotClient(self, publish)


class SimRobotClient:
    """A connection's robot-scoped client; owns transport, never physics."""

    def __init__(self, config: SimulatedDeviceFactory, publish: Feedback) -> None:
        self._config = config
        self._publish = publish
        self._joints = config.robot.joint_names
        self._resources: ExitStack | None = None
        self._command: PubSubTransport[Any] | None = None
        self._position: list[float] | None = None
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.RLock()
        self._accept_feedback = False

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._resources is not None:
                return
            with ExitStack() as resources:
                with self._lock:
                    self._accept_feedback = True
                try:
                    velocity = self._config.robot.command_kind == "velocity"
                    command = make_transport(
                        self._config.topics["cmd_vel" if velocity else "motor_command"],
                        Twist if velocity else MotorCommandArray,
                        g=self._config.g,
                    )
                    resources.callback(command.stop)
                    command.start()
                    for kind in self._config.feedback_ports:
                        transport = make_transport(
                            self._config.topics[kind], FEEDBACK_TYPES[kind], g=self._config.g
                        )
                        resources.callback(transport.stop)
                        transport.start()
                        resources.callback(
                            transport.subscribe(functools.partial(self._feedback, kind))
                        )
                except BaseException:
                    with self._lock:
                        self._accept_feedback = False
                        self._position = None
                    raise
                self._command = command
                self._resources = resources.pop_all()

    def _feedback(self, kind: str, message: Any) -> None:
        with self._lock:
            if not self._accept_feedback:
                return
            if kind == "motor_states":
                if tuple(message.name) != self._joints or len(message.position) != len(
                    self._joints
                ):
                    raise ValueError("simulated motor feedback does not match the declared joints")
                self._position = list(message.position)
        self._publish(kind, message)

    def write(self, values: dict[str, float]) -> None:
        """Send checked connection values through the declared command interface."""
        with self._lifecycle_lock:
            if self._command is None:
                raise RuntimeError("simulated motor device has not started")
            if self._config.robot.command_kind == "velocity":
                self.set_velocity(twist_from_values(values, self._config.robot.base_name))
            else:
                self.set_motor_command(motor_command_from_values(values, self._joints))

    def set_motor_command(self, command: MotorCommandArray) -> None:
        if self._config.robot.command_kind != "motor":
            raise ValueError("this robot is controlled by velocity, not external motor commands")
        self._send(command)

    def set_velocity(self, command: Twist) -> None:
        if self._config.robot.command_kind != "velocity":
            raise ValueError("this robot requires external motor commands")
        self._send(command)

    def _send(self, command: MotorCommandArray | Twist) -> None:
        with self._lifecycle_lock:
            if self._command is None:
                raise RuntimeError("simulated motor device has not started")
            self._command.publish(command)

    def halt(self) -> None:
        """Damp effort motors immediately, and hold native position servos.

        Before any feedback the world already holds its authored start pose.
        Native position servos use their own gains and accept only positions.
        """
        with self._lifecycle_lock:
            if self._config.robot.command_kind == "velocity":
                if self._command is not None:
                    self._command.publish(Twist())
                return
            with self._lock:
                position = self._position
            if self._command is None or position is None:
                return
            position_only = set(self._config.robot.position_only_joints)
            gains = [
                (0.0, 0.0 if name in position_only else self._config.halt_damping)
                for name in self._joints
            ]
            zeros = [0.0] * len(self._joints)
            self._command.publish(
                MotorCommandArray(
                    q=position,
                    dq=zeros,
                    tau=zeros,
                    kp=[gain[0] for gain in gains],
                    kd=[gain[1] for gain in gains],
                )
            )

    def stop(self) -> None:
        with self._lifecycle_lock:
            resources, self._resources = self._resources, None
            self._command = None
            with self._lock:
                self._accept_feedback = False
                self._position = None
            if resources is not None:
                resources.close()
