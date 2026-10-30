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

"""G1 motor connection, retaining the same public ports for hardware and simulation."""

from __future__ import annotations

from importlib import import_module
from pathlib import Path
from typing import TYPE_CHECKING, ClassVar, Literal, cast

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.connection.device import ConnectionDevice
from dimos.control.contract.convert import motor_command_to_values
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import PD_INTERFACES, Limits
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.g1.motor_device import G1_JOINT_NAMES, G1MotorDevice

if TYPE_CHECKING:
    from dimos.sim2.spec import SimulatedRobot


class G1WholeBodyConnectionConfig(ConnectionConfig):
    source: Literal["g1"] = "g1"
    pd: bool = True
    halt_damping: float = Field(default=5.0, ge=0.0, allow_inf_nan=False)
    network_interface: str = Field(default="")
    release_sport_mode: bool = True
    publish_rate_hz: float = 500.0
    frame_id: str = "g1_pelvis"
    mode_machine: int = 5
    # Stiffness soft-start on taking low-level control: the first commands go
    # out with full damping (kd) but kp and tau scaled in over this window,
    # mirroring the remote's damp -> lock-stand feel. Full commanded stiffness
    # in the very first frame slams the robot from wherever it hangs to the
    # commanded pose. 0 disables.
    soft_start_seconds: float = Field(default=3.0, ge=0.0, allow_inf_nan=False)


class G1WholeBodyConnection(ConnectionModule):
    """One public G1 interface with either the DDS device or a supplied simulator device."""

    config: G1WholeBodyConnectionConfig
    default_sim_scene: ClassVar[str] = "logistics.xml"

    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]
    imu: Out[Imu]

    @staticmethod
    def simulated_robot(config: G1WholeBodyConnectionConfig, scene: Path) -> SimulatedRobot:
        """Supply the model binding consumed by build-time attach_simulation.

        The connection stays in the blueprint; ConnectionModule selects its
        injected simulation device. G1 holds the GR00T home pose until commanded.
        """
        # Hardware imports need no simulation extra; resolve this backend only
        # when composition selects it.
        simulation = import_module("dimos.robot.unitree.g1.simulation")
        return cast("SimulatedRobot", simulation.simulated_robot(config, scene))

    @property
    def limits(self) -> dict[str, Limits]:
        # PD positions are virtual spring targets, not requested joint poses.
        # Preserve their unbounded range; the existing controller owns target limits.
        if self._limits is None:
            self._limits = {
                f"{joint}/{interface}": (
                    Limits(lo=0.0) if interface in (Interface.KP, Interface.KD) else Limits()
                )
                for joint in G1_JOINT_NAMES
                for interface in PD_INTERFACES
            }
        return self._limits

    def create_hardware_device(self) -> ConnectionDevice:
        return G1MotorDevice(self.config, self.publish_feedback)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.motor_command.subscribe(self._on_motor_command)))

    def _on_motor_command(self, message: MotorCommandArray) -> None:
        columns = (message.q, message.dq, message.kp, message.kd, message.tau)
        if any(len(column) != len(G1_JOINT_NAMES) for column in columns):
            self._report(
                "G1 motor commands need one value per motor in every column", rejection=True
            )
            return
        try:
            values = motor_command_to_values(message, G1_JOINT_NAMES)
        except (ValueError, TypeError) as error:
            self._report(str(error), rejection=True)
            return
        self.command(values)
