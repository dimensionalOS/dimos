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

"""Convert K1 motor commands to the existing sim2 timed action contract."""

from typing import Any

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.control_msgs import ActionCommand, JointCommand
from dimos_lcm.std_msgs import Header, Time

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.booster.k1.control_config import JOINTS
from dimos.sim2.module import SimModule


class K1SimConnectionConfig(ModuleConfig):
    simulation: str = "SimModule"


class K1SimConnection(Module):
    config: K1SimConnectionConfig
    motor_command: In[MotorCommandArray]
    actions: Out[ActionCommand]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._clock_id: str | None = None

    @rpc
    def build(self) -> None:
        super().build()
        simulation = RPCClient.remote(SimModule, self.config.simulation, rpc=self.rpc)
        try:
            self._clock_id = simulation.simulation_config()["sim_clock_id"]
        finally:
            simulation.stop_rpc_client()
        self.actions.transport.start()

    async def handle_motor_command(self, command: MotorCommandArray) -> None:
        if self._clock_id is None:
            raise RuntimeError("K1 sim connection must be built before commands")
        self.actions.publish(motor_action(command, self._clock_id))


def motor_action(command: MotorCommandArray, clock_id: str) -> ActionCommand:
    ns = round(command.timestamp * 1e9)
    return ActionCommand(
        header=SampleHeader(clock_id=clock_id, time_ns=ns, source_id="k1/policy", sequence=ns),
        based_on_time_ns=ns,
        apply_at_ns=ns,
        valid_until_ns=ns + 100_000_000,
        joint_commands_length=5,
        joint_commands=[
            JointCommand(
                header=Header(stamp=Time(sec=ns // 10**9, nsec=ns % 10**9)),
                joint_names_length=len(JOINTS),
                joint_names=list(JOINTS),
                interface_name=name,
                values_length=len(values),
                values=list(values),
            )
            for name, values in (
                ("position", command.q),
                ("velocity", command.dq),
                ("kp", command.kp),
                ("kd", command.kd),
                ("effort", command.tau),
            )
        ],
    )
