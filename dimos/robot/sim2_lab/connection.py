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

"""One connection class for this robot; the sim client is an ordinary device object.

The canonical model has local joint names for the existing control key contract.
An instance namespaces that same model for the world and TF. No second URDF.
"""

from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule
from dimos.control.contract.keys import Interface, Key
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.sim2_lab.model import PUSHER
from dimos.sim2.client import SimRobotClient
from dimos.sim2.module import SimulationSpec
from dimos.sim2.types import Frame, JointTargets, WorldState


class PusherConfig(ConnectionConfig):
    source: str = "left"
    model: RobotModel = PUSHER
    joint_commands: tuple[Interface, ...] = (Interface.POSITION,)
    deadman_timeout_s: float = 0.5


class PusherConnection(ConnectionModule):
    config: PusherConfig
    _sim: SimulationSpec
    targets: Out[JointTargets]
    world_frame: In[Frame]
    position_command: In[JointState]

    def connect(self) -> None:
        self._layout = self._sim.describe()
        self._names = tuple(
            self.config.source + "/" + j.name
            for j in self.config.model.load().joints
            if j.type != "fixed"
        )
        self._indices = [self._layout.joints.index(name) for name in self._names]
        self._client = SimRobotClient(self._names, self.targets.publish)
        self._observed: WorldState | None = None

    async def handle_world_frame(self, frame: Frame) -> None:
        if hasattr(self, "_client"):
            state = frame.state
            self._observed = state
            self._client.observe(state)

    async def handle_position_command(self, state: JointState) -> None:
        self.command(
            {
                str(Key.of(self.config.source, name, Interface.POSITION)): value
                for name, value in zip(state.name, state.position, strict=True)
            }
        )

    def write(self, values: dict[str, float]) -> None:
        positions = {
            self.config.source + "/" + Key(key).resource: value for key, value in values.items()
        }
        self._client.write(positions, self.config.deadman_timeout_s)

    def halt(self) -> None:
        if self._observed is not None:
            self._client.write(
                dict(zip(self._names, self._observed.position[self._indices], strict=True))
            )

    @rpc
    def move(self, x: float, y: float = 0.0) -> None:
        """Set slide positions in metres; refresh while moving (deadman .5 seconds)."""
        self.command(
            {
                str(Key.of(self.config.source, "slide_x", Interface.POSITION)): x,
                str(Key.of(self.config.source, "slide_y", Interface.POSITION)): y,
            }
        )
