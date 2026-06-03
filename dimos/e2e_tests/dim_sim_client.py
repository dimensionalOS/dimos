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

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.dimsim.scene_client import SceneClient


class DimSimClient:
    _client: SceneClient | None = None

    def __init__(self) -> None:
        self._client = None
        self._goal_request: LCMTransport[PoseStamped] = LCMTransport("/goal_request", PoseStamped)

    def start(self) -> None:
        # self.client should be started lazily to avoid starting the dimsim
        # process before pytest fixtures are ready
        self._goal_request.start()

    def stop(self) -> None:
        self.client.stop()
        self._goal_request.stop()

    @property
    def client(self) -> SceneClient:
        if self._client is None:
            self._client = SceneClient()
            self._client.start()
        return self._client

    def set_agent_position(self, x: float, y: float, z: float = 0.52) -> None:
        self.client.set_agent_position(y, z, x)

    def add_wall(self, x1: float, y1: float, x2: float, y2: float) -> None:
        self.client.add_wall(y1, x1, y2, x2)

    def publish_goal(self, x: float, y: float) -> None:
        self._goal_request.publish(
            PoseStamped(
                position=(x, y, 0),
                orientation=(0, 0, 0, 1),
                frame_id="world",
            )
        )
