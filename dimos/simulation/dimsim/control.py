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

from typing import cast

from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.simulation.dimsim.scene_client import SceneClient
from dimos.simulation.scene_controls import PlanarBounds


class DimSimSceneControl:
    """Map canonical DimOS navigation controls to the DimSim browser scene."""

    provider_name = "dimsim"

    def __init__(self) -> None:
        self._client: SceneClient | None = None
        self._goal_request = cast(
            "PubSubTransport[PoseStamped]",
            make_transport("goal_request", PoseStamped),
        )

    def start(self) -> None:
        self._goal_request.start()

    def stop(self) -> None:
        if self._client is not None:
            self._client.stop()
            self._client = None
        self._goal_request.stop()

    @property
    def client(self) -> SceneClient:
        if self._client is None:
            self._client = SceneClient()
            self._client.start()
        return self._client

    def set_agent_position(
        self,
        x: float,
        y: float,
        z: float | None = None,
    ) -> None:
        self.client.set_agent_position(y, 0.52 if z is None else z, x)

    def add_wall(self, x1: float, y1: float, x2: float, y2: float) -> None:
        self.client.add_wall(y1, x1, y2, x2)

    def publish_goal(self, x: float, y: float) -> None:
        self._goal_request.broadcast(
            None,
            PoseStamped(
                position=(x, y, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
            ),
        )

    def semantic_object_bounds(self, query: str) -> PlanarBounds:
        bounds = self.client.get_semantic_object_bounds(query)
        minimum = bounds["min"]
        maximum = bounds["max"]
        # DimSim is Three.js Y-up. Its bridge publishes (z, x, y) as
        # canonical DimOS (x, y, z), so apply the same mapping to the AABB.
        return PlanarBounds(
            min_x=float(minimum["z"]),
            min_y=float(minimum["x"]),
            max_x=float(maximum["z"]),
            max_y=float(maximum["x"]),
        )


__all__ = ["DimSimSceneControl"]
