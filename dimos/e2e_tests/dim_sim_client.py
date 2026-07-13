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

import json
from typing import Any

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

    # ------------------------------------------------------------------ #
    # Human NPC helpers
    #
    # Robot/map frame (x, y) on the floor maps to Three.js (x=y, y=up=0, z=x),
    # the same swap used by set_agent_position/add_wall above. Callers stay in
    # robot-frame coordinates; the swap lives here.
    # ------------------------------------------------------------------ #

    def add_human(
        self,
        name: str,
        x: float,
        y: float,
        *,
        model_url: str = "/agent-model/robot.glb",
        animation: str | int = "idle",
    ) -> dict[str, Any]:
        """Spawn a named, animated NPC actor at robot-frame ``(x, y)`` on the floor."""
        return self.client.add_npc(
            url=model_url,
            name=name,
            position=(y, 0.0, x),
            animation=animation,
        )

    def move_human(self, name: str, x: float, y: float) -> None:
        """Teleport a spawned NPC to robot-frame ``(x, y)`` (keeps its floor height)."""
        self.client.exec(
            f"const o = scene.getObjectByName({json.dumps(name)});"
            f"if (o) {{ o.position.x = {y}; o.position.z = {x}; }}"
            "return true;"
        )

    def get_human_position(self, name: str) -> tuple[float, float]:
        """Read a spawned NPC's position back in robot-frame ``(x, y)``."""
        pos = self.client.exec(
            f"const o = scene.getObjectByName({json.dumps(name)});"
            "return o ? { x: o.position.x, z: o.position.z } : null;"
        )
        if not pos:
            raise KeyError(f"NPC not found in scene: {name}")
        # invert the swap: robot_x = threejs_z, robot_y = threejs_x
        return float(pos["z"]), float(pos["x"])

    def remove_human(self, name: str) -> bool:
        """Remove a spawned NPC by name."""
        return self.client.remove_npc(name)

    def add_prop(
        self,
        name: str,
        x: float,
        y: float,
        *,
        geometry: str = "box",
        size: tuple[float, ...] = (1.0, 1.0, 1.0),
        color: int = 0x888888,
        up: float = 0.5,
    ) -> dict[str, Any]:
        """Add a static prop (furniture stand-in) at robot-frame ``(x, y)``.

        ``up`` is the Three.js height of the prop's center (so its base rests on
        the floor). ``size`` follows the ``add_object`` convention per geometry.
        """
        return self.client.add_object(
            geometry=geometry,
            size=size,
            color=color,
            position=(y, up, x),
            name=name,
            dynamic=False,
            collider="box",
        )

    def remove_prop(self, name: str) -> bool:
        """Remove a spawned prop by name."""
        return self.client.remove_object(name)

    def object_exists(self, name: str) -> bool:
        """Whether a named object is present in the scene."""
        return bool(
            self.client.exec(
                f"return scene.getObjectByName({json.dumps(name)}) ? true : false;"
            )
        )

    def refresh_lidar(self) -> bool:
        """Resend the physics snapshot so server-side lidar sees spawned NPCs/props."""
        return self.client.refresh_lidar_snapshot()
