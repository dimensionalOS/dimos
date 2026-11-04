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

"""Scene commands for the MuJoCo Go2 (dim-go2-sim) a launched ``dimos --simulation mujoco`` runs, through
its RPCs, and the NJ ranch scan its e2e tests and evals run in."""

from __future__ import annotations

import math
from typing import Any

from dimos.core.rpc_client import RPCClient
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PointStamped import PointStamped

RANCH = "supersplat/3f89bbd3"
# The run arg that puts the sim's Go2 in the ranch.
RANCH_ARG = f"--go2webrtcsim.scene.package={RANCH}"
# A blind drive (DirectCmdVelExplorer) through every room the robot fits into without the planner,
# from the spawn in the living room; the bottom bedrooms' doors are too tight, so it looks in from
# the cross hall. Planned offline on the cooked collision boxes, 0.33 m from anything.
RANCH_TOUR = [
    (4.03, -0.97), (5.03, -3.17), (3.18, -3.17), (2.83, -5.47), (2.93, -7.12), (3.03, -7.27),
    (4.33, -7.27), (5.53, -5.97), (4.43, -7.22), (3.78, -7.47), (3.58, -7.57), (2.43, -7.38),
    (2.73, -6.97), (2.83, -5.47), (2.73, -4.47), (1.58, -4.07), (2.43, -4.12), (2.68, -3.38),
    (2.28, -3.03), (1.22, -2.12), (1.18, -1.97), (0.78, -2.38), (-1.47, -2.57), (-3.42, -2.62),
    (-4.22, -2.62), (-4.72, -0.92), (-4.97, 1.03), (-3.52, 2.12), (-2.82, 2.12), (-2.57, 4.53),
    (-2.67, 7.88), (-3.97, 9.03), (-2.77, 8.03), (-2.57, 4.53), (-2.57, 2.33), (-3.38, 2.12),
    (-3.88, 1.83), (-4.28, 1.23), (-4.72, 0.18), (-4.72, -1.02), (-4.47, -2.38), (-2.47, -2.62),
    (-1.42, -0.17), (-0.97, 0.48), (0.93, 0.48), (1.33, 1.08), (1.33, 2.08), (0.33, 3.03),
]  # fmt: skip


class Go2SimClient:
    """Teleports the sim's Go2, drops walls in its scene, reads its labelled objects, and sends nav goals."""

    def __init__(self) -> None:
        self._sim: Any = None
        self._goal: PubSubTransport[PointStamped] = make_transport("/clicked_point", PointStamped)

    def start(self) -> None:
        self._goal.start()

    def stop(self) -> None:
        if self._sim is not None:
            self._sim.stop_rpc_client()
        self._goal.stop()

    @property
    def sim(self) -> Any:
        # Imported on first use, so the e2e tests collect without dim-go2-sim installed.
        if self._sim is None:
            from dimos.robot.unitree.go2.connections.webrtc_sim import Go2WebRtcSim

            self._sim = RPCClient(None, Go2WebRtcSim)
        return self._sim

    def teleport(self, x: float, y: float, yaw: float = 0.0) -> None:
        self.sim.teleport(x, y, yaw)

    def add_wall(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        thickness: float = 0.1,
        height: float = 2.0,
    ) -> int:
        """A wall along x or y from (x1, y1) to (x2, y2), DimSimClient.add_wall's way; its box index."""
        if x1 != x2 and y1 != y2:
            raise ValueError("the sim's walls run along x or y")
        half = thickness / 2
        return int(
            self.sim.add_box(
                min(x1, x2) - half,
                min(y1, y2) - half,
                0.0,
                max(x1, x2) + half,
                max(y1, y2) + half,
                height,
            )
        )

    def remove_wall(self, index: int) -> None:
        self.sim.remove_box(index)

    def objects(self, name: str | None = None) -> list[dict[str, Any]]:
        """The scene's labelled objects (all, or those called *name*): name, room, detail, box."""
        return [o for o in self.sim.objects() if name is None or o["name"] == name]

    def centres(self, name: str) -> list[tuple[float, float]]:
        """Where each object called *name* is, on the floor."""
        return centres(self.objects(), name)

    def publish_goal(self, x: float, y: float) -> None:
        self._goal.publish(PointStamped(x, y, 0.0, frame_id="world"))


def ranch_labels() -> dict[str, Any]:
    """The ranch's labels as dim-go2-sim ships them (what Go2Sim.objects and rooms answer), without a running sim."""
    import json

    from dim_go2_sim.supersplat import bundled_labels

    path = bundled_labels(RANCH.split("/")[1])
    assert path is not None, "dim-go2-sim ships no ranch labels; it needs >= 0.5.3"
    labels: dict[str, Any] = json.loads(path.read_text())
    return labels


def centres(objects: list[dict[str, Any]], name: str) -> list[tuple[float, float]]:
    return [
        ((o["box"][0] + o["box"][3]) / 2, (o["box"][1] + o["box"][4]) / 2)
        for o in objects
        if o["name"] == name
    ]


def distance_to_nearest(x: float, y: float, points: list[tuple[float, float]]) -> float:
    return min(math.hypot(x - px, y - py) for px, py in points)
