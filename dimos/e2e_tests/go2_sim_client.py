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
import time
from typing import Any

from dimos.core.rpc_client import RPCClient
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

RANCH = "supersplat/3f89bbd3"
# The run arg that puts the sim's Go2 in the ranch.
RANCH_ARG = f"--go2webrtcsim.scene.package={RANCH}"
# The most open spot in each labelled room (the ranch's labels), where the Go2 can turn on the spot;
# from the cooked collision boxes, each at least 0.5 m from anything.
RANCH_VIEWPOINTS = [
    (4.12, 3.43),  # living room
    (5.23, -3.38),  # entry
    (5.73, -6.52),  # office
    (2.73, -4.17),  # hallway
    (4.03, -8.68),  # south-east bedroom
    (-0.72, -8.38),  # south-west bedroom
    (0.03, -4.22),  # hall bathroom
    (-1.62, -2.42),  # dining room
    (-0.02, 3.28),  # kitchen
    (-5.18, 1.03),  # family room
    (-5.18, 4.43),  # main bathroom
    (-2.72, 4.23),  # north hall
    (-3.22, 8.98),  # north bedroom
]


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

    def add_floor(self, x1: float, y1: float, x2: float, y2: float) -> int:
        """A slab whose top is the floor (z = 0) from (x1, y1) to (x2, y2), for the lidar to see where
        the scene has none (outside a scan's walls); its box index."""
        return int(self.sim.add_box(min(x1, x2), min(y1, y2), -0.05, max(x1, x2), max(y1, y2), 0.0))

    def remove_wall(self, index: int) -> None:
        self.sim.remove_box(index)

    def objects(self, name: str | None = None) -> list[dict[str, Any]]:
        """The scene's labelled objects (all, or those called *name*): name, room, detail, box."""
        return [o for o in self.sim.objects() if name is None or o["name"] == name]

    def centres(self, name: str) -> list[tuple[float, float]]:
        """Where each object called *name* is, on the floor."""
        return centres(self.objects(), name)

    def look_around_ranch(self, turn_rate: float = 1.0) -> None:
        """Show the robot every room: teleported to each of RANCH_VIEWPOINTS, it turns once on the
        spot, so the camera and lidar sweep the room (a blind drive between them catches on doorways)."""
        cmd_vel: PubSubTransport[Twist] = make_transport("/cmd_vel", Twist)
        cmd_vel.start()
        try:
            for x, y in RANCH_VIEWPOINTS:
                self.teleport(x, y, 0.0)
                end = time.monotonic() + 1.0 + 2 * math.pi / turn_rate  # a second to settle
                while time.monotonic() < end:
                    cmd_vel.publish(Twist(linear=Vector3(), angular=Vector3(0.0, 0.0, turn_rate)))
                    time.sleep(0.1)
            cmd_vel.publish(Twist())
        finally:
            cmd_vel.stop()

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
