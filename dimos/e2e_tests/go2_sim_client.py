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

from collections.abc import Callable, Iterator
import math
import time
from typing import Any

import pytest

from dimos.core.rpc_client import RPCClient
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

RANCH = "supersplat/3f89bbd3"
# The run arg that puts the sim's Go2 in the ranch.
RANCH_ARG = f"--go2webrtcsim.scene.package={RANCH}"
# A walk through the ranch for the robot to be teleported along, a metre a step (0.3 m or more from
# anything, on the cooked collision boxes), so the lidar maps every room and the floor joining them;
# True is the most open spot in a room, where it turns once for the camera. It ends in the living
# room, where it starts, out of sight of every bed.
RANCH_TOUR = [
    (4.12, 2.38, False), (4.12, 1.38, False), (4.12, 0.38, False), (4.12, -0.62, False),
    (4.38, -1.52, False), (4.73, -2.42, False), (5.23, -3.38, True), (4.28, -3.22, False),
    (3.28, -3.22, False), (2.98, -4.03, False), (2.98, -5.02, False), (2.98, -6.02, False),
    (2.98, -7.02, False), (3.88, -7.22, False), (4.78, -6.97, False), (5.73, -6.52, True),
    (4.83, -6.77, False), (3.98, -7.22, False), (3.03, -7.07, False), (2.98, -6.12, False),
    (2.98, -5.12, False), (2.73, -4.17, True), (2.73, -5.17, False), (2.88, -6.12, False),
    (3.03, -7.07, False), (3.62, -7.77, False), (4.03, -8.68, True), (3.62, -7.83, False),
    (2.88, -7.42, False), (1.93, -7.52, False), (0.97, -7.67, False), (0.12, -7.97, False),
    (-0.72, -8.38, True), (0.28, -8.32, False), (1.18, -8.02, False), (1.93, -7.52, False),
    (2.68, -6.97, False), (2.68, -5.97, False), (2.68, -4.97, False), (2.33, -4.28, False),
    (1.33, -4.28, False), (0.03, -4.22, True), (1.03, -4.22, False), (2.03, -4.22, False),
    (2.62, -3.62, False), (1.98, -3.17, False), (1.03, -3.12, False), (0.18, -2.72, False),
    (-0.72, -2.47, False), (-1.62, -2.42, True), (-2.57, -2.57, False), (-3.58, -2.57, False),
    (-4.42, -2.32, False), (-4.42, -1.32, False), (-4.67, -0.42, False), (-4.92, 0.48, False),
    (-5.18, 1.03, True), (-4.42, 1.78, False), (-3.58, 2.12, False), (-2.67, 2.28, False),
    (-2.72, 3.23, False), (-2.92, 4.12, False), (-3.92, 4.12, False), (-5.18, 4.43, True),
    (-4.17, 4.43, False), (-3.17, 4.32, False), (-2.72, 4.23, True), (-2.72, 5.23, False),
    (-2.72, 6.23, False), (-2.72, 7.23, False), (-2.88, 8.18, False), (-3.22, 8.98, True),
    (-2.72, 8.18, False), (-2.62, 7.18, False), (-2.62, 6.18, False), (-2.62, 5.18, False),
    (-2.62, 4.18, False), (-2.62, 3.18, False), (-2.72, 2.23, False), (-3.62, 2.03, False),
    (-4.17, 1.28, False), (-4.53, 0.38, False), (-4.67, -0.52, False), (-4.47, -1.47, False),
    (-4.42, -2.42, False), (-3.47, -2.57, False), (-2.47, -2.57, False), (-2.27, -1.67, False),
    (-1.88, -0.82, False), (-1.27, -0.07, False), (-0.52, 0.53, False), (0.47, 0.53, False),
    (1.18, 0.93, False), (1.28, 1.88, False), (0.72, 2.68, False), (-0.02, 3.28, True),
    (0.93, 3.08, False), (1.83, 2.88, False), (2.68, 2.78, False), (3.53, 3.18, False),
    (4.12, 3.43, True),
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

    def add_floor(self, x1: float, y1: float, x2: float, y2: float) -> int:
        """A slab whose top is the floor (z = 0) from (x1, y1) to (x2, y2), for the lidar to see where
        the scene has none (outside a scan's walls); its box index."""
        return int(self.sim.add_box(min(x1, x2), min(y1, y2), -0.05, max(x1, x2), max(y1, y2), 0.0))

    def remove_wall(self, index: int) -> None:
        self.sim.remove_box(index)

    def objects(self, name: str | None = None) -> list[dict[str, Any]]:
        """The scene's labelled objects (all, or those called *name*): name, room, detail, box."""
        return [o for o in self.sim.objects() if name is None or o["name"] == name]

    def footprints(self, name: str) -> list[tuple[float, float, float, float]]:
        """The floor rectangle (xmin, ymin, xmax, ymax) of each object called *name*."""
        return footprints(self.objects(), name)

    def look_around_ranch(self, turn_rate: float = 1.0, step_s: float = 0.5) -> None:
        """Show the robot the whole ranch: teleported along RANCH_TOUR facing the way it goes, it
        turns once on the spot in each room (a blind drive between waypoints catches on doorways)."""
        cmd_vel: PubSubTransport[Twist] = make_transport("/cmd_vel", Twist)
        cmd_vel.start()
        try:
            previous = RANCH_TOUR[0][:2]
            for x, y, turn in RANCH_TOUR:
                try:
                    self.teleport(x, y, math.atan2(y - previous[1], x - previous[0]))
                except ValueError:  # the sim refused it (raised here as the sim raised it)
                    if turn:
                        raise
                    continue  # a step whose heading puts a leg in a doorframe; the next one will do
                previous = (x, y)
                end = time.monotonic() + step_s + (2 * math.pi / turn_rate if turn else 0.0)
                while time.monotonic() < end:
                    rate = turn_rate if turn else 0.0
                    cmd_vel.publish(Twist(linear=Vector3(), angular=Vector3(0.0, 0.0, rate)))
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


def footprints(objects: list[dict[str, Any]], name: str) -> list[tuple[float, float, float, float]]:
    return [
        (o["box"][0], o["box"][1], o["box"][3], o["box"][4]) for o in objects if o["name"] == name
    ]


def distance_to_nearest(
    x: float, y: float, rectangles: list[tuple[float, float, float, float]]
) -> float:
    """How far (x, y) is from the nearest of *rectangles* (0 inside one)."""
    return min(
        math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))
        for x0, y0, x1, y1 in rectangles
    )


# fixtures for the MuJoCo ranch tests, kept out of conftest.py (it stays DimSim's)
@pytest.fixture
def go2_sim() -> Iterator[Go2SimClient]:
    client = Go2SimClient()
    client.start()
    yield client
    client.stop()


@pytest.fixture
def explore_ranch(go2_sim: Go2SimClient) -> Callable[[], None]:
    return go2_sim.look_around_ranch
