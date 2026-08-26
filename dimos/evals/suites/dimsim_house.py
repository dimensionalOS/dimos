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

"""Interactive suite — dimsim apartment, parity with test_dimsim_spatial_memory.

The e2e test asserts ``wait_until_odom_position(-3.567, -1.332, threshold=2)``
after "go to the bed"; here the same success condition is a graded ramp scored
against the live mem2 store written by the ``go2-memory`` Recorder.

Exploration (the e2e ``explore_house`` fixture) is the case's ``setup`` — the
agent needs spatial memory of the apartment before it can navigate it.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

from dimos.evals.scorers import final, ramp
from dimos.evals.types import InteractiveEnvironment, InteractiveEval, Suite
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from dimos.e2e_tests.dim_sim_client import DimSimClient
    from dimos.memory.store.base import Store

BED = Vector3(-3.567, -1.332, 0.0)

_HOUSE_TOUR = [
    (3.881, 4.803),
    (4.160, 1.615),
    (1.596, 1.505),
    (1.649, 0.137),
    (-3.644, -0.064),
    (-3.759, -2.661),
    (-4.186, -4.830),
    (-3.759, -2.661),
    (-1.070, -3.285),
    (-2.504, -2.452),
    (-2.647, 5.243),
    (-3.663, 3.591),
    (-1.178, 1.974),
    (-2.416, 2.629),
    (-2.581, 0.164),
    (1.834, 0.072),
    (3.010, -3.883),
    (1.756, -3.742),
    (6.336, -4.077),
    (8.264, -5.119),
    (6.258, -0.964),
    (6.453, 5.327),
]


def _explore_house(sim: DimSimClient) -> None:
    import time

    from dimos.core.transport import LCMTransport
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.simulation.mujoco.direct_cmd_vel_explorer import DirectCmdVelExplorer

    # dimsim spawns the robot well after MCP is up — wait for odom before driving
    seen: list[PoseStamped] = []
    probe: LCMTransport[PoseStamped] = LCMTransport("/odom", PoseStamped)
    probe.subscribe(lambda msg, *args: seen.append(msg))
    deadline = time.time() + 180.0
    while not seen and time.time() < deadline:
        time.sleep(1.0)
    if not seen:
        raise TimeoutError("no /odom within 180s — robot never spawned")

    explorer = DirectCmdVelExplorer()
    explorer.linear_speed = 0.5
    explorer.start()
    try:
        explorer.follow_points(_HOUSE_TOUR)
    finally:
        explorer.stop()


def _xy_distance_to(target: Vector3) -> Callable[[Store], float]:
    def distance(store: Store) -> float:
        p = store.streams.odom.last().data.position
        d = Vector3(p.x - target.x, p.y - target.y, 0.0).length()
        return ramp(d, band=2.0)  # e2e parity: threshold=2 -> full credit inside 2m

    return distance


go_to_bed = InteractiveEval(
    id="dimsim_go_to_bed",
    inputs="go to the bed",
    score=_xy_distance_to(BED),
    aggregate=final,
    interval_s=2.0,
    timeout_s=180.0,  # e2e parity
    blueprint="unitree-go2-agentic go2-memory",
    environment=InteractiveEnvironment(
        simulator="dimsim",
        scene="apartment",
        setup=_explore_house,
    ),
    tags=frozenset({"nav", "system"}),
)

SUITE: Suite = [go_to_bed]
