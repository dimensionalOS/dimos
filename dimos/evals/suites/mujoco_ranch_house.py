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

"""Interactive suite: the MuJoCo Go2 in the NJ ranch scan, parity with test_mujoco_ranch_spatial_memory.

The robot drives the ranch tour first (the environment's setup), then is told "go to the bed"; it
scores on how near the last odom pose is to any of the ranch's three beds (their boxes from
dim-go2-sim's labels). The environment runs --agentic, so the agent is the one it brings::

    dimos evals run dimos.evals.suites.mujoco_ranch_house --agent dimos.evals.agents.mcp_client_adapter
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

from dimos.e2e_tests.go2_sim_client import RANCH_TOUR, centres, distance_to_nearest, ranch_labels
from dimos.evals.environments.go2_sim import Go2SimEnvironment
from dimos.evals.scorers import ramp
from dimos.evals.types import EvalCase, Outcome, Suite, recording

if TYPE_CHECKING:
    from dimos.e2e_tests.go2_sim_client import Go2SimClient


def _explore_ranch(sim: Go2SimClient) -> None:
    import time

    from dimos.core.transport_factory import make_transport
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.simulation.mujoco.direct_cmd_vel_explorer import DirectCmdVelExplorer

    seen: list[PoseStamped] = []
    probe = make_transport("/odom", PoseStamped)
    probe.start()
    unsubscribe = probe.subscribe(lambda msg, *args: seen.append(msg))
    deadline = time.time() + 180.0
    while not seen and time.time() < deadline:
        time.sleep(1.0)
    unsubscribe()
    probe.stop()
    if not seen:
        raise TimeoutError("no /odom within 180s")

    explorer = DirectCmdVelExplorer()
    explorer.linear_speed = 0.5
    explorer.start()
    try:
        explorer.follow_points(RANCH_TOUR)
    finally:
        explorer.stop()


def _ended_near(name: str) -> Callable[[Outcome], float]:
    """Where the robot ended up, against the nearest object called *name*."""

    def grade(o: Outcome) -> float:
        store = recording(o)
        try:
            p = store.streams.odom.last().data.position
        finally:
            store.stop()
        d = distance_to_nearest(p.x, p.y, centres(ranch_labels()["objects"], name))
        return ramp(max(0.0, d - 2.0), band=2.0)  # full credit inside the e2e 2 m threshold

    return grade


go_to_bed = EvalCase(
    id="mujoco_ranch_go_to_bed",
    inputs="go to the bed",
    environment=Go2SimEnvironment(setup=_explore_ranch),
    grade=_ended_near("bed"),
    timeout_s=180.0,  # e2e parity
    tags=frozenset({"nav", "system"}),
)

SUITE: Suite = [go_to_bed]
