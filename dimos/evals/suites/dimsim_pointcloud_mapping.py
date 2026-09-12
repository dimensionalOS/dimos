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

"""Map a live DimSim apartment and count its rooms.

    uv run dimos evals run dimos.evals.suites.dimsim_pointcloud_mapping \
        --agent <agent module>
"""

from __future__ import annotations

from collections.abc import Callable
import math

from dimos.evals.environments.sim import Sim
from dimos.evals.scorers import exact, first_number
from dimos.evals.types import EvalCase, Outcome, Suite, recording

# unitree-go2 decomposed into its atoms
# (dimos/robot/unitree/go2/blueprints/smart/unitree_go2.py) minus the
# autonomous WavefrontFrontierExplorer and PatrollingModule: deliberate
# move_to navigation keeps working, the exploration shortcut is removed at
# the source.
STACK = (
    "unitree-go2-basic voxel-grid-mapper cost-mapper replanning-a-star-planner "
    "movement-manager mcp-server unitree-skill-container"
)

ROOMS: dict[str, tuple[float, float]] = {
    "living_dining": (2.0, 2.5),  # sectional, TV, dining table
    "kitchen": (-4.0, 2.5),  # fridge, gas range, sink
    "bedroom": (-3.0, -2.5),  # queen bed, desk
    "bathroom": (3.0, -2.5),  # bathtub, toilet, vanity
}
N_ROOMS = len(ROOMS)

INSTRUCTION = (
    "Map the entire reachable apartment and determine how many distinct rooms "
    "it contains. Count wall-separated interior rooms: an open-plan area is "
    "one room, and outdoor space is not a room. Move only with deliberate "
    "move_to calls using world x/y coordinates; do not use exploration, "
    "patrol, follow, or other autonomous navigation tools. Decide what lidar "
    "history you need. When you are confident, answer with one integer and "
    "nothing else."
)


def grade_rooms(visit_radius_m: float = 1.5) -> Callable[[Outcome], float]:
    """0.5 for the exact room count in the reply, plus 0.5 times the fraction
    of rooms whose representative point the recorded odometry came within
    *visit_radius_m* of. Every representative point sits ~2 m past its room's
    doorway, so the default radius requires actually entering the room.
    Passing at the default threshold of 1.0 means the count was right and
    every room was physically entered."""

    def grade(o: Outcome) -> float:
        try:
            count = exact(float(N_ROOMS), first_number(o.trajectory.final_answer))
        except ValueError:  # no number in the reply — coverage can still score
            count = 0.0
        store = recording(o)
        try:
            path = [(p.data.position.x, p.data.position.y) for p in store.streams.odom]
        finally:
            store.stop()
        visited = sum(
            any(math.hypot(x - rx, y - ry) <= visit_radius_m for x, y in path)
            for rx, ry in ROOMS.values()
        )
        return 0.5 * count + 0.5 * visited / N_ROOMS

    return grade


count_rooms = EvalCase(
    id="dimsim_count_rooms",
    inputs=INSTRUCTION,
    environment=Sim(blueprint=STACK, simulator="dimsim", scene="apartment"),
    grade=grade_rooms(),
    timeout_s=1200.0,  # room for several blocking move_to calls (each up to ~100 s)
    tags=frozenset({"nav", "pointcloud", "system"}),
)

SUITE: Suite = [count_rooms]
