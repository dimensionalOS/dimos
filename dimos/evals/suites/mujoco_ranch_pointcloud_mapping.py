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

"""Map the MuJoCo Go2's NJ ranch scan and count its bedrooms (dimsim_pointcloud_mapping's counterpart).

The DimSim apartment's rooms are wall-separated; the ranch's living areas open into each other
(the kitchen, dining and family rooms are one open plan, the entry and living room share a wide
opening), so "how many rooms" has no one answer there. Bedrooms (a room with a bed in it) do: three.

    dimos evals run dimos.evals.suites.mujoco_ranch_pointcloud_mapping --agent <agent module>
"""

from __future__ import annotations

from collections.abc import Callable
import math

from dimos.evals.environments.go2_sim import Go2SimEnvironment
from dimos.evals.scorers import exact, first_number
from dimos.evals.types import EvalCase, Outcome, Suite, recording

# A point inside each bedroom, past its doorway (from dim-go2-sim's ranch labels).
BEDROOMS: dict[str, tuple[float, float]] = {
    "north bedroom": (-4.5, 9.0),  # king bed, palm, dog bed
    "south-west bedroom": (0.0, -8.5),  # bed, dollhouse
    "south-east bedroom": (4.6, -8.8),  # bed, desk
}
N_BEDROOMS = len(BEDROOMS)

INSTRUCTION = (
    "Map the entire reachable house and determine how many bedrooms it contains. A bedroom is "
    "a room with a bed for a person in it; a dog bed or a toy bed doesn't make one. Move only "
    "with deliberate move_to calls using world x/y coordinates; do not use exploration, patrol, "
    "follow, or other autonomous navigation tools. Decide what lidar history you need. When you "
    "are confident, answer with one integer and nothing else."
)


def grade_bedrooms(visit_radius_m: float = 1.5) -> Callable[[Outcome], float]:
    """0.5 for the exact bedroom count, plus 0.5 times the fraction of bedrooms whose point the
    recorded odometry came within *visit_radius_m* of, as grade_rooms scores the apartment."""

    def grade(o: Outcome) -> float:
        try:
            count = exact(float(N_BEDROOMS), first_number(o.trajectory.final_answer))
        except ValueError:
            count = 0.0
        store = recording(o)
        try:
            path = [(p.data.position.x, p.data.position.y) for p in store.streams.odom]
        finally:
            store.stop()
        visited = sum(
            any(math.hypot(x - rx, y - ry) <= visit_radius_m for x, y in path)
            for rx, ry in BEDROOMS.values()
        )
        return 0.5 * count + 0.5 * visited / N_BEDROOMS

    return grade


count_bedrooms = EvalCase(
    id="mujoco_ranch_count_bedrooms",
    inputs=INSTRUCTION,
    environment=Go2SimEnvironment(),
    grade=grade_bedrooms(),
    timeout_s=1200.0,
    tags=frozenset({"nav", "pointcloud", "system"}),
)

SUITE: Suite = [count_bedrooms]
