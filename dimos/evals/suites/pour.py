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


"""Pour: a red cup holds 100 blue balls that stand in for water; pour them into a teal bowl.
Same table, robot, cameras and solver settings as ``binbottle``.

The balls slide on three joints each (no spin) and are frictionless against everything but the
table, so they run out of a tipped cup like a liquid and spilled ones stop where they land.
"""

from dimos.evals.scorers import BALLS, pour_partial
from dimos.evals.suites.binbottle import environment
from dimos.evals.suites.robosuite import GUIDANCE
from dimos.evals.types import EvalCase, Suite

TASK = (
    "The red cup is full of blue balls that stand in for water. Pour them into the teal bowl, "
    "spilling as few as you can."
)
BODIES = ("cup_main", "bowl_main", *BALLS)

SUITE: Suite = [
    EvalCase(
        id="pour_cup_to_bowl",
        inputs=f"{TASK} {GUIDANCE}",
        environment=environment("pour", BODIES),
        # 0.1 touched the cup + 0.2 lifted it + 0.7 x share of the balls in the bowl; passing
        # needs at least 90 of the 100 in the bowl.
        grade=pour_partial,
        threshold=0.93,
        timeout_s=3600.0,
        tags=frozenset({"mujoco", "pour", "manipulation"}),
    ),
]
