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


"""Pour through dimOS's Python API: pour the cup of balls into the bowl with ``Dimos.connect()``
and the ``Arm`` SDK instead of MCP tool calls. Same scene, task and grader as ``pour``.

dimos evals run dimos.evals.suites.pour_py --agent dimos.evals.agents.pi
"""

from dimos.evals.scorers import pour_partial
from dimos.evals.suites.binbottle_py import GUIDANCE, environment
from dimos.evals.suites.pour import BODIES, TASK
from dimos.evals.types import EvalCase, Suite

SUITE: Suite = [
    EvalCase(
        id="pour_py_cup_to_bowl",
        inputs=f"{TASK} {GUIDANCE}",
        environment=environment("pour", BODIES),
        # 0.1 touched the cup + 0.2 lifted it + 0.7 x share of the balls in the bowl; passing
        # needs at least 90 of the 100 in the bowl.
        grade=pour_partial,
        threshold=0.93,
        timeout_s=3600.0,
        tags=frozenset({"mujoco", "pour", "manipulation", "python"}),
    ),
]
