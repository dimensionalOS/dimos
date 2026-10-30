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

"""A real xArm7 at a table: one pick, graded by the operator who watched it.

The arm's address is the coordinator's ``addresses`` setting, here as an env override:

    CONTROLCOORDINATOR__ADDRESSES='{"arm": "192.168.1.185"}' \
        dimos evals run dimos.evals.suites.hardware_xarm7 --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from dimos.evals.environments.hardware import HardwareEnvironment
from dimos.evals.scorers import human
from dimos.evals.types import EvalCase, Suite

TASK = "Pick up the cup from the table and hold it in the air."

SUITE: Suite = [
    EvalCase(
        id="xarm7_pick_cup",
        inputs=TASK,
        environment=HardwareEnvironment(
            blueprint=["xarm-grasp", "mcp-server", "observe-skill"],
            task=TASK,
            stop_tool="cancel",
            settle_on_joints=True,
            module_env={"MANIPULATIONMODULE__DEFAULT_SPEED_SCALE": "0.3"},
        ),
        grade=human(TASK),
        timeout_s=300.0,
        tags=frozenset({"hardware", "xarm7", "manipulation", "pick"}),
    ),
]
