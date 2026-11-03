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

"""A real Go2: one short move, graded by the operator who watched it.

There is no stop tool: stopping dimos lays the Go2 down.

    ROBOT_IP=192.168.123.161 dimos evals run dimos.evals.suites.hardware_go2 \
        --agent dimos.evals.agents.pi
"""

from __future__ import annotations

from dimos.evals.environments.hardware import HardwareEnvironment
from dimos.evals.scorers import human
from dimos.evals.types import EvalCase, Suite

TASK = "Walk forward about one meter, then turn around to face where you started."

SUITE: Suite = [
    EvalCase(
        id="go2_walk_and_turn",
        inputs=TASK,
        environment=HardwareEnvironment(
            blueprint=["unitree-go2", "mcp-server", "unitree-skill-container"],
            task=TASK,
        ),
        grade=human(TASK),
        timeout_s=300.0,
        tags=frozenset({"hardware", "go2", "navigation"}),
    ),
]
