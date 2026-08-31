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

"""The guessability ablation: instruction only."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from dimos.evals.agents.lib.chat import ChatAgent, single_call
from dimos.evals.types import Environment, RunningEnvironment, Trajectory

BLIND_BLOCK: dict[str, str] = {
    "type": "text",
    "text": "[observations withheld — answer anyway]",
}


@dataclass
class Blind(ChatAgent):
    """One model call, instruction only. Never reads the recording."""

    def preflight(self, environment: Environment) -> None:
        """Any environment."""

    def run(self, inputs: str, env: RunningEnvironment, run_dir: Path) -> Trajectory:
        return single_call(self, [BLIND_BLOCK], inputs, run_dir)
