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

from pydantic import Field

from dimos.evals.agents.single_call import Blocks, SingleCallAgent, SingleCallAgentConfig
from dimos.evals.constants import OBSERVATION_SYSTEM_PROMPT
from dimos.evals.types import RunningEnvironment

BLIND_BLOCK: dict[str, str] = {
    "type": "text",
    "text": "[observations withheld — answer anyway]",
}


class BlindConfig(SingleCallAgentConfig):
    system_prompt: str | None = OBSERVATION_SYSTEM_PROMPT
    request_timeout_s: float | None = Field(default=None, gt=0, allow_inf_nan=False)
    max_retries: int | None = Field(default=None, ge=0)
    max_output_tokens: int | None = Field(default=None, gt=0)


class Blind(SingleCallAgent):
    """One model call, instruction only. Never reads the recording."""

    config: BlindConfig

    def _observation_blocks(self, env: RunningEnvironment) -> Blocks:
        return [BLIND_BLOCK]
