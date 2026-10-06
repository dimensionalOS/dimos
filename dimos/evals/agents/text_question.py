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

"""One model call with the original text question and no added context."""

from pydantic import Field

from dimos.evals.agents.lib.single_call import (
    Blocks,
    SingleCallAgent,
    SingleCallAgentConfig,
)
from dimos.evals.types import RunningEnvironment


class TextQuestionConfig(SingleCallAgentConfig):
    system_prompt: str | None = None
    request_timeout_s: float | None = Field(default=60.0, gt=0, allow_inf_nan=False)
    max_retries: int | None = Field(default=0, ge=0)
    max_output_tokens: int | None = Field(default=2048, gt=0)


class TextQuestion(SingleCallAgent):
    """The prompt is the complete task; no observations or tools are added.

    The provider request timeout is capped by the case budget. It bounds
    network operations, not total job duration; a job deadline remains useful
    for provider stalls or unexpectedly expensive setup.
    """

    config: TextQuestionConfig

    def _observation_blocks(self, env: RunningEnvironment) -> Blocks:
        if env.streams or env.mcp_url:
            raise ValueError("TextQuestion requires a prompt-only environment")
        return []
