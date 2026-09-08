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

"""Shared model invocation and tracing for agents that answer in one call."""

from __future__ import annotations

from abc import abstractmethod
from pathlib import Path
import time
from typing import Any

from langchain_core.language_models import BaseChatModel
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage, SystemMessage
from langchain_core.outputs import ChatGeneration

from dimos.agents.llm_trace import latest_pair, write_normalized
from dimos.evals.agent import Agent, AgentConfig
from dimos.evals.agents.lib.langchain_to_atif import append_ai_message_to_atif
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.types import RunningEnvironment, Trajectory

DEFAULT_MODEL = "gpt-5.6-luna"
EVAL_SYSTEM_PROMPT = "Answer the question using only the provided observations."

Blocks = list[str | dict[str, Any]]


class SingleCallAgentConfig(AgentConfig):
    model: str = DEFAULT_MODEL
    system_prompt: str = EVAL_SYSTEM_PROMPT
    chat_model: BaseChatModel | None = None


class SingleCallAgent(Agent):
    """Encode observations, call a chat model once, and record the result.

    ``chat_model`` injects a LangChain model, including a fake for offline evals.
    Otherwise ``model`` uses the production model factory with wire tracing.
    """

    config: SingleCallAgentConfig

    @abstractmethod
    def _observation_blocks(self, env: RunningEnvironment) -> Blocks:
        """The observations this agent includes before the instruction."""

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        blocks = self._observation_blocks(env)
        if self.config.chat_model is None:
            # The production factory loads optional model-provider dependencies.
            from dimos.agents.mcp.mcp_client import init_model

            chat = init_model(self.config.model, trace_dir=run_dir / "raw")
            model_name = self.config.model
        else:
            chat = self.config.chat_model
            model_name = type(chat).__name__
        trajectory = TrajectoryBuilder(inputs, name=type(self).__name__, model=model_name)
        messages: list[BaseMessage] = [
            SystemMessage(self.config.system_prompt),
            HumanMessage(content=[*blocks, {"type": "text", "text": inputs}]),
        ]
        started_at = time.time()
        started = time.monotonic()
        result = chat.generate([messages])
        latency_s = time.monotonic() - started
        generation = result.generations[0][0]
        if not isinstance(generation, ChatGeneration) or not isinstance(
            generation.message, AIMessage
        ):
            raise TypeError("chat model must return an AIMessage")
        pair = latest_pair(run_dir / "raw", 0)
        if pair is None:
            pair = write_normalized(run_dir / "raw", messages, result)
        append_ai_message_to_atif(
            trajectory,
            generation.message,
            request=pair[1],
            response=pair[2],
            latency_s=latency_s,
            at=started_at,
        )
        return trajectory.build("answer")
