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
from contextlib import ExitStack
from pathlib import Path
import time
from typing import Any

import httpx
from langchain_core.language_models import BaseChatModel
from langchain_core.messages import AIMessage, BaseMessage, HumanMessage, SystemMessage
from langchain_core.outputs import ChatGeneration
from openai import APIError, APITimeoutError
from pydantic import Field

from dimos.agents.llm_trace import latest_pair, tracing_http_client, write_normalized
from dimos.evals.agents.base import Agent, ModelAgentConfig
from dimos.evals.agents.lib.langchain_to_atif import append_ai_message_to_atif
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.types import RunningEnvironment, Trajectory

Blocks = list[str | dict[str, Any]]


class SingleCallAgentConfig(ModelAgentConfig):
    system_prompt: str | None = "Answer the question using only the provided observations."
    chat_model: BaseChatModel | None = None
    request_timeout_s: float | None = Field(default=None, gt=0, allow_inf_nan=False)
    max_retries: int | None = Field(default=None, ge=0)
    max_output_tokens: int | None = Field(default=None, gt=0)


class SingleCallAgent(Agent):
    """Encode observations, call a chat model once, and record the result.

    ``chat_model`` injects a LangChain model, including a fake for offline evals.
    Otherwise ``model`` uses the production model factory with wire tracing.
    """

    config: SingleCallAgentConfig

    def validate_tools(self) -> None:
        if self.config.allowed_tools:
            raise ValueError(f"{type(self).__name__} has no tools")

    @abstractmethod
    def _observation_blocks(self, env: RunningEnvironment) -> Blocks:
        """The observations this agent includes before the instruction."""

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        blocks = self._observation_blocks(env)
        with ExitStack() as resources:
            if self.config.chat_model is None:
                # The production factory loads optional model-provider dependencies.
                from dimos.agents.mcp.mcp_client import init_model

                request_timeout = self.config.request_timeout_s
                if request_timeout is not None:
                    if timeout_s <= 0:
                        return TrajectoryBuilder(
                            inputs, name=type(self).__name__, model=self.config.model
                        ).build("timeout", error="case budget expired before the request")
                    request_timeout = min(request_timeout, timeout_s)
                client = resources.enter_context(tracing_http_client(run_dir / "raw"))
                chat = init_model(
                    self.config.model,
                    http_client=client,
                    timeout=request_timeout,
                    max_retries=self.config.max_retries,
                    max_tokens=self.config.max_output_tokens,
                )
                model_name = self.config.model
            else:
                chat = self.config.chat_model
                model_name = type(chat).__name__
            return self._generate(inputs, blocks, chat, model_name, run_dir)

    def _generate(
        self,
        inputs: str,
        blocks: Blocks,
        chat: BaseChatModel,
        model_name: str,
        run_dir: Path,
    ) -> Trajectory:
        trajectory = TrajectoryBuilder(inputs, name=type(self).__name__, model=model_name)
        messages: list[BaseMessage] = []
        if self.config.system_prompt:
            messages.append(SystemMessage(self.config.system_prompt))
        messages.append(HumanMessage(content=[*blocks, {"type": "text", "text": inputs}]))
        started_at = time.time()
        started = time.monotonic()
        try:
            result = chat.generate([messages])
        except (APITimeoutError, httpx.TimeoutException) as exc:
            return trajectory.build("timeout", error=f"{type(exc).__name__}: {exc}")
        except (APIError, httpx.TransportError) as exc:
            return trajectory.build("error", error=f"{type(exc).__name__}: {exc}")
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
