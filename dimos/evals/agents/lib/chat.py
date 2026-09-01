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

"""What every agent that calls a chat model itself shares: its configuration
(:class:`ChatAgent`) and the one call it makes (:func:`single_call`)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from langchain_core.messages import HumanMessage, SystemMessage

from dimos.evals.agents.lib.langchain_recorder import LangChainRecorder
from dimos.evals.types import Trajectory

DEFAULT_MODEL = "gpt-5.6-luna"

EVAL_SYSTEM_PROMPT = "Answer the question using only the provided observations."

Blocks = list[str | dict[str, Any]]  # HumanMessage content blocks


@dataclass
class ChatAgent:
    """What every agent that calls a chat model itself is configured with.

    ``chat_model`` injects a model instance (house convention: a fake in
    tests) and is recorded under its class name; None builds ``model`` as
    production does, with the wire trace on. ``modules`` are blueprint atoms
    added to a ``Sim`` case's stack; an environment that launches nothing
    rejects them.
    """

    model: str = DEFAULT_MODEL
    system_prompt: str = EVAL_SYSTEM_PROMPT
    chat_model: Any | None = None
    modules: str = ""

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        """A direct chat call has no tools."""
        return ()

    def chat(self, run_dir: Path) -> tuple[Any, str]:
        """The chat model for one run and the name to record for it."""
        if self.chat_model is not None:
            return self.chat_model, type(self.chat_model).__name__
        from dimos.agents.mcp.mcp_client import _init_model

        return _init_model(self.model, trace_dir=run_dir / "raw"), self.model


def single_call(agent: ChatAgent, blocks: Blocks, inputs: str, run_dir: Path) -> Trajectory:
    """One model call: *blocks* then the instruction."""
    chat, model = agent.chat(run_dir)
    recorder = LangChainRecorder(
        inputs, name=type(agent).__name__, model=model, raw_dir=run_dir / "raw"
    )
    chat.invoke(
        [
            SystemMessage(agent.system_prompt),
            HumanMessage(content=[*blocks, {"type": "text", "text": inputs}]),
        ],
        config={"callbacks": [recorder]},
    )
    return recorder.build("answer")
