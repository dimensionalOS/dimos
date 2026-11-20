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

"""The harness agent (`dimos agent`) behind the BaseAgent contract."""

import json
from pathlib import Path
import re
import shutil
import subprocess
from threading import Lock, Thread
from typing import Any

from langchain_core.messages import AIMessage, HumanMessage
from reactivex.disposable import Disposable

from dimos.agents.base_agent import BaseAgent
from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out

_CONVERSATION = re.compile(r"^Conversation (\d+)$", re.MULTILINE)


class HarnessAgentConfig(ModuleConfig):
    cli: str = "dimos"
    # Where the harness agent's file and shell tools work. None: this process's directory.
    cwd: Path | None = None
    # An existing harness conversation to continue. None: the first turn opens one.
    conversation: int | None = None


class HarnessAgent(BaseAgent[HumanMessage, AIMessage]):
    """One harness conversation, called like any other agent.

    Each turn runs `dimos agent run --json` against the harness gateway, which starts if it
    is not running. The conversation lives in the gateway's store, so it outlasts this
    object; pass its number as `conversation` to pick it up again. Text only: images in the
    message are not sent.
    """

    config: HarnessAgentConfig

    agent_input: In[HumanMessage]
    agent_output: Out[AIMessage]

    _turn: Lock

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._turn = Lock()
        self.conversation = self.config.conversation

    @rpc
    def start(self) -> None:
        super().start()

        def on_input(message: HumanMessage) -> None:
            # ponytail: a thread per message; turns queue on the lock, not in arrival order
            # under contention. A queue and one worker if ordering matters.
            Thread(target=self.infer, args=(message,), daemon=True).start()

        # No transport: the agent is used as a plain class and only infer() reaches it.
        if self.agent_input.transport is not None:
            self.register_disposable(Disposable(self.agent_input.subscribe(on_input)))

    @rpc
    def infer(self, x: HumanMessage) -> AIMessage:
        """Send x as one turn of the conversation and return the agent's final answer."""
        with self._turn:
            self.agent_idle.publish(False)
            self.agent.publish(x)
            try:
                answer = self._run(str(x.text))
            finally:
                self.agent_idle.publish(True)
            self.agent.publish(answer)
            self.agent_output.publish(answer)
            return answer

    def _run(self, prompt: str) -> AIMessage:
        command = [shutil.which(self.config.cli) or self.config.cli, "agent", "run", "--json"]
        if self.conversation is not None:
            command += ["--conversation", str(self.conversation)]
        if self.config.cwd is not None:
            command += ["--cwd", str(self.config.cwd)]
        done = subprocess.run([*command, prompt], capture_output=True, text=True)
        opened = _CONVERSATION.search(done.stderr)
        if opened is not None:
            self.conversation = int(opened[1])
        return _answer(done.stdout, done.stderr)


def _answer(stdout: str, stderr: str) -> AIMessage:
    """The answer in the JSON lines `dimos agent run --json` printed."""
    # Native-module warnings can land on stdout too; only the JSON lines count.
    lines = [json.loads(line) for line in stdout.splitlines() if line.startswith("{")]
    outcome = next((line["outcome"] for line in reversed(lines) if "outcome" in line), None)
    if outcome is None:
        raise RuntimeError(f"dimos agent run printed no outcome:\n{stderr.strip()[-2000:]}")
    if outcome["status"] != "done":
        raise RuntimeError(f"dimos agent run ended unanswered: {outcome.get('reason')}")
    for line in lines:
        item = line.get("item", {})
        if item.get("kind") == "assistant" and item.get("entry") == outcome["answer"]:
            blocks = item["message"]["content"]
            return AIMessage("".join(b["text"] for b in blocks if b.get("type") == "text"))
    raise RuntimeError(f"dimos agent run named answer {outcome['answer']} but did not print it")
