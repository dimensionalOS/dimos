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

from threading import Event, Thread

from langchain_core.messages import AIMessage
from langchain_core.messages.base import BaseMessage
from reactivex.disposable import Disposable

from dimos.agents.agent import AgentSpec
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In, Out


class AgentTestRunner(Module):
    agent_spec: AgentSpec
    agent: In[BaseMessage]
    agent_idle: In[bool]
    finished: Out[bool]
    added: Out[bool]

    def __init__(self, messages: list[BaseMessage]) -> None:
        super().__init__()
        self._messages = messages
        self._idle_event = Event()
        self._subscription_ready = Event()
        self._thread = Thread(target=self._thread_loop, daemon=True)

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(Disposable(self.agent.subscribe(self._on_agent_message)))
        self._disposables.add(Disposable(self.agent_idle.subscribe(self._on_agent_idle)))
        # Signal that subscription is ready
        self._subscription_ready.set()

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    def on_system_modules(self, _modules: list[RPCClient]) -> None:
        self._thread.start()

    def _on_agent_idle(self, idle: bool) -> None:
        if idle:
            self._idle_event.set()

    def _on_agent_message(self, message: BaseMessage) -> None:
        # Check for final AIMessage (no tool calls) to signal completion
        is_ai = isinstance(message, AIMessage)
        has_tool_calls = hasattr(message, "tool_calls") and message.tool_calls
        if is_ai and not has_tool_calls:
            self.added.publish(True)

    def _thread_loop(self) -> None:
        # Ensure subscription is ready before sending messages
        if not self._subscription_ready.wait(5):
            raise TimeoutError("Timed out waiting for subscription to be ready.")

        for message in self._messages:
            self._idle_event.clear()
            self.agent_spec.add_message(message)
            if not self._idle_event.wait(60):
                raise TimeoutError("Timed out waiting for message to be processed.")

        self.finished.publish(True)
