# Copyright 2025 Dimensional Inc.
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

import queue

from reactivex.disposable import Disposable

from dimos.agents import Output, Reducer, Stream, skill  # type: ignore[attr-defined]
from dimos.core import pLCMTransport, rpc
from dimos.core.module import Module
from dimos.core.rpc_client import RpcCall


class HumanInput(Module):
    running: bool = False

    @skill(stream=Stream.call_agent, reducer=Reducer.string, output=Output.human, hide_skill=True)  # type: ignore[arg-type]
    def human(self):  # type: ignore[no-untyped-def]
        """receives human input, no need to run this, it's running implicitly"""
        if self.running:
            return "already running"
        self.running = True
        transport = pLCMTransport("/human_input")  # type: ignore[var-annotated]

        msg_queue = queue.Queue()  # type: ignore[var-annotated]
        unsub = transport.subscribe(msg_queue.put)  # type: ignore[func-returns-value]
        self._disposables.add(Disposable(unsub))
        yield from iter(msg_queue.get, None)

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    def set_LlmAgent_register_skills(self, callable: RpcCall) -> None:
        callable.set_rpc(self.rpc)  # type: ignore[arg-type]
        callable(self, run_implicit_name="human")


human_input = HumanInput.blueprint

__all__ = ["HumanInput", "human_input"]
