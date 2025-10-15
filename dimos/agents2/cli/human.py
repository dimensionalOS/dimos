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

from dimos.agents2 import Output, Reducer, Stream, skill
from dimos.core import Module, pLCMTransport, rpc
from reactivex.disposable import Disposable


class HumanInput(Module):
    running: bool = False

    @skill(stream=Stream.call_agent, reducer=Reducer.string, output=Output.human)
    def human(self):
        """receives human input, no need to run this, it's running implicitly"""
        if self.running:
            return "already running"
        self.running = True
        transport = pLCMTransport("/human_input")

        msg_queue = queue.Queue()
        unsub = transport.subscribe(msg_queue.put)
        self._disposables.add(Disposable(unsub))
        for message in iter(msg_queue.get, None):
            yield message

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()
