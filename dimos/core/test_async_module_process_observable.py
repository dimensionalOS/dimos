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

from queue import Queue
import string

import pytest
import reactivex as rx
from reactivex import operators as ops

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.core.transport import pLCMTransport


class StartModule(Module):
    uppercase: Out[str]

    @rpc
    def start(self) -> None:
        super().start()

        observable = rx.interval(0.1).pipe(
            ops.take(len(string.ascii_lowercase)),
            ops.map(lambda i: string.ascii_lowercase[i]),
        )

        self.process_observable(observable, self.handle_letter)

    async def handle_letter(self, letter: str) -> None:
        self.uppercase.publish(letter.upper())


@pytest.fixture
def start_module():
    blueprint = StartModule.blueprint()
    coordinator = ModuleCoordinator.build(blueprint)
    yield
    coordinator.stop()


@pytest.fixture
def get_collected_letters():
    uppercase_transport = pLCMTransport("/uppercase")
    uppercase_transport.start()
    queue = Queue()
    uppercase_transport.subscribe(queue.put)

    def _get_collected_letters() -> list[str]:
        return "".join([queue.get(timeout=4) for _ in range(26)])

    yield _get_collected_letters

    uppercase_transport.stop()


@pytest.mark.slow
def test_async_module_process_observable(get_collected_letters, start_module):
    """
    Tests that process_observable correctly processes items from an observable
    in an async manner.

    Most of the logic is in get_collected_letters, because we need to setup the
    subscription to the result before starting the module. This is because the
    module emits from the start method.

    The strict equality below also locks down the serial-delivery contract: the
    per-subscription dispatcher must invoke `handle_letter` once per item in the
    order they were emitted (the source emits at 100ms intervals, slower than the
    near-zero handler runtime, so no LATEST coalescing should occur).
    """
    collected = get_collected_letters()
    assert len(collected) == 26
    assert collected == "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
