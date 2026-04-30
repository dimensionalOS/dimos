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

import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import pLCMTransport


class DoubleModule(Module):
    a: In[int]
    double_a: Out[int]

    async def handle_a(self, a: int) -> None:
        self.double_a.publish(a * 2)


@pytest.fixture
def start_double_module():
    blueprint = DoubleModule.blueprint()
    coordinator = ModuleCoordinator.build(blueprint)
    yield
    coordinator.stop()


@pytest.fixture
def a_transport():
    a_tr = pLCMTransport("/a")
    a_tr.start()
    yield a_tr
    a_tr.stop()


@pytest.fixture
def double_a_transport():
    double_a_tr = pLCMTransport("/double_a")
    double_a_tr.start()
    yield double_a_tr
    double_a_tr.stop()


@pytest.mark.slow
def test_async_module_handles(start_double_module, a_transport, double_a_transport):
    queue = Queue()
    double_a_transport.subscribe(queue.put)
    a_transport.publish(42)
    doubled = queue.get(timeout=0.1)
    assert doubled == 84
