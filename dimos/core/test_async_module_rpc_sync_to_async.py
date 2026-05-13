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
from typing import Protocol

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.core.transport import pLCMTransport
from dimos.spec.utils import Spec


class ModuleA(Module):
    @rpc
    def a(self, x: int) -> int:
        return x * 1000


class ModuleB(Module):
    @rpc
    async def b(self, x: int) -> int:
        return x * 100


class ASpec(Spec, Protocol):
    def a(self, x: int) -> int: ...


class BSpec(Spec, Protocol):
    # ModuleB.b is async but we use sync in the spec
    def b(self, x: int) -> int: ...


class ModuleAB(Module):
    _a: ASpec
    _b: BSpec

    @rpc
    def ab(self, x: int) -> int:
        return self._a.a(x) + self._b.b(x)


class ABSpec(Spec, Protocol):
    # ModuleAB.ab is sync but we use async in the spec
    async def ab(self, x: int) -> int: ...


class StartModule(Module):
    in_value: In[int]
    out_value: Out[int]
    _ab: ABSpec

    @rpc
    async def handle_in_value(self, x: int) -> None:
        ret = await self._ab.ab(x)
        self.out_value.publish(ret)


@pytest.fixture
def start_module():
    blueprint = autoconnect(
        StartModule.blueprint(),
        ModuleA.blueprint(),
        ModuleB.blueprint(),
        ModuleAB.blueprint(),
    )
    coordinator = ModuleCoordinator.build(blueprint)
    yield
    coordinator.stop()


@pytest.fixture
def in_transport():
    ret = pLCMTransport("/in_value")
    ret.start()
    yield ret
    ret.stop()


@pytest.fixture
def out_transport():
    ret = pLCMTransport("/out_value")
    ret.start()
    yield ret
    ret.stop()


def test_async_module_rpc_sync_to_async(start_module, in_transport, out_transport):
    """
    Test that you can call a synchronous RPC from an asynchronous RPC and vice versa.
    """
    queue = Queue()
    out_transport.subscribe(queue.put)
    in_transport.publish(4)
    cubed = queue.get(timeout=0.1)
    assert cubed == 4400
