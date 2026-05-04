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

import asyncio
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


class MakeCube(Module):
    @rpc
    async def make_cube(self, x: int) -> int:
        await asyncio.sleep(0.001)  # Just so it's actually async.
        return x * x * x


class MakeCubeSpec(Spec, Protocol):
    async def make_cube(self, x: int) -> int: ...


class StartModule(Module):
    a: In[int]
    cube_a: Out[int]
    _cuber: MakeCubeSpec

    @rpc
    async def handle_a(self, x: int) -> None:
        cube = await self._cuber.make_cube(x)
        self.cube_a.publish(cube)


@pytest.fixture
def start_cube_module():
    blueprint = autoconnect(StartModule.blueprint(), MakeCube.blueprint())
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
def cube_a_transport():
    cube_a_tr = pLCMTransport("/cube_a")
    cube_a_tr.start()
    yield cube_a_tr
    cube_a_tr.stop()


@pytest.mark.slow
def test_async_module_rpc(start_cube_module, a_transport, cube_a_transport):
    queue = Queue()
    cube_a_transport.subscribe(queue.put)
    a_transport.publish(3)
    cubed = queue.get(timeout=0.1)
    assert cubed == 27
