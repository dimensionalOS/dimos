# Copyright 2025-2026 Dimensional Inc.
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
from collections.abc import Callable

import pytest

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.type.map import Map as Mapper


@pytest.fixture
def dimos():
    ret = ModuleCoordinator()
    ret.start()
    try:
        yield ret
    finally:
        ret.stop()


class Consumer:
    testf: Callable[[int], int]

    def __init__(self, counter=None) -> None:
        self.testf = counter
        self._tasks: set[asyncio.Task[None]] = set()
        print("consumer init with", counter)

    async def waitcall(self, n: int):
        async def task() -> None:
            await asyncio.sleep(n)

            print("sleep finished, calling")
            res = await self.testf(n)
            print("res is", res)

        background_task = asyncio.create_task(task())
        self._tasks.add(background_task)
        background_task.add_done_callback(self._tasks.discard)
        return n


class Counter(Module):
    @rpc
    def addten(self, x: int):
        print(f"counter adding to {x}")
        return x + 10


@pytest.mark.tool
def test_basic(dimos) -> None:
    counter = dimos.deploy(Counter)
    consumer = dimos.deploy(
        Consumer,
        counter=lambda x: counter.addten(x).result(),
    )

    print(consumer)
    print(counter)
    print("starting consumer")
    consumer.start().result()

    res = consumer.inc(10).result()

    print("result is", res)
    assert res == 20


@pytest.mark.tool
def test_mapper_start(dimos) -> None:
    mapper = dimos.deploy(Mapper)
    mapper.lidar.transport = LCMTransport("/lidar", PointCloud2)
    print("start res", mapper.start().result())


@pytest.mark.tool
def test_counter(dimos) -> None:
    counter = dimos.deploy(Counter)
    assert counter.addten(10) == 20
