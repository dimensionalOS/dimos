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
import asyncio
import functools
import time
from typing import Callable

import pytest
from reactivex import operators as ops

from dimos import core
from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.protocol import pubsub
from dimos.robot.global_planner import AstarPlanner
from dimos.robot.local_planner.simple import SimplePlanner
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.robot.unitree_webrtc.type.map import Map as Mapper
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.types.costmap import Costmap
from dimos.types.vector import Vector
from dimos.utils.reactive import backpressure, getter_streaming
from dimos.utils.testing import TimedSensorReplay


@pytest.fixture
def dimos():
    return core.start(2)


@pytest.fixture
def client():
    return core.start(2)


class Consumer:
    testf: Callable[[int], int]

    def __init__(self, counter=None):
        self.testf = counter
        print("consumer init with", counter)

    async def waitcall(self, n: int):
        async def task():
            await asyncio.sleep(n)

            print("sleep finished, calling")
            res = await self.testf(n)
            print("res is", res)

        asyncio.create_task(task())
        return n


class Counter(Module):
    @rpc
    def addten(self, x: int):
        print(f"counter adding to {x}")
        return x + 10


@pytest.mark.tool
def test_wait(client):
    counter = client.submit(Counter, actor=True).result()

    async def addten(n):
        return await counter.addten(n)

    consumer = client.submit(Consumer, counter=addten, actor=True).result()

    print("waitcall1", consumer.waitcall(2).result())
    print("waitcall2", consumer.waitcall(2).result())
    time.sleep(1)


@pytest.mark.tool
def test_basic(dimos):
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
def test_mapper_start(dimos):
    mapper = dimos.deploy(Mapper)
    mapper.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
    print("start res", mapper.start().result())


if __name__ == "__main__":
    dimos = core.start(2)
    test_basic(dimos)
    test_mapper_start(dimos)


@pytest.mark.tool
def test_counter(dimos):
    counter = dimos.deploy(Counter)
    assert counter.addten(10) == 20
