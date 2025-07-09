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
import time
from contextlib import contextmanager
from typing import Any, Callable, List, Tuple

import pytest

from dimos.core import Module, rpc
from dimos.protocol.rpc.lcmrpc import LCMRPC
from dimos.protocol.rpc.spec import RPCClient, RPCServer

testgrid: List[Callable] = []


class MyModule(Module):
    @rpc
    def add(self, a: int, b: int) -> int:
        print("A + B", a + b)
        return a + b

    @rpc
    def subtract(self, a: int, b: int) -> int:
        print("A - B", a - b)
        return a - b


@contextmanager
def lcm_rpc_context():
    server = LCMRPC(autoconf=True)
    client = LCMRPC(autoconf=True)
    server.start()
    client.start()
    yield [server, client]
    server.stop()
    client.stop()


testgrid.append(lcm_rpc_context)


try:
    from dimos.protocol.rpc.redisrpc import RedisRPC

    @contextmanager
    def redis_rpc_context():
        server = RedisRPC()
        client = RedisRPC()
        server.start()
        client.start()
        yield [server, client]
        server.stop()
        client.stop()

    testgrid.append(redis_rpc_context)

except (ConnectionError, ImportError):
    print("Redis not available")


@pytest.mark.parametrize("rpc_context", testgrid)
def test_basics(rpc_context):
    with rpc_context() as (server, client):

        def remote_function(a: int, b: int):
            return a + b

        server.serve_rpc(remote_function, "add")

        msgs = []

        def receive_msg(response):
            msgs.append(response)
            print(f"Received response: {response}")

        client.call("add", [1, 2], receive_msg)

        time.sleep(0.1)
        assert len(msgs) > 0


@pytest.mark.parametrize("rpc_context", testgrid)
def test_module_autobind(rpc_context):
    with rpc_context() as (server, client):
        module = MyModule()

        server.serve_module_rpc(module)

        server.serve_module_rpc(module, "testmodule")

        msgs = []

        def receive_msg(msg):
            msgs.append(msg)

        client.call("MyModule/add", [1, 2], receive_msg)
        client.call("testmodule/subtract", [3, 1], receive_msg)

        time.sleep(0.1)
        assert msgs == [3, 2]
        assert len(msgs) == 2


@pytest.mark.parametrize("rpc_context", testgrid)
def test_module_autobind(rpc_context):
    with rpc_context() as (server, client):
        module = MyModule()

        server.serve_module_rpc(module)
        server.serve_module_rpc(module, "testmodule")

        msgs = []

        def receive_msg(msg):
            msgs.append(msg)

        client.call("MyModule/add", [1, 2], receive_msg)
        client.call("testmodule/subtract", [3, 1], receive_msg)

        time.sleep(0.1)
        assert len(msgs) == 2
        assert msgs == [3, 2]


@pytest.mark.parametrize("rpc_context", testgrid)
def test_sync(rpc_context):
    with rpc_context() as (server, client):
        module = MyModule()

        server.serve_module_rpc(module)
        assert 3 == client.call_sync("MyModule/add", [1, 2])


@pytest.mark.parametrize("rpc_context", testgrid)
def test_async(rpc_context):
    with rpc_context() as (server, client):
        module = MyModule()
        server.serve_module_rpc(module)

        async def atest():
            print("RUNING TEST")
            val = await client.call_async("MyModule/add", [1, 2])
            print("ASYNC TEST RESULT", val)
            assert val == 3

        asyncio.run(atest())
