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

from dimos.core import Module, rpc, start, stop
from dimos.protocol.rpc.lcmrpc import LCMRPC
from dimos.protocol.rpc.spec import RPCClient, RPCServer
from dimos.protocol.service.lcmservice import autoconf

testgrid: List[Callable] = []


# test module we'll use for binding RPC methods
class MyModule(Module):
    @rpc
    def add(self, a: int, b: int = 30) -> int:
        print(f"A + B = {a + b}")
        return a + b

    @rpc
    def subtract(self, a: int, b: int) -> int:
        print(f"A - B = {a - b}")
        return a - b


# This tests a generic RPC-over-PubSub implementation that can be used via any
# pubsub transport such as LCM or Redis in this test.
#
# (For transport systems that have call/reply type of functionaltity, we will
# not use PubSubRPC but implement protocol native RPC conforimg to
# RPCClient/RPCServer spec in spec.py)


# LCMRPC (mixed in PassThroughPubSubRPC into lcm pubsub)
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


# RedisRPC (mixed in in PassThroughPubSubRPC into redis pubsub)
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

        # You can bind an arbitrary function to arbitrary name
        # topics are:
        #
        # - /rpc/add/req
        # - /rpc/add/res
        server.serve_rpc(remote_function, "add")

        msgs = []

        def receive_msg(response):
            msgs.append(response)
            print(f"Received response: {response}")

        client.call("add", ([1, 2], {}), receive_msg)

        time.sleep(0.1)
        assert len(msgs) > 0


@pytest.mark.parametrize("rpc_context", testgrid)
def test_module_autobind(rpc_context):
    with rpc_context() as (server, client):
        module = MyModule()
        print("\n")

        # We take an endpoint name from __class__.__name__,
        # so topics are:
        #
        # - /rpc/MyModule/method_name1/req
        # - /rpc/MyModule/method_name1/res
        #
        # - /rpc/MyModule/method_name2/req
        # - /rpc/MyModule/method_name2/res
        #
        # etc
        server.serve_module_rpc(module)

        # can override the __class__.__name__ with something else
        server.serve_module_rpc(module, "testmodule")

        msgs = []

        def receive_msg(msg):
            msgs.append(msg)

        client.call("MyModule/add", ([1, 2], {}), receive_msg)
        client.call("testmodule/subtract", ([3, 1], {}), receive_msg)

        time.sleep(0.1)
        assert len(msgs) == 2
        assert msgs == [3, 2]


# Default rpc.call() either doesn't wait for response or accepts a callback
# but also we support different calling strategies,
#
# can do blocking calls
@pytest.mark.parametrize("rpc_context", testgrid)
def test_sync(rpc_context):
    with rpc_context() as (server, client):
        module = MyModule()
        print("\n")

        server.serve_module_rpc(module)
        assert 3 == client.call_sync("MyModule/add", ([1, 2], {}))


# Default rpc.call() either doesn't wait for response or accepts a callback
# but also we support different calling strategies,
#
# can do blocking calls
@pytest.mark.parametrize("rpc_context", testgrid)
def test_kwargs(rpc_context):
    with rpc_context() as (server, client):
        module = MyModule()
        print("\n")

        server.serve_module_rpc(module)

        assert 3 == client.call_sync("MyModule/add", ([1, 2], {}))


# or async calls as well
@pytest.mark.parametrize("rpc_context", testgrid)
@pytest.mark.asyncio
async def test_async(rpc_context):
    with rpc_context() as (server, client):
        module = MyModule()
        print("\n")
        server.serve_module_rpc(module)
        assert 3 == await client.call_async("MyModule/add", ([1, 2], {}))


# or async calls as well
@pytest.mark.module
def test_rpc_full_deploy():
    autoconf()

    # test module we'll use for binding RPC methods
    class CallerModule(Module):
        remote: Callable[[int, int], int]

        def __init__(self, remote: Callable[[int, int], int]):
            self.remote = remote
            super().__init__()

        @rpc
        def add(self, a: int, b: int = 30) -> int:
            return self.remote(a, b)

    dimos = start(2)

    module = dimos.deploy(MyModule)
    caller = dimos.deploy(CallerModule, module.add)
    print("deployed", module)
    print("deployed", caller)

    # standard list args
    assert caller.add(1, 2) == 3
    # default args
    assert caller.add(1) == 31
    # kwargs
    assert caller.add(1, b=1) == 2

    dimos.shutdown()
