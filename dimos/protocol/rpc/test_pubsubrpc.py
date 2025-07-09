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

import time
from contextlib import contextmanager
from typing import Any, Callable, List, Tuple

import pytest

from dimos.protocol.rpc.lcmrpc import LCMRPC
from dimos.protocol.rpc.spec import RPCClient, RPCServer

testgrid: List[Callable] = []


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

        client.call_cb("add", [1, 2], receive_msg)

        time.sleep(0.2)
        assert len(msgs) > 0
        server.stop()
        client.stop()
