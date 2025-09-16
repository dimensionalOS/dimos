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

import threading
import time

import pytest

from dimos.protocol.rpc.lcmrpc import LCMRPC
from dimos.protocol.service.lcmservice import autoconf


@pytest.fixture(scope="session", autouse=True)
def setup_lcm_autoconf():
    """Setup LCM autoconf once for the entire test session"""
    autoconf()
    yield


@pytest.fixture
def lcm_server():
    """Fixture that provides started LCMRPC server"""
    server = LCMRPC()
    server.start()

    yield server

    server.stop()


@pytest.fixture
def lcm_client():
    """Fixture that provides started LCMRPC client"""
    client = LCMRPC()
    client.start()

    yield client

    client.stop()


def test_lcmrpc_timeout_no_reply(lcm_server, lcm_client):
    """Test that RPC calls timeout when no reply is received"""
    server = lcm_server
    client = lcm_client

    # Track if the function was called
    function_called = threading.Event()

    # Serve a function that never responds
    def never_responds(a: int, b: int):
        # Signal that the function was called
        function_called.set()
        # Simulating a server that receives the request but never sends a reply
        time.sleep(1)  # Long sleep to ensure timeout happens first
        return a + b

    server.serve_rpc(never_responds, "slow_add")

    # Test with call_sync and explicit timeout
    start_time = time.time()

    # Should raise TimeoutError when timeout occurs
    with pytest.raises(TimeoutError, match="RPC call to 'slow_add' timed out after 0.1 seconds"):
        client.call_sync("slow_add", ([1, 2], {}), rpc_timeout=0.1)

    elapsed = time.time() - start_time

    # Should timeout after ~0.1 seconds
    assert elapsed < 0.3, f"Timeout took too long: {elapsed}s"

    # Verify the function was actually called
    assert function_called.wait(0.5), "Server function was never called"


def test_lcmrpc_timeout_nonexistent_service(lcm_client):
    """Test that RPC calls timeout when calling a non-existent service"""
    client = lcm_client

    # Call a service that doesn't exist
    start_time = time.time()

    # Should raise TimeoutError when timeout occurs
    with pytest.raises(
        TimeoutError, match="RPC call to 'nonexistent/service' timed out after 0.1 seconds"
    ):
        client.call_sync("nonexistent/service", ([1, 2], {}), rpc_timeout=0.1)

    elapsed = time.time() - start_time

    # Should timeout after ~0.1 seconds
    assert elapsed < 0.3, f"Timeout took too long: {elapsed}s"


def test_lcmrpc_callback_with_timeout(lcm_server, lcm_client):
    """Test that callback-based RPC calls handle timeouts properly"""
    server = lcm_server
    client = lcm_client
    # Track if the function was called
    function_called = threading.Event()

    # Serve a function that never responds
    def never_responds(a: int, b: int):
        function_called.set()
        time.sleep(1)
        return a + b

    server.serve_rpc(never_responds, "slow_add")

    callback_called = threading.Event()
    received_value = []

    def callback(value):
        received_value.append(value)
        callback_called.set()

    # Make the call with callback
    unsub = client.call("slow_add", ([1, 2], {}), callback)

    # Wait for a short time - callback should not be called
    callback_called.wait(0.2)
    assert not callback_called.is_set(), "Callback should not have been called"
    assert len(received_value) == 0

    # Verify the server function was actually called
    assert function_called.wait(0.5), "Server function was never called"

    # Clean up - unsubscribe if possible
    if unsub:
        unsub()


def test_lcmrpc_normal_operation(lcm_server, lcm_client):
    """Sanity check that normal RPC calls still work"""
    server = lcm_server
    client = lcm_client

    def quick_add(a: int, b: int):
        return a + b

    server.serve_rpc(quick_add, "add")

    # Normal call should work quickly
    start_time = time.time()
    result = client.call_sync("add", ([5, 3], {}), rpc_timeout=0.5)[0]
    elapsed = time.time() - start_time

    assert result == 8
    assert elapsed < 0.2, f"Normal call took too long: {elapsed}s"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
