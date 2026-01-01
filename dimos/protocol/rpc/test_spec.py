#!/usr/bin/env python3

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

"""Grid tests for RPC implementations to ensure spec compliance."""

import asyncio
from collections.abc import Callable
from contextlib import contextmanager
import threading
import time
from typing import Any

import pytest

from dimos.protocol.rpc.pubsubrpc import LCMRPC, ShmRPC
from dimos.protocol.rpc.rpc_utils import RemoteError


class CustomTestError(Exception):
    """Custom exception for testing."""

    pass


# Build testdata list with available implementations
testdata: list[tuple[Callable[[], Any], str]] = []


# Context managers for different RPC implementations
@contextmanager
def lcm_rpc_context():
    """Context manager for LCMRPC implementation."""
    from dimos.protocol.service.lcmservice import autoconf

    autoconf()
    server = LCMRPC()
    client = LCMRPC()
    server.start()
    client.start()

    try:
        yield server, client
    finally:
        server.stop()
        client.stop()


testdata.append((lcm_rpc_context, "lcm"))


@contextmanager
def shm_rpc_context():
    """Context manager for Shared Memory RPC implementation."""
    # Create two separate instances that communicate through shared memory segments
    server = ShmRPC(prefer="cpu")
    client = ShmRPC(prefer="cpu")
    server.start()
    client.start()

    try:
        yield server, client
    finally:
        server.stop()
        client.stop()


testdata.append((shm_rpc_context, "shm"))

# Try to add RedisRPC if available
try:
    from dimos.protocol.rpc.redisrpc import RedisRPC

    @contextmanager
    def redis_rpc_context():
        """Context manager for RedisRPC implementation."""
        server = RedisRPC()
        client = RedisRPC()
        server.start()
        client.start()

        try:
            yield server, client
        finally:
            server.stop()
            client.stop()

    testdata.append((redis_rpc_context, "redis"))
except (ImportError, ConnectionError):
    print("RedisRPC not available")


# Test functions that will be served
def add_function(a: int, b: int) -> int:
    """Simple addition function for testing."""
    return a + b


def failing_function(msg: str) -> str:
    """Function that raises exceptions for testing."""
    if msg == "fail":
        raise ValueError("Test error message")
    elif msg == "custom":
        raise CustomTestError("Custom error")
    return f"Success: {msg}"


def slow_function(delay: float) -> str:
    """Function that takes time to execute."""
    time.sleep(delay)
    return f"Completed after {delay} seconds"


# Grid tests


@pytest.mark.parametrize("rpc_context, impl_name", testdata)
def test_basic_sync_call(rpc_context, impl_name: str) -> None:
    """Test basic synchronous RPC calls."""
    with rpc_context() as (server, client):
        # Serve the function
        unsub = server.serve_rpc(add_function, "add")

        try:
            # Make sync call
            result, _ = client.call_sync("add", ([5, 3], {}), rpc_timeout=2.0)
            assert result == 8

            # Test with different arguments
            result, _ = client.call_sync("add", ([10, -2], {}), rpc_timeout=2.0)
            assert result == 8

        finally:
            unsub()


@pytest.mark.parametrize("rpc_context, impl_name", testdata)
@pytest.mark.asyncio
@pytest.mark.skip(
    reason="Async RPC calls have a deadlock issue when run in the full test suite (works in isolation)"
)
async def test_async_call(rpc_context, impl_name: str) -> None:
    """Test asynchronous RPC calls."""
    with rpc_context() as (server, client):
        # Serve the function
        unsub = server.serve_rpc(add_function, "add_async")

        try:
            # Make async call
            result = await client.call_async("add_async", ([7, 4], {}))
            assert result == 11

            # Test multiple async calls
            results = await asyncio.gather(
                client.call_async("add_async", ([1, 2], {})),
                client.call_async("add_async", ([3, 4], {})),
                client.call_async("add_async", ([5, 6], {})),
            )
            assert results == [3, 7, 11]

        finally:
            unsub()


@pytest.mark.parametrize("rpc_context, impl_name", testdata)
def test_callback_call(rpc_context, impl_name: str) -> None:
    """Test callback-based RPC calls."""
    with rpc_context() as (server, client):
        # Serve the function
        unsub_server = server.serve_rpc(add_function, "add_callback")

        try:
            # Test with callback
            event = threading.Event()
            received_value = None

            def callback(val) -> None:
                nonlocal received_value
                received_value = val
                event.set()

            client.call("add_callback", ([20, 22], {}), callback)
            assert event.wait(2.0)
            assert received_value == 42

        finally:
            unsub_server()


@pytest.mark.parametrize("rpc_context, impl_name", testdata)
def test_exception_handling_sync(rpc_context, impl_name: str) -> None:
    """Test that exceptions are properly passed through sync RPC calls."""
    with rpc_context() as (server, client):
        # Serve the function that can raise exceptions
        unsub = server.serve_rpc(failing_function, "test_exc")

        try:
            # Test successful call
            result, _ = client.call_sync("test_exc", (["ok"], {}), rpc_timeout=2.0)
            assert result == "Success: ok"

            # Test builtin exception - should raise actual ValueError
            with pytest.raises(ValueError) as exc_info:
                client.call_sync("test_exc", (["fail"], {}), rpc_timeout=2.0)
            assert "Test error message" in str(exc_info.value)
            # Check that the cause contains the remote traceback
            assert isinstance(exc_info.value.__cause__, RemoteError)
            assert "failing_function" in exc_info.value.__cause__.remote_traceback

            # Test custom exception - should raise RemoteError
            with pytest.raises(RemoteError) as exc_info:
                client.call_sync("test_exc", (["custom"], {}), rpc_timeout=2.0)
            assert "Custom error" in str(exc_info.value)
            assert "CustomTestError" in exc_info.value.remote_type
            assert "failing_function" in exc_info.value.remote_traceback

        finally:
            unsub()


@pytest.mark.parametrize("rpc_context, impl_name", testdata)
@pytest.mark.asyncio
async def test_exception_handling_async(rpc_context, impl_name: str) -> None:
    """Test that exceptions are properly passed through async RPC calls."""
    with rpc_context() as (server, client):
        # Serve the function that can raise exceptions
        unsub = server.serve_rpc(failing_function, "test_exc_async")

        try:
            # Test successful call
            result = await client.call_async("test_exc_async", (["ok"], {}))
            assert result == "Success: ok"

            # Test builtin exception
            with pytest.raises(ValueError) as exc_info:
                await client.call_async("test_exc_async", (["fail"], {}))
            assert "Test error message" in str(exc_info.value)
            assert isinstance(exc_info.value.__cause__, RemoteError)

            # Test custom exception
            with pytest.raises(RemoteError) as exc_info:
                await client.call_async("test_exc_async", (["custom"], {}))
            assert "Custom error" in str(exc_info.value)
            assert "CustomTestError" in exc_info.value.remote_type

        finally:
            unsub()


@pytest.mark.parametrize("rpc_context, impl_name", testdata)
def test_exception_handling_callback(rpc_context, impl_name: str) -> None:
    """Test that exceptions are properly passed through callback-based RPC calls."""
    with rpc_context() as (server, client):
        # Serve the function that can raise exceptions
        unsub_server = server.serve_rpc(failing_function, "test_exc_cb")

        try:
            # Test with callback - exception should be passed to callback
            event = threading.Event()
            received_value = None

            def callback(val) -> None:
                nonlocal received_value
                received_value = val
                event.set()

            # Test successful call
            client.call("test_exc_cb", (["ok"], {}), callback)
            assert event.wait(2.0)
            assert received_value == "Success: ok"
            event.clear()

            # Test failed call - exception should be passed to callback
            client.call("test_exc_cb", (["fail"], {}), callback)
            assert event.wait(2.0)
            assert isinstance(received_value, ValueError)
            assert "Test error message" in str(received_value)
            assert isinstance(received_value.__cause__, RemoteError)

        finally:
            unsub_server()


@pytest.mark.parametrize("rpc_context, impl_name", testdata)
def test_timeout(rpc_context, impl_name: str) -> None:
    """Test that RPC calls properly timeout."""
    with rpc_context() as (server, client):
        # Serve a slow function
        unsub = server.serve_rpc(slow_function, "slow")

        try:
            # Call with short timeout should fail
            # Using 10 seconds sleep to ensure it would definitely timeout
            with pytest.raises(TimeoutError) as exc_info:
                client.call_sync("slow", ([2.0], {}), rpc_timeout=0.1)
            assert "timed out" in str(exc_info.value)

            # Call with sufficient timeout should succeed
            result, _ = client.call_sync("slow", ([0.01], {}), rpc_timeout=1.0)
            assert "Completed after 0.01 seconds" in result

        finally:
            unsub()


@pytest.mark.parametrize("rpc_context, impl_name", testdata)
def test_nonexistent_service(rpc_context, impl_name: str) -> None:
    """Test calling a service that doesn't exist."""
    with rpc_context() as (_server, client):
        # Don't serve any function, just try to call
        with pytest.raises(TimeoutError) as exc_info:
            client.call_sync("nonexistent", ([1, 2], {}), rpc_timeout=0.1)
        assert "nonexistent" in str(exc_info.value)
        assert "timed out" in str(exc_info.value)


@pytest.mark.parametrize("rpc_context, impl_name", testdata)
def test_multiple_services(rpc_context, impl_name: str) -> None:
    """Test serving multiple RPC functions simultaneously."""
    with rpc_context() as (server, client):
        # Serve multiple functions
        unsub1 = server.serve_rpc(add_function, "service1")
        unsub2 = server.serve_rpc(lambda x: x * 2, "service2")
        unsub3 = server.serve_rpc(lambda s: s.upper(), "service3")

        try:
            # Call all services
            result1, _ = client.call_sync("service1", ([3, 4], {}), rpc_timeout=1.0)
            assert result1 == 7

            result2, _ = client.call_sync("service2", ([21], {}), rpc_timeout=1.0)
            assert result2 == 42

            result3, _ = client.call_sync("service3", (["hello"], {}), rpc_timeout=1.0)
            assert result3 == "HELLO"

        finally:
            unsub1()
            unsub2()
            unsub3()


@pytest.mark.parametrize("rpc_context, impl_name", testdata)
def test_concurrent_calls(rpc_context, impl_name: str) -> None:
    """Test making multiple concurrent RPC calls."""
    # Skip for SharedMemory - double-buffered architecture can't handle concurrent bursts
    # The channel only holds 2 frames, so 1000 rapid concurrent responses overwrite each other
    if impl_name == "shm":
        pytest.skip("SharedMemory uses double-buffering; can't handle 1000 concurrent responses")

    with rpc_context() as (server, client):
        # Serve a function that we'll call concurrently
        unsub = server.serve_rpc(add_function, "concurrent_add")

        try:
            # Make multiple concurrent calls using threads
            results = []
            threads = []

            def make_call(a, b) -> None:
                result, _ = client.call_sync("concurrent_add", ([a, b], {}), rpc_timeout=2.0)
                results.append(result)

            # Start 1000 concurrent calls
            for i in range(1000):
                t = threading.Thread(target=make_call, args=(i, i + 1))
                threads.append(t)
                t.start()

            # Wait for all threads to complete
            for t in threads:
                t.join(timeout=10.0)

            # Verify all calls succeeded
            assert len(results) == 1000
            # Results should be [1, 3, 5, 7, 9, 11, 13, 15, 17, 19] but may be in any order
            expected = [i + (i + 1) for i in range(1000)]
            assert sorted(results) == sorted(expected)

        finally:
            unsub()


if __name__ == "__main__":
    # Run tests for debugging
    pytest.main([__file__, "-v"])
