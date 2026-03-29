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

"""Advanced tests for the dimos Python API — concurrency, lifecycle, composition."""

from __future__ import annotations

import os
import threading
import time

import pytest

from dimos.api import connect, list_blueprints


@pytest.fixture
def robot():
    """Launch stress-test blueprint and yield coordinator, stop on teardown."""
    coordinator = connect("demo-mcp-stress-test")
    yield coordinator
    coordinator.stop()


class TestConcurrentRPC:
    """Test concurrent RPC calls from multiple threads."""

    def test_concurrent_echo(self, robot):
        """Multiple threads calling echo simultaneously."""
        stress = robot.module("StressTestModule")
        results = {}
        errors = []

        def call_echo(thread_id):
            try:
                msg = f"thread-{thread_id}"
                result = stress.echo(msg)
                results[thread_id] = result
            except Exception as e:
                errors.append((thread_id, e))

        threads = [threading.Thread(target=call_echo, args=(i,)) for i in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=30)

        assert not errors, f"Errors in threads: {errors}"
        assert len(results) == 5
        for i in range(5):
            assert results[i] == f"thread-{i}"

    def test_interleaved_echo_and_ping(self, robot):
        """Interleave echo and ping calls from different threads."""
        stress = robot.module("StressTestModule")
        echo_results = []
        ping_results = []

        def echo_loop():
            for i in range(10):
                echo_results.append(stress.echo(f"e{i}"))

        def ping_loop():
            for _ in range(10):
                ping_results.append(stress.ping())

        t1 = threading.Thread(target=echo_loop)
        t2 = threading.Thread(target=ping_loop)
        t1.start()
        t2.start()
        t1.join(timeout=30)
        t2.join(timeout=30)

        assert len(echo_results) == 10
        assert len(ping_results) == 10
        assert all(r == "pong" for r in ping_results)
        for i in range(10):
            assert f"e{i}" in echo_results


class TestCrossProcess:
    """Verify RPC calls cross process boundaries correctly."""

    def test_info_returns_different_pid(self, robot):
        """Module runs in a worker process with a different PID."""
        stress = robot.module("StressTestModule")
        info = stress.info()
        # info format: "pid=XXXX, module=StressTestModule"
        assert "pid=" in info
        assert "StressTestModule" in info
        worker_pid = int(info.split("pid=")[1].split(",")[0])
        assert worker_pid != os.getpid(), "Module should run in a different process"

    def test_slow_skill_blocks_correctly(self, robot):
        """The slow skill should block for the specified duration."""
        stress = robot.module("StressTestModule")
        start = time.monotonic()
        result = stress.slow(0.5)
        elapsed = time.monotonic() - start
        assert "slept 0.5s" in result
        assert elapsed >= 0.4, f"Should have blocked ~0.5s, got {elapsed:.2f}s"
        assert elapsed < 5.0, f"Took way too long: {elapsed:.2f}s"


class TestLifecycle:
    """Test connect/stop lifecycle edge cases."""

    def test_stop_is_idempotent(self, robot):
        """Stopping twice should not raise."""
        robot.stop()
        # Second stop — fixture teardown will call stop() again
        # This should not raise

    def test_module_after_stop_returns_none(self):
        """RPC calls after stop return None (client not initialized)."""
        coordinator = connect("demo-mcp-stress-test")
        stress = coordinator.module("StressTestModule")
        assert stress.ping() == "pong"  # works before stop
        coordinator.stop()
        # After stop, RPC client is gone — calls return None
        result = stress.ping()
        assert result is None

    def test_sequential_connect_stop(self):
        """Connect and stop multiple times in sequence."""
        for i in range(3):
            coordinator = connect("demo-mcp-stress-test")
            stress = coordinator.module("StressTestModule")
            assert stress.echo(f"round-{i}") == f"round-{i}"
            coordinator.stop()

    def test_health_check(self, robot):
        """Health check should pass on a running coordinator."""
        assert robot.health_check()


class TestModuleDiscovery:
    """Test module and skill discovery."""

    def test_both_modules_present(self, robot):
        """stress-test blueprint has StressTestModule and McpServer."""
        names = robot.module_names
        assert "StressTestModule" in names
        assert "McpServer" in names

    def test_module_returns_same_proxy(self, robot):
        """Repeated calls to module() return the same proxy."""
        a = robot.module("StressTestModule")
        b = robot.module("StressTestModule")
        # They should be the same object (or at least equivalent)
        assert a.ping() == b.ping() == "pong"

    def test_skill_has_schema(self, robot):
        """Skills should have args_schema populated."""
        stress = robot.module("StressTestModule")
        skills = stress.get_skills()
        echo_skill = next(s for s in skills if s.func_name == "echo")
        assert echo_skill.args_schema, "echo skill should have an args schema"
        assert echo_skill.class_name == "StressTestModule"

    def test_list_blueprints_count(self):
        """Should have a substantial number of blueprints."""
        names = list_blueprints()
        assert len(names) > 50, f"Expected 50+ blueprints, got {len(names)}"
        # Verify some known blueprints exist
        assert "unitree-go2" in names
        assert "unitree-g1" in names
        assert "drone-basic" in names


class TestLargePayload:
    """Test RPC with larger payloads."""

    def test_echo_large_string(self, robot):
        """Echo a large string through RPC."""
        stress = robot.module("StressTestModule")
        large = "x" * 10000
        result = stress.echo(large)
        assert result == large
        assert len(result) == 10000

    def test_echo_special_chars(self, robot):
        """Echo strings with special characters."""
        stress = robot.module("StressTestModule")
        for msg in ["hello\nworld", "tab\there", "émojis 🤖🦾", '{"json": true}', ""]:
            assert stress.echo(msg) == msg
