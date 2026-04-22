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

"""Exhaustive tests for dimos/utils/thread_utils.py

Covers: ThreadSafeVal, ModuleThread, AsyncModuleThread, ModuleProcess, safe_thread_map.
Focuses on deadlocks, race conditions, idempotency, and edge cases under load.
"""

from __future__ import annotations

import asyncio
import os
import pickle
import sys
import threading
import time
from unittest import mock

import pytest
from reactivex.disposable import CompositeDisposable

from dimos.utils.thread_utils import (
    AsyncModuleThread,
    ModuleProcess,
    ModuleThread,
    ThreadSafeVal,
    safe_thread_map,
)

# Helpers: fake ModuleBase for testing ModuleThread / AsyncModuleThread / ModuleProcess


class FakeModule:
    """Minimal stand-in for ModuleBase — just needs _disposables."""

    def __init__(self) -> None:
        self._disposables = CompositeDisposable()

    def dispose(self) -> None:
        self._disposables.dispose()


# ThreadSafeVal Tests


class TestThreadSafeVal:
    def test_basic_get_set(self) -> None:
        v = ThreadSafeVal(42)
        assert v.get() == 42
        v.set(99)
        assert v.get() == 99

    def test_context_manager_returns_value(self) -> None:
        v = ThreadSafeVal("hello")
        with v as val:
            assert val == "hello"

    def test_set_inside_context_manager_no_deadlock(self) -> None:
        """The critical test: set() inside a with block must NOT deadlock.

        This was a confirmed bug when using threading.Lock (non-reentrant).
        Fixed by using threading.RLock.
        """
        v = ThreadSafeVal(0)
        result = threading.Event()

        def do_it() -> None:
            with v as val:
                v.set(val + 1)
            result.set()

        t = threading.Thread(target=do_it)
        t.start()
        t.join(timeout=2)
        assert result.is_set(), "Deadlocked! set() inside with block hung"
        assert v.get() == 1

    def test_get_inside_context_manager_no_deadlock(self) -> None:
        v = ThreadSafeVal(10)
        result = threading.Event()

        def do_it() -> None:
            with v:
                _ = v.get()
            result.set()

        t = threading.Thread(target=do_it)
        t.start()
        t.join(timeout=2)
        assert result.is_set(), "Deadlocked! get() inside with block hung"

    def test_context_manager_blocks_other_threads(self) -> None:
        """While one thread holds the lock via `with`, others should block on set()."""
        v = ThreadSafeVal(0)
        gate = threading.Event()
        other_started = threading.Event()
        other_finished = threading.Event()

        def holder() -> None:
            with v:
                gate.wait(timeout=5)  # hold the lock until signaled

        def setter() -> None:
            other_started.set()
            v.set(42)  # should block until holder releases
            other_finished.set()

        t1 = threading.Thread(target=holder)
        t2 = threading.Thread(target=setter)
        t1.start()
        time.sleep(0.05)  # let holder acquire lock
        t2.start()
        other_started.wait(timeout=2)
        time.sleep(0.1)
        # setter should be blocked
        assert not other_finished.is_set(), "set() did not block while lock was held"
        gate.set()  # release holder
        t1.join(timeout=2)
        t2.join(timeout=2)
        assert other_finished.is_set()
        assert v.get() == 42

    def test_concurrent_increments(self) -> None:
        """Many threads doing atomic read-modify-write should not lose updates."""
        v = ThreadSafeVal(0)
        n_threads = 50
        n_increments = 100

        def incrementer() -> None:
            for _ in range(n_increments):
                with v as val:
                    v.set(val + 1)

        threads = [threading.Thread(target=incrementer) for _ in range(n_threads)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10)
        assert v.get() == n_threads * n_increments

    def test_concurrent_increments_stress(self) -> None:
        """Run the concurrent increment test multiple times to catch races."""
        for _ in range(10):
            self.test_concurrent_increments()

    def test_pickle_roundtrip(self) -> None:
        v = ThreadSafeVal({"key": [1, 2, 3]})
        data = pickle.dumps(v)
        v2 = pickle.loads(data)
        assert v2.get() == {"key": [1, 2, 3]}
        # Verify the new instance has a working lock
        with v2 as val:
            v2.set({**val, "new": True})
        assert v2.get()["new"] is True

    def test_repr(self) -> None:
        v = ThreadSafeVal("test")
        assert repr(v) == "ThreadSafeVal('test')"

    def test_dict_type(self) -> None:
        v = ThreadSafeVal({"running": False, "count": 0})
        with v as s:
            v.set({**s, "running": True})
        assert v.get() == {"running": True, "count": 0}

    def test_string_literal_type(self) -> None:
        """Simulates the ModState pattern from module.py."""
        v = ThreadSafeVal("init")
        with v as state:
            if state == "init":
                v.set("started")
        assert v.get() == "started"

        with v as state:
            if state == "stopped":
                pass  # no-op
            else:
                v.set("stopped")
        assert v.get() == "stopped"

    def test_nested_with_no_deadlock(self) -> None:
        """RLock should allow the same thread to nest with blocks."""
        v = ThreadSafeVal(0)
        result = threading.Event()

        def do_it() -> None:
            with v:
                with v as val2:
                    v.set(val2 + 1)
            result.set()

        t = threading.Thread(target=do_it)
        t.start()
        t.join(timeout=2)
        assert result.is_set(), "Nested with blocks deadlocked!"


# ModuleThread Tests


class TestModuleThread:
    def test_basic_lifecycle(self) -> None:
        mod = FakeModule()
        ran = threading.Event()

        def target() -> None:
            ran.set()

        mt = ModuleThread(module=mod, target=target, name="test-basic")
        mt.start()
        ran.wait(timeout=2)
        assert ran.is_set()
        mt.stop()
        assert not mt.is_alive

    def test_auto_start(self) -> None:
        mod = FakeModule()
        started = threading.Event()
        mt = ModuleThread(module=mod, target=started.set, name="test-autostart")
        mt.start()
        started.wait(timeout=2)
        assert started.is_set()
        mt.stop()

    def test_deferred_start(self) -> None:
        mod = FakeModule()
        started = threading.Event()
        mt = ModuleThread(module=mod, target=started.set, name="test-deferred")
        time.sleep(0.1)
        assert not started.is_set()
        mt.start()
        started.wait(timeout=2)
        assert started.is_set()
        mt.stop()

    def test_stopping_property(self) -> None:
        mod = FakeModule()
        saw_stopping = threading.Event()
        holder: list[ModuleThread] = []

        def target() -> None:
            while holder[0].status.get() == "running":
                time.sleep(0.01)
            saw_stopping.set()

        mt = ModuleThread(module=mod, target=target, name="test-stopping")
        holder.append(mt)
        mt.start()
        time.sleep(0.05)
        mt.stop()
        saw_stopping.wait(timeout=2)
        assert saw_stopping.is_set()

    def test_stop_idempotent(self) -> None:
        mod = FakeModule()
        mt = ModuleThread(module=mod, target=lambda: time.sleep(0.01), name="test-idem")
        mt.start()
        time.sleep(0.05)
        mt.stop()
        mt.stop()  # second call should not raise
        mt.stop()  # third call should not raise

    def test_stop_from_managed_thread_no_deadlock(self) -> None:
        """The thread calling stop() on itself should not deadlock."""
        mod = FakeModule()
        result = threading.Event()
        holder: list[ModuleThread] = []

        def target() -> None:
            holder[0].stop()  # stop ourselves — should not deadlock
            result.set()

        mt = ModuleThread(module=mod, target=target, name="test-self-stop")
        holder.append(mt)
        mt.start()
        result.wait(timeout=3)
        assert result.is_set(), "Deadlocked when thread called stop() on itself"

    def test_dispose_stops_thread(self) -> None:
        """Module dispose should stop the thread via the registered Disposable."""
        mod = FakeModule()
        running = threading.Event()
        holder: list[ModuleThread] = []

        def target() -> None:
            running.set()
            while holder[0].status.get() == "running":
                time.sleep(0.01)

        mt = ModuleThread(module=mod, target=target, name="test-dispose")
        holder.append(mt)
        mt.start()
        running.wait(timeout=2)
        mod.dispose()
        time.sleep(0.1)
        assert not mt.is_alive

    def test_concurrent_stop_calls(self) -> None:
        """Multiple threads calling stop() concurrently should not crash."""
        mod = FakeModule()
        holder: list[ModuleThread] = []

        def target() -> None:
            while holder[0].status.get() == "running":
                time.sleep(0.01)

        mt = ModuleThread(module=mod, target=target, name="test-concurrent-stop")
        holder.append(mt)
        mt.start()
        time.sleep(0.05)

        errors = []

        def stop_it() -> None:
            try:
                mt.stop()
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=stop_it) for _ in range(20)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5)
        assert not errors, f"Concurrent stop() raised: {errors}"

    def test_close_timeout_respected(self) -> None:
        """If the thread ignores the stop signal, stop() should return after close_timeout."""
        mod = FakeModule()
        cancel = threading.Event()

        def stubborn_target() -> None:
            cancel.wait(10)  # blocks but can be released for cleanup

        mt = ModuleThread(
            module=mod, target=stubborn_target, name="test-timeout", close_timeout=0.2
        )
        mt.start()
        start = time.monotonic()
        mt.stop()
        elapsed = time.monotonic() - start
        assert elapsed < 1.0, f"stop() took {elapsed}s, expected ~0.2s"
        # Release the thread so it doesn't leak
        cancel.set()
        mt._thread.join(timeout=1.0)

    def test_stop_concurrent_with_dispose(self) -> None:
        """Calling stop() and dispose() concurrently should not crash."""
        for _ in range(20):
            mod = FakeModule()
            holder: list[ModuleThread] = []

            def target(h: list[ModuleThread] = holder) -> None:
                while h[0].status.get() == "running":
                    time.sleep(0.001)

            mt = ModuleThread(module=mod, target=target, name="test-stop-dispose")
            holder.append(mt)
            mt.start()
            time.sleep(0.02)
            # Race: stop and dispose from different threads
            t1 = threading.Thread(target=mt.stop)
            t2 = threading.Thread(target=mod.dispose)
            t1.start()
            t2.start()
            t1.join(timeout=3)
            t2.join(timeout=3)


# AsyncModuleThread Tests


class TestAsyncModuleThread:
    def test_creates_loop_and_thread(self) -> None:
        mod = FakeModule()
        amt = AsyncModuleThread(module=mod)
        amt.start()
        assert amt.loop is not None
        assert amt.loop.is_running()
        assert amt.is_alive
        amt.stop()
        assert not amt.is_alive

    def test_stop_idempotent(self) -> None:
        mod = FakeModule()
        amt = AsyncModuleThread(module=mod)
        amt.start()
        amt.stop()
        amt.stop()  # should not raise
        amt.stop()

    def test_dispose_stops_loop(self) -> None:
        mod = FakeModule()
        amt = AsyncModuleThread(module=mod)
        amt.start()
        assert amt.is_alive
        mod.dispose()
        time.sleep(0.1)
        assert not amt.is_alive

    def test_can_schedule_coroutine(self) -> None:
        mod = FakeModule()
        amt = AsyncModuleThread(module=mod)
        amt.start()
        result = []

        async def coro() -> None:
            result.append(42)

        future = asyncio.run_coroutine_threadsafe(coro(), amt.loop)
        future.result(timeout=2)
        assert result == [42]
        amt.stop()

    def test_stop_with_pending_work(self) -> None:
        """Stop should succeed even with long-running tasks on the loop."""
        mod = FakeModule()
        amt = AsyncModuleThread(module=mod)
        amt.start()
        started = threading.Event()

        async def slow_coro() -> None:
            started.set()
            await asyncio.sleep(10)

        asyncio.run_coroutine_threadsafe(slow_coro(), amt.loop)
        started.wait(timeout=2)
        # stop() should not hang waiting for the coroutine
        start = time.monotonic()
        amt.stop()
        elapsed = time.monotonic() - start
        assert elapsed < 5.0, f"stop() hung for {elapsed}s with pending coroutine"

    def test_concurrent_stop(self) -> None:
        mod = FakeModule()
        amt = AsyncModuleThread(module=mod)
        amt.start()
        errors = []

        def stop_it() -> None:
            try:
                amt.stop()
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=stop_it) for _ in range(20)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=5)
        assert not errors


# ModuleProcess Tests


# Helper: path to a python that sleeps or echoes
PYTHON = sys.executable


class TestModuleProcess:
    def test_basic_lifecycle(self) -> None:
        mod = FakeModule()
        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "import time; time.sleep(30)"],
            shutdown_timeout=2.0,
        )
        mp.start()
        assert mp.is_alive
        assert mp.pid is not None
        mp.stop()
        assert not mp.is_alive
        assert mp.pid is None

    def test_stop_idempotent(self) -> None:
        mod = FakeModule()
        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "import time; time.sleep(30)"],
            shutdown_timeout=1.0,
        )
        mp.start()
        mp.stop()
        mp.stop()  # should not raise
        mp.stop()

    def test_dispose_stops_process(self) -> None:
        mod = FakeModule()
        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "import time; time.sleep(30)"],
            shutdown_timeout=2.0,
        )
        mp.start()
        mod.dispose()
        time.sleep(0.5)
        assert not mp.is_alive

    def test_on_exit_fires_on_natural_exit(self) -> None:
        """on_exit should fire when the process exits on its own."""
        mod = FakeModule()
        exit_called = threading.Event()

        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "print('done')"],
            on_exit=exit_called.set,
        )
        mp.start()
        exit_called.wait(timeout=5)
        assert exit_called.is_set(), "on_exit was not called after natural process exit"

    def test_on_exit_fires_on_crash(self) -> None:
        mod = FakeModule()
        exit_called = threading.Event()

        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "import sys; sys.exit(1)"],
            on_exit=exit_called.set,
        )
        mp.start()
        exit_called.wait(timeout=5)
        assert exit_called.is_set(), "on_exit was not called after process crash"

    def test_on_exit_not_fired_on_stop(self) -> None:
        """on_exit should NOT fire when stop() kills the process."""
        mod = FakeModule()
        exit_called = threading.Event()

        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "import time; time.sleep(30)"],
            on_exit=exit_called.set,
            shutdown_timeout=2.0,
        )
        mp.start()
        time.sleep(0.2)  # let watchdog start
        mp.stop()
        time.sleep(1.0)  # give watchdog time to potentially fire
        assert not exit_called.is_set(), "on_exit fired after intentional stop()"

    def test_stdout_logged(self) -> None:
        mod = FakeModule()
        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "print('hello from subprocess')"],
        )
        mp.start()
        time.sleep(1.0)  # let output be read
        mp.stop()

    def test_stderr_logged(self) -> None:
        mod = FakeModule()
        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "import sys; sys.stderr.write('error msg\\n')"],
        )
        mp.start()
        time.sleep(1.0)
        mp.stop()

    def test_log_json_mode(self) -> None:
        mod = FakeModule()
        mp = ModuleProcess(
            module=mod,
            args=[
                PYTHON,
                "-c",
                """import json; print(json.dumps({"event": "test", "key": "val"}))""",
            ],
            log_json=True,
        )
        mp.start()
        time.sleep(1.0)
        mp.stop()

    def test_log_json_malformed(self) -> None:
        mod = FakeModule()
        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "print('not json')"],
            log_json=True,
        )
        mp.start()
        time.sleep(1.0)
        mp.stop()

    def test_stop_process_that_ignores_sigterm(self) -> None:
        """Process that ignores SIGTERM should be killed with SIGKILL."""
        mod = FakeModule()
        mp = ModuleProcess(
            module=mod,
            args=[
                PYTHON,
                "-c",
                "import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)",
            ],
            shutdown_timeout=0.5,
            kill_timeout=2.0,
        )
        mp.start()
        time.sleep(0.2)
        start = time.monotonic()
        mp.stop()
        elapsed = time.monotonic() - start
        assert not mp.is_alive
        # Should take roughly shutdown_timeout (0.5) + a bit for SIGKILL
        assert elapsed < 5.0

    def test_stop_already_dead_process(self) -> None:
        """stop() on a process that already exited should not raise."""
        mod = FakeModule()
        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "pass"],  # exits immediately
        )
        mp.start()
        time.sleep(1.0)  # let it die
        mp.stop()  # should not raise

    def test_concurrent_stop(self) -> None:
        mod = FakeModule()
        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "import time; time.sleep(30)"],
            shutdown_timeout=2.0,
        )
        mp.start()
        errors = []

        def stop_it() -> None:
            try:
                mp.stop()
            except Exception as e:
                errors.append(e)

        threads = [threading.Thread(target=stop_it) for _ in range(20)]
        for t in threads:
            t.start()
        for t in threads:
            t.join(timeout=10)
        assert not errors, f"Concurrent stop() raised: {errors}"

    def test_on_exit_calls_module_stop_no_deadlock(self) -> None:
        """Simulate the real pattern: on_exit=module.stop, which disposes the
        ModuleProcess, which tries to stop its watchdog from inside the watchdog.
        Must not deadlock.
        """
        mod = FakeModule()
        stop_called = threading.Event()

        def fake_module_stop() -> None:
            """Simulates module.stop() -> _stop() -> dispose()"""
            mod.dispose()
            stop_called.set()

        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "pass"],  # exits immediately
            on_exit=fake_module_stop,
        )
        mp.start()
        stop_called.wait(timeout=5)
        assert stop_called.is_set(), "Deadlocked! on_exit -> dispose -> stop chain hung"

    def test_on_exit_calls_module_stop_no_deadlock_stress(self) -> None:
        """Run the deadlock test multiple times under load."""
        for _i in range(10):
            self.test_on_exit_calls_module_stop_no_deadlock()

    def test_deferred_start(self) -> None:
        mod = FakeModule()
        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "import time; time.sleep(30)"],
        )
        assert not mp.is_alive
        mp.start()
        assert mp.is_alive
        mp.stop()

    def test_env_passed(self) -> None:
        mod = FakeModule()
        exit_called = threading.Event()

        mp = ModuleProcess(
            module=mod,
            args=[
                PYTHON,
                "-c",
                "import os, sys; sys.exit(0 if os.environ.get('MY_VAR') == '42' else 1)",
            ],
            env={**os.environ, "MY_VAR": "42"},
            on_exit=exit_called.set,
        )
        mp.start()
        exit_called.wait(timeout=5)
        # Process should have exited with 0 (our on_exit fires for all unmanaged exits)
        assert exit_called.is_set()

    def test_cwd_passed(self) -> None:
        mod = FakeModule()
        mp = ModuleProcess(
            module=mod,
            args=[PYTHON, "-c", "import os; print(os.getcwd())"],
            cwd="/tmp",
        )
        mp.start()
        time.sleep(1.0)
        mp.stop()


# safe_thread_map Tests


class TestSafeThreadMap:
    def test_empty_input(self) -> None:
        assert safe_thread_map([], lambda x: x) == []

    def test_all_succeed(self) -> None:
        result = safe_thread_map([1, 2, 3], lambda x: x * 2)
        assert result == [2, 4, 6]

    def test_preserves_order(self) -> None:
        def slow(x: int) -> int:
            time.sleep(0.01 * (10 - x))
            return x

        result = safe_thread_map(list(range(10)), slow)
        assert result == list(range(10))

    def test_all_fail_raises_exception_group(self) -> None:
        def fail(x: int) -> int:
            raise ValueError(f"fail-{x}")

        with pytest.raises(ExceptionGroup) as exc_info:
            safe_thread_map([1, 2, 3], fail)
        assert len(exc_info.value.exceptions) == 3

    def test_partial_failure(self) -> None:
        def maybe_fail(x: int) -> int:
            if x == 2:
                raise ValueError("fail")
            return x

        with pytest.raises(ExceptionGroup) as exc_info:
            safe_thread_map([1, 2, 3], maybe_fail)
        assert len(exc_info.value.exceptions) == 1

    def test_on_errors_callback(self) -> None:
        def fail(x: int) -> int:
            if x == 2:
                raise ValueError("boom")
            return x * 10

        cleanup_called = False

        def on_errors(outcomes, successes, errors):
            nonlocal cleanup_called
            cleanup_called = True
            assert len(errors) == 1
            assert len(successes) == 2
            return successes  # return successful results

        result = safe_thread_map([1, 2, 3], fail, on_errors)
        assert cleanup_called
        assert sorted(result) == [10, 30]

    def test_on_errors_can_raise(self) -> None:
        def fail(x: int) -> int:
            raise ValueError("boom")

        def on_errors(outcomes, successes, errors):
            raise RuntimeError("custom error")

        with pytest.raises(RuntimeError, match="custom error"):
            safe_thread_map([1], fail, on_errors)

    def test_waits_for_all_before_raising(self) -> None:
        """Even if one fails fast, all others should complete."""
        completed = []

        def work(x: int) -> int:
            if x == 0:
                raise ValueError("fast fail")
            time.sleep(0.2)
            completed.append(x)
            return x

        with pytest.raises(ExceptionGroup):
            safe_thread_map([0, 1, 2, 3], work)
        # All non-failing items should have completed
        assert sorted(completed) == [1, 2, 3]


# Integration: ModuleProcess on_exit -> dispose chain (the CI bug scenario)


class TestModuleProcessDisposeChain:
    """Tests the exact pattern that caused the CI bug:
    process exits -> watchdog fires on_exit -> module.stop() -> dispose ->
    ModuleProcess.stop() -> tries to stop watchdog from inside watchdog thread.
    """

    @staticmethod
    def _make_fake_stop(mod: FakeModule, done: threading.Event) -> Callable:
        def fake_stop() -> None:
            mod.dispose()
            done.set()

        return fake_stop

    def test_chain_no_deadlock_fast_exit(self) -> None:
        """Process exits immediately."""
        for _ in range(20):
            mod = FakeModule()
            done = threading.Event()
            mp = ModuleProcess(
                module=mod,
                args=[PYTHON, "-c", "pass"],
                on_exit=self._make_fake_stop(mod, done),
            )
            mp.start()
            assert done.wait(timeout=5), "Deadlock in dispose chain (fast exit)"

    def test_chain_no_deadlock_slow_exit(self) -> None:
        """Process runs briefly then exits."""
        for _ in range(10):
            mod = FakeModule()
            done = threading.Event()
            mp = ModuleProcess(
                module=mod,
                args=[PYTHON, "-c", "import time; time.sleep(0.1)"],
                on_exit=self._make_fake_stop(mod, done),
            )
            mp.start()
            assert done.wait(timeout=5), "Deadlock in dispose chain (slow exit)"

    def test_chain_concurrent_with_external_stop(self) -> None:
        """Process exits naturally while external code calls stop()."""
        for _ in range(20):
            mod = FakeModule()
            done = threading.Event()
            mp = ModuleProcess(
                module=mod,
                args=[PYTHON, "-c", "import time; time.sleep(0.05)"],
                on_exit=self._make_fake_stop(mod, done),
                shutdown_timeout=1.0,
            )
            mp.start()
            # Race: the process might exit naturally or we might stop it
            time.sleep(0.03)
            mp.stop()
            # Either way, should not deadlock
            time.sleep(1.0)

    def test_dispose_with_artificial_delay(self) -> None:
        """Add artificial delay near cleanup to simulate heavy CPU load."""
        original_stop = ModuleThread.stop

        def slow_stop(self_mt: ModuleThread) -> None:
            time.sleep(0.05)  # simulate load
            original_stop(self_mt)

        for _ in range(10):
            mod = FakeModule()
            done = threading.Event()
            with mock.patch.object(ModuleThread, "stop", slow_stop):
                mp = ModuleProcess(
                    module=mod,
                    args=[PYTHON, "-c", "pass"],
                    on_exit=self._make_fake_stop(mod, done),
                )
                mp.start()
                assert done.wait(timeout=10), "Deadlock with slow ModuleThread.stop()"


from dimos.utils.typing_utils import ExceptionGroup
