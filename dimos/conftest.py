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
import threading

import pytest


@pytest.fixture
def event_loop():
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


_session_threads = set()
_seen_threads = set()
_seen_threads_lock = threading.RLock()
_before_test_threads = {}  # Map test name to set of thread IDs before test

_skip_for = ["lcm", "heavy", "ros"]


@pytest.fixture(scope="module")
def dimos_cluster():
    from dimos.core import start

    dimos = start(4)
    try:
        yield dimos
    finally:
        dimos.stop()


@pytest.hookimpl()
def pytest_sessionfinish(session):
    """Track threads that exist at session start - these are not leaks."""

    yield

    # Check for session-level thread leaks at teardown
    final_threads = [
        t
        for t in threading.enumerate()
        if t.name != "MainThread" and t.ident not in _session_threads
    ]

    if final_threads:
        thread_info = [f"{t.name} (daemon={t.daemon})" for t in final_threads]
        pytest.fail(
            f"\n{len(final_threads)} thread(s) leaked during test session: {thread_info}\n"
            "Session-scoped fixtures must clean up all threads in their teardown."
        )


@pytest.fixture(autouse=True)
def monitor_threads(request):
    # Skip monitoring for tests marked with specified markers
    if any(request.node.get_closest_marker(marker) for marker in _skip_for):
        yield
        return

    # Capture threads before test runs
    test_name = request.node.nodeid
    with _seen_threads_lock:
        _before_test_threads[test_name] = {
            t.ident for t in threading.enumerate() if t.ident is not None
        }

    yield

    with _seen_threads_lock:
        before = _before_test_threads.get(test_name, set())
        current = {t.ident for t in threading.enumerate() if t.ident is not None}

        # New threads are ones that exist now but didn't exist before this test
        new_thread_ids = current - before

        if not new_thread_ids:
            return

        # Get the actual thread objects for new threads
        new_threads = [
            t for t in threading.enumerate() if t.ident in new_thread_ids and t.name != "MainThread"
        ]

        # Filter out expected persistent threads that are shared globally
        # These threads are intentionally left running and cleaned up on process exit
        expected_persistent_thread_prefixes = [
            "Dask-Offload",
            # HuggingFace safetensors conversion thread - no user cleanup API
            # https://github.com/huggingface/transformers/issues/29513
            "Thread-auto_conversion",
        ]
        new_threads = [
            t
            for t in new_threads
            if not any(t.name.startswith(prefix) for prefix in expected_persistent_thread_prefixes)
        ]

        # Filter out threads we've already seen (from previous tests)
        truly_new = [t for t in new_threads if t.ident not in _seen_threads]

        # Mark all new threads as seen
        for t in new_threads:
            if t.ident is not None:
                _seen_threads.add(t.ident)

        if not truly_new:
            return

        thread_names = [t.name for t in truly_new]

        pytest.fail(
            f"Non-closed threads created during this test. Thread names: {thread_names}. "
            "Please look at the first test that fails and fix that."
        )
