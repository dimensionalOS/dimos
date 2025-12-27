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
import atexit
import gc
import threading

import pytest


def _cleanup_open3d_tensors() -> None:
    """Clean up Open3D tensor resources before process exit.

    Open3D's MemoryManagerStatistic tracks tensor allocations and will
    force EXIT_FAILURE if any are unfreed at program end. This happens
    because Python's garbage collector doesn't always run before C++
    destructors during interpreter shutdown.

    This atexit handler clears functools caches that may hold references
    to Open3D objects, then runs garbage collection.
    """
    # Clear functools.cache on PointCloud2 methods that hold Open3D refs
    try:
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        for method_name in [
            "get_axis_aligned_bounding_box",
            "get_oriented_bounding_box",
            "get_bounding_box_dimensions",
        ]:
            method = getattr(PointCloud2, method_name, None)
            if method is not None and hasattr(method, "cache_clear"):
                method.cache_clear()
    except ImportError:
        pass

    gc.collect()
    gc.collect()  # Run twice to handle reference cycles


_atexit_registered = False


def pytest_configure(config: pytest.Config) -> None:
    """Register cleanup handler after Open3D modules may have been loaded.

    atexit handlers run in LIFO order. By registering our cleanup handler
    in pytest_configure (which runs after conftest imports), we ensure it
    runs AFTER any Open3D atexit handlers but BEFORE module unloading.
    """
    global _atexit_registered
    if not _atexit_registered:
        atexit.register(_cleanup_open3d_tensors)
        _atexit_registered = True


def pytest_unconfigure(config: pytest.Config) -> None:
    """Clean up resources before pytest exits.

    This hook runs before atexit handlers and helps ensure Open3D tensors
    are garbage collected before Open3D's memory checker runs.
    """
    _cleanup_open3d_tensors()


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
