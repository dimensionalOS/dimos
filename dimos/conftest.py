# Copyright 2025-2026 Dimensional Inc.
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
import os
import platform
import threading

from dotenv import load_dotenv
import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.protocol.service.lcmservice import autoconf

load_dotenv()


def _has_ros() -> bool:
    try:
        import rclpy  # noqa: F401

        return True
    except ImportError:
        return False


def _is_macos() -> bool:
    return platform.system() == "Darwin"


def pytest_configure(config):
    config.addinivalue_line("markers", "tool: dev tooling")
    config.addinivalue_line("markers", "slow: tests that are too slow for the fast loop")
    config.addinivalue_line("markers", "mujoco: tests which open mujoco")
    config.addinivalue_line("markers", "skipif_in_ci: skip when CI env var is set")
    config.addinivalue_line("markers", "skipif_no_openai: skip when OPENAI_API_KEY is not set")
    config.addinivalue_line("markers", "skipif_no_alibaba: skip when ALIBABA_API_KEY is not set")
    config.addinivalue_line("markers", "skipif_no_ros: skip when ROS dependencies are not present")
    config.addinivalue_line("markers", "skipif_macos_bug: skip known-buggy tests on macOS")
    config.addinivalue_line("markers", "skipif_macos: skip tests not intended to run on macOS")

    # Propagate coverage collection to subprocesses.
    if os.environ.get("_DIMOS_COV"):
        os.environ["COVERAGE_PROCESS_START"] = str(config.rootpath / "pyproject.toml")


@pytest.hookimpl()
def pytest_collection_modifyitems(config, items):
    _skipif_markers = {
        "skipif_in_ci": (bool(os.getenv("CI")), "Skipped in CI"),
        "skipif_no_openai": (not os.getenv("OPENAI_API_KEY"), "OPENAI_API_KEY not set"),
        "skipif_no_alibaba": (not os.getenv("ALIBABA_API_KEY"), "ALIBABA_API_KEY not set"),
        "skipif_no_ros": (not _has_ros(), "ROS dependencies are not present"),
        "skipif_macos_bug": (_is_macos(), "Some tests are buggy on Mac OS"),
        "skipif_macos": (_is_macos(), "Not intended to run on macOS"),
    }
    for marker_name, (condition, reason) in _skipif_markers.items():
        if condition:
            skip = pytest.mark.skip(reason=reason)
            for item in items:
                if item.get_closest_marker(marker_name):
                    item.add_marker(skip)


@pytest.fixture
def event_loop():
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


@pytest.fixture(scope="session", autouse=True)
def _autoconf(request):
    """Run autoconf() before all tests with capture suspended so people see `sudo` commands."""

    capman = request.config.pluginmanager.getplugin("capturemanager")
    capman.suspend_global_capture(in_=True)
    try:
        autoconf()
    finally:
        capman.resume_global_capture()


_session_threads = set()
_seen_threads = set()
_seen_threads_lock = threading.RLock()
_before_test_threads = {}  # Map test name to set of thread IDs before test


@pytest.fixture(scope="module")
def dimos_cluster():
    dimos = ModuleCoordinator()
    dimos.start()
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
