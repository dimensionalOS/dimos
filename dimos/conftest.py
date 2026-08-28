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

import atexit
from contextlib import suppress
import faulthandler
import hashlib
import os
import pathlib
import platform
import tempfile
import threading
import time
import uuid

# Tag every pytest descendant so a sidecar watchdog can sweep strays (dimsim, rerun, etc).
DIMOS_PYTEST_RUN_ID_ENV = "DIMOS_PYTEST_RUN_ID"
_worker = os.environ.get("PYTEST_XDIST_WORKER")
if not _worker:
    os.environ[DIMOS_PYTEST_RUN_ID_ENV] = f"pytest-{uuid.uuid4().hex[:16]}"

# Pin every pytest session to its own LCM bus and its own zenoh scouting group
# *before* any dimos module is imported (``LCMConfig`` captures
# ``LCM_DEFAULT_URL`` at import time), so
# messages from processes outside the session (a dev ``dimos`` daemon, a
# leaked DimSim bridge, a concurrent pytest run) can't leak into
# subscribe_all/pattern tests. It has to be an env var (not just a fixture)
# because subprocesses spawned by tests (``ModuleCoordinator`` workers, the
# DimSim Deno bridge) create their own LCM instances and inherit our env.
#
# Zenoh needs the same treatment for the same reason: its default discovery is
# loopback multicast, which every worker and every concurrent session on this
# machine shares. ``ZENOH_SCOUT_ADDR`` moves each onto its own group.
#
# Buckets are seeded with the session run id so concurrent sessions on one
# machine can't collide. xdist workers mix in the worker name, and also get
# a per-worker MCP port (``GlobalConfig`` captures ``MCP_PORT``) and state
# dir (``run_registry`` captures ``XDG_STATE_HOME``), which only collide
# between parallel workers. Exporting ``LCM_DEFAULT_URL`` yourself opts a
# single-worker session out of the isolation, deliberately joining it to an
# external bus; ``ZENOH_SCOUT_ADDR`` does the same for zenoh.


def _lcm_bucket(seed: str) -> int:
    return int.from_bytes(hashlib.blake2b(seed.encode(), digest_size=2).digest(), "big") % 5000


_run_id = os.environ[DIMOS_PYTEST_RUN_ID_ENV]
if _worker:
    _BUCKET = _lcm_bucket(f"{_run_id}:{_worker}")
    os.environ["LCM_DEFAULT_URL"] = f"udpm://239.255.76.67:{7700 + _BUCKET}?ttl=0"
    os.environ["ZENOH_SCOUT_ADDR"] = f"224.0.0.224:{17700 + _BUCKET}"
    os.environ["MCP_PORT"] = str(20000 + _BUCKET)
    os.environ["XDG_STATE_HOME"] = tempfile.mkdtemp(prefix=f"dimos-test-state-{_worker}-")
else:
    _BUCKET = _lcm_bucket(_run_id)
    os.environ.setdefault("LCM_DEFAULT_URL", f"udpm://239.255.76.67:{7700 + _BUCKET}?ttl=0")
    os.environ.setdefault("ZENOH_SCOUT_ADDR", f"224.0.0.224:{17700 + _BUCKET}")

# Raise the open-file limit. Each LCM transport opens at least one
# multicast socket; with pytest-xdist workers running many in parallel,
# the macOS default soft cap (~256) gets exhausted and tests fail with
# `OSError: [Errno 24] Too many open files`. The autoconf path normally
# bumps this via `MaxFileConfiguratorMacOS` (sudo launchctl), but we
# disable autoconf for tests, so do it directly here against the per-
# process limit (no sudo required up to the hard cap).
with suppress(ImportError, ValueError, OSError):
    import resource

    soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
    target = min(65536, hard)
    if soft < target:
        resource.setrlimit(resource.RLIMIT_NOFILE, (target, hard))

from dotenv import load_dotenv
import pytest
import tqdm

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.coordination.process_lifecycle import spawn_watchdog
from dimos.utils.testing.waiting import retry_until as _retry_until, wait_until as _wait_until

# The first tqdm bar constructed in the process spawns a TMonitor daemon thread that lives until
# interpreter exit. If that first bar happens inside a test, monitor_threads flags it as a leak. The
# monitor only re-tunes miniters for smooth interactive rendering, so disable it for tests.
tqdm.tqdm.monitor_interval = 0

load_dotenv()


def _has_ros() -> bool:
    try:
        import rclpy  # noqa: F401

        return True
    except ImportError:
        return False


def _is_macos() -> bool:
    return platform.system() == "Darwin"


def _has_turbojpeg() -> bool:
    try:
        from turbojpeg import TurboJPEG

        TurboJPEG()
        return True
    except Exception:
        return False


# A segfault in a C extension leaves NO kernel log line: faulthandler installs a
# SIGSEGV handler, and the kernel only prints "segfault at ..." for *unhandled*
# signals. The Python traceback is therefore the only evidence, and it has two
# gaps we close here.
#
# 1. pytest's builtin faulthandler plugin writes to stderr, which xdist shares
#    between workers with no attribution. A per-process file (printed by CI on
#    failure) says which worker crashed.
# 2. That plugin is disarmed at unconfigure. Crashes during interpreter
#    finalization print nothing — and our LCM handler threads are daemons that
#    outlive the session. The atexit re-arm covers that window.
#
# The file is deliberately never closed: closing it would leave faulthandler
# writing to a dead fd during finalization. Every process thus leaves a file
# behind, usually empty; nobody reads them unless something crashed.
_crash_log = None


def _arm_crash_dumps() -> None:
    global _crash_log
    if _crash_log is None:
        # RUNNER_TEMP is the per-job temp dir on CI runners; /tmp on the
        # self-hosted machines is shared between jobs and never cleaned.
        base = os.environ.get("DIMOS_CRASH_DIR") or os.path.join(
            os.environ.get("RUNNER_TEMP") or tempfile.gettempdir(), "pytest-crash"
        )
        worker = os.environ.get("PYTEST_XDIST_WORKER", "main")
        crash_path = pathlib.Path(base) / f"crash-{worker}-{os.getpid()}.log"
        try:
            crash_path.parent.mkdir(parents=True, exist_ok=True)
            _crash_log = crash_path.open("w")
        except OSError as exc:  # never let diagnostics break a test run
            print(f"crash diagnostics disabled ({base}): {exc}")
            return
        atexit.register(_arm_crash_dumps)
    faulthandler.enable(file=_crash_log, all_threads=True)


def pytest_sessionstart(session):
    # Runs strictly after every pytest_configure, which is the only ordering
    # guarantee strong enough to win the handler back from the builtin plugin
    # (it arms stderr during configure).
    _arm_crash_dumps()


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "self_hosted: tests that need the self-hosted runner (LFS, ROS, CUDA, etc.)",
    )
    config.addinivalue_line("markers", "mujoco: tests which open mujoco")
    config.addinivalue_line(
        "markers", "self_hosted_large: tests that need a high-memory self-hosted runner"
    )
    config.addinivalue_line(
        "markers",
        "skipif_in_ci: skip when CI is set (override: DIMOS_FULL_TEST_RUN=1)",
    )
    config.addinivalue_line(
        "markers",
        "web_browser: cockpit browser e2e (playwright chromium); runs in the CI web job",
    )
    config.addinivalue_line(
        "markers",
        "bake_e2e: dimos bake e2e (builds rust); runs in the CI rust job",
    )
    config.addinivalue_line("markers", "skipif_no_openai: skip when OPENAI_API_KEY is not set")
    config.addinivalue_line("markers", "skipif_no_alibaba: skip when ALIBABA_API_KEY is not set")
    config.addinivalue_line("markers", "skipif_no_ros: skip when ROS dependencies are not present")
    config.addinivalue_line(
        "markers",
        "skipif_no_turbojpeg: skip when native libturbojpeg is missing — "
        "except in CI, where it runs anyway so a missing dep fails loudly",
    )
    config.addinivalue_line("markers", "skipif_macos_bug: skip known-buggy tests on macOS")
    config.addinivalue_line("markers", "skipif_macos: skip tests not intended to run on macOS")
    config.addinivalue_line(
        "markers", "skipif_aarch64: skip tests not intended to run on aarch64 (Linux ARM)"
    )

    if config.pluginmanager.hasplugin("_cov"):
        os.environ["COVERAGE_PROCESS_START"] = str(config.rootpath / "pyproject.toml")

    # Only spawn on the controller, without doing it on xdist workers.
    if not hasattr(config, "workerinput"):
        spawn_watchdog(
            os.environ[DIMOS_PYTEST_RUN_ID_ENV],
            env_var=DIMOS_PYTEST_RUN_ID_ENV,
        )


@pytest.fixture(autouse=True)
def _restore_global_config():
    """Undo global_config mutations after every test.

    A build from a parsed config resets the singleton to the parse's full
    resolution. With a hermetic parse (environ={}) that reverts mcp_port to
    its schema default, and every later test on the worker then binds the
    port every other worker also defaults to.
    """
    from dimos.core.global_config import global_config

    snapshot = global_config.model_dump()
    yield
    global_config.update(**snapshot)


@pytest.fixture(scope="session")
def mcp_port() -> int:
    """The MCP server port pinned for this xdist worker (or the default)."""
    return int(os.environ.get("MCP_PORT", "9990"))


@pytest.fixture(scope="session")
def mcp_url(mcp_port: int) -> str:
    """The MCP server URL pinned for this xdist worker (or the default)."""
    return f"http://localhost:{mcp_port}/mcp"


@pytest.fixture(scope="session")
def lcm_url() -> str:
    """The LCM bus URL pinned for this xdist worker (or the default)."""
    return os.environ.get("LCM_DEFAULT_URL", "udpm://239.255.76.67:7667?ttl=0")


@pytest.fixture
def wait_until():
    """Poll a predicate until it's true or a timeout elapses. See dimos.utils.testing.waiting."""
    return _wait_until


@pytest.fixture
def retry_until():
    """Retry an action until a threading.Event fires. See dimos.utils.testing.waiting."""
    return _retry_until


@pytest.hookimpl(tryfirst=True)
def pytest_collection_modifyitems(config, items):
    _skipif_markers = {
        "skipif_in_ci": (
            bool(os.getenv("CI")) and not os.getenv("DIMOS_FULL_TEST_RUN"),
            "Skipped in CI",
        ),
        "skipif_no_openai": (not os.getenv("OPENAI_API_KEY"), "OPENAI_API_KEY not set"),
        "skipif_no_alibaba": (not os.getenv("ALIBABA_API_KEY"), "ALIBABA_API_KEY not set"),
        "skipif_no_ros": (not _has_ros(), "ROS dependencies are not present"),
        "skipif_no_turbojpeg": (
            not _has_turbojpeg() and not os.getenv("CI"),
            "native libturbojpeg unavailable",
        ),
        "skipif_macos_bug": (_is_macos(), "Some tests are buggy on Mac OS"),
        "skipif_macos": (_is_macos(), "Not intended to run on macOS"),
        "skipif_aarch64": (
            platform.machine() == "aarch64",
            "Not intended to run on aarch64 (Linux ARM)",
        ),
    }
    for marker_name, (condition, reason) in _skipif_markers.items():
        if condition:
            skip = pytest.mark.skip(reason=reason)
            for item in items:
                if item.get_closest_marker(marker_name):
                    item.add_marker(skip)


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

    # Check for session-level thread leaks at teardown. A stopped thread that
    # lingers in the registry (e.g. a zenoh pyo3-closure thread awaiting reaping)
    # is not a leak, so only count threads that are still running.
    final_threads = [
        t
        for t in threading.enumerate()
        if t.name != "MainThread" and t.ident not in _session_threads and t.is_alive()
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

    # Threads intentionally left running for the whole process and cleaned up on
    # exit, so they don't count as per-test leaks.
    expected_persistent_thread_prefixes = [
        "Dask-Offload",
        # HuggingFace safetensors conversion thread - no user cleanup API
        # https://github.com/huggingface/transformers/issues/29513
        "Thread-auto_conversion",
    ]
    # Zenoh callback threads belong to a session in the process-wide pool, which
    # outlives the test that first opened it on purpose -- sharing one session is
    # the point of the pool, and closing it per test would cut module-scoped
    # fixtures off from their transports mid-module. They go at interpreter exit.
    # The name is "Thread-<n> (pyo3-closure)", so this cannot be a prefix match.
    expected_persistent_thread_infix = "(pyo3-closure)"

    def live_new_threads():
        # Threads created during this test that are still running. A thread that
        # has already stopped is not a leak -- it's done, just not yet reaped
        # from threading's registry -- so we key on is_alive(), not presence.
        result = []
        for t in threading.enumerate():
            if t.ident is None or t.ident in before or t.name == "MainThread":
                continue
            if any(t.name.startswith(prefix) for prefix in expected_persistent_thread_prefixes):
                continue
            if expected_persistent_thread_infix in t.name:
                continue
            if t.is_alive():
                result.append(t)
        return result

    # Some C extensions tear their callback threads down asynchronously, so a
    # thread can stay alive briefly after the test cleaned up its owner (notably
    # zenoh's pyo3-closure threads, freed shortly after the session is closed).
    # A single snapshot races that teardown and flags a thread that is about to
    # exit. Give new threads a grace period to drain; only ones that stay alive
    # are real leaks (a genuinely leaked thread never exits).
    deadline = time.monotonic() + 5.0
    leaked = live_new_threads()
    while leaked and time.monotonic() < deadline:
        time.sleep(0.02)
        leaked = live_new_threads()

    if not leaked:
        return

    with _seen_threads_lock:
        # Report each leaked thread only once across the session.
        truly_new = [t for t in leaked if t.ident not in _seen_threads]
        for t in leaked:
            _seen_threads.add(t.ident)

    if not truly_new:
        return

    thread_names = [t.name for t in truly_new]

    pytest.fail(
        f"Non-closed threads created during this test. Thread names: {thread_names}. "
        "Please look at the first test that fails and fix that."
    )
