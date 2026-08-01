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

import os
import signal
import sys
import time
from typing import TYPE_CHECKING

import psutil
import pytest

from dimos.core.coordination.python_worker import PythonWorker, reset_forkserver_context
from dimos.core.coordination.worker_launcher import CommandWorkerLauncher, VenvWorkerLauncher
from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.core.runtime_environment import PythonProjectLaunchMaterial
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Vector3 import Vector3

if TYPE_CHECKING:
    from dimos.core.resource_monitor.stats import WorkerStats


class SimpleModule(Module):
    output: Out[Vector3]
    input: In[Vector3]

    counter: int = 0

    @rpc
    def start(self) -> None:
        pass

    @rpc
    def increment(self) -> int:
        self.counter += 1
        return self.counter

    @rpc
    def get_counter(self) -> int:
        return self.counter


class AnotherModule(Module):
    value: int = 100

    @rpc
    def start(self) -> None:
        pass

    @rpc
    def add(self, n: int) -> int:
        self.value += n
        return self.value

    @rpc
    def get_value(self) -> int:
        return self.value


class ThirdModule(Module):
    multiplier: int = 1

    @rpc
    def start(self) -> None:
        pass

    @rpc
    def multiply(self, n: int) -> int:
        self.multiplier *= n
        return self.multiplier

    @rpc
    def get_multiplier(self) -> int:
        return self.multiplier


class HeavyModule(Module):
    dedicated_worker = True

    @rpc
    def start(self) -> None:
        pass


class NoisyModule(Module):
    @rpc
    def start(self) -> None:
        print("stdout noise from worker")
        print("stderr noise from worker", file=sys.stderr)

    @rpc
    def ping(self) -> str:
        print("stdout rpc noise from worker")
        print("stderr rpc noise from worker", file=sys.stderr)
        return "pong"


class AnotherHeavyModule(Module):
    dedicated_worker = True

    @rpc
    def start(self) -> None:
        pass


@pytest.fixture
def create_worker_manager():
    manager = None

    def _create(n_workers):
        nonlocal manager
        g = GlobalConfig(n_workers=n_workers)
        manager = WorkerManagerPython(g=g)
        manager.start()
        return manager

    yield _create

    if manager is not None:
        manager.stop()


@pytest.mark.skipif_macos_bug
def test_venv_worker_launch_deploy_rpc_shutdown() -> None:
    manager = WorkerManagerPython(
        g=GlobalConfig(n_workers=1),
        worker_launcher=VenvWorkerLauncher(sys.executable, startup_timeout=5.0),
    )
    try:
        module = manager.deploy(SimpleModule, global_config, {})
        module.start()
        assert module.increment() == 1
        assert module.increment() == 2
        assert module.get_counter() == 2
        module.stop()
    finally:
        manager.stop()


def test_venv_worker_missing_python_executable() -> None:
    worker = PythonWorker(launcher=VenvWorkerLauncher("/does/not/exist/python"))

    with pytest.raises(FileNotFoundError, match="Python executable"):
        worker.start_process()


def _write_fake_uv(path, python_executable: str = sys.executable) -> None:
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import os, subprocess, sys\n"
        "assert sys.argv[1:4] == ['run', '--no-sync', 'python']\n"
        "assert os.environ.get('DIMOS_TEST_SENTINEL') == 'kept'\n"
        "assert os.getcwd() == os.environ.get('DIMOS_EXPECTED_CWD')\n"
        f"raise SystemExit(subprocess.call([{python_executable!r}, *sys.argv[4:]]))\n"
    )
    path.chmod(0o755)


def _write_fake_pixi(path) -> None:
    path.write_text(
        "#!/usr/bin/env python3\n"
        "import os, subprocess, sys\n"
        "assert sys.argv[1:3] == ['run', 'uv']\n"
        "uv = os.path.join(os.path.dirname(__file__), 'uv')\n"
        "raise SystemExit(subprocess.call([uv, *sys.argv[3:]]))\n"
    )
    path.chmod(0o755)


@pytest.mark.skipif_macos_bug
def test_command_worker_launch_deploy_rpc_shutdown_with_fake_uv(tmp_path, monkeypatch) -> None:
    _write_fake_uv(tmp_path / "uv")
    monkeypatch.setenv("PATH", f"{tmp_path}:{os.environ.get('PATH', '')}")
    monkeypatch.setenv("DIMOS_RUN_SURVIVES", "1")
    material = PythonProjectLaunchMaterial(
        argv_prefix=["uv", "run", "--no-sync", "python"],
        cwd=tmp_path,
        env={"DIMOS_TEST_SENTINEL": "kept", "DIMOS_EXPECTED_CWD": str(tmp_path)},
    )
    manager = WorkerManagerPython(
        g=GlobalConfig(n_workers=1),
        worker_launcher=CommandWorkerLauncher(material, startup_timeout=5.0),
    )
    try:
        module = manager.deploy(SimpleModule, global_config, {})
        module.start()
        assert module.increment() == 1
        module.stop()
    finally:
        manager.stop()


@pytest.mark.skipif_macos_bug
def test_command_worker_launch_deploy_rpc_shutdown_with_fake_pixi(tmp_path, monkeypatch) -> None:
    _write_fake_uv(tmp_path / "uv")
    _write_fake_pixi(tmp_path / "pixi")
    monkeypatch.setenv("PATH", f"{tmp_path}:{os.environ.get('PATH', '')}")
    material = PythonProjectLaunchMaterial(
        argv_prefix=["pixi", "run", "uv", "run", "--no-sync", "python"],
        cwd=tmp_path,
        env={"DIMOS_TEST_SENTINEL": "kept", "DIMOS_EXPECTED_CWD": str(tmp_path)},
    )
    manager = WorkerManagerPython(
        g=GlobalConfig(n_workers=1),
        worker_launcher=CommandWorkerLauncher(material, startup_timeout=5.0),
    )
    try:
        module = manager.deploy(SimpleModule, global_config, {})
        assert module.increment() == 1
        module.stop()
    finally:
        manager.stop()


@pytest.mark.skipif_macos_bug
def test_command_worker_timeout_terminates_wrapper_child(tmp_path) -> None:
    marker = tmp_path / "orphan-child-marker"
    wrapper = tmp_path / "wrapper"
    wrapper.write_text(
        "#!/usr/bin/env python3\n"
        "import subprocess, sys, time\n"
        f"subprocess.Popen([sys.executable, '-c', \"import pathlib, time; pathlib.Path({str(marker)!r}).touch(); time.sleep(30)\"])\n"
        "time.sleep(30)\n"
    )
    wrapper.chmod(0o755)
    material = PythonProjectLaunchMaterial(argv_prefix=[str(wrapper)], cwd=tmp_path)
    worker = PythonWorker(launcher=CommandWorkerLauncher(material, startup_timeout=0.2))

    with pytest.raises(TimeoutError):
        worker.start_process()

    deadline = time.monotonic() + 3.0
    while not marker.exists() and time.monotonic() < deadline:
        time.sleep(0.05)
    time.sleep(0.2)
    assert not any(
        str(marker) in " ".join(proc.info.get("cmdline") or [])
        for proc in psutil.process_iter(["cmdline"])
    )


@pytest.mark.skipif_macos_bug
def test_command_worker_exit_before_connect_kills_sigterm_ignoring_child(tmp_path) -> None:
    marker = tmp_path / "sigterm-ignoring-child-marker"
    wrapper = tmp_path / "wrapper"
    child_script = (
        "import pathlib, signal, time; "
        f"pathlib.Path({str(marker)!r}).touch(); "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        "time.sleep(30)"
    )
    wrapper.write_text(
        "#!/usr/bin/env python3\n"
        "import subprocess, sys, time\n"
        f"subprocess.Popen([sys.executable, '-c', {child_script!r}])\n"
        "while not "
        f"__import__('pathlib').Path({str(marker)!r}).exists():\n"
        "    time.sleep(0.01)\n"
        "raise SystemExit(42)\n"
    )
    wrapper.chmod(0o755)
    material = PythonProjectLaunchMaterial(argv_prefix=[str(wrapper)], cwd=tmp_path)
    worker = PythonWorker(launcher=CommandWorkerLauncher(material, startup_timeout=2.0))

    with pytest.raises(RuntimeError, match="exit code 42"):
        worker.start_process()

    assert marker.exists()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if not any(
            str(marker) in " ".join(proc.info.get("cmdline") or [])
            for proc in psutil.process_iter(["cmdline"])
        ):
            break
        time.sleep(0.05)
    assert not any(
        str(marker) in " ".join(proc.info.get("cmdline") or [])
        for proc in psutil.process_iter(["cmdline"])
    )


@pytest.mark.skipif_macos_bug
def test_command_worker_shutdown_fallback_kills_stopped_process_group(
    tmp_path, monkeypatch
) -> None:
    _write_fake_uv(tmp_path / "uv")
    monkeypatch.setenv("PATH", f"{tmp_path}:{os.environ.get('PATH', '')}")
    material = PythonProjectLaunchMaterial(
        argv_prefix=["uv", "run", "--no-sync", "python"],
        cwd=tmp_path,
        env={"DIMOS_TEST_SENTINEL": "kept", "DIMOS_EXPECTED_CWD": str(tmp_path)},
    )
    manager = WorkerManagerPython(
        g=GlobalConfig(n_workers=1),
        worker_launcher=CommandWorkerLauncher(material, startup_timeout=5.0),
    )
    manager.start()
    pid = manager.workers[0].pid
    assert pid is not None
    os.killpg(pid, signal.SIGSTOP)

    manager.stop()

    deadline = time.monotonic() + 3.0
    while psutil.pid_exists(pid) and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not psutil.pid_exists(pid)


@pytest.mark.skipif_macos_bug
def test_venv_worker_connection_timeout_cleanup(tmp_path) -> None:
    sleeper = tmp_path / "sleepy-python"
    sleeper.write_text("#!/bin/sh\nsleep 30\n")
    sleeper.chmod(0o755)
    worker = PythonWorker(launcher=VenvWorkerLauncher(str(sleeper), startup_timeout=0.2))

    with pytest.raises(TimeoutError, match="Timed out"):
        worker.start_process()

    time.sleep(0.2)
    assert not any(
        "sleepy-python" in " ".join(proc.info.get("cmdline") or [])
        for proc in psutil.process_iter(["cmdline"])
    )


@pytest.mark.skipif_macos_bug
def test_venv_worker_stdout_stderr_noise_does_not_corrupt_control() -> None:
    manager = WorkerManagerPython(
        g=GlobalConfig(n_workers=1),
        worker_launcher=VenvWorkerLauncher(sys.executable, startup_timeout=5.0),
    )
    try:
        module = manager.deploy(NoisyModule, global_config, {})
        module.start()
        assert module.ping() == "pong"
        module.stop()
    finally:
        manager.stop()


@pytest.mark.skipif_macos_bug
def test_venv_worker_startup_import_error_propagates(tmp_path) -> None:
    failing_python = tmp_path / "failing-python"
    failing_python.write_text("#!/bin/sh\nexit 42\n")
    failing_python.chmod(0o755)
    worker = PythonWorker(launcher=VenvWorkerLauncher(str(failing_python), startup_timeout=1.0))

    with pytest.raises(RuntimeError, match="exit code 42"):
        worker.start_process()


@pytest.mark.skipif_macos_bug
def test_worker_manager_basic(create_worker_manager):
    worker_manager = create_worker_manager(n_workers=2)
    module = worker_manager.deploy(SimpleModule, global_config, {})
    module.start()

    result = module.increment()
    assert result == 1

    result = module.increment()
    assert result == 2

    result = module.get_counter()
    assert result == 2

    module.stop()


@pytest.mark.skipif_macos_bug
def test_worker_manager_default_forkserver_lifecycle() -> None:
    manager = WorkerManagerPython(g=GlobalConfig(n_workers=1))
    try:
        manager.start()
        worker = manager.workers[0]
        pid = worker.pid
        assert pid is not None

        module = manager.deploy(SimpleModule, global_config, {})
        module.start()
        assert module.increment() == 1
        assert module.get_counter() == 1
        module.stop()

        worker.shutdown()
        deadline = time.monotonic() + 3.0
        while psutil.pid_exists(pid) and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not psutil.pid_exists(pid)
    finally:
        manager.stop()
        reset_forkserver_context()


@pytest.mark.skipif_macos_bug
def test_worker_manager_multiple_different_modules(create_worker_manager):
    worker_manager = create_worker_manager(n_workers=2)
    module1 = worker_manager.deploy(SimpleModule, global_config, {})
    module2 = worker_manager.deploy(AnotherModule, global_config, {})

    module1.start()
    module2.start()

    # Each module has its own state
    module1.increment()
    module1.increment()
    module2.add(10)

    assert module1.get_counter() == 2
    assert module2.get_value() == 110

    # Stop modules to clean up threads
    module1.stop()
    module2.stop()


@pytest.mark.skipif_macos_bug
def test_worker_manager_parallel_deployment(create_worker_manager):
    worker_manager = create_worker_manager(n_workers=2)
    modules = worker_manager.deploy_parallel(
        [
            (SimpleModule, global_config, {}),
            (AnotherModule, global_config, {}),
            (ThirdModule, global_config, {}),
        ]
    )

    assert len(modules) == 3
    module1, module2, module3 = modules

    # Start all modules
    module1.start()
    module2.start()
    module3.start()

    # Each module has its own state
    module1.increment()
    module2.add(50)
    module3.multiply(5)

    assert module1.get_counter() == 1
    assert module2.get_value() == 150
    assert module3.get_multiplier() == 5

    # Stop modules
    module1.stop()
    module2.stop()
    module3.stop()


@pytest.mark.skipif_macos_bug
def test_collect_stats(create_worker_manager):
    from dimos.core.resource_monitor.monitor import StatsMonitor

    manager = create_worker_manager(n_workers=2)
    module1 = manager.deploy(SimpleModule, global_config, {})
    module2 = manager.deploy(AnotherModule, global_config, {})
    module1.start()
    module2.start()

    # Use a capturing logger to collect stats via StatsMonitor
    captured: list[list[WorkerStats]] = []

    class CapturingLogger:
        def log_stats(self, coordinator, workers):
            captured.append(workers)

        def stop(self):
            pass

    monitor = StatsMonitor(manager, resource_logger=CapturingLogger(), interval=0.5)
    monitor.start()
    import time

    time.sleep(1.5)
    monitor.stop()

    assert len(captured) >= 1
    stats = captured[-1]
    assert len(stats) == 2

    for s in stats:
        assert s.alive is True
        assert s.pid > 0
        assert s.pss >= 0
        assert s.num_threads >= 1
        assert s.num_fds >= 0
        assert s.io_read_bytes >= 0
        assert s.io_write_bytes >= 0

    # At least one worker should report module names
    all_modules = [name for s in stats for name in s.modules]
    assert "SimpleModule" in all_modules
    assert "AnotherModule" in all_modules

    module1.stop()
    module2.stop()


@pytest.mark.skipif_macos_bug
def test_worker_pool_modules_share_workers(create_worker_manager):
    manager = create_worker_manager(n_workers=1)
    module1 = manager.deploy(SimpleModule, global_config, {})
    module2 = manager.deploy(AnotherModule, global_config, {})

    module1.start()
    module2.start()

    # Verify isolated state
    module1.increment()
    module1.increment()
    module2.add(10)

    assert module1.get_counter() == 2
    assert module2.get_value() == 110

    # Verify only 1 worker process was used
    assert len(manager._workers) == 1
    assert manager._workers[0].module_count == 2

    module1.stop()
    module2.stop()


@pytest.fixture
def manager_and_modules():
    """Fixture that tracks deployed modules and stops them on teardown."""
    manager = None
    modules = []

    def _create(n_workers):
        nonlocal manager
        g = GlobalConfig(n_workers=n_workers)
        manager = WorkerManagerPython(g=g)
        manager.start()
        return manager, modules

    yield _create

    for m in reversed(modules):
        m.stop()
    if manager is not None:
        manager.stop()


@pytest.mark.skipif_macos_bug
def test_health_check_alive_workers(manager_and_modules):
    manager, modules = manager_and_modules(n_workers=2)
    module = manager.deploy(SimpleModule, global_config, {})
    modules.append(module)
    module.start()

    assert manager.health_check() is True


@pytest.mark.skipif_macos_bug
def test_add_workers_grows_pool(manager_and_modules):
    manager, modules = manager_and_modules(n_workers=1)
    manager.add_workers(2)

    assert len(manager._workers) == 3

    # Deploy on the expanded pool and verify it works
    module = manager.deploy(SimpleModule, global_config, {})
    modules.append(module)
    module.start()
    assert module.increment() == 1


@pytest.mark.skipif_macos_bug
def test_load_balancing_distributes_modules(manager_and_modules):
    manager, modules = manager_and_modules(n_workers=2)

    for _ in range(4):
        m = manager.deploy(SimpleModule, global_config, {})
        modules.append(m)
        m.start()

    # Each worker should have 2 modules (even distribution)
    counts = [w.module_count for w in manager._workers]
    assert counts == [2, 2]


@pytest.mark.skipif_macos_bug
def test_dedicated_worker_gets_own_process(manager_and_modules):
    manager, modules = manager_and_modules(n_workers=2)

    heavy = manager.deploy(HeavyModule, global_config, {})
    modules.append(heavy)
    heavy.start()

    for _ in range(3):
        m = manager.deploy(SimpleModule, global_config, {})
        modules.append(m)
        m.start()

    counts = sorted(w.module_count for w in manager._workers)
    # One worker hosts only the dedicated module; the other hosts the 3 light ones.
    assert counts == [1, 3]
    assert sum(1 for w in manager._workers if w.dedicated) == 1


@pytest.mark.skipif_macos_bug
def test_dedicated_workers_trigger_autoscale(manager_and_modules):
    manager, modules = manager_and_modules(n_workers=2)

    heavy1 = manager.deploy(HeavyModule, global_config, {})
    modules.append(heavy1)
    heavy1.start()
    heavy2 = manager.deploy(AnotherHeavyModule, global_config, {})
    modules.append(heavy2)
    heavy2.start()

    # 2 dedicated modules require >= 4 total workers so non-dedicated workers
    # at least match the dedicated count.
    assert len(manager._workers) == 4
    assert sum(1 for w in manager._workers if w.dedicated) == 2
