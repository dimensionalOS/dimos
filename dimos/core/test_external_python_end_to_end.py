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

from __future__ import annotations

from collections.abc import Generator
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import cast

import pytest

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.external_python_worker import ExternalPythonWorker
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.coordination.worker_manager_external_python import WorkerManagerExternalPython
from dimos.core.rpc_client import RPCClient
from examples.external_python_module.contract import ExampleExternal
from examples.external_python_module.run import ExampleConsumer


@pytest.fixture
def running_external_example() -> Generator[
    tuple[
        ModuleCoordinator, WorkerManagerExternalPython, list[ExternalPythonWorker], list[RPCClient]
    ],
    None,
    None,
]:
    if shutil.which("pixi") is None:
        pytest.skip("Pixi is required for the external Python runtime E2E")

    coordinator = ModuleCoordinator()
    manager = cast("WorkerManagerExternalPython", coordinator._managers["external-python"])
    workers: list[ExternalPythonWorker] = []
    proxies: list[RPCClient] = []
    child_pids: list[int] = []
    try:
        coordinator.start()
        coordinator.load_blueprint(
            autoconnect(
                ExampleExternal.blueprint(initial_multiplier=3), ExampleConsumer.blueprint()
            )
        )
        workers.extend(manager._workers.values())
        child_pids.extend(pid for worker in workers if (pid := worker.pid) is not None)
        proxies.extend(cast("RPCClient", proxy) for proxy in coordinator._deployed_modules.values())
        yield coordinator, manager, workers, proxies
    finally:
        workers.extend(worker for worker in manager._workers.values() if worker not in workers)
        child_pids.extend(pid for worker in workers if (pid := worker.pid) is not None)
        proxies.extend(
            cast("RPCClient", proxy)
            for proxy in coordinator._deployed_modules.values()
            if proxy not in proxies
        )
        try:
            coordinator.stop()
        finally:
            try:
                for proxy in proxies:
                    proxy.stop_rpc_client()
            finally:
                for transport in coordinator._transport_registry.values():
                    transport.stop()
        for worker in workers:
            assert worker.pid is None
        for pid in child_pids:
            with pytest.raises(ProcessLookupError):
                os.kill(pid, 0)


def test_external_example_runs_through_real_coordinator_and_restarts(
    running_external_example: tuple[
        ModuleCoordinator, WorkerManagerExternalPython, list[ExternalPythonWorker], list[RPCClient]
    ],
) -> None:
    coordinator, manager, workers, proxies = running_external_example
    external_proxy = coordinator.get_instance(ExampleExternal)
    consumer_proxy = coordinator.get_instance(ExampleConsumer)

    assert external_proxy.get_multiplier() == 3
    assert external_proxy.set_multiplier(5) == "External multiplier set to 5"
    assert external_proxy.get_multiplier() == 5
    assert consumer_proxy is not None
    first_worker = next(iter(manager._workers.values()))
    first_pid = first_worker.pid
    assert first_pid is not None
    workers.append(first_worker)
    proxies.append(external_proxy)

    coordinator.restart_module(ExampleExternal, reload_source=False)

    replacement_proxy = coordinator.get_instance(ExampleExternal)
    replacement_worker = next(iter(manager._workers.values()))
    workers.append(replacement_worker)
    proxies.append(replacement_proxy)
    assert replacement_worker.pid is not None
    assert replacement_worker.pid != first_pid
    assert replacement_proxy.get_multiplier() == 3


def test_external_example_entrypoint_exits_after_clean_restart() -> None:
    if shutil.which("pixi") is None:
        pytest.skip("Pixi is required for the external Python runtime E2E")

    root = Path(__file__).resolve().parents[2]
    process = subprocess.Popen(
        [sys.executable, "examples/external_python_module/run.py"],
        cwd=root,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
        text=True,
    )
    try:
        stdout, stderr = process.communicate(timeout=60)
    finally:
        if process.poll() is None:
            try:
                os.killpg(process.pid, 9)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)

    assert process.returncode == 0, stderr
    assert "external multiplier: 3" in stdout
    assert "restarted external multiplier: 3" in stdout
    with pytest.raises(ProcessLookupError):
        os.killpg(process.pid, 0)
