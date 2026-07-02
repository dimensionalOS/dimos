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

import importlib
import os
from pathlib import Path
import subprocess
import sys

import pytest

from dimos.core.coordination.worker_launcher import CommandWorkerLauncher, VenvWorkerLauncher
from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.runtime_environment import PythonProjectRuntimeEnvironment, RuntimePlacement

REPO_ROOT = Path(__file__).resolve().parents[2]
EXAMPLE_ROOT = REPO_ROOT / "examples" / "dimos-demo-worker-module"
EXAMPLE_SRC = EXAMPLE_ROOT / "src"


def _example_pythonpath() -> str:
    paths = [str(EXAMPLE_SRC), str(REPO_ROOT)]
    paths.extend(path for path in sys.path if path)
    if existing := os.environ.get("PYTHONPATH"):
        paths.append(existing)
    return os.pathsep.join(paths)


def test_demo_runtime_project_contract_import_does_not_import_runtime(monkeypatch) -> None:
    monkeypatch.syspath_prepend(str(EXAMPLE_SRC))
    sys.modules.pop("dimos_demo_worker_module.blueprint", None)
    sys.modules.pop("dimos_demo_worker_module.contract", None)
    sys.modules.pop("dimos_demo_worker_module.runtime", None)

    blueprint_module = importlib.import_module("dimos_demo_worker_module.blueprint")

    assert "dimos_demo_worker_module.contract" in sys.modules
    assert "dimos_demo_worker_module.runtime" not in sys.modules
    placement = blueprint_module.demo_worker_runtime_blueprint.runtime_placement_map[
        blueprint_module.DemoWorkerModule
    ]
    assert placement.implementation == "dimos_demo_worker_module.runtime.DemoWorkerRuntimeModule"


@pytest.mark.skipif_macos_bug
def test_demo_runtime_module_executes_through_runtime_worker_rpc(monkeypatch) -> None:
    monkeypatch.syspath_prepend(str(EXAMPLE_SRC))
    contract_module = importlib.import_module("dimos_demo_worker_module.contract")
    demo_contract = contract_module.DemoWorkerModule
    placement = RuntimePlacement(
        runtime="demo-worker-test-runtime",
        implementation="dimos_demo_worker_module.runtime.DemoWorkerRuntimeModule",
    )
    launcher = VenvWorkerLauncher(
        Path(sys.executable),
        env={"PYTHONPATH": _example_pythonpath()},
        runtime_name=placement.runtime,
    )
    manager = WorkerManagerPython(
        g=GlobalConfig(n_workers=1, viewer="none"),
        worker_launcher=launcher,
    )
    module = None

    try:
        manager.start()
        module = manager.deploy(
            demo_contract,
            global_config,
            {},
            runtime_placement=placement,
        )
        transform = module.transform
        assert transform(" hello ") == "demo-runtime:HELLO"
    finally:
        if module is not None:
            module.stop()
        manager.stop()


@pytest.mark.skipif_macos_bug
def test_demo_runtime_project_executes_with_project_worker(monkeypatch) -> None:
    monkeypatch.syspath_prepend(str(EXAMPLE_SRC))
    contract_module = importlib.import_module("dimos_demo_worker_module.contract")
    demo_contract = contract_module.DemoWorkerModule
    runtime = PythonProjectRuntimeEnvironment(
        name="demo-worker-test-runtime",
        project=EXAMPLE_ROOT,
    )
    subprocess.run(
        ("uv", "sync", "--locked"),
        cwd=EXAMPLE_ROOT,
        check=True,
        capture_output=True,
        text=True,
        timeout=180,
    )
    placement = RuntimePlacement(
        runtime=runtime.name,
        implementation="dimos_demo_worker_module.runtime.DemoWorkerRuntimeModule",
    )
    launcher = CommandWorkerLauncher(runtime.resolve_python_project())
    manager = WorkerManagerPython(
        g=GlobalConfig(n_workers=1, viewer="none"),
        worker_launcher=launcher,
    )
    module = None

    try:
        manager.start()
        module = manager.deploy(
            demo_contract,
            global_config,
            {},
            runtime_placement=placement,
        )
        runtime_python = module.runtime_python()
        runtime_dependency_label = module.runtime_dependency_label
        assert str(EXAMPLE_ROOT / ".venv") in runtime_python
        assert runtime_dependency_label() == "RuntimeDependency"
    finally:
        if module is not None:
            module.stop()
        manager.stop()
