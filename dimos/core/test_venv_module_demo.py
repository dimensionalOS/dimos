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

from dimos.core.coordination.worker_manager_python import WorkerManagerPython
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.runtime_environment import (
    PythonProjectRuntimeEnvironment,
    RuntimeEnvironmentRegistry,
    RuntimePlacement,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
EXAMPLE_ROOT = REPO_ROOT / "examples" / "dimos-demo-worker-module"
EXAMPLE_SRC = EXAMPLE_ROOT / "src"


def _example_pythonpath() -> str:
    paths = [str(EXAMPLE_SRC)]
    paths.extend(
        path for path in sys.path if path and ("site-packages" in path or "dist-packages" in path)
    )
    if existing := os.environ.get("PYTHONPATH"):
        paths.append(existing)
    return os.pathsep.join(dict.fromkeys(paths))


@pytest.fixture
def demo_runtime_worker(monkeypatch):
    monkeypatch.syspath_prepend(str(EXAMPLE_SRC))
    contract_module = importlib.import_module("dimos_demo_worker_module.contract")
    demo_contract = contract_module.DemoWorkerModule
    runtime = PythonProjectRuntimeEnvironment(
        name="demo-worker-test-runtime",
        project=EXAMPLE_ROOT,
        env={"PYTHONPATH": _example_pythonpath(), "UV_PYTHON": sys.executable},
    )
    subprocess.run(
        ("uv", "sync", "--locked"),
        cwd=EXAMPLE_ROOT,
        env={**os.environ, **runtime.env},
        check=True,
        capture_output=True,
        text=True,
        timeout=180,
    )
    placement = RuntimePlacement(
        runtime=runtime.name,
        implementation="dimos_demo_worker_module.runtime.DemoWorkerRuntimeModule",
    )
    manager = WorkerManagerPython(
        g=GlobalConfig(n_workers=1, viewer="none"),
    )
    manager.register_runtime_environments(RuntimeEnvironmentRegistry().register(runtime))
    manager.start()
    module = manager.deploy(
        demo_contract,
        global_config,
        {},
        runtime_placement=placement,
    )
    try:
        yield module
    finally:
        module.stop()
        manager.stop()


def test_demo_runtime_project_contract_import_does_not_import_runtime(monkeypatch) -> None:
    monkeypatch.syspath_prepend(str(EXAMPLE_SRC))
    monkeypatch.delitem(sys.modules, "dimos_demo_worker_module.blueprint", raising=False)
    monkeypatch.delitem(sys.modules, "dimos_demo_worker_module.contract", raising=False)
    monkeypatch.delitem(sys.modules, "dimos_demo_worker_module.runtime", raising=False)

    blueprint_module = importlib.import_module("dimos_demo_worker_module.blueprint")

    assert "dimos_demo_worker_module.contract" in sys.modules
    assert "dimos_demo_worker_module.runtime" not in sys.modules
    placement = blueprint_module.demo_worker_runtime_blueprint.runtime_placement_map[
        blueprint_module.DemoWorkerModule
    ]
    assert placement.implementation == "dimos_demo_worker_module.runtime.DemoWorkerRuntimeModule"


def test_example_pythonpath_excludes_host_stdlib(monkeypatch) -> None:
    site_packages = "/host/.venv/lib/python3.12/site-packages"
    stdlib = "/usr/lib/python3.12"
    monkeypatch.setattr(sys, "path", [stdlib, site_packages, ""])
    monkeypatch.delenv("PYTHONPATH", raising=False)

    entries = _example_pythonpath().split(os.pathsep)

    assert str(EXAMPLE_SRC) in entries
    assert site_packages in entries
    assert stdlib not in entries


@pytest.mark.skipif_macos_bug
def test_demo_runtime_project_executes_with_project_worker(demo_runtime_worker) -> None:
    runtime_python = demo_runtime_worker.runtime_python()
    runtime_dependency_label = demo_runtime_worker.runtime_dependency_label
    assert str(EXAMPLE_ROOT / ".venv") in runtime_python
    assert runtime_dependency_label() == "RuntimeDependency"
