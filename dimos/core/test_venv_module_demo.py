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

from pathlib import Path
import sys
from types import MappingProxyType

import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.runtime_environment import PythonVenvRuntimeEnvironment

_BUILD_WITHOUT_RERUN = MappingProxyType({"g": {"viewer": "none", "n_workers": 1}})
_REPO_ROOT = Path(__file__).resolve().parents[2]
_DEMO_SRC = _REPO_ROOT / "packages" / "dimos-demo-worker-module" / "src"


@pytest.fixture
def demo_import_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.syspath_prepend(str(_DEMO_SRC))
    sys.modules.pop("dimos_demo_worker_module", None)
    sys.modules.pop("dimos_demo_worker_module.blueprint", None)
    sys.modules.pop("dimos_demo_worker_module.runtime_dependency", None)


def test_demo_blueprint_imports_and_builds_without_worker_dependency_in_coordinator(
    demo_import_path: None,
) -> None:
    from dimos_demo_worker_module import VenvDemoConsumer, VenvDemoPublisher, venv_demo_blueprint

    runtime = PythonVenvRuntimeEnvironment(
        name="dimos-demo-venv",
        python_executable=Path(sys.executable),
        env={"PYTHONPATH": str(_DEMO_SRC)},
    )
    coordinator = ModuleCoordinator.build(venv_demo_blueprint(runtime), _BUILD_WITHOUT_RERUN.copy())
    try:
        assert coordinator.get_instance(VenvDemoPublisher) is not None
        assert coordinator.get_instance(VenvDemoConsumer) is not None
        assert coordinator._module_manager_keys[VenvDemoPublisher] == "python:dimos-demo-venv"
        assert coordinator._module_manager_keys[VenvDemoConsumer] == "python"
    finally:
        coordinator.stop()


def test_demo_runtime_runs_package_helper_in_venv_worker(
    demo_import_path: None,
) -> None:
    from dimos_demo_worker_module import VENV_DEMO_ENV_NAME, VenvDemoConsumer, venv_demo_blueprint

    runtime = PythonVenvRuntimeEnvironment(
        name=VENV_DEMO_ENV_NAME,
        python_executable=Path(sys.executable),
        env={"PYTHONPATH": str(_DEMO_SRC)},
    )
    coordinator = ModuleCoordinator.build(venv_demo_blueprint(runtime), _BUILD_WITHOUT_RERUN.copy())
    try:
        consumer = coordinator.get_instance(VenvDemoConsumer)
        assert consumer is not None
        assert consumer.consume_message("ok") == "worker-runtime::ok"
    finally:
        coordinator.stop()
