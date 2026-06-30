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
import shutil
import sys
from types import ModuleType

import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.runtime_environment import PythonProjectRuntimeEnvironment

_BUILD_WITHOUT_RERUN = {"g": {"viewer": "none", "n_workers": 1}}
_REPO_ROOT = Path(__file__).resolve().parents[2]
_GPD_DEMO_PROJECT = _REPO_ROOT / "packages" / "dimos-gpd-worker-demo"
_GPD_DEMO_SRC = _GPD_DEMO_PROJECT / "src"
_GPD_COMMIT = "c088d8ae2f7965b067e9a12b3c0dacdbe9da924a"


@pytest.fixture
def gpd_demo_import_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.syspath_prepend(str(_GPD_DEMO_SRC))
    sys.modules.pop("dimos_gpd_worker_demo", None)
    sys.modules.pop("dimos_gpd_worker_demo.blueprint", None)


def test_gpd_demo_package_pins_gpd_dependency() -> None:
    pyproject = (_GPD_DEMO_PROJECT / "pyproject.toml").read_text()
    assert '"dimos"' in pyproject
    assert 'dimos = { path = "../..", editable = true }' in pyproject
    assert "gpd @ git+https://github.com/TomCC7/gpd.git" in pyproject
    assert _GPD_COMMIT in pyproject


def test_gpd_demo_pixi_project_has_native_build_dependencies() -> None:
    pixi = (_GPD_DEMO_PROJECT / "pixi.toml").read_text()

    for package in (
        "cmake",
        "compilers",
        "eigen",
        "opencv",
        "pcl",
        "pkg-config",
        "python",
        "uv",
    ):
        assert f"{package} = " in pixi


def test_gpd_demo_blueprint_imports_safely_and_places_project_runtime(
    gpd_demo_import_path: None,
) -> None:
    from dimos_gpd_worker_demo import GPD_DEMO_ENV_NAME, GpdImportProbe, gpd_worker_demo_blueprint

    blueprint = gpd_worker_demo_blueprint()
    environment = blueprint.runtime_environment_registry.environments[GPD_DEMO_ENV_NAME]

    assert isinstance(environment, PythonProjectRuntimeEnvironment)
    assert environment.project == _GPD_DEMO_PROJECT.resolve()
    assert blueprint.runtime_placement_map[GpdImportProbe] == GPD_DEMO_ENV_NAME


def test_gpd_import_rpc_lazily_imports_stubbed_gpd_core(
    gpd_demo_import_path: None,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from dimos_gpd_worker_demo import GpdImportProbe

    gpd_module = ModuleType("gpd")
    gpd_core_module = ModuleType("gpd.core")
    gpd_core_module.__file__ = "/stub/gpd/core.py"
    monkeypatch.setitem(sys.modules, "gpd", gpd_module)
    monkeypatch.setitem(sys.modules, "gpd.core", gpd_core_module)

    result = GpdImportProbe.import_gpd_core(object())

    assert result == "gpd import ok: gpd.core (/stub/gpd/core.py)"


@pytest.mark.skipif(shutil.which("pixi") is None, reason="Pixi is not installed")
def test_gpd_demo_pixi_project_runtime_resolves_when_prepared(
    gpd_demo_import_path: None,
) -> None:
    prepared_python = _GPD_DEMO_PROJECT / ".venv" / "bin" / "python"
    if not prepared_python.exists():
        pytest.skip("GPD demo project runtime is not prepared")

    from dimos_gpd_worker_demo import GPD_DEMO_ENV_NAME, gpd_worker_demo_blueprint

    environment = gpd_worker_demo_blueprint().runtime_environment_registry.environments[
        GPD_DEMO_ENV_NAME
    ]
    material = environment.resolve_python_project()

    assert material.argv_prefix == ["pixi", "run", "uv", "run", "--no-sync", "python"]
    assert material.cwd == _GPD_DEMO_PROJECT.resolve()
    assert material.prepared_python == prepared_python


@pytest.mark.skipif(shutil.which("pixi") is None, reason="Pixi is not installed")
def test_gpd_demo_pixi_project_runtime_imports_gpd_when_prepared(
    gpd_demo_import_path: None,
) -> None:
    prepared_python = _GPD_DEMO_PROJECT / ".venv" / "bin" / "python"
    if not prepared_python.exists():
        pytest.skip("GPD demo project runtime is not prepared")

    from dimos_gpd_worker_demo import GpdImportProbe, gpd_worker_demo_blueprint

    coordinator = ModuleCoordinator.build(
        gpd_worker_demo_blueprint(),
        _BUILD_WITHOUT_RERUN.copy(),
    )
    try:
        probe = coordinator.get_instance(GpdImportProbe)
        assert probe.import_gpd_core().startswith("gpd import ok: gpd.core (")
    finally:
        coordinator.stop()
