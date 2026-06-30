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

import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys
from types import ModuleType

from dimos_gpd_grasp_demo import (
    GPD_RUNTIME_HELP,
    GPDGraspGenModule,
    NormalizedGraspCandidate,
    pointcloud_to_gpd_xyz,
)
import numpy as np
import pytest

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.runtime_environment import (
    PythonProjectLaunchMaterial,
    PythonProjectRuntimeEnvironment,
)
from dimos.msgs.grasping_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

_BUILD_WITHOUT_RERUN = {"g": {"viewer": "none", "n_workers": 1}}
_REPO_ROOT = Path(__file__).resolve().parents[2]
_GPD_GRASP_DEMO_PROJECT = _REPO_ROOT / "packages" / "dimos-gpd-grasp-demo"
_GPD_GRASP_DEMO_SRC = _GPD_GRASP_DEMO_PROJECT / "src"
_GPD_COMMIT = "c088d8ae2f7965b067e9a12b3c0dacdbe9da924a"


class FakeGpdProjectRuntime(PythonProjectRuntimeEnvironment):
    _fake_package_root: Path

    def __init__(self, fake_package_root: Path) -> None:
        super().__init__(name="fake-gpd-project", project=_REPO_ROOT)
        object.__setattr__(self, "_fake_package_root", fake_package_root)

    def resolve_python_project(self) -> PythonProjectLaunchMaterial:
        pythonpath = os.pathsep.join(
            [
                str(self._fake_package_root),
                str(_GPD_GRASP_DEMO_SRC),
                str(_REPO_ROOT),
                os.environ.get("PYTHONPATH", ""),
            ]
        )
        return PythonProjectLaunchMaterial(
            argv_prefix=[sys.executable],
            cwd=_REPO_ROOT,
            env={"PYTHONPATH": pythonpath},
            runtime_name=self.name,
            project=self.project,
            convention="test-fake-gpd",
            prepared_python=Path(sys.executable),
        )


@pytest.fixture
def gpd_grasp_demo_import_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.syspath_prepend(str(_GPD_GRASP_DEMO_SRC))
    sys.modules.pop("dimos_gpd_grasp_demo", None)
    sys.modules.pop("dimos_gpd_grasp_demo.blueprint", None)


def test_gpd_grasp_demo_package_pins_gpd_dependency() -> None:
    pyproject = (_GPD_GRASP_DEMO_PROJECT / "pyproject.toml").read_text()
    assert 'name = "dimos-gpd-grasp-demo"' in pyproject
    assert '"dimos"' in pyproject
    assert 'dimos = { path = "../..", editable = true }' in pyproject
    assert "gpd @ git+https://github.com/TomCC7/gpd.git" in pyproject
    assert _GPD_COMMIT in pyproject
    assert "allow-direct-references = true" in pyproject


def test_gpd_grasp_demo_package_is_discoverable_without_path_patch() -> None:
    sys.modules.pop("dimos_gpd_grasp_demo", None)
    sys.modules.pop("dimos_gpd_grasp_demo.blueprint", None)

    spec = importlib.util.find_spec("dimos_gpd_grasp_demo")

    assert spec is not None
    assert spec.origin is not None
    assert _GPD_GRASP_DEMO_SRC in Path(spec.origin).parents


def test_gpd_grasp_demo_pixi_project_has_native_build_dependencies() -> None:
    pixi = (_GPD_GRASP_DEMO_PROJECT / "pixi.toml").read_text()

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


def test_gpd_grasp_demo_blueprint_imports_without_gpd_core(
    gpd_grasp_demo_import_path: None,
) -> None:
    sys.modules.pop("gpd", None)
    sys.modules.pop("gpd.core", None)

    from dimos_gpd_grasp_demo import (
        GPD_GRASP_DEMO_ENV_NAME,
        GPD_GRASP_DEMO_PROJECT,
        GpdGraspImportProbe,
        gpd_grasp_demo_blueprint,
    )

    assert "gpd.core" not in sys.modules

    blueprint = gpd_grasp_demo_blueprint()
    environment = blueprint.runtime_environment_registry.environments[GPD_GRASP_DEMO_ENV_NAME]

    assert isinstance(environment, PythonProjectRuntimeEnvironment)
    assert GPD_GRASP_DEMO_PROJECT == _GPD_GRASP_DEMO_PROJECT.resolve()
    assert environment.project == _GPD_GRASP_DEMO_PROJECT.resolve()
    assert blueprint.runtime_placement_map[GpdGraspImportProbe] == GPD_GRASP_DEMO_ENV_NAME


def test_gpd_grasp_gen_blueprint_places_only_generator_in_project_runtime(
    gpd_grasp_demo_import_path: None,
) -> None:
    from dimos_gpd_grasp_demo import (
        GPD_GRASP_DEMO_ENV_NAME,
        GPD_GRASP_DEMO_PROJECT,
        GPDGraspGenModule,
        gpd_grasp_gen_blueprint,
    )

    blueprint = gpd_grasp_gen_blueprint()
    environment = blueprint.runtime_environment_registry.environments[GPD_GRASP_DEMO_ENV_NAME]

    assert isinstance(blueprint, Blueprint)
    assert isinstance(environment, PythonProjectRuntimeEnvironment)
    assert environment.project == GPD_GRASP_DEMO_PROJECT
    assert blueprint.runtime_placement_map == {GPDGraspGenModule: GPD_GRASP_DEMO_ENV_NAME}


def test_gpd_grasp_import_rpc_lazily_imports_stubbed_gpd_core(
    gpd_grasp_demo_import_path: None,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from dimos_gpd_grasp_demo import GpdGraspImportProbe

    assert "gpd.core" not in sys.modules

    gpd_module = ModuleType("gpd")
    gpd_core_module = ModuleType("gpd.core")
    gpd_core_module.__file__ = "/stub/gpd/core.py"
    monkeypatch.setitem(sys.modules, "gpd", gpd_module)
    monkeypatch.setitem(sys.modules, "gpd.core", gpd_core_module)

    result = GpdGraspImportProbe.import_gpd_core(object())

    assert result == "gpd import ok: gpd.core (/stub/gpd/core.py)"


def test_gpd_grasp_gen_converts_pointcloud_to_backend_xyz() -> None:
    pointcloud = PointCloud2.from_numpy(
        np.array([[0.0, 0.1, 0.2], [np.nan, 1.0, 1.0], [0.3, 0.4, 0.5]], dtype=np.float32),
        frame_id="camera",
        timestamp=12.5,
    )

    xyz = pointcloud_to_gpd_xyz(pointcloud)

    assert xyz.dtype == np.float32
    assert xyz.flags.c_contiguous
    np.testing.assert_allclose(xyz, [[0.0, 0.1, 0.2], [0.3, 0.4, 0.5]])


def test_gpd_grasp_gen_publishes_candidates_and_pose_array(mocker) -> None:  # type: ignore[no-untyped-def]
    received_points: list[np.ndarray] = []

    def backend(points: np.ndarray) -> list[NormalizedGraspCandidate]:
        received_points.append(points)
        return [
            NormalizedGraspCandidate(
                position=(0.1, 0.2, 0.3),
                orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                score=0.75,
                width=0.04,
            )
        ]

    module = GPDGraspGenModule(backend=backend)
    publish = mocker.patch.object(module.grasp_candidates, "publish")
    pointcloud = PointCloud2.from_numpy(
        np.array([[0.0, 0.0, 0.0], [0.1, 0.1, 0.1]], dtype=np.float32),
        frame_id="object_frame",
        timestamp=42.25,
    )

    try:
        poses = module.generate_grasps(pointcloud)

        assert poses is not None
        assert poses.header.frame_id == "object_frame"
        assert poses.header.timestamp == pytest.approx(42.25)
        assert len(poses) == 1
        np.testing.assert_allclose(received_points[0], pointcloud.points_f32())
        published = publish.call_args.args[0]
        assert isinstance(published, GraspCandidateArray)
        assert published.frame_id == "object_frame"
        assert len(published) == 1
        assert published[0].score == pytest.approx(0.75)
        assert published[0].jaw_width == pytest.approx(0.04)
        assert published.to_rerun() is not None
    finally:
        module.stop()


def test_gpd_grasp_gen_empty_pointcloud_publishes_empty_debug(mocker) -> None:  # type: ignore[no-untyped-def]
    backend = mocker.Mock(return_value=[])
    module = GPDGraspGenModule(backend=backend)
    publish = mocker.patch.object(module.grasp_candidates, "publish")

    try:
        result = module.generate_grasps(PointCloud2.from_numpy(np.zeros((0, 3), dtype=np.float32)))

        assert result is None
        backend.assert_not_called()
        published = publish.call_args.args[0]
        assert isinstance(published, GraspCandidateArray)
        assert len(published) == 0
        assert published.to_rerun() is not None
    finally:
        module.stop()


def test_gpd_grasp_gen_all_nonfinite_pointcloud_skips_backend(mocker) -> None:  # type: ignore[no-untyped-def]
    backend = mocker.Mock(return_value=[])
    module = GPDGraspGenModule(backend=backend)
    publish = mocker.patch.object(module.grasp_candidates, "publish")
    pointcloud = PointCloud2.from_numpy(
        np.array([[np.nan, 0.0, 0.0], [np.inf, 1.0, 1.0]], dtype=np.float32),
        frame_id="invalid_frame",
    )

    try:
        result = module.generate_grasps(pointcloud)

        assert result is None
        backend.assert_not_called()
        published = publish.call_args.args[0]
        assert isinstance(published, GraspCandidateArray)
        assert published.frame_id == "invalid_frame"
        assert len(published) == 0
    finally:
        module.stop()


def test_gpd_grasp_gen_valid_cloud_backend_empty_returns_empty_candidates(mocker) -> None:  # type: ignore[no-untyped-def]
    backend = mocker.Mock(return_value=[])
    module = GPDGraspGenModule(backend=backend)
    publish = mocker.patch.object(module.grasp_candidates, "publish")
    pointcloud = PointCloud2.from_numpy(
        np.array([[0.0, 0.0, 0.0], [0.02, 0.0, 0.0]], dtype=np.float32),
        frame_id="valid_empty",
    )

    try:
        result = module.generate_grasps(pointcloud)

        assert result is None
        backend.assert_called_once()
        np.testing.assert_allclose(backend.call_args.args[0], pointcloud.points_f32())
        published = publish.call_args.args[0]
        assert isinstance(published, GraspCandidateArray)
        assert published.frame_id == "valid_empty"
        assert len(published) == 0
    finally:
        module.stop()


def test_gpd_grasp_gen_rejects_malformed_pointcloud(mocker) -> None:  # type: ignore[no-untyped-def]
    pointcloud = PointCloud2.from_numpy(np.zeros((1, 3), dtype=np.float32))
    mocker.patch.object(pointcloud, "points_f32", return_value=np.zeros((3,), dtype=np.float32))

    with pytest.raises(ValueError, match="shape"):
        pointcloud_to_gpd_xyz(pointcloud)


def test_gpd_grasp_gen_backend_import_failure_has_runtime_help(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sys.modules.pop("gpd", None)
    sys.modules.pop("gpd.core", None)
    pointcloud = PointCloud2.from_numpy(np.array([[0.0, 0.0, 0.0]], dtype=np.float32))
    module = GPDGraspGenModule()

    try:
        with pytest.raises(
            RuntimeError, match="GPD grasp detection backend is unavailable"
        ) as exc_info:
            module.generate_grasps(pointcloud)

        assert GPD_RUNTIME_HELP in str(exc_info.value)
        assert "gpd.core" not in sys.modules
    finally:
        module.stop()


def test_gpd_grasp_gen_backend_api_failure_has_runtime_help(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    gpd_module = ModuleType("gpd")
    gpd_core_module = ModuleType("gpd.core")
    monkeypatch.setitem(sys.modules, "gpd", gpd_module)
    monkeypatch.setitem(sys.modules, "gpd.core", gpd_core_module)
    pointcloud = PointCloud2.from_numpy(np.array([[0.0, 0.0, 0.0]], dtype=np.float32))
    module = GPDGraspGenModule()

    try:
        with pytest.raises(
            RuntimeError, match="GPD grasp detection backend is unavailable"
        ) as exc_info:
            module.generate_grasps(pointcloud)

        assert GPD_RUNTIME_HELP in str(exc_info.value)
    finally:
        module.stop()


def test_gpd_grasp_gen_project_runtime_rpc_uses_command_worker(
    gpd_grasp_demo_import_path: None,
    tmp_path: Path,
) -> None:
    fake_gpd = tmp_path / "gpd"
    fake_gpd.mkdir()
    (fake_gpd / "__init__.py").write_text("")
    (fake_gpd / "core.py").write_text(
        "class Cloud:\n"
        "    def __init__(self, points):\n"
        "        self.points = points\n"
        "\n"
        "class GraspDetector:\n"
        "    @classmethod\n"
        "    def from_preset(cls, preset):\n"
        "        return cls()\n"
        "\n"
        "    def detect_grasps(self, cloud):\n"
        "        assert cloud.points.shape[0] > 0\n"
        "        return [{\n"
        "            'position': [0.11, 0.12, 0.13],\n"
        "            'orientation': [0.0, 0.0, 0.0, 1.0],\n"
        "            'score': 0.9,\n"
        "            'width': 0.05,\n"
        "        }]\n"
    )

    from dimos_gpd_grasp_demo import gpd_grasp_gen_blueprint

    coordinator = ModuleCoordinator.build(
        gpd_grasp_gen_blueprint(runtime=FakeGpdProjectRuntime(tmp_path)),
        _BUILD_WITHOUT_RERUN.copy(),
    )
    try:
        detector = coordinator.get_instance(GPDGraspGenModule)
        poses = detector.generate_grasps(
            PointCloud2.from_numpy(
                np.array([[0.0, 0.0, 0.0], [0.02, 0.0, 0.0]], dtype=np.float32),
                frame_id="rpc_frame",
                timestamp=3.5,
            )
        )

        assert poses is not None
        assert poses.header.frame_id == "rpc_frame"
        assert poses.header.timestamp == pytest.approx(3.5)
        assert len(poses) == 1
        np.testing.assert_allclose(poses[0].position.to_tuple(), (0.11, 0.12, 0.13))
    finally:
        coordinator.stop()


@pytest.mark.skipif(shutil.which("pixi") is None, reason="Pixi is not installed")
def test_gpd_grasp_demo_pixi_project_runtime_resolves_when_prepared(
    gpd_grasp_demo_import_path: None,
) -> None:
    prepared_python = _GPD_GRASP_DEMO_PROJECT / ".venv" / "bin" / "python"
    if not prepared_python.exists():
        pytest.skip("GPD grasp demo project runtime is not prepared")

    from dimos_gpd_grasp_demo import GPD_GRASP_DEMO_ENV_NAME, gpd_grasp_demo_blueprint

    environment = gpd_grasp_demo_blueprint().runtime_environment_registry.environments[
        GPD_GRASP_DEMO_ENV_NAME
    ]
    material = environment.resolve_python_project()

    assert material.argv_prefix == ["pixi", "run", "uv", "run", "--no-sync", "python"]
    assert material.cwd == _GPD_GRASP_DEMO_PROJECT.resolve()
    assert material.prepared_python == prepared_python


@pytest.mark.skipif(shutil.which("pixi") is None, reason="Pixi is not installed")
def test_gpd_grasp_demo_pixi_project_runtime_imports_gpd_when_prepared(
    gpd_grasp_demo_import_path: None,
) -> None:
    prepared_python = _GPD_GRASP_DEMO_PROJECT / ".venv" / "bin" / "python"
    if not prepared_python.exists():
        pytest.skip("GPD grasp demo project runtime is not prepared")

    from dimos_gpd_grasp_demo import GpdGraspImportProbe, gpd_grasp_demo_blueprint

    coordinator = ModuleCoordinator.build(
        gpd_grasp_demo_blueprint(),
        _BUILD_WITHOUT_RERUN.copy(),
    )
    try:
        probe = coordinator.get_instance(GpdGraspImportProbe)
        assert probe.import_gpd_core().startswith("gpd import ok: gpd.core (")
    finally:
        coordinator.stop()


@pytest.mark.skipif(shutil.which("pixi") is None, reason="Pixi is not installed")
def test_gpd_grasp_demo_prepared_runtime_runs_actual_adapter_on_tiny_cloud() -> None:
    prepared_python = _GPD_GRASP_DEMO_PROJECT / ".venv" / "bin" / "python"
    if not prepared_python.exists():
        pytest.skip("GPD grasp demo project runtime is not prepared")

    script = """
import numpy as np
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos_gpd_grasp_demo import GPDGraspGenModule

module = GPDGraspGenModule()
try:
    xs, ys = np.meshgrid(np.linspace(-0.03, 0.03, 9), np.linspace(-0.03, 0.03, 9))
    zs = np.zeros_like(xs)
    pointcloud = PointCloud2.from_numpy(np.column_stack([xs.ravel(), ys.ravel(), zs.ravel()]).astype(np.float32))
    poses = module.generate_grasps(pointcloud)
    if poses is None or len(poses) == 0:
        raise AssertionError('real GPD adapter returned no grasps; cannot validate output normalization')
    first = poses[0]
    first.position.to_tuple()
    first.orientation.to_tuple()
finally:
    module.stop()
"""
    subprocess.run(
        [str(prepared_python), "-c", script],
        cwd=_GPD_GRASP_DEMO_PROJECT,
        check=True,
        timeout=60,
    )
