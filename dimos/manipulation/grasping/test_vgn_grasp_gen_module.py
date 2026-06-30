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

import builtins
from collections.abc import Generator, Sequence
from dataclasses import dataclass
import os
from types import ModuleType
from typing import Protocol, cast

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.grasping.grasp_gen_spec import TSDFGraspGenSpec
from dimos.manipulation.grasping.vgn_grasp_gen_module import (
    VGNGraspGenModule,
    _load_vgn_network,
    _select_torch_device,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.grasping_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.grasping_msgs.TargetBounds import TargetBounds
from dimos.msgs.reconstruction_msgs.TSDFGrid import TSDFGrid
from dimos.spec.utils import spec_annotation_compliance, spec_structural_compliance


@dataclass(slots=True)
class _FakeGrasp:
    pose: np.ndarray
    width: float = 0.06


class _TSDFAdapterLike(Protocol):
    voxel_size: float

    def get_grid(self) -> np.ndarray: ...


class _StateWithTSDF(Protocol):
    tsdf: _TSDFAdapterLike


class _FakeDetector:
    def __init__(self, grasps: Sequence[_FakeGrasp], scores: Sequence[float]) -> None:
        self._grasps = grasps
        self._scores = scores
        self.seen_grid_shape: tuple[int, ...] | None = None
        self.seen_voxel_size: float | None = None

    def __call__(self, state: object) -> tuple[Sequence[_FakeGrasp], Sequence[float], float]:
        tsdf = cast("_StateWithTSDF", state).tsdf
        self.seen_grid_shape = tsdf.get_grid().shape
        self.seen_voxel_size = tsdf.voxel_size
        return self._grasps, self._scores, 0.0


class _FakeTorch:
    def __init__(self) -> None:
        self.loaded_path: object | None = None
        self.loaded_map_location: str | None = None
        self.empty_device: str | None = None
        self.cuda = _FakeCuda(self)

    def device(self, name: str) -> str:
        return name

    def load(self, path: object, *, map_location: str) -> dict[str, int]:
        self.loaded_path = path
        self.loaded_map_location = map_location
        return {"weight": 1}

    def empty(self, size: int, *, device: str) -> list[int]:
        self.empty_device = device
        return [0] * size


class _FakeCuda:
    def __init__(self, torch: _FakeTorch) -> None:
        self._torch = torch
        self.available = True
        self.count = 1
        self.synchronized = False

    def is_available(self) -> bool:
        return self.available

    def device_count(self) -> int:
        return self.count

    def synchronize(self) -> None:
        self.synchronized = True


class _FakeNetwork:
    def __init__(self) -> None:
        self.state_dict: dict[str, int] | None = None
        self.device: object | None = None
        self.evaluated = False

    def load_state_dict(self, state_dict: dict[str, int]) -> None:
        self.state_dict = state_dict

    def to(self, device: object) -> _FakeNetwork:
        self.device = device
        return self

    def eval(self) -> None:
        self.evaluated = True


def _tsdf_grid(
    *,
    shape: tuple[int, int, int, int] = (1, 40, 40, 40),
    frame_id: str = "world",
    ts: float = 10.0,
) -> TSDFGrid:
    return TSDFGrid(
        distances=np.ones(shape, dtype=np.float32),
        weights=np.ones(shape[1:], dtype=np.float32),
        voxel_size=0.01,
        truncation_distance=0.04,
        origin=Vector3(0.1, 0.2, 0.3),
        frame_id=frame_id,
        ts=ts,
    )


def _grasp_matrix(translation: tuple[float, float, float]) -> np.ndarray:
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, 3] = np.array(translation, dtype=np.float64)
    return matrix


@pytest.fixture(autouse=True)
def _stop_created_modules(mocker: MockerFixture) -> Generator[None, None, None]:
    modules: list[VGNGraspGenModule] = []
    original_init = VGNGraspGenModule.__init__

    def tracked_init(self: VGNGraspGenModule, **kwargs: object) -> None:
        original_init(self, **kwargs)
        modules.append(self)

    mocker.patch.object(VGNGraspGenModule, "__init__", tracked_init)
    yield
    for module in modules:
        module.stop()


def test_invalid_tsdf_shape_is_rejected() -> None:
    module = VGNGraspGenModule()

    with pytest.raises(ValueError, match="VGN expects TSDF shape"):
        module.generate_grasps_from_tsdf(_tsdf_grid(shape=(1, 20, 20, 20)))


def test_vgn_module_implements_tsdf_grasp_gen_spec() -> None:
    module = VGNGraspGenModule()

    assert spec_structural_compliance(module, TSDFGraspGenSpec)
    assert spec_annotation_compliance(module, TSDFGraspGenSpec)


def test_missing_model_path_fails_before_import(mocker) -> None:  # type: ignore[no-untyped-def]
    module = VGNGraspGenModule(model_path_env="DIMOS_TEST_VGN_MODEL_PATH")
    import_spy = mocker.spy(builtins, "__import__")
    mocker.patch.dict(os.environ, {"DIMOS_TEST_VGN_MODEL_PATH": ""}, clear=False)

    with pytest.raises(RuntimeError, match="VGN model path is required"):
        module.generate_grasps_from_tsdf(_tsdf_grid())

    imported_names = [call.args[0] for call in import_spy.call_args_list if call.args]
    assert "vgn.detection" not in imported_names


def test_missing_model_file_reports_path(tmp_path) -> None:  # type: ignore[no-untyped-def]
    missing_model = tmp_path / "missing" / "vgn_conv.pth"
    module = VGNGraspGenModule(model_path=str(missing_model))

    with pytest.raises(RuntimeError, match="VGN model file does not exist"):
        module.generate_grasps_from_tsdf(_tsdf_grid())


def test_ros_visualization_import_failure_uses_headless_detector(
    mocker: MockerFixture, tmp_path
) -> None:  # type: ignore[no-untyped-def]
    model_path = tmp_path / "vgn_conv.pth"
    model_path.write_bytes(b"checkpoint")
    module = VGNGraspGenModule(model_path=str(model_path))
    detector = _FakeDetector([], [])
    mocker.patch(
        "dimos.manipulation.grasping.vgn_grasp_gen_module._HeadlessVGNDetector",
        return_value=detector,
    )
    mocker.patch(
        "dimos.manipulation.grasping.vgn_grasp_gen_module.importlib.util.find_spec",
        return_value=object(),
    )
    original_import = builtins.__import__

    def fake_import(
        name: str,
        globals: dict[str, object] | None = None,
        locals: dict[str, object] | None = None,
        fromlist: tuple[str, ...] = (),
        level: int = 0,
    ) -> ModuleType:
        if name == "vgn.detection":
            raise ModuleNotFoundError("No module named 'sensor_msgs'", name="sensor_msgs")
        return original_import(name, globals, locals, fromlist, level)

    mocker.patch.object(builtins, "__import__", side_effect=fake_import)

    assert module._get_detector() is detector


def test_headless_loader_loads_checkpoint_on_cpu_before_moving_to_device(tmp_path) -> None:  # type: ignore[no-untyped-def]
    fake_torch = _FakeTorch()
    network = _FakeNetwork()
    model_path = tmp_path / "vgn_conv.pth"
    model_path.write_bytes(b"checkpoint")
    requested_models: list[str] = []

    def get_network(model_name: str) -> _FakeNetwork:
        requested_models.append(model_name)
        return network

    result = _load_vgn_network(fake_torch, get_network, model_path, "cuda:0")

    assert result is network
    assert requested_models == ["conv"]
    assert fake_torch.loaded_path == model_path
    assert fake_torch.loaded_map_location == "cpu"
    assert network.state_dict == {"weight": 1}
    assert network.device == "cuda:0"
    assert network.evaluated is True


def test_torch_device_selection_smoke_tests_cuda() -> None:
    fake_torch = _FakeTorch()

    assert _select_torch_device(fake_torch) == "cuda:0"
    assert fake_torch.empty_device == "cuda:0"
    assert fake_torch.cuda.synchronized is True


def test_torch_device_selection_falls_back_to_cpu_when_cuda_unusable() -> None:
    fake_torch = _FakeTorch()
    fake_torch.cuda.available = True
    fake_torch.cuda.count = 0

    assert _select_torch_device(fake_torch) == "cpu"


def test_fake_detector_converts_origin_and_publishes_outputs(mocker) -> None:  # type: ignore[no-untyped-def]
    module = VGNGraspGenModule(output_frame="world")
    detector = _FakeDetector([_FakeGrasp(_grasp_matrix((0.01, 0.02, 0.03)))], [0.9])
    mocker.patch.object(module, "_detector", detector)
    candidates_out: list[GraspCandidateArray] = []
    poses_out = []
    module.grasp_candidates.subscribe(candidates_out.append)
    module.grasp_poses.subscribe(poses_out.append)

    result = module.generate_grasps_from_tsdf(_tsdf_grid())

    assert result is not None
    assert detector.seen_grid_shape == (1, 40, 40, 40)
    assert detector.seen_voxel_size == pytest.approx(0.01)
    assert len(result) == 1
    candidate = result[0]
    assert candidate.id == "vgn-0"
    assert candidate.score == pytest.approx(0.9)
    assert candidate.jaw_width == pytest.approx(0.06)
    assert candidate.pose.position.x == pytest.approx(0.11)
    assert candidate.pose.position.y == pytest.approx(0.22)
    assert candidate.pose.position.z == pytest.approx(0.33)
    assert candidates_out == [result]
    assert len(poses_out) == 1
    assert len(poses_out[0].poses) == 1


def test_missing_world_transform_returns_none_and_does_not_publish(mocker) -> None:  # type: ignore[no-untyped-def]
    module = VGNGraspGenModule(output_frame="world")
    mocker.patch.object(
        module,
        "_detector",
        _FakeDetector([_FakeGrasp(_grasp_matrix((0.01, 0.02, 0.03)))], [0.9]),
    )
    get_transform = mocker.patch.object(module.tf, "get", return_value=None)
    candidates_out: list[GraspCandidateArray] = []
    module.grasp_candidates.subscribe(candidates_out.append)

    result = module.generate_grasps_from_tsdf(_tsdf_grid(frame_id="camera"))

    assert result is None
    assert candidates_out == []
    get_transform.assert_called_once_with("world", "camera", 10.0, 0.1)


def test_world_transform_is_applied(mocker) -> None:  # type: ignore[no-untyped-def]
    module = VGNGraspGenModule(output_frame="world")
    mocker.patch.object(
        module,
        "_detector",
        _FakeDetector([_FakeGrasp(_grasp_matrix((0.01, 0.02, 0.03)))], [0.9]),
    )
    transform = Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion([0.0, 0.0, 0.0, 1.0]),
        frame_id="world",
        child_frame_id="camera",
        ts=10.0,
    )
    mocker.patch.object(module.tf, "get", return_value=transform)

    result = module.generate_grasps_from_tsdf(_tsdf_grid(frame_id="camera"))

    assert result is not None
    assert result[0].pose.position.x == pytest.approx(1.11)
    assert result[0].pose.position.y == pytest.approx(2.22)
    assert result[0].pose.position.z == pytest.approx(3.33)


def test_generate_latest_without_tsdf_publishes_empty_candidates() -> None:
    module = VGNGraspGenModule(output_frame="world")
    candidates_out: list[GraspCandidateArray] = []
    module.grasp_candidates.subscribe(candidates_out.append)

    message = module.generate_latest_grasps()

    assert message == "No TSDF available for grasp generation"
    assert len(candidates_out) == 1
    assert len(candidates_out[0]) == 0
    assert candidates_out[0].frame_id == "world"


def test_target_bounds_without_latest_tsdf_clears_candidates_without_result() -> None:
    module = VGNGraspGenModule(output_frame="world")
    candidates_out: list[GraspCandidateArray] = []
    module.grasp_candidates.subscribe(candidates_out.append)

    result = module.generate_grasps_for_target_bounds(
        target_center=Vector3(0.2, 0.3, 0.4),
        target_size=Vector3(0.1, 0.1, 0.1),
        target_frame_id="world",
        target_ts=12.0,
    )

    assert result is None
    assert len(candidates_out) == 1
    assert len(candidates_out[0]) == 0
    assert candidates_out[0].frame_id == "world"


def test_target_mask_does_not_mutate_original_and_suppresses_outside_voxels() -> None:
    module = VGNGraspGenModule(output_frame="world")
    tsdf = _tsdf_grid()
    tsdf.distances.fill(0.25)
    assert tsdf.weights is not None
    original_distances = tsdf.distances.copy()
    original_weights = tsdf.weights.copy()
    bounds = TargetBounds(
        center=Vector3(0.1, 0.2, 0.3),
        size=Vector3(0.01, 0.01, 0.01),
        frame_id="world",
        ts=tsdf.ts,
    )

    masked, bounds_in_tsdf = module._target_masked_tsdf(tsdf, bounds, cushion_m=0.0)

    assert masked is not None
    assert bounds_in_tsdf is not None
    assert masked.weights is not None
    assert np.array_equal(tsdf.distances, original_distances)
    assert np.array_equal(tsdf.weights, original_weights)
    assert masked.distances[0, 0, 0, 0] == pytest.approx(0.25)
    assert masked.distances[0, 1, 0, 0] == pytest.approx(1.0)
    assert masked.weights[0, 0, 0] == pytest.approx(1.0)
    assert masked.weights[1, 0, 0] == pytest.approx(0.0)


def test_target_bounds_transform_failure_returns_none_and_clears(mocker: MockerFixture) -> None:
    module = VGNGraspGenModule(output_frame="world")
    module._latest_tsdf = _tsdf_grid(frame_id="tsdf")
    mocker.patch.object(module.tf, "get", return_value=None)
    candidates_out: list[GraspCandidateArray] = []
    module.grasp_candidates.subscribe(candidates_out.append)

    result = module.generate_grasps_for_target_bounds(
        target_center=Vector3(0.0, 0.0, 0.0),
        target_size=Vector3(0.1, 0.1, 0.1),
        target_frame_id="object",
        target_ts=12.0,
    )

    assert result is None
    assert len(candidates_out) == 1
    assert len(candidates_out[0]) == 0


def test_target_bounds_path_publishes_only_filtered_candidates(mocker: MockerFixture) -> None:
    module = VGNGraspGenModule(output_frame="world")
    module._latest_tsdf = _tsdf_grid()
    mocker.patch.object(
        module,
        "_detector",
        _FakeDetector(
            [
                _FakeGrasp(_grasp_matrix((0.0, 0.0, 0.0))),
                _FakeGrasp(_grasp_matrix((0.25, 0.25, 0.25))),
            ],
            [0.95, 0.9],
        ),
    )
    candidates_out: list[GraspCandidateArray] = []
    bounds_out: list[TargetBounds] = []
    masked_out: list[TSDFGrid] = []
    module.grasp_candidates.subscribe(candidates_out.append)
    module.grasp_target_bounds.subscribe(bounds_out.append)
    module.target_masked_tsdf.subscribe(masked_out.append)

    result = module.generate_grasps_for_target_bounds(
        target_center=Vector3(0.1, 0.2, 0.3),
        target_size=Vector3(0.04, 0.04, 0.04),
        target_frame_id="world",
        target_ts=10.0,
        cushion_m=0.0,
    )

    assert result is not None
    assert [candidate.id for candidate in result] == ["vgn-0"]
    assert candidates_out == [result]
    assert len(bounds_out) == 1
    assert len(masked_out) == 1
