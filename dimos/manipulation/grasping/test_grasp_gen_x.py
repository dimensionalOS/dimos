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

"""Hermetic tests for the import-safe GraspGenX adapter."""

from __future__ import annotations

import inspect
import subprocess
import sys
from typing import Any

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec, LegacyGraspGenSpec
import dimos.manipulation.grasping.grasp_gen_x as grasp_gen_x
from dimos.manipulation.grasping.grasp_gen_x import (
    GraspGenXConfig,
    GraspGenXError,
    GraspGenXModule,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header


def config(**overrides: object) -> GraspGenXConfig:
    values: dict[str, object] = {
        "gripper": {
            "extents_open": (0.1, 0.1, 0.1),
            "offset_open": (0.0, 0.0, 0.0),
            "extents_half_open": (0.1, 0.1, 0.1),
            "offset_half_open": (0.0, 0.0, 0.0),
            "fingertip_depth": 0.1,
        },
    }
    values.update(overrides)
    return GraspGenXConfig(**values)  # type: ignore[arg-type]


def module_args(value: GraspGenXConfig | None = None) -> dict[str, Any]:
    return (value or config()).model_dump(exclude={"rpc_transport", "tf_transport", "g"})


def cloud(points: np.ndarray | None = None) -> PointCloud2:
    xyz = np.zeros((1, 3), dtype=np.float32) if points is None else points
    return PointCloud2.from_numpy(xyz, frame_id="camera", timestamp=12.5)


def test_public_adapter_import_does_not_load_optional_runtime() -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys; "
                "import dimos.manipulation.grasping.grasp_gen_x; "
                "assert 'dimos.manipulation.grasping.grasp_gen_x_runtime' not in sys.modules"
            ),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


@pytest.fixture
def runtime(mocker: MockerFixture) -> Any:
    create_runtime = mocker.patch.object(grasp_gen_x, "_create_runtime")
    instance = create_runtime.return_value
    instance.infer.return_value = (
        np.repeat(np.eye(4, dtype=np.float32)[None], 1, axis=0),
        np.asarray([0.5], dtype=np.float32),
    )
    return create_runtime


def test_messages_round_trip_empty_and_score() -> None:
    value = GraspCandidateArray(Header(3.0, "camera"), [GraspCandidate(Pose(1, 2, 3), 0.25)])
    decoded = GraspCandidateArray.decode(value.encode())

    assert decoded.header.frame_id == "camera"
    assert decoded.header.timestamp == pytest.approx(3.0)
    assert decoded.candidates[0].score == pytest.approx(0.25)
    assert (
        GraspCandidateArray.decode(
            GraspCandidateArray(Header(3.0, "camera"), []).encode()
        ).candidates
        == []
    )


def test_ranked_spec_is_canonical_during_legacy_contract_transition() -> None:
    legacy_signature = inspect.signature(LegacyGraspGenSpec.generate_grasps)
    signature = inspect.signature(GraspGenSpec.propose_grasps)

    assert list(legacy_signature.parameters) == [
        "self",
        "pointcloud",
        "scene_pointcloud",
    ]
    assert list(signature.parameters) == ["self", "object_pointcloud"]
    assert signature.parameters["object_pointcloud"].annotation.__name__ == "PointCloud2"
    assert signature.return_annotation is GraspCandidateArray


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("family", "unsupported"),
        ("extents_open", (0.1, 0.1)),
        ("extents_open", (0.0, 0.1, 0.1)),
        ("extents_open", (0.6, 0.1, 0.1)),
        ("offset_open", (0.6, 0.0, 0.0)),
        ("offset_open", (np.nan, 0.0, 0.0)),
        ("fingertip_depth", 0.0),
    ],
)
def test_gripper_constraints_are_declared_by_fields(field: str, value: object) -> None:
    gripper = config().gripper.model_dump()

    with pytest.raises(ValueError):
        config(gripper={**gripper, field: value})


@pytest.mark.parametrize("value", [0, -1, True])
def test_candidate_limit_is_a_strict_positive_integer(value: object) -> None:
    with pytest.raises(ValueError):
        config(max_candidates=value)


def test_rigid_transform_relational_validation() -> None:
    with pytest.raises(ValueError, match="orthonormal"):
        config(
            grasp_frame_to_tcp=(
                (2.0, 0.0, 0.0, 0.0),
                (0.0, 1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )
        )


def test_start_is_synchronous_and_idempotent(runtime: Any) -> None:
    module = GraspGenXModule(**module_args())
    try:
        module.start()
        module.start()

        runtime.assert_called_once_with(module.config)
        assert len(module.propose_grasps(cloud())) == 1
    finally:
        module.stop()


def test_start_failure_is_explicit(runtime: Any) -> None:
    runtime.side_effect = RuntimeError("CUDA unavailable")
    module = GraspGenXModule(**module_args())
    try:
        with pytest.raises(GraspGenXError, match="initialize"):
            module.start()
    finally:
        module.stop()


def test_adapter_sorts_stably_truncates_and_applies_tcp_transform(runtime: Any) -> None:
    poses = np.repeat(np.eye(4, dtype=np.float32)[None], 3, axis=0)
    poses[:, 0, 3] = [1.0, 2.0, 3.0]
    runtime.return_value.infer.return_value = (
        poses,
        np.asarray([0.5, 0.5, 0.9], dtype=np.float32),
    )
    cfg = config(
        max_candidates=2,
        grasp_frame_to_tcp=(
            (0.0, -1.0, 0.0, 10.0),
            (1.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        ),
    )
    module = GraspGenXModule(**module_args(cfg))
    try:
        module.start()
        result = module.propose_grasps(cloud())

        assert [candidate.score for candidate in result] == pytest.approx([0.9, 0.5])
        assert [candidate.pose.position.x for candidate in result] == pytest.approx([13.0, 11.0])
        assert result.header.frame_id == "camera"
        assert result.header.timestamp == pytest.approx(12.5)
    finally:
        module.stop()


def test_empty_backend_result_preserves_input_header(runtime: Any) -> None:
    runtime.return_value.infer.return_value = (
        np.empty((0, 4, 4), dtype=np.float32),
        np.empty(0, dtype=np.float32),
    )
    module = GraspGenXModule(**module_args())
    try:
        module.start()
        result = module.propose_grasps(cloud())

        assert result.header.frame_id == "camera"
        assert result.header.timestamp == pytest.approx(12.5)
        assert result.candidates == []
    finally:
        module.stop()


@pytest.mark.parametrize(
    "points",
    [
        np.array([[np.nan, 0.0, 0.0]], dtype=np.float32),
        np.empty((0, 3), dtype=np.float32),
        np.zeros((2, 2), dtype=np.float32),
    ],
)
def test_invalid_cloud_points_are_rejected(runtime: Any, points: np.ndarray) -> None:
    module = GraspGenXModule(**module_args())
    try:
        module.start()
        with pytest.raises(ValueError, match="pointcloud|XYZ"):
            module.propose_grasps(cloud(points))
        runtime.return_value.infer.assert_not_called()
    finally:
        module.stop()


@pytest.mark.parametrize(
    "backend",
    [
        (np.ones((2, 4, 4)), np.ones(1)),
        (np.full((1, 4, 4), np.nan), np.ones(1)),
        (np.ones((1, 4, 4)), np.array([np.inf])),
    ],
)
def test_invalid_backend_outputs_are_rejected(
    runtime: Any, backend: tuple[np.ndarray, np.ndarray]
) -> None:
    runtime.return_value.infer.return_value = backend
    module = GraspGenXModule(**module_args())
    try:
        module.start()
        with pytest.raises(ValueError):
            module.propose_grasps(cloud())
    finally:
        module.stop()


def test_inference_failure_is_wrapped(runtime: Any) -> None:
    runtime.return_value.infer.side_effect = RuntimeError("backend")
    module = GraspGenXModule(**module_args())
    try:
        module.start()
        with pytest.raises(GraspGenXError, match="inference"):
            module.propose_grasps(cloud())
    finally:
        module.stop()


def test_not_started_and_missing_metadata_are_rejected(runtime: Any) -> None:
    module = GraspGenXModule(**module_args())
    missing_frame = cloud()
    missing_frame.frame_id = ""
    missing_timestamp = cloud()
    missing_timestamp.ts = None
    try:
        with pytest.raises(GraspGenXError, match="not been started"):
            module.propose_grasps(cloud())
        module.start()
        with pytest.raises(ValueError, match="frame_id"):
            module.propose_grasps(missing_frame)
        with pytest.raises(ValueError, match="timestamp"):
            module.propose_grasps(missing_timestamp)
    finally:
        module.stop()
