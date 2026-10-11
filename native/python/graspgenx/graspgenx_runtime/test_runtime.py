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

"""Proposal behavior tests run in the isolated project's environment."""

from __future__ import annotations

import inspect
from typing import Any

import graspgenx_runtime.runtime as runtime_module
from graspgenx_runtime.runtime import GraspGenXRuntimeModule
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.experimental.isolated_python.bootstrap import validate_runtime
from dimos.manipulation.grasping.grasp_gen_x.module import (
    GraspGenXConfig,
    GraspGenXError,
    GraspGenXModule,
)
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def config(**overrides: Any) -> GraspGenXConfig:
    values: dict[str, Any] = {
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


def cloud(points: np.ndarray | None = None, frame_id: str = "camera") -> PointCloud2:
    xyz = np.zeros((1, 3), dtype=np.float32) if points is None else points
    return PointCloud2.from_numpy(xyz, frame_id=frame_id, timestamp=12.5)


BODY = ({"extents": (0.02, 0.02, 0.02), "offset": (0.0, 0.0, 0.0)},)


@pytest.fixture
def runtime(mocker: MockerFixture) -> Any:
    create_runtime = mocker.patch.object(runtime_module, "_create_runtime")
    instance = create_runtime.return_value
    instance.infer.return_value = (
        np.repeat(np.eye(4, dtype=np.float32)[None], 1, axis=0),
        np.asarray([0.5], dtype=np.float32),
    )
    return create_runtime


def test_start_is_synchronous_and_idempotent(runtime: Any) -> None:
    module = GraspGenXRuntimeModule(_isolated_python_runtime=True, **module_args())
    try:
        module.start()
        module.start()

        runtime.assert_called_once_with(module.config)
        assert len(module.propose_grasps(cloud())) == 1
    finally:
        module.stop()


def test_start_failure_is_explicit(runtime: Any) -> None:
    runtime.side_effect = RuntimeError("CUDA unavailable")
    module = GraspGenXRuntimeModule(_isolated_python_runtime=True, **module_args())
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
    module = GraspGenXRuntimeModule(_isolated_python_runtime=True, **module_args(cfg))
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
    module = GraspGenXRuntimeModule(_isolated_python_runtime=True, **module_args())
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
    module = GraspGenXRuntimeModule(_isolated_python_runtime=True, **module_args())
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
    module = GraspGenXRuntimeModule(_isolated_python_runtime=True, **module_args())
    try:
        module.start()
        with pytest.raises(ValueError):
            module.propose_grasps(cloud())
    finally:
        module.stop()


def test_inference_failure_is_wrapped(runtime: Any) -> None:
    runtime.return_value.infer.side_effect = RuntimeError("backend")
    module = GraspGenXRuntimeModule(_isolated_python_runtime=True, **module_args())
    try:
        module.start()
        with pytest.raises(GraspGenXError, match="inference"):
            module.propose_grasps(cloud())
    finally:
        module.stop()


def test_not_started_and_missing_metadata_are_rejected(runtime: Any) -> None:
    module = GraspGenXRuntimeModule(_isolated_python_runtime=True, **module_args())
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


def test_runtime_implements_the_host_contract() -> None:
    validate_runtime(GraspGenXModule, GraspGenXRuntimeModule)
    assert inspect.signature(GraspGenXModule.propose_grasps) == inspect.signature(
        GraspGenXRuntimeModule.propose_grasps
    )
    assert inspect.signature(GraspGenXModule.propose_grasps_in_scene) == inspect.signature(
        GraspGenXRuntimeModule.propose_grasps_in_scene
    )


def test_scene_check_drops_colliding_grasps_and_counts_them(runtime: Any) -> None:
    poses = np.repeat(np.eye(4, dtype=np.float32)[None], 3, axis=0)
    poses[:, 0, 3] = [1.0, 2.0, 3.0]
    runtime.return_value.infer.return_value = (poses, np.asarray([0.2, 0.9, 0.5]))
    runtime.return_value.collision_free.return_value = np.asarray([True, False, True])
    cfg = config(
        gripper_body=BODY,
        grasp_frame_to_tcp=(
            (1.0, 0.0, 0.0, 10.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
        ),
    )
    module = GraspGenXRuntimeModule(_isolated_python_runtime=True, **module_args(cfg))
    scene = np.asarray([[0.0, 0.0, -0.1], [0.1, 0.0, -0.1]], dtype=np.float32)
    try:
        module.start()
        result = module.propose_grasps_in_scene(cloud(), cloud(scene), 0.01)

        assert result.removed == 1
        assert [c.score for c in result.grasps] == pytest.approx([0.5, 0.2])
        assert [c.pose.position.x for c in result.grasps] == pytest.approx([13.0, 11.0])
        assert result.grasps.header.frame_id == "camera"
        # The check sees the model's own grasp poses, not the TCP poses.
        scene_arg, poses_arg, threshold = runtime.return_value.collision_free.call_args.args
        np.testing.assert_array_equal(scene_arg, scene)
        np.testing.assert_array_equal(poses_arg, poses)
        assert threshold == pytest.approx(0.01)
    finally:
        module.stop()


def test_an_empty_scene_drops_nothing(runtime: Any) -> None:
    module = GraspGenXRuntimeModule(
        _isolated_python_runtime=True, **module_args(config(gripper_body=BODY))
    )
    try:
        module.start()
        result = module.propose_grasps_in_scene(
            cloud(), cloud(np.empty((0, 3), dtype=np.float32)), 0.01
        )

        assert result.removed == 0
        assert len(result.grasps) == 1
        runtime.return_value.collision_free.assert_not_called()
    finally:
        module.stop()


@pytest.mark.parametrize(
    ("body", "scene", "threshold", "match"),
    [
        ((), cloud(), 0.01, "gripper_body"),
        (BODY, cloud(frame_id="world"), 0.01, "frame"),
        (BODY, cloud(np.array([[np.inf, 0.0, 0.0]], dtype=np.float32)), 0.01, "finite"),
        (BODY, cloud(), 0.0, "collision_threshold"),
    ],
)
def test_scene_check_rejects_bad_inputs(
    runtime: Any, body: Any, scene: PointCloud2, threshold: float, match: str
) -> None:
    module = GraspGenXRuntimeModule(
        _isolated_python_runtime=True, **module_args(config(gripper_body=body))
    )
    try:
        module.start()
        with pytest.raises(ValueError, match=match):
            module.propose_grasps_in_scene(cloud(), scene, threshold)
        runtime.return_value.infer.assert_not_called()
    finally:
        module.stop()


def test_scene_check_failure_is_wrapped(runtime: Any) -> None:
    runtime.return_value.collision_free.side_effect = RuntimeError("CUDA out of memory")
    module = GraspGenXRuntimeModule(
        _isolated_python_runtime=True, **module_args(config(gripper_body=BODY))
    )
    try:
        module.start()
        with pytest.raises(GraspGenXError, match="collision check"):
            module.propose_grasps_in_scene(cloud(), cloud(), 0.01)
    finally:
        module.stop()


def test_proposals_sample_the_configured_count(runtime: Any) -> None:
    module = GraspGenXRuntimeModule(
        _isolated_python_runtime=True, **module_args(config(num_samples=500))
    )
    try:
        module.start()
        module.propose_grasps(cloud())

        assert runtime.return_value.infer.call_args.args[1] == 500
    finally:
        module.stop()


def test_scene_check_runs_on_every_sample_before_keeping_the_best(runtime: Any) -> None:
    poses = np.repeat(np.eye(4, dtype=np.float32)[None], 5, axis=0)
    poses[:, 0, 3] = [1.0, 2.0, 3.0, 4.0, 5.0]
    runtime.return_value.infer.return_value = (poses, np.asarray([0.9, 0.8, 0.7, 0.6, 0.5]))
    # The two best grasps hit the scene; three lower ones are clear.
    runtime.return_value.collision_free.return_value = np.asarray([False, False, True, True, True])
    module = GraspGenXRuntimeModule(
        _isolated_python_runtime=True,
        **module_args(config(gripper_body=BODY, max_candidates=2)),
    )
    try:
        module.start()
        result = module.propose_grasps_in_scene(cloud(), cloud(), 0.01)

        _, checked, _ = runtime.return_value.collision_free.call_args.args
        assert len(checked) == 5
        assert [c.score for c in result.grasps] == pytest.approx([0.7, 0.6])
        assert result.removed == 2
        assert result.sampled == 5
        # Keeping the best two first and checking only those would leave none.
        assert result.clear_in_top == 0
        assert result.infer_s >= 0.0
        assert result.check_s >= 0.0
    finally:
        module.stop()


def test_scene_check_takes_a_sample_count_per_call(runtime: Any) -> None:
    runtime.return_value.collision_free.return_value = np.asarray([True])
    module = GraspGenXRuntimeModule(
        _isolated_python_runtime=True,
        **module_args(config(gripper_body=BODY, num_samples=200)),
    )
    try:
        module.start()
        module.propose_grasps_in_scene(cloud(), cloud(), 0.01, num_samples=1000)
        assert runtime.return_value.infer.call_args.args[1] == 1000
        module.propose_grasps_in_scene(cloud(), cloud(), 0.01)
        assert runtime.return_value.infer.call_args.args[1] == 200
    finally:
        module.stop()


@pytest.mark.parametrize("num_samples", [0, -5, True, 2.5])
def test_scene_check_rejects_a_bad_sample_count(runtime: Any, num_samples: Any) -> None:
    module = GraspGenXRuntimeModule(
        _isolated_python_runtime=True, **module_args(config(gripper_body=BODY))
    )
    try:
        module.start()
        with pytest.raises(ValueError, match="num_samples"):
            module.propose_grasps_in_scene(cloud(), cloud(), 0.01, num_samples=num_samples)
        runtime.return_value.infer.assert_not_called()
    finally:
        module.stop()
