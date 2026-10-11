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

"""Host contract tests; the isolated runtime is not imported here."""

from __future__ import annotations

import inspect
import os
from pathlib import Path
import subprocess
import sys
from typing import Any

import numpy as np
import pytest

from dimos.manipulation.grasping.grasp_gen_spec import (
    GraspGenSpec,
    LegacyGraspGenSpec,
    SceneGraspGenSpec,
)
from dimos.manipulation.grasping.grasp_gen_x.module import (
    GraspGenXConfig,
    GraspGenXModule,
    GripperBox,
    gripper_body_points,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header
from dimos.spec.utils import spec_annotation_compliance


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


def test_public_adapter_import_does_not_load_optional_runtime() -> None:
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import sys; "
                "import dimos.manipulation.grasping.grasp_gen_x.module; "
                "assert 'graspgenx' not in sys.modules; assert 'graspgenx_runtime.backend' not in sys.modules"
            ),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


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
def test_gripper_constraints_are_declared_by_fields(field: str, value: Any) -> None:
    gripper = config().gripper.model_dump()

    with pytest.raises(ValueError):
        config(gripper={**gripper, field: value})


@pytest.mark.parametrize("value", [0, -1, True, 2.5, "2.5", "-1"])
def test_candidate_limit_is_a_strict_positive_integer(value: Any) -> None:
    with pytest.raises(ValueError):
        config(max_candidates=value)


def test_counts_read_from_the_environment_are_numbers() -> None:
    assert config(max_candidates="100", num_samples=" 300 ").max_candidates == 100
    assert config(num_samples="300").num_samples == 300


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


def test_host_collection_excludes_the_nested_runtime_suite() -> None:
    # This is a new pytest controller, not an outer xdist worker. Give it its
    # own run ID so its watchdog cannot sweep the parent session's workers.
    env = {key: value for key, value in os.environ.items() if not key.startswith("PYTEST_XDIST_")}
    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "pytest",
            "--collect-only",
            "--no-cov",
            "-q",
            str(Path(__file__).parent),
        ],
        capture_output=True,
        text=True,
        timeout=60,
        env=env,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "test_public_adapter_import_does_not_load_optional_runtime" in result.stdout
    assert "test_runtime.py" not in result.stdout


def test_graspgenx_offers_both_grasp_specs() -> None:
    assert spec_annotation_compliance(GraspGenXModule, GraspGenSpec)
    assert spec_annotation_compliance(GraspGenXModule, SceneGraspGenSpec)


def test_gripper_body_points_cover_each_box_surface() -> None:
    box = GripperBox(extents=(0.04, 0.02, 0.1), offset=(0.01, -0.02, 0.05))
    spacing = 0.007
    points = gripper_body_points([box], spacing).astype(np.float64)

    local = points - np.asarray(box.offset)
    half = np.asarray(box.extents) / 2.0
    # Every point lies on a face.
    assert np.all(np.abs(local) <= half + 1e-6)
    assert np.all(np.any(np.isclose(np.abs(local), half, atol=1e-6), axis=1))
    # Every spot on the surface is within half a grid diagonal of a point.
    rng = np.random.default_rng(0)
    spots = rng.uniform(-half, half, size=(2000, 3))
    axis = rng.integers(0, 3, size=len(spots))
    spots[np.arange(len(spots)), axis] = half[axis] * rng.choice([-1.0, 1.0], size=len(spots))
    nearest = np.linalg.norm(spots[:, None, :] - local[None, :, :], axis=2).min(axis=1)
    assert nearest.max() <= spacing / np.sqrt(2.0) + 1e-9


def test_no_gripper_body_gives_no_points() -> None:
    assert gripper_body_points([]).shape == (0, 3)
    with pytest.raises(ValueError, match="spacing"):
        gripper_body_points([], spacing=0.0)


@pytest.mark.parametrize(
    "box",
    [
        {"extents": (0.0, 0.01, 0.01), "offset": (0.0, 0.0, 0.0)},
        {"extents": (0.01, 0.01), "offset": (0.0, 0.0, 0.0)},
        {"extents": (0.01, 0.01, 0.01), "offset": (0.0, 0.0, np.inf)},
    ],
)
def test_gripper_body_boxes_are_validated(box: dict[str, Any]) -> None:
    with pytest.raises(ValueError):
        config(gripper_body=(box,))
