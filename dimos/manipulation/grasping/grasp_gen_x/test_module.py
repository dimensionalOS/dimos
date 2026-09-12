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
from pathlib import Path
import subprocess
import sys
from typing import Any

import numpy as np
import pytest

from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec, LegacyGraspGenSpec
from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.std_msgs.Header import Header


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


@pytest.mark.parametrize("value", [0, -1, True])
def test_candidate_limit_is_a_strict_positive_integer(value: Any) -> None:
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


def test_host_collection_excludes_the_nested_runtime_suite() -> None:
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
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "test_public_adapter_import_does_not_load_optional_runtime" in result.stdout
    assert "test_runtime.py" not in result.stdout
