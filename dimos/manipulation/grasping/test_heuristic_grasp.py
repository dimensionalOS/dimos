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

from collections.abc import Iterator
import math

import numpy as np
import pytest

from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import spec_annotation_compliance


def _cloud(
    points: np.ndarray, *, frame_id: str = "world", timestamp: float | None = 1.0
) -> PointCloud2:
    return PointCloud2.from_numpy(points.astype(np.float32), frame_id=frame_id, timestamp=timestamp)


@pytest.fixture
def module() -> Iterator[HeuristicGraspModule]:
    instance = HeuristicGraspModule()
    yield instance
    instance.stop()


def test_heuristic_grasp_implements_grasp_provider_spec(module: HeuristicGraspModule) -> None:
    assert spec_annotation_compliance(module, GraspGenSpec)


def test_heuristic_grasp_proposes_centered_top_down_pose(module: HeuristicGraspModule) -> None:
    proposals = module.propose_grasps(
        _cloud(
            np.asarray(
                [
                    [-0.10, -0.02, 0.10],
                    [-0.10, 0.02, 0.10],
                    [0.10, -0.02, 0.20],
                    [0.10, 0.02, 0.20],
                ]
            )
        )
    )

    assert len(proposals.candidates) == 1
    assert proposals.header.frame_id == "world"
    assert proposals.header.timestamp == pytest.approx(1.0)
    pose = proposals.candidates[0].pose
    assert pose.position.x == pytest.approx(0.0)
    assert pose.position.y == pytest.approx(0.0)
    assert pose.position.z == pytest.approx(0.15)
    approach = pose.orientation.rotate_vector(Vector3(0.0, 0.0, 1.0))
    assert approach.z == pytest.approx(-1.0)
    assert proposals.candidates[0].score == pytest.approx(1.0)


def test_heuristic_grasp_centres_a_tall_object_on_its_top(module: HeuristicGraspModule) -> None:
    # A can seen from one side: a round lid on the axis, and the near face
    # below it, offset toward the camera.
    rng = np.random.default_rng(0)
    angle = rng.uniform(0.0, 2.0 * np.pi, 300)
    radius = np.sqrt(rng.uniform(0.0, 1.0, 300)) * 0.033
    lid = np.column_stack(
        [0.3 + radius * np.cos(angle), -0.2 + radius * np.sin(angle), np.full(300, 0.10)]
    )
    face_angle = rng.uniform(-np.pi / 2, np.pi / 2, 900)
    face = np.column_stack(
        [
            0.3 - 0.033 * np.cos(face_angle),
            -0.2 + 0.033 * np.sin(face_angle),
            rng.uniform(0.0, 0.07, 900),
        ]
    )
    candidates = module.propose_grasps(_cloud(np.vstack([lid, face])))

    position = candidates.candidates[0].pose.position
    assert position.x == pytest.approx(0.3, abs=0.005)
    assert position.y == pytest.approx(-0.2, abs=0.005)


def test_heuristic_grasp_offers_every_yaw_for_an_object_that_fits_both_ways() -> None:
    module = HeuristicGraspModule(yaw_candidates=8)
    try:
        # a 6 x 7 cm slab: a clear narrow axis, yet it fits the 9 cm jaws either way
        rng = np.random.default_rng(1)
        pts = np.column_stack(
            [
                rng.uniform(-0.03, 0.03, 500),
                rng.uniform(-0.035, 0.035, 500),
                rng.uniform(0.0, 0.01, 500),
            ]
        )
        candidates = module.propose_grasps(_cloud(pts))
        assert len(candidates.candidates) == 8
        # a 6 x 19 cm bar only fits across its narrow axis
        pts = np.column_stack(
            [
                rng.uniform(-0.03, 0.03, 500),
                rng.uniform(-0.095, 0.095, 500),
                rng.uniform(0.0, 0.01, 500),
            ]
        )
        assert len(module.propose_grasps(_cloud(pts)).candidates) == 2
    finally:
        module.stop()


def test_heuristic_grasp_aligns_jaw_axis_with_narrow_axis(module: HeuristicGraspModule) -> None:
    proposals = module.propose_grasps(
        _cloud(
            np.asarray(
                [
                    [-0.02, -0.10, 0.10],
                    [0.02, -0.10, 0.10],
                    [-0.02, 0.10, 0.20],
                    [0.02, 0.10, 0.20],
                ]
            )
        )
    )

    jaw_axis = proposals.candidates[0].pose.orientation.rotate_vector(Vector3(0.0, 1.0, 0.0))
    assert abs(jaw_axis.x) == pytest.approx(1.0)
    assert jaw_axis.y == pytest.approx(0.0, abs=1e-6)
    assert jaw_axis.z == pytest.approx(0.0, abs=1e-6)


def test_heuristic_grasp_canonicalizes_pca_eigenvector_sign(
    module: HeuristicGraspModule, monkeypatch: pytest.MonkeyPatch
) -> None:
    values = np.asarray([1.0, 4.0])
    same_axis = np.asarray([[1.0, 0.0], [0.0, 1.0]])
    opposite_axis = np.asarray([[-1.0, 0.0], [0.0, 1.0]])

    monkeypatch.setattr(np.linalg, "eigh", lambda _: (values, same_axis))
    positive_yaw = module._narrow_axis_yaw(np.zeros((3, 2), dtype=np.float32))
    monkeypatch.setattr(np.linalg, "eigh", lambda _: (values, opposite_axis))
    negative_yaw = module._narrow_axis_yaw(np.zeros((3, 2), dtype=np.float32))

    assert negative_yaw == pytest.approx(positive_yaw)


@pytest.mark.parametrize(
    "points, frame_id, timestamp, error",
    [
        (
            np.asarray([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]]),
            "world",
            1.0,
            "at least three",
        ),
        (np.asarray([[0.0, 0.0, math.nan]] * 3), "world", 1.0, "finite"),
        (np.zeros((3, 3)), "", 1.0, "frame_id"),
        (np.zeros((3, 3)), "world", None, "timestamp"),
        (np.zeros((3, 3)), "world", math.nan, "finite timestamp"),
        (np.zeros((3, 3)), "world", math.inf, "finite timestamp"),
    ],
)
def test_heuristic_grasp_rejects_invalid_pointclouds(
    module: HeuristicGraspModule,
    points: np.ndarray,
    frame_id: str,
    timestamp: float | None,
    error: str,
) -> None:
    with pytest.raises(ValueError, match=error):
        module.propose_grasps(_cloud(points, frame_id=frame_id, timestamp=timestamp))
