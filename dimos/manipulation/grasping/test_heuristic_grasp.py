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


def _block(yaw: float, size: tuple[float, float] = (0.04, 0.04)) -> np.ndarray:
    """A box's top and side surfaces, turned ``yaw`` about z, centred at (0.5, 0.1)."""
    rng = np.random.default_rng(0)
    local = rng.uniform(-0.5, 0.5, (400, 3)) * np.asarray([*size, 0.04])
    turn = np.asarray([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    xy = local[:, :2] @ turn.T + np.asarray([0.5, 0.1])
    return np.column_stack([xy, local[:, 2] + 0.83])


@pytest.mark.parametrize("yaw_deg", [0.0, 17.0, 30.0, 45.0, -38.0])
def test_square_block_jaws_close_across_its_faces_at_any_yaw(
    module: HeuristicGraspModule, yaw_deg: float
) -> None:
    proposals = module.propose_grasps(_cloud(_block(math.radians(yaw_deg))))

    assert len(proposals.candidates) == 1  # the two sides are the same width
    pose = proposals.candidates[0].pose
    assert (pose.position.x, pose.position.y) == pytest.approx((0.5, 0.1), abs=2e-3)
    jaw = pose.orientation.rotate_vector(Vector3(0.0, 1.0, 0.0))
    jaw_deg = math.degrees(math.atan2(jaw.y, jaw.x))
    # Square: either pair of faces; the jaw axis lies along a face normal, modulo 90 degrees.
    assert (jaw_deg - yaw_deg + 45.0) % 90.0 - 45.0 == pytest.approx(0.0, abs=2.0)


def test_narrow_block_gets_both_sides_when_the_jaws_span_the_long_one(
    module: HeuristicGraspModule,
) -> None:
    proposals = module.propose_grasps(_cloud(_block(math.radians(20.0), size=(0.07, 0.03))))

    first, second = proposals.candidates
    assert first.score > second.score
    jaws = [c.pose.orientation.rotate_vector(Vector3(0.0, 1.0, 0.0)) for c in (first, second)]
    # First across the 3 cm side, i.e. along the box's local y axis at 20 + 90 degrees.
    first_deg = math.degrees(math.atan2(jaws[0].y, jaws[0].x))
    assert (first_deg - 110.0 + 90.0) % 180.0 - 90.0 == pytest.approx(0.0, abs=2.0)
    assert abs(jaws[0].x * jaws[1].x + jaws[0].y * jaws[1].y) == pytest.approx(0.0, abs=1e-6)


def test_a_cloud_of_only_the_top_is_grasped_below_it(module: HeuristicGraspModule) -> None:
    top = _block(0.0)
    top[:, 2] = 0.85

    pose = module.propose_grasps(_cloud(top)).candidates[0].pose

    assert pose.position.z == pytest.approx(0.83)


def test_too_long_a_side_is_not_offered(module: HeuristicGraspModule) -> None:
    proposals = module.propose_grasps(_cloud(_block(0.0, size=(0.20, 0.04))))

    assert len(proposals.candidates) == 1


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
