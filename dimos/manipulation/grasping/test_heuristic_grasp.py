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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.heuristic_grasp import HeuristicGraspModule
from dimos.msgs.geometry import pose_matrix
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.spec.utils import spec_annotation_compliance


def _cloud(
    points: np.ndarray, *, frame_id: str = "world", stamp: Time | None = None
) -> PointCloud2:
    return pointcloud_from_xyz(
        points.astype(np.float32), header=Header(frame_id=frame_id, stamp=stamp or Time(sec=1))
    )


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
    assert proposals.header.stamp == Time(sec=1)
    pose = proposals.candidates[0].pose
    assert pose.position.x == pytest.approx(0.0)
    assert pose.position.y == pytest.approx(0.0)
    assert pose.position.z == pytest.approx(0.15)
    approach = pose_matrix(pose)[:3, 2]
    assert approach[2] == pytest.approx(-1.0)
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

    jaw_axis = pose_matrix(proposals.candidates[0].pose)[:3, 1]
    assert abs(jaw_axis[0]) == pytest.approx(1.0)
    assert jaw_axis[1] == pytest.approx(0.0, abs=1e-6)
    assert jaw_axis[2] == pytest.approx(0.0, abs=1e-6)


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
    "points, frame_id, stamp, error",
    [
        (np.zeros((2, 3)), "world", Time(), "at least three"),
        (np.asarray([[0.0, 0.0, math.nan]] * 3), "world", Time(), "finite"),
        (np.zeros((3, 3)), "", Time(), "frame_id"),
        (np.zeros((3, 3)), "world", Time(nanosec=1000000000), "nanosec"),
    ],
)
def test_heuristic_grasp_rejects_invalid_pointclouds(module, points, frame_id, stamp, error):
    with pytest.raises(ValueError, match=error):
        module.propose_grasps(_cloud(points, frame_id=frame_id, stamp=stamp))


@pytest.mark.parametrize("stamp", [Time(), Time(sec=1700000000, nanosec=123456789)])
def test_heuristic_grasp_preserves_exact_header_without_aliasing(module, stamp):
    cloud = _cloud(np.zeros((3, 3)), stamp=stamp)
    proposals = module.propose_grasps(cloud)
    assert proposals.header == cloud.header
    proposals.header.frame_id = "changed"
    assert cloud.header.frame_id == "world"
