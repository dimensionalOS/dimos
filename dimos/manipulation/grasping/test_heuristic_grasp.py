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
from typing import Any

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


def _box(x: tuple[float, float], y: tuple[float, float], z: tuple[float, float]) -> np.ndarray:
    """Points filling an axis-aligned box on a 5 mm grid, bounds in metres."""
    axes = [np.arange(low, high + 1e-9, 0.005) for low, high in (x, y, z)]
    return np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)


# Finger pad offsets like the xArm Gripper's, metres; no palm limit.
PADS = {"pad_behind_tcp": 0.0167, "fingertip_reach": 0.0031}
CLEARANCE = 0.005


def _grasp_z(points: np.ndarray, **config: Any) -> float:
    """Tool tip height (metres) of the module's grasp on some points."""
    instance = HeuristicGraspModule(**config)
    try:
        return float(instance.propose_grasps(_cloud(points)).candidates[0].pose.position.z)
    finally:
        instance.stop()


def test_the_closed_pads_land_at_mid_height_of_a_tall_box() -> None:
    """A 12 cm box: the tool tip goes 1.67 cm below mid-height, so the pads are at 6 cm."""
    z = _grasp_z(_box((-0.02, 0.02), (-0.02, 0.02), (0.0, 0.12)), **PADS)

    assert z + PADS["pad_behind_tcp"] == pytest.approx(0.06, abs=1e-6)


def test_on_a_short_box_the_fingertips_stay_clear_of_the_table_and_the_pads_still_hold_it() -> None:
    """A box 3 cm tall, like a tuna can, on a table at z = 0. Pads at its mid-height
    would put the tool tip 2 mm under the table."""
    tcp = _grasp_z(_box((-0.03, 0.03), (-0.02, 0.02), (0.0, 0.03)), **PADS)

    assert tcp - PADS["fingertip_reach"] == pytest.approx(CLEARANCE)
    assert 0.0 < tcp + PADS["pad_behind_tcp"] < 0.03


def test_on_a_box_taller_than_the_fingers_the_palm_stays_above_its_top() -> None:
    points = _box((-0.02, 0.02), (-0.02, 0.02), (0.0, 0.20))

    z = _grasp_z(points, palm_behind_tcp=0.05, **PADS)

    top = np.quantile(points[:, 2].astype(np.float32), 0.95)
    assert z + 0.05 == pytest.approx(top + CLEARANCE, abs=1e-6)


def test_the_module_takes_the_pad_offsets_and_the_switch() -> None:
    points = _box((-0.02, 0.02), (-0.02, 0.02), (0.0, 0.12))

    assert _grasp_z(points, pad_behind_tcp=0.0167) == pytest.approx(0.0433)
    assert _grasp_z(points, pad_behind_tcp=0.0167, heuristic_aim_pads=False) == pytest.approx(0.06)
