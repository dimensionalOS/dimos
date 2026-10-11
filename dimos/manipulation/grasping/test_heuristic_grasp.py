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
from dimos.manipulation.grasping.heuristic_grasp import (
    HeuristicGraspConfig,
    HeuristicGraspModule,
    _narrow_axis_yaw,
    heuristic_grasps,
)
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
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    values = np.asarray([1.0, 4.0])
    same_axis = np.asarray([[1.0, 0.0], [0.0, 1.0]])
    opposite_axis = np.asarray([[-1.0, 0.0], [0.0, 1.0]])

    monkeypatch.setattr(np.linalg, "eigh", lambda _: (values, same_axis))
    positive_yaw = _narrow_axis_yaw(np.zeros((3, 2), dtype=np.float32))
    monkeypatch.setattr(np.linalg, "eigh", lambda _: (values, opposite_axis))
    negative_yaw = _narrow_axis_yaw(np.zeros((3, 2), dtype=np.float32))

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


def _jaw_axis(candidate: Any) -> Vector3:
    return candidate.pose.orientation.rotate_vector(Vector3(0.0, 1.0, 0.0))


def test_a_long_object_gets_a_grasp_in_each_half(module: HeuristicGraspModule) -> None:
    """A 20 cm bar, 4 cm wide: too long to turn the jaws across, so no turned grasp."""
    proposals = module.propose_grasps(_cloud(_box((-0.10, 0.10), (-0.02, 0.02), (0.0, 0.04))))

    centre, *halves = proposals.candidates
    assert [c.score for c in proposals.candidates] == pytest.approx([1.0, 0.8, 0.7])
    assert sorted(c.pose.position.x for c in halves) == pytest.approx([-0.05, 0.05], abs=0.006)
    for candidate in proposals.candidates:
        assert candidate.pose.position.y == pytest.approx(0.0, abs=1e-6)
        assert candidate.pose.position.z == pytest.approx(0.02, abs=0.003)
        # Every grasp closes across the bar's width.
        assert abs(_jaw_axis(candidate).y) == pytest.approx(1.0)


def test_a_half_grasp_sits_on_its_own_half(module: HeuristicGraspModule) -> None:
    """A bar whose right half is 10 cm tall: each half is grasped halfway up itself."""
    points = np.vstack(
        [
            _box((-0.10, -0.005), (-0.02, 0.02), (0.0, 0.04)),
            _box((0.0, 0.10), (-0.02, 0.02), (0.0, 0.10)),
        ]
    )

    proposals = module.propose_grasps(_cloud(points))

    tall, short = proposals.candidates[1:3]
    assert tall.pose.position.x > 0.0
    assert tall.pose.position.z == pytest.approx(0.05, abs=0.006)
    assert short.pose.position.x < 0.0
    # Below the short half's 4 cm top, where a grasp at the whole bar's height would miss.
    assert 0.01 < short.pose.position.z < 0.035


def test_a_half_wider_than_the_jaws_is_not_grasped(module: HeuristicGraspModule) -> None:
    """A T shape: the 12 cm wide crossbar does not fit in the jaws, the 4 cm stem does."""
    points = np.vstack(
        [
            _box((-0.15, -0.005), (-0.02, 0.02), (0.0, 0.04)),
            _box((0.0, 0.03), (-0.06, 0.06), (0.0, 0.04)),
        ]
    )

    proposals = module.propose_grasps(_cloud(points))

    assert [c.score for c in proposals.candidates] == pytest.approx([1.0, 0.8])
    assert proposals.candidates[1].pose.position.x < 0.0


def test_a_small_object_also_gets_the_grasp_turned_a_quarter_turn(
    module: HeuristicGraspModule,
) -> None:
    """A 6 cm by 4 cm block fits in the jaws either way, and is too short to split."""
    proposals = module.propose_grasps(_cloud(_box((-0.03, 0.03), (-0.02, 0.02), (0.0, 0.05))))

    centre, turned = proposals.candidates
    assert turned.score == pytest.approx(0.5)
    assert turned.pose.position == centre.pose.position
    assert abs(_jaw_axis(centre).y) == pytest.approx(1.0)
    assert abs(_jaw_axis(turned).x) == pytest.approx(1.0)


def test_the_grasp_width_and_split_length_are_configurable() -> None:
    module = HeuristicGraspModule(max_grasp_width=0.25, min_split_length=0.5)
    try:
        proposals = module.propose_grasps(_cloud(_box((-0.10, 0.10), (-0.02, 0.02), (0.0, 0.04))))
    finally:
        module.stop()

    # Too short to split at 0.5 m, but a 20 cm length now fits across the jaws.
    assert [c.score for c in proposals.candidates] == pytest.approx([1.0, 0.5])


def test_a_block_too_wide_for_the_jaws_with_clearance_gets_no_turned_grasp(
    module: HeuristicGraspModule,
) -> None:
    """9 cm long, more than the 8.5 cm opening. Spread evenly, its points' middle
    90% spans only 8.1 cm, so the measure must reach the ends."""
    rng = np.random.default_rng(0)
    points = rng.uniform((-0.045, -0.02, 0.0), (0.045, 0.02, 0.05), size=(4000, 3))

    proposals = module.propose_grasps(_cloud(points))

    assert [c.score for c in proposals.candidates] == pytest.approx([1.0])


def test_a_square_turned_45_degrees_gets_no_grasp_across_its_diagonal(
    module: HeuristicGraspModule,
) -> None:
    """A 7 cm square on its corner measures 9.9 cm along the world axes."""
    square = _box((-0.035, 0.035), (-0.035, 0.035), (0.0, 0.05))
    c, s = math.cos(math.pi / 4.0), math.sin(math.pi / 4.0)
    turned = square @ np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])

    proposals = module.propose_grasps(_cloud(turned))

    assert [c.score for c in proposals.candidates] == pytest.approx([1.0])


def test_a_round_footprint_gets_no_turned_grasp(module: HeuristicGraspModule) -> None:
    """Turned a quarter turn, the jaws would hold a 6 cm can the same way."""
    angles = np.linspace(0.0, 2.0 * math.pi, 72, endpoint=False)
    rim = np.stack([0.03 * np.cos(angles), 0.03 * np.sin(angles)], axis=-1)
    points = np.vstack([np.column_stack([rim, np.full(len(rim), z)]) for z in (0.0, 0.05, 0.1)])

    proposals = module.propose_grasps(_cloud(points))

    assert [c.score for c in proposals.candidates] == pytest.approx([1.0])


def test_the_plain_function_gives_the_module_grasps() -> None:
    cloud = _cloud(_box((-0.10, 0.10), (-0.02, 0.02), (0.0, 0.04)))
    module = HeuristicGraspModule(max_grasp_width=0.25, min_split_length=0.15)
    try:
        expected = module.propose_grasps(cloud)
    finally:
        module.stop()

    proposals = heuristic_grasps(
        cloud, HeuristicGraspConfig(max_grasp_width=0.25, min_split_length=0.15)
    )

    assert proposals.header.frame_id == expected.header.frame_id
    assert proposals.header.timestamp == expected.header.timestamp
    assert [(c.pose, c.score) for c in proposals.candidates] == [
        (c.pose, c.score) for c in expected.candidates
    ]


# Finger pad offsets like the xArm Gripper's, metres; no palm limit.
PADS = HeuristicGraspConfig(pad_behind_tcp=0.0167, fingertip_reach=0.0031)


def test_the_closed_pads_land_at_mid_height_of_a_tall_box() -> None:
    """A 12 cm box: the tool tip goes 1.67 cm below mid-height, so the pads are at 6 cm."""
    proposals = heuristic_grasps(_cloud(_box((-0.02, 0.02), (-0.02, 0.02), (0.0, 0.12))), PADS)

    pose = proposals.candidates[0].pose
    assert pose.position.z + PADS.pad_behind_tcp == pytest.approx(0.06, abs=1e-6)


def test_on_a_short_box_the_fingertips_stay_clear_of_the_table_and_the_pads_still_hold_it() -> None:
    """A box 3 cm tall, like a tuna can, on a table at z = 0. Pads at its mid-height
    would put the tool tip 2 mm under the table."""
    proposals = heuristic_grasps(_cloud(_box((-0.03, 0.03), (-0.02, 0.02), (0.0, 0.03))), PADS)

    for candidate in proposals.candidates:
        tcp = candidate.pose.position.z
        assert tcp - PADS.fingertip_reach == pytest.approx(PADS.gripper_clearance)
        assert 0.0 < tcp + PADS.pad_behind_tcp < 0.03


def test_on_a_box_taller_than_the_fingers_the_palm_stays_above_its_top() -> None:
    config = PADS.model_copy(update={"palm_behind_tcp": 0.05})
    points = _box((-0.02, 0.02), (-0.02, 0.02), (0.0, 0.20))

    proposals = heuristic_grasps(_cloud(points), config)

    top = np.quantile(points[:, 2].astype(np.float32), 0.95)
    palm = proposals.candidates[0].pose.position.z + config.palm_behind_tcp
    assert palm == pytest.approx(top + config.gripper_clearance, abs=1e-6)


# A 10 cm bar, 3 cm tall on its left half and 8 cm on its right: with these
# rules it gets the centre grasp, one on each half and the turned one.
_STEPPED_BAR = np.vstack(
    [
        _box((-0.0525, -0.0025), (-0.02, 0.02), (0.0, 0.03)),
        _box((0.0025, 0.0525), (-0.02, 0.02), (0.0, 0.08)),
    ]
)
_EVERY_KIND = {"max_grasp_width": 0.25, "min_split_length": 0.08}


def test_every_kind_of_grasp_aims_its_pads() -> None:
    off = heuristic_grasps(
        _cloud(_STEPPED_BAR), HeuristicGraspConfig(heuristic_aim_pads=False, **_EVERY_KIND)
    )
    on = heuristic_grasps(_cloud(_STEPPED_BAR), PADS.model_copy(update=_EVERY_KIND))

    assert [c.score for c in on.candidates] == pytest.approx([1.0, 0.8, 0.7, 0.5])
    lowest = PADS.gripper_clearance + PADS.fingertip_reach
    for before, after in zip(off.candidates, on.candidates, strict=True):
        assert after.pose.orientation == before.pose.orientation
        assert after.pose.position.z == pytest.approx(
            max(before.pose.position.z - PADS.pad_behind_tcp, lowest)
        )


def test_with_aim_pads_off_the_grasps_are_the_tool_tip_at_mid_height_as_before() -> None:
    config = HeuristicGraspConfig(
        heuristic_aim_pads=False,
        pad_behind_tcp=0.0167,
        fingertip_reach=0.0031,
        palm_behind_tcp=0.05,
        **_EVERY_KIND,
    )

    proposals = heuristic_grasps(_cloud(_STEPPED_BAR), config)

    # Each grasp's position, orientation (x, y, z, w) and score, as computed
    # before the pads were aimed.
    y, down = 3.469446951953614e-18, (-1.0, -3.8738279251049335e-09, 2.3720354844636946e-25)
    w = 6.123233995736766e-17
    turned = (-0.7071067784473375, -0.7071067839257575, 4.3297802979502905e-17)
    assert [_values(c) for c in proposals.candidates] == [
        (0.017500000074505806, y, 0.03750000149011612, *down, w, 1.0),
        (0.027499999850988388, y, 0.03999999910593033, *down, w, 0.8),
        (-0.027499999850988388, y, 0.014999999664723873, *down, w, 0.7),
        (0.017500000074505806, y, 0.03750000149011612, *turned, 4.3297802644046424e-17, 0.5),
    ]


def _values(candidate: Any) -> tuple[float, ...]:
    p, q = candidate.pose.position, candidate.pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w, candidate.score)


def test_the_module_takes_the_pad_offsets_and_the_switch() -> None:
    cloud = _cloud(_box((-0.02, 0.02), (-0.02, 0.02), (0.0, 0.12)))
    aimed = HeuristicGraspModule(pad_behind_tcp=0.0167)
    plain = HeuristicGraspModule(pad_behind_tcp=0.0167, heuristic_aim_pads=False)
    try:
        assert aimed.propose_grasps(cloud).candidates[0].pose.position.z == pytest.approx(0.0433)
        assert plain.propose_grasps(cloud).candidates[0].pose.position.z == pytest.approx(0.06)
    finally:
        aimed.stop()
        plain.stop()
