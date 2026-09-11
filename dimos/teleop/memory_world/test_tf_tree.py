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

import numpy as np
import pytest

from dimos.teleop.memory_world.tf_tree import (
    TfTree,
    level_camera_roll,
    pose_matrix,
    quaternion_from_matrix,
    slerp,
)

IDENTITY = (0.0, 0.0, 0.0, 1.0)
YAW_90 = (0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5))
YAW_180 = (0.0, 0.0, 1.0, 0.0)


def test_levelling_keeps_the_view_and_takes_the_roll_from_the_world() -> None:
    """The cart recordings' tf rolls the camera; levelling must keep where it looks."""
    for roll_deg in (0.0, 35.0, -49.0, 179.0):
        roll = np.deg2rad(roll_deg)
        forward = np.array([0.6, -0.8, 0.0])  # optical z, level
        down = np.array([0.0, 0.0, -1.0])
        right = np.cross(down, forward)
        y = np.cos(roll) * down + np.sin(roll) * right  # optical y, rolled
        m = np.eye(4)
        m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = np.cross(y, forward), y, forward, [1.0, 2.0, 3.0]
        levelled = level_camera_roll(m)
        assert np.allclose(levelled[:3, 2], forward)  # looks the same way
        assert np.allclose(levelled[:3, 3], [1.0, 2.0, 3.0])  # stands in the same place
        assert levelled[2, 1] < -0.99  # y is world down
        r = levelled[:3, :3]
        assert np.allclose(r.T @ r, np.eye(3), atol=1e-9) and np.linalg.det(r) > 0


def test_levelling_leaves_a_camera_looking_straight_down_alone() -> None:
    m = np.eye(4)
    m[:3, 2] = [0.0, 0.0, -1.0]  # every roll is as good as any other
    assert np.allclose(level_camera_roll(m), m)


def test_static_chain_composes_parent_to_child() -> None:
    tree = TfTree()
    tree.add("world", "body", 0.0, (1.0, 0.0, 0.0), YAW_90)
    tree.add("body", "camera", 0.0, (1.0, 0.0, 0.0), IDENTITY)

    matrix = tree.lookup("world", "camera", 5.0)

    # The camera sits 1 m along the body's x, which yaw 90 turns into world +y.
    assert matrix is not None
    assert matrix[:3, 3] == pytest.approx((1.0, 1.0, 0.0))


def test_lookup_walks_edges_backwards_too() -> None:
    tree = TfTree()
    tree.add("world", "body", 0.0, (1.0, 2.0, 0.0), IDENTITY)
    tree.add("body", "camera", 0.0, (0.5, 0.0, 0.0), IDENTITY)

    matrix = tree.lookup("camera", "world", 0.0)

    assert matrix is not None
    assert matrix[:3, 3] == pytest.approx((-1.5, -2.0, 0.0))


def test_dynamic_edge_interpolates_between_samples() -> None:
    tree = TfTree()
    tree.add("world", "body", 10.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("world", "body", 12.0, (2.0, 0.0, 0.0), YAW_180)

    matrix = tree.lookup("world", "body", 11.0)

    assert matrix is not None
    assert matrix[:3, 3] == pytest.approx((1.0, 0.0, 0.0))
    # Halfway to a 180 degree yaw is a 90 degree yaw: body x points along world y.
    assert matrix[:3, 0] == pytest.approx((0.0, 1.0, 0.0), abs=1e-6)


def test_lookup_holds_the_ends_within_tolerance_and_refuses_beyond() -> None:
    tree = TfTree()
    tree.add("world", "body", 10.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("world", "body", 12.0, (2.0, 0.0, 0.0), IDENTITY)

    held = tree.lookup("world", "body", 12.05, tolerance_s=0.1)
    assert held is not None
    assert held[:3, 3] == pytest.approx((2.0, 0.0, 0.0))
    assert tree.lookup("world", "body", 12.5, tolerance_s=0.1) is None
    assert tree.lookup("world", "body", 9.5, tolerance_s=0.1) is None


def test_unconnected_frames_have_no_transform() -> None:
    tree = TfTree()
    tree.add("world", "body", 0.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("other", "thing", 0.0, (0.0, 0.0, 0.0), IDENTITY)
    assert tree.lookup("world", "thing", 0.0) is None
    assert tree.lookup("world", "nowhere", 0.0) is None


def test_samples_may_arrive_out_of_order() -> None:
    tree = TfTree()
    tree.add("world", "body", 12.0, (2.0, 0.0, 0.0), IDENTITY)
    tree.add("world", "body", 10.0, (0.0, 0.0, 0.0), IDENTITY)
    matrix = tree.lookup("world", "body", 11.5)
    assert matrix is not None
    assert matrix[:3, 3] == pytest.approx((1.5, 0.0, 0.0))


def test_span_is_where_every_edge_on_the_path_has_data() -> None:
    tree = TfTree()
    tree.add("world", "body", 10.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("world", "body", 20.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("body", "camera", 12.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("body", "camera", 30.0, (0.0, 0.0, 0.0), IDENTITY)
    assert tree.span("world", "camera") == (12.0, 20.0)
    assert tree.span("world", "nowhere") is None


def test_an_edge_published_once_spans_from_then_on() -> None:
    """A camera hung on the body by one tf message (no tf_static) is held afterwards,
    so the span of the path is bounded by the moving edge, not collapsed to an instant."""
    tree = TfTree()
    tree.add("world", "body", 10.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("world", "body", 20.0, (0.0, 0.0, 0.0), IDENTITY)
    tree.add("body", "camera", 12.0, (0.0, 0.0, 1.0), IDENTITY)
    assert tree.span("world", "camera") == (12.0, 20.0)
    assert tree.lookup("world", "camera", 19.0) is not None


def test_quaternion_round_trips_through_the_matrix() -> None:
    for quat in [IDENTITY, (0.0, 0.218, 0.0, 0.976), (0.5, 0.5, 0.5, 0.5), YAW_90]:
        unit = np.array(quat) / np.linalg.norm(quat)
        back = np.array(quaternion_from_matrix(pose_matrix((0, 0, 0), tuple(unit))[:3, :3]))
        assert np.allclose(back, unit, atol=1e-6) or np.allclose(back, -unit, atol=1e-6)


def test_slerp_halfway_between_identity_and_a_quarter_turn() -> None:
    halfway = slerp(np.array(IDENTITY), np.array(YAW_90), 0.5)
    angle = 2 * np.degrees(np.arccos(halfway[3]))
    assert angle == pytest.approx(45.0, abs=1e-6)
