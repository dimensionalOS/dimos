# Copyright 2025-2026 Dimensional Inc.
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

"""Progress-indexed full-pose reference: arc-length build, windowed
projection, pose interpolation (incl. yaw wrap), and spatial rates."""

from __future__ import annotations

import math

import pytest

from dimos.control.tasks.holonomic_pose_follower_task.progress_reference import (
    ProgressPathReference,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path


def _pose(x, y, yaw):
    return PoseStamped(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def _straight_rotate(length=2.0, yaw_end=math.pi / 2, n=40):
    return Path(poses=[_pose(length * i / n, 0.0, yaw_end * i / n) for i in range(n + 1)])


def _circle(radius=1.0, n=100):
    poses = []
    for i in range(n + 1):
        th = 2 * math.pi * i / n
        poses.append(_pose(radius * math.sin(th), radius * (1 - math.cos(th)), 0.0))
    return Path(poses=poses)


def test_rejects_degenerate_paths():
    with pytest.raises(ValueError):
        ProgressPathReference(Path(poses=[_pose(0, 0, 0)]))
    with pytest.raises(ValueError):
        ProgressPathReference(Path(poses=[_pose(0, 0, 0), _pose(0, 0, 1.0)]))


def test_arc_length_and_end_pose():
    ref = ProgressPathReference(_straight_rotate(length=2.0))
    assert ref.length == pytest.approx(2.0)
    x, y, yaw = ref.end_pose()
    assert (x, y) == pytest.approx((2.0, 0.0))
    assert yaw == pytest.approx(math.pi / 2)


def test_sample_interpolates_position_and_commanded_yaw():
    ref = ProgressPathReference(_straight_rotate(length=2.0, yaw_end=math.pi / 2))
    s = ref.sample(1.0)  # halfway
    assert s.x == pytest.approx(1.0)
    assert s.y == pytest.approx(0.0)
    assert s.yaw == pytest.approx(math.pi / 4)  # commanded yaw, NOT the tangent (0)
    assert (s.tangent_x, s.tangent_y) == pytest.approx((1.0, 0.0))
    assert s.dyaw_ds == pytest.approx(math.pi / 4, rel=1e-6)


def test_sample_clamps_to_path():
    ref = ProgressPathReference(_straight_rotate(length=2.0))
    assert ref.sample(-1.0).x == pytest.approx(0.0)
    assert ref.sample(99.0).x == pytest.approx(2.0)


def test_yaw_interpolation_across_pi_wrap():
    # Commanded yaw sweeps 170deg -> 190deg (i.e. -170deg): interpolation must
    # go THROUGH pi, not backwards through 0.
    a, b = math.radians(170), math.radians(190)
    path = Path(poses=[_pose(i * 0.1, 0.0, a + (b - a) * i / 10) for i in range(11)])
    ref = ProgressPathReference(path)
    mid = ref.sample(ref.length / 2)
    assert abs(mid.yaw) == pytest.approx(math.pi, abs=1e-6)
    end = ref.sample(ref.length)
    assert end.yaw == pytest.approx(math.radians(-170), abs=1e-9)


def test_advance_projects_onto_path():
    ref = ProgressPathReference(_straight_rotate(length=2.0))
    s = ref.advance(0.5, 0.3)  # off-path point projects onto y=0
    assert s == pytest.approx(0.5)


def test_advance_reference_waits_when_robot_stalls():
    ref = ProgressPathReference(_straight_rotate(length=2.0))
    s1 = ref.advance(0.5, 0.0)
    s2 = ref.advance(0.5, 0.0)  # robot did not move
    assert s1 == s2 == pytest.approx(0.5)


def test_advance_is_windowed_on_closed_path():
    # Circle: start == goal. Standing at the start must NOT match the far end.
    ref = ProgressPathReference(_circle(radius=1.0))
    s = ref.advance(0.0, 0.0)
    assert s < 0.5
    # ...and walking the first quarter advances monotonically.
    prev = s
    for i in range(1, 26):
        th = 2 * math.pi * i / 100
        s = ref.advance(math.sin(th), 1 - math.cos(th))
        assert s >= prev - 1e-9
        prev = s
    assert s == pytest.approx(2 * math.pi * 0.25, abs=0.05)


def test_advance_allows_bounded_backslide():
    ref = ProgressPathReference(_straight_rotate(length=2.0), back_m=0.3)
    ref.advance(1.0, 0.0)
    s = ref.advance(0.2, 0.0)  # robot shoved backwards by 0.8 m
    assert s == pytest.approx(0.7)  # backslide capped at back_m


def test_coincident_waypoints_keep_later_yaw():
    # Duplicate position with a different yaw = stop-and-rotate hint; the
    # later yaw survives as the target at that station.
    path = Path(poses=[_pose(0, 0, 0), _pose(1.0, 0, 0), _pose(1.0, 0, 1.0), _pose(2.0, 0, 1.0)])
    ref = ProgressPathReference(path)
    assert ref.length == pytest.approx(2.0)
    assert ref.sample(1.0).yaw == pytest.approx(1.0)


def test_max_rates_ahead_sees_upcoming_yaw_demand():
    # Flat commanded yaw for 1 m, then a fast 90deg twist over 0.5 m.
    poses = [_pose(i * 0.1, 0.0, 0.0) for i in range(11)]
    poses += [_pose(1.0 + i * 0.1, 0.0, (math.pi / 2) * i / 5) for i in range(1, 6)]
    ref = ProgressPathReference(Path(poses=poses))
    dyaw_flat, _ = ref.max_rates_ahead(0.0, 0.5)
    dyaw_ahead, _ = ref.max_rates_ahead(0.6, 0.5)
    assert dyaw_flat == pytest.approx(0.0, abs=1e-9)
    assert dyaw_ahead == pytest.approx(math.pi, rel=0.05)  # 90deg over 0.5 m


def test_curvature_reported_for_position_turns():
    ref = ProgressPathReference(_circle(radius=1.0))
    _, kappa = ref.max_rates_ahead(ref.length / 2, 0.3)
    assert kappa == pytest.approx(1.0, rel=0.1)
