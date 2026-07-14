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

"""Unit tests for the agent-facing lidar signal skills (wishlist item 3)."""

from __future__ import annotations

import math

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3, make_vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.skills.mapping.lidar_signal_skills import LidarSignalSkills


def _feed(module: LidarSignalSkills, pts: np.ndarray) -> None:
    module._client.add_lidar(PointCloud2.from_numpy(pts.astype(np.float32), frame_id="world"))


def _pose(x: float, y: float, z: float = 0.0, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        position=make_vector3(x, y, z),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
        frame_id="world",
    )


def _room_cloud() -> np.ndarray:
    """A 4 x 2 x 1 m "room": dense clusters at both corners so the robust
    percentile bounds (>=1% of points at each extreme) equal the true ones."""
    return np.vstack([np.zeros((50, 3)), np.tile([4.0, 2.0, 1.0], (50, 1))])


def test_measure_space_reports_dimensions() -> None:
    module = LidarSignalSkills()
    try:
        _feed(module, _room_cloud())
        out = module.measure_space()
        assert "4.0 x 2.0 m" in out
        assert "1.0 m tall" in out
    finally:
        module.stop()


def test_measure_space_ignores_stray_returns() -> None:
    """A few long-range strays (reflections, through-glass returns) must not
    inflate the reported dimensions -- raw min/max bounds once turned a
    ~44x76m building into a "143.6 x 184.2 m, 104.5 m tall" answer."""
    module = LidarSignalSkills()
    try:
        strays = np.array([[200.0, -150.0, 80.0], [-90.0, 300.0, -40.0]])
        _feed(module, np.vstack([_room_cloud(), strays]))
        out = module.measure_space()
        assert "4.0 x 2.0 m" in out
        assert "1.0 m tall" in out
    finally:
        module.stop()


def test_measure_space_no_data() -> None:
    module = LidarSignalSkills()
    try:
        assert "No lidar data" in module.measure_space()
    finally:
        module.stop()


def test_nearest_obstacle_needs_odom() -> None:
    module = LidarSignalSkills()
    try:
        _feed(module, np.array([[3, 0, 0.5]], dtype=float))
        assert "position unknown" in module.nearest_obstacle_distance()
    finally:
        module.stop()


def test_nearest_obstacle_from_robot_pose() -> None:
    module = LidarSignalSkills()
    try:
        # Robot at origin; a wall point 3 m ahead at body height, plus a floor
        # point right under it that the body band must ignore.
        _feed(module, np.array([[3.0, 0.0, 0.5], [0.2, 0.0, -0.5]], dtype=float))
        module._on_odom(_pose(0.0, 0.0, 0.0))
        out = module.nearest_obstacle_distance()
        assert "3.0" in out and "m away" in out
    finally:
        module.stop()


def test_check_clearance_uses_robot_heading() -> None:
    module = LidarSignalSkills()
    try:
        # Obstacle to the east (+x) at 2 m, body height.
        _feed(module, np.array([[2.0, 0.0, 0.5]], dtype=float))
        # Facing east (yaw 0): the cone sees it.
        module._on_odom(_pose(0.0, 0.0, 0.0, yaw=0.0))
        ahead = module.check_clearance()
        assert "straight ahead" in ahead and "2.0" in ahead
        # Facing north (yaw 90°): the east obstacle is outside the forward cone.
        module._on_odom(_pose(0.0, 0.0, 0.0, yaw=math.pi / 2))
        assert "Clear straight ahead" in module.check_clearance()
    finally:
        module.stop()


def test_check_clearance_explicit_heading() -> None:
    module = LidarSignalSkills()
    try:
        _feed(module, np.array([[2.0, 0.0, 0.5]], dtype=float))
        module._on_odom(_pose(0.0, 0.0, 0.0))
        # Heading 0° world = toward the obstacle; 180° = away.
        assert "2.0" in module.check_clearance(heading_deg=0.0)
        assert "Clear toward 180°" in module.check_clearance(heading_deg=180.0)
    finally:
        module.stop()


def test_coverage_report() -> None:
    module = LidarSignalSkills()
    try:
        assert "Nothing scanned yet" in module.coverage_report()
        # A dense 2x2 m patch at floor height.
        rng = np.random.default_rng(0)
        patch = np.column_stack(
            [rng.uniform(0, 2, 4000), rng.uniform(0, 2, 4000), np.zeros(4000)]
        )
        _feed(module, patch)
        out = module.coverage_report()
        assert "1 lidar frame" in out
        assert "m²" in out
    finally:
        module.stop()
