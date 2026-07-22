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

import math

import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.dannav.geometry.path_distancer import PathDistancer


def posed_path(points_yaws):
    return Path(
        frame_id="map",
        poses=[
            PoseStamped(
                position=(x, y, 0.0),
                orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
            )
            for x, y, yaw in points_yaws
        ],
    )


class TestPoseYawAtProgress:
    def test_constant_stamped_yaw_differs_from_tangent(self):
        # Straight +x path, every pose stamped sideways (strafe)
        d = PathDistancer(posed_path([(x, 0.0, math.pi / 2) for x in (0.0, 1.0, 2.0)]))
        for s in (0.0, 0.5, 1.0, 1.7, 2.0):
            assert d.pose_yaw_at_progress(s) == pytest.approx(math.pi / 2)
        assert d.yaw_at_progress(1.0) == pytest.approx(0.0)  # tangent unaffected

    def test_interpolates_between_waypoints(self):
        d = PathDistancer(posed_path([(0.0, 0.0, 0.0), (1.0, 0.0, 1.0)]))
        assert d.pose_yaw_at_progress(0.5) == pytest.approx(0.5)

    def test_shortest_arc_across_pi(self):
        # 170° -> -170°: shortest arc passes through 180°, not through 0
        d = PathDistancer(
            posed_path([(0.0, 0.0, math.radians(170)), (1.0, 0.0, math.radians(-170))])
        )
        mid = d.pose_yaw_at_progress(0.5)
        assert abs(mid) == pytest.approx(math.pi, abs=1e-6)

    def test_clamped_beyond_ends(self):
        d = PathDistancer(posed_path([(0.0, 0.0, 0.2), (1.0, 0.0, 0.8)]))
        assert d.pose_yaw_at_progress(-5.0) == pytest.approx(0.2)
        assert d.pose_yaw_at_progress(5.0) == pytest.approx(0.8)
