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

"""Tests for geometry_msgs.PointStamped — msg -> lcm bytes -> msg roundtrip."""

import time

from dimos_lcm.geometry_msgs import Point as LCMPoint

from dimos.msgs.geometry_msgs.Point import Point
from dimos.msgs.geometry_msgs.PointStamped import PointStamped


def test_point_inherits_lcm() -> None:
    """Point wrapper inherits from LCMPoint."""
    assert isinstance(Point(1.0, 2.0, 3.0), LCMPoint)


def test_lcm_encode_decode() -> None:
    """Test encoding and decoding of PointStamped to/from binary LCM format."""
    source = PointStamped(
        x=1.5,
        y=-2.5,
        z=3.5,
        ts=time.time(),
        frame_id="/world/grid",
    )
    binary_msg = source.lcm_encode()
    dest = PointStamped.lcm_decode(binary_msg)

    assert isinstance(dest, PointStamped)
    assert dest is not source
    assert dest.x == source.x
    assert dest.y == source.y
    assert dest.z == source.z
    assert abs(dest.ts - source.ts) < 1e-6
    assert dest.frame_id == source.frame_id


def test_to_pose_stamped() -> None:
    """Test conversion to PoseStamped with identity orientation."""
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

    pt = PointStamped(x=1.0, y=2.0, z=3.0, ts=500.0, frame_id="/map")
    pose = pt.to_pose_stamped()

    assert isinstance(pose, PoseStamped)
    assert pose.x == 1.0
    assert pose.y == 2.0
    assert pose.z == 3.0
    assert pose.orientation.w == 1.0
    assert pose.ts == 500.0
    assert pose.frame_id == "/map"
