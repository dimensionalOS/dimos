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

# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Roundtrip + field tests for the jnav Marker message."""

from __future__ import annotations

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.navigation.jnav.msgs.Marker import Marker


def _pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    return pose


def test_marker_roundtrip_preserves_all_fields() -> None:
    marker = Marker(
        pose=_pose(-52.93, -55.95, 0.4),
        marker="test_waypoint_2",
        map="dim_city",
        ts=1781565207.5,
        frame_id="map",
    )
    decoded = Marker.lcm_decode(marker.lcm_encode())

    assert decoded.marker == "test_waypoint_2"
    assert decoded.map == "dim_city"
    assert decoded.frame_id == "map"
    assert decoded.ts == marker.ts
    assert decoded.pose.position.x == -52.93
    assert decoded.pose.position.y == -55.95
    assert decoded.pose.position.z == 0.4


def test_marker_defaults() -> None:
    marker = Marker()
    assert marker.marker == ""
    assert marker.map == ""
    assert marker.frame_id == "map"
    assert marker.ts > 0  # auto-stamped
    # empty optional strings survive the roundtrip
    decoded = Marker.lcm_decode(marker.lcm_encode())
    assert decoded.marker == "" and decoded.map == ""


def test_marker_unicode_name() -> None:
    decoded = Marker.lcm_decode(Marker(marker="café_door", map="map_α").lcm_encode())  # noqa: RUF001
    assert decoded.marker == "café_door"
    assert decoded.map == "map_α"  # noqa: RUF001
