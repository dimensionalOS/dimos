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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, PointStamped
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_vis import (
    _global_map_colors,
    _goal_colors,
    _waypoint_colors,
)


def test_global_map_uses_generated_xyz_and_height_colors():
    cloud = pointcloud_from_xyz(
        np.array([[1.0, 2.0, 0.0], [3.0, 4.0, 2.0]]),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    result = _global_map_colors(cdr_decode(cdr_encode(cloud), type(cloud)))
    np.testing.assert_array_equal(
        result.positions.as_arrow_array().to_pylist(), [[1, 2, 0], [3, 4, 2]]
    )
    assert result.colors.as_arrow_array().to_pylist() == [0x1E50C8FF, 0x3BDB64FF]


@pytest.mark.parametrize("render", [_goal_colors, _waypoint_colors])
def test_generated_stamped_targets_keep_position_and_reject_nonfinite(render):
    target = PointStamped(
        header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)), point=Point(x=1, y=2, z=3)
    )
    result = render(cdr_decode(cdr_encode(target), PointStamped))
    np.testing.assert_allclose(result.positions.as_arrow_array().to_pylist(), [[1, 2, 3.3]])
    target.point.x = float("nan")
    assert render(target) is None
