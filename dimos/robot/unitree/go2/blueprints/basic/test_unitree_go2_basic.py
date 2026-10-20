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

"""Go2 blueprint rendering consumes the same generated values as live ports."""

import numpy as np

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.dimos_msgs.msg import LineSegment3D, LineSegments3D
from dimos.messages.geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Quaternion
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.occupancy import occupancy_from_array
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.robot.unitree.go2.blueprints.basic import unitree_go2_basic as view
from dimos.robot.unitree.go2.connection import GO2Connection


def test_camera_and_pose_overrides_accept_generated_port_values():
    camera = view._convert_camera_info(GO2Connection.camera_info_static)
    assert camera[0][0] == "/world/color_image"
    assert camera[0][1].resolution.as_arrow_array().to_pylist() == [[1280.0, 720.0]]
    pose = Pose(Point(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    entries = dict(view._plot_odom(PoseStamped(Header(Time(0, 0), "world"), pose)))
    assert "world/odom/message" in entries
    assert "world/odom" in entries
    assert entries["plots/odom/x"].scalars.as_arrow_array().to_pylist() == [1.0]
    assert entries["plots/odom/y"].scalars.as_arrow_array().to_pylist() == [2.0]


def test_map_and_pgo_overrides_preserve_rendered_geometry():
    header = Header(Time(0, 0), "world")
    cloud = pointcloud_from_xyz(np.array([[1.0, 2.0, 0.2], [9.0, 8.0, -0.1]]), header=header)
    points = view._convert_global_map(cloud).positions.as_arrow_array().to_pylist()
    np.testing.assert_allclose(points, [[1.0, 2.0, 0.2]])
    pose = Pose(Point(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    keyframes = dict(view._convert_pgo_keyframes(PoseArray(header, [pose])))
    assert keyframes["world/pgo/keyframes"].positions.as_arrow_array().to_pylist() == [
        [1.0, 2.0, 3.0]
    ]
    edges = LineSegments3D(header, [LineSegment3D(Point(1.0, 2.0, 3.0), Point(4.0, 5.0, 6.0), 0.9)])
    loops = dict(view._convert_pgo_loops(edges))
    assert loops["world/pgo/loops"].strips.as_arrow_array().to_pylist() == [
        [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
    ]
    grid = occupancy_from_array(np.array([[0, 100]], dtype=np.int8), header=header, resolution=0.5)
    mesh = view._convert_navigation_costmap(grid)
    np.testing.assert_allclose(
        mesh.vertex_positions.as_arrow_array().to_pylist(),
        [[0.0, 0.0, 0.015], [1.0, 0.0, 0.015], [1.0, 0.5, 0.015], [0.0, 0.5, 0.015]],
    )
