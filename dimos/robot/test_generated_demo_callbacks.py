# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

import numpy as np

from dimos.message_utils import camera_info, image, pointcloud
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import Point, PointStamped
from dimos.messages.std_msgs.msg import Header
from dimos.robot.diy.alfred.blueprints.vis_nav import _ir_image, _ir_pinhole
from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_vis import (
    _global_map_colors,
    _goal_colors,
    _waypoint_colors,
)
from dimos.robot.unitree.go2.blueprints.navigation.unitree_go2_hol_path_controller import (
    _render_global_map,
)
from dimos.simulation.habitat.blueprints import _small_points


def test_alfred_ir_callbacks_route_generated_headers_to_the_existing_camera_panes():
    frame = "camera_front_infra1_optical_frame"
    header = Header(Time(17, 1), frame)
    source = image.from_array(np.array([[10, 20]], dtype=np.uint8), encoding="mono8", header=header)
    [(entity, pixels)] = _ir_image(source)
    assert entity == "world/camera/infra1"
    assert pixels.buffer.as_arrow_array().to_pylist() == [[10, 10, 10, 20, 20, 20]]
    calibration = camera_info.from_intrinsics(2.0, 3.0, 0.5, 0.5, 2, 1, header=header)
    [(entity, pinhole)] = _ir_pinhole(calibration)
    assert entity == "world/camera/infra1"
    assert pinhole.parent_frame.as_arrow_array().to_pylist() == [
        "tf#/camera_front_infra1_optical_frame"
    ]
    source.header.frame_id = "unrelated_camera"
    calibration.header.frame_id = "unrelated_camera"
    assert _ir_image(source) is None
    assert _ir_pinhole(calibration) is None


def test_generated_cloud_overrides_keep_world_spheres_and_small_screen_points():
    source = pointcloud.from_xyz_rgb(
        np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
        np.array([[255, 0, 0], [0, 255, 0]], dtype=np.uint8),
        header=Header(Time(17, 1), "world"),
    )
    spheres = _render_global_map(source)
    dots = _small_points(source)
    for value in (spheres, dots):
        assert value.positions.as_arrow_array().to_pylist() == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        assert value.colors.as_arrow_array().to_pylist() == [4278190335, 16711935]
    np.testing.assert_allclose(spheres.radii.as_arrow_array().to_pylist(), [0.025])
    np.testing.assert_allclose(dots.radii.as_arrow_array().to_pylist(), [-1.0])


def test_g1_generated_cloud_and_navigation_points_preserve_existing_visual_values():
    source = pointcloud.from_xyz(
        np.array([[1.0, 2.0, 0.0], [3.0, 4.0, 1.0]]), header=Header(Time(17, 1), "world")
    )
    cloud = _global_map_colors(source)
    assert cloud.positions.as_arrow_array().to_pylist() == [[1.0, 2.0, 0.0], [3.0, 4.0, 1.0]]
    assert cloud.colors.as_arrow_array().to_pylist() == [508610815, 1004233983]
    point = PointStamped(Header(Time(17, 1), "world"), Point(1.0, 2.0, 3.0))
    np.testing.assert_allclose(
        _waypoint_colors(point).positions.as_arrow_array().to_pylist(), [[1.0, 2.0, 3.3]]
    )
    np.testing.assert_allclose(
        _goal_colors(point).positions.as_arrow_array().to_pylist(), [[1.0, 2.0, 3.3]]
    )
    point.point.x = float("nan")
    assert _waypoint_colors(point) is None
    assert _goal_colors(point) is None
