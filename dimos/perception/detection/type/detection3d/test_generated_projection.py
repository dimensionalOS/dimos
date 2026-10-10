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

from types import SimpleNamespace

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    Vector3,
)
from dimos_generated.sensor_msgs.msg import CameraInfo, RegionOfInterest
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.agents.skills.visual_servoing.detection_navigation import DetectionNavigation
from dimos.msgs.image import image_from_array
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC


def detection(image):
    return SimpleNamespace(
        image=image,
        bbox=(0, 0, 3, 3),
        track_id=1,
        class_id=0,
        confidence=1.0,
        name="target",
        ts=1.0,
    )


def test_depth_projection_preserves_header_and_transforms():
    header = Header(frame_id="camera", stamp=Time(sec=1700000000, nanosec=123456789))
    depth = image_from_array(
        np.full((4, 4), 2000, dtype=np.uint16), encoding="16UC1", header=header
    )
    camera = CameraInfo(
        width=4,
        height=4,
        k=np.array([2, 0, 0, 0, 2, 0, 0, 0, 1], dtype=np.float64),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        distortion_model="",
        d=np.array([], dtype=np.float64),
        r=np.zeros(9, dtype=np.float64),
        p=np.zeros(12, dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )
    tf = TransformStamped(
        header=Header(frame_id="camera", stamp=Time(sec=0, nanosec=0)),
        child_frame_id="world",
        transform=Transform(
            translation=Vector3(x=-10, y=0.0, z=0.0), rotation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
        ),
    )
    result = Detection3DPC.from_depth(detection(depth), depth, camera, tf, filters=[])
    assert result is not None
    assert result.pointcloud.header.frame_id == "world"
    assert result.pointcloud.header.stamp == header.stamp
    assert result.pose.header == result.pointcloud.header
    assert result.center.x == pytest.approx(11.5)
    assert result.center.y == pytest.approx(1.5)
    assert result.center.z == pytest.approx(2)
    np.testing.assert_array_equal(pointcloud_xyz(result.pointcloud)[0], [10, 0, 2])


def test_cloud_projection_rejects_outside_and_behind_points():
    cloud = pointcloud_from_xyz(
        np.array([[0, 0, 2], [1, 1, 2], [20, 20, 2], [0, 0, -1]]),
        header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
    )
    camera = CameraInfo(
        width=4,
        height=4,
        k=np.array([2, 0, 0, 0, 2, 0, 0, 0, 1], dtype=np.float64),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        distortion_model="",
        d=np.array([], dtype=np.float64),
        r=np.zeros(9, dtype=np.float64),
        p=np.zeros(12, dtype=np.float64),
        binning_x=0,
        binning_y=0,
        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
    )
    tf = TransformStamped(
        header=Header(frame_id="camera", stamp=Time(sec=0, nanosec=0)),
        child_frame_id="world",
        transform=Transform(
            rotation=Quaternion(w=1, x=0.0, y=0.0, z=0.0), translation=Vector3(x=0.0, y=0.0, z=0.0)
        ),
    )
    result = Detection3DPC.from_2d(detection(None), cloud, camera, tf, filters=[])
    assert result is not None
    np.testing.assert_array_equal(pointcloud_xyz(result.pointcloud), [[0, 0, 2], [1, 1, 2]])
    assert result.get_bounding_box_dimensions() == (1, 1, 0)


def test_generated_cloud_target_produces_cdr_navigation_twist():
    points = np.array([[2.0 + i * 0.01, 0, 1] for i in range(40)])
    cloud = pointcloud_from_xyz(
        points, header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0))
    )
    controller = DetectionNavigation(
        None,
        CameraInfo(
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            height=0,
            width=0,
            distortion_model="",
            d=np.array([], dtype=np.float64),
            k=np.zeros(9, dtype=np.float64),
            r=np.zeros(9, dtype=np.float64),
            p=np.zeros(12, dtype=np.float64),
            binning_x=0,
            binning_y=0,
            roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
        ),
    )
    target = controller._compute_robust_target_position(cloud, Vector3(x=0.0, y=0.0, z=0.0))
    assert target is not None
    assert 2.0 <= target.x < 2.1
    tf = TransformStamped(
        transform=Transform(
            rotation=Quaternion(w=1, x=0.0, y=0.0, z=0.0), translation=Vector3(x=0.0, y=0.0, z=0.0)
        ),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        child_frame_id="",
    )
    command = controller._compute_twist_from_3d(target, tf)
    decoded = cdr_decode(cdr_encode(command), Twist)
    assert 0 < decoded.linear.x <= 0.5
    assert decoded.angular.z == 0
