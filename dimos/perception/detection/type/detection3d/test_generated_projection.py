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

from contextlib import ExitStack

import numpy as np
import pytest
from reactivex import operators as ops
from reactivex.subject import Subject

from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.geometry_msgs.msg import (
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos.messages.sensor_msgs.msg import CameraInfo, PointCloud2, PointField, RegionOfInterest
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.image import image_from_array
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_view, pointcloud_xyz
from dimos.perception.detection.module3D import _timed_cloud
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.types.timestamped import align_timestamped


def detection(image):
    return Detection2DBBox(
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


@pytest.mark.parametrize("batched", [False, True], ids=["single", "batched"])
def test_cloud_projection_rejects_outside_and_behind_points(batched):
    records = np.array(
        [(0, 0, 2, 41), (1, 1, 2, 42), (20, 20, 2, 43), (0, 0, -1, 44)],
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("id", "<u4")],
    )
    cloud = PointCloud2(
        header=Header(frame_id="world", stamp=Time(sec=1700000000, nanosec=123456789)),
        height=1,
        width=4,
        point_step=16,
        row_step=64,
        fields=[
            PointField(name=name, offset=i * 4, datatype=7 if i < 3 else 6, count=1)
            for i, name in enumerate(("x", "y", "z", "id"))
        ],
        data=np.frombuffer(records.tobytes(), dtype=np.uint8),
        is_bigendian=False,
        is_dense=False,
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
    image = image_from_array(
        np.zeros((4, 4, 3), dtype=np.uint8), encoding="rgb8", header=cloud.header
    )
    det = detection(image)
    if batched:
        results = ImageDetections3DPC.from_2d(
            ImageDetections2D(image, [det]), cloud, camera, tf, filters=[]
        )
        assert len(results) == 1
        result = results[0]
    else:
        result = Detection3DPC.from_2d(det, cloud, camera, tf, filters=[])
    assert result is not None
    np.testing.assert_array_equal(pointcloud_xyz(result.pointcloud), [[0, 0, 2], [1, 1, 2]])
    assert result.get_bounding_box_dimensions() == (1, 1, 0)
    assert result.pointcloud.header == cloud.header
    assert result.pointcloud.fields == cloud.fields
    np.testing.assert_array_equal(pointcloud_view(result.pointcloud).reshape(-1), records[:2])


def test_generated_cloud_alignment_uses_source_time_without_mutating_wire_values():
    image = image_from_array(
        np.zeros((4, 4, 3), dtype=np.uint8),
        encoding="rgb8",
        header=Header(frame_id="camera", stamp=Time(sec=1, nanosec=1)),
    )
    cloud = pointcloud_from_xyz(
        np.array([[1, 2, 3]]), header=Header(frame_id="world", stamp=Time(sec=1, nanosec=100000001))
    )
    frame = ImageDetections2D(image, [detection(image)])
    results = []
    with ExitStack() as cleanup:
        frames, clouds = Subject(), Subject()
        cleanup.callback(frames.dispose)
        cleanup.callback(clouds.dispose)
        subscription = align_timestamped(
            frames, clouds.pipe(ops.map(_timed_cloud)), match_tolerance=0.25
        ).subscribe(results.append)
        cleanup.callback(subscription.dispose)
        clouds.on_next(cloud)
        frames.on_next(frame)
        assert len(results) == 1
        assert results[0][0] is frame
        assert results[0][1].value is cloud
        assert cloud.header.stamp.nanosec == 100000001
        assert not hasattr(cloud, "ts")
