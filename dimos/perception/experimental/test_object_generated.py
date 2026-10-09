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

"""Object geometry and deduplication use generated ROS values."""

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from dimos_generated.sensor_msgs.msg import Image, PointCloud2
from dimos_generated.std_msgs.msg import Header
from dimos_generated.vision_msgs.msg import Detection3D
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np

from dimos.msgs.image import image_from_array
from dimos.msgs.pointcloud import pointcloud_from_xyz_rgb, pointcloud_rgb, pointcloud_xyz
from dimos.perception.experimental.object import Object, aggregate_pointclouds
from dimos.perception.experimental.objectDB import ObjectDB


def _object(object_id, x, timestamp):
    return Object(
        object_id=object_id,
        name="cup",
        center=Vector3(x=x, y=0.0, z=0.0),
        size=Vector3(x=0.1, y=0.2, z=0.3),
        pose=PoseStamped(
            header=Header(frame_id="world", stamp=Time(sec=1700000000, nanosec=123456789)),
            pose=Pose(
                position=Point(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        pointcloud=PointCloud2(
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            height=0,
            width=0,
            fields=[],
            is_bigendian=False,
            point_step=0,
            row_step=0,
            data=np.array([], dtype=np.uint8),
            is_dense=False,
        ),
        bbox=(0.0, 0.0, 1.0, 1.0),
        track_id=-1,
        class_id=0,
        confidence=0.9,
        ts=timestamp,
        frame_id="world",
        image=Image(
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            height=0,
            width=0,
            encoding="",
            is_bigendian=0,
            step=0,
            data=np.array([], dtype=np.uint8),
        ),
    )


def test_detection_cdr_preserves_source_pose_header_and_current_center():
    obj = _object("cup-1", 0.2, 1.0)
    obj.set_center(Vector3(x=0.4, y=0.5, z=0.6))
    decoded = cdr_decode(obj.to_detection3d_msg().encode(), Detection3D)
    assert decoded.header == obj.pose.header
    assert decoded.bbox.center == obj.pose.pose
    assert decoded.bbox.size == obj.size
    assert decoded.results[0].hypothesis.class_id == "cup"
    assert decoded.results[0].hypothesis.score == 0.9
    decoded.bbox.center.position.x = 99
    assert obj.center.x == 0.4
    assert obj.pose.pose.position.x == 0.4


def test_spatial_deduplication_and_nearest_lookup_use_generated_centers():
    database = ObjectDB(distance_threshold=0.2, min_detections_for_permanent=1)
    first = _object("first", 0.0, 1.0)
    nearby = _object("nearby", 0.1, 2.0)
    distant = _object("distant", 2.0, 3.0)
    assert database.add_objects([first]) == [first]
    assert database.add_objects([nearby]) == [first]
    assert first.center.x == 0.1
    assert database.add_objects([distant]) == [distant]
    assert database.find_nearest(Vector3(x=1.8, y=0.0, z=0.0)) is distant
    assert database.find_nearest(Vector3(x=0.2, y=0.0, z=0.0)) is first


def test_generated_object_cloud_aggregation_and_dictionary_preserve_values():
    obj = _object("abc", 0.2, 1.0)
    stamp = Time(sec=1700000000, nanosec=987654321)
    obj.pointcloud = pointcloud_from_xyz_rgb(
        np.array([[1.0, 2.0, 3.0]], dtype=np.float32),
        np.array([[255, 128, 0]], dtype=np.uint8),
        header=Header(frame_id="world", stamp=stamp),
    )
    pixels = np.zeros((2, 3, 3), dtype=np.uint8)
    obj.image = image_from_array(pixels, encoding="rgb8", header=obj.pointcloud.header)
    exported = obj.to_dict()
    np.testing.assert_array_equal(exported["pointcloud"][0], [[1.0, 2.0, 3.0]])
    np.testing.assert_allclose(exported["pointcloud"][1], [[1.0, 128 / 255.0, 0.0]])
    np.testing.assert_array_equal(exported["image"], pixels)
    merged = aggregate_pointclouds([obj])
    decoded = cdr_decode(cdr_encode(merged), PointCloud2)
    assert decoded.header.stamp == stamp and decoded.header.frame_id == "world"
    np.testing.assert_array_equal(pointcloud_xyz(decoded), [[1.0, 2.0, 3.0]])
    assert pointcloud_rgb(decoded) is not None
    assert aggregate_pointclouds([]).width == 0
