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

"""Generated point/stamp roundtrips and explicit pose construction."""

from dimos_generated.geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion
from dimos_generated.std_msgs.msg import Header
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.time import time_from_nanoseconds, time_from_seconds, to_nanoseconds


def test_point_has_standard_cdr_schema() -> None:
    point = Point(x=1, y=2, z=3)
    decoded = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(point.encode(), Point.msg_name)
    assert (decoded.x, decoded.y, decoded.z) == (1, 2, 3)


def test_cdr_encode_decode() -> None:
    source = PointStamped(
        point=Point(x=1.5, y=-2.5, z=3.5),
        header=Header(stamp=time_from_nanoseconds(1700000000123456789), frame_id="/world/grid"),
    )
    dest = PointStamped.decode(source.encode())
    assert isinstance(dest, PointStamped)
    assert dest is not source
    assert dest.point.x == source.point.x
    assert dest.point.y == source.point.y
    assert dest.point.z == source.point.z
    assert to_nanoseconds(dest.header.stamp) == to_nanoseconds(source.header.stamp)
    assert dest.header.frame_id == source.header.frame_id


def test_explicit_pose_stamped_conversion() -> None:
    point = PointStamped(
        point=Point(x=1, y=2, z=3), header=Header(stamp=time_from_seconds(500), frame_id="/map")
    )
    pose = PoseStamped(
        header=point.header, pose=Pose(position=point.point, orientation=Quaternion(w=1))
    )
    assert isinstance(pose, PoseStamped)
    assert pose.pose.position.x == 1
    assert pose.pose.position.y == 2
    assert pose.pose.position.z == 3
    assert pose.pose.orientation.w == 1
    assert to_nanoseconds(pose.header.stamp) == 500000000000
    assert pose.header.frame_id == "/map"
    pose.pose.position.x = 9
    assert point.point.x == 1
