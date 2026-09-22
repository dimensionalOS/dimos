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
# SPDX-License-Identifier: Apache-2.0

import math

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.std_msgs.msg import Header
import pytest

from dimos.msgs.geometry import (
    compose_transforms,
    inverse_transform,
    pose_from_transform,
    quaternion_from_euler,
    transform_from_odometry,
    transform_from_pose,
    yaw,
)


@pytest.mark.parametrize("seconds,nanoseconds", [(0, 0), (-1, 500000000), (1700000000, 123456789)])
def test_odometry_transform_preserves_frames_stamp_and_pose(seconds, nanoseconds):
    message = Odometry(child_frame_id="robot/lidar")
    message.header.frame_id = "robot/odom"
    message.header.stamp = Time(sec=seconds, nanosec=nanoseconds)
    message.pose.pose.position = Point(x=1, y=2, z=3)
    message.pose.pose.orientation = Quaternion(z=0.6, w=0.8)

    transformed = transform_from_odometry(message)

    assert transformed.header.frame_id == "robot/odom"
    assert transformed.child_frame_id == "robot/lidar"
    assert transformed.header.stamp.sec == seconds
    assert transformed.header.stamp.nanosec == nanoseconds
    assert (
        transformed.transform.translation.x,
        transformed.transform.translation.y,
        transformed.transform.translation.z,
    ) == (1, 2, 3)
    assert transformed.transform.rotation == message.pose.pose.orientation
    transformed.header.frame_id = "changed"
    transformed.transform.rotation.w = 0
    assert message.header.frame_id == "robot/odom"
    assert message.pose.pose.orientation.w == 0.8


def _edge(parent, child, xyz, rotation, stamp=Time(sec=1700000000, nanosec=123456789)):
    return TransformStamped(
        header=Header(frame_id=parent, stamp=stamp),
        child_frame_id=child,
        transform=Transform(translation=Vector3(x=xyz[0], y=xyz[1], z=xyz[2]), rotation=rotation),
    )


def test_transform_composition_rotates_translation_and_preserves_exact_source_stamp():
    first = _edge("world", "arm", (1, -1, 0), quaternion_from_euler(0, 0, math.pi / 2))
    second = _edge("arm", "tool", (2, 3, 4), Quaternion(w=1), Time(sec=7))

    result = compose_transforms(first, second)

    assert result.header == first.header
    assert result.child_frame_id == "tool"
    translation = result.transform.translation
    assert (translation.x, translation.y, translation.z) == pytest.approx((-2, 1, 4))
    assert yaw(result.transform.rotation) == pytest.approx(math.pi / 2)
    result.header.stamp.nanosec = 0
    result.transform.translation.x = 99
    assert first.header.stamp.nanosec == 123456789
    assert first.transform.translation.x == 1
    assert second.transform.translation.x == 2


def test_inverse_composes_to_identity_and_swaps_frames_without_losing_nanoseconds():
    original = _edge("map", "camera", (2, 3, -4), quaternion_from_euler(0.3, -0.2, 1.1))
    inverse = inverse_transform(original)
    result = compose_transforms(original, inverse)

    assert inverse.header.frame_id == "camera"
    assert inverse.child_frame_id == "map"
    assert inverse.header.stamp == original.header.stamp
    translation = result.transform.translation
    assert (translation.x, translation.y, translation.z) == pytest.approx((0, 0, 0), abs=1e-12)
    rotation = result.transform.rotation
    assert (rotation.x, rotation.y, rotation.z, abs(rotation.w)) == pytest.approx((0, 0, 0, 1))


def test_transform_composition_rejects_disconnected_frames():
    first = _edge("a", "b", (0, 0, 0), Quaternion(w=1))
    second = _edge("c", "d", (0, 0, 0), Quaternion(w=1))
    with pytest.raises(ValueError, match="Cannot compose frames"):
        compose_transforms(first, second)


@pytest.mark.parametrize(
    "rotation", [Quaternion(w=0), Quaternion(x=math.nan, w=1), Quaternion(w=math.inf)]
)
def test_transform_math_rejects_invalid_rotations(rotation):
    edge = _edge("a", "b", (0, 0, 0), rotation)
    with pytest.raises(ValueError):
        inverse_transform(edge)
    with pytest.raises(ValueError):
        compose_transforms(edge, _edge("b", "c", (0, 0, 0), Quaternion(w=1)))


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_transform_math_rejects_nonfinite_translations_and_angles(value):
    edge = _edge("a", "b", (value, 0, 0), Quaternion(w=1))
    with pytest.raises(ValueError, match="Translation components"):
        inverse_transform(edge)
    with pytest.raises(ValueError, match="Euler angles"):
        quaternion_from_euler(value, 0, 0)


def test_pose_conversion_copies_nested_fields_and_preserves_header():
    edge = _edge("map", "robot", (1, 2, 3), Quaternion(z=0.6, w=0.8))
    pose = pose_from_transform(edge)
    assert pose.header == edge.header
    assert pose.pose.position == Point(x=1, y=2, z=3)
    assert pose.pose.orientation == edge.transform.rotation
    pose.header.frame_id = "changed"
    pose.pose.orientation.w = 0
    assert edge.header.frame_id == "map"
    assert edge.transform.rotation.w == 0.8


def test_pose_transform_preserves_header_and_does_not_alias_input():
    source = PoseStamped(
        header=Header(frame_id="odom", stamp=Time(sec=-1, nanosec=987654321)),
        pose=Pose(position=Point(x=1, y=2, z=3), orientation=Quaternion(z=0.6, w=0.8)),
    )
    result = transform_from_pose(source, child_frame_id="robot/base")
    decoded = TransformStamped.decode(result.encode())
    assert decoded.header == source.header
    assert decoded.child_frame_id == "robot/base"
    assert decoded.transform.translation == Vector3(x=1, y=2, z=3)
    assert decoded.transform.rotation == source.pose.orientation
    result.header.frame_id = "changed"
    result.transform.rotation.w = 0
    assert source.header.frame_id == "odom"
    assert source.pose.orientation.w == 0.8
