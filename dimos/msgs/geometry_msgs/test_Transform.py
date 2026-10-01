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
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.geometry import (
    compose_transform_values,
    compose_transforms,
    point_from_array,
    quaternion_from_euler,
    relative_pose_transform,
    transform_from_pose,
    transform_pose_local,
    vector_array,
)


def test_transform_initialization() -> None:
    tf = Transform()
    assert tf.translation.x == 0.0
    assert tf.translation.y == 0.0
    assert tf.translation.z == 0.0
    assert tf.rotation.x == 0.0
    assert tf.rotation.y == 0.0
    assert tf.rotation.z == 0.0
    assert tf.rotation.w == 1.0
    trans = Vector3(x=1.0, y=2.0, z=3.0)
    rot = Quaternion(x=0.0, y=0.0, z=0.707107, w=0.707107)
    tf2 = Transform(translation=trans, rotation=rot)
    assert tf2.translation == trans
    assert tf2.rotation == rot
    tf5 = Transform(translation=Vector3(x=7.0, y=8.0, z=9.0))
    assert tf5.translation.x == 7.0
    assert tf5.translation.y == 8.0
    assert tf5.translation.z == 9.0
    assert tf5.rotation.w == 1.0
    tf6 = Transform(rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    assert np.allclose(vector_array(tf6.translation), 0)
    assert tf6.rotation.w == 1.0
    tf7 = Transform(translation=Vector3(x=1, y=2, z=3), rotation=Quaternion())
    assert tf7.translation == Vector3(x=1, y=2, z=3)
    assert tf7.rotation == Quaternion()
    tf8 = Transform(translation=Vector3(x=4, y=5, z=6))
    assert tf8.translation == Vector3(x=4, y=5, z=6)
    assert tf8.rotation.w == 1.0
    tf9 = Transform(rotation=Quaternion(x=0, y=0, z=1, w=0))
    assert np.allclose(vector_array(tf9.translation), 0)
    assert tf9.rotation == Quaternion(x=0, y=0, z=1, w=0)


def test_transform_identity() -> None:
    tf = Transform()
    assert np.allclose(vector_array(tf.translation), 0)
    assert tf.rotation.x == 0.0
    assert tf.rotation.y == 0.0
    assert tf.rotation.z == 0.0
    assert tf.rotation.w == 1.0
    assert tf == Transform()


def test_transform_equality() -> None:
    tf1 = Transform(translation=Vector3(x=1, y=2, z=3), rotation=Quaternion(x=0, y=0, z=0, w=1))
    tf2 = Transform(translation=Vector3(x=1, y=2, z=3), rotation=Quaternion(x=0, y=0, z=0, w=1))
    tf3 = Transform(translation=Vector3(x=1, y=2, z=4), rotation=Quaternion(x=0, y=0, z=0, w=1))
    tf4 = Transform(translation=Vector3(x=1, y=2, z=3), rotation=Quaternion(x=0, y=0, z=1, w=0))
    assert tf1 == tf2
    assert tf1 != tf3
    assert tf1 != tf4
    assert tf1 != "not a transform"


def test_transform_independent_cdr_fields() -> None:
    source = Transform(
        translation=Vector3(x=1.5, y=-2.0, z=3.14), rotation=Quaternion(z=0.707107, w=0.707107)
    )
    decoded = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(source.encode(), Transform.msg_name)
    assert [decoded.translation.x, decoded.translation.y, decoded.translation.z] == [
        1.5,
        -2.0,
        3.14,
    ]
    assert [decoded.rotation.x, decoded.rotation.y, decoded.rotation.z, decoded.rotation.w] == [
        0,
        0,
        0.707107,
        0.707107,
    ]


def test_pose_add_transform() -> None:
    initial_pose = Pose(position=Point(x=1.0, y=0.0, z=0.0))
    angle = np.pi / 2
    transform = Transform(
        translation=Vector3(x=2.0, y=1.0, z=0.0),
        rotation=Quaternion(x=0.0, y=0.0, z=np.sin(angle / 2), w=np.cos(angle / 2)),
    )
    transformed_pose = transform_pose_local(initial_pose, transform)
    assert np.isclose(transformed_pose.position.x, 3.0, atol=1e-10)
    assert np.isclose(transformed_pose.position.y, 1.0, atol=1e-10)
    assert np.isclose(transformed_pose.position.z, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose.orientation.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose.orientation.y, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose.orientation.z, np.sin(angle / 2), atol=1e-10)
    assert np.isclose(transformed_pose.orientation.w, np.cos(angle / 2), atol=1e-10)
    found_tf = relative_pose_transform(initial_pose, transformed_pose)
    assert found_tf == transform


def test_pose_add_transform_with_rotation() -> None:
    angle = np.pi / 2
    initial_pose = Pose(
        position=Point(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion(x=0.0, y=0.0, z=np.sin(angle / 2), w=np.cos(angle / 2)),
    )
    rotation_angle = np.pi / 4
    transform1 = Transform(
        translation=Vector3(x=1.0, y=0.0, z=0.0),
        rotation=Quaternion(
            x=0.0, y=0.0, z=np.sin(rotation_angle / 2), w=np.cos(rotation_angle / 2)
        ),
    )
    transform2 = Transform(
        translation=Vector3(x=0.0, y=1.0, z=1.0), rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    transformed_pose1 = transform_pose_local(initial_pose, transform1)
    transformed_pose2 = transform_pose_local(
        transform_pose_local(initial_pose, transform1), transform2
    )
    assert np.isclose(transformed_pose1.position.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose1.position.y, 1.0, atol=1e-10)
    assert np.isclose(transformed_pose1.position.z, 0.0, atol=1e-10)
    total_angle1 = angle + rotation_angle
    assert np.isclose(transformed_pose1.orientation.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose1.orientation.y, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose1.orientation.z, np.sin(total_angle1 / 2), atol=1e-10)
    assert np.isclose(transformed_pose1.orientation.w, np.cos(total_angle1 / 2), atol=1e-10)
    sqrt2_2 = np.sqrt(2) / 2
    expected_x = 0.0 - sqrt2_2
    expected_y = 1.0 - sqrt2_2
    expected_z = 1.0
    assert np.isclose(transformed_pose2.position.x, expected_x, atol=1e-10)
    assert np.isclose(transformed_pose2.position.y, expected_y, atol=1e-10)
    assert np.isclose(transformed_pose2.position.z, expected_z, atol=1e-10)
    total_angle2 = total_angle1
    assert np.isclose(transformed_pose2.orientation.x, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose2.orientation.y, 0.0, atol=1e-10)
    assert np.isclose(transformed_pose2.orientation.z, np.sin(total_angle2 / 2), atol=1e-10)
    assert np.isclose(transformed_pose2.orientation.w, np.cos(total_angle2 / 2), atol=1e-10)


def test_encode_decode() -> None:
    angle = np.pi / 2
    transform = Transform(
        translation=Vector3(x=2.0, y=1.0, z=0.0),
        rotation=Quaternion(x=0.0, y=0.0, z=np.sin(angle / 2), w=np.cos(angle / 2)),
    )
    data = transform.encode()
    decoded_transform = Transform.decode(data)
    assert decoded_transform == transform


def test_transform_addition() -> None:
    t1 = Transform(translation=Vector3(x=1, y=0, z=0), rotation=Quaternion(x=0, y=0, z=0, w=1))
    t2 = Transform(translation=Vector3(x=2, y=0, z=0), rotation=Quaternion(x=0, y=0, z=0, w=1))
    t3 = compose_transform_values(t1, t2)
    assert t3.translation == Vector3(x=3, y=0, z=0)
    assert t3.rotation == Quaternion(x=0, y=0, z=0, w=1)
    t1 = Transform(translation=Vector3(x=1, y=0, z=0), rotation=Quaternion(x=0, y=0, z=0, w=1))
    angle = np.pi / 2
    t2 = Transform(
        translation=Vector3(x=1, y=0, z=0),
        rotation=Quaternion(x=0, y=0, z=np.sin(angle / 2), w=np.cos(angle / 2)),
    )
    t3 = compose_transform_values(t1, t2)
    assert t3.translation == Vector3(x=2, y=0, z=0)
    assert np.isclose(t3.rotation.x, 0.0, atol=1e-10)
    assert np.isclose(t3.rotation.y, 0.0, atol=1e-10)
    assert np.isclose(t3.rotation.z, np.sin(angle / 2), atol=1e-10)
    assert np.isclose(t3.rotation.w, np.cos(angle / 2), atol=1e-10)
    t1 = Transform(
        translation=Vector3(x=0, y=0, z=0),
        rotation=Quaternion(x=0, y=0, z=np.sin(angle / 2), w=np.cos(angle / 2)),
    )
    t2 = Transform(translation=Vector3(x=1, y=0, z=0), rotation=Quaternion(x=0, y=0, z=0, w=1))
    t3 = compose_transform_values(t1, t2)
    assert np.isclose(t3.translation.x, 0.0, atol=1e-10)
    assert np.isclose(t3.translation.y, 1.0, atol=1e-10)
    assert np.isclose(t3.translation.z, 0.0, atol=1e-10)
    assert np.isclose(t3.rotation.z, np.sin(angle / 2), atol=1e-10)
    assert np.isclose(t3.rotation.w, np.cos(angle / 2), atol=1e-10)


def test_transform_frame_tracking() -> None:
    first = TransformStamped(
        header=Header(frame_id="world"),
        child_frame_id="robot",
        transform=Transform(translation=Vector3(x=1)),
    )
    second = TransformStamped(
        header=Header(frame_id="robot"),
        child_frame_id="sensor",
        transform=Transform(translation=Vector3(x=2)),
    )
    result = compose_transforms(first, second)
    assert result.header.frame_id == "world"
    assert result.child_frame_id == "sensor"
    assert result.transform.translation == Vector3(x=3)
    with pytest.raises(TypeError):
        compose_transform_values(first.transform, "not a transform")


def test_transform_from_pose() -> None:
    pose = Pose(position=Point(x=1, y=2, z=3), orientation=Quaternion(z=0.707, w=0.707))
    result = transform_from_pose(
        PoseStamped(header=Header(frame_id="world"), pose=pose), child_frame_id="base_link"
    )
    np.testing.assert_array_equal(
        vector_array(result.transform.translation), vector_array(pose.position)
    )
    assert result.transform.rotation == pose.orientation
    assert result.header.frame_id == "world"
    assert result.child_frame_id == "base_link"


def test_transform_from_ros() -> None:
    stamp = Time(sec=123, nanosec=456789123)
    first = transform_from_pose(
        PoseStamped(
            header=Header(stamp=stamp, frame_id="base_link"),
            pose=Pose(
                position=Point(x=1, y=-1), orientation=quaternion_from_euler(0, 0, math.pi / 6)
            ),
        ),
        child_frame_id="arm_base_link",
    )
    second = transform_from_pose(
        PoseStamped(
            header=Header(stamp=stamp, frame_id="arm_base_link"),
            pose=Pose(
                position=Point(x=1, y=1), orientation=quaternion_from_euler(0, 0, math.pi / 6)
            ),
        ),
        child_frame_id="end",
    )
    result = compose_transforms(first, second)
    assert result.transform.translation.x == pytest.approx(1.366, abs=1e-3)
    assert result.transform.translation.y == pytest.approx(0.366, abs=1e-3)
    assert result.header.stamp == stamp


def test_transform_from_pose_stamped() -> None:
    pose = PoseStamped(
        header=Header(stamp=Time(sec=123, nanosec=456789123), frame_id="map"),
        pose=Pose(position=Point(x=4, y=5, z=6), orientation=Quaternion(y=0.707, w=0.707)),
    )
    result = transform_from_pose(pose, child_frame_id="robot_base")
    np.testing.assert_array_equal(
        vector_array(result.transform.translation), vector_array(pose.pose.position)
    )
    assert result.transform.rotation == pose.pose.orientation
    assert result.header == pose.header
    assert result.child_frame_id == "robot_base"


@pytest.mark.parametrize("values", [[1.0, 2.0, 3.0], (7.0, 8.0, 9.0), [10.0, 11.0, 12.0]])
def test_transform_from_coordinate_arrays(values) -> None:
    pose = PoseStamped(pose=Pose(position=point_from_array(values)))
    result = transform_from_pose(pose, child_frame_id="base_link")
    np.testing.assert_array_equal(vector_array(result.transform.translation), values)
    assert result.transform.rotation == Quaternion()


@pytest.mark.parametrize("value", ["not a pose", 42, None])
def test_transform_from_pose_invalid_type(value) -> None:
    with pytest.raises(TypeError):
        transform_from_pose(value, child_frame_id="base_link")
