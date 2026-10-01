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

"""Generated field construction replaces polymorphic rich-message constructors.

Array/dictionary inputs are unpacked explicitly. Stamped types contain a header
and a payload; timestamps default to zero and never sample the wall clock.
"""

from copy import deepcopy

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    PointStamped,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
    Twist,
    TwistStamped,
    TwistWithCovariance,
    TwistWithCovarianceStamped,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Odometry, Path
from dimos_generated.sensor_msgs.msg import JointState, Joy
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.geometry import (
    point_from_array,
    quaternion_array,
    quaternion_euler,
    quaternion_from_array,
    vector_array,
    vector_from_array,
)
from dimos.msgs.time import to_nanoseconds


@pytest.mark.parametrize(
    "values", [[1.0, 2.0, 3.0, 4.0], (1, 2, 3, 4), range(4), np.array([1.0, 2.0, 3.0, 4.0])]
)
def test_quaternion_explicit_array_conversion(values) -> None:
    q = quaternion_from_array(values)
    np.testing.assert_array_equal(quaternion_array(q), values)
    assert all(type(value) is float for value in (q.x, q.y, q.z, q.w))
    assert Quaternion.decode(q.encode()) == q


def test_quaternion_defaults_and_partial_fields() -> None:
    assert Quaternion() == Quaternion(x=0, y=0, z=0, w=1)
    assert Quaternion(z=1) == Quaternion(x=0, y=0, z=1, w=1)
    # Explicit zero w is retained, even though it is not a spatial rotation.
    assert Quaternion.decode(Quaternion(w=0).encode()).w == 0


@pytest.mark.parametrize("values", [[1, 2, 3], [1, 2, 3, 4, 5], (1, 2), np.array([1.0, 2.0, 3.0])])
def test_quaternion_array_wrong_size(values) -> None:
    with pytest.raises(ValueError, match="exactly 4 components"):
        quaternion_from_array(values)


@pytest.mark.parametrize("position", [[1, 2, 3], (1, 2, 3), np.array([1.0, 2.0, 3.0])])
@pytest.mark.parametrize(
    "orientation", [[0, 0, 0, 1], (0, 0, 0, 1), np.array([0.0, 0.0, 0.0, 1.0])]
)
def test_pose_explicit_numeric_fields(position, orientation) -> None:
    pose = Pose(position=point_from_array(position), orientation=quaternion_from_array(orientation))
    np.testing.assert_array_equal(vector_array(pose.position), [1, 2, 3])
    np.testing.assert_array_equal(quaternion_array(pose.orientation), [0, 0, 0, 1])
    assert type(pose.position) is Point
    assert type(pose.orientation) is Quaternion


def test_pose_explicit_dictionary_and_pair_unpacking() -> None:
    data = {"position": [1, 2, 3], "orientation": [0, 0, 0, 1]}
    pair = (data["position"], data["orientation"])
    expected = Pose(position=Point(x=1, y=2, z=3))
    assert (
        Pose(
            position=point_from_array(data["position"]),
            orientation=quaternion_from_array(data["orientation"]),
        )
        == expected
    )
    assert (
        Pose(position=point_from_array(pair[0]), orientation=quaternion_from_array(pair[1]))
        == expected
    )
    assert Pose(position=Point(x=5)).position == Point(x=5, y=0, z=0)


@pytest.mark.parametrize("values", [[1, 2, 3], (1, 2, 3), np.array([1.0, 2.0, 3.0])])
def test_twist_explicit_arrays(values) -> None:
    twist = Twist(linear=vector_from_array(values), angular=vector_from_array([4, 5, 6]))
    np.testing.assert_array_equal(vector_array(twist.linear), [1, 2, 3])
    np.testing.assert_array_equal(vector_array(twist.angular), [4, 5, 6])
    assert Twist(linear=twist.linear).angular == Vector3()
    assert Twist(angular=twist.angular).linear == Vector3()


def test_twist_euler_conversion_is_explicit() -> None:
    twist = Twist(
        linear=Vector3(x=1, y=2, z=3), angular=vector_from_array(quaternion_euler(Quaternion()))
    )
    assert twist.linear == Vector3(x=1, y=2, z=3)
    assert twist.angular == Vector3()


@pytest.mark.parametrize(
    "message",
    [
        Quaternion(x=0.1, y=0.2, z=0.3, w=0.4),
        Pose(position=Point(x=1, y=2, z=3)),
        Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=4, y=5, z=6)),
        PoseWithCovariance(pose=Pose(position=Point(x=1, y=2, z=3)), covariance=list(range(36))),
        TwistWithCovariance(twist=Twist(linear=Vector3(x=1, y=2, z=3)), covariance=list(range(36))),
        JointState(header=Header(stamp=Time(sec=5), frame_id="f"), name=["a"], position=[1.0]),
        Joy(header=Header(stamp=Time(sec=5), frame_id="f"), axes=[1.0], buttons=[1]),
    ],
)
def test_copy_and_cdr_roundtrip_preserve_fields(message) -> None:
    copied = deepcopy(message)
    assert copied == message
    assert copied is not message
    assert type(message).decode(message.encode()) == message
    independent = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(
        message.encode(), message.msg_name
    )
    assert independent.__msgtype__ == message.msg_name


def test_copy_mutation_is_independent() -> None:
    pose = Pose(position=Point(x=1, y=2, z=3))
    pose_copy = deepcopy(pose)
    pose_copy.position.x = 99
    assert pose.position.x == 1
    twist = Twist(linear=Vector3(x=1, y=2, z=3))
    twist_copy = deepcopy(twist)
    twist_copy.linear.x = 99
    assert twist.linear.x == 1
    covariance = PoseWithCovariance(covariance=list(range(36)))
    covariance_copy = deepcopy(covariance)
    covariance_copy.covariance[0] = 99
    assert covariance.covariance[0] == 0
    joints = JointState(name=["a"], position=[1.0])
    joints_copy = deepcopy(joints)
    joints_copy.name[0] = "b"
    assert joints.name == ["a"]
    joy = Joy(axes=[1.0], buttons=[1])
    joy_copy = deepcopy(joy)
    joy_copy.axes[0] = 2
    assert joy.axes == [1.0]


@pytest.mark.parametrize("message_type", [PoseWithCovariance, TwistWithCovariance])
def test_covariance_defaults_layout_and_length(message_type) -> None:
    assert list(message_type().covariance) == [0.0] * 36
    covariance = np.arange(36.0).reshape(6, 6)
    message = message_type(covariance=covariance.flatten().tolist())
    np.testing.assert_array_equal(np.asarray(message.covariance).reshape(6, 6), covariance)
    for invalid in ([1, 2, 3], [0.0] * 35, [0.0] * 37):
        with pytest.raises(RuntimeError, match="Unable to cast"):
            message_type(covariance=invalid)


def test_sensor_explicit_fields_and_defaults() -> None:
    joints = JointState(
        header=Header(stamp=Time(sec=5), frame_id="f"),
        name=["a"],
        position=[1.0],
        velocity=[2.0],
        effort=[3.0],
    )
    assert (joints.name, joints.position, joints.velocity, joints.effort) == (
        ["a"],
        [1.0],
        [2.0],
        [3.0],
    )
    assert joints.header == Header(stamp=Time(sec=5), frame_id="f")
    assert JointState().name == []
    assert JointState().velocity == []
    assert JointState().effort == []
    pair = ([1.0, 2.0], [1, 0])
    joy = Joy(axes=pair[0], buttons=pair[1])
    assert (joy.axes, joy.buttons) == pair
    assert (Joy().axes, Joy().buttons) == ([], [])
    assert Joy().header.frame_id == ""


STAMPED_TYPES = [
    PoseStamped,
    TwistStamped,
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
    JointState,
    Joy,
    Path,
    PointStamped,
    TransformStamped,
    Odometry,
]


@pytest.mark.parametrize("message_type", STAMPED_TYPES)
@pytest.mark.parametrize(
    "stamp",
    [Time(), Time(sec=5), Time(sec=-1, nanosec=999999999), Time(sec=1700000000, nanosec=123456789)],
)
def test_exact_stamps_survive_cdr(message_type, stamp) -> None:
    message = message_type(header=Header(stamp=stamp, frame_id="odom"))
    decoded = message_type.decode(message.encode())
    assert decoded.header == message.header
    assert to_nanoseconds(decoded.header.stamp) == stamp.sec * 1000000000 + stamp.nanosec


@pytest.mark.parametrize("message_type", STAMPED_TYPES)
def test_omitted_stamp_is_zero_without_wall_clock(message_type) -> None:
    message = message_type()
    assert message.header.stamp == Time()
    assert message_type.decode(message.encode()).header.stamp == Time()


def test_stamped_payloads_are_nested_values() -> None:
    pose = Pose(position=Point(x=1, y=2, z=3), orientation=Quaternion(z=0.1, w=1))
    stamped = PoseStamped(header=Header(stamp=Time(sec=5), frame_id="odom"), pose=pose)
    assert not isinstance(stamped, Pose)
    assert stamped.pose == pose
    assert deepcopy(stamped.pose) == pose
    twist = Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=4, y=5, z=6))
    assert TwistStamped(twist=twist).twist == twist
    pwc = PoseWithCovarianceStamped(pose=PoseWithCovariance(pose=pose, covariance=list(range(36))))
    assert pwc.pose.pose == pose
    assert list(pwc.pose.covariance) == list(range(36))
    twc = TwistWithCovarianceStamped(
        twist=TwistWithCovariance(twist=twist, covariance=list(range(36)))
    )
    assert twc.twist.twist == twist
    assert list(twc.twist.covariance) == list(range(36))


@pytest.mark.parametrize(
    "message_type",
    [Pose, Quaternion, Twist, PoseWithCovariance, TwistWithCovariance, JointState, Joy],
)
def test_unknown_fields_rejected(message_type) -> None:
    with pytest.raises(TypeError):
        message_type(bogus=1)


@pytest.mark.parametrize(
    "message_type,kwargs",
    [
        (Pose, {"position": [1, 2, 3]}),
        (Pose, {"position": "x"}),
        (Pose, {"orientation": [0, 0, 0, 1]}),
        (Pose, {"position": None}),
        (Twist, {"linear": [1, 2, 3]}),
        (Twist, {"angular": Quaternion()}),
        (PoseStamped, {"ts": 5}),
        (PoseStamped, {"frame_id": "odom"}),
        (PoseStamped, {"position": Point(x=1)}),
    ],
)
def test_retired_polymorphic_fields_rejected(message_type, kwargs) -> None:
    with pytest.raises((TypeError, RuntimeError)):
        message_type(**kwargs)


@pytest.mark.parametrize("values", [[], [1, 2], [1, 2, 3, 4], [[1, 2, 3], [4, 5, 6]]])
def test_position_array_shape_rejected(values) -> None:
    with pytest.raises(ValueError, match="3 vector components"):
        point_from_array(values)
