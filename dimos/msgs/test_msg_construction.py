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
    Transform,
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
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest
from rosbags.serde.errors import SerdeError
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
    assert cdr_decode(cdr_encode(q), Quaternion) == q


def test_quaternion_defaults_and_partial_fields() -> None:
    assert Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) == Quaternion(x=0, y=0, z=0, w=1)
    assert Quaternion(z=1, x=0.0, y=0.0, w=1.0) == Quaternion(x=0, y=0, z=1, w=1)
    # Explicit zero w is retained, even though it is not a spatial rotation.
    assert cdr_decode(cdr_encode(Quaternion(w=0, x=0.0, y=0.0, z=0.0)), Quaternion).w == 0


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
    expected = Pose(
        position=Point(x=1, y=2, z=3), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
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
    assert Pose(
        position=Point(x=5, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    ).position == Point(x=5, y=0, z=0)


@pytest.mark.parametrize("values", [[1, 2, 3], (1, 2, 3), np.array([1.0, 2.0, 3.0])])
def test_twist_explicit_arrays(values) -> None:
    twist = Twist(linear=vector_from_array(values), angular=vector_from_array([4, 5, 6]))
    np.testing.assert_array_equal(vector_array(twist.linear), [1, 2, 3])
    np.testing.assert_array_equal(vector_array(twist.angular), [4, 5, 6])
    assert Twist(linear=twist.linear, angular=Vector3(x=0.0, y=0.0, z=0.0)).angular == Vector3(
        x=0.0, y=0.0, z=0.0
    )
    assert Twist(angular=twist.angular, linear=Vector3(x=0.0, y=0.0, z=0.0)).linear == Vector3(
        x=0.0, y=0.0, z=0.0
    )


def test_twist_euler_conversion_is_explicit() -> None:
    twist = Twist(
        linear=Vector3(x=1, y=2, z=3),
        angular=vector_from_array(quaternion_euler(Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))),
    )
    assert twist.linear == Vector3(x=1, y=2, z=3)
    assert twist.angular == Vector3(x=0.0, y=0.0, z=0.0)


@pytest.mark.parametrize(
    "message",
    [
        Quaternion(x=0.1, y=0.2, z=0.3, w=0.4),
        Pose(position=Point(x=1, y=2, z=3), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=4, y=5, z=6)),
        PoseWithCovariance(
            pose=Pose(
                position=Point(x=1, y=2, z=3), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
            covariance=np.asarray(list(range(36)), dtype=np.float64),
        ),
        TwistWithCovariance(
            twist=Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.0, y=0.0, z=0.0)),
            covariance=np.asarray(list(range(36)), dtype=np.float64),
        ),
        JointState(
            header=Header(stamp=Time(sec=5, nanosec=0), frame_id="f"),
            name=["a"],
            position=np.array([1.0], dtype=np.float64),
            velocity=np.array([], dtype=np.float64),
            effort=np.array([], dtype=np.float64),
        ),
        Joy(
            header=Header(stamp=Time(sec=5, nanosec=0), frame_id="f"),
            axes=np.array([1.0], dtype=np.float32),
            buttons=np.array([1], dtype=np.int32),
        ),
    ],
)
def test_copy_and_cdr_roundtrip_preserve_fields(message) -> None:
    copied = deepcopy(message)
    assert cdr_encode(copied) == cdr_encode(message)
    assert copied is not message
    assert cdr_encode(cdr_decode(cdr_encode(message), type(message))) == cdr_encode(message)
    independent = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(
        cdr_encode(message), message.__msgtype__
    )
    assert independent.__msgtype__ == message.__msgtype__


def test_copy_mutation_is_independent() -> None:
    pose = Pose(position=Point(x=1, y=2, z=3), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    pose_copy = deepcopy(pose)
    pose_copy.position.x = 99
    assert pose.position.x == 1
    twist = Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.0, y=0.0, z=0.0))
    twist_copy = deepcopy(twist)
    twist_copy.linear.x = 99
    assert twist.linear.x == 1
    covariance = PoseWithCovariance(
        covariance=np.asarray(list(range(36)), dtype=np.float64),
        pose=Pose(
            position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        ),
    )
    covariance_copy = deepcopy(covariance)
    covariance_copy.covariance[0] = 99
    assert covariance.covariance[0] == 0
    joints = JointState(
        name=["a"],
        position=np.array([1.0], dtype=np.float64),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        velocity=np.array([], dtype=np.float64),
        effort=np.array([], dtype=np.float64),
    )
    joints_copy = deepcopy(joints)
    joints_copy.name[0] = "b"
    assert joints.name == ["a"]
    joy = Joy(
        axes=np.array([1.0], dtype=np.float32),
        buttons=np.array([1], dtype=np.int32),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    joy_copy = deepcopy(joy)
    joy_copy.axes[0] = 2
    assert joy.axes == [1.0]


@pytest.mark.parametrize("message_type", [PoseWithCovariance, TwistWithCovariance])
def test_covariance_layout_and_length_at_native_codec_boundary(message_type) -> None:
    payload = (
        {"pose": Pose(position=Point(0.0, 0.0, 0.0), orientation=Quaternion(0.0, 0.0, 0.0, 1.0))}
        if message_type is PoseWithCovariance
        else {"twist": Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))}
    )
    covariance = np.arange(36.0).reshape(6, 6)
    message = message_type(**payload, covariance=covariance.flatten())
    restored = cdr_decode(cdr_encode(message), message_type)
    np.testing.assert_array_equal(restored.covariance.reshape(6, 6), covariance)
    for invalid in ([1, 2, 3], [0.0] * 35, [0.0] * 37):
        with pytest.raises(SerdeError, match="array length"):
            cdr_encode(message_type(**payload, covariance=np.array(invalid, dtype=np.float64)))


def test_sensor_explicit_fields_and_empty_arrays() -> None:
    header = Header(stamp=Time(sec=5, nanosec=0), frame_id="f")
    joints = JointState(header, ["a"], np.array([1.0]), np.array([2.0]), np.array([3.0]))
    restored = cdr_decode(cdr_encode(joints), JointState)
    assert restored.header == header and restored.name == ["a"]
    for actual, expected in [
        (restored.position, [1.0]),
        (restored.velocity, [2.0]),
        (restored.effort, [3.0]),
    ]:
        np.testing.assert_array_equal(actual, expected)
    joy = Joy(header, np.array([1.0, 2.0], dtype=np.float32), np.array([1, 0], dtype=np.int32))
    restored_joy = cdr_decode(cdr_encode(joy), Joy)
    np.testing.assert_array_equal(restored_joy.axes, [1.0, 2.0])
    np.testing.assert_array_equal(restored_joy.buttons, [1, 0])
    empty = Joy(header, np.array([], dtype=np.float32), np.array([], dtype=np.int32))
    restored_empty = cdr_decode(cdr_encode(empty), Joy)
    assert restored_empty.axes.size == restored_empty.buttons.size == 0
    assert restored_empty.header == header


def stamped_message(message_type, header):
    pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
    twist = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    pose_covariance = PoseWithCovariance(pose, np.zeros(36, dtype=np.float64))
    twist_covariance = TwistWithCovariance(twist, np.zeros(36, dtype=np.float64))
    fields = {
        PoseStamped: {"pose": pose},
        TwistStamped: {"twist": twist},
        PoseWithCovarianceStamped: {"pose": pose_covariance},
        TwistWithCovarianceStamped: {"twist": twist_covariance},
        JointState: {
            "name": [],
            "position": np.array([], dtype=np.float64),
            "velocity": np.array([], dtype=np.float64),
            "effort": np.array([], dtype=np.float64),
        },
        Joy: {"axes": np.array([], dtype=np.float32), "buttons": np.array([], dtype=np.int32)},
        Path: {"poses": []},
        PointStamped: {"point": pose.position},
        TransformStamped: {
            "child_frame_id": "",
            "transform": Transform(Vector3(0.0, 0.0, 0.0), pose.orientation),
        },
        Odometry: {"child_frame_id": "", "pose": pose_covariance, "twist": twist_covariance},
    }
    return message_type(header=header, **fields[message_type])


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
    [
        Time(sec=0, nanosec=0),
        Time(sec=5, nanosec=0),
        Time(sec=-1, nanosec=999999999),
        Time(sec=1700000000, nanosec=123456789),
    ],
)
def test_exact_stamps_survive_cdr(message_type, stamp) -> None:
    message = stamped_message(message_type, Header(stamp=stamp, frame_id="odom"))
    decoded = cdr_decode(cdr_encode(message), message_type)
    assert decoded.header == message.header
    assert to_nanoseconds(decoded.header.stamp) == stamp.sec * 1000000000 + stamp.nanosec


@pytest.mark.parametrize("message_type", STAMPED_TYPES)
def test_native_constructor_requires_fields_and_explicit_zero_never_samples_wall_clock(
    message_type,
) -> None:
    with pytest.raises(TypeError):
        message_type()
    message = stamped_message(message_type, Header(stamp=Time(0, 0), frame_id=""))
    assert message.header.stamp == Time(sec=0, nanosec=0)
    assert cdr_decode(cdr_encode(message), message_type).header.stamp == Time(sec=0, nanosec=0)


def test_stamped_payloads_are_nested_values() -> None:
    pose = Pose(position=Point(x=1, y=2, z=3), orientation=Quaternion(z=0.1, w=1, x=0.0, y=0.0))
    stamped = PoseStamped(header=Header(stamp=Time(sec=5, nanosec=0), frame_id="odom"), pose=pose)
    assert not isinstance(stamped, Pose)
    assert stamped.pose == pose
    assert deepcopy(stamped.pose) == pose
    twist = Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=4, y=5, z=6))
    assert (
        TwistStamped(twist=twist, header=Header(stamp=Time(sec=0, nanosec=0), frame_id="")).twist
        == twist
    )
    pwc = PoseWithCovarianceStamped(
        pose=PoseWithCovariance(
            pose=pose, covariance=np.asarray(list(range(36)), dtype=np.float64)
        ),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    assert pwc.pose.pose == pose
    assert list(pwc.pose.covariance) == list(range(36))
    twc = TwistWithCovarianceStamped(
        twist=TwistWithCovariance(
            twist=twist, covariance=np.asarray(list(range(36)), dtype=np.float64)
        ),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
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
        (Twist, {"angular": Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)}),
        (PoseStamped, {"ts": 5}),
        (PoseStamped, {"frame_id": "odom"}),
        (PoseStamped, {"position": Point(x=1, y=0.0, z=0.0)}),
    ],
)
def test_retired_polymorphic_fields_rejected(message_type, kwargs) -> None:
    with pytest.raises((TypeError, RuntimeError)):
        message_type(**kwargs)


@pytest.mark.parametrize("values", [[], [1, 2], [1, 2, 3, 4], [[1, 2, 3], [4, 5, 6]]])
def test_position_array_shape_rejected(values) -> None:
    with pytest.raises(ValueError, match="3 vector components"):
        point_from_array(values)
