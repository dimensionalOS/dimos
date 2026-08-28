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

"""Construction semantics for the hot message types.

Every accepted construction form for the types whose ``__init__`` used to be a
``plum`` multiple-dispatch stack, plus the error cases. These pin the contract
that the dispatch removal had to preserve.
"""

import time

from dimos_lcm.geometry_msgs import (
    Point as LCMPoint,
    Pose as LCMPose,
    PoseWithCovariance as LCMPoseWithCovariance,
    Quaternion as LCMQuaternion,
    Twist as LCMTwist,
    TwistWithCovariance as LCMTwistWithCovariance,
    Vector3 as LCMVector3,
)
from dimos_lcm.sensor_msgs import JointState as LCMJointState, Joy as LCMJoy
import numpy as np
import pytest

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.PoseWithCovariance import PoseWithCovariance
from dimos.msgs.geometry_msgs.PoseWithCovarianceStamped import PoseWithCovarianceStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.TwistWithCovariance import TwistWithCovariance
from dimos.msgs.geometry_msgs.TwistWithCovarianceStamped import TwistWithCovarianceStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.Joy import Joy

IDENTITY = (0.0, 0.0, 0.0, 1.0)


# Quaternion


def test_quaternion_empty_is_identity() -> None:
    assert Quaternion().to_tuple() == IDENTITY


def test_quaternion_four_positional_floats() -> None:
    q = Quaternion(1.0, 2.0, 3.0, 4.0)
    assert q.to_tuple() == (1.0, 2.0, 3.0, 4.0)


def test_quaternion_four_positional_ints_are_coerced_to_float() -> None:
    q = Quaternion(1, 2, 3, 4)
    assert q.to_tuple() == (1.0, 2.0, 3.0, 4.0)
    assert all(type(c) is float for c in q.to_tuple())


def test_quaternion_from_list() -> None:
    assert Quaternion([1.0, 2.0, 3.0, 4.0]).to_tuple() == (1.0, 2.0, 3.0, 4.0)


def test_quaternion_from_tuple() -> None:
    assert Quaternion((1.0, 2.0, 3.0, 4.0)).to_tuple() == (1.0, 2.0, 3.0, 4.0)


def test_quaternion_from_range() -> None:
    assert Quaternion(range(4)).to_tuple() == (0.0, 1.0, 2.0, 3.0)


def test_quaternion_from_ndarray() -> None:
    assert Quaternion(np.array([1.0, 2.0, 3.0, 4.0])).to_tuple() == (1.0, 2.0, 3.0, 4.0)


def test_quaternion_copy_constructor() -> None:
    src = Quaternion(0.1, 0.2, 0.3, 0.4)
    assert Quaternion(src).to_tuple() == src.to_tuple()


def test_quaternion_from_lcm_keeps_lcm_defaults() -> None:
    """The LCM base defaults w to 0.0, unlike the dimos identity default."""
    assert Quaternion(LCMQuaternion()).to_tuple() == (0.0, 0.0, 0.0, 0.0)


def test_quaternion_from_lcm_with_values() -> None:
    assert Quaternion(LCMQuaternion(1.0, 2.0, 3.0, 4.0)).to_tuple() == (1.0, 2.0, 3.0, 4.0)


def test_quaternion_sequence_wrong_length_raises() -> None:
    for bad in ([1, 2, 3], [1, 2, 3, 4, 5], (1, 2)):
        with pytest.raises(ValueError, match=r"exactly 4 components"):
            Quaternion(bad)


def test_quaternion_ndarray_wrong_size_raises() -> None:
    with pytest.raises(ValueError, match=r"exactly 4 components"):
        Quaternion(np.array([1.0, 2.0, 3.0]))


def test_quaternion_lcm_roundtrip_uses_the_sequence_form() -> None:
    """``_lcm_decode_one`` builds from a 4-tuple, so decoding exercises that path."""
    decoded = Quaternion.lcm_decode(Quaternion(1.0, 2.0, 3.0, 4.0).lcm_encode())
    assert decoded.to_tuple() == (1.0, 2.0, 3.0, 4.0)


# Pose


def test_pose_empty_is_origin_with_identity() -> None:
    p = Pose()
    assert p.position.to_tuple() == (0.0, 0.0, 0.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_three_positional_floats() -> None:
    p = Pose(1, 2, 3)
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_seven_positional_floats() -> None:
    p = Pose(1, 2, 3, 0, 0, 0, 1)
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_from_position_sequence_only() -> None:
    p = Pose([1, 2, 3])
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_from_position_and_orientation_sequences() -> None:
    p = Pose([1, 2, 3], [0, 0, 0, 1])
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_from_vector3_and_quaternion() -> None:
    p = Pose(Vector3(1, 2, 3), Quaternion(0, 0, 0, 1))
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_from_ndarrays() -> None:
    p = Pose(np.array([1.0, 2.0, 3.0]), np.array([0.0, 0.0, 0.0, 1.0]))
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_keyword_position_only() -> None:
    p = Pose(position=[1, 2, 3])
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_keyword_orientation_only() -> None:
    p = Pose(orientation=[0, 0, 0, 1])
    assert p.position.to_tuple() == (0.0, 0.0, 0.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_keyword_both() -> None:
    p = Pose(position=[1, 2, 3], orientation=[0, 0, 0, 1])
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)


def test_pose_positional_position_with_keyword_orientation() -> None:
    p = Pose([1, 2, 3], orientation=[0, 0, 0, 1])
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_explicit_none_falls_back_to_defaults() -> None:
    p = Pose(None, None)
    assert p.position.to_tuple() == (0.0, 0.0, 0.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_from_pair_tuple() -> None:
    p = Pose(([1, 2, 3], [0, 0, 0, 1]))
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_from_three_tuple_is_a_position() -> None:
    """A 3-tuple of numbers is a position, not a (position, orientation) pair."""
    p = Pose((1, 2, 3))
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)


def test_pose_from_dict() -> None:
    p = Pose({"position": [1, 2, 3], "orientation": [0, 0, 0, 1]})
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_from_dict_missing_key_raises_keyerror() -> None:
    with pytest.raises(KeyError):
        Pose({"position": [1, 2, 3]})
    with pytest.raises(KeyError):
        Pose({})


def test_pose_copy_constructor() -> None:
    src = Pose(1, 2, 3, 0, 0, 0, 1)
    copy = Pose(src)
    assert copy.position.to_tuple() == (1.0, 2.0, 3.0)
    assert copy.position is not src.position


def test_pose_from_posestamped_downcasts_to_pose() -> None:
    src = PoseStamped(ts=5.0, frame_id="odom", position=[1, 2, 3])
    p = Pose(src)
    assert type(p) is Pose
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)


def test_pose_from_lcm() -> None:
    # Build the sub-messages explicitly: the generated LCM classes share one
    # mutable default instance across every construction, so LCMPose().position
    # carries whatever the last writer left there.
    p = Pose(
        LCMPose(position=LCMPoint(1.0, 2.0, 3.0), orientation=LCMQuaternion(0.0, 0.0, 0.0, 1.0))
    )
    assert p.position.to_tuple() == (1.0, 2.0, 3.0)
    assert p.orientation.to_tuple() == IDENTITY


def test_pose_position_and_orientation_are_wrapper_types() -> None:
    p = Pose(1, 2, 3)
    assert isinstance(p.position, Vector3)
    assert isinstance(p.orientation, Quaternion)


# Twist


def test_twist_empty_is_zero() -> None:
    t = Twist()
    assert t.linear.to_tuple() == (0.0, 0.0, 0.0)
    assert t.angular.to_tuple() == (0.0, 0.0, 0.0)


def test_twist_from_two_sequences() -> None:
    t = Twist([1, 2, 3], [4, 5, 6])
    assert t.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert t.angular.to_tuple() == (4.0, 5.0, 6.0)


def test_twist_from_two_vector3() -> None:
    t = Twist(Vector3(1, 2, 3), Vector3(4, 5, 6))
    assert t.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert t.angular.to_tuple() == (4.0, 5.0, 6.0)


def test_twist_from_ndarrays() -> None:
    t = Twist(np.array([1.0, 2.0, 3.0]), np.array([4.0, 5.0, 6.0]))
    assert t.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert t.angular.to_tuple() == (4.0, 5.0, 6.0)


def test_twist_angular_quaternion_is_converted_to_euler() -> None:
    t = Twist([1, 2, 3], Quaternion(0, 0, 0, 1))
    assert t.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert t.angular.to_tuple() == (0.0, 0.0, 0.0)


def test_twist_copy_constructor() -> None:
    src = Twist([1, 2, 3], [4, 5, 6])
    copy = Twist(src)
    assert copy.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert copy.linear is not src.linear


def test_twist_from_lcm() -> None:
    t = Twist(LCMTwist(linear=LCMVector3(1.0, 2.0, 3.0), angular=LCMVector3(4.0, 5.0, 6.0)))
    assert t.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert t.angular.to_tuple() == (4.0, 5.0, 6.0)


def test_twist_from_twiststamped_downcasts() -> None:
    t = Twist(TwistStamped(linear=[1, 2, 3], angular=[4, 5, 6]))
    assert type(t) is Twist
    assert t.linear.to_tuple() == (1.0, 2.0, 3.0)


def test_twist_keyword_both() -> None:
    t = Twist(linear=[1, 2, 3], angular=[4, 5, 6])
    assert t.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert t.angular.to_tuple() == (4.0, 5.0, 6.0)


def test_twist_keyword_linear_only() -> None:
    t = Twist(linear=[1, 2, 3])
    assert t.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert t.angular.to_tuple() == (0.0, 0.0, 0.0)


def test_twist_keyword_angular_only() -> None:
    t = Twist(angular=[4, 5, 6])
    assert t.linear.to_tuple() == (0.0, 0.0, 0.0)
    assert t.angular.to_tuple() == (4.0, 5.0, 6.0)


def test_twist_keyword_angular_quaternion_is_converted_to_euler() -> None:
    t = Twist(linear=[1, 2, 3], angular=Quaternion(0, 0, 0, 1))
    assert t.angular.to_tuple() == (0.0, 0.0, 0.0)


# PoseWithCovariance


def test_pwc_empty() -> None:
    p = PoseWithCovariance()
    assert p.pose.position.to_tuple() == (0.0, 0.0, 0.0)
    assert np.array_equal(p.covariance, np.zeros(36))


def test_pwc_from_pose_only() -> None:
    p = PoseWithCovariance(Pose(1, 2, 3))
    assert p.pose.position.to_tuple() == (1.0, 2.0, 3.0)
    assert np.array_equal(p.covariance, np.zeros(36))


def test_pwc_from_pose_and_covariance() -> None:
    p = PoseWithCovariance(Pose(1, 2, 3), np.arange(36.0))
    assert np.array_equal(p.covariance, np.arange(36.0))


def test_pwc_explicit_none_covariance_is_zeros() -> None:
    assert np.array_equal(PoseWithCovariance(Pose(1, 2, 3), None).covariance, np.zeros(36))


def test_pwc_covariance_wrong_size_raises() -> None:
    with pytest.raises(ValueError, match=r"reshape"):
        PoseWithCovariance(Pose(1, 2, 3), [1, 2, 3])


def test_pwc_copy_constructor_copies_covariance() -> None:
    src = PoseWithCovariance(Pose(1, 2, 3), np.arange(36.0))
    copy = PoseWithCovariance(src)
    assert np.array_equal(copy.covariance, src.covariance)
    assert copy.covariance is not src.covariance


def test_pwc_from_lcm() -> None:
    lcm_pose = LCMPose(
        position=LCMPoint(1.0, 2.0, 3.0), orientation=LCMQuaternion(0.0, 0.0, 0.0, 1.0)
    )
    p = PoseWithCovariance(LCMPoseWithCovariance(pose=lcm_pose, covariance=list(range(36))))
    assert p.pose.position.to_tuple() == (1.0, 2.0, 3.0)
    assert np.array_equal(p.covariance, np.arange(36.0))


def test_pwc_from_dict_with_pose_object() -> None:
    p = PoseWithCovariance({"pose": Pose(1, 2, 3)})
    assert p.pose.position.to_tuple() == (1.0, 2.0, 3.0)
    assert np.array_equal(p.covariance, np.zeros(36))


def test_pwc_from_dict_with_covariance() -> None:
    p = PoseWithCovariance({"pose": Pose(1, 2, 3), "covariance": list(range(36))})
    assert np.array_equal(p.covariance, np.arange(36.0))


def test_pwc_covariance_matrix_view() -> None:
    p = PoseWithCovariance(Pose(), np.arange(36.0))
    assert p.covariance_matrix.shape == (6, 6)


# TwistWithCovariance


def test_twc_empty() -> None:
    t = TwistWithCovariance()
    assert t.twist.linear.to_tuple() == (0.0, 0.0, 0.0)
    assert np.array_equal(t.covariance, np.zeros(36))


def test_twc_from_twist_only() -> None:
    t = TwistWithCovariance(Twist([1, 2, 3], [4, 5, 6]))
    assert t.twist.linear.to_tuple() == (1.0, 2.0, 3.0)


def test_twc_from_twist_and_covariance() -> None:
    t = TwistWithCovariance(Twist([1, 2, 3], [4, 5, 6]), np.arange(36.0))
    assert np.array_equal(t.covariance, np.arange(36.0))


def test_twc_from_linear_angular_pair() -> None:
    t = TwistWithCovariance(([1, 2, 3], [4, 5, 6]))
    assert t.twist.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert t.twist.angular.to_tuple() == (4.0, 5.0, 6.0)


def test_twc_copy_constructor() -> None:
    src = TwistWithCovariance(Twist([1, 2, 3], [4, 5, 6]), np.arange(36.0))
    copy = TwistWithCovariance(src)
    assert np.array_equal(copy.covariance, src.covariance)
    assert copy.covariance is not src.covariance


def test_twc_from_lcm() -> None:
    lcm_twist = LCMTwist(linear=LCMVector3(1.0, 2.0, 3.0), angular=LCMVector3(4.0, 5.0, 6.0))
    t = TwistWithCovariance(LCMTwistWithCovariance(twist=lcm_twist, covariance=list(range(36))))
    assert t.twist.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert np.array_equal(t.covariance, np.arange(36.0))


def test_twc_from_dict() -> None:
    t = TwistWithCovariance({"twist": Twist([1, 2, 3], [4, 5, 6])})
    assert t.twist.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert np.array_equal(t.covariance, np.zeros(36))


def test_twc_from_dict_with_covariance() -> None:
    t = TwistWithCovariance({"twist": Twist([1, 2, 3], [4, 5, 6]), "covariance": list(range(36))})
    assert np.array_equal(t.covariance, np.arange(36.0))


# JointState


def test_jointstate_empty_has_empty_lists() -> None:
    j = JointState()
    assert (j.name, j.position, j.velocity, j.effort) == ([], [], [], [])
    assert j.frame_id == ""


def test_jointstate_keywords() -> None:
    j = JointState(ts=5.0, frame_id="f", name=["a"], position=[1.0])
    assert (j.ts, j.frame_id, j.name, j.position) == (5.0, "f", ["a"], [1.0])
    assert (j.velocity, j.effort) == ([], [])


def test_jointstate_all_positional() -> None:
    j = JointState(5.0, "f", ["a"], [1.0], [2.0], [3.0])
    assert (j.name, j.position, j.velocity, j.effort) == (["a"], [1.0], [2.0], [3.0])


def test_jointstate_from_dict() -> None:
    j = JointState({"name": ["a"], "position": [1.0]})
    assert (j.name, j.position) == (["a"], [1.0])


def test_jointstate_from_empty_dict() -> None:
    j = JointState({})
    assert (j.name, j.position, j.velocity, j.effort) == ([], [], [], [])


def test_jointstate_copy_constructor_copies_lists() -> None:
    src = JointState(ts=5.0, frame_id="f", name=["a"], position=[1.0])
    copy = JointState(src)
    assert copy.ts == 5.0
    assert copy.name == ["a"]
    assert copy.name is not src.name


def test_jointstate_from_lcm() -> None:
    j = JointState(LCMJointState())
    assert j.name == []


# Joy


def test_joy_empty() -> None:
    j = Joy()
    assert (j.axes, j.buttons, j.frame_id) == ([], [], "")


def test_joy_keywords() -> None:
    j = Joy(ts=5.0, frame_id="f", axes=[1.0], buttons=[1])
    assert (j.ts, j.frame_id, j.axes, j.buttons) == (5.0, "f", [1.0], [1])


def test_joy_all_positional() -> None:
    j = Joy(5.0, "f", [1.0], [1])
    assert (j.axes, j.buttons) == ([1.0], [1])


def test_joy_from_axes_buttons_pair() -> None:
    j = Joy(([1.0, 2.0], [1, 0]))
    assert (j.axes, j.buttons) == ([1.0, 2.0], [1, 0])


def test_joy_from_dict() -> None:
    j = Joy({"axes": [1.0], "buttons": [1]})
    assert (j.axes, j.buttons) == ([1.0], [1])


def test_joy_from_empty_dict() -> None:
    j = Joy({})
    assert (j.axes, j.buttons) == ([], [])


def test_joy_copy_constructor_copies_lists() -> None:
    src = Joy(ts=5.0, axes=[1.0], buttons=[1])
    copy = Joy(src)
    assert copy.ts == 5.0
    assert copy.axes == [1.0]
    assert copy.axes is not src.axes


def test_joy_from_lcm() -> None:
    j = Joy(LCMJoy())
    assert j.axes == []


# Stamped wrappers: ts / frame_id plumbing


def test_posestamped_defaults_stamp_to_now() -> None:
    before = time.time()
    ps = PoseStamped()
    assert before <= ps.ts <= time.time()
    assert ps.frame_id == ""
    assert ps.position.to_tuple() == (0.0, 0.0, 0.0)


def test_posestamped_keeps_explicit_ts_and_frame() -> None:
    ps = PoseStamped(ts=5.0, frame_id="odom", position=[1, 2, 3], orientation=[0, 0, 0, 1])
    assert (ps.ts, ps.frame_id) == (5.0, "odom")
    assert ps.position.to_tuple() == (1.0, 2.0, 3.0)


def test_posestamped_positional_ts_and_frame() -> None:
    ps = PoseStamped(5.0, "odom")
    assert (ps.ts, ps.frame_id) == (5.0, "odom")


def test_posestamped_is_a_pose() -> None:
    assert isinstance(PoseStamped(), Pose)


def test_posestamped_inherits_pose_positional_forms() -> None:
    """PoseStamped(x, y, z) and the 7-float form come from Pose."""
    assert PoseStamped(3.2, 1.5, 0.0).position.to_tuple() == (3.2, 1.5, 0.0)
    seven = PoseStamped(1, 2, 0, 0, 0, 0.1, 1)
    assert seven.position.to_tuple() == (1.0, 2.0, 0.0)
    assert seven.orientation.to_tuple() == (0.0, 0.0, 0.1, 1.0)


def test_posestamped_inherits_pose_sequence_forms() -> None:
    ps = PoseStamped([1, 2, 3], [0, 0, 0, 1])
    assert ps.position.to_tuple() == (1.0, 2.0, 3.0)
    assert ps.orientation.to_tuple() == IDENTITY


def test_twiststamped_inherits_twist_positional_form() -> None:
    ts = TwistStamped([1, 2, 3], [4, 5, 6])
    assert ts.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert ts.angular.to_tuple() == (4.0, 5.0, 6.0)


def test_twiststamped_keeps_explicit_ts_and_frame() -> None:
    ts = TwistStamped(ts=5.0, frame_id="base", linear=[1, 2, 3], angular=[4, 5, 6])
    assert (ts.ts, ts.frame_id) == (5.0, "base")
    assert ts.linear.to_tuple() == (1.0, 2.0, 3.0)


def test_twiststamped_defaults_stamp_to_now() -> None:
    before = time.time()
    t = TwistStamped()
    assert before <= t.ts <= time.time()


def test_pwcs_keeps_explicit_ts_and_frame() -> None:
    p = PoseWithCovarianceStamped(ts=5.0, frame_id="map", pose=Pose(1, 2, 3))
    assert (p.ts, p.frame_id) == (5.0, "map")
    assert p.pose.position.to_tuple() == (1.0, 2.0, 3.0)


def test_pwcs_with_covariance() -> None:
    p = PoseWithCovarianceStamped(ts=5.0, pose=Pose(1, 2, 3), covariance=list(range(36)))
    assert np.array_equal(p.covariance, np.arange(36.0))


def test_pwcs_empty_defaults() -> None:
    p = PoseWithCovarianceStamped()
    assert p.pose.position.to_tuple() == (0.0, 0.0, 0.0)
    assert np.array_equal(p.covariance, np.zeros(36))


def test_twcs_keeps_explicit_ts_and_frame() -> None:
    t = TwistWithCovarianceStamped(ts=5.0, frame_id="base", twist=Twist([1, 2, 3], [4, 5, 6]))
    assert (t.ts, t.frame_id) == (5.0, "base")
    assert t.twist.linear.to_tuple() == (1.0, 2.0, 3.0)


def test_twcs_with_covariance() -> None:
    t = TwistWithCovarianceStamped(
        ts=5.0, twist=Twist([1, 2, 3], [4, 5, 6]), covariance=list(range(36))
    )
    assert np.array_equal(t.covariance, np.arange(36.0))


def test_twcs_empty_defaults() -> None:
    t = TwistWithCovarianceStamped()
    assert t.twist.linear.to_tuple() == (0.0, 0.0, 0.0)


# The ``ts`` sentinel
#
# ``ts`` defaults to None and means "stamp me now". 0.0 is an ordinary
# timestamp and is stored as given — it used to be swallowed by a
# ``ts if ts != 0`` sentinel that made it unrepresentable.

STAMPED_TYPES = [
    PoseStamped,
    TwistStamped,
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
    JointState,
    Joy,
    Path,
    PointStamped,
    Transform,
    Odometry,
]


@pytest.mark.parametrize("msg_type", STAMPED_TYPES, ids=lambda t: t.__name__)
def test_zero_ts_is_a_real_timestamp(msg_type: type) -> None:
    assert msg_type(ts=0.0).ts == 0.0


@pytest.mark.parametrize("msg_type", STAMPED_TYPES, ids=lambda t: t.__name__)
def test_omitted_ts_stamps_now(msg_type: type) -> None:
    before = time.time()
    assert before <= msg_type().ts <= time.time()


@pytest.mark.parametrize("msg_type", STAMPED_TYPES, ids=lambda t: t.__name__)
def test_explicit_ts_is_honoured(msg_type: type) -> None:
    assert msg_type(ts=5.0).ts == 5.0


def test_lcm_decode_of_an_unstamped_message_keeps_the_zero_stamp() -> None:
    """A producer that never set the header now decodes to ts=0.0, not receive-time."""
    decoded = PoseStamped.lcm_decode(PoseStamped(ts=0.0, frame_id="odom").lcm_encode())
    assert decoded.ts == 0.0


# Forms that used to build a silently-broken object
#
# Under plum these calls matched no overload and fell through to the LCM base
# __init__, which stores its arguments verbatim — leaving fields holding raw
# scalars/lists instead of Vector3/Quaternion/Pose, to blow up later in
# __repr__ or on encode. They now either do the obvious thing or raise.


def test_pose_from_scalar_is_an_x_only_position() -> None:
    """Vector3(x) means (x, 0, 0), so Pose(5) is a position, not a raw 5."""
    assert Pose(5).position.to_tuple() == (5.0, 0.0, 0.0)


def test_pose_from_string_raises() -> None:
    with pytest.raises(TypeError):
        Pose("x")


def test_pose_from_two_numeric_positionals_raises() -> None:
    """(1, 2) is a position and an orientation, and 2 is not a quaternion."""
    with pytest.raises(TypeError):
        Pose(1, 2)


def test_pose_from_three_tuple_of_sequences_raises() -> None:
    with pytest.raises((TypeError, ValueError)):
        Pose(([1, 2, 3], [0, 0, 0, 1], [9]))


def test_quaternion_partial_positionals_raise() -> None:
    """A quaternion needs all four components; 3 no longer zero-fills w."""
    for bad in ((1, 2, 3), (1, 2), (5,)):
        with pytest.raises((TypeError, ValueError)):
            Quaternion(*bad)


def test_quaternion_accepts_field_keywords() -> None:
    assert Quaternion(x=1, y=2, z=3, w=4).to_tuple() == (1.0, 2.0, 3.0, 4.0)


def test_quaternion_partial_field_keywords_default_to_identity() -> None:
    assert Quaternion(z=1).to_tuple() == (0.0, 0.0, 1.0, 1.0)


def test_quaternion_unknown_keyword_raises() -> None:
    with pytest.raises(TypeError):
        Quaternion(bogus=1)


def test_twist_from_single_sequence_is_linear_only() -> None:
    t = Twist([1, 2, 3])
    assert t.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert t.angular.to_tuple() == (0.0, 0.0, 0.0)


def test_twist_from_scalars_raises() -> None:
    with pytest.raises(TypeError):
        Twist("x", "y")


def test_twist_mixed_positional_and_keyword_converts_both() -> None:
    t = Twist([1, 2, 3], angular=[4, 5, 6])
    assert isinstance(t.linear, Vector3)
    assert t.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert t.angular.to_tuple() == (4.0, 5.0, 6.0)


def test_twist_unknown_keyword_raises() -> None:
    with pytest.raises(TypeError):
        Twist(bogus=1)


def test_pose_unknown_keyword_raises() -> None:
    with pytest.raises(TypeError):
        Pose(bogus=1)


def test_pwc_pair_tuple_is_a_pose_and_a_covariance() -> None:
    """The documented (pose, covariance) tuple form now actually fires."""
    p = PoseWithCovariance((Pose(1, 2, 3), list(range(36))))
    assert p.pose.position.to_tuple() == (1.0, 2.0, 3.0)
    assert np.array_equal(p.covariance, np.arange(36.0))


def test_pwc_accepts_field_keywords() -> None:
    p = PoseWithCovariance(pose=Pose(1, 2, 3), covariance=list(range(36)))
    assert p.pose.position.to_tuple() == (1.0, 2.0, 3.0)
    assert np.array_equal(p.covariance, np.arange(36.0))


def test_twc_accepts_field_keywords() -> None:
    t = TwistWithCovariance(twist=Twist([1, 2, 3], [4, 5, 6]))
    assert t.twist.linear.to_tuple() == (1.0, 2.0, 3.0)


def test_twc_pair_tuple_is_a_twist_and_a_covariance() -> None:
    t = TwistWithCovariance((Twist([1, 2, 3], [4, 5, 6]), list(range(36))))
    assert t.twist.linear.to_tuple() == (1.0, 2.0, 3.0)
    assert np.array_equal(t.covariance, np.arange(36.0))


def test_pwc_dict_pose_may_be_any_pose_convertable() -> None:
    p = PoseWithCovariance({"pose": [1, 2, 3], "covariance": list(range(36))})
    assert p.pose.position.to_tuple() == (1.0, 2.0, 3.0)
    assert np.array_equal(p.covariance, np.arange(36.0))


def test_jointstate_dict_carries_ts_and_frame_id() -> None:
    j = JointState({"ts": 5.0, "frame_id": "f"})
    assert (j.ts, j.frame_id) == (5.0, "f")
    assert j.name == []


def test_joy_dict_carries_ts_and_frame_id() -> None:
    j = Joy({"ts": 5.0, "frame_id": "f"})
    assert (j.ts, j.frame_id) == (5.0, "f")
    assert j.axes == []


def test_posestamped_string_positional_is_a_timestamp_not_a_position() -> None:
    """A str first positional is a (bogus) ts; it no longer corrupts the position."""
    assert PoseStamped("f").position.to_tuple() == (0.0, 0.0, 0.0)
