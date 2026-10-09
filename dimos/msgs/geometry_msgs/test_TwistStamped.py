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

import pickle

from dimos_generated.geometry_msgs.msg import Twist, TwistStamped, Vector3
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
from rosbags.typesys import Stores, get_typestore

from dimos.msgs.time import time_from_nanoseconds


def test_cdr_encode_decode() -> None:
    source = TwistStamped(
        header=Header(stamp=time_from_nanoseconds(1234567890123456789), frame_id=""),
        twist=Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.1, y=0.2, z=0.3)),
    )
    binary = cdr_encode(source)
    decoded = cdr_decode(binary, TwistStamped)
    assert isinstance(decoded, TwistStamped)
    assert decoded is not source
    assert cdr_encode(decoded) == binary
    independent = get_typestore(Stores.ROS2_JAZZY).deserialize_cdr(binary, source.__msgtype__)
    assert (independent.header.stamp.sec, independent.header.stamp.nanosec) == (
        1234567890,
        123456789,
    )
    assert (independent.twist.linear.x, independent.twist.linear.y, independent.twist.linear.z) == (
        1,
        2,
        3,
    )
    assert (
        independent.twist.angular.x,
        independent.twist.angular.y,
        independent.twist.angular.z,
    ) == (0.1, 0.2, 0.3)


def test_pickle_encode_decode() -> None:
    source = TwistStamped(
        header=Header(stamp=time_from_nanoseconds(1234567890123456789), frame_id=""),
        twist=Twist(linear=Vector3(x=1, y=2, z=3), angular=Vector3(x=0.1, y=0.2, z=0.3)),
    )
    decoded = pickle.loads(pickle.dumps(source))
    assert isinstance(decoded, TwistStamped)
    assert decoded is not source
    assert decoded.encode() == cdr_encode(source)
