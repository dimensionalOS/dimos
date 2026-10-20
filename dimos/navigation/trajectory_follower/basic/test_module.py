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

import pytest

from dimos.core.cdr_transport import CdrTransport
from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.time import time_from_seconds
from dimos.navigation.trajectory_follower.basic.module import BasicPathFollower, lookahead_distance
from dimos.protocol.rpc.pubsubrpc import LCMRPC

MOUNT_Z = 0.163


class MemoryRPC(LCMRPC):
    def __init__(self, **kwargs):
        super().__init__(url="memq://", **kwargs)


@pytest.fixture
def module_and_tf():
    module = BasicPathFollower(rpc_transport=MemoryRPC)
    wire = CdrTransport("basic_tf", module.tf.type, backend="lcm", url="memq://")
    module.tf.transport = wire
    try:
        yield module, module.tfbuffer
    finally:
        module.stop()
        wire.stop()


def _mount():
    return TransformStamped(
        header=Header(frame_id="base_link", stamp=time_from_seconds(1.0)),
        child_frame_id="mid360_link",
        transform=Transform(
            translation=Vector3(0.0, 0.0, MOUNT_Z), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
        ),
    )


def _odom_edge():
    return TransformStamped(
        header=Header(frame_id="odom", stamp=time_from_seconds(1.0)),
        child_frame_id="mid360_link",
        transform=Transform(
            translation=Vector3(1.0, 2.0, 3.0), rotation=Quaternion(0.0, 0.0, 0.0, 1.0)
        ),
    )


def test_lookup_pose_steers_from_the_tf_base_pose(module_and_tf):
    module, tf = module_and_tf
    tf.receive_transform(_mount(), _odom_edge())
    pose = module._lookup_pose()
    assert pose is not None
    assert abs(pose.pose.position.z - (3.0 - MOUNT_Z)) < 1e-9


def test_lookup_pose_is_none_without_the_mount_tf(module_and_tf):
    module, tf = module_and_tf
    tf.receive_transform(_odom_edge())
    assert module._lookup_pose() is None


def test_lookup_retries_are_throttled_during_an_outage(module_and_tf, mocker):
    module, tf = module_and_tf
    lookup = mocker.spy(tf, "get_pose")
    assert module._lookup_pose() is None
    assert module._lookup_pose() is None
    assert lookup.call_count == 1
    tf.receive_transform(_mount(), _odom_edge())
    module._next_lookup = 0.0
    assert module._lookup_pose() is not None


def test_lookahead_floor_at_low_speed() -> None:
    assert lookahead_distance(0.1, 1.5, 0.4, 1.5) == 0.4


def test_lookahead_scales_in_linear_region() -> None:
    assert lookahead_distance(0.5, 1.5, 0.4, 1.5) == 0.75


def test_lookahead_clamped_at_ceiling() -> None:
    assert lookahead_distance(2.0, 1.5, 0.4, 1.5) == 1.5
