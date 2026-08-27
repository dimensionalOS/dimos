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

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.basic_path_follower.module import BasicPathFollower, lookahead_distance
from dimos.protocol.tf.tf import MultiTBuffer

MOUNT_Z = 0.163


class FakeTF(MultiTBuffer):
    """In-memory tf with the dispose() hook and call counter the module tests need."""

    def __init__(self) -> None:
        super().__init__()
        self.gets = 0

    def get(
        self,
        parent_frame: str,
        child_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
        *,
        forward_tolerance: float = 0.0,
    ) -> Transform | None:
        self.gets += 1
        return super().get(
            parent_frame,
            child_frame,
            time_point,
            time_tolerance,
            forward_tolerance=forward_tolerance,
        )

    def dispose(self) -> None:
        pass


def _mount() -> Transform:
    return Transform(
        translation=Vector3(0.0, 0.0, MOUNT_Z),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="base_link",
        child_frame_id="mid360_link",
        ts=1.0,
    )


def _odom_edge() -> Transform:
    return Transform(
        translation=Vector3(1.0, 2.0, 3.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="odom",
        child_frame_id="mid360_link",
        ts=1.0,
    )


def test_lookup_pose_steers_from_the_tf_base_pose() -> None:
    tf = FakeTF()
    tf.receive_transform(_mount())
    tf.receive_transform(_odom_edge())
    module = BasicPathFollower()
    module._tf = tf
    try:
        pose = module._lookup_pose()
        assert pose is not None
        assert abs(pose.position.z - (3.0 - MOUNT_Z)) < 1e-9
    finally:
        module.stop()


def test_lookup_pose_is_none_without_the_mount_tf() -> None:
    tf = FakeTF()
    tf.receive_transform(_odom_edge())
    module = BasicPathFollower()
    module._tf = tf
    try:
        assert module._lookup_pose() is None
    finally:
        module.stop()


def test_lookup_retries_are_throttled_during_an_outage() -> None:
    tf = FakeTF()
    module = BasicPathFollower()
    module._tf = tf
    try:
        assert module._lookup_pose() is None
        assert module._lookup_pose() is None
        assert tf.gets == 1
        tf.receive_transform(_mount())
        tf.receive_transform(_odom_edge())
        module._next_lookup = 0.0
        assert module._lookup_pose() is not None
    finally:
        module.stop()


def test_lookahead_floor_at_low_speed() -> None:
    assert lookahead_distance(0.1, 1.5, 0.4, 1.5) == 0.4


def test_lookahead_scales_in_linear_region() -> None:
    assert lookahead_distance(0.5, 1.5, 0.4, 1.5) == 0.75


def test_lookahead_clamped_at_ceiling() -> None:
    assert lookahead_distance(2.0, 1.5, 0.4, 1.5) == 1.5
