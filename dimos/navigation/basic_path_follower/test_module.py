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

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.basic_path_follower.module import BasicPathFollower, lookahead_distance
from dimos.protocol.tf.tf import MultiTBuffer


class FakeTF(MultiTBuffer):
    def stop(self) -> None:
        pass


def _odom() -> Odometry:
    return Odometry(
        ts=1.0,
        frame_id="odom",
        child_frame_id="mid360_link",
        pose=Pose(Vector3(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
    )


def test_on_odometry_steers_from_the_base_pose():
    tf = FakeTF()
    tf.receive_transform(
        Transform(
            translation=Vector3(0.0, 0.0, 0.163),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="mid360_link",
            ts=1.0,
        )
    )
    module = BasicPathFollower()
    module._tf = tf
    try:
        module._on_odometry(_odom())
        assert module._current_pose is not None
        assert abs(module._current_pose.position.z - (3.0 - 0.163)) < 1e-9
    finally:
        module.stop()


def test_on_odometry_drops_frames_without_the_mount_tf():
    module = BasicPathFollower()
    module._tf = FakeTF()
    try:
        module._on_odometry(_odom())
        assert module._current_pose is None
    finally:
        module.stop()


def test_lookahead_floor_at_low_speed():
    assert lookahead_distance(0.1, 1.5, 0.4, 1.5) == 0.4


def test_lookahead_scales_in_linear_region():
    assert lookahead_distance(0.5, 1.5, 0.4, 1.5) == 0.75


def test_lookahead_clamped_at_ceiling():
    assert lookahead_distance(2.0, 1.5, 0.4, 1.5) == 1.5
