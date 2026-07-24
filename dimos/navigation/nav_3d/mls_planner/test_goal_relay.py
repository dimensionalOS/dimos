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
from dimos.navigation.nav_3d.mls_planner.goal_relay import GoalRelay
from dimos.protocol.tf.tf import MultiTBuffer

MOUNT_Z = 0.163


class FakeTF(MultiTBuffer):
    """In-memory tf with the stop() hook and call counter the module tests need."""

    def __init__(self) -> None:
        super().__init__()
        self.gets = 0

    def get(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        self.gets += 1
        return super().get(*args, **kwargs)

    def stop(self) -> None:
        pass


def _mount() -> Transform:
    return Transform(
        translation=Vector3(0.0, 0.0, MOUNT_Z),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="base_link",
        child_frame_id="mid360_link",
        ts=1.0,
    )


def _odom(z: float = 3.0) -> Odometry:
    return Odometry(
        ts=1.0,
        frame_id="odom",
        child_frame_id="mid360_link",
        pose=Pose(Vector3(1.0, 2.0, z), Quaternion(0.0, 0.0, 0.0, 1.0)),
    )


def _relay(tf: FakeTF, **config) -> tuple[GoalRelay, list]:  # type: ignore[no-untyped-def]
    module = GoalRelay(**config)
    module._tf = tf
    captured: list = []
    module.start_pose.subscribe(captured.append)
    return module, captured


def test_start_pose_is_ground_projected():
    tf = FakeTF()
    tf.receive_transform(_mount())
    module, captured = _relay(tf, lidar_height=0.45)
    try:
        module._on_odometry(_odom())
        # Base sits MOUNT_Z below the sensor, then drops by the base's height
        # above ground (0.45 - MOUNT_Z): together exactly the lidar height.
        assert len(captured) == 1
        assert abs(captured[0].position.z - (3.0 - 0.45)) < 1e-9
    finally:
        module.stop()


def test_drops_frames_without_the_mount_tf():
    module, captured = _relay(FakeTF(), lidar_height=0.45)
    try:
        module._on_odometry(_odom())
        assert captured == []
    finally:
        module.stop()


def test_no_lidar_height_skips_the_ground_correction():
    tf = FakeTF()
    tf.receive_transform(_mount())
    module, captured = _relay(tf)
    try:
        module._on_odometry(_odom())
        assert len(captured) == 1
        assert abs(captured[0].position.z - (3.0 - MOUNT_Z)) < 1e-9
    finally:
        module.stop()


def test_mount_is_looked_up_once():
    tf = FakeTF()
    tf.receive_transform(_mount())
    module, captured = _relay(tf, lidar_height=0.45)
    try:
        module._on_odometry(_odom())
        module._on_odometry(_odom(z=4.0))
        assert len(captured) == 2
        assert abs(captured[1].position.z - (4.0 - 0.45)) < 1e-9
        assert tf.gets == 1
    finally:
        module.stop()
