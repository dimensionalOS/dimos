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
from dimos.navigation.tf_pose import OdomBasePose, base_height_above_ground
from dimos.protocol.tf.tf import MultiTBuffer

IDENTITY = Quaternion(0.0, 0.0, 0.0, 1.0)
MOUNT_Z = 0.163
LIDAR_HEIGHT = 0.45


class CountingTF(MultiTBuffer):
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


def _mount(z: float = MOUNT_Z, pitch: float = 0.0) -> Transform:
    return Transform(
        translation=Vector3(0.0, 0.0, z),
        rotation=Quaternion.from_euler(Vector3(0.0, pitch, 0.0)),
        frame_id="base_link",
        child_frame_id="mid360_link",
        ts=1.0,
    )


def _odom(orientation: Quaternion = IDENTITY) -> Odometry:
    return Odometry(
        ts=1.0,
        frame_id="odom",
        child_frame_id="mid360_link",
        pose=Pose(Vector3(1.0, 2.0, 3.0), orientation),
    )


def test_translates_to_base_frame() -> None:
    tf = MultiTBuffer()
    tf.receive_transform(_mount())
    pose = OdomBasePose(tf, "base_link").resolve(_odom())
    assert pose is not None
    assert pose.frame_id == "odom"
    assert pose.ts == 1.0
    assert abs(pose.position.x - 1.0) < 1e-9
    assert abs(pose.position.y - 2.0) < 1e-9
    assert abs(pose.position.z - (3.0 - MOUNT_Z)) < 1e-9


def test_composes_out_the_mount_pitch() -> None:
    # A level body reads its own mount tilt as the sensor's world orientation, so
    # composing the mount out returns identity.
    mount = _mount(pitch=0.3)
    tf = MultiTBuffer()
    tf.receive_transform(mount)
    pose = OdomBasePose(tf, "base_link").resolve(_odom(orientation=mount.rotation))
    assert pose is not None
    assert pose.orientation.angle_to(IDENTITY) < 1e-5


def test_preserves_body_yaw_under_mount_tilt() -> None:
    mount = _mount(pitch=0.3)
    body = Quaternion.from_euler(Vector3(0.0, 0.0, 0.7))
    tf = MultiTBuffer()
    tf.receive_transform(mount)
    pose = OdomBasePose(tf, "base_link").resolve(_odom(orientation=body * mount.rotation))
    assert pose is not None
    assert pose.orientation.angle_to(body) < 1e-5


def test_drops_frames_until_the_mount_leg_arrives() -> None:
    tf = MultiTBuffer()
    resolver = OdomBasePose(tf, "base_link")
    assert resolver.resolve(_odom()) is None
    tf.receive_transform(_mount())
    resolver._next_lookup = 0.0
    assert resolver.resolve(_odom()) is not None


def test_missing_leg_lookups_are_throttled() -> None:
    tf = CountingTF()
    resolver = OdomBasePose(tf, "base_link")
    assert resolver.resolve(_odom()) is None
    assert resolver.resolve(_odom()) is None
    assert tf.gets == 1


def test_mount_leg_is_looked_up_once() -> None:
    tf = CountingTF()
    tf.receive_transform(_mount())
    resolver = OdomBasePose(tf, "base_link")
    assert resolver.resolve(_odom()) is not None
    assert resolver.resolve(_odom()) is not None
    assert tf.gets == 1


def test_base_frame_odometry_passes_through() -> None:
    resolver = OdomBasePose(MultiTBuffer(), "base_link")
    msg = Odometry(ts=1.0, frame_id="odom", child_frame_id="base_link")
    pose = resolver.resolve(msg)
    assert pose is not None
    assert pose.frame_id == "odom"


def test_base_height_above_ground() -> None:
    assert abs(base_height_above_ground(LIDAR_HEIGHT, _mount()) - (LIDAR_HEIGHT - MOUNT_Z)) < 1e-9
