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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    Vector3,
)
from dimos_generated.nav_msgs.msg import Path
from dimos_generated.std_msgs.msg import Bool, Header

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
    ) -> TransformStamped | None:
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


def _mount() -> TransformStamped:
    return TransformStamped(
        header=Header(frame_id="base_link", stamp=Time(sec=1)),
        child_frame_id="mid360_link",
        transform=Transform(translation=Vector3(z=MOUNT_Z), rotation=Quaternion(w=1)),
    )


def _odom_edge() -> TransformStamped:
    return TransformStamped(
        header=Header(frame_id="odom", stamp=Time(sec=1)),
        child_frame_id="mid360_link",
        transform=Transform(translation=Vector3(x=1, y=2, z=3), rotation=Quaternion(w=1)),
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
        assert abs(pose.pose.position.z - (3.0 - MOUNT_Z)) < 1e-9
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


def test_generated_path_produces_velocity_and_arrival():
    module = BasicPathFollower()
    commands = []
    arrivals = []
    module.nav_cmd_vel.subscribe(lambda value: commands.append(Twist.decode(value.encode())))
    module.goal_reached.subscribe(lambda value: arrivals.append(Bool.decode(value.encode())))
    try:
        path = Path(
            poses=[
                PoseStamped(pose=Pose(position=Point(x=x), orientation=Quaternion(w=1)))
                for x in [0, 1, 2]
            ]
        )
        module._on_path(Path.decode(path.encode()))
        waypoints = module._waypoints
        assert waypoints is not None
        module._step(path.poses[0], waypoints)
        assert commands[-1].linear.x > 0
        assert not arrivals
        module._step(path.poses[-1], waypoints)
        assert commands[-1] == Twist()
        assert arrivals[-1].data
        module._on_path(Path())
        assert module._waypoints is None
    finally:
        module.stop()
