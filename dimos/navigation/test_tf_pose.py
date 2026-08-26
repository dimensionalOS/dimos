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
from dimos.navigation.tf_pose import TfPose
from dimos.protocol.tf.tf import MultiTBuffer

IDENTITY = Quaternion(0.0, 0.0, 0.0, 1.0)


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


class _Clock:
    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


def _body_at(x: float, ts: float) -> Transform:
    return Transform(
        translation=Vector3(x, 0.0, 0.3),
        rotation=IDENTITY,
        frame_id="odom",
        child_frame_id="base_link",
        ts=ts,
    )


def test_tf_pose_is_the_world_to_base_edge():
    tf, clock = MultiTBuffer(), _Clock()
    tf.receive_transform(_body_at(1.0, ts=5.0))
    pose = TfPose(tf, "base_link", max_age_s=2.5, clock=clock).get("odom")
    assert pose is not None
    assert (pose.position.x, pose.position.z, pose.ts, pose.frame_id) == (1.0, 0.3, 5.0, "odom")


def test_tf_pose_goes_stale_when_the_stamp_stops_advancing():
    tf, clock = MultiTBuffer(), _Clock()
    src = TfPose(tf, "base_link", max_age_s=2.5, clock=clock)
    tf.receive_transform(_body_at(1.0, ts=5.0))
    assert src.get("odom") is not None
    clock.t += 2.5
    assert src.get("odom") is not None, "the boundary is exclusive"
    clock.t += 0.1
    assert src.get("odom") is None
    # a fresh stamp is a live edge again, whatever the robot's clock says
    tf.receive_transform(_body_at(2.0, ts=5.1))
    assert src.get("odom") is not None


def test_tf_pose_measures_age_on_its_own_clock_not_the_stamp():
    tf, clock = MultiTBuffer(), _Clock()
    src = TfPose(tf, "base_link", max_age_s=2.5, clock=clock)
    tf.receive_transform(_body_at(1.0, ts=0.0))  # a robot clock nowhere near ours
    assert src.get("odom") is not None


def test_tf_pose_retries_a_missing_edge_once_per_period():
    tf, clock = CountingTF(), _Clock()
    src = TfPose(tf, "base_link", max_age_s=2.5, clock=clock)
    for _ in range(5):
        assert src.get("odom") is None
    assert tf.gets == 1
    clock.t += TfPose.RETRY_PERIOD_S
    assert src.get("odom") is None
    assert tf.gets == 2
