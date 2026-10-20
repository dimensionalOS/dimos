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

from dimos.messages.geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.time import time_from_nanoseconds, time_from_seconds, to_seconds
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
    ) -> TransformStamped | None:
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


def _body_at(x: float, ts: float) -> TransformStamped:
    return TransformStamped(
        header=Header(frame_id="odom", stamp=time_from_seconds(ts)),
        child_frame_id="base_link",
        transform=Transform(translation=Vector3(x=x, y=0.0, z=0.3), rotation=IDENTITY),
    )


def test_tf_pose_is_the_world_to_base_edge():
    tf, clock = MultiTBuffer(), _Clock()
    tf.receive_transform(_body_at(1.0, ts=5.0))
    pose = TfPose(tf, "base_link", max_age_s=2.5, clock=clock).get("odom")
    assert pose is not None
    assert (
        pose.pose.position.x,
        pose.pose.position.z,
        to_seconds(pose.header.stamp),
        pose.header.frame_id,
    ) == (1.0, 0.3, 5.0, "odom")


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


@pytest.mark.parametrize("stamp", [0, -1, 1700000000123456789])
def test_tf_pose_observes_single_nanosecond_progress_without_float_conversion(stamp):
    tf, clock = MultiTBuffer(), _Clock()
    source = TfPose(tf, "base_link", max_age_s=2.5, clock=clock)
    transform = _body_at(1.0, 0.0)
    transform.header.stamp = time_from_nanoseconds(stamp)
    tf.receive_transform(transform)
    first = source.get("odom")
    assert first.header == transform.header
    first.pose.position.x = 100
    assert source.get("odom").pose.position.x == 1
    clock.t += 2.6
    assert source.get("odom") is None
    transform.header.stamp = time_from_nanoseconds(stamp + 1)
    tf.receive_transform(transform)
    fresh = source.get("odom")
    assert fresh.header == transform.header and fresh.pose.position.x == 1
