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

"""The Spot SDK boundary publishes generated odometry and its matching TF edge."""

from types import SimpleNamespace

from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.tf2_msgs.msg import TFMessage
import pytest

from dimos.experimental.robot.bosdyn.spot.effectors.high_level import SpotHighLevel


@pytest.fixture
def module():
    instance = SpotHighLevel(odom_frame_id="vision", base_frame_id="body")
    try:
        yield instance
    finally:
        instance.stop()


def test_odometry_and_tf_preserve_sdk_pose_velocity_and_source_time(module, mocker):
    odometry_publish = mocker.patch.object(module.odometry, "publish")
    tf_publish = mocker.patch.object(module.tf, "publish")
    pose = SimpleNamespace(x=1.0, y=2.0, z=3.0, rot=SimpleNamespace(x=0, y=0, z=0.6, w=0.8))
    velocity = SimpleNamespace(
        linear=SimpleNamespace(x=4, y=5, z=6), angular=SimpleNamespace(x=7, y=8, z=9)
    )

    module._publish_odom(pose, velocity, -0.5)

    odometry_publish.assert_called_once()
    tf_publish.assert_called_once()
    odometry = Odometry.decode(odometry_publish.call_args.args[0].encode())
    tf = TFMessage.decode(tf_publish.call_args.args[0].encode())
    assert odometry.header.frame_id == "vision"
    assert odometry.child_frame_id == "body"
    assert (odometry.header.stamp.sec, odometry.header.stamp.nanosec) == (-1, 500000000)
    assert (
        odometry.pose.pose.position.x,
        odometry.pose.pose.position.y,
        odometry.pose.pose.position.z,
    ) == (1, 2, 3)
    assert odometry.pose.pose.orientation.z == 0.6
    assert odometry.twist.twist.linear.y == 5
    assert odometry.twist.twist.angular.z == 9
    assert len(tf.transforms) == 1
    edge = tf.transforms[0]
    assert edge.header == odometry.header
    assert edge.child_frame_id == odometry.child_frame_id
    assert edge.transform.translation.x == odometry.pose.pose.position.x
    assert edge.transform.rotation == odometry.pose.pose.orientation
