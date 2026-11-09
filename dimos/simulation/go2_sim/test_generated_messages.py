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

"""Generated simulation outputs preserve frames, values and timestamps."""

from types import SimpleNamespace

import numpy as np

from dimos.core.module import Module
from dimos.messages.geometry_msgs.msg import PoseStamped
from dimos.messages.nav_msgs.msg import Odometry
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.simulation.go2_sim.world import ODOM_FRAME_ID, SENSOR_FRAME_ID, SimGo2World


def test_sensor_and_ground_truth_outputs_share_the_exact_tick_stamp(mocker):
    module = SimGo2World()
    odometry = mocker.patch.object(module.odometry, "publish")
    transforms = mocker.patch.object(module.tf, "publish")
    truth = mocker.patch.object(module.ground_truth, "publish")
    sim = SimpleNamespace(
        sensor_pose=lambda: (np.array([1.0, 2.0, 3.0]), np.eye(3)),
        sensor_velocity=lambda: (np.array([0.1, 0.2, 0.3]), np.array([0.4, 0.5, 0.6])),
        base_pose=lambda: (np.array([4.0, 5.0, 6.0]), np.eye(3)),
    )
    try:
        module._publish_poses(sim, 123.25)
        value = odometry.call_args.args[0]
        tf = transforms.call_args.args[0]
        base = truth.call_args.args[0]
        assert type(value) is Odometry
        assert type(tf) is TFMessage
        assert type(base) is PoseStamped
        assert value.header.frame_id == ODOM_FRAME_ID
        assert value.child_frame_id == SENSOR_FRAME_ID
        assert value.header.stamp == tf.transforms[0].header.stamp == base.header.stamp
        assert (value.header.stamp.sec, value.header.stamp.nanosec) == (123, 250000000)
        assert (
            value.pose.pose.position.x,
            value.pose.pose.position.y,
            value.pose.pose.position.z,
        ) == (1, 2, 3)
        assert value.twist.twist.linear.y == 0.2
        assert value.twist.twist.angular.z == 0.6
        assert (base.pose.position.x, base.pose.position.y, base.pose.position.z) == (4, 5, 6)
    finally:
        Module.stop(module)
