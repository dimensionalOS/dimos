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

"""The coordinator's base joints, read back as the wheel odometry AlfredHighLevel publishes."""

from __future__ import annotations

from collections.abc import Iterator
import math

import pytest

from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.diy.alfred.coordinator_base_odometry import CoordinatorBaseOdometry
from dimos.robot.diy.alfred.effector_high_level import AlfredHighLevelConfig

BASE = ["flowbase/vx", "flowbase/vy", "flowbase/wz"]
TICK = 0.01  # the coordinator's 100 Hz


@pytest.fixture
def relay() -> Iterator[tuple[CoordinatorBaseOdometry, list[Odometry]]]:
    module = CoordinatorBaseOdometry()
    published: list[Odometry] = []
    module.wheel_odometry.subscribe(published.append)
    try:
        yield module, published
    finally:
        module.stop()


def _state(ts: float, x: float, y: float, yaw: float) -> JointState:
    """A coordinator tick: an arm joint, the lift and the base, as it publishes them."""
    return JointState(
        ts=ts,
        name=["left_arm/joint1", "pillar/lift", *BASE],
        position=[0.3, -0.05, x, y, yaw],
        velocity=[0.0] * 5,
        effort=[0.0] * 5,
    )


def test_the_base_pose_leaves_as_alfred_high_levels_wheel_odometry(
    relay: tuple[CoordinatorBaseOdometry, list[Odometry]],
) -> None:
    module, published = relay
    module._relay(_state(100.0, 1.0, 2.0, 0.5))
    (odometry,) = published
    assert odometry.ts == 100.0
    defaults = AlfredHighLevelConfig.model_fields
    assert (odometry.frame_id, odometry.child_frame_id) == (
        defaults["wheel_odom_frame_id"].default,
        defaults["base_frame_id"].default,
    )
    pose = odometry.pose.pose
    assert (pose.position.x, pose.position.y, pose.position.z) == (1.0, 2.0, 0.0)
    assert pose.orientation.to_euler().z == pytest.approx(0.5)


def test_it_publishes_at_the_flowbases_rate_with_the_body_twist(
    relay: tuple[CoordinatorBaseOdometry, list[Odometry]],
) -> None:
    """100 Hz of ticks become 50 Hz of odometry; 0.2 m/s along a 0.5 rad heading reads as
    0.2 m/s forward, then 0.1 rad/s on the spot as that yaw rate."""
    module, published = relay
    for tick in range(101):
        t = tick * TICK
        module._relay(_state(t, 0.2 * t * math.cos(0.5), 0.2 * t * math.sin(0.5), 0.5))
    assert len(published) == 51
    forward = published[-1].twist.twist
    assert (forward.linear.x, forward.linear.y, forward.angular.z) == pytest.approx((0.2, 0, 0))

    x, y = 0.2 * math.cos(0.5), 0.2 * math.sin(0.5)
    for tick in range(102, 151):
        t = tick * TICK
        module._relay(_state(t, x, y, 0.5 + 0.1 * (t - 1.0)))
    turning = published[-1].twist.twist
    assert (turning.linear.x, turning.linear.y, turning.angular.z) == pytest.approx((0, 0, 0.1))


def test_a_state_without_the_base_is_ignored(
    relay: tuple[CoordinatorBaseOdometry, list[Odometry]],
) -> None:
    module, published = relay
    module._relay(JointState(ts=1.0, name=["pillar/lift"], position=[-0.05]))
    assert published == []
