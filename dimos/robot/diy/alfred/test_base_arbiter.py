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

"""The teleop/navigation mux Alfred used to get from MovementManager."""

from __future__ import annotations

import pytest

from dimos.control.task import ControlMode
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.diy.alfred.base_arbiter import (
    BASE_HARDWARE_ID,
    BASE_JOINTS,
    NAV_PRIORITY,
    TELE_PRIORITY,
    AlfredBaseArbiter,
    PublishingTwistBase,
    arbiter_tasks,
    base_hardware,
)


@pytest.fixture
def arbiter(mocker):
    mocker.patch("dimos.control.coordinator.TickLoop")
    coordinator = AlfredBaseArbiter(
        publish_joint_state=False,
        hardware=[base_hardware()],
        tasks=arbiter_tasks(),
    )
    published: list[Twist] = []
    mocker.patch.object(coordinator.cmd_vel, "publish", published.append)
    coordinator.start()
    try:
        yield coordinator, published
    finally:
        coordinator.stop()


class TestPublishingTwistBase:
    def test_velocities_become_a_twist_on_the_stream(self):
        published: list[Twist] = []
        base = PublishingTwistBase(published.append)

        assert base.write_velocities([0.5, -0.25, 1.5])

        assert (published[-1].linear.x, published[-1].linear.y) == (0.5, -0.25)
        assert published[-1].angular.z == 1.5

    def test_odometry_is_left_to_the_driving_module(self):
        assert PublishingTwistBase(lambda _: None).read_odometry() is None

    def test_a_wrong_width_command_publishes_nothing(self):
        published: list[Twist] = []
        base = PublishingTwistBase(published.append)

        assert not base.write_velocities([0.5, -0.25])
        assert published == []


class TestArbitration:
    def test_teleop_outranks_navigation(self):
        assert TELE_PRIORITY > NAV_PRIORITY

    def test_each_source_reaches_only_its_own_task(self, arbiter):
        coordinator, _ = arbiter

        coordinator._map_twist_to_base_joints(
            "nav_joint_command",
            Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)),
        )
        coordinator._map_twist_to_base_joints(
            "tele_joint_command",
            Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 2.0)),
        )

        assert coordinator.get_task("nav")._velocities == [1.0, 0.0, 0.0]
        assert coordinator.get_task("tele")._velocities == [0.0, 0.0, 2.0]

    def test_teleop_is_idle_until_it_is_driven(self, arbiter):
        coordinator, _ = arbiter

        coordinator._map_twist_to_base_joints(
            "nav_joint_command",
            Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0)),
        )

        assert coordinator.get_task("nav").is_active()
        assert not coordinator.get_task("tele").is_active()

    def test_the_arbitrated_command_leaves_on_cmd_vel(self, arbiter):
        coordinator, published = arbiter

        coordinator._hardware[BASE_HARDWARE_ID].write_command(
            dict(zip(BASE_JOINTS, [0.5, 0.0, 1.5], strict=True)), ControlMode.VELOCITY
        )

        assert (published[-1].linear.x, published[-1].angular.z) == (0.5, 1.5)

    def test_neither_task_holds_zeros_when_idle(self, arbiter):
        """Holding zeros would keep the task active and make every tick a Portal RPC."""
        coordinator, _ = arbiter

        for name in ("nav", "tele"):
            assert not coordinator.get_task(name)._config.zero_on_timeout
