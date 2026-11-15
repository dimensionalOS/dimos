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

import threading

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.galaxea.r1pro.connection import R1ProConnection


@pytest.fixture
def connection(mocker):
    stopped = threading.Event()
    commands = []

    def publish(message):
        commands.append(message)
        if message.linear.x == 0:
            stopped.set()

    connection = R1ProConnection(cmd_vel_timeout_s=0.1, boot_vendor_stack=False)
    mocker.patch.object(connection, "_publish_cmd_vel", side_effect=publish)
    connection._start_cmd_vel_watchdog()
    try:
        yield connection, commands, stopped
    finally:
        connection.stop()


def test_robot_side_watchdog_stops_after_command_source_disappears(connection):
    robot, commands, stopped = connection
    robot._on_cmd_vel(Twist(linear=Vector3(0.4, 0, 0)))

    assert stopped.wait(timeout=1)
    assert [command.linear.x for command in commands] == [0.4, 0.0]
    assert commands[-1].angular.z == 0


def test_shutdown_stops_once_and_late_callbacks_cannot_restart_motion(connection):
    robot, commands, stopped = connection
    robot._on_cmd_vel(Twist(linear=Vector3(0.4, 0, 0)))
    robot._stop_cmd_vel_watchdog()
    robot._stop_cmd_vel_watchdog()
    robot._on_cmd_vel(Twist(linear=Vector3(0.9, 0, 0)))

    assert [command.linear.x for command in commands] == [0.4, 0.0]
    assert stopped.is_set()


def test_a_new_command_after_expiry_gets_its_own_watchdog(connection):
    robot, commands, stopped = connection
    robot._on_cmd_vel(Twist(linear=Vector3(0.2, 0, 0)))
    assert stopped.wait(timeout=1)
    stopped.clear()
    robot._on_cmd_vel(Twist(linear=Vector3(0.4, 0, 0)))

    assert stopped.wait(timeout=1)
    assert [command.linear.x for command in commands] == [0.2, 0.0, 0.4, 0.0]
