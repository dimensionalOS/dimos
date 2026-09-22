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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.sensor_msgs.msg import JointState
from dimos_generated.std_msgs.msg import Header
import pytest

from dimos.manipulation.planning.monitor.robot_state_monitor import RobotStateMonitor
from dimos.manipulation.planning.spec.protocols import WorldSpec


@pytest.fixture()
def monitor(mocker):
    world = mocker.Mock(spec=WorldSpec)
    monitor = RobotStateMonitor(world, threading.RLock(), ["arm/a", "arm/b"])
    monitor.start()
    try:
        yield monitor, world
    finally:
        monitor.stop()


def test_generated_feedback_reorders_joints_and_preserves_source_header(monitor):
    monitor, world = monitor
    received = []
    monitor.add_state_callback(received.append)
    source = JointState(
        header=Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="robot"),
        name=["arm/b", "other", "arm/a"],
        position=[2.0, 9.0, 1.0],
        velocity=[0.2, 0.9, 0.1],
    )
    monitor.on_joint_state(JointState.decode(source.encode()))
    assert monitor.get_current_positions().tolist() == [1.0, 2.0]
    assert monitor.get_current_velocities().tolist() == [0.1, 0.2]
    expected = JointState(header=source.header, name=["arm/a", "arm/b"], position=[1.0, 2.0])
    world.sync_from_joint_state.assert_called_once_with(expected)
    assert received == [expected]
    source.header.stamp.nanosec = 0
    assert received[0].header.stamp.nanosec == 123456789


def test_missing_joint_does_not_replace_valid_state(monitor):
    monitor, world = monitor
    monitor.on_joint_state(JointState(name=["arm/a", "arm/b"], position=[1.0, 2.0]))
    monitor.on_joint_state(JointState(name=["arm/a"], position=[9.0]))
    assert monitor.get_current_positions().tolist() == [1.0, 2.0]
    assert world.sync_from_joint_state.call_count == 1


def test_stopped_monitor_does_not_sync(monitor):
    monitor, world = monitor
    monitor.stop()
    monitor.on_joint_state(JointState(name=["arm/a", "arm/b"], position=[1.0, 2.0]))
    assert monitor.get_current_positions() is None
    world.sync_from_joint_state.assert_not_called()
