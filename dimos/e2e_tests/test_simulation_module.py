# Copyright 2025-2026 Dimensional Inc.
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

"""End-to-end tests for the simulation module."""

import os

import pytest

from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState


def _positions_within_tolerance(
    positions: list[float],
    target: list[float],
    tolerance: float,
) -> bool:
    if len(positions) < len(target):
        return False
    return all(abs(positions[i] - target[i]) <= tolerance for i in range(len(target)))


@pytest.mark.skipif(bool(os.getenv("CI")), reason="LCM doesn't work in CI.")
@pytest.mark.e2e
class TestSimulationModuleE2E:
    def test_xarm7_joint_state_published(self, lcm_spy, start_blueprint) -> None:
        joint_state_topic = "/xarm/joint_states#sensor_msgs.JointState"
        lcm_spy.save_topic(joint_state_topic)

        start_blueprint("simulation-xarm7")
        lcm_spy.wait_for_saved_topic(joint_state_topic, timeout=15.0)

        with lcm_spy._messages_lock:
            raw_joint_state = lcm_spy.messages[joint_state_topic][0]

        joint_state = JointState.lcm_decode(raw_joint_state)
        assert len(joint_state.name) == 8
        assert len(joint_state.position) == 8

    def test_xarm7_robot_state_published(self, lcm_spy, start_blueprint) -> None:
        robot_state_topic = "/xarm/robot_state#sensor_msgs.RobotState"
        lcm_spy.save_topic(robot_state_topic)

        start_blueprint("simulation-xarm7")
        lcm_spy.wait_for_saved_topic(robot_state_topic, timeout=15.0)

        with lcm_spy._messages_lock:
            raw_robot_state = lcm_spy.messages[robot_state_topic][0]

        robot_state = RobotState.lcm_decode(raw_robot_state)
        assert robot_state.mt_able in (0, 1)

    def test_xarm7_joint_command_updates_joint_state(self, lcm_spy, start_blueprint) -> None:
        joint_state_topic = "/xarm/joint_states#sensor_msgs.JointState"
        joint_command_topic = "/xarm/joint_position_command#sensor_msgs.JointCommand"
        lcm_spy.save_topic(joint_state_topic)

        start_blueprint("simulation-xarm7")
        lcm_spy.wait_for_saved_topic(joint_state_topic, timeout=15.0)

        target_positions = [0.2, -0.2, 0.1, -0.1, 0.15, -0.15, 0.05]
        lcm_spy.publish(joint_command_topic, JointCommand(positions=target_positions))

        tolerance = 0.03
        lcm_spy.wait_for_message_result(
            joint_state_topic,
            JointState,
            predicate=lambda msg: _positions_within_tolerance(
                list(msg.position),
                target_positions,
                tolerance,
            ),
            fail_message=("joint_state did not reach commanded positions within tolerance"),
            timeout=10.0,
        )
