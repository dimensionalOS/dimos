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

"""Feedback and command contracts at the simulator/coordinator boundary."""

import math

import pytest

from dimos.hardware.spec import JointLimits
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS
from dimos.simulation.behavior.r1pro_bridge import BehaviorR1ProBridge, ordered_state
from dimos.simulation.behavior.r1pro_model import MODEL_JOINTS


def test_feedback_is_reordered_by_name_with_efforts():
    state = JointState(name=["b", "a"], position=[2, 1], velocity=[4, 3], effort=[6, 5])
    actual = ordered_state(state, ("a", "b"))
    assert actual.name == ["r1pro/a", "r1pro/b"]
    assert actual.position == [1, 2]
    assert actual.velocity == [3, 4]
    assert actual.effort == [5, 6]


@pytest.mark.parametrize(
    "state",
    [
        JointState(name=["a"], position=[1]),
        JointState(name=["a", "a"], position=[1, 2], velocity=[0, 0], effort=[0, 0]),
        JointState(name=["a"], position=[math.nan], velocity=[0], effort=[0]),
    ],
)
def test_invalid_feedback_is_rejected(state):
    with pytest.raises(ValueError):
        ordered_state(state, ("a",))


@pytest.fixture
def bridge(mocker):
    n = len(UPPER_BODY_JOINTS)
    mocker.patch(
        "dimos.simulation.behavior.r1pro_bridge.upper_body_limits",
        return_value=JointLimits([-1.0] * n, [1.0] * n, [1.0] * n),
    )
    module = BehaviorR1ProBridge()
    try:
        yield module
    finally:
        module.stop()


def test_coordinator_targets_reach_simulator_in_native_joint_names(bridge, mocker):
    publish = mocker.patch.object(bridge.joint_command, "publish")
    bridge._on_command(MotorCommandArray(q=[0.2] * len(UPPER_BODY_JOINTS)))
    publish.assert_called_once()
    message = publish.call_args.args[0]
    assert message.name == list(UPPER_BODY_JOINTS)
    assert message.position == [0.2] * len(UPPER_BODY_JOINTS)


@pytest.mark.parametrize("positions", [[0.2], [math.nan] * 18, [2.0] * 18])
def test_invalid_coordinator_targets_never_reach_simulator(bridge, mocker, positions):
    publish = mocker.patch.object(bridge.joint_command, "publish")
    bridge._on_command(MotorCommandArray(q=positions))
    publish.assert_not_called()


def test_planner_tracks_base_and_grippers_without_exposing_them_to_coordinator(bridge, mocker):
    motor = mocker.patch.object(bridge.motor_states, "publish")
    planner = mocker.patch.object(bridge.planning_joint_state, "publish")
    bridge._on_odom(PoseStamped(ts=10.0, position=[2.0, 3.0, 0.0], frame_id="world"))
    names = list(reversed(MODEL_JOINTS))
    bridge._on_state(
        JointState(
            ts=10.1,
            name=names,
            position=[0.2] * len(names),
            velocity=[0.0] * len(names),
            effort=[0.0] * len(names),
        )
    )
    assert motor.call_args.args[0].name == [f"r1pro/{name}" for name in UPPER_BODY_JOINTS]
    state = planner.call_args.args[0]
    assert state.name[:3] == ["r1pro/base_x", "r1pro/base_y", "r1pro/base_yaw"]
    assert state.position[:3] == [2.0, 3.0, 0.0]
    assert state.name[3:] == [f"r1pro/{name}" for name in MODEL_JOINTS]


def test_stale_base_pose_cannot_update_planning_model(bridge, mocker):
    mocker.patch.object(bridge.motor_states, "publish")
    planner = mocker.patch.object(bridge.planning_joint_state, "publish")
    bridge._on_odom(PoseStamped(ts=1.0, frame_id="world"))
    n = len(MODEL_JOINTS)
    bridge._on_state(
        JointState(
            ts=2.0,
            name=list(MODEL_JOINTS),
            position=[0.0] * n,
            velocity=[0.0] * n,
            effort=[0.0] * n,
        )
    )
    planner.assert_not_called()
