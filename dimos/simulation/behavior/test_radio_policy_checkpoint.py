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

"""Policy intents preserve targets and use the tested checkpoint boundary."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.manipulation.manipulation_spec import ExecutionResult, ExecutionStatus
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.simulation.behavior.radio_checkpoint import (
    RadioEpisodeTerminalError,
    RadioGraspCheckpoint,
)
from dimos.simulation.behavior.radio_policy import RadioPolicySupervisor
from dimos.simulation.behavior.radio_policy_checkpoint import RadioCheckpointPolicyMotion


def test_bridge_keeps_pose_and_passes_exact_dispatch_cancel_and_auxiliary_contract(mocker):
    checkpoint = mocker.Mock(spec=RadioGraspCheckpoint)
    bridge = RadioCheckpointPolicyMotion(checkpoint, ("torso",))
    dispatch, cancelled = mocker.Mock(), mocker.Mock(return_value=False)
    bridge.move_intent(
        [1, 2, 3], [0, 0, 0, 1], 12, {"phase": "reorient"}, dispatch=dispatch, cancelled=cancelled
    )
    args, kwargs = checkpoint.move.call_args
    assert args[0].position.to_tuple() == (1, 2, 3)
    assert args[0].orientation.to_tuple() == (0, 0, 0, 1)
    assert args[1] == "reorient"
    assert kwargs == {
        "timeout": 12,
        "auxiliary_torso": True,
        "contact": None,
        "dispatch": dispatch,
        "cancelled": cancelled,
    }


def test_press_uses_caller_surface_and_normal_without_task_target_substitution(mocker):
    checkpoint = mocker.Mock(spec=RadioGraspCheckpoint)
    radio = np.eye(4)
    radio[:3, :3] = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    radio[:3, 3] = [1, 2, 3]
    checkpoint._observation.return_value = {"measured_gripper": 0.08, "radio_pose": radio.tolist()}
    bridge = RadioCheckpointPolicyMotion(checkpoint, ())
    bridge.move_intent(
        [4, 5, 6],
        [0, 0, 0, 1],
        10,
        {"phase": "table_press", "surface_world": [1, 2.01, 3], "normal_world": [0, 1, 0]},
        dispatch=mocker.Mock(),
        cancelled=lambda: False,
    )
    args, kwargs = checkpoint.move.call_args
    assert args[0].position.to_tuple() == (4, 5, 6)
    assert kwargs["contact"]["surface_in_radio"] == pytest.approx([0.01, 0, 0])
    assert kwargs["contact"]["outward_normal_in_radio"] == pytest.approx([1, 0, 0])
    assert kwargs["contact"]["source"] == "caller_sensor_intent"


def test_checkpoint_policy_converts_base_intent_and_exposes_no_terminal_goal_truth(mocker):
    motion = mocker.Mock(spec=RadioCheckpointPolicyMotion)
    motion.move_intent.side_effect = RadioEpisodeTerminalError(
        {
            "kind": "TASK_GOAL_MET",
            "stop_confirmed": True,
            "goal_status": {"satisfied": [0], "unsatisfied": []},
        }
    )
    arm = mocker.Mock()
    arm.rpc.cancel.return_value = ExecutionResult(ExecutionStatus.NO_EXECUTION)
    service = RadioPolicySupervisor(
        arm, motion, lambda: {}, lambda: Transform(translation=Vector3(1, 2, 3))
    )
    try:
        action = service.move_checkpoint_pose(
            [0.1, 0.2, 0.3],
            [0, 0, 0, 1],
            "table_press",
            contact_position=[0.2, 0.3, 0.4],
            contact_normal=[1, 0, 0],
        )
        service._worker.join(timeout=1)
        assert not service._worker.is_alive()
        assert motion.move_intent.call_args.args[0] == pytest.approx([1.1, 2.2, 3.3])
        assert motion.move_intent.call_args.args[3]["surface_world"] == pytest.approx(
            [1.2, 2.3, 3.4]
        )
        result = service.status(action.id)
        assert result.state == "cancelled" and result.stop_confirmed
        assert result.error == "episode_ended"
        assert "TASK_GOAL_MET" not in repr(result) and "satisfied" not in repr(result)
    finally:
        service.close()


def test_checkpoint_press_without_sensor_intent_is_rejected_before_dispatch(mocker):
    motion = mocker.Mock(spec=RadioCheckpointPolicyMotion)
    service = RadioPolicySupervisor(mocker.Mock(), motion, lambda: {}, Transform.identity)
    try:
        with pytest.raises(ValueError, match="Declare contact"):
            service.move_checkpoint_pose([1, 2, 3], [0, 0, 0, 1], "table_press")
        motion.move_intent.assert_not_called()
    finally:
        service.close()
