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

"""Loaded grippers must retain their commanded preload between trajectories."""

from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.tray_delivery import _arm_motion


def test_phase_handoffs_keep_grip_command_instead_of_relaxing_to_measured_opening(mocker):
    control, sim = mocker.Mock(), mocker.Mock()
    measured = dict.fromkeys(R1PRO_PICK_PLACE_JOINTS, 0.0)
    measured.update({name: 0.012 for name in R1PRO_PICK_PLACE_JOINTS[-2:]})
    control.get_joint_positions.return_value = measured
    command = [0.0] * 18 + [0.009, 0.009]
    state = {"inside_bin": True, "tray": {"bimanual_grasp": True, "tilt_radians": 0.0}}
    sim.task_state.return_value = state
    execute = mocker.patch("dimos.robot.galaxea.r1pro.tray_delivery._execute", return_value=state)
    report = {"last_arm_command": command}
    points = [
        {"phase": phase, "positions": command, "seconds": 1.0}
        for phase in ("extend_over_table", "lower_onto_table")
    ]

    _arm_motion(control, sim, points, report)

    assert execute.call_count == 2
    for call in execute.call_args_list:
        assert call.args[3][0].positions[-2:] == [0.009, 0.009]
