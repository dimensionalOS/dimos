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

import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.tray_delivery import _arm_motion, _check_cargo, _execute


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


@pytest.mark.parametrize(
    ("field", "message"),
    [("inside_bin", "Cargo left"), ("upright", "bottle tipped")],
)
def test_delivery_stops_when_any_cargo_is_outside_or_tipped(field, message):
    state = {
        "inside_bin": True,
        "upright": True,
        "tray": {"tilt_radians": 0.0, "bimanual_grasp": True},
    }
    state[field] = False
    with pytest.raises(RuntimeError, match=message):
        _check_cargo(state, grasp=True)


@pytest.mark.parametrize("contact", ["mattress_b", "chair"])
def test_pickup_accepts_other_meshes_of_its_source_support_but_rejects_other_objects(
    mocker, contact
):
    control, sim = mocker.Mock(), mocker.Mock()
    clock = [0.0]
    mocker.patch(
        "dimos.robot.galaxea.r1pro.tray_delivery.time.monotonic", side_effect=lambda: clock[0]
    )
    control.execute_trajectory.return_value = mocker.Mock(status=TrajectoryExecutionStatus.ACCEPTED)
    control.get_joint_positions.return_value = {"joint": 0.0}
    sim.task_state.side_effect = lambda: {
        "inside_bin": True,
        "upright": True,
        "tray": {
            "tilt_radians": 0.0,
            "bimanual_grasp": True,
            "velocity_norm": 0.0,
            "support_geoms": [contact] if clock[0] < 0.2 else [],
        },
    }
    report = {
        "stages": [],
        "initial": {"tray": {"support_geoms": ["mattress_a"]}},
        "pickup_support_geoms": ["mattress_a", "mattress_b"],
    }
    points = [
        TrajectoryPoint(positions=[0.0], velocities=[0.0], time_from_start=t) for t in [0.0, 0.2]
    ]

    def run():
        return _execute(
            control,
            sim,
            ["joint"],
            points,
            "tray_manipulation",
            "lift_tray",
            report,
            grasp=True,
            pause=lambda seconds: clock.__setitem__(0, clock[0] + seconds),
        )

    if contact == "chair":
        with pytest.raises(RuntimeError, match="unexpected object"):
            run()
    else:
        assert run()["tray"]["support_geoms"] == []
    control.cancel_trajectory.assert_called_once_with("tray_manipulation")
