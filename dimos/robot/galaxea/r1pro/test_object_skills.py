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

"""Object commands must preserve target and hand intent and report actual outcomes."""

import json
from pathlib import Path
import threading

import numpy as np
import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.object_skills import R1ProObjectSkills, resolve_object


@pytest.fixture
def rows():
    return [
        dict(
            id="object_1",
            object="task_object_1",
            index=0,
            shape="box",
            left_m=-0.2,
            distance_m=0.6,
            inside=False,
            upright=True,
            released=True,
        ),
        dict(
            id="object_2",
            object="task_object_2",
            index=1,
            shape="box",
            left_m=-0.1,
            distance_m=0.3,
            inside=False,
            upright=True,
            released=True,
        ),
        dict(
            id="object_3",
            object="task_object_3",
            index=2,
            shape="bottle",
            left_m=-0.5,
            distance_m=0.7,
            inside=False,
            upright=True,
            released=True,
        ),
        dict(
            id="object_4",
            object="task_object_4",
            index=3,
            shape="cylinder",
            left_m=-0.7,
            distance_m=0.8,
            inside=True,
            upright=True,
            released=True,
        ),
    ]


@pytest.mark.parametrize(
    "selector,index",
    [
        ("rightmost", 2),
        ("leftmost", 1),
        ("nearest", 1),
        ("furthest", 2),
        ("bottle", 2),
        ("object_1", 0),
    ],
)
def test_selection_uses_source_geometry_and_stable_ids(rows, selector, index):
    assert resolve_object(rows, selector) == index


@pytest.mark.parametrize("selector", ["box", "object_4", "object_5", "middle"])
def test_ambiguous_missing_or_packed_objects_are_never_substituted(rows, selector):
    with pytest.raises(ValueError):
        resolve_object(rows, selector)


@pytest.fixture
def skills(mocker, tmp_path, rows):
    module = R1ProObjectSkills()
    module._sim = mocker.Mock()
    module._control = mocker.Mock()
    module._policy = mocker.Mock()
    module._sim.object_state.return_value = dict(objects=rows, at_home=True, sim_time=1.0)
    module._sim.prepare_object_session.return_value = dict(output=str(tmp_path))
    module._sim.is_simulation_running.return_value = True
    module._control.list_tasks.return_value = ["policy_rollout"]
    module._control.cancel_trajectory.return_value.safe = True
    module._policy.stop_rollout.return_value = dict(active=False)
    yield module
    module.stop()


def test_left_hand_request_never_runs_the_right_policy(skills, mocker):
    run = mocker.patch("dimos.robot.galaxea.r1pro.object_skills.run_object_pick")
    result = json.loads(skills.pick_object("nearest", "left"))
    assert result["accepted"] is False
    assert result["reason"] == "unsupported_arm"
    run.assert_not_called()
    skills._sim.object_state.assert_not_called()
    assert json.loads(skills.wait_for_action(0))["state"] == "idle"


def test_failed_pick_can_recover_but_is_never_reported_as_success(skills, mocker):
    def fail(policy, sim, index, report, **kwargs):
        report.update(history=[{}], success=False, reason="pick_timeout")

    mocker.patch("dimos.robot.galaxea.r1pro.object_skills.run_object_pick", side_effect=fail)
    recovered = mocker.patch.object(skills, "_recover")
    assert json.loads(skills.pick_object("rightmost"))["accepted"]
    result = json.loads(skills.wait_for_action(5))
    assert result["state"] == "failed"
    assert result["success"] is False
    assert result["recovery_required"] is False
    recovered.assert_called_once()
    assert "rightmost" == json.loads(Path(result["evidence"]).read_text())["requested_object"]


def test_second_command_is_rejected_while_action_runs_and_stop_does_not_reset(skills, mocker):
    entered = threading.Event()

    def running(policy, sim, index, report, **kwargs):
        report["history"] = [{}]
        entered.set()
        kwargs["pause"](5)

    run = mocker.patch(
        "dimos.robot.galaxea.r1pro.object_skills.run_object_pick", side_effect=running
    )
    assert json.loads(skills.pick_object("object_1"))["accepted"]
    assert entered.wait(2)
    assert json.loads(skills.pick_object("object_2"))["accepted"] is False
    skills.stop_action()
    result = json.loads(skills.wait_for_action(5))
    assert result["state"] == "cancelled"
    assert result["recovery_required"] is True
    skills._sim.reset.assert_not_called()
    assert run.call_count == 1


def test_scene_reports_the_current_measured_inventory(skills, rows):
    result = json.loads(skills.get_scene())
    assert result["objects"] == rows
    assert result["action"]["state"] == "idle"


def test_recovery_tolerates_numerical_feedback_noise_at_joint_stop(skills, mocker):
    q = [0.0] * 18 + [0.05, 0.05000002]
    limits = [[-3, 3]] * 18 + [[0, 0.05]] * 2
    skills._sim.prepare_object_session.return_value = {"limits": limits}
    skills._sim.plan_object_recovery.return_value = [
        {"phase": "recovery_release", "positions": q, "seconds": 0.8}
    ]
    skills._sim.object_state.return_value = {"objects": [], "robot_obstacles": []}
    skills._control.get_joint_positions.return_value = dict(
        zip(R1PRO_PICK_PLACE_JOINTS, q, strict=True)
    )
    skills._control.execute_trajectory.return_value.status = TrajectoryExecutionStatus.ACCEPTED
    clock = [0.0]
    mocker.patch(
        "dimos.robot.galaxea.r1pro.object_skills.time.monotonic", side_effect=lambda: clock[0]
    )
    mocker.patch.object(
        skills, "_pause", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)
    )
    report = {}
    skills._recover(report)
    trajectory = skills._control.execute_trajectory.call_args.args[0]
    values = np.array([p.positions for p in trajectory.points])
    assert np.all(values[:, -1] <= 0.05)
    assert values[0, -1] == 0.05
    skills._sim.finish_object_recovery.assert_called_once()


def test_pick_stops_with_held_object_and_never_calls_place(skills, mocker):
    def hold(policy, sim, index, report, **kwargs):
        assert kwargs["grasp_only"] is True
        report.update(success=True, history=[{"holding": True}])

    mocker.patch("dimos.robot.galaxea.r1pro.object_skills.run_object_pick", side_effect=hold)
    place = mocker.patch("dimos.robot.galaxea.r1pro.object_skills.run_object_place")
    assert json.loads(skills.pick_object("object_1"))["accepted"]
    outcome = json.loads(skills.wait_for_action(5))
    assert outcome["state"] == "completed"
    place.assert_not_called()
    skills._sim.reset.assert_not_called()


def test_place_requires_a_held_object_and_preserves_destination_and_hand(skills, mocker):
    place = mocker.patch("dimos.robot.galaxea.r1pro.object_skills.run_object_place")
    assert json.loads(skills.place_object())["accepted"] is False
    skills._sim.object_state.return_value.update(holding=True, held_object="object_1")
    assert json.loads(skills.place_object("kitchen"))["reason"] == "unsupported_destination"
    assert json.loads(skills.place_object("tray", "left"))["reason"] == "unsupported_arm"
    place.assert_not_called()


def test_held_object_blocks_another_pick_but_can_be_explicitly_placed(skills, mocker):
    skills._sim.object_state.return_value.update(holding=True, held_object="object_1")
    pick = mocker.patch("dimos.robot.galaxea.r1pro.object_skills.run_object_pick")
    assert json.loads(skills.pick_object("object_2"))["accepted"] is False
    pick.assert_not_called()

    def placed(policy, sim, report, **kwargs):
        report.update(success=True, history=[{"pick_complete": True}])

    place = mocker.patch(
        "dimos.robot.galaxea.r1pro.object_skills.run_object_place", side_effect=placed
    )
    assert json.loads(skills.place_object())["accepted"]
    outcome = json.loads(skills.wait_for_action(5))
    assert outcome["success"] is True
    assert json.loads(Path(outcome["evidence"]).read_text())["held_object"] == "object_1"
    place.assert_called_once()
