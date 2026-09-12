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

"""Interactive object selection is measured, explicit and deterministic."""

import json

import pytest

from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.robot.galaxea.r1pro.home_skills import R1ProHomeSkills, select_object
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS


@pytest.fixture
def bottles():
    return [
        {"id": "bottle_1", "bottle": 1, "left_m": -0.2, "distance_m": 0.6},
        {"id": "bottle_4", "bottle": 4, "left_m": -0.1, "distance_m": 0.3},
        {"id": "bottle_5", "bottle": 5, "left_m": -0.5, "distance_m": 0.7},
    ]


@pytest.mark.parametrize(
    ("selector", "index"),
    [("rightmost", 4), ("leftmost", 3), ("nearest", 3), ("furthest", 4), ("bottle_1", 0)],
)
def test_selects_requested_bottle_in_robot_frame(bottles, selector, index):
    assert select_object(bottles, selector) == index


def test_missing_bottle_never_substitutes_another(bottles):
    with pytest.raises(ValueError, match="Choose bottle"):
        select_object(bottles, "bottle_2")


def test_empty_inventory_reports_no_eligible_bottle():
    with pytest.raises(ValueError, match="No eligible"):
        select_object([], "nearest")


@pytest.fixture
def home_skills(mocker):
    skills = R1ProHomeSkills()
    skills._sim = mocker.Mock()
    skills._control = mocker.Mock()
    skills._policy = mocker.Mock()
    skills._policy.stop_rollout.return_value = {"active": False, "last_error": None}
    yield skills
    skills.stop()


@pytest.mark.parametrize("contacts_after", [2, None])
def test_unloading_seeks_support_before_opening_and_has_a_finite_limit(
    home_skills, mocker, contacts_after
):
    skills = home_skills
    commands = []
    clock = [0.0]
    joints = list(R1PRO_PICK_PLACE_JOINTS)
    q = [0.0] * len(joints)
    skills._sim.inventory.return_value = {"bottles": []}
    skills._sim.task_state.return_value = {
        "robot_obstacles": [],
        "tray": {"support_geoms": ["table"]},
        "bottles": [],
    }
    skills._control.get_joint_positions.side_effect = lambda: dict(zip(joints, q, strict=True))
    mocker.patch(
        "dimos.robot.galaxea.r1pro.home_skills.time.monotonic", side_effect=lambda: clock[0]
    )
    mocker.patch.object(
        skills, "_pause", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)
    )

    def execute(trajectory, task):
        commands.append(trajectory.points[-1].positions)
        q[:] = trajectory.points[-1].positions
        return mocker.Mock(status=TrajectoryExecutionStatus.ACCEPTED)

    skills._control.execute_trajectory.side_effect = execute

    def bottle_state(index):
        supported = contacts_after is not None and q[0] <= -0.003 * contacts_after + 1e-9
        return {
            "support_geoms": ["table"] if supported else [],
            "grasped": q[-1] == 0,
            "released": q[-1] == 0.05,
            "upright": True,
            "velocity_norm": 0.0,
            "position": [1.0, 2.0, 0.8],
        }

    skills._sim.bottle_state.side_effect = bottle_state

    def descent(index, support):
        target = q.copy()
        target[0] -= 0.003
        return {
            "supported": False,
            "waypoint": {"phase": "unload_seek_support", "seconds": 0.6, "positions": target},
        }

    skills._sim.plan_bottle_descent.side_effect = descent
    plan = {
        "support_geom": "table",
        "target": [1.0, 2.0, 0.73],
        "waypoints": [
            {"phase": "unload_release", "seconds": 1.0, "positions": [0.0] * 19 + [0.05]}
        ],
    }
    report = {"stages": []}
    if contacts_after is None:
        with pytest.raises(RuntimeError, match="24 mm"):
            skills._unload(0, plan, report)
        assert len(commands) == 8
        assert all(command[-1] == 0.0 for command in commands)
    else:
        skills._unload(0, plan, report)
        assert len(commands) == contacts_after + 1
        assert all(command[-1] == 0.0 for command in commands[:-1])
        assert commands[-1][0] == pytest.approx(-0.006)
        assert commands[-1][-1] == 0.05
        assert report["success"] is True


def test_unresolved_failure_blocks_new_motion_but_allows_explicit_reset(home_skills, mocker):
    home_skills._needs_recovery = True
    operation = mocker.Mock()
    result = json.loads(home_skills._start_action("pick nearest", operation))
    assert result["accepted"] is False
    assert result["reason"] == "recovery_required"
    operation.assert_not_called()


def test_reset_restores_observations_and_clears_delivery_progress(home_skills, mocker):
    skills = home_skills
    skills._station = "kitchen"
    skills._delivered = {0: "dining_table"}
    skills._arm_command = [1.0] * 20
    skills._sim.reset.return_value = True
    skills._sim.packing_state.return_value = {"ready_for_pick": True}

    def run_now(name, operation):
        report = {"stages": []}
        operation(report)
        return str(report)

    mocker.patch.object(skills, "_start_action", side_effect=run_now)
    skills.reset_scene()
    skills._policy.stop_rollout.assert_called_once()
    skills._sim.reset.assert_called_once()
    skills._sim.restore_packing_observations.assert_called_once()
    assert skills._policy.clear_rollout_observations.call_count == 2
    assert skills._station == "worktop"
    assert skills._delivered == {}
    assert skills._arm_command is None


@pytest.mark.parametrize("obstacle", ["task_bottle_2", "chair"])
def test_recovery_only_permits_the_supported_contact_it_is_releasing(home_skills, mocker, obstacle):
    skills = home_skills
    joints = list(R1PRO_PICK_PLACE_JOINTS)
    clock = [0.0]
    skills._sim.plan_pick_recovery.return_value = [
        {
            "phase": "recovery_release",
            "seconds": 0.8,
            "positions": [0.0] * len(joints),
            "allowed_contacts": ["task_bottle_2"],
        }
    ]
    skills._sim.inventory.return_value = {"bottles": []}
    skills._sim.task_state.return_value = {
        "robot_obstacles": [obstacle],
        "tray": {"support_geoms": ["table"]},
    }
    skills._control.get_joint_positions.return_value = dict.fromkeys(joints, 0.0)
    skills._control.execute_trajectory.return_value = mocker.Mock(
        status=TrajectoryExecutionStatus.ACCEPTED
    )
    mocker.patch(
        "dimos.robot.galaxea.r1pro.home_skills.time.monotonic", side_effect=lambda: clock[0]
    )
    mocker.patch.object(
        skills, "_pause", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)
    )
    report = {"stages": []}
    if obstacle == "chair":
        with pytest.raises(RuntimeError, match="contacted an obstacle"):
            skills._recover_pick(report)
        assert "recovery" not in report
    else:
        skills._recover_pick(report)
        assert report["recovery"]["success"] is True
    skills._control.cancel_trajectory.assert_called_with("tray_manipulation")


@pytest.mark.parametrize("safe", [True, False])
def test_recovery_checks_cancellation_instead_of_rejecting_an_old_inference_error(
    home_skills, mocker, safe
):
    skills = home_skills
    skills._policy.stop_rollout.return_value = {
        "active": False,
        "last_error": "camera observation is stale",
    }
    skills._control.cancel_trajectory.return_value = mocker.Mock(
        safe=safe, message="unconfirmed stop"
    )
    if safe:
        skills._stop_act_for_recovery()
        skills._policy.clear_rollout_observations.assert_called_once()
    else:
        with pytest.raises(RuntimeError, match="cancellation was uncertain"):
            skills._stop_act_for_recovery()
        skills._policy.clear_rollout_observations.assert_not_called()
    skills._control.cancel_trajectory.assert_called_once_with("policy_rollout")


@pytest.fixture
def recovery_control(home_skills, mocker):
    skills = home_skills
    clock = [0.0]
    joints = list(R1PRO_PICK_PLACE_JOINTS)
    skills._sim.inventory.return_value = {"bottles": []}
    skills._sim.task_state.return_value = {
        "robot_obstacles": [],
        "tray": {"support_geoms": ["table"]},
    }
    skills._control.get_joint_positions.return_value = dict.fromkeys(joints, 0.0)
    skills._control.execute_trajectory.return_value = mocker.Mock(
        status=TrajectoryExecutionStatus.ACCEPTED
    )
    mocker.patch(
        "dimos.robot.galaxea.r1pro.home_skills.time.monotonic", side_effect=lambda: clock[0]
    )
    mocker.patch.object(
        skills, "_pause", side_effect=lambda seconds: clock.__setitem__(0, clock[0] + seconds)
    )
    return skills, clock, joints


def test_recovery_stops_lowering_when_worktop_contact_never_arrives(recovery_control):
    skills, _, joints = recovery_control
    skills._sim.plan_pick_recovery.return_value = [
        {
            "phase": "recovery_seek_support",
            "seconds": 0.6,
            "positions": [0.0] * len(joints),
        }
    ]
    with pytest.raises(RuntimeError, match="no worktop support after 24 mm"):
        skills._recover_pick({"stages": []})
    assert skills._control.execute_trajectory.call_count == 8


def test_recovery_waits_for_joint_settling_before_declaring_home(recovery_control):
    skills, clock, joints = recovery_control
    skills._sim.plan_pick_recovery.return_value = [
        {
            "phase": "recovery_posture",
            "seconds": 1.0,
            "positions": [0.0] * len(joints),
        }
    ]
    skills._control.get_joint_positions.side_effect = lambda: dict.fromkeys(
        joints, 0.02 if clock[0] < 2.0 else 0.0
    )
    report = {"stages": []}
    skills._recover_pick(report)
    assert clock[0] >= 2.0
    assert report["recovery"]["success"] is True
