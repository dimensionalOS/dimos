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

"""Where the home buttons and the manual moves send each arm, and how fast."""

from collections.abc import Iterator
from itertools import pairwise
import json
from pathlib import Path
from typing import Any
from unittest.mock import Mock

import pytest
import pytest_mock

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.manipulation.arm_home import (
    ArmHomeModule,
    HomeArm,
    load_saved_homes,
    manual_trajectory,
)
from dimos.manipulation.manipulation_spec import (
    ExecutionResult,
    ExecutionStatus,
    ManipulationSpec,
    PlanningGroupInfo,
    PlanResult,
    PlanStatus,
)
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.testing.waiting import wait_until

HOME = [0.5, -0.4]
REST = [0.0, 0.0]
VIA = [0.2, 0.3]
SPEEDS = [2.0, 4.0]
ARMS: dict[str, dict[str, Any]] = {
    side: {
        "home": HOME,
        "rest": REST,
        "via": VIA,
        "max_joint_speeds": SPEEDS,
        "button": button,
        "grip": grip,
    }
    for side, button, grip in (("left", "X", "LG"), ("right", "A", "RG"))
}
MEASURED = {"l1": 0.1, "l2": 0.2, "r1": -0.1, "r2": -0.2}


@pytest.fixture
def planner() -> Mock:
    planner = Mock(spec=ManipulationSpec)
    planner.list_planning_groups.return_value = (
        PlanningGroupInfo("left", ("l1", "l2"), "base", "left_tip", True),
        PlanningGroupInfo("right", ("r1", "r2"), "base", "right_tip", True),
    )
    planner.plan_to_joints.return_value = PlanResult(
        PlanStatus.SUCCEEDED, plan=GeneratedPlan(("left",), JointTrajectory())
    )
    planner.execute.return_value = ExecutionResult(ExecutionStatus.ACCEPTED)
    planner.wait_for_execution.return_value = ExecutionResult(ExecutionStatus.COMPLETED)
    return planner


@pytest.fixture
def coordinator() -> Mock:
    coordinator = Mock(spec=ControlCoordinator)
    coordinator.get_joint_positions.return_value = dict(MEASURED)
    coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ACCEPTED
    )
    return coordinator


@pytest.fixture
def module(
    planner: Mock, coordinator: Mock, tmp_path: Path, mocker: pytest_mock.MockerFixture
) -> Iterator[ArmHomeModule]:
    mocker.patch("dimos.manipulation.arm_home.HOME_STATE_DIR", tmp_path)
    instance = ArmHomeModule(robot="rig", arms=ARMS)
    instance.manipulation = planner
    instance.coordinator = coordinator
    yield instance
    instance.stop()


def _buttons(*names: str) -> Buttons:
    msg = Buttons()
    for name in names:
        msg.set_attribute(name, True)
    return msg


def _planned(planner: Mock) -> list[tuple[str, list[float]]]:
    return [
        (arm, list(target.position))
        for call in planner.plan_to_joints.call_args_list
        for arm, target in call.args[0].items()
    ]


def test_manual_trajectory_runs_each_leg_at_the_given_fraction_of_the_slowest_joint() -> None:
    trajectory = manual_trajectory(["a", "b"], [[0.0, 0.0], [1.0, 0.0], [1.0, 2.0]], SPEEDS, 0.2)

    # Leg 1: joint a moves 1 rad at 0.2 * 2 rad/s, times 1.5 for the smooth profile.
    # Leg 2: joint b moves 2 rad at 0.2 * 4 rad/s, times 1.5.
    assert trajectory.duration == pytest.approx(3.75 + 3.75)
    assert trajectory.sample(0.0)[0] == pytest.approx([0.0, 0.0])
    assert trajectory.sample(3.75)[0] == pytest.approx([1.0, 0.0])
    assert trajectory.sample(7.5)[0] == pytest.approx([1.0, 2.0])
    times = [point.time_from_start for point in trajectory.points]
    assert times == sorted(set(times))
    peak = max(
        abs(after.positions[0] - before.positions[0])
        / (after.time_from_start - before.time_from_start)
        for before, after in pairwise(trajectory.points)
    )
    assert peak <= 0.2 * SPEEDS[0] + 1e-9


def test_manual_trajectory_skips_a_leg_that_does_not_move() -> None:
    trajectory = manual_trajectory(["a", "b"], [[1.0, 2.0], [1.0, 2.0], [0.0, 2.0]], SPEEDS, 0.5)

    times = [point.time_from_start for point in trajectory.points]
    assert times == sorted(set(times))
    assert trajectory.duration == pytest.approx(1.5)


def test_manual_home_starts_where_the_arm_is_and_passes_the_via_pose(
    module: ArmHomeModule, coordinator: Mock
) -> None:
    assert module.manual_move("right", "home").startswith("started")

    [call] = coordinator.execute_trajectory.call_args_list
    trajectory = call.args[0]
    assert trajectory.joint_names == ["r1", "r2"]
    positions = [point.positions for point in trajectory.points]
    assert positions[0] == pytest.approx([-0.1, -0.2])
    assert any(position == pytest.approx(VIA) for position in positions)
    assert positions[-1] == pytest.approx(HOME)


def test_manual_de_home_ends_at_rest_and_reports_a_refusal(
    module: ArmHomeModule, coordinator: Mock
) -> None:
    coordinator.execute_trajectory.return_value = TrajectoryExecutionResult(
        TrajectoryExecutionStatus.ESTOPPED, "stopped"
    )

    assert module.manual_move("left", "rest") == "refused: stopped"
    assert coordinator.execute_trajectory.call_args.args[0].points[-1].positions == REST


def test_a_saved_home_replaces_the_default_and_survives_a_restart(
    module: ArmHomeModule, planner: Mock, tmp_path: Path
) -> None:
    assert module.save_home("left") == [0.1, 0.2]

    assert module.get_homes() == {
        "left": {"home": [0.1, 0.2], "saved": True},
        "right": {"home": HOME, "saved": False},
    }
    assert json.loads((tmp_path / "rig.json").read_text()) == {"left": [0.1, 0.2]}
    arms = {name: HomeArm(**spec) for name, spec in ARMS.items()}
    assert load_saved_homes(tmp_path / "rig.json", arms) == {"left": [0.1, 0.2]}

    module._on_pressed(_buttons("left_primary"))
    wait_until(lambda: planner.wait_for_execution.called, timeout=5.0, interval=0.01)
    assert _planned(planner) == [("left", [0.1, 0.2])]

    assert module.clear_home("left") == HOME
    assert json.loads((tmp_path / "rig.json").read_text()) == {}


def test_each_home_button_plans_its_own_arm(module: ArmHomeModule, planner: Mock) -> None:
    module._on_pressed(_buttons("right_primary"))
    wait_until(lambda: planner.wait_for_execution.called, timeout=5.0, interval=0.01)

    assert _planned(planner) == [("right", HOME)]


def test_a_home_button_is_ignored_while_its_own_grip_is_held(
    module: ArmHomeModule, planner: Mock
) -> None:
    module._on_held(_buttons("left_grip"))
    module._on_pressed(_buttons("left_primary"))

    planner.plan_to_joints.assert_not_called()


def test_the_other_arm_s_grip_does_not_block_a_home_button(
    module: ArmHomeModule, planner: Mock
) -> None:
    """The operator can keep teleoperating one arm while sending the other home."""
    module._on_held(_buttons("right_grip"))
    module._on_pressed(_buttons("left_primary"))
    wait_until(lambda: planner.wait_for_execution.called, timeout=5.0, interval=0.01)

    assert _planned(planner) == [("left", HOME)]


def test_a_saved_home_with_the_wrong_joint_count_is_refused(tmp_path: Path) -> None:
    path = tmp_path / "rig.json"
    path.write_text(json.dumps({"left": [0.1, 0.2, 0.3]}))
    arms = {name: HomeArm(**spec) for name, spec in ARMS.items()}

    with pytest.raises(ValueError, match="invalid home for 'left'"):
        load_saved_homes(path, arms)
