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

from __future__ import annotations

import threading
from typing import cast
from unittest.mock import MagicMock, create_autospec

import pytest

from dimos.agents.skill_result import SkillResult
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.mobile.target_observation import TargetObservation
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.unitree.g1.manip_stance import PourReachMap
from dimos.robot.unitree.g1.watering_task import (
    BaseCommandSink,
    WateringInputs,
    WateringManipulationSpec,
    WateringSequence,
    WateringState,
    WateringTaskConfig,
)
from dimos.spec.utils import spec_annotation_compliance

NOW = 100.0
TARGET_ID = "plant_pot_1"


@pytest.fixture
def reach_map() -> PourReachMap:
    return PourReachMap.load()


@pytest.fixture
def manipulation() -> MagicMock:
    mock = create_autospec(WateringManipulationSpec, instance=True)
    mock.reset.return_value = SkillResult.ok("reset")
    mock.latch_base_pose.return_value = True
    mock.plan_to_pose.return_value = True
    mock.clear_planned_path.return_value = True
    mock.get_error.return_value = ""
    mock.move_to_pose.return_value = SkillResult.ok("moved")
    mock.go_init.return_value = SkillResult.ok("home")
    return mock


@pytest.fixture
def base() -> MagicMock:
    return create_autospec(BaseCommandSink, instance=True)


def _inputs(
    reach_map: PourReachMap,
    *,
    target_observed_at: float = NOW,
    at_reachable_stance: bool = True,
    base_z: float = 0.74,
) -> WateringInputs:
    pot_x, pot_y = 1.0, 0.0
    if at_reachable_stance:
        offset_x, offset_y = reach_map.best_offset(margin_cells=3)
        base_x, base_y = pot_x - offset_x, pot_y - offset_y
    else:
        base_x, base_y = 0.0, 0.0
    inputs = WateringInputs()
    inputs.update_target(
        TargetObservation(
            object_id=TARGET_ID,
            label="plant pot",
            pose=PoseStamped(
                ts=target_observed_at,
                frame_id="world",
                position=[pot_x, pot_y, 0.0],
                orientation=[0.0, 0.0, 0.0, 1.0],
            ),
            source="sim_ground_truth",
            observed_at=target_observed_at,
        )
    )
    inputs.update_base_pose(
        PoseStamped(
            ts=NOW,
            frame_id="world",
            position=[base_x, base_y, base_z],
            orientation=[0.0, 0.0, 0.0, 1.0],
        )
    )
    return inputs


def _sequence(
    reach_map: PourReachMap,
    inputs: WateringInputs,
    base: MagicMock,
    manipulation: MagicMock,
    cancelled: threading.Event,
    transitions: list[WateringState],
    wait: MagicMock | None = None,
) -> WateringSequence:
    return WateringSequence(
        config=WateringTaskConfig(
            settle_seconds=0.0,
            hold_seconds=0.0,
            target_max_age=2.0,
            base_pose_max_age=2.0,
        ),
        inputs=inputs,
        base=cast("BaseCommandSink", base),
        manipulation=cast("WateringManipulationSpec", manipulation),
        reach_map=reach_map,
        cancelled=cancelled,
        transition=lambda state, _message, _attempt: transitions.append(state),
        wait=wait or MagicMock(return_value=False),
        monotonic=lambda: NOW,
        wall_time=lambda: NOW,
    )


def test_manipulation_module_satisfies_the_task_dependency() -> None:
    assert spec_annotation_compliance(ManipulationModule, WateringManipulationSpec)


def test_successful_sequence_preserves_the_verified_sim_order(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    transitions: list[WateringState] = []
    sequence = _sequence(
        reach_map,
        _inputs(reach_map),
        base,
        manipulation,
        threading.Event(),
        transitions,
    )

    result = sequence.run(TARGET_ID)

    assert result.success
    assert result.state is WateringState.COMPLETED
    assert transitions == [
        WateringState.WAITING_INPUT,
        WateringState.RESETTING,
        WateringState.APPROACHING,
        WateringState.SETTLING,
        WateringState.LATCHING_BASE,
        WateringState.VERIFYING_REACH,
        WateringState.MOVING_OVER_TARGET,
        WateringState.TIPPING,
        WateringState.HOLDING,
        WateringState.RETURNING,
        WateringState.COMPLETED,
    ]
    manipulation.latch_base_pose.assert_called_once()
    assert manipulation.plan_to_pose.call_count == 2
    assert manipulation.move_to_pose.call_count == 2
    manipulation.go_init.assert_called_once_with("g1", "g1/right_arm")
    base.send.assert_not_called()
    assert base.stop.call_count == 2


def test_approach_only_reaches_the_stance_without_touching_manipulation(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    transitions: list[WateringState] = []
    sequence = _sequence(
        reach_map,
        _inputs(reach_map),
        base,
        manipulation,
        threading.Event(),
        transitions,
    )

    result = sequence.run_approach(TARGET_ID)

    assert result.success
    assert result.state is WateringState.COMPLETED
    assert "arm was not moved" in result.message
    assert transitions == [
        WateringState.WAITING_INPUT,
        WateringState.APPROACHING,
        WateringState.SETTLING,
        WateringState.COMPLETED,
    ]
    manipulation.reset.assert_not_called()
    manipulation.latch_base_pose.assert_not_called()
    manipulation.plan_to_pose.assert_not_called()
    manipulation.move_to_pose.assert_not_called()
    manipulation.go_init.assert_not_called()
    base.send.assert_not_called()
    assert base.stop.call_count == 2


def test_pour_only_executes_the_arm_without_commanding_base_motion(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    transitions: list[WateringState] = []
    sequence = _sequence(
        reach_map,
        _inputs(reach_map),
        base,
        manipulation,
        threading.Event(),
        transitions,
    )

    result = sequence.run_pour(TARGET_ID)

    assert result.success
    assert result.state is WateringState.COMPLETED
    assert "without moving the base" in result.message
    assert transitions == [
        WateringState.WAITING_INPUT,
        WateringState.RESETTING,
        WateringState.LATCHING_BASE,
        WateringState.VERIFYING_REACH,
        WateringState.MOVING_OVER_TARGET,
        WateringState.TIPPING,
        WateringState.HOLDING,
        WateringState.RETURNING,
        WateringState.COMPLETED,
    ]
    manipulation.latch_base_pose.assert_called_once()
    assert manipulation.plan_to_pose.call_count == 2
    assert manipulation.move_to_pose.call_count == 2
    manipulation.go_init.assert_called_once_with("g1", "g1/right_arm")
    base.send.assert_not_called()
    assert base.stop.call_count == 2


def test_pour_only_rejects_an_unreachable_stance_before_arm_motion(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    transitions: list[WateringState] = []
    sequence = _sequence(
        reach_map,
        _inputs(reach_map, at_reachable_stance=False),
        base,
        manipulation,
        threading.Event(),
        transitions,
    )

    result = sequence.run_pour(TARGET_ID)

    assert not result.success
    assert result.state is WateringState.FAILED
    assert "outside the verified pour region" in result.message
    manipulation.reset.assert_not_called()
    manipulation.latch_base_pose.assert_not_called()
    manipulation.plan_to_pose.assert_not_called()
    manipulation.move_to_pose.assert_not_called()
    base.send.assert_not_called()
    assert base.stop.call_count == 2


def test_pour_only_converts_ground_relative_height_into_the_lio_world_frame(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    sequence = _sequence(
        reach_map,
        _inputs(reach_map, base_z=-0.27),
        base,
        manipulation,
        threading.Event(),
        [],
    )

    result = sequence.run_pour(TARGET_ID)

    assert result.success
    expected_world_z = -0.27 - 0.74 + 0.90
    planned_pose = manipulation.plan_to_pose.call_args_list[0].args[0]
    assert planned_pose.position.z == pytest.approx(expected_world_z)
    assert manipulation.move_to_pose.call_args_list[0].kwargs["z"] == pytest.approx(
        expected_world_z
    )


def test_cancellation_during_approach_stops_before_latching_or_pouring(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    cancelled = threading.Event()
    wait = MagicMock(side_effect=lambda _seconds: (cancelled.set() or True))
    transitions: list[WateringState] = []
    sequence = _sequence(
        reach_map,
        _inputs(reach_map, at_reachable_stance=False),
        base,
        manipulation,
        cancelled,
        transitions,
        wait,
    )

    result = sequence.run(TARGET_ID)

    assert not result.success
    assert result.state is WateringState.CANCELLED
    assert transitions[-1] is WateringState.CANCELLED
    base.send.assert_called_once()
    base.stop.assert_called_once()
    manipulation.latch_base_pose.assert_not_called()
    manipulation.move_to_pose.assert_not_called()


def test_stale_target_fails_before_the_robot_moves(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    transitions: list[WateringState] = []
    sequence = _sequence(
        reach_map,
        _inputs(reach_map, target_observed_at=NOW - 3.0),
        base,
        manipulation,
        threading.Event(),
        transitions,
    )

    result = sequence.run(TARGET_ID)

    assert not result.success
    assert result.state is WateringState.FAILED
    assert "stale" in result.message.lower()
    manipulation.reset.assert_not_called()
    base.send.assert_not_called()
    base.stop.assert_called_once()


def test_reach_verification_exhausts_bounded_retries(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    manipulation.plan_to_pose.return_value = False
    manipulation.get_error.return_value = "no IK"
    transitions: list[WateringState] = []
    sequence = _sequence(
        reach_map,
        _inputs(reach_map),
        base,
        manipulation,
        threading.Event(),
        transitions,
    )

    result = sequence.run(TARGET_ID)

    assert not result.success
    assert result.state is WateringState.FAILED
    assert "after 3 attempts" in result.message
    assert manipulation.latch_base_pose.call_count == 3
    assert manipulation.plan_to_pose.call_count == 6
    assert manipulation.reset.call_count == 4
    # A failed arm-plan check must not trigger an unverified blind base nudge.
    base.send.assert_not_called()
    manipulation.move_to_pose.assert_not_called()
