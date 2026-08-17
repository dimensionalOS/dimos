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

import math
import threading
from typing import cast
from unittest.mock import MagicMock, create_autospec

import pytest

from dimos.agents.skill_result import SkillResult
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.mobile.target_observation import TargetObservation
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.manip_stance import (
    DEFAULT_SPOUT_OFFSET_IN_PALM,
    POUR_Z,
    RIGHT_PALM_FRAME,
    TIP_RADIANS,
    WATERING_SPOUT_FRAME,
    PourReachMap,
    palm_pose_for_spout,
    pot_in_base_frame,
)
from dimos.robot.unitree.g1.watering_task import (
    ApproachCommandSink,
    WateringInputs,
    WateringManipulationSpec,
    WateringSequence,
    WateringState,
    WateringTaskConfig,
    build_approach_preview,
    watering_spout_transform,
)
from dimos.spec.utils import spec_annotation_compliance

NOW = 100.0
TARGET_ID = "plant_pot_1"


def test_spout_tcp_offset_moves_the_palm_and_keeps_the_spout_fixed() -> None:
    spout = Vector3(1.0, 2.0, 0.9)
    offset = (0.0, 0.20, 0.0)

    upright = palm_pose_for_spout(
        spout,
        Quaternion.from_euler(Vector3(0.0, 0.0, 0.0)),
        offset,
    )
    tipped = palm_pose_for_spout(
        spout,
        Quaternion.from_euler(Vector3(-math.pi / 2.0, 0.0, 0.0)),
        offset,
    )

    assert tuple(upright.position) == pytest.approx((1.0, 1.8, 0.9))
    # Rolling the can left rotates its +y spout below the palm, so the palm
    # rises 20 cm while the water-exit point stays at the requested pour Z.
    assert tuple(tipped.position) == pytest.approx((1.0, 2.0, 1.1))


def test_spout_tcp_has_an_explicit_palm_fixed_tf() -> None:
    transform = watering_spout_transform((0.0, 0.20, 0.0), ts=123.0)

    assert transform.frame_id == RIGHT_PALM_FRAME
    assert transform.child_frame_id == WATERING_SPOUT_FRAME
    assert tuple(transform.translation) == pytest.approx((0.0, 0.20, 0.0))
    assert tuple(transform.rotation) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert transform.ts == 123.0


@pytest.fixture
def reach_map() -> PourReachMap:
    return PourReachMap.load()


@pytest.fixture
def manipulation() -> MagicMock:
    mock = create_autospec(WateringManipulationSpec, instance=True)
    mock.reset.return_value = SkillResult.ok("reset")
    mock.latch_base_pose.return_value = True
    mock.get_error.return_value = ""
    mock.move_to_pose.return_value = SkillResult.ok("moved")
    mock.go_init.return_value = SkillResult.ok("home")
    return mock


@pytest.fixture
def base() -> MagicMock:
    return create_autospec(ApproachCommandSink, instance=True)


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
    monotonic: MagicMock | None = None,
    config: WateringTaskConfig | None = None,
) -> WateringSequence:
    return WateringSequence(
        config=config
        or WateringTaskConfig(
            settle_seconds=0.0,
            hold_seconds=0.0,
            target_max_age=2.0,
            base_pose_max_age=2.0,
        ),
        inputs=inputs,
        approach_commands=cast("ApproachCommandSink", base),
        manipulation=cast("WateringManipulationSpec", manipulation),
        reach_map=reach_map,
        cancelled=cancelled,
        transition=lambda state, _message, _attempt: transitions.append(state),
        wait=wait or MagicMock(return_value=False),
        monotonic=monotonic or (lambda: NOW),
        wall_time=lambda: NOW,
    )


def test_manipulation_module_satisfies_the_task_dependency() -> None:
    assert spec_annotation_compliance(ManipulationModule, WateringManipulationSpec)


def test_approach_preview_is_an_oriented_pose_path(reach_map: PourReachMap) -> None:
    inputs = _inputs(reach_map, at_reachable_stance=False)
    base_yaw = 1.1
    inputs.update_base_pose(
        PoseStamped(
            ts=NOW,
            frame_id="world",
            position=[0.0, 0.0, 0.74],
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, base_yaw)),
        )
    )
    snapshot = inputs.snapshot(TARGET_ID)
    assert snapshot is not None

    path, goal = build_approach_preview(snapshot, reach_map, margin_cells=3)

    dx = goal.position.x - path.poses[0].position.x
    dy = goal.position.y - path.poses[0].position.y
    assert path.poses[0].orientation.euler[2] == pytest.approx(math.atan2(dy, dx))
    offset = reach_map.best_offset(margin_cells=3)
    expected_yaw = -math.atan2(offset[1], offset[0])
    # Final yaw faces the pot with the right-arm map's preferred lateral
    # bearing, not the robot's arbitrary pre-walk heading.
    assert path.poses[-1].orientation.euler[2] == pytest.approx(expected_yaw)
    assert path.poses[-1].orientation.euler[2] == pytest.approx(goal.orientation.euler[2])
    assert pot_in_base_frame(
        (1.0, 0.0),
        (float(goal.position.x), float(goal.position.y)),
        float(goal.orientation.euler[2]),
    ) == pytest.approx(offset)


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
        WateringState.MOVING_OVER_TARGET,
        WateringState.TIPPING,
        WateringState.HOLDING,
        WateringState.UNTIPPING,
        WateringState.RETURNING,
        WateringState.COMPLETED,
    ]
    manipulation.latch_base_pose.assert_called_once()
    assert manipulation.move_to_pose.call_count == 3
    assert all(
        call.kwargs["pre_lift"] is False for call in manipulation.move_to_pose.call_args_list
    )
    assert all(
        call.kwargs["preview_duration"] == 0.0 for call in manipulation.move_to_pose.call_args_list
    )
    manipulation.go_init.assert_called_once_with(
        "g1", "g1/right_arm", pre_lift=False, preview_duration=0.0
    )
    base.start.assert_called_once()
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
    manipulation.move_to_pose.assert_not_called()
    manipulation.go_init.assert_not_called()
    base.start.assert_called_once()
    assert base.stop.call_count == 2


def test_settle_stage_reacquires_without_consuming_the_approach_deadline(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    inputs = _inputs(reach_map)
    arrived_snapshot = inputs.snapshot(TARGET_ID)
    assert arrived_snapshot is not None
    arrived_pose = arrived_snapshot.base_pose
    wait_calls = 0

    def drift_then_recover(_seconds: float) -> bool:
        nonlocal wait_calls
        wait_calls += 1
        if wait_calls == 1:
            inputs.update_base_pose(
                PoseStamped(
                    ts=NOW,
                    frame_id="world",
                    position=[0.0, 0.0, 0.74],
                    orientation=[0.0, 0.0, 0.0, 1.0],
                )
            )
        else:
            inputs.update_base_pose(arrived_pose)
        return False

    transitions: list[WateringState] = []
    sequence = _sequence(
        reach_map,
        inputs,
        base,
        manipulation,
        threading.Event(),
        transitions,
        wait=MagicMock(side_effect=drift_then_recover),
        config=WateringTaskConfig(
            settle_seconds=3.0,
            target_max_age=2.0,
            base_pose_max_age=2.0,
        ),
    )

    result = sequence.run_approach(TARGET_ID)

    assert result.success
    assert result.state is WateringState.COMPLETED
    assert transitions == [
        WateringState.WAITING_INPUT,
        WateringState.APPROACHING,
        WateringState.SETTLING,
        WateringState.COMPLETED,
    ]
    assert base.start.call_count == 2


def test_settle_timeout_accepts_the_closest_pose_instead_of_failing(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    inputs = _inputs(reach_map)

    def drift_while_settling(_seconds: float) -> bool:
        inputs.update_base_pose(
            PoseStamped(
                ts=NOW,
                frame_id="world",
                position=[0.0, 0.0, 0.74],
                orientation=[0.0, 0.0, 0.0, 1.0],
            )
        )
        return False

    transitions: list[WateringState] = []
    sequence = _sequence(
        reach_map,
        inputs,
        base,
        manipulation,
        threading.Event(),
        transitions,
        wait=MagicMock(side_effect=drift_while_settling),
        monotonic=MagicMock(side_effect=[0.0, 0.0, 0.0, 0.0, 16.0]),
        config=WateringTaskConfig(
            approach_timeout=90.0,
            settle_timeout=15.0,
            settle_seconds=3.0,
            target_max_age=2.0,
            base_pose_max_age=2.0,
        ),
    )

    result = sequence.run_approach(TARGET_ID)

    assert result.success
    assert result.state is WateringState.COMPLETED
    assert transitions == [
        WateringState.WAITING_INPUT,
        WateringState.APPROACHING,
        WateringState.SETTLING,
        WateringState.SETTLING,
        WateringState.COMPLETED,
    ]


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
        WateringState.MOVING_OVER_TARGET,
        WateringState.TIPPING,
        WateringState.HOLDING,
        WateringState.UNTIPPING,
        WateringState.RETURNING,
        WateringState.COMPLETED,
    ]
    manipulation.latch_base_pose.assert_called_once()
    assert manipulation.move_to_pose.call_count == 3
    manipulation.go_init.assert_called_once_with(
        "g1", "g1/right_arm", pre_lift=False, preview_duration=0.0
    )
    base.start.assert_not_called()
    assert base.stop.call_count == 2


def test_pour_uses_spout_tcp_and_keeps_the_palm_fixed_while_tipping(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    sequence = _sequence(
        reach_map,
        _inputs(reach_map),
        base,
        manipulation,
        threading.Event(),
        [],
        config=WateringTaskConfig(
            spout_offset_in_palm=(0.0, 0.20, 0.0),
            settle_seconds=0.0,
            hold_seconds=0.0,
            target_max_age=2.0,
            base_pose_max_age=2.0,
        ),
    )

    result = sequence.run_pour(TARGET_ID)

    assert result.success
    upright_move = manipulation.move_to_pose.call_args_list[0].kwargs
    tipped_move = manipulation.move_to_pose.call_args_list[1].kwargs
    untipped_move = manipulation.move_to_pose.call_args_list[2].kwargs
    assert (upright_move["x"], upright_move["y"], upright_move["z"]) == pytest.approx(
        (tipped_move["x"], tipped_move["y"], tipped_move["z"])
    )
    assert upright_move["z"] == pytest.approx(POUR_Z + 0.20)
    assert untipped_move == upright_move


def test_failed_untip_does_not_sweep_the_loaded_arm_back_to_init(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    manipulation.move_to_pose.side_effect = [
        SkillResult.ok("upright"),
        SkillResult.ok("tipped"),
        SkillResult.fail("PLANNING_FAILED", "cannot untip safely"),
    ]
    manipulation.get_error.return_value = "collision near torso"
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

    assert not result.success
    assert result.state is WateringState.FAILED
    assert "failed to return tool upright" in result.message
    assert transitions[-2:] == [WateringState.UNTIPPING, WateringState.FAILED]
    manipulation.go_init.assert_not_called()


def test_pour_only_relies_on_move_planning_as_the_live_reach_gate(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    manipulation.move_to_pose.return_value = SkillResult.fail("PLANNING_FAILED", "no IK")
    manipulation.get_error.return_value = "IK failed: NO_SOLUTION: qp-error"
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
    assert (
        "Failed to move over target: PLANNING_FAILED: no IK; "
        "planner: IK failed: NO_SOLUTION: qp-error"
    ) == result.message
    assert manipulation.reset.call_count == 2
    manipulation.latch_base_pose.assert_called_once()
    manipulation.move_to_pose.assert_called_once()
    base.start.assert_not_called()
    assert base.stop.call_count == 2


def test_pour_only_projects_an_offline_map_miss_to_the_nearest_robust_offset(
    reach_map: PourReachMap,
    manipulation: MagicMock,
    base: MagicMock,
) -> None:
    sequence = _sequence(
        reach_map,
        _inputs(reach_map, at_reachable_stance=False),
        base,
        manipulation,
        threading.Event(),
        [],
    )

    result = sequence.run_pour(TARGET_ID)

    assert result.success
    assert manipulation.move_to_pose.call_count == 3
    expected = reach_map.closest_offset((1.0, 0.0), margin_cells=3)
    upright_move = manipulation.move_to_pose.call_args_list[0].kwargs
    assert (upright_move["x"], upright_move["y"]) == pytest.approx(expected)
    base.start.assert_not_called()


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
    expected_world_z = -0.27 - 0.74 + reach_map.pour_z
    upright_move = manipulation.move_to_pose.call_args_list[0].kwargs
    expected_palm = palm_pose_for_spout(
        Vector3(1.0, 0.0, expected_world_z),
        Quaternion.from_euler(Vector3(TIP_RADIANS, 0.0, upright_move["yaw"])),
        DEFAULT_SPOUT_OFFSET_IN_PALM,
    )
    assert upright_move["z"] == pytest.approx(expected_palm.position.z)


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
    base.start.assert_called_once()
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
    base.start.assert_not_called()
    base.stop.assert_called_once()


def test_full_run_does_not_repeat_probe_plans_before_pouring(
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
    manipulation.latch_base_pose.assert_called_once()
    assert manipulation.move_to_pose.call_count == 3
    assert manipulation.reset.call_count == 1
    base.start.assert_called_once()
