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

import pytest

from dimos.robot.unitree.g1.manip_last_mile import (
    MAX_ANGULAR,
    MAX_LINEAR,
    MIN_ANGULAR,
    ApproachControllerConfig,
    ApproachPhase,
    Twist2D,
    approach_step,
)


def test_turns_in_place_before_walking() -> None:
    step = approach_step((0.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0))

    assert step.phase is ApproachPhase.TURN_TO_PATH
    assert step.command.vx == 0.0
    assert step.command.vy == 0.0
    assert step.command.wz < 0.0


def test_walk_is_forward_only_with_no_strafe() -> None:
    step = approach_step((0.0, 0.0, 0.05), (1.0, 0.0, 0.0))

    assert step.phase is ApproachPhase.WALK_FORWARD
    assert 0.0 < step.command.vx <= MAX_LINEAR
    assert step.command.vy == 0.0
    assert abs(step.command.wz) <= MAX_ANGULAR


def test_walk_heading_correction_can_decay_below_in_place_turn_rate() -> None:
    step = approach_step((0.0, 0.0, 0.01), (1.0, 0.0, 0.0))

    assert step.phase is ApproachPhase.WALK_FORWARD
    assert step.command.wz == pytest.approx(-0.012)
    assert abs(step.command.wz) < MIN_ANGULAR


def test_holonomic_servo_holds_yaw_and_translates_directly_to_stance() -> None:
    config = ApproachControllerConfig(
        holonomic=True,
        max_linear=0.18,
        max_lateral=0.18,
    )

    step = approach_step(
        (0.0, 0.0, math.pi / 2),
        (1.0, 0.0, math.pi / 2),
        config,
    )

    assert step.phase is ApproachPhase.SERVO_TO_STANCE
    assert step.command.vx == pytest.approx(0.0, abs=1e-12)
    assert step.command.vy == pytest.approx(-0.18)
    assert step.command.wz == 0.0


def test_holonomic_servo_limits_translation_vector_and_uses_shortest_yaw() -> None:
    config = ApproachControllerConfig(
        holonomic=True,
        max_linear=0.18,
        max_lateral=0.18,
    )

    step = approach_step(
        (0.0, 0.0, math.radians(170.0)),
        (1.0, 1.0, math.radians(-170.0)),
        config,
    )

    assert step.phase is ApproachPhase.SERVO_TO_STANCE
    assert abs(step.command.vx) <= 0.18
    assert abs(step.command.vy) <= 0.18
    assert step.command.wz > 0.0


def test_holonomic_servo_commands_usable_gait_speed_for_small_error() -> None:
    config = ApproachControllerConfig(
        holonomic=True,
        min_linear=0.10,
        position_tolerance=0.01,
    )

    step = approach_step((0.0, 0.0, 0.0), (0.04, -0.02, 0.0), config)

    assert step.phase is ApproachPhase.SERVO_TO_STANCE
    assert step.command.vx == pytest.approx(0.10)
    assert step.command.vy == pytest.approx(-0.10)


def test_turns_to_final_yaw_only_after_reaching_position() -> None:
    step = approach_step((0.98, 0.0, 0.0), (1.0, 0.0, math.pi / 2))

    assert step.phase is ApproachPhase.TURN_TO_STANCE
    assert step.command.vx == 0.0
    assert step.command.vy == 0.0
    assert step.command.wz > 0.0


def test_arrives_only_when_position_and_yaw_are_satisfied() -> None:
    step = approach_step((0.98, 0.01, 0.02), (1.0, 0.0, 0.0))

    assert step.phase is ApproachPhase.ARRIVED
    assert step.command.is_stop


def test_far_or_non_finite_goal_blocks_without_motion() -> None:
    config = ApproachControllerConfig(max_approach_distance=1.0)

    far = approach_step((0.0, 0.0, 0.0), (1.1, 0.0, 0.0), config)
    invalid = approach_step((0.0, 0.0, 0.0), (math.nan, 0.0, 0.0), config)

    assert far.phase is ApproachPhase.BLOCKED
    assert invalid.phase is ApproachPhase.BLOCKED
    assert far.command.is_stop
    assert invalid.command.is_stop


def test_controller_converges_with_unicycle_kinematics() -> None:
    current = [0.0, -0.5, math.radians(100.0)]
    goal = (1.0, 0.2, math.radians(-30.0))
    dt = 0.05

    for _ in range(2000):
        step = approach_step((current[0], current[1], current[2]), goal)
        assert step.command.vy == 0.0
        assert step.command.vx >= 0.0
        if step.arrived:
            break
        assert not step.blocked
        current[0] += math.cos(current[2]) * step.command.vx * dt
        current[1] += math.sin(current[2]) * step.command.vx * dt
        current[2] += step.command.wz * dt
    else:
        pytest.fail("turn-walk-turn controller did not converge")

    assert math.hypot(goal[0] - current[0], goal[1] - current[1]) <= 0.06


def test_stop_twist_is_all_zero() -> None:
    assert Twist2D().is_stop
    assert not Twist2D(vx=0.1).is_stop
