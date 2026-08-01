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

import pytest

from dimos.teleop.openarm_mini.mapping import (
    OpenArmMiniMappingError,
    combine_side_commands,
    map_side_readings,
)


def _readings() -> dict[str, float]:
    return {
        "joint_1": 0.1,
        "joint_2": 0.2,
        "joint_3": 0.3,
        "joint_4": 0.4,
        "joint_5": 0.5,
        "joint_6": 0.6,
        "joint_7": 0.7,
    }


def test_mapping_uses_direct_arm_joint_assignment_and_follower_names() -> None:
    command = map_side_readings("left", _readings())

    assert list(command.positions_by_joint) == [f"openarm_left_joint{i}" for i in range(1, 8)]
    assert command.positions_by_joint["openarm_left_joint1"] == pytest.approx(0.1)
    assert command.positions_by_joint["openarm_left_joint6"] == pytest.approx(0.6)
    assert command.positions_by_joint["openarm_left_joint7"] == pytest.approx(0.7)
    assert not hasattr(command, "gripper_position")


def test_combined_command_uses_openarm_follower_joint_names() -> None:
    left = map_side_readings("left", _readings())
    right = map_side_readings("right", _readings())

    joint_state = combine_side_commands([left, right])

    assert joint_state.name == [
        *[f"openarm_left_joint{i}" for i in range(1, 8)],
        *[f"openarm_right_joint{i}" for i in range(1, 8)],
    ]
    assert len(joint_state.position) == 14


def test_mapping_can_emit_configured_target_joint_names() -> None:
    target_names = [f"right_arm/openarm_right_joint{i}" for i in range(1, 8)]

    command = map_side_readings("right", _readings(), target_joint_names=target_names)

    assert list(command.positions_by_joint) == target_names
    assert command.positions_by_joint["right_arm/openarm_right_joint1"] == pytest.approx(0.1)
    assert not hasattr(command, "gripper_position")


def test_follower_joint_limits_clamp_sender_side() -> None:
    readings = _readings()
    readings["joint_1"] = 5.0
    readings["joint_4"] = -1.0

    left = map_side_readings("left", readings)
    right = map_side_readings("right", readings)

    assert left.positions_by_joint["openarm_left_joint1"] == pytest.approx(1.35)
    assert right.positions_by_joint["openarm_right_joint1"] == pytest.approx(3.45)
    assert left.positions_by_joint["openarm_left_joint4"] == pytest.approx(-0.01)


def test_jump_threshold_rejects_large_leader_discontinuity_after_clamp() -> None:
    previous = map_side_readings("left", _readings()).positions_by_joint
    readings = _readings()
    readings["joint_2"] = -1.0

    with pytest.raises(OpenArmMiniMappingError, match="exceeds"):
        map_side_readings(
            "left",
            readings,
            previous_positions_by_joint=previous,
            max_joint_jump_radians=0.5,
        )


def test_missing_leader_arm_joint_reading_is_rejected() -> None:
    readings = _readings()
    del readings["joint_4"]

    with pytest.raises(OpenArmMiniMappingError, match="missing"):
        map_side_readings("left", readings)


def test_gripper_reading_is_not_required() -> None:
    command = map_side_readings("right", _readings())

    assert list(command.positions_by_joint) == [f"openarm_right_joint{i}" for i in range(1, 8)]
