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

import math

import pytest

from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_LEFT_ARM_JOINTS
from dimos.teleop.yam_leader.mapping import (
    YAM_FOLLOWER_JOINT_LIMITS,
    YAM_LEADER_MOTORS,
    combine_side_commands,
    low_pass_alpha,
    map_side_readings,
    smooth_positions,
    streamed_trajectory,
    yam_follower_joint_names,
)

_LEFT = list(yam_follower_joint_names("left"))
_RIGHT = list(yam_follower_joint_names("right"))


def _readings(**overrides: float) -> dict[str, float]:
    readings = {f"joint_{i}": 0.1 * i for i in range(1, 8)}
    readings.update(overrides)
    return readings


def test_leader_joint_n_drives_follower_joint_n_per_side() -> None:
    left = map_side_readings("left", _readings(), gripper_travel_radians=1.0)
    right = map_side_readings("right", _readings(), gripper_travel_radians=1.0)

    assert list(left.positions_by_joint) == _LEFT
    assert list(right.positions_by_joint) == _RIGHT
    assert list(left.positions_by_joint.values()) == pytest.approx([0.1 * i for i in range(1, 7)])
    assert yam_follower_joint_names("left") == tuple(
        f"arm/{j}" for j in DUAL_OPENYAM_LEFT_ARM_JOINTS
    )
    with pytest.raises(ValueError):
        yam_follower_joint_names("centre")  # type: ignore[arg-type]


def test_positions_clamp_to_the_yam_urdf_limits() -> None:
    command = map_side_readings(
        "left", _readings(joint_1=9.0, joint_2=-1.0, joint_6=-9.0), gripper_travel_radians=1.0
    )
    positions = list(command.positions_by_joint.values())

    assert positions[0] == YAM_FOLLOWER_JOINT_LIMITS[0][1]
    assert positions[1] == YAM_FOLLOWER_JOINT_LIMITS[1][0]
    assert positions[5] == YAM_FOLLOWER_JOINT_LIMITS[5][0]


def test_trigger_squeeze_closes_the_gripper() -> None:
    released = map_side_readings("left", _readings(joint_7=0.0), gripper_travel_radians=1.0)
    half = map_side_readings("left", _readings(joint_7=0.5), gripper_travel_radians=1.0)
    full = map_side_readings("left", _readings(joint_7=1.4), gripper_travel_radians=1.0)
    mirrored = map_side_readings("left", _readings(joint_7=-0.5), gripper_travel_radians=1.0)

    assert released.gripper_opening == 1.0
    assert half.gripper_opening == pytest.approx(0.5)
    assert full.gripper_opening == 0.0
    assert mirrored.gripper_opening == pytest.approx(0.5)
    with pytest.raises(ValueError):
        map_side_readings("left", _readings(), gripper_travel_radians=0.0)


def test_missing_motor_and_joint_jump_are_rejected() -> None:
    readings = _readings()
    del readings["joint_7"]
    with pytest.raises(ValueError, match="missing motors"):
        map_side_readings("left", readings, gripper_travel_radians=1.0)

    first = map_side_readings("left", _readings(), gripper_travel_radians=1.0)
    with pytest.raises(ValueError, match="jump"):
        map_side_readings(
            "left",
            _readings(joint_3=2.5),
            gripper_travel_radians=1.0,
            previous_positions_by_joint=first.positions_by_joint,
            max_joint_jump_radians=0.75,
        )
    assert len(YAM_LEADER_MOTORS) == 7


def test_combine_keeps_left_then_right_order() -> None:
    left = map_side_readings("left", _readings(), gripper_travel_radians=1.0)
    right = map_side_readings("right", _readings(joint_1=0.9), gripper_travel_radians=1.0)
    joint_state = combine_side_commands([left, right])

    assert list(joint_state.name) == [*_LEFT, *_RIGHT]
    assert joint_state.position[0] == pytest.approx(0.1)
    assert joint_state.position[6] == pytest.approx(0.9)


def test_low_pass_alpha_rises_with_cutoff_and_period() -> None:
    slow = low_pass_alpha(2.0, 0.02)
    fast = low_pass_alpha(6.0, 0.02)
    assert 0.0 < slow < fast < 1.0
    assert low_pass_alpha(6.0, 0.04) > fast
    assert low_pass_alpha(6.0, 0.02) == pytest.approx(0.02 / (1.0 / (2.0 * math.pi * 6.0) + 0.02))
    with pytest.raises(ValueError):
        low_pass_alpha(0.0, 0.02)
    with pytest.raises(ValueError):
        low_pass_alpha(6.0, 0.0)


def test_smooth_positions_passes_the_first_sample_then_blends() -> None:
    first = smooth_positions(None, {"a": 1.0, "b": 2.0}, 0.5)
    assert first == {"a": 1.0, "b": 2.0}
    blended = smooth_positions(first, {"a": 3.0, "b": 2.0, "c": 9.0}, 0.5)
    assert blended == {"a": 2.0, "b": 2.0, "c": 9.0}
    assert smooth_positions(first, {"a": 3.0}, 1.0) == {"a": 3.0}
    with pytest.raises(ValueError):
        smooth_positions(first, {"a": 3.0}, 0.0)


def test_streamed_trajectory_is_one_point_until_a_target_was_sent() -> None:
    target = {"left_joint1": 0.2, "right_joint1": -0.1}
    first = streamed_trajectory(None, target, 0.04)
    assert first.joint_names == ["left_joint1", "right_joint1"]
    assert len(first.points) == 1 and first.points[0].positions == [0.2, -0.1]
    assert first.points[0].time_from_start == 0.0

    partial = streamed_trajectory({"left_joint1": 0.0}, target, 0.04)
    assert len(partial.points) == 1

    second = streamed_trajectory({"left_joint1": 0.0, "right_joint1": 0.0}, target, 0.04)
    assert [point.time_from_start for point in second.points] == [0.0, 0.04]
    assert second.points[0].positions == [0.0, 0.0]
    assert second.points[1].positions == [0.2, -0.1]
    assert second.points[1].velocities == [0.0, 0.0]
    with pytest.raises(ValueError):
        streamed_trajectory(None, target, 0.0)
