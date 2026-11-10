# Copyright 2025-2026 Dimensional Inc.
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

from dimos.teleop.leader.mapping import (
    GripperMapping,
    JointMapping,
    SideMapping,
    combine_side_commands,
    low_pass_alpha,
    map_side_readings,
    smooth_positions,
)

ARM = SideMapping(
    joints=(
        JointMapping(motor="joint_1", joint="arm/left_joint1", lower=-2.6, upper=3.1),
        JointMapping(motor="joint_2", joint="arm/left_joint2", lower=0.0, upper=3.6, scale=-1.0),
        JointMapping(motor="joint_3", joint="arm/left_joint3", lower=0.0, upper=3.1, offset=0.5),
    ),
    gripper=GripperMapping(motor="joint_7", travel_radians=1.0),
)


def test_targets_apply_scale_offset_and_clamp() -> None:
    command = map_side_readings(
        "left", ARM, {"joint_1": 0.4, "joint_2": -1.0, "joint_3": 4.0, "joint_7": 0.0}
    )
    assert command.positions_by_joint == {
        "arm/left_joint1": 0.4,
        "arm/left_joint2": 1.0,
        "arm/left_joint3": 3.1,
    }
    assert command.gripper_opening == 1.0


def test_gripper_opening_follows_the_trigger_through_its_travel() -> None:
    readings = {"joint_1": 0.0, "joint_2": 0.0, "joint_3": 0.0, "joint_7": 0.25}
    assert map_side_readings("left", ARM, readings).gripper_opening == pytest.approx(0.75)
    readings["joint_7"] = -2.0
    assert map_side_readings("left", ARM, readings).gripper_opening == 0.0


def test_a_side_without_a_gripper_mapping_reports_no_opening() -> None:
    arm_only = SideMapping(joints=ARM.joints)
    command = map_side_readings("right", arm_only, {"joint_1": 0, "joint_2": 0, "joint_3": 0})
    assert command.gripper_opening is None
    assert arm_only.motor_names() == ("joint_1", "joint_2", "joint_3")


def test_missing_motor_readings_are_rejected() -> None:
    with pytest.raises(ValueError, match="missing motors: \\['joint_3', 'joint_7'\\]"):
        map_side_readings("left", ARM, {"joint_1": 0.0, "joint_2": 0.0})


def test_jump_guard_rejects_a_discontinuity_after_clamping() -> None:
    readings = {"joint_1": 0.0, "joint_2": 0.0, "joint_3": 4.0, "joint_7": 0.0}
    previous = map_side_readings("left", ARM, readings).positions_by_joint
    readings["joint_1"] = 1.0
    with pytest.raises(ValueError, match="arm/left_joint1 jump 1.000 rad exceeds 0.750"):
        map_side_readings(
            "left", ARM, readings, previous_positions_by_joint=previous, max_joint_jump_radians=0.75
        )
    # Clamping happens first: a reading far past the limit is not a jump.
    readings["joint_1"] = 0.0
    readings["joint_3"] = 40.0
    command = map_side_readings(
        "left", ARM, readings, previous_positions_by_joint=previous, max_joint_jump_radians=0.75
    )
    assert command.positions_by_joint["arm/left_joint3"] == 3.1


def test_combined_command_keeps_side_order_and_frame() -> None:
    left = map_side_readings(
        "left", ARM, {"joint_1": 0.1, "joint_2": 0, "joint_3": 0, "joint_7": 0}
    )
    right_map = SideMapping(
        joints=(JointMapping(motor="joint_1", joint="arm/right_joint1", lower=-3.1, upper=2.6),)
    )
    right = map_side_readings("right", right_map, {"joint_1": -0.2})
    state = combine_side_commands([left, right], frame_id="dimos/guarded-joint")
    assert state.name == [
        "arm/left_joint1",
        "arm/left_joint2",
        "arm/left_joint3",
        "arm/right_joint1",
    ]
    assert state.position == pytest.approx([0.1, 0.0, 0.5, -0.2])
    assert state.frame_id == "dimos/guarded-joint"


def test_mapping_rejects_a_motor_or_joint_used_twice() -> None:
    with pytest.raises(ValueError, match="mapped twice"):
        SideMapping(
            joints=(
                JointMapping(motor="joint_1", joint="a", lower=0, upper=1),
                JointMapping(motor="joint_1", joint="b", lower=0, upper=1),
            )
        )
    with pytest.raises(ValueError, match="mapped twice"):
        SideMapping(
            joints=(JointMapping(motor="joint_1", joint="a", lower=0, upper=1),),
            gripper=GripperMapping(motor="joint_1", travel_radians=1.0),
        )


def test_mapping_rejects_inverted_limits_and_zero_scale() -> None:
    with pytest.raises(ValueError, match="lower 1.0 must be below upper 0.0"):
        JointMapping(motor="joint_1", joint="a", lower=1.0, upper=0.0)
    with pytest.raises(ValueError, match="scale must not be zero"):
        JointMapping(motor="joint_1", joint="a", lower=0.0, upper=1.0, scale=0.0)


def test_low_pass_alpha_and_smoothing() -> None:
    alpha = low_pass_alpha(cutoff_hz=1.0 / (2.0 * math.pi), period_s=1.0)
    assert alpha == pytest.approx(0.5)
    smoothed = smooth_positions({"a": 0.0}, {"a": 1.0, "b": 2.0}, alpha)
    assert smoothed == {"a": 0.5, "b": 2.0}
    assert smooth_positions(None, {"a": 1.0}, alpha) == {"a": 1.0}
    with pytest.raises(ValueError, match="cutoff_hz must be positive"):
        low_pass_alpha(0.0, 0.02)
    with pytest.raises(ValueError, match="alpha must be in"):
        smooth_positions(None, {"a": 1.0}, 1.5)
