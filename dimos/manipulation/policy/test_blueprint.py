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

"""Tests for the policy-module blueprint helper.

The teleop-preempts invariant: any teleop task that shares joints with the
policy node MUST have a strictly higher arbitration priority than the
policy's streaming servo task. The helper either auto-derives a safe
priority or refuses to build with an explicit error.
"""

from __future__ import annotations

import pytest

from dimos.control.coordinator import TaskConfig
from dimos.manipulation.policy import policy_engage_buttons, policy_servo_task_config


def _teleop(
    name: str,
    joints: list[str],
    priority: int = 50,
    hand: str | None = None,
) -> TaskConfig:
    return TaskConfig(
        name=name,
        type="single_arm_pink_ik",
        joint_names=joints,
        priority=priority,
        hand=hand,  # type: ignore[arg-type]
    )


def test_auto_derives_priority_below_overlapping_teleop():
    teleop = _teleop("teleop_arm", joints=["arm/j1", "arm/j2", "arm/gripper"], priority=50)

    cfg = policy_servo_task_config(
        name="policy_servo",
        joint_names=["arm/j1", "arm/j2", "arm/gripper"],
        teleop_tasks=[teleop],
    )
    assert cfg.type == "servo"
    assert cfg.joint_names == ["arm/j1", "arm/j2", "arm/gripper"]
    assert cfg.priority == 49


def test_auto_priority_takes_min_of_overlapping_teleop_priorities():
    t1 = _teleop("teleop_a", joints=["arm/j1"], priority=70)
    t2 = _teleop("teleop_b", joints=["arm/j1"], priority=40)
    cfg = policy_servo_task_config(
        name="policy",
        joint_names=["arm/j1"],
        teleop_tasks=[t1, t2],
    )
    assert cfg.priority == 39  # one below the lowest teleop priority


def test_no_overlapping_teleop_yields_default_priority_zero():
    t = _teleop("teleop_other_arm", joints=["other/j1"], priority=50)
    cfg = policy_servo_task_config(
        name="policy",
        joint_names=["arm/j1"],
        teleop_tasks=[t],
    )
    assert cfg.priority == 0


def test_explicit_priority_below_teleop_is_accepted():
    t = _teleop("teleop_arm", joints=["arm/j1"], priority=50)
    cfg = policy_servo_task_config(
        name="policy",
        joint_names=["arm/j1"],
        teleop_tasks=[t],
        priority=10,
    )
    assert cfg.priority == 10


def test_explicit_priority_equal_to_teleop_raises():
    t = _teleop("teleop_arm", joints=["arm/j1"], priority=50)
    with pytest.raises(ValueError) as excinfo:
        policy_servo_task_config(
            name="policy",
            joint_names=["arm/j1"],
            teleop_tasks=[t],
            priority=50,
        )
    msg = str(excinfo.value)
    assert "teleop_arm" in msg
    assert "50" in msg


def test_explicit_priority_above_teleop_raises():
    t = _teleop("teleop_arm", joints=["arm/j1"], priority=50)
    with pytest.raises(ValueError) as excinfo:
        policy_servo_task_config(
            name="policy",
            joint_names=["arm/j1"],
            teleop_tasks=[t],
            priority=60,
        )
    assert "teleop_arm" in str(excinfo.value)


def test_non_overlapping_teleop_is_ignored_for_invariant():
    # Same priority as policy is OK if joints don't overlap.
    t = _teleop("teleop_other", joints=["other/j1"], priority=10)
    cfg = policy_servo_task_config(
        name="policy",
        joint_names=["arm/j1"],
        teleop_tasks=[t],
        priority=10,
    )
    assert cfg.priority == 10


def test_non_teleop_task_does_not_constrain_priority():
    # Trajectory tasks aren't button-triggered, so they don't participate
    # in the teleop-preempt invariant even when joints overlap.
    traj = TaskConfig(name="traj_arm", type="trajectory", joint_names=["arm/j1"], priority=10)
    cfg = policy_servo_task_config(
        name="policy",
        joint_names=["arm/j1"],
        teleop_tasks=[traj],
        priority=10,
    )
    assert cfg.priority == 10


def test_empty_joint_names_rejected():
    with pytest.raises(ValueError, match="joint_names"):
        policy_servo_task_config(name="policy", joint_names=[])


# ── policy_engage_buttons ────────────────────────────────────────────────


def test_engage_buttons_right_hand_overlap_only():
    right = _teleop("teleop_right", joints=["arm/j1"], hand="right")
    left = _teleop("teleop_left", joints=["other/j1"], hand="left")
    assert policy_engage_buttons(["arm/j1"], [right, left]) == ["right_primary"]


def test_engage_buttons_both_hands_overlap():
    right = _teleop("teleop_right", joints=["right_arm/j1"], hand="right")
    left = _teleop("teleop_left", joints=["left_arm/j1"], hand="left")
    assert policy_engage_buttons(["right_arm/j1", "left_arm/j1"], [right, left]) == [
        "left_primary",
        "right_primary",
    ]


def test_engage_buttons_no_overlap_returns_empty():
    t = _teleop("teleop_other", joints=["other/j1"], hand="right")
    assert policy_engage_buttons(["arm/j1"], [t]) == []


def test_engage_buttons_skips_tasks_with_no_hand():
    # An overlapping teleop task with hand=None contributes no button.
    t_no_hand = _teleop("teleop_no_hand", joints=["arm/j1"], hand=None)
    assert policy_engage_buttons(["arm/j1"], [t_no_hand]) == []


def test_engage_buttons_dedupes_repeated_hand():
    t1 = _teleop("teleop_right_a", joints=["arm/j1"], hand="right")
    t2 = _teleop("teleop_right_b", joints=["arm/j2"], hand="right")
    assert policy_engage_buttons(["arm/j1", "arm/j2"], [t1, t2]) == ["right_primary"]


def test_engage_buttons_ignores_non_teleop_task_types():
    # Trajectory tasks aren't button-triggered, so they don't contribute
    # even with overlapping joints and a configured hand.
    traj = TaskConfig(
        name="traj",
        type="trajectory",
        joint_names=["arm/j1"],
        priority=10,
        hand="right",  # type: ignore[arg-type]
    )
    assert policy_engage_buttons(["arm/j1"], [traj]) == []
