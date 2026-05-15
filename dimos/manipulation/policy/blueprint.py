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

"""Blueprint helpers for wiring `PolicyModule` to a `ControlCoordinator`.

The key invariant: when a teleop task and a policy node share joints on
the same coordinator, the teleop task SHALL have a strictly higher
arbitration priority than the policy's streaming servo task. The helpers
here either auto-derive a safe priority for the policy servo task or
fail to build with an error identifying the violating teleop tasks.
"""

from __future__ import annotations

from collections.abc import Sequence

from dimos.control.coordinator import TELEOP_BUTTON_TASK_TYPES, TaskConfig

# Public alias — operators can override the wording in error messages by
# passing a custom prefix to `policy_servo_task_config(label=...)`.
_DEFAULT_LABEL = "policy servo task"


def policy_servo_task_config(
    *,
    name: str,
    joint_names: Sequence[str],
    teleop_tasks: Sequence[TaskConfig] = (),
    priority: int | None = None,
    label: str = _DEFAULT_LABEL,
) -> TaskConfig:
    """Build a `TaskConfig` for the `JointServoTask` paired with a `PolicyModule`.

    Behavior:

    - When `priority` is `None`, derives the highest priority strictly
      below all overlapping teleop task priorities. Returns priority 0
      when no teleop task overlaps.
    - When `priority` is provided, validates it is strictly less than the
      priority of every overlapping teleop task. Raises `ValueError` with
      the offending task name(s) on violation.

    Only teleop-style tasks (those in
    `dimos.control.coordinator.TELEOP_BUTTON_TASK_TYPES`) participate in
    the invariant; non-teleop tasks are ignored because they do not arm
    via the human-in-the-loop button frames.
    """
    if not joint_names:
        raise ValueError("policy_servo_task_config: joint_names must be non-empty")

    overlapping_teleop = _overlapping_teleop_tasks(joint_names, teleop_tasks)

    if priority is None:
        if not overlapping_teleop:
            chosen = 0
        else:
            chosen = min(t.priority for t in overlapping_teleop) - 1
    else:
        violators = [t for t in overlapping_teleop if t.priority <= priority]
        if violators:
            details = ", ".join(f"'{t.name}' (priority={t.priority})" for t in violators)
            raise ValueError(
                f"{label} '{name}': priority={priority} must be strictly less than every "
                f"overlapping teleop task's priority. Violating tasks: {details}"
            )
        chosen = priority

    return TaskConfig(
        name=name,
        type="servo",
        joint_names=list(joint_names),
        priority=chosen,
    )


def policy_engage_buttons(
    joint_names: Sequence[str],
    teleop_tasks: Sequence[TaskConfig],
) -> list[str]:
    """Derive `PolicyModule.teleop_engage_buttons` from overlapping teleop tasks.

    For each teleop task whose joints overlap `joint_names`, map its
    `hand` field (`"left"` / `"right"`) to the corresponding `_primary`
    button name. Returns a sorted, de-duplicated list. Tasks with
    `hand=None` are skipped.

    This is the canonical way to wire the policy node's preempt buttons
    in a deployment blueprint: the engage button set is then guaranteed
    to match the teleop tasks that can actually preempt the policy on
    its joints (no over-eager preempt from an unrelated arm's button).
    """
    overlapping = _overlapping_teleop_tasks(joint_names, teleop_tasks)
    hands = {t.hand for t in overlapping if t.hand is not None}
    return sorted(f"{h}_primary" for h in hands)


def _overlapping_teleop_tasks(
    joint_names: Sequence[str], teleop_tasks: Sequence[TaskConfig]
) -> list[TaskConfig]:
    policy_joints = set(joint_names)
    return [
        t
        for t in teleop_tasks
        if t.type in TELEOP_BUTTON_TASK_TYPES and (set(t.joint_names) & policy_joints)
    ]


__all__ = ["policy_engage_buttons", "policy_servo_task_config"]
