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

"""Shared control-task stubs for tests.

Not named ``test_*``/``*_test`` so pytest does not collect it; both
``test_control.py`` and ``test_coordinator_routing.py`` import ``RecordingTask``
from here rather than reaching into each other's test modules.

``turning_joints`` writes out a test robot's ``ControlDescription`` by hand,
for tests that have no robot model.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from dimos.control.contract.description import (
    ControlDescription,
    Limits,
    Resource,
    ResourceKind,
)
from dimos.control.contract.keys import Interface, Unit
from dimos.control.task import (
    BaseControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped


class RecordingTask(BaseControlTask):
    """Stub task that records every stream handler invocation."""

    def __init__(self, name: str, joints: frozenset[str] = frozenset()) -> None:
        self._name = name
        self._joints = frozenset(joints)
        self.cartesian_calls: list[tuple[Any, float]] = []
        self.left_cartesian_calls: list[tuple[Pose | PoseStamped, float]] = []
        self.right_cartesian_calls: list[tuple[Pose | PoseStamped, float]] = []
        self.ee_twist_calls: list[tuple[Any, float]] = []
        self.joint_command_calls: list[tuple[Any, float]] = []
        self.buttons_calls: list[Any] = []
        self.gripper_calls: list[tuple[Any, float]] = []

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=self._joints)

    def is_active(self) -> bool:
        return False

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return None

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass

    def on_cartesian_command(self, pose: Any, t_now: float) -> bool:
        self.cartesian_calls.append((pose, t_now))
        return True

    def on_left_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        self.left_cartesian_calls.append((pose, t_now))
        return True

    def on_right_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        self.right_cartesian_calls.append((pose, t_now))
        return True

    def on_ee_twist_command(self, twist: Any, t_now: float) -> bool:
        self.ee_twist_calls.append((twist, t_now))
        return True

    def on_joint_command(self, msg: Any, t_now: float) -> bool:
        self.joint_command_calls.append((msg, t_now))
        return True

    def on_gripper_command(self, msg: Any, t_now: float) -> bool:
        self.gripper_calls.append((msg, t_now))
        return True

    def on_buttons(self, msg: Any) -> bool:
        self.buttons_calls.append(msg)
        return True

    def on_teleop_buttons(self, msg: Any, t_now: float) -> bool:
        # Mirrors TeleopIKTask: the uniform handler delegates to on_buttons.
        return self.on_buttons(msg)


#: The unit of each joint interface, for a joint that turns.
_TURNING: Mapping[str, Unit] = {
    Interface.POSITION: Unit.RAD,
    Interface.VELOCITY: Unit.RAD_PER_S,
    Interface.EFFORT: Unit.NM,
    Interface.KP: Unit.UNITLESS,
    Interface.KD: Unit.UNITLESS,
}


def turning_joints(
    source: str,
    joints: Sequence[str],
    *,
    state: Sequence[Interface],
    command: Sequence[Interface],
    limits: Mapping[str, Limits] | None = None,
    others: Sequence[Resource] = (),
    state_rate_hz: float = 100.0,
    deadman_timeout_s: float = 0.1,
) -> ControlDescription:
    """Describe a robot of turning joints by hand, without a robot model.

    Args:
        source: The robot's name, the first part of every key, e.g. "mock".
        joints: The joint names, in order.
        state: What each joint reports.
        command: What each joint can be told.
        limits: Limits by full name, such as "mock/joint1/position". A
            command with no limit is unlimited.
        others: Parts to add after the joints, such as an IMU.
        state_rate_hz: How many times a second it reports.
        deadman_timeout_s: How long it waits for a command, in seconds,
            before halting.
    """
    resources = tuple(
        Resource(
            name=joint,
            kind=ResourceKind.JOINT,
            state_interfaces=tuple(state),
            command_interfaces=tuple(command),
            units={interface: _TURNING[interface] for interface in (*state, *command)},
        )
        for joint in joints
    )
    return ControlDescription(
        source=source,
        resources=(*resources, *others),
        limits=dict(limits or {}),
        state_rate_hz=state_rate_hz,
        deadman_timeout_s=deadman_timeout_s,
    )
