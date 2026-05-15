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

"""Unit tests for the `RobotContract` Protocol and `GripperBinarization`."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np

from dimos.manipulation.policy.contract import (
    GripperBinarization,
    LeRobotFrame,
    RobotContract,
)
from dimos.msgs.sensor_msgs.JointState import JointState


def test_gripper_binarization_defaults():
    cfg = GripperBinarization(open_pos=0.85, closed_pos=0.0)
    assert cfg.enabled is True
    assert cfg.threshold == 0.7
    assert cfg.open_pos == 0.85
    assert cfg.closed_pos == 0.0


def test_gripper_binarization_custom_threshold():
    cfg = GripperBinarization(open_pos=1.0, closed_pos=0.0, threshold=0.42, enabled=False)
    assert cfg.threshold == 0.42
    assert cfg.enabled is False


def test_robot_contract_is_runtime_checkable():
    assert isinstance(RobotContract, type)
    # The Protocol must allow isinstance checks (i.e. @runtime_checkable).
    # Constructing a synthetic instance and probing isinstance exercises that.
    inst = _StructuralContract()
    assert isinstance(inst, RobotContract)


def test_structural_contract_without_inheritance_satisfies_protocol():
    # A concrete class that defines all the required attributes/methods
    # (without inheriting from RobotContract) must satisfy the runtime check.
    inst = _StructuralContract()
    assert isinstance(inst, RobotContract)


def test_lerobot_frame_typed_dict_importable():
    # Construction is the simplest way to assert TypedDict is well-formed.
    frame: LeRobotFrame = {"task": "hello"}
    assert frame["task"] == "hello"


class _StructuralContract:
    """A bare-minimum implementation that does NOT inherit RobotContract."""

    cameras: Mapping[str, tuple[int, int, int]] = {"cam": (4, 4, 3)}
    state_joint_names: Sequence[str] = ("a", "b")
    action_joint_names: Sequence[str] = ("a", "b")
    gripper_joint: str | None = None
    gripper_binarization = GripperBinarization(open_pos=1.0, closed_pos=0.0, enabled=False)

    def features(self) -> dict[str, dict[str, Any]]:
        return {}

    def rerun_entities(self) -> Sequence[str]:
        return ()

    def from_rerun_row(self, row: Any) -> dict[str, Any]:
        return {}

    def from_messages(
        self,
        images: Mapping[str, Any],
        state: JointState,
        *,
        action: JointState | None = None,
        task: str = "",
    ) -> dict[str, Any]:
        return {}

    def to_command(self, action_vec: np.ndarray) -> JointState:
        return JointState(name=[], position=[])
