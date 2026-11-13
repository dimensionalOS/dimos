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

from __future__ import annotations

from collections.abc import Iterator
from typing import Any

import pytest

from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.leader.mapping import GripperMapping, JointMapping, SideMapping
from dimos.teleop.leader.module import (
    GUARDED_JOINT_FRAME_ID,
    LeaderTeleopModule,
    LeaderTeleopModuleConfig,
)

MAPPINGS = {
    "left": SideMapping(
        joints=(
            JointMapping(
                leader_joint="leader/left_joint_1", joint="arm/left_joint1", lower=-3, upper=3
            ),
            JointMapping(
                leader_joint="leader/left_joint_2", joint="arm/left_joint2", lower=-3, upper=3
            ),
        ),
        gripper=GripperMapping(leader_joint="leader/left_joint_7", travel_radians=1.0),
    ),
    "right": SideMapping(
        joints=(
            JointMapping(
                leader_joint="leader/right_joint_1", joint="arm/right_joint1", lower=-3, upper=3
            ),
            JointMapping(
                leader_joint="leader/right_joint_2", joint="arm/right_joint2", lower=-3, upper=0.5
            ),
        )
    ),
}


def _reading(**positions: float) -> JointState:
    return JointState(
        name=[f"leader/{name}" for name in positions], position=list(positions.values())
    )


BOTH = _reading(
    left_joint_1=0.1, left_joint_2=0.9, left_joint_7=0.5, right_joint_1=0.1, right_joint_2=0.9
)


@pytest.fixture
def make() -> Iterator[Any]:
    modules: list[LeaderTeleopModule] = []

    def build(**kwargs: Any) -> LeaderTeleopModule:
        module = LeaderTeleopModule(
            **LeaderTeleopModuleConfig(mappings=MAPPINGS, **kwargs).model_dump()
        )
        modules.append(module)
        return module

    yield build
    for module in modules:
        module.stop()


def test_a_reading_publishes_clamped_targets_and_the_mapped_gripper(make: Any, mocker: Any) -> None:
    module = make()
    joints = mocker.patch.object(module.leader_joint_command, "publish")
    left = mocker.patch.object(module.left_gripper_command, "publish")
    right = mocker.patch.object(module.right_gripper_command, "publish")

    module.on_reading(BOTH)

    command = joints.call_args.args[0]
    assert command.name == [
        "arm/left_joint1",
        "arm/left_joint2",
        "arm/right_joint1",
        "arm/right_joint2",
    ]
    assert command.position == pytest.approx([0.1, 0.9, 0.1, 0.5])
    assert command.frame_id == GUARDED_JOINT_FRAME_ID
    assert left.call_args.args[0].data == pytest.approx(0.5)
    right.assert_not_called()


def test_a_side_missing_from_the_reading_is_skipped(make: Any) -> None:
    commands = make().commands_for(_reading(right_joint_1=0.2, right_joint_2=0.3))
    assert commands is not None
    assert [command.side for command in commands] == ["right"]


def test_a_jump_drops_the_reading_and_the_next_one_recovers(make: Any) -> None:
    module = make(max_joint_jump_radians=0.5)
    assert module.commands_for(BOTH) is not None
    assert (
        module.commands_for(_reading(left_joint_1=2.0, left_joint_2=0.9, left_joint_7=0.5)) is None
    )
    assert module.commands_for(BOTH) is not None


def test_filter_moves_targets_a_fraction_toward_each_reading(make: Any) -> None:
    module = make(command_filter_cutoff_hz=6.0, reading_period_s=0.01)
    module.commands_for(_reading(right_joint_1=0.0, right_joint_2=0.0))
    commands = module.commands_for(_reading(right_joint_1=0.2, right_joint_2=0.0))
    assert commands is not None
    moved = commands[0].positions_by_joint["arm/right_joint1"]
    assert 0.0 < moved < 0.2
