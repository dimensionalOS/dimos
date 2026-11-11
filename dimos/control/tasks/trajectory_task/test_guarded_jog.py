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

import pytest

from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
)
from dimos.message_utils import joint_state as joint_state_utils


@pytest.fixture
def task() -> JointTrajectoryTask:
    return JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=["arm/a", "arm/b"]))


def state(now: float) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"arm/a": 0.1, "arm/b": 0.2}),
        t_now=now,
        dt=0.01,
    )


@pytest.mark.parametrize("explicit_stop", [False, True])
def test_guarded_jog_holds_measured_position_on_expiry_or_stop(
    task: JointTrajectoryTask, explicit_stop: bool
) -> None:
    assert task.on_joint_command(
        joint_state_utils.from_mapping(
            {"frame_id": "dimos/guarded-joint", "name": ["arm/a"], "position": [1.0]}
        ),
        1.0,
    )
    moving = task.compute(state(1.1))
    assert moving is not None and moving.positions[0] > 0.1
    if explicit_stop:
        assert task.on_joint_command(
            joint_state_utils.from_mapping(
                {"frame_id": "dimos/guarded-joint", "name": ["arm/a"], "velocity": [0.0]}
            ),
            1.2,
        )
    stopped = task.compute(state(1.2 if explicit_stop else 1.4))
    assert stopped is not None
    assert stopped.joint_names == ["arm/a"]
    assert stopped.positions == [0.1]
    assert not task.is_active()
    assert task.compute(state(2.0)) is None


def test_guarded_stop_does_not_cancel_an_ordinary_position_goal(task: JointTrajectoryTask) -> None:
    task.on_joint_command(
        joint_state_utils.from_mapping(
            {"frame_id": "dimos/guarded-joint", "name": ["arm/a"], "position": [1.0]}
        ),
        1.0,
    )
    task.on_joint_command(
        joint_state_utils.from_mapping({"name": ["arm/a"], "position": [2.0]}), 1.1
    )
    task.on_joint_command(
        joint_state_utils.from_mapping(
            {"frame_id": "dimos/guarded-joint", "name": ["arm/a"], "velocity": [0.0]}
        ),
        1.2,
    )
    output = task.compute(state(2.0))
    assert output is not None and output.positions[0] > 0.1
    assert task.is_active()


def test_new_guarded_input_gets_a_new_deadline(task: JointTrajectoryTask) -> None:
    for now in [1.0, 1.3]:
        assert task.on_joint_command(
            joint_state_utils.from_mapping(
                {"frame_id": "dimos/guarded-joint", "name": ["arm/a"], "position": [1.0]}
            ),
            now,
        )
    assert task.compute(state(1.5)).positions[0] > 0.1
    assert task.compute(state(1.8)).positions == [0.1]
