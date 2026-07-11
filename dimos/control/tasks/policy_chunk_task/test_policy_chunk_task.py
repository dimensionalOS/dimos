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

import threading

from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.policy_chunk_task.policy_chunk_task import (
    PolicyChunkControlTask,
    PolicyChunkControlTaskConfig,
)
from dimos.robot_learning.policy_rollout.models import RobotPolicyActionChunk


def _state(pos: dict[str, float] | None = None) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions=pos or {"arm/j0": 1.0, "arm/j1": 2.0}),
        t_now=1.0,
        dt=0.01,
    )


def _chunk(values: tuple[tuple[float, ...], ...], space: str = "space") -> RobotPolicyActionChunk:
    return RobotPolicyActionChunk(space_id=space, values=values)


def _task(**kwargs: object) -> PolicyChunkControlTask:
    config_kwargs = {
        "joint_names": ["arm/j0", "arm/j1"],
        "accepted_action_space_id": "space",
        "action_scale": 0.1,
        **kwargs,
    }
    return PolicyChunkControlTask("policy", PolicyChunkControlTaskConfig(**config_kwargs))


def test_accepts_and_executes_prefix_with_ticks_per_action() -> None:
    task = _task(ticks_per_action=2, execute_first_n=2)
    assert task.on_robot_policy_action_chunk(_chunk(((0.5, -0.5), (1.0, 0.0), (-1.0, 1.0))), 1.0)

    first = task.compute(_state())
    second = task.compute(_state())
    third = task.compute(_state())

    assert first is not None
    assert first.joint_names == ["arm/j0", "arm/j1"]
    assert first.positions == [1.05, 1.95]
    assert second is not None
    assert second.positions == [1.05, 1.95]
    assert third is not None
    assert third.positions == [1.1, 2.0]


def test_validation_rejects_wrong_space_and_bad_values() -> None:
    task = _task()
    assert not task.on_robot_policy_action_chunk(_chunk(((0.0, 0.0),), space="other"), 1.0)
    assert not task.on_robot_policy_action_chunk(_chunk(((2.0, 0.0),)), 1.0)
    assert not task.on_robot_policy_action_chunk(_chunk(((float("nan"), 0.0),)), 1.0)
    assert not task.is_active()


def test_validation_rejects_unsupported_action_dim() -> None:
    task = _task(gripper_joint_name="arm/gripper", gripper_action_index=2)
    assert not task.on_robot_policy_action_chunk(_chunk(((0.0, 0.0),)), 1.0)


def test_refill_trigger_called_once_when_queue_empty() -> None:
    calls = 0

    def trigger() -> None:
        nonlocal calls
        calls += 1

    task = _task()
    task.set_policy_trigger(trigger)
    assert task.compute(_state()) is None
    assert task.compute(_state()) is None
    assert calls == 1


def test_refill_trigger_records_status_counts() -> None:
    class Status:
        accepted = False
        status = "not_ready"

    task = _task()
    task.set_policy_trigger(Status)

    assert task.compute(_state()) is None

    diagnostics = task.diagnostics()
    assert diagnostics["refill_triggers"] == 0
    assert diagnostics["inference_status_counts"] == {"not_ready": 1}


def test_stale_deactivation_returns_no_command() -> None:
    task = _task(stale_timeout_ticks=0)
    assert task.on_robot_policy_action_chunk(_chunk(((0.0, 0.0), (0.0, 0.0))), 1.0)
    assert task.compute(_state()) is not None
    assert task.compute(_state()) is None
    assert not task.is_active()


def test_gripper_mapping_appends_configured_joint() -> None:
    task = _task(
        gripper_joint_name="arm/gripper",
        gripper_action_index=2,
        gripper_min=0.02,
        gripper_max=0.08,
    )
    assert task.on_robot_policy_action_chunk(_chunk(((0.0, 0.0, 1.0),)), 1.0)
    output = task.compute(_state())
    assert output is not None
    assert output.joint_names == ["arm/j0", "arm/j1", "arm/gripper"]
    assert output.positions == [1.0, 2.0, 0.08]


def test_coordinator_routes_policy_chunks_to_opt_in_tasks() -> None:
    coordinator = object.__new__(ControlCoordinator)
    task = _task(action_mapping="target")
    coordinator._task_lock = threading.Lock()
    coordinator._tasks = {task.name: task}

    coordinator._on_robot_policy_action_chunk(_chunk(((0.3, -0.3),)))

    output = task.compute(_state())
    assert output is not None
    assert output.positions == [0.03, -0.03]


def test_claim_priority_supports_arbitration() -> None:
    task = _task(priority=42)
    claim = task.claim()
    assert claim.priority == 42
    assert claim.joints == frozenset({"arm/j0", "arm/j1"})
