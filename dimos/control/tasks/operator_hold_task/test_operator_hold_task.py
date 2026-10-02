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

"""Unit tests for the operator hold task on its own, without a coordinator."""

from __future__ import annotations

import pytest

from dimos.control.coordinator import TaskConfig
from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    OperatorHoldStatus,
    OperatorHoldTask,
    create_task,
    operator_hold_task,
)


def _state(positions: dict[str, float], t_now: float = 0.0) -> CoordinatorState:
    return CoordinatorState(joints=JointStateSnapshot(joint_positions=positions), t_now=t_now)


def test_idle_until_requested():
    task = OperatorHoldTask()

    assert not task.is_active()
    assert task.claim().joints == frozenset()
    assert task.compute(_state({"arm/joint1": 0.5})) is None
    status = task.get_status()
    assert status == OperatorHoldStatus(on=False, stamp=status.stamp)


def test_freezes_each_joint_at_first_sight_and_base_joints_at_zero():
    task = OperatorHoldTask()
    task.request("manual", "stuck")

    first = task.compute(_state({"arm/joint1": 0.5, "base/vx": 1.5, "base/wz": -0.2}))

    assert first is not None
    assert first.mode is ControlMode.SERVO_POSITION
    assert dict(zip(first.joint_names, first.positions, strict=True)) == {
        "arm/joint1": 0.5,
        "base/vx": 0.0,
        "base/wz": 0.0,
    }
    # Later readings do not move the hold; a joint seen later is frozen where it is then.
    later = task.compute(_state({"arm/joint1": 0.9, "base/vx": 2.0, "arm/joint2": -1.0}))
    assert later is not None
    assert dict(zip(later.joint_names, later.positions, strict=True)) == {
        "arm/joint1": 0.5,
        "base/vx": 0.0,
        "base/wz": 0.0,
        "arm/joint2": -1.0,
    }
    assert task.claim().joints == {"arm/joint1", "base/vx", "base/wz", "arm/joint2"}
    assert task.claim().priority == 100


def test_non_finite_readings_are_not_frozen():
    task = OperatorHoldTask()
    task.request("failsafe")

    output = task.compute(_state({"arm/joint1": float("nan"), "arm/joint2": 0.1}))

    assert output is not None
    assert output.joint_names == ["arm/joint2"]


def test_request_rejects_unknown_route_and_stays_off():
    task = OperatorHoldTask()

    with pytest.raises(ValueError):
        task.request("operator")

    assert not task.is_active()


def test_repeat_request_refreshes_reason_but_keeps_start_and_positions():
    task = OperatorHoldTask()
    first = task.request("agent", "first")
    task.compute(_state({"arm/joint1": 0.5}))

    second = task.request("failsafe", "second")
    output = task.compute(_state({"arm/joint1": 0.9}))

    assert second.route == "failsafe" and second.reason == "second"
    assert second.started_at == first.started_at
    assert output is not None and output.positions == [0.5]


def test_status_goes_out_on_first_tick_then_every_interval_then_on_acknowledge():
    published: list[OperatorHoldStatus] = []
    task = OperatorHoldTask(status_interval=0.5)
    task.set_status_publisher(published.append)
    task.request("manual", "stuck")
    assert published == []

    for t_now in (0.0, 0.2, 0.4, 0.5, 0.7, 1.0):
        task.compute(_state({"arm/joint1": 0.0}, t_now=t_now))
    task.acknowledge()
    task.compute(_state({"arm/joint1": 0.0}, t_now=5.0))

    assert [status.on for status in published] == [True, True, True, False]
    assert published[-1].route == "" and published[-1].started_at == 0.0


def test_acknowledge_clears_everything():
    task = OperatorHoldTask()
    task.request("manual")
    task.compute(_state({"arm/joint1": 0.5}))

    task.acknowledge()

    assert not task.is_active()
    assert task.claim().joints == frozenset()
    assert task.compute(_state({"arm/joint1": 0.5})) is None


def test_status_interval_must_be_positive():
    with pytest.raises(ValueError):
        OperatorHoldTask(status_interval=0.0)


def test_blueprint_helper_and_factory():
    cfg = operator_hold_task()
    assert cfg.name == OPERATOR_HOLD_TASK_NAME
    assert cfg.type == "operator_hold"
    assert cfg.priority == 100

    task = create_task(cfg, hardware={})
    assert isinstance(task, OperatorHoldTask)
    assert task.name == OPERATOR_HOLD_TASK_NAME

    with pytest.raises(ValueError, match="must be named"):
        create_task(TaskConfig(name="hold", type="operator_hold"), hardware={})
    with pytest.raises(ValueError, match="leave joint_names empty"):
        create_task(
            TaskConfig(name=OPERATOR_HOLD_TASK_NAME, type="operator_hold", joint_names=["a/b"]),
            hardware={},
        )
