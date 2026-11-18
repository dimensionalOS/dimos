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

"""The pass-through task: the latest sample as it came, the measured positions once stale."""

from __future__ import annotations

import math

import pytest

from dimos.control._control_test_helpers import fake_arm_model
from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection_source import ConnectionSource
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import Limits
from dimos.control.coordinator import TaskConfig
from dimos.control.routing import Routing
from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.passthrough_task.passthrough_task import (
    PASSTHROUGH_PRIORITY,
    PassthroughStatus,
    PassthroughTask,
)
from dimos.control.tasks.registry import control_task_registry
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.testing.cbor import roundtrip

JOINTS = ["arm/joint1", "arm/joint2"]
LIMITS = {joint: Limits(lo=-1.0, hi=1.0) for joint in JOINTS}


def _task(stale_after_s: float = 0.1) -> PassthroughTask:
    return PassthroughTask("passthrough", JOINTS, LIMITS, stale_after_s=stale_after_s)


def _state(t_now: float, **positions: float) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(
            joint_positions={f"arm/{joint}": value for joint, value in positions.items()}
        ),
        t_now=t_now,
    )


def _sample(**positions: float) -> JointState:
    return JointState(
        name=[f"arm/{joint}" for joint in positions], position=list(positions.values())
    )


def _positions(task: PassthroughTask, state: CoordinatorState) -> dict[str, float] | None:
    output = task.compute(state)
    if output is None:
        return None
    assert output.mode is ControlMode.POSITION and output.positions is not None
    return dict(zip(output.joint_names, output.positions, strict=True))


def test_a_fresh_sample_is_output_as_it_came_a_step_included() -> None:
    task = _task()
    assert _positions(task, _state(0.0, joint1=0.0, joint2=0.0)) is None and task.stale

    assert task.on_joint_command(_sample(joint1=0.1, joint2=0.2), t_now=1.0)
    assert _positions(task, _state(1.01, joint1=0.0, joint2=0.0)) == {
        "arm/joint1": 0.1,
        "arm/joint2": 0.2,
    }
    assert not task.stale
    # A 30 deg step goes out whole on the next tick: nothing limits or smooths it.
    step = 0.1 + math.radians(30.0)
    assert task.on_joint_command(_sample(joint1=step, joint2=0.2), t_now=1.02)
    assert _positions(task, _state(1.03, joint1=0.1, joint2=0.2)) == {
        "arm/joint1": step,
        "arm/joint2": 0.2,
    }


def test_stale_targets_latch_the_measured_position_once_until_the_next_sample() -> None:
    task = _task()
    task.on_joint_command(_sample(joint1=0.5, joint2=0.2), t_now=1.0)
    assert _positions(task, _state(1.09, joint1=0.4, joint2=0.2)) == {
        "arm/joint1": 0.5,
        "arm/joint2": 0.2,
    }
    assert not task.stale

    # Past 0.1 s it latches where the arm is, once: a sagging arm is not followed.
    assert _positions(task, _state(1.11, joint1=0.3, joint2=0.25)) == {
        "arm/joint1": 0.3,
        "arm/joint2": 0.25,
    }
    assert task.stale
    assert _positions(task, _state(1.5, joint1=0.1, joint2=0.0)) == {
        "arm/joint1": 0.3,
        "arm/joint2": 0.25,
    }
    assert task.get_status().stale

    task.on_joint_command(_sample(joint1=0.6), t_now=2.0)
    assert _positions(task, _state(2.0, joint1=0.1, joint2=0.0)) == {"arm/joint1": 0.6}
    assert not task.stale


def test_hold_latches_at_once_and_the_next_sample_goes_on() -> None:
    task = _task()
    task.on_joint_command(_sample(joint1=0.5, joint2=0.2), t_now=1.0)
    _positions(task, _state(1.0, joint1=0.45, joint2=0.2))

    status = task.hold()

    assert status == PassthroughStatus(stale=True, joints=("arm/joint1", "arm/joint2"))
    assert task.stale
    assert _positions(task, _state(1.01, joint1=0.3, joint2=0.1)) == {
        "arm/joint1": 0.45,
        "arm/joint2": 0.2,
    }
    task.on_joint_command(_sample(joint1=0.5, joint2=0.2), t_now=1.02)
    assert _positions(task, _state(1.03, joint1=0.3, joint2=0.1)) == {
        "arm/joint1": 0.5,
        "arm/joint2": 0.2,
    }
    assert not task.stale
    assert roundtrip(status) == status


@pytest.mark.parametrize(
    "sample",
    [
        _sample(joint1=0.9, joint2=1.5),
        _sample(joint1=float("nan"), joint2=0.0),
        JointState(name=["arm/joint1", "arm/joint2"], position=[0.1]),
        JointState(name=["arm/joint1"], velocity=[0.0]),
    ],
    ids=["past-a-limit", "not-finite", "unpaired", "no-positions"],
)
def test_a_bad_sample_is_refused_whole(sample: JointState) -> None:
    task = _task()
    task.on_joint_command(_sample(joint1=0.1, joint2=0.2), t_now=1.0)

    assert not task.on_joint_command(sample, t_now=1.01)

    assert task.get_status().refused == 1
    assert _positions(task, _state(1.02, joint1=0.0, joint2=0.0)) == {
        "arm/joint1": 0.1,
        "arm/joint2": 0.2,
    }


def test_only_its_own_joints_are_taken_and_only_those_sent_are_output() -> None:
    task = _task()
    assert not task.on_joint_command(_sample(joint3=0.1), t_now=1.0)
    assert task.on_joint_command(
        JointState(name=["arm/joint2", "arm/left_finger"], position=[0.3, 9.0]), t_now=1.0
    )
    assert _positions(task, _state(1.0, joint1=0.0, joint2=0.0)) == {"arm/joint2": 0.3}
    assert task.claim().joints == frozenset(JOINTS)
    assert task.claim().priority == PASSTHROUGH_PRIORITY


def test_an_estop_latches_and_refuses_samples_until_it_is_cleared() -> None:
    task = _task()
    task.on_joint_command(_sample(joint1=0.5, joint2=0.2), t_now=1.0)
    _positions(task, _state(1.0, joint1=0.4, joint2=0.2))

    task.set_estop(True)

    assert task.stale and task.get_status().estopped
    assert not task.on_joint_command(_sample(joint1=0.9, joint2=0.2), t_now=1.01)
    assert _positions(task, _state(1.02, joint1=0.3, joint2=0.2)) == {
        "arm/joint1": 0.4,
        "arm/joint2": 0.2,
    }
    task.set_estop(False)
    assert task.stale
    assert task.on_joint_command(_sample(joint1=0.6, joint2=0.2), t_now=1.03)
    assert _positions(task, _state(1.03, joint1=0.3, joint2=0.2)) == {
        "arm/joint1": 0.6,
        "arm/joint2": 0.2,
    }


def test_the_registry_makes_it_with_each_joints_limit_from_its_connection() -> None:
    model = fake_arm_model()
    source = ConnectionSource(
        ConnectionConfig(
            source="arm",
            model=model,
            joint_commands={"joint1": (Interface.POSITION,), "joint2": (Interface.POSITION,)},
        ),
        ["position_command"],
        ["joint_state"],
        0.2,
    )
    cfg = TaskConfig(
        name="calibration", type="passthrough", joint_names=JOINTS, params={"stale_after_s": 0.2}
    )

    task = control_task_registry.create("passthrough", cfg, connections={"arm": source})

    assert isinstance(task, PassthroughTask)
    task.on_joint_command(_sample(joint1=3.0), t_now=0.0)
    assert task.on_joint_command(_sample(joint1=3.2), t_now=0.0) is False
    bindings = control_task_registry.bindings_for("passthrough")
    assert [(b.stream, b.handler, b.routing) for b in bindings.consumes] == [
        ("joint_command", "on_joint_command", Routing.CLAIM_OVERLAP)
    ]
    assert bindings.exposes == frozenset({"hold", "get_status"})
    with pytest.raises(ValueError, match="on no robot driven through a connection module"):
        control_task_registry.create(
            "passthrough", TaskConfig(name="x", type="passthrough", joint_names=["arm/joint9"])
        )
    with pytest.raises(ValueError, match="stale_after_s"):
        control_task_registry.create(
            "passthrough",
            TaskConfig(
                name="x", type="passthrough", joint_names=JOINTS, params={"stale_after_s": 0}
            ),
            connections={"arm": source},
        )
