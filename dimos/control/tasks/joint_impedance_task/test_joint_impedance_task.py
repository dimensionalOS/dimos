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

from dataclasses import dataclass

import pytest

from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import PD_INTERFACES
from dimos.control.coordinator import TaskConfig
from dimos.control.task import ControlMode, CoordinatorState, JointStateSnapshot
from dimos.control.tasks.joint_impedance_task.joint_impedance_task import (
    JointImpedanceTask,
    create_task,
)
from dimos.msgs.sensor_msgs.JointState import JointState

GUARDED = "dimos/guarded-joint"


@dataclass
class Robot:
    """A connection robot as the task sees it, without building a real model."""

    joint_command_interfaces: dict[str, frozenset[Interface]]


def make_task(max_torque: float, velocity_limit: float) -> JointImpedanceTask:
    config = TaskConfig(
        name="follow",
        type="joint_impedance",
        joint_names=["arm/j1"],
        params={
            "stiffness": {"arm/j1": 20.0},
            "damping": {"arm/j1": 1.5},
            "max_torque": {"arm/j1": max_torque},
            "velocity_limits": {"arm/j1": velocity_limit},
        },
    )
    robot = Robot({"arm/j1": frozenset(PD_INTERFACES)})
    return create_task(config, {"arm": robot})


def tick(t: float, measured: float) -> CoordinatorState:
    return CoordinatorState(
        joints=JointStateSnapshot(joint_positions={"arm/j1": measured}), t_now=t, dt=0.01
    )


def test_follows_the_leader_at_the_speed_limit_and_lets_go_when_it_stops():
    task = make_task(max_torque=10.0, velocity_limit=2.0)
    assert task.on_joint_command(JointState(frame_id=GUARDED, name=["arm/j1"], position=[1.0]), 0.0)

    # Starts where the joint is measured and moves 2 rad/s * 0.01 s per tick.
    first = task.compute(tick(0.0, measured=0.1))
    assert first is not None
    assert first.mode is ControlMode.IMPEDANCE
    assert first.positions == pytest.approx([0.12])
    assert first.velocities == [0.0]
    assert (first.stiffness, first.damping, first.efforts) == ([20.0], [1.5], None)
    second = task.compute(tick(0.01, measured=0.11))
    assert second is not None
    assert second.positions == pytest.approx([0.14])
    assert second.velocities == pytest.approx([2.0])

    # The leader pauses: let go at once.
    assert task.on_joint_command(
        JointState(frame_id=GUARDED, name=["arm/j1"], velocity=[0.0]), 0.02
    )
    assert task.compute(tick(0.02, measured=0.12)) is None
    assert not task.is_active()

    # The leader goes silent: let go after 0.4 s.
    assert task.on_joint_command(JointState(name=["arm/j1"], position=[1.0]), 1.0)
    assert task.compute(tick(1.3, measured=0.12)) is not None
    assert task.compute(tick(1.5, measured=0.12)) is None


def test_a_blocked_follower_pushes_no_harder_than_max_torque():
    task = make_task(max_torque=2.0, velocity_limit=5.0)
    task.on_joint_command(JointState(name=["arm/j1"], position=[3.0]), 0.0)

    # The follower stays at 0 however long the leader waits at 3 rad.
    outputs = [task.compute(tick(0.01 * i, measured=0.0)) for i in range(10)]

    # 2 Nm / 20 Nm/rad = 0.1 rad from where the joint is.
    targets = [out.positions[0] for out in outputs if out is not None and out.positions]
    assert len(targets) == 10 and max(targets) == pytest.approx(0.1)
    last = outputs[-1]
    assert last is not None
    assert last.positions == pytest.approx([0.1])
    assert last.velocities == [0.0]
