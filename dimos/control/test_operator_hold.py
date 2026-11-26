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

"""Operator hold through the coordinator: request, preempt, reject, publish, e-stop, acknowledge.

A rig drives one arm and one twist base through connection ports on a real
coordinator, with the canonical trajectory task and the hold task, then ticks
it by hand so every assertion is deterministic.
"""

from __future__ import annotations

from collections.abc import Iterator
import time
from unittest.mock import MagicMock

import pytest

from dimos.control._control_test_helpers import LocalTransport, Recorder, fake_arm_model
from dimos.control.components import make_joints, make_twist_base_joints
from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.keys import Interface
from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.operator_hold_task.operator_hold_task import (
    OPERATOR_HOLD_TASK_NAME,
    OperatorHoldStatus,
    OperatorHoldTask,
    operator_hold_task,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryExecutionStatus,
)
from dimos.control.tasks.velocity_task.velocity_task import (
    JointVelocityTask,
    JointVelocityTaskConfig,
)
from dimos.control.tick_loop import TickLoop
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState

ARM_JOINTS = make_joints("arm", 3)
ARM_POSITIONS = [0.1, 0.2, 0.3]
BASE_JOINTS = make_twist_base_joints("base")


class TeleopStub(BaseControlTask):
    """Priority-50 stand-in for operator teleop: commands fixed arm positions once started."""

    def __init__(self, positions: list[float]) -> None:
        self._name = "teleop"
        self._positions = positions
        self.started = False

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=frozenset(ARM_JOINTS), priority=50, mode=ControlMode.SERVO_POSITION
        )

    def is_active(self) -> bool:
        return self.started

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return JointCommandOutput(
            joint_names=list(ARM_JOINTS),
            positions=list(self._positions),
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass


def _trajectory() -> JointTrajectory:
    """One second from the arm's current pose to +0.5 rad on every joint."""
    return JointTrajectory(
        joint_names=list(ARM_JOINTS),
        points=[
            TrajectoryPoint(positions=list(ARM_POSITIONS), time_from_start=0.0),
            TrajectoryPoint(positions=[p + 0.5 for p in ARM_POSITIONS], time_from_start=1.0),
        ],
    )


class HoldCoordinator(ControlCoordinator):
    """Ports for the arm and the base."""

    arm_position_command: Out[JointState]
    arm_joint_state: In[JointState]
    base_velocity_command: Out[JointState]
    base_joint_state: In[JointState]


class Rig:
    def __init__(self) -> None:
        self.coordinator = HoldCoordinator(
            publish_joint_state=False,
            reading_timeout_s=60.0,
            connections=[
                ConnectionConfig(
                    source="arm",
                    model=fake_arm_model(("joint1", "joint2", "joint3")),
                    joint_commands={
                        name: (Interface.POSITION,) for name in ("joint1", "joint2", "joint3")
                    },
                ),
                ConnectionConfig(
                    source="base",
                    model=fake_arm_model(("vx", "vy", "wz")),
                    joint_commands={name: (Interface.VELOCITY,) for name in ("vx", "vy", "wz")},
                ),
            ],
        )
        self.readings = {name: LocalTransport() for name in ("arm_joint_state", "base_joint_state")}
        for name, transport in self.readings.items():
            self.coordinator.set_transport(name, transport)  # type: ignore[arg-type]
        self.sent: list[tuple[str, JointState]] = []
        for name in ("arm_position_command", "base_velocity_command"):
            self.coordinator.set_transport(name, Recorder(name, self.sent))  # type: ignore[arg-type]
        self.coordinator.start()
        assert self.coordinator._tick_loop is not None
        self._loop = self.coordinator._tick_loop
        self.report_arm(ARM_POSITIONS)
        self.readings["base_joint_state"].publish(
            JointState(name=BASE_JOINTS, velocity=[0.0] * len(BASE_JOINTS))
        )

        self.trajectory_task = JointTrajectoryTask(
            JointTrajectoryTaskConfig(
                joint_names=ARM_JOINTS,
                velocity_limits=dict.fromkeys(ARM_JOINTS, 1000.0),
            )
        )
        self.coordinator.add_task(self.trajectory_task, task_type="trajectory")
        hold = self.coordinator._create_task_from_config(operator_hold_task())
        assert isinstance(hold, OperatorHoldTask)
        self.hold_task = hold
        self.coordinator.add_task(self.hold_task, task_type="operator_hold")

        self.statuses: list[OperatorHoldStatus] = []
        self.hold_task.set_status_publisher(self.statuses.append)

    def tick(self) -> None:
        self._loop._tick()

    def report_arm(self, positions: list[float]) -> None:
        still = [0.0] * len(ARM_JOINTS)
        self.readings["arm_joint_state"].publish(
            JointState(name=ARM_JOINTS, position=list(positions), velocity=still, effort=still)
        )

    def request(self, route: str = "manual", reason: str = "arm stuck") -> OperatorHoldStatus:
        return self.coordinator.task_invoke(  # type: ignore[no-any-return]
            OPERATOR_HOLD_TASK_NAME, "request", {"route": route, "reason": reason}
        )

    def acknowledge(self) -> OperatorHoldStatus:
        return self.coordinator.task_invoke(OPERATOR_HOLD_TASK_NAME, "acknowledge")  # type: ignore[no-any-return]

    def arm_writes(self) -> list[list[float]]:
        return [list(msg.position) for name, msg in self.sent if name == "arm_position_command"]

    def base_writes(self) -> list[list[float]]:
        return [list(msg.velocity) for name, msg in self.sent if name == "base_velocity_command"]


@pytest.fixture
def rig(mocker) -> Iterator[Rig]:
    mocker.patch.object(TickLoop, "start")
    rig = Rig()
    try:
        yield rig
    finally:
        rig.coordinator.stop()


class TestRequest:
    def test_holds_every_joint_where_it_is(self, rig):
        status = rig.request(route="manual", reason="arm stuck")

        assert status.on and status.route == "manual" and status.reason == "arm stuck"
        assert status.started_at > 0.0
        rig.tick()
        assert rig.hold_task.claim().joints == frozenset([*ARM_JOINTS, *BASE_JOINTS])
        assert rig.arm_writes()[-1] == pytest.approx(ARM_POSITIONS)
        # Truthful odometry does not make a velocity-only resource position-held.
        assert rig.base_writes()[-1] == [0.0] * len(BASE_JOINTS)

    def test_keeps_the_first_positions_if_the_arm_drifts(self, rig):
        rig.request()
        rig.tick()
        rig.report_arm([0.15, 0.2, 0.3])

        rig.tick()

        assert rig.arm_writes()[-1] == pytest.approx(ARM_POSITIONS)
        assert rig.hold_task.claim().joints == frozenset([*ARM_JOINTS, *BASE_JOINTS])

    def test_second_hold_task_is_refused_by_name(self, rig):
        assert rig.coordinator.add_task(OperatorHoldTask(), task_type="operator_hold") is False
        assert rig.coordinator.describe_task(OPERATOR_HOLD_TASK_NAME)["commands"].keys() == {
            "request",
            "acknowledge",
            "get_status",
        }


class TestWhileHeld:
    def test_running_trajectory_is_preempted(self, rig):
        rig.tick()
        assert (
            rig.coordinator.execute_trajectory(_trajectory()).status
            is TrajectoryExecutionStatus.ACCEPTED
        )
        rig.tick()
        assert rig.trajectory_task.get_state() is TrajectoryState.EXECUTING

        rig.request()
        rig.tick()

        assert rig.trajectory_task.get_state() is TrajectoryState.ABORTED
        assert not rig.trajectory_task.is_active()
        assert rig.arm_writes()[-1] == pytest.approx(ARM_POSITIONS)

    def test_trajectory_sent_during_hold_is_accepted_then_aborted_by_preemption(self, rig):
        rig.request(reason="arm stuck")
        rig.tick()

        result = rig.coordinator.execute_trajectory(_trajectory())
        rig.tick()

        assert result.status is TrajectoryExecutionStatus.ACCEPTED
        assert rig.trajectory_task.get_state() is TrajectoryState.ABORTED
        assert not rig.trajectory_task.is_active()
        assert rig.arm_writes()[-1] == pytest.approx(ARM_POSITIONS)  # the arm did not move


def test_velocity_stream_is_preempted_through_hold_estop_clear_and_ack(rig: Rig) -> None:
    velocity = JointVelocityTask(
        "velocity", JointVelocityTaskConfig(joint_names=BASE_JOINTS, priority=50, timeout=10.0)
    )
    velocity.on_preempted = MagicMock(wraps=velocity.on_preempted)
    assert rig.coordinator.add_task(velocity)
    moving = [0.4, -0.2, 0.1]
    assert velocity.set_velocities(moving, time.perf_counter())
    rig.tick()
    assert rig.base_writes()[-1] == moving

    rig.request()
    rig.tick()
    assert rig.arm_writes()[-1] == pytest.approx(ARM_POSITIONS)
    assert rig.base_writes()[-1] == [0.0] * len(BASE_JOINTS)
    velocity.on_preempted.assert_called_once_with(
        by_task=OPERATOR_HOLD_TASK_NAME, joints=frozenset(BASE_JOINTS)
    )
    # Live updates cannot drive through a hold.
    assert velocity.set_velocities(moving, time.perf_counter())
    rig.tick()
    assert rig.base_writes()[-1] == [0.0] * len(BASE_JOINTS)

    assert rig.coordinator.set_estop(True)
    assert not velocity.set_velocities(moving, time.perf_counter())
    assert rig.acknowledge().on  # Cannot release while e-stop is latched.
    writes_at_stop = len(rig.base_writes())
    rig.tick()
    assert len(rig.base_writes()) == writes_at_stop
    assert not rig.hold_task.claim().joints
    assert rig.coordinator.set_estop(False)
    assert not velocity.is_streaming()  # Cached pre-stop intent was discarded.
    rig.report_arm(ARM_POSITIONS)
    rig.readings["base_joint_state"].publish(
        JointState(name=BASE_JOINTS, velocity=[0.0] * len(BASE_JOINTS))
    )
    rig.tick()
    assert rig.base_writes()[-1] == [0.0] * len(BASE_JOINTS)
    # Fresh post-clear streaming is still preempted until the operator acknowledges.
    assert velocity.set_velocities(moving, time.perf_counter())
    rig.tick()
    assert rig.base_writes()[-1] == [0.0] * len(BASE_JOINTS)
    assert rig.acknowledge().on is False
    fresh = [-0.3, 0.1, 0.0]
    assert velocity.set_velocities(fresh, time.perf_counter())
    rig.tick()
    assert rig.base_writes()[-1] == fresh


class TestEstop:
    def test_estop_cancels_the_trajectory_and_turns_the_hold_on(self, rig):
        rig.tick()
        rig.coordinator.execute_trajectory(_trajectory())
        rig.tick()
        writes_before = len(rig.arm_writes())

        assert rig.coordinator.set_estop(True)
        rig.tick()

        assert rig.trajectory_task.get_state() is TrajectoryState.ABORTED
        status = rig.coordinator.task_invoke(OPERATOR_HOLD_TASK_NAME, "get_status")
        assert status.on and status.route == "failsafe" and status.reason == "e-stop"
        assert status.estopped
        assert rig.statuses[-1].estopped
        assert len(rig.arm_writes()) == writes_before  # nothing commands the arm while stopped

    def test_clear_holds_the_arm_where_it_is_then_until_acknowledged(self, rig):
        rig.coordinator.set_estop(True)
        rig.tick()
        assert rig.coordinator.set_estop(False)
        rig.report_arm([0.15, 0.25, 0.35])
        rig.readings["base_joint_state"].publish(
            JointState(name=BASE_JOINTS, velocity=[0.0] * len(BASE_JOINTS))
        )
        rig.tick()

        status = rig.coordinator.task_invoke(OPERATOR_HOLD_TASK_NAME, "get_status")
        assert status.on and status.route == "failsafe" and not status.estopped
        assert rig.arm_writes()[-1] == pytest.approx([0.15, 0.25, 0.35])
        assert rig.base_writes()[-1] == [0.0] * len(BASE_JOINTS)

        assert rig.acknowledge().on is False
        assert not rig.hold_task.is_active()


class TestAcknowledge:
    def test_turns_the_hold_off(self, rig):
        rig.request()
        rig.tick()

        status = rig.acknowledge()

        assert status.on is False and status.route == "" and status.started_at == 0.0
        assert rig.statuses[-1].on is False
        assert not rig.hold_task.is_active()
        assert rig.hold_task.claim().joints == frozenset()
        assert (
            rig.coordinator.execute_trajectory(_trajectory()).status
            is TrajectoryExecutionStatus.ACCEPTED
        )

    def test_autonomy_does_not_resume_but_teleop_can_take_over(self, rig):
        teleop = TeleopStub([0.4, 0.5, 0.6])
        rig.coordinator.add_task(teleop)
        rig.tick()
        assert (
            rig.coordinator.execute_trajectory(_trajectory()).status
            is TrajectoryExecutionStatus.ACCEPTED
        )
        rig.tick()
        rig.request()
        rig.tick()

        rig.acknowledge()
        rig.tick()

        assert rig.trajectory_task.get_state() is TrajectoryState.ABORTED
        assert rig.arm_writes()[-1] == pytest.approx(ARM_POSITIONS)  # held where it stopped
        teleop.started = True
        rig.tick()
        assert rig.arm_writes()[-1] == pytest.approx([0.4, 0.5, 0.6])
