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

"""Operator hold through the coordinator: request, preempt, reject, publish, acknowledge.

A rig wires one mock arm, one mock twist base, the canonical trajectory task
and the hold task into a real, unmodified coordinator, then ticks it by hand
so every assertion is deterministic.
"""

from __future__ import annotations

from collections.abc import Iterator
from unittest.mock import MagicMock

import pytest

from dimos.control.components import (
    HardwareComponent,
    HardwareType,
    make_joints,
    make_twist_base_joints,
)
from dimos.control.coordinator import ControlCoordinator
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    JointStateSnapshot,
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
from dimos.control.tick_loop import TickLoop
from dimos.hardware.drive_trains.spec import TwistBaseAdapter
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState

ARM_JOINTS = make_joints("arm", 3)
ARM_POSITIONS = [0.1, 0.2, 0.3]
BASE_JOINTS = make_twist_base_joints("base")
BASE_ODOMETRY = [1.5, -0.5, 0.25]


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


class Rig:
    def __init__(self) -> None:
        self.arm = MagicMock(spec=ManipulatorAdapter)
        self.arm.get_dof.return_value = len(ARM_JOINTS)
        self.arm.read_joint_positions.return_value = list(ARM_POSITIONS)
        self.arm.read_joint_velocities.return_value = [0.0] * len(ARM_JOINTS)
        self.arm.read_joint_efforts.return_value = [0.0] * len(ARM_JOINTS)
        self.arm.write_joint_positions.return_value = True
        self.arm.set_control_mode.return_value = True

        self.base = MagicMock(spec=TwistBaseAdapter)
        self.base.get_dof.return_value = len(BASE_JOINTS)
        self.base.read_velocities.return_value = [0.0] * len(BASE_JOINTS)
        self.base.read_odometry.return_value = list(BASE_ODOMETRY)
        self.base.write_velocities.return_value = True

        self.coordinator = ControlCoordinator(publish_joint_state=False)
        self.coordinator.add_hardware(
            self.arm,
            HardwareComponent(
                hardware_id="arm", hardware_type=HardwareType.MANIPULATOR, joints=ARM_JOINTS
            ),
        )
        self.coordinator.add_hardware(
            self.base,
            HardwareComponent(
                hardware_id="base", hardware_type=HardwareType.BASE, joints=BASE_JOINTS
            ),
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

        self._loop = TickLoop(
            tick_rate=100.0,
            hardware=self.coordinator._hardware,
            hardware_lock=self.coordinator._hardware_lock,
            tasks=self.coordinator._tasks,
            task_lock=self.coordinator._task_lock,
            joint_to_hardware=self.coordinator._joint_to_hardware,
        )

    def tick(self) -> None:
        self._loop._tick()

    def request(self, route: str = "manual", reason: str = "arm stuck") -> OperatorHoldStatus:
        return self.coordinator.task_invoke(  # type: ignore[no-any-return]
            OPERATOR_HOLD_TASK_NAME, "request", {"route": route, "reason": reason}
        )

    def acknowledge(self) -> OperatorHoldStatus:
        return self.coordinator.task_invoke(OPERATOR_HOLD_TASK_NAME, "acknowledge")  # type: ignore[no-any-return]

    def arm_writes(self) -> list[list[float]]:
        return [call.args[0] for call in self.arm.write_joint_positions.call_args_list]

    def base_writes(self) -> list[list[float]]:
        return [call.args[0] for call in self.base.write_velocities.call_args_list]


@pytest.fixture
def rig() -> Iterator[Rig]:
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
        assert rig.hold_task.claim().joints == frozenset(ARM_JOINTS) | frozenset(BASE_JOINTS)
        assert rig.arm_writes()[-1] == pytest.approx(ARM_POSITIONS)
        # A base "joint" is a velocity, so holding it means zero, not its odometry.
        assert rig.base_writes()[-1] == [0.0, 0.0, 0.0]

    def test_keeps_the_first_positions_if_the_arm_drifts(self, rig):
        rig.request()
        rig.tick()
        rig.arm.read_joint_positions.return_value = [0.15, 0.2, 0.3]

        rig.tick()

        assert rig.arm_writes()[-1] == pytest.approx(ARM_POSITIONS)

    def test_unknown_route_raises_and_holds_nothing(self, rig):
        with pytest.raises(ValueError):
            rig.request(route="typo")

        assert not rig.hold_task.is_active()
        rig.tick()
        assert rig.arm_writes() == []

    def test_second_hold_task_is_refused_by_name(self, rig):
        assert rig.coordinator.add_task(OperatorHoldTask(), task_type="operator_hold") is False
        assert rig.coordinator.describe_task(OPERATOR_HOLD_TASK_NAME)["commands"].keys() == {
            "request",
            "acknowledge",
            "get_status",
        }


class TestWhileHeld:
    def test_running_trajectory_is_preempted(self, rig):
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


class TestStatus:
    def test_first_tick_publishes(self, rig):
        rig.request(route="failsafe", reason="fault")
        assert rig.statuses == []

        rig.tick()

        assert len(rig.statuses) == 1
        status = rig.statuses[0]
        assert status.on and status.route == "failsafe" and status.reason == "fault"
        assert status.started_at > 0.0 and status.stamp >= status.started_at

    def test_repeats_once_per_second_while_on(self, rig):
        rig.request(route="agent", reason="need help")
        snapshot = JointStateSnapshot(
            joint_positions=dict(zip(ARM_JOINTS, ARM_POSITIONS, strict=True))
        )

        for t_now in (0.0, 0.5, 0.99, 1.0, 1.5, 2.0, 2.5, 3.0):
            rig.hold_task.compute(CoordinatorState(joints=snapshot, t_now=t_now, dt=0.01))

        assert [status.on for status in rig.statuses] == [True, True, True, True]
        assert {status.route for status in rig.statuses} == {"agent"}
        assert len({status.started_at for status in rig.statuses}) == 1

    def test_get_status_reads_without_changing_anything(self, rig):
        assert rig.coordinator.task_invoke(OPERATOR_HOLD_TASK_NAME, "get_status").on is False
        rig.request()
        assert rig.coordinator.task_invoke(OPERATOR_HOLD_TASK_NAME, "get_status").on is True
        assert rig.statuses == []


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
        assert (
            rig.coordinator.execute_trajectory(_trajectory()).status
            is TrajectoryExecutionStatus.ACCEPTED
        )
        rig.tick()
        rig.request()
        rig.tick()

        rig.acknowledge()
        writes_before = len(rig.arm_writes())
        rig.tick()

        assert rig.trajectory_task.get_state() is TrajectoryState.ABORTED
        assert len(rig.arm_writes()) == writes_before  # nothing is commanding the arm
        teleop.started = True
        rig.tick()
        assert rig.arm_writes()[-1] == pytest.approx([0.4, 0.5, 0.6])

    def test_acknowledge_when_not_held_publishes_nothing(self, rig):
        status = rig.acknowledge()

        assert status.on is False
        assert rig.statuses == []
