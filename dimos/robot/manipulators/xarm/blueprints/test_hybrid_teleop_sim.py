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

"""OperatorHoldCoordinator: hold and e-stop status streams and the operator_estop RPC.

A rig wires one mock arm, the trajectory task and the hold task into a real
OperatorHoldCoordinator, then ticks it by hand so every assertion is
deterministic.
"""

from __future__ import annotations

from collections.abc import Iterator
from unittest.mock import MagicMock, call

import pytest

from dimos.control.components import HardwareComponent, HardwareType, make_joints
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
from dimos.control.tick_loop import TickLoop
from dimos.hardware.manipulators.spec import ManipulatorAdapter
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.robot.manipulators.xarm.blueprints.hybrid_teleop_sim import OperatorHoldCoordinator

ARM_JOINTS = make_joints("arm", 3)
ARM_POSITIONS = [0.1, 0.2, 0.3]


class EstopStub(BaseControlTask):
    """Idle task that records the last set_estop call; None until the first one."""

    def __init__(self) -> None:
        self._name = "estop_stub"
        self.estopped: bool | None = None

    def claim(self) -> ResourceClaim:
        return ResourceClaim(joints=frozenset(), priority=0, mode=ControlMode.SERVO_POSITION)

    def is_active(self) -> bool:
        return False

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        return None

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        pass

    def set_estop(self, estopped: bool) -> None:
        self.estopped = estopped


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

        self.coordinator = OperatorHoldCoordinator(publish_joint_state=False)
        self.coordinator.add_hardware(
            self.arm,
            HardwareComponent(
                hardware_id="arm", hardware_type=HardwareType.MANIPULATOR, joints=ARM_JOINTS
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
        self.estop_task = EstopStub()
        self.coordinator.add_task(self.estop_task)

        self.hold_statuses: list[Bool] = []
        self.coordinator.operator_hold_status.subscribe(self.hold_statuses.append)
        self.estop_statuses: list[Bool] = []
        self.coordinator.operator_estop_status.subscribe(self.estop_statuses.append)

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

    def arm_writes(self) -> list[list[float]]:
        return [call.args[0] for call in self.arm.write_joint_positions.call_args_list]


@pytest.fixture
def rig() -> Iterator[Rig]:
    rig = Rig()
    try:
        yield rig
    finally:
        rig.coordinator.stop()


class TestHoldStatus:
    def test_request_holds_the_arm_and_sends_true(self, rig):
        status = rig.coordinator.task_invoke(
            OPERATOR_HOLD_TASK_NAME, "request", {"route": "manual", "reason": "arm stuck"}
        )
        assert isinstance(status, OperatorHoldStatus) and status.on
        assert rig.hold_statuses == []

        rig.tick()

        assert rig.arm_writes()[-1] == pytest.approx(ARM_POSITIONS)
        assert [type(msg) for msg in rig.hold_statuses] == [Bool]
        assert rig.hold_statuses[-1].data is True

    def test_acknowledge_releases_and_sends_false(self, rig):
        rig.coordinator.task_invoke(OPERATOR_HOLD_TASK_NAME, "request", {"route": "manual"})
        rig.tick()

        rig.coordinator.task_invoke(OPERATOR_HOLD_TASK_NAME, "acknowledge")

        assert [msg.data for msg in rig.hold_statuses] == [True, False]
        assert not rig.hold_task.is_active()
        assert (
            rig.coordinator.execute_trajectory(_trajectory()).status
            is TrajectoryExecutionStatus.ACCEPTED
        )


class TestOperatorEstop:
    def test_stop_latches_then_holds_then_cancels(self, rig):
        steps = MagicMock()
        for name in ("set_estop", "task_invoke", "cancel_trajectory"):
            getattr(steps, name).side_effect = getattr(rig.coordinator, name)
            setattr(rig.coordinator, name, getattr(steps, name))

        rig.coordinator.operator_estop(True)

        assert steps.mock_calls == [
            call.set_estop(True),
            call.task_invoke(
                OPERATOR_HOLD_TASK_NAME, "request", {"route": "failsafe", "reason": "e-stop"}
            ),
            call.cancel_trajectory(),
        ]

    def test_stop_freezes_the_arm_mid_trajectory(self, rig):
        rig.coordinator.execute_trajectory(_trajectory())
        rig.tick()

        rig.coordinator.operator_estop(True)

        assert rig.estop_task.estopped is True
        assert rig.trajectory_task.get_state() is TrajectoryState.ABORTED
        hold = rig.hold_task.get_status()
        assert hold.on and hold.route == "failsafe" and hold.reason == "e-stop"
        assert [type(msg) for msg in rig.estop_statuses] == [Bool]
        assert rig.estop_statuses[-1].data is True
        rig.tick()
        assert rig.arm_writes()[-1] == pytest.approx(ARM_POSITIONS)

    def test_a_failed_step_does_not_skip_the_rest(self, rig):
        rig.coordinator.task_invoke = MagicMock(side_effect=RuntimeError("hold failed"))
        rig.coordinator.execute_trajectory(_trajectory())
        rig.tick()

        rig.coordinator.operator_estop(True)

        assert rig.estop_task.estopped is True
        assert rig.trajectory_task.get_state() is TrajectoryState.ABORTED

    def test_clear_releases_the_latch_but_not_the_hold(self, rig):
        rig.coordinator.operator_estop(True)

        rig.coordinator.operator_estop(False)

        assert rig.estop_task.estopped is False
        assert rig.hold_task.get_status().on
        assert [msg.data for msg in rig.estop_statuses] == [True, False]

    def test_start_sends_the_current_state(self):
        coordinator = OperatorHoldCoordinator(publish_joint_state=False)
        statuses: list[Bool] = []
        coordinator.operator_estop_status.subscribe(statuses.append)
        try:
            coordinator.start()
        finally:
            coordinator.stop()

        assert [msg.data for msg in statuses] == [False]
