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

"""OperatorHoldCoordinator: the hold and e-stop status streams.

A rig wires one mock arm, the trajectory task and the hold task into a real
OperatorHoldCoordinator, then ticks it by hand so every assertion is
deterministic.
"""

from __future__ import annotations

from collections.abc import Iterator
from unittest.mock import MagicMock

import pytest

from dimos.control.components import HardwareComponent, HardwareType, make_joints
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
from dimos.robot.manipulators.xarm.blueprints.hybrid_teleop_sim import OperatorHoldCoordinator

ARM_JOINTS = make_joints("arm", 3)
ARM_POSITIONS = [0.1, 0.2, 0.3]


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


class TestEstopStatus:
    def test_set_estop_sends_true_then_false(self, rig):
        rig.coordinator.set_estop(True)

        assert [type(msg) for msg in rig.estop_statuses] == [Bool]
        assert rig.estop_statuses[-1].data is True
        assert rig.hold_task.get_status().estopped
        assert rig.hold_statuses[-1].data is True  # the e-stop turned the hold on

        rig.coordinator.set_estop(False)

        assert [msg.data for msg in rig.estop_statuses] == [True, False]
        assert rig.hold_statuses[-1].data is True  # on until acknowledged

    def test_start_sends_the_current_state(self):
        coordinator = OperatorHoldCoordinator(publish_joint_state=False)
        statuses: list[Bool] = []
        coordinator.operator_estop_status.subscribe(statuses.append)
        try:
            coordinator.start()
        finally:
            coordinator.stop()

        assert [msg.data for msg in statuses] == [False]
