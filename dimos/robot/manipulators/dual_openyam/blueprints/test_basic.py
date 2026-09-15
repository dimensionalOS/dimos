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


from pathlib import Path

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryExecutionStatus,
)
from dimos.hardware.whole_body.mock.adapter import MockWholeBodyAdapter
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.assets.model import LoadedRobotModel, RobotModel
from dimos.robot.manipulators.dual_openyam.blueprints.basic import DualOpenYamCoordinator
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_ARM_JOINTS


@pytest.mark.parametrize("ports", [{}, {"left_can_port": "left", "right_can_port": "right"}])
def test_setup_supplies_model_limits_to_trajectory_execution(mocker, ports):
    names = DUAL_OPENYAM_ARM_JOINTS
    # Different bounds and reverse document order catch accidental positional mapping.
    bounds = {name: (-float(i + 1), float(i + 2)) for i, name in enumerate(names)}
    xml = (
        '<robot name="test"><link name="base"/>'
        + "".join(
            f'<link name="{name}_link"/><joint name="{name}" type="revolute">'
            f'<parent link="base"/><child link="{name}_link"/>'
            f'<limit lower="{bounds[name][0]}" upper="{bounds[name][1]}"/></joint>'
            for name in reversed(names)
        )
        + "</robot>"
    )
    mocker.patch.object(
        RobotModel, "load", return_value=LoadedRobotModel(xml, Path("model.urdf"), {})
    )
    mocker.patch.object(ControlCoordinator, "_setup_from_config")
    coordinator = DualOpenYamCoordinator(**ports)
    try:
        coordinator._setup_from_config()
        component = coordinator.config.hardware[0]
        adapter = MockWholeBodyAdapter(dof=len(component.joints))
        hardware = ConnectedWholeBody(adapter, component)
        task = JointTrajectoryTask(JointTrajectoryTaskConfig(joint_names=names), {"dual": hardware})
        limits = hardware.get_limits()
        assert limits is not None
        assert limits.position_lower == [*[bounds[name][0] for name in names], 0.0, 0.0]
        assert limits.position_upper == [*[bounds[name][1] for name in names], 1.0, 1.0]
        for name in (names[0], names[6]):
            valid = JointTrajectory(joint_names=[name], points=[TrajectoryPoint(positions=[0.0])])
            assert task.execute(valid, {}).status is TrajectoryExecutionStatus.ACCEPTED
            invalid = JointTrajectory(
                joint_names=[name], points=[TrajectoryPoint(positions=[bounds[name][1] + 0.1])]
            )
            assert (
                task.execute(invalid, {}).status
                is TrajectoryExecutionStatus.POSITION_LIMIT_VIOLATION
            )
    finally:
        coordinator.stop()
