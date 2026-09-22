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

"""R1 Pro real-hardware and planar-preview blueprint contracts."""

import pytest

from dimos.control.components import HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control
from dimos.robot.galaxea.r1pro.blueprints.manipulation.r1pro_planar_preview import (
    r1pro_planar_preview,
)
from dimos.robot.galaxea.r1pro.config import R1PRO_PLANAR_BASE
from dimos.robot.galaxea.r1pro.connection import R1PRO_UPPER_BODY_JOINTS


def _coordinator_config(blueprint) -> ControlCoordinatorConfig:
    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    atom = next(
        atom for atom in blueprint.active_blueprints if issubclass(atom.module, ControlCoordinator)
    )
    return ControlCoordinatorConfig.model_validate(parsed.module_kwargs(atom.name))


def _forward(distance: float, duration: float) -> JointTrajectory:
    velocity = [distance / duration, 0.0, 0.0]
    return JointTrajectory(
        joint_names=["x", "y", "yaw"],
        points=[
            TrajectoryPoint(time_from_start=0.0, positions=[0.0, 0.0, 0.0], velocities=velocity),
            TrajectoryPoint(
                time_from_start=duration, positions=[distance, 0.0, 0.0], velocities=velocity
            ),
        ],
    )


def test_planar_preview_plans_on_the_planar_base_limits() -> None:
    parsed = BlueprintConfigParser(r1pro_planar_preview).parse(environ={})
    atom = next(
        atom
        for atom in r1pro_planar_preview.active_blueprints
        if issubclass(atom.module, ManipulationModule)
    )
    manipulation = ManipulationModuleConfig.model_validate(parsed.module_kwargs(atom.name))

    assert manipulation.visualization.backend == "viser"
    assert manipulation.trajectory_parametrization is None
    prepared = prepare_robot_model(manipulation.model)
    assert prepared.joint_space.velocity_limits[:3] == R1PRO_PLANAR_BASE.velocity_limits
    assert prepared.joint_space.acceleration_limits[:3] == R1PRO_PLANAR_BASE.acceleration_limits


def test_planar_preview_drives_its_mock_base_from_a_base_trajectory(wait_until) -> None:
    """The base half of a whole-body plan has to move this blueprint's mock chassis."""
    config = _coordinator_config(r1pro_planar_preview)
    coordinator = ControlCoordinator(
        publish_joint_state=False,
        tick_rate=config.tick_rate,
        hardware=config.hardware,
        tasks=config.tasks,
    )
    coordinator.start()
    try:
        accepted = coordinator.task_invoke(
            "base_trajectory", "execute", {"trajectory": _forward(0.2, duration=0.5)}
        )
        assert accepted.status is TrajectoryExecutionStatus.ACCEPTED
        wait_until(
            lambda: coordinator.task_invoke("base_trajectory", "get_status", {}).state
            is TrajectoryState.COMPLETED,
            timeout=5.0,
            message="the base trajectory never completed",
        )
        assert coordinator.get_joint_positions()["chassis/vx"] == pytest.approx(0.2, abs=0.03)
    finally:
        coordinator.stop()


def test_real_control_keeps_planar_joints_out_of_hardware() -> None:
    blueprint = r1pro_control()
    parsed = BlueprintConfigParser(blueprint).parse(environ={})
    coordinator_atom = next(
        atom for atom in blueprint.active_blueprints if issubclass(atom.module, ControlCoordinator)
    )
    coordinator = ControlCoordinatorConfig.model_validate(
        parsed.module_kwargs(coordinator_atom.name)
    )

    assert [hardware.hardware_type for hardware in coordinator.hardware] == [
        HardwareType.WHOLE_BODY,
        HardwareType.BASE,
    ]
    assert coordinator.hardware[0].joints == R1PRO_UPPER_BODY_JOINTS
    assert coordinator.hardware[1].joints == make_twist_base_joints("chassis")
    real_joint_names = {
        joint_name for hardware in coordinator.hardware for joint_name in hardware.joints
    }
    assert set(R1PRO_PLANAR_BASE.joint_names).isdisjoint(real_joint_names)
