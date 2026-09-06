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

"""R1 Pro planar planning blueprint contracts."""

from copy import deepcopy
from typing import cast

import pytest

from dimos.control.components import HardwareType, make_twist_base_joints
from dimos.control.coordinator import ControlCoordinator, ControlCoordinatorConfig
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.planning.factory import create_planning_stack
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.validation import validate_robot_model_config
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control
from dimos.robot.galaxea.r1pro.blueprints.manipulation.r1pro_planner_coordinator import (
    r1pro_planner_coordinator,
)
from dimos.robot.galaxea.r1pro.config import (
    R1PRO_DESCRIPTION_SOURCE,
    R1PRO_MODEL,
    R1PRO_MODEL_PATH,
    R1PRO_PLANAR_BASE,
    R1PRO_PLANAR_MODEL,
    R1PRO_PLANNING_JOINTS,
    R1PRO_UPPER_BODY_PLANNING_JOINTS,
    make_r1pro_model_config,
    make_r1pro_planar_model_config,
)
from dimos.robot.galaxea.r1pro.connection import R1PRO_UPPER_BODY_JOINTS
from dimos.robot.galaxea.r1pro.joints import (
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    TORSO_JOINTS,
    coordinator_name,
)


def test_r1pro_uses_pinned_official_2026_description() -> None:
    assert R1PRO_DESCRIPTION_SOURCE.url == "https://github.com/userguide-galaxea/URDF"
    assert R1PRO_DESCRIPTION_SOURCE.ref == "2e5d31e1784481a34d178006c0d0e18e0a84a82a"
    assert cast("tuple[str, ...]", R1PRO_MODEL_PATH.parts)[-4:] == (
        "R1Pro",
        "urdf_r1pro_g1z_2026",
        "urdf",
        "r1pro_2026.urdf",
    )


def test_r1pro_config_exposes_disjoint_mobile_bimanual_groups() -> None:
    config = make_r1pro_planar_model_config()

    assert config.model is R1PRO_PLANAR_MODEL
    assert config.base_link == R1PRO_PLANAR_BASE.root_link
    assert config.joint_names == list(R1PRO_PLANNING_JOINTS)
    assert [group.name for group in config.planning_groups] == [
        "left_arm",
        "right_arm",
        "torso",
        "moving_base",
    ]
    assert config.planning_groups[0].joint_names == tuple(
        coordinator_name(joint) for joint in LEFT_ARM_JOINTS
    )
    assert config.planning_groups[0].base_link == "left_arm_base_link"
    assert config.planning_groups[0].tip_link == "left_gripper_link"
    assert config.planning_groups[1].joint_names == tuple(
        coordinator_name(joint) for joint in RIGHT_ARM_JOINTS
    )
    assert config.planning_groups[1].base_link == "right_arm_base_link"
    assert config.planning_groups[1].tip_link == "right_gripper_link"
    assert config.planning_groups[2].joint_names == tuple(
        coordinator_name(joint) for joint in TORSO_JOINTS
    )
    assert config.planning_groups[2].base_link == "base_link"
    assert config.planning_groups[2].tip_link is None
    assert config.planning_groups[3].joint_names == R1PRO_PLANAR_BASE.joint_names
    assert config.planning_groups[3].base_link == R1PRO_PLANAR_BASE.root_link
    assert config.planning_groups[3].tip_link is None

    grouped_joints = [
        joint_name for group in config.planning_groups for joint_name in group.joint_names
    ]
    assert len(grouped_joints) == len(set(grouped_joints))
    assert set(grouped_joints) == set(R1PRO_PLANNING_JOINTS)


def test_r1pro_hardware_model_matches_upper_body_feedback() -> None:
    config = make_r1pro_model_config()

    assert config.model is R1PRO_MODEL
    assert config.base_link == "base_link"
    assert config.joint_names == list(R1PRO_UPPER_BODY_PLANNING_JOINTS)
    assert [group.name for group in config.planning_groups] == [
        "left_arm",
        "right_arm",
        "torso",
    ]


def test_r1pro_groups_compose_arm_pose_targets_with_mobile_auxiliaries() -> None:
    registry = PlanningGroupRegistry(make_r1pro_planar_model_config().planning_groups)

    left_mobile = registry.select(("left_arm", "torso", "moving_base"))
    bimanual_mobile = registry.select(("left_arm", "right_arm", "torso", "moving_base"))

    assert left_mobile.joint_names == (
        *(coordinator_name(joint) for joint in LEFT_ARM_JOINTS),
        *(coordinator_name(joint) for joint in TORSO_JOINTS),
        *R1PRO_PLANAR_BASE.joint_names,
    )
    assert set(bimanual_mobile.joint_names) == set(R1PRO_PLANNING_JOINTS)


def test_r1pro_blueprint_wires_viser_planner_to_fake_hardware() -> None:
    parsed = BlueprintConfigParser(r1pro_planner_coordinator).parse(environ={})
    manipulation_atom = next(
        atom
        for atom in r1pro_planner_coordinator.active_blueprints
        if issubclass(atom.module, ManipulationModule)
    )
    coordinator_atom = next(
        atom
        for atom in r1pro_planner_coordinator.active_blueprints
        if issubclass(atom.module, ControlCoordinator)
    )
    manipulation = ManipulationModuleConfig.model_validate(
        parsed.module_kwargs(manipulation_atom.name)
    )
    coordinator = ControlCoordinatorConfig.model_validate(
        parsed.module_kwargs(coordinator_atom.name)
    )

    assert manipulation.visualization.backend == "viser"
    assert coordinator.hardware[0].hardware_type == HardwareType.WHOLE_BODY
    assert coordinator.hardware[0].adapter_type == "mock_whole_body"
    assert coordinator.hardware[0].joints == list(R1PRO_PLANNING_JOINTS)
    assert coordinator.tasks[0].joint_names == list(R1PRO_PLANNING_JOINTS)


def test_r1pro_real_control_keeps_planar_positions_unwired() -> None:
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
    assert coordinator.tasks[0].joint_names == R1PRO_UPPER_BODY_JOINTS
    assert coordinator.tasks[1].joint_names == make_twist_base_joints("chassis")
    assert set(R1PRO_PLANAR_BASE.joint_names).isdisjoint(R1PRO_UPPER_BODY_JOINTS)


@pytest.mark.self_hosted
def test_r1pro_materialized_model_matches_configured_coordinates() -> None:
    config = make_r1pro_planar_model_config()

    model = validate_robot_model_config(config)

    assert model.root_link == R1PRO_PLANAR_BASE.root_link
    assert {joint.name for joint in model.joints if joint.type != "fixed"} == set(
        R1PRO_PLANNING_JOINTS
    )


@pytest.mark.self_hosted
def test_r1pro_arm_ik_leaves_the_singular_zero_pose() -> None:
    config = make_r1pro_planar_model_config()
    world, kinematics, _planner = create_planning_stack(config)
    groups = PlanningGroupRegistry(config.planning_groups)
    zero = JointState(name=config.joint_names, position=[0.0] * len(config.joint_names))

    with world.scratch_context() as ctx:
        assert world.is_collision_free(ctx)
        targets = {
            group_id: deepcopy(world.get_group_ee_pose(ctx, group_id))
            for group_id in ("left_arm", "right_arm")
        }

    for group_id, target in targets.items():
        target.position.x += 0.05
        target.position.z += 0.005
        result = kinematics.solve_pose_targets(
            world,
            {groups.get(group_id): target},
            seed=zero,
            check_collision=True,
            max_attempts=1,
        )

        assert result.status == IKStatus.SUCCESS
        assert result.position_error <= 0.001
        assert result.orientation_error <= 0.01
        assert result.joint_state is not None
        assert result.joint_state.name == list(groups.get(group_id).joint_names)
