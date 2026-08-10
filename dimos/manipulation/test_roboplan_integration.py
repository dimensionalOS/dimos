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

"""Self-hosted integration tests for official RoboPlan Cartesian planning."""

import importlib
from pathlib import Path
from typing import Any

import pytest

pytest.importorskip("roboplan.cartesian_planning")
roboplan_world_module = importlib.import_module("dimos.manipulation.planning.world.roboplan_world")
roboplan_planner_module = importlib.import_module(
    "dimos.manipulation.planning.planners.roboplan_planner"
)

from dimos.manipulation.planning.planners.roboplan_config import (
    RoboPlanCartesianPathConfig,
    RoboPlanPlannerConfig,
)
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.utils.kinematics_utils import compute_pose_error
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.xarm.config import (
    make_dual_xarm6_model_config,
    make_xarm6_model_config,
)
from dimos.utils.transform_utils import pose_to_matrix

pytestmark = pytest.mark.self_hosted


@pytest.fixture
def roboplan_types() -> tuple[type[Any], type[Any]]:
    """Reload real bindings after any fake-binding tests in the same pytest process."""
    world_type = importlib.reload(roboplan_world_module).RoboPlanWorld
    planner_type = importlib.reload(roboplan_planner_module).RoboPlanPlanner
    return world_type, planner_type


def _sync_zero_state(
    world: Any,
    joint_names: list[str],
) -> None:
    world.sync_from_joint_state(
        JointState(name=joint_names, position=[0.0] * len(joint_names)),
    )


def test_real_roboplan_plans_fixed_orientation_cartesian_path(
    roboplan_types: tuple[type[Any], type[Any]],
) -> None:
    config = make_xarm6_model_config()
    if not Path(config.model_path).exists():
        pytest.skip(f"xArm model is unavailable: {config.model_path}")

    world_type, planner_type = roboplan_types
    world = world_type()
    world.load_model(config)
    world.finalize()
    planner = planner_type(world, RoboPlanPlannerConfig())
    _sync_zero_state(world, config.joint_names)
    group_id = "manipulator"
    selection = world._planning_groups.select((group_id,))
    start = JointState(
        name=list(selection.joint_names), position=[0.0] * len(selection.joint_names)
    )
    with world.scratch_context() as ctx:
        planner._apply_selected_state(ctx, start)
        start_pose = world.get_group_ee_pose(ctx, group_id)

    result = planner.plan_cartesian_path(
        world,
        selection,
        start,
        {
            group_id: (
                Transform.identity(),
                Transform(translation=Vector3(0.005, 0.0, 0.0)),
            )
        },
        RoboPlanCartesianPathConfig(
            speed_mode="time_optimal",
            toppra_blend_deviation=0.0,
        ),
    )

    assert result.status == PlanningStatus.SUCCESS, result.message
    assert len(result.path) >= 2
    assert result.timestamps is not None
    assert len(result.timestamps) == len(result.path)
    with world.scratch_context() as ctx:
        planner._apply_selected_state(ctx, result.path[-1])
        final_pose = world.get_group_ee_pose(ctx, group_id)
    expected = pose_to_matrix(start_pose)
    expected[0, 3] += 0.005
    position_error, orientation_error = compute_pose_error(pose_to_matrix(final_pose), expected)
    assert position_error <= 0.005
    assert orientation_error <= 0.01


def test_real_roboplan_synchronizes_different_length_dual_arm_targets(
    roboplan_types: tuple[type[Any], type[Any]],
) -> None:
    config = make_dual_xarm6_model_config()
    if not Path(config.model_path).exists():
        pytest.skip(f"xArm model is unavailable: {config.model_path}")

    world_type, planner_type = roboplan_types
    world = world_type()
    world.load_model(config)
    world.finalize()
    planner = planner_type(world, RoboPlanPlannerConfig())
    _sync_zero_state(world, config.joint_names)
    left_group_id = "left_arm"
    right_group_id = "right_arm"
    selection = world._planning_groups.select((left_group_id, right_group_id))
    start = JointState(
        name=list(selection.joint_names), position=[0.0] * len(selection.joint_names)
    )

    result = planner.plan_cartesian_path(
        world,
        selection,
        start,
        {
            left_group_id: (
                Transform.identity(),
                Transform(translation=Vector3(0.0015, 0.001, 0.0)),
                Transform(translation=Vector3(0.003, 0.0, 0.0)),
            ),
            right_group_id: (
                Transform.identity(),
                Transform(translation=Vector3(0.005, 0.0, 0.0)),
            ),
        },
        RoboPlanCartesianPathConfig(),
    )

    assert result.status == PlanningStatus.SUCCESS, result.message
    assert result.timestamps is not None
    assert len(result.timestamps) == len(result.path)
    assert all(state.name == list(selection.joint_names) for state in result.path)
