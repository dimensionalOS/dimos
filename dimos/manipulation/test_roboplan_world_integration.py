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

"""Self-hosted integration tests for real RoboPlan bindings."""

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.utils.kinematics_utils import compute_pose_error
from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.xarm.config import make_xarm6_model_config
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

pytestmark = pytest.mark.self_hosted

_ROBOPLAN_CORE = pytest.importorskip("roboplan.core")
_ROBOPLAN_OPTIMAL_IK = pytest.importorskip("roboplan.optimal_ik")


def test_real_roboplan_linear_tcp_oink_plans_xarm6_mimic_model() -> None:
    """Exercise real Oink linear TCP planning with a mimic-joint xArm6 model."""
    config = make_xarm6_model_config(name="left_arm")
    if not Path(config.model_path).exists():
        pytest.skip(f"xArm model is unavailable: {config.model_path}")

    world = RoboPlanWorld()
    world.add_robot(config)
    world.finalize()
    group_id = world._planning_groups.primary_pose_group_id_for_robot("left_arm")
    assert group_id == "left_arm/manipulator"
    selection = world._planning_groups.select((group_id,))
    start = JointState(
        name=list(selection.joint_names), position=[0.0] * len(selection.joint_names)
    )

    with world.scratch_context() as ctx:
        world._set_selection_state(ctx, selection, start)
        start_pose = world.get_group_ee_pose(ctx, group_id)

    target_matrix = pose_to_matrix(start_pose)
    target_matrix[0, 3] += 0.005
    target_pose = matrix_to_pose(target_matrix)
    target = PoseStamped(
        frame_id="world",
        position=target_pose.position,
        orientation=target_pose.orientation,
    )

    result = world.plan_cartesian_path(
        world,
        selection,
        start,
        {group_id: target},
        path_mode="linear",
        timeout=5.0,
    )

    assert result.status == PlanningStatus.SUCCESS, result.message
    assert len(result.path) >= 2
    assert result.path[-1].name == list(selection.joint_names)
    with world.scratch_context() as ctx:
        world._set_selection_state(ctx, selection, result.path[-1])
        final_pose = world.get_group_ee_pose(ctx, group_id)
    position_error, orientation_error = compute_pose_error(
        pose_to_matrix(final_pose), pose_to_matrix(target)
    )
    assert position_error <= 1e-3
    assert orientation_error <= 1e-2


def test_real_roboplan_linear_tcp_oink_plans_dual_xarm6_right_arm_plus_x() -> None:
    """Exercise world-frame Oink targets in a dual-arm composite scene."""
    left_config = make_xarm6_model_config(name="left_arm", y_offset=0.3)
    right_config = make_xarm6_model_config(name="right_arm", y_offset=-0.3)
    if not Path(left_config.model_path).exists():
        pytest.skip(f"xArm model is unavailable: {left_config.model_path}")

    world = RoboPlanWorld()
    world.add_robot(left_config)
    world.add_robot(right_config)
    world.finalize()
    group_id = world._planning_groups.primary_pose_group_id_for_robot("right_arm")
    assert group_id == "right_arm/manipulator"
    selection = world._planning_groups.select((group_id,))
    start = JointState(
        name=list(selection.joint_names), position=[0.0] * len(selection.joint_names)
    )

    with world.scratch_context() as ctx:
        world._set_selection_state(ctx, selection, start)
        start_pose = world.get_group_ee_pose(ctx, group_id)

    target_matrix = pose_to_matrix(start_pose)
    target_matrix[0, 3] += 0.005
    target_pose = matrix_to_pose(target_matrix)
    target = PoseStamped(
        frame_id="world",
        position=target_pose.position,
        orientation=target_pose.orientation,
    )

    result = world.plan_cartesian_path(
        world,
        selection,
        start,
        {group_id: target},
        path_mode="linear",
        timeout=5.0,
    )

    assert result.status == PlanningStatus.SUCCESS, result.message
    assert len(result.path) >= 2
    assert result.path[-1].name == list(selection.joint_names)
    with world.scratch_context() as ctx:
        world._set_selection_state(ctx, selection, result.path[-1])
        final_pose = world.get_group_ee_pose(ctx, group_id)
    position_error, orientation_error = compute_pose_error(
        pose_to_matrix(final_pose), pose_to_matrix(target)
    )
    assert position_error <= 1e-3
    assert orientation_error <= 1e-2
