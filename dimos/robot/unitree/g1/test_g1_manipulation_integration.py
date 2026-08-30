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

"""Self-hosted integration coverage for G1 full-body manipulation planning."""

import pytest

pytest.importorskip("roboplan.core")

from dimos.manipulation.planning.planners import roboplan_planner as roboplan_planner_module
from dimos.manipulation.planning.planners.roboplan_config import RoboPlanPlannerConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.world import roboplan_world as roboplan_world_module
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.unitree.g1.manip_config import (
    G1_LEFT_ARM_JOINTS,
    G1_READY_JOINTS,
    G1_RIGHT_ARM_JOINTS,
    g1_manipulation_model_config,
)

pytestmark = pytest.mark.self_hosted


def test_full_body_state_supports_collision_checked_arm_only_plan() -> None:
    config = g1_manipulation_model_config()
    world = roboplan_world_module.RoboPlanWorld()
    world.load_model(config)
    world.finalize()
    full_body_state = JointState(
        name=list(config.joint_names),
        position=[0.0] * len(config.joint_names),
    )
    world.sync_from_joint_state(full_body_state)

    assert world.is_ready()
    assert world.check_config_collision_free(full_body_state)

    selection = world._planning_groups.select(("left_arm", "right_arm"))
    arm_joint_names = [*G1_LEFT_ARM_JOINTS, *G1_RIGHT_ARM_JOINTS]
    start = JointState(name=arm_joint_names, position=[0.0] * len(arm_joint_names))
    goal = JointState(
        name=arm_joint_names,
        position=[*G1_READY_JOINTS["left_arm"], *G1_READY_JOINTS["right_arm"]],
    )

    result = roboplan_planner_module.RoboPlanPlanner(
        world, RoboPlanPlannerConfig()
    ).plan_selected_joint_path(
        world,
        selection,
        start,
        goal,
        timeout=3.0,
    )

    assert result.status is PlanningStatus.SUCCESS, result.message
    assert result.path
    assert all(point.name == arm_joint_names for point in result.path)
