# Copyright 2025-2026 Dimensional Inc.
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
"""The grasp blueprint's planning world knows the arms, the cameras, the table
and the bin: a plan into the bin wall must fail, free space must plan."""

from __future__ import annotations

from collections.abc import Iterator
import importlib.util
from unittest.mock import MagicMock

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.planning.planners.config import RRTConnectPlannerConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    DUAL_OPENYAM_GRASP_PINK,
    DUAL_OPENYAM_STATIC_BOXES,
    DUAL_OPENYAM_TABLE_TOP_Z,
    dual_openyam_grasp_model_config,
)
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_ARM_JOINTS

pytestmark = [
    pytest.mark.self_hosted,
    pytest.mark.skipif(importlib.util.find_spec("pydrake") is None, reason="Drake not installed"),
]

REST = JointState(name=list(DUAL_OPENYAM_ARM_JOINTS), position=[0.0] * 12)
HOME = [0.0, 1.047, 1.047, 0.0, 0.0, 0.0] * 2
# Right-arm configurations found by sweeping joints 1 to 4 (left arm at rest).
RIGHT_TCP_IN_BIN = [0.0] * 6 + [0.2, 1.9, 1.0, 0.2, 0.0, 0.0]
RIGHT_TCP_BELOW_TABLE = [0.0] * 6 + [-0.6, 0.2, 0.0, -1.2, 0.0, 0.0]
RIGHT_TCP_FREE = [0.0] * 6 + [-0.6, 0.0, 0.0, 0.0, 0.0, 0.0]


@pytest.fixture(params=["roboplan", "drake"])
def module(request: pytest.FixtureRequest) -> Iterator[ManipulationModule]:
    """The blueprint runs RoboPlan; Drake is the second opinion."""
    coordinator = MagicMock(spec=ControlCoordinator)
    coordinator.get_joint_positions.return_value = {}
    coordinator.task_invoke.return_value = TrajectoryStatus(state=TrajectoryState.COMPLETED)
    planner = {"drake": RRTConnectPlannerConfig()}
    mod = ManipulationModule(
        model=dual_openyam_grasp_model_config(),
        planning_timeout=15.0,
        world_backend=request.param,
        kinematics=DUAL_OPENYAM_GRASP_PINK,
        visualization={"backend": "none"},
        static_boxes=DUAL_OPENYAM_STATIC_BOXES,
        world_frame="world",
        **({"planner": planner[request.param]} if request.param in planner else {}),
    )
    mod._control_coordinator = coordinator
    mod.coordinator_joint_state = None  # type: ignore[assignment]
    mod.voxel_map = None  # type: ignore[assignment]
    mod.objects = None  # type: ignore[assignment]
    try:
        mod.start()
        mod._on_joint_state(REST)
        yield mod
    finally:
        mod.stop()


def _tcp(module: ManipulationModule, q: list[float]) -> PoseStamped:
    world = module._world_monitor.world  # type: ignore[union-attr]
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, JointState(name=list(DUAL_OPENYAM_ARM_JOINTS), position=q))
        return world.get_group_ee_pose(ctx, "right_manipulator")


def test_rest_and_home_poses_are_collision_free(module: ManipulationModule) -> None:
    assert module.is_collision_free([0.0] * 12)
    assert module.is_collision_free(HOME)
    assert module.is_collision_free(RIGHT_TCP_FREE)


def test_the_table_and_the_bin_are_in_the_world(module: ManipulationModule) -> None:
    assert {"table", "bin", "camera_post", "overhead_camera"} <= set(module.get_obstacles())


def test_fingertips_in_the_bin_or_under_the_table_collide(module: ManipulationModule) -> None:
    in_bin = _tcp(module, RIGHT_TCP_IN_BIN).position
    assert 0.40 < in_bin.x < 0.69 and -0.12 < in_bin.y < 0.12 and in_bin.z < 0.065
    assert not module.is_collision_free(RIGHT_TCP_IN_BIN)

    below = _tcp(module, RIGHT_TCP_BELOW_TABLE).position
    assert below.z < DUAL_OPENYAM_TABLE_TOP_Z
    assert not module.is_collision_free(RIGHT_TCP_BELOW_TABLE)


def test_plans_reach_free_space_and_refuse_the_bin(module: ManipulationModule) -> None:
    assert module.plan_to_poses({"right_manipulator": _tcp(module, HOME)}).succeeded
    module._on_joint_state(REST)
    assert not module.plan_to_poses({"right_manipulator": _tcp(module, RIGHT_TCP_IN_BIN)}).succeeded
