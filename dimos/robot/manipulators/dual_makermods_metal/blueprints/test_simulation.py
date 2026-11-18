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

"""dual-metal-sim moves the MuJoCo rig: the planner plans and the rig executes."""

from collections.abc import Iterator

import pytest
from pytest_mock import MockerFixture

from dimos.control.coordinator import ControlCoordinator
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import CommandStatus, ExecutionStatus, PlanStatus
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.dual_makermods_metal.blueprints.simulation import dual_metal_sim
from dimos.robot.manipulators.dual_makermods_metal.config import ARM_JOINTS, DUAL_METAL
from dimos.robot.manipulators.dual_makermods_metal.sim_connection import DualMetalSimConnection
from dimos.robot.manipulators.dual_openyam._sim_test_helpers import Deployed, deployed
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import DUAL_OPENYAM_TABLE_TOP_Z
from dimos.utils.testing.waiting import wait_until

# Without gravity compensation, as on the robot, each arm settles below its
# target by its weight over the motors' stiffness: up to 0.065 rad at the
# shoulder and elbow in these postures, about 2 cm at the gripper.
SAG_RAD = 0.08
SAG_M = 0.03


@pytest.fixture
def planning_stack(mocker: MockerFixture) -> Iterator[Deployed]:
    """dual-metal-sim: the rig, its coordinator and the planner."""
    # Modules built here serve no RPCs: their streams get in-memory transports.
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    with deployed(
        dual_metal_sim,
        DualMetalSimConnection,
        ControlCoordinator,
        ManipulationModule,
        # These checks need no camera, and so no OpenGL context.
        settings={DualMetalSimConnection: {"cameras": False}},
    ) as stack:
        # The planner has the rig's joints once they have passed through the coordinator.
        planner = stack.modules[ManipulationModule]
        wait_until(lambda: planner.get_current_joints() is not None, timeout=20.0)
        yield stack


@pytest.mark.self_hosted
def test_planner_plans_and_the_rig_executes(planning_stack: Deployed) -> None:
    planner = planning_stack.modules[ManipulationModule]

    # The left hand 5 cm straight up from where the arm starts, the jaw still
    # pointing down.
    start = planner.get_ee_pose("left_arm")
    assert start is not None
    up = PoseStamped(
        frame_id="world",
        position=[start.position.x, start.position.y, start.position.z + 0.05],
        orientation=start.orientation,
    )
    assert planner.plan_to_poses({"left_arm": up}).status is PlanStatus.SUCCEEDED

    # A joint target for the left arm.
    targets = dict(
        zip(
            [f"arm/{name}" for name in ARM_JOINTS["left"]],
            [0.4, -1.5, 1.6, -1.2, 0.3, 0.4],
            strict=True,
        )
    )
    goal = JointState(name=list(targets), position=list(targets.values()))
    assert planner.plan_to_joints({"left_arm": goal}).status is PlanStatus.SUCCEEDED
    assert planner.execute(timeout=60.0).status is ExecutionStatus.COMPLETED
    wait_until(lambda: planning_stack.distance_from(targets) < SAG_RAD, timeout=10.0)

    # A pose target for the right arm: the jaw pointing down, its mount 25 cm
    # above the table. The fingers point along gripper_base's z.
    above_table = [0.28, -0.25, DUAL_OPENYAM_TABLE_TOP_Z + 0.25]
    fingers_down = PoseStamped(
        frame_id="world", position=above_table, orientation=[1.0, 0.0, 0.0, 0.0]
    )
    assert planner.plan_to_poses({"right_arm": fingers_down}).status is PlanStatus.SUCCEEDED
    assert planner.execute(timeout=60.0).status is ExecutionStatus.COMPLETED

    def reached() -> list[float]:
        pose = planner.get_ee_pose("right_arm")
        assert pose is not None
        return [pose.position.x, pose.position.y, pose.position.z]

    wait_until(lambda: reached() == pytest.approx(above_table, abs=SAG_M), timeout=10.0)

    # The planner's gripper command reaches the jaw of the group it names.
    opened = DUAL_METAL.gripper_travel()["right_joint7"][1]
    assert planner.set_gripper_position(1.0, planning_group="right_arm").status is (
        CommandStatus.SUCCEEDED
    )
    wait_until(
        lambda: planning_stack.measured()["arm/right_joint7"] == pytest.approx(opened, abs=1e-3),
        timeout=10.0,
    )
    assert planning_stack.measured()["arm/left_joint7"] == pytest.approx(0.0, abs=1e-3)
