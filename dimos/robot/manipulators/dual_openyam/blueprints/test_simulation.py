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

"""The simulated blueprints are the real ones with only the connection swapped."""

from collections.abc import Callable, Iterator
from typing import Any

import numpy as np
from numpy.typing import NDArray
import pytest
from pytest_mock import MockerFixture

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.coordination.blueprints import Blueprint
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import CommandStatus, ExecutionStatus, PlanStatus
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.all_blueprints import all_blueprints
from dimos.robot.manipulators.dual_openyam._sim_test_helpers import Deployed, deployed
from dimos.robot.manipulators.dual_openyam.blueprints.basic import (
    dual_openyam_planner_coordinator,
)
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import (
    DUAL_OPENYAM_CAMERA_TRANSFORM,
    DUAL_OPENYAM_RECORD_TOPICS,
    DUAL_OPENYAM_TABLE_TOP_Z,
    DUAL_OPENYAM_VIEW_TOPICS,
    dual_openyam_grasp,
)
from dimos.robot.manipulators.dual_openyam.blueprints.simulation import (
    dual_openyam_grasp_sim,
    dual_openyam_sim,
    dual_openyam_sim_connection,
    teleop_webxr_dual_openyam_sim,
)
from dimos.robot.manipulators.dual_openyam.blueprints.teleop import teleop_webxr_dual_openyam
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM,
    DUAL_OPENYAM_HOME_JOINTS,
    dual_openyam_arm_joints,
)
from dimos.robot.manipulators.dual_openyam.conftest import PlannedFrame
from dimos.robot.manipulators.dual_openyam.connection import DualOpenYamConnection
from dimos.robot.manipulators.dual_openyam.sim_connection import DualOpenYamSimConnection
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.testing.waiting import wait_until

# What a stack may drive the robot through: the robot, its mock or the MuJoCo rig.
CONNECTIONS = (DualOpenYamConnection, MockJointConnection, DualOpenYamSimConnection)
# All a simulated stack leaves out of the real one: the connection and the cameras.
HARDWARE = (*CONNECTIONS, RealSenseCamera)
CASES = [
    ("dual-openyam-sim", dual_openyam_sim, dual_openyam_planner_coordinator),
    ("teleop-webxr-dual-openyam-sim", teleop_webxr_dual_openyam_sim, teleop_webxr_dual_openyam),
    ("dual-openyam-grasp-sim", dual_openyam_grasp_sim, dual_openyam_grasp),
]
ARM = [f"arm/{name}" for name in DUAL_OPENYAM.arm_joint_names]
# These checks need no camera, and so no OpenGL context.
PHYSICS_ALONE = {DualOpenYamSimConnection: {"cameras": False}}


@pytest.mark.parametrize(("name", "simulated", "real"), CASES, ids=[case[0] for case in CASES])
def test_simulated_stack_is_the_real_one_on_the_mujoco_rig(
    name: str, simulated: Blueprint, real: Blueprint, mocker: MockerFixture
) -> None:
    load = mocker.patch(
        "dimos.robot.assets.model.RobotModel.load", side_effect=AssertionError("must stay lazy")
    )
    assert all_blueprints[name].startswith(
        "dimos.robot.manipulators.dual_openyam.blueprints.simulation:"
    )

    def others(blueprint: Blueprint) -> list[tuple[str, type, dict[str, Any]]]:
        return [
            (atom.name, atom.module, atom.kwargs)
            for atom in blueprint.active_blueprints
            if atom.module not in HARDWARE
        ]

    assert others(simulated) == others(real)
    [connection] = [atom for atom in simulated.active_blueprints if atom.module in CONNECTIONS]
    assert connection.module is DualOpenYamSimConnection
    [coordinator] = [
        atom for atom in simulated.active_blueprints if issubclass(atom.module, ControlCoordinator)
    ]
    # The coordinator's declaration of the robot is the one the rig is built from.
    [declaration] = coordinator.kwargs["connections"]
    driver = ConnectionConfig.model_validate(
        {
            key: value
            for key, value in connection.kwargs.items()
            if key in ConnectionConfig.model_fields
        }
    )
    assert driver.source == declaration.source == "arm"
    assert driver.model == declaration.model
    assert driver.joint_commands == declaration.joint_commands
    assert simulated.remapping_map[(connection.name, "position_command")] == "arm_position_command"
    assert simulated.remapping_map[(coordinator.name, "arm_joint_state")] == "joint_state"
    load.assert_not_called()


def test_grasp_stack_gets_its_camera_streams_and_one_camera_mount_from_the_rig() -> None:
    def topics(blueprint: Blueprint, direction: str, *modules: type) -> set[tuple[str, type]]:
        return {
            (blueprint.remapping_map.get((atom.name, stream.name), stream.name), stream.type)
            for atom in blueprint.active_blueprints
            if not modules or atom.module in modules
            for stream in atom.streams
            if stream.direction == direction
        }

    real, simulated = dual_openyam_grasp, dual_openyam_grasp_sim
    # What a module reads, the recorder keeps or the viewer shows of the real cameras.
    listed = {*DUAL_OPENYAM_RECORD_TOPICS.split(","), *DUAL_OPENYAM_VIEW_TOPICS}
    read = topics(real, "in") | {
        stream for stream in topics(real, "out", RealSenseCamera) if stream[0] in listed
    }
    from_cameras = topics(real, "out", RealSenseCamera) & read
    assert {name for name, _ in from_cameras} == {
        "color_image",
        "depth_image",
        "camera_info",
        "pointcloud",
        "tf",
        "left_wrist_color_image",
        "left_wrist_camera_info",
        "right_wrist_color_image",
        "right_wrist_camera_info",
    }
    assert from_cameras <= topics(simulated, "out", DualOpenYamSimConnection)
    assert RealSenseCamera not in [atom.module for atom in simulated.active_blueprints]

    [rig] = [atom for atom in simulated.active_blueprints if atom.module in CONNECTIONS]
    [planner] = [atom for atom in simulated.active_blueprints if atom.module is ManipulationModule]
    # The map needs the overhead cloud, and that camera's mount has one publisher: the
    # planner, with the transform the simulated camera is placed by.
    assert rig.kwargs["enable_pointcloud"] is True
    assert rig.kwargs["publish_overhead_mount"] is False
    assert planner.kwargs["static_transforms"] == [DUAL_OPENYAM_CAMERA_TRANSFORM]


def test_composing_the_simulated_connection_downloads_no_model(mocker: MockerFixture) -> None:
    download = mocker.patch(
        "dimos.utils.data.get_data", side_effect=AssertionError("Config must not download models")
    )

    blueprint = dual_openyam_sim_connection()

    assert [atom.module for atom in blueprint.blueprints] == [DualOpenYamSimConnection]
    download.assert_not_called()


@pytest.mark.parametrize("simulated", [case[1] for case in CASES], ids=[case[0] for case in CASES])
def test_simulated_stack_needs_no_can_port_and_takes_the_rigs_options(
    simulated: Blueprint,
) -> None:
    parser = BlueprintConfigParser(simulated)

    connection = parser.parse(
        ["--headless", "false", "--cameras", "false", "--fps", "15"], environ={}
    ).module_kwargs("dualopenyamsimconnection")
    assert (connection["headless"], connection["cameras"], connection["fps"]) == (False, False, 15)
    # The real connection is switched off, and its options with it.
    with pytest.raises(BlueprintConfigError, match="left-can-port"):
        parser.parse(["--left-can-port", "can0"], environ={})


@pytest.fixture
def planning_stack(no_rpc: None) -> Iterator[Deployed]:
    """dual-openyam-sim: the rig, its coordinator and the planner."""
    with deployed(
        dual_openyam_sim,
        DualOpenYamSimConnection,
        ControlCoordinator,
        ManipulationModule,
        settings=PHYSICS_ALONE,
    ) as stack:
        # The planner has the rig's joints once they have passed through the coordinator.
        planner = stack.modules[ManipulationModule]
        wait_until(lambda: planner.get_current_joints() is not None, timeout=20.0)
        yield stack


@pytest.fixture
def teleop_stack(no_rpc: None) -> Iterator[Deployed]:
    """teleop-webxr-dual-openyam-sim's rig and coordinator, without its WebXR server and viewer."""
    with deployed(
        teleop_webxr_dual_openyam_sim,
        DualOpenYamSimConnection,
        ControlCoordinator,
        settings=PHYSICS_ALONE,
    ) as stack:
        coordinator = stack.modules[ControlCoordinator]
        wait_until(lambda: set(ARM) <= set(coordinator.get_joint_positions()), timeout=10.0)
        yield stack


@pytest.mark.self_hosted
def test_coordinator_runs_a_trajectory_and_a_jaw_on_the_rig(planning_stack: Deployed) -> None:
    coordinator = planning_stack.modules[ControlCoordinator]
    goal = [0.3, 0.9, 1.3, 0.2, -0.2, 0.4, -0.3, 0.9, 1.3, -0.2, 0.2, -0.4]

    result = coordinator.execute_trajectory(
        JointTrajectory(
            joint_names=ARM,
            points=[
                TrajectoryPoint(time_from_start=0.0, positions=DUAL_OPENYAM_HOME_JOINTS),
                TrajectoryPoint(time_from_start=2.0, positions=goal),
            ],
        )
    )
    assert result.status is TrajectoryExecutionStatus.ACCEPTED
    assert coordinator.task_invoke("left_arm_gripper", "set_normalized", {"values": [1.0]})

    targets = dict(zip(ARM, goal, strict=True))
    wait_until(lambda: planning_stack.distance_from(targets) < 0.02, timeout=15.0)
    wait_until(lambda: planning_stack.measured()["arm/left_gripper"] > 0.98, timeout=10.0)
    assert planning_stack.measured()["arm/right_gripper"] == pytest.approx(0.0, abs=0.01)


@pytest.mark.self_hosted
@pytest.mark.xfail(
    strict=True,
    reason=(
        "dual-openyam-planner-coordinator's planning model does not exclude each arm's link4 "
        "against its gripper, whose hulls always overlap on dual_openyam_workcell_v7, so the "
        "planner refuses every start: 'Start configuration is in collision, cannot plan!'. "
        "The grasp blueprint's model excludes the pair."
    ),
)
def test_planner_plans_and_the_rig_executes(planning_stack: Deployed) -> None:
    planner = planning_stack.modules[ManipulationModule]

    # A joint target for the left arm.
    targets = dict(
        zip(
            [f"arm/{name}" for name in dual_openyam_arm_joints("left")],
            [0.4, 0.9, 1.3, 0.2, -0.2, 0.4],
            strict=True,
        )
    )
    goal = JointState(name=list(targets), position=list(targets.values()))
    assert planner.plan_to_joints({"left_manipulator": goal}).status is PlanStatus.SUCCEEDED
    assert planner.execute(timeout=60.0).status is ExecutionStatus.COMPLETED
    wait_until(lambda: planning_stack.distance_from(targets) < 0.02, timeout=10.0)

    # A pose target for the right arm: fingers down, 15 cm above the table.
    above_table = [0.37, -0.09, DUAL_OPENYAM_TABLE_TOP_Z + 0.15]
    fingers_down = PoseStamped(
        frame_id="world", position=above_table, orientation=[0.0, 0.0, 1.0, 0.0]
    )
    assert planner.plan_to_poses({"right_manipulator": fingers_down}).status is PlanStatus.SUCCEEDED
    assert planner.execute(timeout=60.0).status is ExecutionStatus.COMPLETED

    def reached() -> list[float]:
        pose = planner.get_ee_pose("right_manipulator")
        assert pose is not None
        return [pose.position.x, pose.position.y, pose.position.z]

    wait_until(lambda: reached() == pytest.approx(above_table, abs=5e-3), timeout=10.0)

    # The planner's gripper command reaches the jaw of the group it names.
    opened = planner.set_gripper_position(1.0, planning_group="right_manipulator")
    assert opened.status is CommandStatus.SUCCEEDED
    wait_until(lambda: planning_stack.measured()["arm/right_gripper"] > 0.98, timeout=10.0)
    assert planning_stack.measured()["arm/left_gripper"] == pytest.approx(0.0, abs=0.01)


@pytest.mark.self_hosted
def test_a_held_grip_moves_that_arm_with_its_controller(
    teleop_stack: Deployed, planned_frame: PlannedFrame
) -> None:
    def grasp_frame(side: str) -> NDArray[np.float64]:
        """Where the rig's joints put that arm's grasp frame, by the planning model."""
        now = teleop_stack.measured()
        joints = {name.removeprefix("arm/"): now[name] for name in ARM}
        return np.array(planned_frame(f"{side}_grasp_frame", joints).translation)

    def operator(raised: float) -> None:
        """One sample of an operator holding the left grip, the controller ``raised`` metres."""
        buttons = Buttons()
        buttons.left_grip = True
        teleop_stack.topics["teleop_buttons"].broadcast(None, buttons)
        teleop_stack.topics["left_cartesian_command"].broadcast(
            None, PoseStamped(position=[0.0, 0.0, raised])
        )

    def hold(raised: float, until: Callable[[], bool]) -> None:
        """Keep the operator's samples coming until the condition holds."""

        def sampled() -> bool:
            operator(raised)
            return until()

        wait_until(sampled, timeout=10.0, interval=0.05)

    coordinator = teleop_stack.modules[ControlCoordinator]
    start = {side: grasp_frame(side) for side in ("left", "right")}
    right_arm = {name: teleop_stack.measured()[name] for name in ARM[6:]}

    # The grip engages where the arm is: some ticks on, the arm has not moved.
    engaged = coordinator.get_tick_count()
    hold(0.0, lambda: coordinator.get_tick_count() > engaged + 20)
    assert grasp_frame("left") == pytest.approx(start["left"], abs=2e-3)

    # Then the grasp frame follows the controller 5 cm up, and nothing else moves.
    hold(0.05, lambda: abs(grasp_frame("left")[2] - start["left"][2] - 0.05) < 5e-3)
    assert grasp_frame("left")[:2] == pytest.approx(start["left"][:2], abs=5e-3)
    assert grasp_frame("right") == pytest.approx(start["right"], abs=1e-3)
    assert teleop_stack.distance_from(right_arm) < 1e-3
