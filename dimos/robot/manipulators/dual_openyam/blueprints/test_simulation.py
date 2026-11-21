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

"""The simulated blueprints move the MuJoCo rig: trajectories, the planner and teleop."""

from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any

import numpy as np
from numpy.typing import NDArray
import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.trajectory_task.trajectory_task import TrajectoryExecutionStatus
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import CommandStatus, ExecutionStatus, PlanStatus
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.damiao._damiao_test_helpers import LocalTransport
from dimos.robot.manipulators.dual_openyam.blueprints.simulation import (
    dual_openyam_sim,
    teleop_webxr_dual_openyam_sim,
)
from dimos.robot.manipulators.dual_openyam.config import (
    DUAL_OPENYAM,
    DUAL_OPENYAM_HOME_JOINTS,
    dual_openyam_arm_joints,
)
from dimos.robot.manipulators.dual_openyam.conftest import PlannedFrame
from dimos.robot.manipulators.dual_openyam.sim_connection import DualOpenYamSimConnection
from dimos.robot.manipulators.dual_openyam.sim_scene import DUAL_OPENYAM_TABLE_TOP_Z
from dimos.teleop.webxr.controller_types import Buttons
from dimos.utils.testing.waiting import wait_until

ARM = [f"arm/{name}" for name in DUAL_OPENYAM.arm_joint_names]


@dataclass
class Deployed:
    """Modules of a blueprint running in this process, and the transport of every topic."""

    modules: dict[type, Any]
    topics: dict[str, LocalTransport[Any]]

    def measured(self) -> dict[str, float]:
        """The rig's newest joint positions, by joint name."""
        reading = self.topics["joint_state"].latest
        return dict(zip(reading.name, reading.position, strict=True))

    def distance_from(self, targets: dict[str, float]) -> float:
        """How far the farthest of the named joints is from its target."""
        now = self.measured()
        return max(abs(now[name] - target) for name, target in targets.items())


@contextmanager
def deployed(blueprint: Blueprint, *wanted: type) -> Iterator[Deployed]:
    """Run the blueprint's modules of the wanted types in this process.

    They are wired topic by topic as the blueprint wires them, with in-memory
    transports, and started in the order given.
    """
    stack = Deployed({}, {})
    try:
        for atom in blueprint.active_blueprints:
            kinds = [kind for kind in wanted if issubclass(atom.module, kind)]
            if not kinds:
                continue
            kwargs = dict(atom.kwargs)
            if atom.module is DualOpenYamSimConnection:
                # Physics alone: these checks need no OpenGL context.
                kwargs["cameras"] = False
            module = atom.module(**kwargs)
            stack.modules[kinds[0]] = module
            for stream in atom.streams:
                topic = blueprint.remapping_map.get((atom.name, stream.name), stream.name)
                module.set_transport(stream.name, stack.topics.setdefault(topic, LocalTransport()))
        if ManipulationModule in stack.modules:
            planner = stack.modules[ManipulationModule]
            # The blueprint hands the planner its coordinator; here it is the object itself.
            planner.set_module_ref("_control_coordinator", stack.modules[ControlCoordinator])
            # Nothing here publishes a voxel map or objects. Without the ports the planner
            # starts no handler for them, as in dimos/manipulation's fixtures.
            planner.voxel_map = None
            planner.objects = None
        for kind in wanted:
            stack.modules[kind].start()
        yield stack
    finally:
        for module in reversed(stack.modules.values()):
            module.stop()


@pytest.fixture
def planning_stack(no_rpc: None) -> Iterator[Deployed]:
    """dual-openyam-sim: the rig, its coordinator and the planner."""
    with deployed(
        dual_openyam_sim, DualOpenYamSimConnection, ControlCoordinator, ManipulationModule
    ) as stack:
        # The planner has the rig's joints once they have passed through the coordinator.
        planner = stack.modules[ManipulationModule]
        wait_until(lambda: planner.get_current_joints() is not None, timeout=20.0)
        yield stack


@pytest.fixture
def teleop_stack(no_rpc: None) -> Iterator[Deployed]:
    """teleop-webxr-dual-openyam-sim's rig and coordinator, without its WebXR server and viewer."""
    with deployed(
        teleop_webxr_dual_openyam_sim, DualOpenYamSimConnection, ControlCoordinator
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
