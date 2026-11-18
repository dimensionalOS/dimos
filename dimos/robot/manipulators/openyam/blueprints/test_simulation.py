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

"""openyam-sim moves the MuJoCo rig: the planner plans and the rig executes."""

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import Blueprint
from dimos.manipulation.manipulation_module import ManipulationModule
from dimos.manipulation.manipulation_spec import CommandStatus, ExecutionStatus, PlanStatus
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.damiao._damiao_test_helpers import LocalTransport
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import DUAL_OPENYAM_TABLE_TOP_Z
from dimos.robot.manipulators.openyam.blueprints.simulation import openyam_sim
from dimos.robot.manipulators.openyam.config import OPENYAM
from dimos.robot.manipulators.openyam.sim_connection import (
    OPENYAM_ROOT_POSITION,
    OpenYamSimConnection,
)
from dimos.utils.testing.waiting import wait_until

ARM = [f"arm/{name}" for name in OPENYAM.arm_joint_names]


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
            if atom.module is OpenYamSimConnection:
                # Physics alone: these checks need no OpenGL context.
                kwargs["cameras"] = False
            module = atom.module(**kwargs)
            stack.modules[kinds[0]] = module
            for stream in atom.streams:
                topic = blueprint.remapping_map.get((atom.name, stream.name), stream.name)
                module.set_transport(stream.name, stack.topics.setdefault(topic, LocalTransport()))
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
def planning_stack(mocker: MockerFixture) -> Iterator[Deployed]:
    """openyam-sim: the rig, its coordinator and the planner."""
    # Modules built here serve no RPCs: their streams get in-memory transports.
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.start")
    mocker.patch("dimos.protocol.rpc.zenohrpc.ZenohRPC.serve_module_rpc")
    with deployed(
        openyam_sim, OpenYamSimConnection, ControlCoordinator, ManipulationModule
    ) as stack:
        # The planner has the rig's joints once they have passed through the coordinator.
        planner = stack.modules[ManipulationModule]
        wait_until(lambda: planner.get_current_joints() is not None, timeout=20.0)
        yield stack


@pytest.mark.self_hosted
def test_planner_plans_and_the_rig_executes(planning_stack: Deployed) -> None:
    planner = planning_stack.modules[ManipulationModule]

    # A joint target.
    targets = dict(zip(ARM, [0.4, 0.9, 1.3, 0.2, -0.2, 0.4], strict=True))
    goal = JointState(name=list(targets), position=list(targets.values()))
    assert planner.plan_to_joints({"manipulator": goal}).status is PlanStatus.SUCCEEDED
    assert planner.execute(timeout=60.0).status is ExecutionStatus.COMPLETED
    wait_until(lambda: planning_stack.distance_from(targets) < 0.02, timeout=10.0)

    # A pose target: fingers down, 15 cm above the table, over the cup at
    # (0.33, 0.28) on the rig. The planner's world is the arm's base.
    x, y, z = OPENYAM_ROOT_POSITION
    above_cup = [0.33 - x, 0.28 - y, DUAL_OPENYAM_TABLE_TOP_Z - z + 0.15]
    # The fingers point along the tip frame's x: a quarter turn about y points them down.
    fingers_down = PoseStamped(
        frame_id="world", position=above_cup, orientation=[0.0, 0.70710678, 0.0, 0.70710678]
    )
    assert planner.plan_to_poses({"manipulator": fingers_down}).status is PlanStatus.SUCCEEDED
    assert planner.execute(timeout=60.0).status is ExecutionStatus.COMPLETED

    def reached() -> list[float]:
        pose = planner.get_ee_pose("manipulator")
        assert pose is not None
        return [pose.position.x, pose.position.y, pose.position.z]

    wait_until(lambda: reached() == pytest.approx(above_cup, abs=5e-3), timeout=10.0)

    # The planner's gripper command reaches the jaw.
    opened = OPENYAM.gripper_travel()["joint7"][1]
    assert planner.set_gripper_position(1.0).status is CommandStatus.SUCCEEDED
    wait_until(
        lambda: planning_stack.measured()["arm/joint7"] == pytest.approx(opened, abs=1e-3),
        timeout=10.0,
    )
