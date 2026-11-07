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

"""Quest teleoperation of two Metals: routing, handedness and left/right isolation.

The behavior test drives the real ``TeleopIKTask`` and coordinator over the real
``DualMetalConnection`` path replaced by ``MockJointConnection``: no CAN bus, no
vendor SDK, and no physical arm are opened.
"""

from collections.abc import Iterator
from pathlib import Path
import tempfile

import numpy as np
import pytest

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprints import BlueprintAtom
from dimos.core.global_config import global_config
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.manipulators.damiao._damiao_test_helpers import (
    MockConnectionRig,
    mock_connection_rig,
)
from dimos.robot.manipulators.dual_makermods_metal.blueprints.teleop import (
    DUAL_METAL_QUEST_JOINTS,
    build_dual_metal_webxr,
    teleop_webxr_dual_metal,
)
from dimos.robot.manipulators.dual_makermods_metal.config import (
    ARM_JOINTS,
    FINGER_JOINTS,
    SIDES,
)
from dimos.robot.manipulators.dual_makermods_metal.connection import DualMetalConnection
from dimos.teleop.webxr.controller_types import Buttons

#: Where each Quest controller sits when the operator starts moving.
_LEFT_REST = PoseStamped(
    frame_id="world",
    position=Vector3(0.0, 0.30, 0.30),
    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
)
_RIGHT_REST = PoseStamped(
    frame_id="world",
    position=Vector3(0.0, -0.30, 0.30),
    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
)


def _moved(pose: PoseStamped, y: float = 0.0, z: float = 0.0) -> PoseStamped:
    return PoseStamped(
        frame_id=pose.frame_id,
        position=Vector3(
            pose.position.x,
            pose.position.y + y,
            pose.position.z + z,
        ),
        orientation=pose.orientation,
    )


def _grips(*, left: bool, right: bool) -> Buttons:
    buttons = Buttons()
    buttons.left_grip = left
    buttons.right_grip = right
    return buttons


def _coordinator_atom() -> BlueprintAtom:
    [atom] = [
        atom
        for atom in teleop_webxr_dual_metal.blueprints
        if issubclass(atom.module, ControlCoordinator)
    ]
    return atom


@pytest.fixture
def rig(mocker) -> Iterator[MockConnectionRig]:
    """The blueprint's own coordinator over a mock pair of Metals, ticked by hand."""
    mocker.patch("dimos.control.tick_loop.TickLoop.start")
    atom = _coordinator_atom()
    coordinator: ControlCoordinator = atom.module(**atom.kwargs)
    with tempfile.TemporaryDirectory() as directory:
        with mock_connection_rig(
            DualMetalConnection, coordinator, tmp_path=Path(directory)
        ) as connection_rig:
            yield connection_rig


def _arm_positions(rig: MockConnectionRig, side: str) -> np.ndarray:
    return np.array([rig.positions[f"arm/{name}"] for name in ARM_JOINTS[side]])


def test_blueprint_routes_each_controller_to_its_own_arm_and_jaw() -> None:
    assert [atom.name for atom in teleop_webxr_dual_metal.active_blueprints] == [
        "armteleopmodule",
        "ControlCoordinator",
        "dualmetalconnection",
        "dualmetalmanipulationmodule",
    ]
    assert teleop_webxr_dual_metal.remapping_map == {
        ("armteleopmodule", "left_controller_output"): "left_cartesian_command",
        ("armteleopmodule", "left_gripper_command"): "left_gripper_command",
        ("armteleopmodule", "right_controller_output"): "right_cartesian_command",
        ("armteleopmodule", "right_gripper_command"): "right_gripper_command",
        ("dualmetalconnection", "position_command"): "arm_position_command",
        ("ControlCoordinator", "arm_joint_state"): "joint_state",
    }

    tasks = _coordinator_atom().kwargs["tasks"]
    assert [(task.name, task.type, task.priority) for task in tasks] == [
        ("teleop_dual_metal_left", "teleop_ik", 20),
        ("teleop_dual_metal_right", "teleop_ik", 20),
        ("left_arm_gripper", "gripper", 20),
        ("right_arm_gripper", "gripper", 20),
        ("joint_trajectory", "trajectory", 10),
    ]
    left_ik, right_ik, left_jaw, right_jaw, trajectory = tasks

    # Each hand owns one arm, and neither task lists the other's joints.
    for side, task in (("left", left_ik), ("right", right_ik)):
        assert task.joint_names == [f"arm/{name}" for name in ARM_JOINTS[side]]
        assert task.params["bindings"] == [{"hand": side, "target_frame": f"{side}_gripper_base"}]
    assert not set(left_ik.joint_names) & set(right_ik.joint_names)

    # Each jaw runs on its own task and its own controller trigger.
    for side, task in (("left", left_jaw), ("right", right_jaw)):
        assert task.joint_names == [f"arm/{FINGER_JOINTS[side][0]}"]
        assert task.stream_bind == {"gripper_command": f"{side}_gripper_command"}
        assert task.params["open_at"] == "lower"
    assert not set(left_jaw.joint_names) & set(right_jaw.joint_names)

    # Recordings see each arm then its jaw, matching the deployed order.
    assert trajectory.joint_names == DUAL_METAL_QUEST_JOINTS


def test_simulation_swaps_the_can_connection_for_the_echo_mock(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "mock")
    simulated = build_dual_metal_webxr()

    assert "dualmetalconnection" not in [atom.name for atom in simulated.active_blueprints]
    [connection] = [atom for atom in simulated.blueprints if atom.module is MockJointConnection]
    assert connection.kwargs["source"] == "arm"
    # The mock names joints as the robot model does, without the coordinator's
    # "arm/" prefix: the connection strips it on the way in and re-adds it out.
    assert connection.kwargs["initial_positions"] == {
        name: 0.0 for side in SIDES for name in (*ARM_JOINTS[side], FINGER_JOINTS[side][0])
    }
    assert simulated.remapping_map[("mockjointconnection", "position_command")] == (
        "arm_position_command"
    )


def test_one_grip_moves_only_its_own_arm(rig: MockConnectionRig) -> None:
    # Park both controllers at rest, then engage only the left grip. Each arm's
    # task owns its own deadman, so the right arm must never be captured.
    rig.inputs["left_cartesian_command"].publish(_LEFT_REST)
    rig.inputs["right_cartesian_command"].publish(_RIGHT_REST)
    rig.inputs["teleop_buttons"].publish(_grips(left=True, right=False))
    rig.tick()
    rig.inputs["left_cartesian_command"].publish(_LEFT_REST)
    rig.inputs["right_cartesian_command"].publish(_RIGHT_REST)
    rig.tick()
    captured_left = _arm_positions(rig, "left")
    captured_right = _arm_positions(rig, "right")

    for _ in range(10):
        rig.inputs["left_cartesian_command"].publish(_moved(_LEFT_REST, y=0.04, z=0.03))
        # The right controller moves too, but its grip stays open.
        rig.inputs["right_cartesian_command"].publish(_moved(_RIGHT_REST, y=-0.10, z=0.10))
        rig.tick()

    left = _arm_positions(rig, "left")
    right = _arm_positions(rig, "right")
    assert np.abs(left - captured_left).max() > 0.05
    assert np.allclose(right, captured_right, atol=1e-9)
    # Neither jaw moves without its trigger.
    assert rig.positions["arm/left_joint7"] == pytest.approx(0.0)
    assert rig.positions["arm/right_joint7"] == pytest.approx(0.0)

    # Releasing the left grip stops the left arm as its controller keeps moving.
    rig.inputs["teleop_buttons"].publish(_grips(left=False, right=False))
    rig.tick()
    released = _arm_positions(rig, "left")
    for _ in range(5):
        rig.inputs["left_cartesian_command"].publish(_moved(_LEFT_REST, y=0.10, z=-0.05))
        rig.tick()
    assert np.allclose(_arm_positions(rig, "left"), released, atol=1e-9)
