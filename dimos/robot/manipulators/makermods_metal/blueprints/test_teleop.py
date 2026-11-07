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

"""Quest teleoperation of one Metal: what the blueprint wires and what the arm does.

The behavior tests drive the real ``TeleopIKTask`` and the real coordinator over
the real ``MetalConnection`` path replaced by ``MockJointConnection``: no CAN
bus, no vendor SDK, and no physical arm are opened.
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
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators.damiao._damiao_test_helpers import (
    MockConnectionRig,
    mock_connection_rig,
)
from dimos.robot.manipulators.makermods_metal.blueprints.teleop import (
    METAL_QUEST_JOINTS,
    METAL_QUEST_MODEL,
    build_metal_webxr,
    teleop_webxr_metal,
)
from dimos.robot.manipulators.makermods_metal.connection import MetalConnection
from dimos.teleop.webxr.controller_types import Buttons
from dimos.teleop.webxr.extensions import ArmTeleopModule

#: Where the Quest right controller sits when the operator starts moving.
_REST_POSE = PoseStamped(
    frame_id="world",
    position=Vector3(0.30, 0.0, 0.30),
    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
)


def _quest_pose(y: float = 0.0, z: float = 0.0) -> PoseStamped:
    """The right controller, nudged from ``_REST_POSE`` by dy and dz metres."""
    return PoseStamped(
        frame_id="world",
        position=Vector3(0.30, y, 0.30 + z),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )


def _grip(held: bool) -> Buttons:
    buttons = Buttons()
    buttons.right_grip = held
    return buttons


def _coordinator_atom() -> BlueprintAtom:
    [atom] = [
        atom
        for atom in teleop_webxr_metal.blueprints
        if issubclass(atom.module, ControlCoordinator)
    ]
    return atom


@pytest.fixture
def rig(mocker) -> Iterator[MockConnectionRig]:
    """The blueprint's own coordinator over a mock Metal, ticked by hand."""
    mocker.patch("dimos.control.tick_loop.TickLoop.start")
    atom = _coordinator_atom()
    coordinator: ControlCoordinator = atom.module(**atom.kwargs)
    with tempfile.TemporaryDirectory() as directory:
        with mock_connection_rig(
            MetalConnection, coordinator, tmp_path=Path(directory)
        ) as connection_rig:
            yield connection_rig


def _arm_positions(rig: MockConnectionRig) -> np.ndarray:
    return np.array([rig.positions[f"arm/joint{index}"] for index in range(1, 7)])


def test_blueprint_wires_one_right_hand_arm_and_a_separate_jaw() -> None:
    assert [atom.name for atom in teleop_webxr_metal.active_blueprints] == [
        "armteleopmodule",
        "ControlCoordinator",
        "metalconnection",
        "manipulationmodule",
    ]
    assert teleop_webxr_metal.remapping_map == {
        ("armteleopmodule", "right_controller_output"): "right_cartesian_command",
        ("metalconnection", "position_command"): "arm_position_command",
        ("ControlCoordinator", "arm_joint_state"): "joint_state",
    }

    tasks = _coordinator_atom().kwargs["tasks"]
    assert [(task.name, task.type, task.priority) for task in tasks] == [
        ("teleop_metal", "teleop_ik", 20),
        ("metal_gripper", "gripper", 20),
        ("joint_trajectory", "trajectory", 10),
    ]
    quest_ik, jaw, trajectory = tasks
    # The six arm joints are the IK task's; the jaw moves on its own task.
    assert quest_ik.joint_names == METAL_QUEST_MODEL.joint_names
    assert jaw.joint_names == ["arm/joint7"]
    assert jaw.stream_bind == {"gripper_command": "right_gripper_command"}
    assert jaw.params["open_at"] == "lower"
    # Recordings and rollouts see the deployed order: six arm joints, then the jaw.
    assert trajectory.joint_names == METAL_QUEST_JOINTS
    assert METAL_QUEST_JOINTS == [*METAL_QUEST_MODEL.joint_names, "arm/joint7"]
    # The URDF has no fingertip frame, so the planner tips at the jaw mount.
    assert quest_ik.params["bindings"] == [{"hand": "right", "target_frame": "gripper_base"}]


def test_quest_input_takes_the_right_hand_and_x_returns_to_zero() -> None:
    [input_atom] = [
        atom for atom in teleop_webxr_metal.blueprints if atom.module is ArmTeleopModule
    ]
    assert input_atom.kwargs["zero_button"] == "X"
    [planner] = [
        atom for atom in teleop_webxr_metal.blueprints if atom.name == "manipulationmodule"
    ]
    assert planner.kwargs["gripper_tasks"] == {"arm": "metal_gripper"}


def test_simulation_swaps_the_can_connection_for_the_echo_mock(monkeypatch) -> None:
    monkeypatch.setattr(global_config, "simulation", "mock")
    simulated = build_metal_webxr()

    assert "metalconnection" not in [atom.name for atom in simulated.active_blueprints]
    [connection] = [atom for atom in simulated.blueprints if atom.module is MockJointConnection]
    assert connection.kwargs["source"] == "arm"
    # The mock names joints as the robot model does, without the coordinator's
    # "arm/" prefix: the connection applies that prefix on its way in.
    assert connection.kwargs["initial_positions"] == {f"joint{index}": 0.0 for index in range(1, 8)}
    # The remap names the mock, so the coordinator still drives its position stream.
    assert simulated.remapping_map[("mockjointconnection", "position_command")] == (
        "arm_position_command"
    )


def test_the_arm_holds_still_until_the_grip_is_held(rig: MockConnectionRig) -> None:
    start = _arm_positions(rig)

    rig.inputs["teleop_buttons"].publish(_grip(False))
    for _ in range(5):
        rig.inputs["right_cartesian_command"].publish(_quest_pose())
        rig.tick()

    assert np.allclose(_arm_positions(rig), start, atol=1e-9)
    assert rig.positions["arm/joint7"] == pytest.approx(0.0)


def test_holding_the_grip_tracks_the_controller_and_releasing_stops(rig: MockConnectionRig) -> None:
    # Engagement clears any pose seen before the grip, so the reference is
    # captured on the first tick after it from the pose the operator holds.
    # Publish that pose again and tick, rather than race the capture.
    rig.inputs["right_cartesian_command"].publish(_REST_POSE)
    rig.inputs["teleop_buttons"].publish(_grip(True))
    rig.tick()
    rig.inputs["right_cartesian_command"].publish(_REST_POSE)
    rig.tick()
    engaged = _arm_positions(rig)

    for _ in range(10):
        rig.inputs["right_cartesian_command"].publish(_quest_pose(y=0.04, z=0.03))
        rig.tick()
    tracked = _arm_positions(rig)

    assert np.abs(tracked - engaged).max() > 0.05
    assert rig.positions["arm/joint7"] == pytest.approx(0.0)

    # Releasing the deadman stops the arm even as the controller keeps moving.
    rig.inputs["teleop_buttons"].publish(_grip(False))
    rig.tick()
    released = _arm_positions(rig)
    for _ in range(5):
        rig.inputs["right_cartesian_command"].publish(_quest_pose(y=-0.10, z=0.10))
        rig.tick()

    assert np.allclose(_arm_positions(rig), released, atol=1e-9)


def test_engaging_again_recaptures_the_reference_without_a_jump(rig: MockConnectionRig) -> None:
    rig.inputs["right_cartesian_command"].publish(_REST_POSE)
    rig.inputs["teleop_buttons"].publish(_grip(True))
    rig.tick()
    for _ in range(5):
        rig.inputs["right_cartesian_command"].publish(_quest_pose(y=0.04))
        rig.tick()
    rig.inputs["teleop_buttons"].publish(_grip(False))
    rig.tick()
    parked = _arm_positions(rig)

    # Engagement clears the previous reference; the capture happens on the
    # following tick, from the pose the operator is holding now. If it fell
    # back to the stale reference the arm would jump by the whole offset.
    rig.inputs["right_cartesian_command"].publish(_quest_pose(y=-0.20, z=0.15))
    rig.inputs["teleop_buttons"].publish(_grip(True))
    rig.tick()
    rig.inputs["right_cartesian_command"].publish(_quest_pose(y=-0.20, z=0.15))
    rig.tick()

    # The capture pose is the current arm pose, so IK may settle within a
    # milliradian; a stale reference would swing the arm by the 0.25 m offset.
    assert np.allclose(_arm_positions(rig), parked, atol=1e-3)


def test_the_trigger_moves_only_the_jaw(rig: MockConnectionRig) -> None:
    rig.inputs["teleop_buttons"].publish(_grip(False))
    rig.tick()
    arm_before = _arm_positions(rig)

    # joint7 is fully open at its lower end, -0.05 m, and closed at 0.
    for normalized, expected in ((1.0, -0.05), (0.5, -0.025), (0.0, 0.0)):
        rig.inputs["right_gripper_command"].publish(Float32(data=normalized))
        for _ in range(3):
            rig.tick()
        assert rig.positions["arm/joint7"] == pytest.approx(expected, abs=1e-4)
        assert np.allclose(_arm_positions(rig), arm_before, atol=1e-9)
