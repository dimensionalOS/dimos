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

"""Quest collection on one Metal: the recorded joints and the deployed wiring."""

from pathlib import Path

import pytest

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import global_config
from dimos.imitation.collection.recorder import CollectionModule
from dimos.robot.manipulators.damiao.connection import DamiaoConnection
from dimos.robot.manipulators.makermods_metal.blueprints.learning_collection import (
    METAL_QUEST_PROFILE,
    metal_quest_collection,
)
from dimos.robot.manipulators.makermods_metal.blueprints.teleop import (
    METAL_QUEST_JOINTS,
    metal_quest_tasks,
    metal_teleop_control,
)

_ARM_JOINTS = [f"arm/joint{i}" for i in range(1, 7)]


def test_profile_records_seven_joints_and_the_wrist_camera() -> None:
    schema = METAL_QUEST_PROFILE.to_schema()
    image = schema.observation["observation.images.wrist"]
    state = schema.observation["observation.state"]
    action = schema.action["action"]

    assert METAL_QUEST_JOINTS == [*_ARM_JOINTS, "arm/joint7"]
    assert schema.robot_type == "makermods_metal"
    assert (image.stream, image.dtype, image.shape) == ("wrist_image", "video", (480, 640, 3))
    assert (state.stream, state.field, state.dtype, state.shape) == (
        "coordinator_joint_state",
        "position",
        "float32",
        (7,),
    )
    assert state.names == METAL_QUEST_JOINTS
    assert (action.stream, action.source_kind, action.names) == (
        "coordinator_joint_command",
        "joint_position_updates",
        METAL_QUEST_JOINTS,
    )


def test_collection_takes_recording_task_camera_and_can_port(tmp_path: Path) -> None:
    parsed = BlueprintConfigParser(metal_quest_collection).parse(
        [
            "--recorder.recording",
            str(tmp_path / "session"),
            "--episodes.task",
            "pick up the cube",
            "--wrist.hardware.camera-index",
            "/dev/video3",
            "--can-port",
            "test-can",
        ],
        environ={},
    )
    atoms = metal_quest_collection.active_blueprints
    [coordinator] = [atom for atom in atoms if issubclass(atom.module, ControlCoordinator)]
    teleop, gripper, trajectory = coordinator.kwargs["tasks"]

    assert issubclass(atoms[0].module, CollectionModule)
    assert parsed.module_configs["recorder"]["recording"] == tmp_path / "session"
    assert parsed.module_configs["wrist"]["hardware"]["camera_index"] == "/dev/video3"
    assert parsed.module_kwargs("metalconnection")["can_port"] == "test-can"
    assert (teleop.type, teleop.priority, teleop.joint_names) == ("teleop_ik", 20, _ARM_JOINTS)
    assert teleop.params["solver_type"] is PinkPoseTargetSolver
    assert teleop.params["bindings"] == [{"hand": "right", "target_frame": "gripper_base"}]
    assert (gripper.name, gripper.joint_names, gripper.params["open_at"]) == (
        "metal_gripper",
        ["arm/joint7"],
        "lower",
    )
    assert gripper.stream_bind == {"gripper_command": "right_gripper_command"}
    assert (trajectory.type, trajectory.priority, trajectory.joint_names) == (
        "trajectory",
        10,
        METAL_QUEST_JOINTS,
    )


def test_simulation_swaps_the_connection_for_the_echo_mock(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")

    blueprint = metal_teleop_control(metal_quest_tasks())

    assert not any(issubclass(atom.module, DamiaoConnection) for atom in blueprint.blueprints)
    [mock] = [atom for atom in blueprint.blueprints if atom.module is MockJointConnection]
    assert mock.kwargs["source"] == "arm"
    assert list(mock.kwargs["initial_positions"]) == [*[f"joint{i}" for i in range(1, 7)], "joint7"]
    assert mock.kwargs["initial_positions"]["joint7"] == 0.0
    assert blueprint.remapping_map == {
        (mock.name, "position_command"): "arm_position_command",
        ("ControlCoordinator", "arm_joint_state"): "joint_state",
    }
