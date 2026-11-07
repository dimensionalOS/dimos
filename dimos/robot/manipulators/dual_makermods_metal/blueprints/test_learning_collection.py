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

"""Quest collection on two Metals: the recorded joints and the deployed wiring."""

from pathlib import Path

import pytest

from dimos.control.connection.mock_joint_connection import MockJointConnection
from dimos.control.coordinator import ControlCoordinator
from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.core.global_config import global_config
from dimos.imitation.collection.recorder import CollectionModule
from dimos.robot.manipulators.damiao.connection import DamiaoConnection
from dimos.robot.manipulators.dual_makermods_metal.blueprints.learning_collection import (
    DUAL_METAL_QUEST_PROFILE,
    dual_metal_quest_collection,
)
from dimos.robot.manipulators.dual_makermods_metal.blueprints.teleop import (
    DUAL_METAL_QUEST_JOINTS,
    dual_metal_quest_tasks,
    dual_metal_teleop_control,
)

_LEFT_ARM = [f"arm/left_joint{i}" for i in range(1, 7)]
_RIGHT_ARM = [f"arm/right_joint{i}" for i in range(1, 7)]


def test_profile_records_fourteen_joints_and_both_wrist_cameras() -> None:
    schema = DUAL_METAL_QUEST_PROFILE.to_schema()
    left = schema.observation["observation.images.left_wrist"]
    right = schema.observation["observation.images.right_wrist"]
    state = schema.observation["observation.state"]
    action = schema.action["action"]

    assert DUAL_METAL_QUEST_JOINTS == [
        *_LEFT_ARM,
        "arm/left_joint7",
        *_RIGHT_ARM,
        "arm/right_joint7",
    ]
    assert schema.robot_type == "dual_makermods_metal"
    assert (left.stream, left.dtype, left.shape) == ("left_wrist_image", "video", (480, 640, 3))
    assert (right.stream, right.dtype, right.shape) == (
        "right_wrist_image",
        "video",
        (480, 640, 3),
    )
    assert (state.stream, state.field, state.dtype, state.shape) == (
        "coordinator_joint_state",
        "position",
        "float32",
        (14,),
    )
    assert state.names == DUAL_METAL_QUEST_JOINTS
    assert (action.stream, action.source_kind, action.names) == (
        "coordinator_joint_command",
        "joint_position_updates",
        DUAL_METAL_QUEST_JOINTS,
    )


def test_collection_takes_recording_task_cameras_and_can_ports(tmp_path: Path) -> None:
    parsed = BlueprintConfigParser(dual_metal_quest_collection).parse(
        [
            "--recorder.recording",
            str(tmp_path / "session"),
            "--episodes.task",
            "hand over the cube",
            "--left-wrist.hardware.camera-index",
            "/dev/video3",
            "--right-wrist.hardware.camera-index",
            "/dev/video5",
            "--left-can-port",
            "left-can",
            "--right-can-port",
            "right-can",
        ],
        environ={},
    )
    atoms = dual_metal_quest_collection.active_blueprints
    [coordinator] = [atom for atom in atoms if issubclass(atom.module, ControlCoordinator)]
    left_teleop, right_teleop, left_gripper, right_gripper, trajectory = coordinator.kwargs["tasks"]
    connection = parsed.module_kwargs("dualmetalconnection")

    assert issubclass(atoms[0].module, CollectionModule)
    assert parsed.module_configs["recorder"]["recording"] == tmp_path / "session"
    assert parsed.module_configs["left_wrist"]["hardware"]["camera_index"] == "/dev/video3"
    assert parsed.module_configs["right_wrist"]["hardware"]["camera_index"] == "/dev/video5"
    assert (connection["left_can_port"], connection["right_can_port"]) == ("left-can", "right-can")
    assert (left_teleop.type, left_teleop.priority, left_teleop.joint_names) == (
        "teleop_ik",
        20,
        _LEFT_ARM,
    )
    assert right_teleop.joint_names == _RIGHT_ARM
    assert left_teleop.params["solver_type"] is PinkPoseTargetSolver
    assert left_teleop.params["bindings"] == [{"hand": "left", "target_frame": "left_gripper_base"}]
    assert right_teleop.params["bindings"] == [
        {"hand": "right", "target_frame": "right_gripper_base"}
    ]
    assert (left_gripper.name, left_gripper.joint_names, left_gripper.params["open_at"]) == (
        "left_arm_gripper",
        ["arm/left_joint7"],
        "lower",
    )
    assert (right_gripper.name, right_gripper.joint_names) == (
        "right_arm_gripper",
        ["arm/right_joint7"],
    )
    assert left_gripper.stream_bind == {"gripper_command": "left_gripper_command"}
    assert right_gripper.stream_bind == {"gripper_command": "right_gripper_command"}
    assert (trajectory.type, trajectory.priority, trajectory.joint_names) == (
        "trajectory",
        10,
        DUAL_METAL_QUEST_JOINTS,
    )


def test_simulation_swaps_the_connection_for_the_echo_mock(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "simulation", "mujoco")

    blueprint = dual_metal_teleop_control(dual_metal_quest_tasks())

    assert not any(issubclass(atom.module, DamiaoConnection) for atom in blueprint.blueprints)
    [mock] = [atom for atom in blueprint.blueprints if atom.module is MockJointConnection]
    assert mock.kwargs["source"] == "arm"
    assert len(mock.kwargs["initial_positions"]) == 14
    assert mock.kwargs["initial_positions"]["left_joint7"] == 0.0
    assert mock.kwargs["initial_positions"]["right_joint7"] == 0.0
    assert blueprint.remapping_map == {
        (mock.name, "position_command"): "arm_position_command",
        ("ControlCoordinator", "arm_joint_state"): "joint_state",
    }
