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

"""OpenYAM hand-guided teaching over the direct connection."""

from types import SimpleNamespace

import numpy as np
import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionModule
from dimos.robot.manipulators.openyam.blueprints.learning_collection import (
    OPENYAM_TEACH_JOINTS,
    openyam_teach_collection,
)
from dimos.robot.manipulators.openyam.blueprints.learning_quest_collection import (
    openyam_quest_collection,
)
from dimos.robot.manipulators.openyam.connection import (
    OPENYAM_TEACH_DAMPING,
    OpenYamConnection,
    OpenYamTeachConnection,
)
from dimos.robot.manipulators.openyam.coordinator import OpenYamCoordinator
from dimos.robot.manipulators.openyam.joints import OPENYAM_ARM_JOINTS


@pytest.mark.parametrize("blueprint", [openyam_teach_collection, openyam_quest_collection])
def test_collection_uses_normal_module_configuration(blueprint, tmp_path):
    parsed = BlueprintConfigParser(blueprint).parse(
        [
            "--recorder.recording",
            str(tmp_path / "session"),
            "--episodes.task",
            "pick up the block",
            "--wrist.hardware.camera-index",
            "/dev/video3",
            "--can-port",
            "test-can",
        ],
        environ={},
    )
    recorder = next(atom for atom in blueprint.active_blueprints if atom.name == "recorder")
    assert recorder.kwargs["recording_schema"].robot_type == "openyam"
    assert parsed.module_configs["recorder"]["recording"] == tmp_path / "session"
    assert parsed.module_configs["wrist"]["hardware"]["camera_index"] == "/dev/video3"


def test_teach_collection_is_a_minimal_native_stack():
    modules = [atom.module for atom in openyam_teach_collection.active_blueprints]
    assert issubclass(modules[0], CollectionModule)
    assert modules[1:] == [
        EpisodeMonitorModule,
        OpenYamCoordinator,
        OpenYamTeachConnection,
        WebcamModule,
    ]


def test_teach_connection_is_zero_stiffness_gravity_compensated_and_passive():
    assert OpenYamTeachConnection.kp == {"arm": [0.0] * len(OPENYAM_ARM_JOINTS)}
    assert OpenYamTeachConnection.kd == {"arm": list(OPENYAM_TEACH_DAMPING)}
    assert OpenYamTeachConnection.gravity_comp is True
    assert OpenYamTeachConnection.passive_grippers == frozenset({"gripper"})
    assert OpenYamTeachConnection.zero_stiffness_joint_names() == [
        *OPENYAM_ARM_JOINTS,
        "gripper",
    ]
    # The position-holding connection is unchanged and declares nothing.
    assert OpenYamConnection.zero_stiffness_joint_names() == []


def test_teach_connection_declares_zero_stiffness_to_the_coordinator():
    coordinator = next(
        atom
        for atom in openyam_teach_collection.active_blueprints
        if atom.module is OpenYamCoordinator
    )
    assert coordinator.kwargs.get("hardware", []) == []
    assert coordinator.kwargs["zero_stiffness_joints"] == OPENYAM_TEACH_JOINTS
    assert [
        (task.name, task.type, task.joint_names, task.priority, task.auto_start, task.params)
        for task in coordinator.kwargs["tasks"]
    ] == [
        (
            "teach_openyam",
            "hand_guiding",
            OPENYAM_TEACH_JOINTS,
            10,
            True,
            {},
        ),
    ]


def test_teach_connection_is_selected_with_the_deployed_joint_names():
    atom = next(
        atom
        for atom in openyam_teach_collection.active_blueprints
        if atom.module is OpenYamTeachConnection
    )
    assert atom.kwargs["source"] == "arm"
    assert OPENYAM_TEACH_JOINTS == [f"arm/{name}" for name in OPENYAM_ARM_JOINTS] + ["arm/gripper"]


def test_teach_recording_profile_keeps_seven_joint_state_and_action():
    recorder = next(
        atom for atom in openyam_teach_collection.active_blueprints if atom.name == "recorder"
    )
    schema = recorder.kwargs["recording_schema"]

    assert schema.robot_type == "openyam"
    state = schema.observation["observation.state"]
    action = schema.action["action"]
    assert (state.stream, state.field, state.dtype, state.shape) == (
        "coordinator_joint_state",
        "position",
        "float32",
        (7,),
    )
    assert state.names == OPENYAM_TEACH_JOINTS
    assert (action.stream, action.names) == ("coordinator_joint_state", OPENYAM_TEACH_JOINTS)


def test_teach_connection_commands_zero_stiffness_on_every_target(mocker):
    gripper = mocker.Mock()
    gripper.motor = SimpleNamespace(position=0.42)
    connection = OpenYamTeachConnection(can_port="can0")
    arm = mocker.Mock()
    arm.positions.return_value = np.zeros(len(OPENYAM_ARM_JOINTS))
    connection._arms = {"arm": arm}
    connection._grippers = {"gripper": gripper}
    connection._physical = SimpleNamespace(gravity=lambda q: np.zeros(len(q)))
    connection._robot = mocker.Mock()
    connection._active = True
    connection._targets = {
        **{f"yam_joint{i}": 0.1 * i for i in range(1, 7)},
        "gripper": 0.9,
    }
    try:
        connection._send_targets()
    finally:
        connection.stop()

    rows = arm.mit_control.call_args.args[0]
    np.testing.assert_allclose(rows[:, 0], [0.0] * 6)
    np.testing.assert_allclose(rows[:, 1], OPENYAM_TEACH_DAMPING)
    np.testing.assert_allclose(rows[:, 2], [0.1 * i for i in range(1, 7)])
    np.testing.assert_allclose(rows[:, 3:], 0.0)
    gripper.mit_control.assert_called_once_with(0.0, 0.0, 0.42, 0.0, 0.0)
    gripper.set_opening.assert_not_called()


def test_teach_connection_enables_a_passive_gripper_at_zero_impedance(mocker):
    gripper = mocker.Mock()
    gripper.motor = SimpleNamespace(position=0.42)
    connection = OpenYamTeachConnection(can_port="can0")
    connection._grippers = {"gripper": gripper}
    connection._robot = mocker.Mock()
    try:
        connection._enable_passive_grippers()
        gripper.set_mode.assert_called_once_with("mit")
        gripper.enable.assert_called_once_with()
        gripper.mit_control.assert_called_once_with(0.0, 0.0, 0.42, 0.0, 0.0)
        connection._robot.tick.assert_called_once_with(1_000)
    finally:
        connection.stop()


def test_teach_collection_can_selector_reaches_the_connection(tmp_path):
    parsed = BlueprintConfigParser(openyam_teach_collection).parse(
        [
            "--recorder.recording",
            str(tmp_path / "session"),
            "--episodes.task",
            "pick",
            "--can-port",
            "/dev/ttyACM0",
        ],
        environ={},
    )
    assert parsed.module_kwargs("openyamteachconnection")["can_port"] == "/dev/ttyACM0"
    coordinator = next(
        atom
        for atom in openyam_teach_collection.active_blueprints
        if issubclass(atom.module, ControlCoordinator)
    )
    assert coordinator.module is OpenYamCoordinator
