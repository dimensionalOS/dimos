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

from threading import Lock

import pytest

from dimos.control.coordinator import ControlCoordinator
from dimos.control.hardware_interface import ConnectedWholeBody
from dimos.control.tasks.registry import control_task_registry
from dimos.control.tick_loop import TickLoop
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.whole_body.spec import IMUState, MotorState, WholeBodyAdapter
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.robot.manipulators.openyam.blueprints.learning_collection import (
    openyam_teach_collection,
)
from dimos.robot.manipulators.openyam.blueprints.learning_quest_collection import (
    openyam_quest_collection,
)
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS


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
        ],
        environ={},
    )
    recorder = next(atom for atom in blueprint.active_blueprints if atom.name == "recorder")
    assert recorder.kwargs["recording_schema"].robot_type == "openyam"
    assert parsed.module_configs["recorder"]["recording"] == tmp_path / "session"
    assert parsed.module_configs["wrist"]["hardware"]["camera_index"] == "/dev/video3"


def test_teach_collection_is_a_minimal_native_stack():
    modules = [atom.module for atom in openyam_teach_collection.active_blueprints]
    assert modules[:2] == [ControlCoordinator, CameraModule]
    assert modules[-1] is EpisodeMonitorModule
    assert len(modules) == 4


def test_teach_collection_sends_zero_stiffness_commands_on_every_tick(mocker):
    coordinator = next(
        atom
        for atom in openyam_teach_collection.active_blueprints
        if atom.module is ControlCoordinator
    )
    component = coordinator.kwargs["hardware"][0]
    adapter = mocker.Mock(spec=WholeBodyAdapter)
    adapter.has_motor_states.return_value = True
    adapter.read_imu.return_value = IMUState()
    adapter.write_motor_commands.return_value = True
    connected = ConnectedWholeBody(adapter, component)
    hardware = {component.hardware_id: connected}
    config = coordinator.kwargs["tasks"][0]
    task = control_task_registry.create(config.type, config, hardware=hardware)
    if config.auto_start:
        task.start()
    loop = TickLoop(
        tick_rate=100.0,
        hardware=hardware,
        hardware_lock=Lock(),
        tasks={task.name: task},
        task_lock=Lock(),
        joint_to_hardware=dict.fromkeys(OPENYAM_JOINTS, component.hardware_id),
    )

    for position in (0.1, 0.2):
        adapter.read_motor_states.return_value = [MotorState(q=position)] * len(OPENYAM_JOINTS)
        loop._tick()

    assert adapter.write_motor_commands.call_count == 2
    for call, position in zip(adapter.write_motor_commands.call_args_list, (0.1, 0.2), strict=True):
        commands = call.args[0]
        assert [command.q for command in commands] == [position] * len(OPENYAM_JOINTS)
        assert [command.kp for command in commands] == [0.0] * len(OPENYAM_JOINTS)
        assert [command.kd for command in commands] == [2.0, 2.0, 2.0, 0.5, 0.5, 0.5, 0.0]
        assert [command.dq for command in commands] == [0.0] * len(OPENYAM_JOINTS)


def test_openyam_teach_collection_uses_gravity_compensation_and_zero_stiffness(
    tmp_path,
) -> None:
    blueprint = openyam_teach_collection
    coordinator = next(
        atom for atom in blueprint.active_blueprints if atom.module is ControlCoordinator
    )
    hardware = coordinator.kwargs["hardware"][0]
    assert hardware.joints == OPENYAM_JOINTS
    assert hardware.wb_config is not None
    assert hardware.wb_config.kp == (0.0,) * len(OPENYAM_JOINTS)
    assert hardware.wb_config.kd == (2.0, 2.0, 2.0, 0.5, 0.5, 0.5, 0.0)
    if hardware.adapter_type == "openyam_damiao":
        assert hardware.adapter_kwargs["runtime_config"].gravity_comp is True
        assert hardware.adapter_kwargs["runtime_config"].passive_grippers == ("gripper",)

    tasks = coordinator.kwargs["tasks"]
    assert [
        (task.name, task.type, task.joint_names, task.priority, task.params) for task in tasks
    ] == [
        (
            "teach_openyam",
            "hand_guiding",
            OPENYAM_JOINTS,
            10,
            {},
        ),
    ]
