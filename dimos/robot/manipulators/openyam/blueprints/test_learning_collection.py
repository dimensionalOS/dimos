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

from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.native_recorder import NativeCollectionRecorder
from dimos.robot.manipulators.openyam.blueprints.learning_collection import (
    learning_collect_quest_openyam,
    learning_collect_teach_openyam,
)
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS


def test_openyam_collection_uses_the_native_recorder() -> None:
    assert learning_collect_quest_openyam.active_blueprints[0].module is NativeCollectionRecorder


def test_openyam_collection_uses_the_selected_global_transport() -> None:
    assert learning_collect_quest_openyam.global_config_overrides == {}
    assert learning_collect_quest_openyam.transport_map == {}
    assert learning_collect_quest_openyam.requirement_checks == ()


def test_native_openyam_paths_are_configurable_from_cli() -> None:
    parsed = BlueprintConfigParser(learning_collect_quest_openyam).parse(
        [
            "--nativecollectionrecorder.store.path",
            "/tmp/native-openyam.mcap",
            "--WristCamera.hardware.camera-index",
            "/dev/v4l/by-id/usb-wrist-camera",
            "--task",
            "pick up the red block",
        ],
        environ={},
    )

    assert parsed.module_kwargs("nativecollectionrecorder")["store"]["path"] == (
        "/tmp/native-openyam.mcap"
    )
    assert parsed.module_kwargs("WristCamera")["hardware"]["camera_index"] == (
        "/dev/v4l/by-id/usb-wrist-camera"
    )
    assert parsed.module_kwargs("episodemonitormodule")["task"] == "pick up the red block"
    assert learning_collect_quest_openyam.active_blueprints[0].kwargs["record_tf"] is False


def test_openyam_teach_collection_is_a_minimal_native_stack() -> None:
    modules = [atom.module for atom in learning_collect_teach_openyam.active_blueprints]

    assert modules == [
        NativeCollectionRecorder,
        EpisodeMonitorModule,
        ControlCoordinator,
        CameraModule,
    ]


def test_openyam_teach_collection_uses_gravity_compensation_and_zero_stiffness() -> None:
    coordinator = next(
        atom
        for atom in learning_collect_teach_openyam.active_blueprints
        if atom.module is ControlCoordinator
    )
    hardware = coordinator.kwargs["hardware"][0]
    assert hardware.joints == OPENYAM_JOINTS
    assert hardware.wb_config is not None
    assert hardware.wb_config.kp == (0.0,) * len(OPENYAM_JOINTS)
    assert hardware.wb_config.kd == (5.0, 5.0, 5.0, 1.5, 1.5, 1.5, 0.0)
    if hardware.adapter_type == "openyam_damiao":
        assert hardware.adapter_kwargs["runtime_config"].gravity_comp is True

    tasks = coordinator.kwargs["tasks"]
    assert [(task.name, task.type, task.joint_names, task.priority) for task in tasks] == [
        ("teach_openyam", "teach", OPENYAM_JOINTS, 10),
        ("arm_gripper", "gripper", [OPENYAM_JOINTS[-1]], 20),
    ]


def test_native_openyam_teach_paths_are_configurable_from_cli() -> None:
    parsed = BlueprintConfigParser(learning_collect_teach_openyam).parse(
        [
            "--nativecollectionrecorder.store.path",
            "/tmp/native-openyam-teach.mcap",
            "--WristCamera.hardware.camera-index",
            "/dev/v4l/by-id/usb-wrist-camera",
            "--task",
            "place the cup",
        ],
        environ={},
    )

    assert parsed.module_kwargs("nativecollectionrecorder")["store"]["path"] == (
        "/tmp/native-openyam-teach.mcap"
    )
    assert parsed.module_kwargs("WristCamera")["hardware"]["camera_index"] == (
        "/dev/v4l/by-id/usb-wrist-camera"
    )
    assert parsed.module_kwargs("episodemonitormodule")["task"] == "place the cup"
