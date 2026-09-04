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

from pathlib import Path

from dimos.control.coordinator import ControlCoordinator
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.native_recorder import NativeCollectionRecorder
from dimos.robot.manipulators.openyam.blueprints.learning_collection import (
    build_quest_collection,
    build_teach_collection,
)
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS


def test_openyam_collection_uses_the_native_recorder(tmp_path: Path) -> None:
    blueprint = build_quest_collection(
        recording=tmp_path / "session.mcap", task="pick up the block", camera_device=3
    )

    recorder = blueprint.active_blueprints[0]
    monitor = next(
        atom for atom in blueprint.active_blueprints if atom.module is EpisodeMonitorModule
    )
    camera = next(atom for atom in blueprint.active_blueprints if atom.module is CameraModule)
    assert recorder.module is NativeCollectionRecorder
    assert recorder.kwargs["store"].path == str(tmp_path / "session.mcap")
    assert recorder.kwargs["record_tf"] is False
    assert monitor.kwargs["task"] == "pick up the block"
    assert camera.kwargs["hardware"].camera_index == 3


def test_openyam_teach_collection_is_a_minimal_native_stack(tmp_path: Path) -> None:
    blueprint = build_teach_collection(recording=tmp_path / "session.mcap", task="place cup")
    modules = [atom.module for atom in blueprint.active_blueprints]

    assert modules == [
        NativeCollectionRecorder,
        EpisodeMonitorModule,
        ControlCoordinator,
        CameraModule,
    ]


def test_openyam_teach_collection_uses_gravity_compensation_and_zero_stiffness(
    tmp_path: Path,
) -> None:
    blueprint = build_teach_collection(recording=tmp_path / "session.mcap", task="place cup")
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
            "trajectory",
            OPENYAM_JOINTS,
            10,
            {"hold_position_when_idle": True},
        ),
    ]
