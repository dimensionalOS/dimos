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

"""Native OpenYAM data-collection builders."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.experimental.memory.rust_recorder import RustMcapStoreConfig, RustRecorder
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.imitation.cameras import CameraDevice, profile_cameras
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.profile import PolicyIOProfile
from dimos.robot.manipulators.openyam.blueprints.teleop import teleop_quest_openyam
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_JOINTS,
    openyam_hardware,
)
from dimos.robot.manipulators.openyam.learning import (
    OPENYAM_QUEST_IO,
    OPENYAM_TEACH_IO,
    OpenYamQuestRecorder,
    OpenYamTeachRecorder,
)


def _collection(
    *,
    recorder: type[RustRecorder],
    profile: PolicyIOProfile,
    recording: Path,
    task: str,
    cameras: dict[str, CameraDevice],
    robot: Blueprint,
) -> Blueprint:
    camera_blueprints, camera_remappings = profile_cameras(profile, cameras)
    return autoconnect(
        recorder.blueprint(
            store=RustMcapStoreConfig(path=str(recording)),
            record_tf=False,
        ),
        EpisodeMonitorModule.blueprint(task=task),
        robot,
        *camera_blueprints,
    ).remappings(camera_remappings)


OPENYAM_TEACH_DAMPING = (2.0, 2.0, 2.0, 0.5, 0.5, 0.5, 0.0)
_openyam_teach_hardware = openyam_hardware()
if _openyam_teach_hardware.adapter_type == "openyam_damiao":
    runtime_config = _openyam_teach_hardware.adapter_kwargs["runtime_config"]
    if not isinstance(runtime_config, DamiaoRuntimeConfig):
        raise TypeError("OpenYAM Damiao hardware requires DamiaoRuntimeConfig")
    _openyam_teach_hardware = replace(
        _openyam_teach_hardware,
        adapter_kwargs={
            **_openyam_teach_hardware.adapter_kwargs,
            "runtime_config": replace(runtime_config, passive_grippers=("gripper",)),
        },
    )
_openyam_teach_hardware = replace(
    _openyam_teach_hardware,
    wb_config=WholeBodyConfig(
        kp=(0.0,) * len(OPENYAM_JOINTS),
        kd=OPENYAM_TEACH_DAMPING,
    ),
)


def build_quest_collection(
    *, recording: Path, task: str, cameras: dict[str, CameraDevice]
) -> Blueprint:
    """Build one Quest-controlled OpenYAM collection session."""
    return _collection(
        recorder=OpenYamQuestRecorder,
        profile=OPENYAM_QUEST_IO,
        recording=recording,
        task=task,
        cameras=cameras,
        robot=teleop_quest_openyam,
    )


def build_teach_collection(
    *, recording: Path, task: str, cameras: dict[str, CameraDevice]
) -> Blueprint:
    """Build one gravity-compensated OpenYAM collection session."""
    robot = ControlCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_openyam_teach_hardware],
        tasks=[
            TaskConfig(
                name="teach_openyam",
                type="trajectory",
                joint_names=list(OPENYAM_JOINTS),
                priority=10,
                params={"hold_position_when_idle": True},
            ),
        ],
    )
    return _collection(
        recorder=OpenYamTeachRecorder,
        profile=OPENYAM_TEACH_IO,
        recording=recording,
        task=task,
        cameras=cameras,
        robot=robot,
    )
