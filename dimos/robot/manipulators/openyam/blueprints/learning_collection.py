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

"""Native OpenYAM data collection."""

from __future__ import annotations

from dataclasses import replace
from datetime import datetime

from dimos.constants import STATE_DIR
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.experimental.memory.rust_recorder import RustMcapStoreConfig
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.native_recorder import NativeCollectionRecorder
from dimos.robot.manipulators.openyam.blueprints.teleop import teleop_quest_openyam
from dimos.robot.manipulators.openyam.config import (
    OPENYAM_GRIPPER_JOINT,
    OPENYAM_JOINTS,
    openyam_hardware,
)
from dimos.robot.manipulators.openyam.learning import OPENYAM_LEARNING_PROFILE


def _session_mcap() -> str:
    return str(STATE_DIR / "recordings" / f"session_openyam_{datetime.now():%Y%m%d_%H%M%S}.mcap")


def _teach_session_mcap() -> str:
    return str(
        STATE_DIR / "recordings" / f"session_openyam_teach_{datetime.now():%Y%m%d_%H%M%S}.mcap"
    )


def _wrist_camera() -> Blueprint:
    return CameraModule.blueprint(
        instance_name="WristCamera",
        hardware=WebcamConfig(
            camera_index=0,
            width=OPENYAM_LEARNING_PROFILE.camera_width,
            height=OPENYAM_LEARNING_PROFILE.camera_height,
            fps=OPENYAM_LEARNING_PROFILE.fps,
            frame_id_prefix=OPENYAM_LEARNING_PROFILE.camera_frame_prefix,
        ),
        frame_id=OPENYAM_LEARNING_PROFILE.camera_frame_id,
    )


learning_collect_quest_openyam = autoconnect(
    NativeCollectionRecorder.blueprint(
        store=RustMcapStoreConfig(path=_session_mcap()),
        record_tf=False,
    ),
    EpisodeMonitorModule.blueprint(),
    teleop_quest_openyam,
    _wrist_camera(),
)


OPENYAM_TEACH_DAMPING = (5.0, 5.0, 5.0, 1.5, 1.5, 1.5, 0.0)
_openyam_teach_hardware = replace(
    openyam_hardware(),
    wb_config=WholeBodyConfig(
        kp=(0.0,) * len(OPENYAM_JOINTS),
        kd=OPENYAM_TEACH_DAMPING,
    ),
)

learning_collect_teach_openyam = autoconnect(
    NativeCollectionRecorder.blueprint(
        store=RustMcapStoreConfig(path=_teach_session_mcap()),
        record_tf=False,
    ),
    EpisodeMonitorModule.blueprint(),
    ControlCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[_openyam_teach_hardware],
        tasks=[
            TaskConfig(
                name="teach_openyam",
                type="teach",
                joint_names=list(OPENYAM_JOINTS),
                priority=10,
            ),
            TaskConfig(
                name="arm_gripper",
                type="gripper",
                joint_names=[OPENYAM_GRIPPER_JOINT],
                priority=20,
            ),
        ],
    ),
    _wrist_camera(),
)
