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

"""Gravity-compensated OpenYAM collection, configured through dimos run."""

from dataclasses import replace

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.native_recorder import collection_recorder
from dimos.robot.manipulators.openyam.collection import OPENYAM_TEACH_COLLECTION
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS, openyam_hardware

OPENYAM_TEACH_DAMPING = (2.0, 2.0, 2.0, 0.5, 0.5, 0.5, 0.0)


def _teach_robot() -> Blueprint:
    hardware = openyam_hardware()
    if hardware.adapter_type == "openyam_damiao":
        runtime_config = hardware.adapter_kwargs["runtime_config"]
        if not isinstance(runtime_config, DamiaoRuntimeConfig):
            raise TypeError("OpenYAM Damiao hardware requires DamiaoRuntimeConfig")
        hardware = replace(
            hardware,
            adapter_kwargs={
                **hardware.adapter_kwargs,
                "runtime_config": replace(runtime_config, passive_grippers=("gripper",)),
            },
        )
    hardware = replace(
        hardware,
        wb_config=WholeBodyConfig(kp=(0.0,) * len(OPENYAM_JOINTS), kd=OPENYAM_TEACH_DAMPING),
    )
    return ControlCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[hardware],
        tasks=[
            TaskConfig(
                name="teach_openyam",
                type="trajectory",
                joint_names=list(OPENYAM_JOINTS),
                priority=10,
                params={"hold_position_when_idle": True},
            )
        ],
    )


openyam_teach_collection = autoconnect(
    _teach_robot(),
    CameraModule.blueprint(
        instance_name="wrist",
        hardware=WebcamConfig(
            camera_index=0, width=640, height=480, fps=30, frame_id_prefix="wrist_image"
        ),
        frame_id="wrist_camera_link",
    ),
    collection_recorder(profile=OPENYAM_TEACH_COLLECTION),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
).remappings(
    [
        ("wrist", "color_image", "wrist_image"),
        ("wrist", "camera_info", "wrist_camera_info"),
        ("wrist", "tf", "wrist_tf"),
    ]
)
