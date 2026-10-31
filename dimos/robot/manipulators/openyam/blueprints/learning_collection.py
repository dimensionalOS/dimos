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

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.hardware.whole_body.damiao.config import DamiaoRuntimeConfig
from dimos.hardware.whole_body.spec import WholeBodyConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import collect
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openyam.config import OPENYAM_JOINTS, openyam_hardware
from dimos.robot.manipulators.openyam.coordinator import OpenYamCoordinator

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
                "runtime_config": replace(
                    runtime_config, gravity_comp=True, passive_grippers=("gripper",)
                ),
            },
        )
    hardware = replace(
        hardware,
        wb_config=WholeBodyConfig(kp=(0.0,) * len(OPENYAM_JOINTS), kd=OPENYAM_TEACH_DAMPING),
    )
    return OpenYamCoordinator.blueprint(
        instance_name="ControlCoordinator",
        hardware=[hardware],
        tasks=[
            TaskConfig(
                name="teach_openyam",
                type="hand_guiding",
                joint_names=list(OPENYAM_JOINTS),
                priority=10,
                auto_start=True,
            )
        ],
    )


_state = FeatureSpec(
    stream="coordinator_joint_state",
    message_type=JointState,
    field="position",
    dtype="float32",
    shape=(len(OPENYAM_JOINTS),),
    names=list(OPENYAM_JOINTS),
)
_profile = CollectionProfile(
    observations={
        "observation.images.wrist": FeatureSpec(
            stream="wrist_image",
            message_type=Image,
            field="data",
            dtype="video",
            shape=(480, 640, 3),
            names=["height", "width", "channels"],
        ),
        "observation.state": _state,
    },
    actions={"action": _state.model_copy(deep=True)},
    metadata={"name": "openyam-teach", "robot_type": "openyam"},
)

openyam_teach_collection = autoconnect(
    collect(profile=_profile),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
    _teach_robot(),
    WebcamModule.blueprint(
        instance_name="wrist",
        hardware=WebcamConfig(camera_index=0, fps=30, frame_id_prefix="wrist_image"),
        frame_id="wrist_camera_link",
    ),
).remappings(
    [
        ("wrist", "color_image", "wrist_image"),
        ("wrist", "camera_info", "wrist_camera_info"),
        ("wrist", "tf", "wrist_tf"),
    ]
)
