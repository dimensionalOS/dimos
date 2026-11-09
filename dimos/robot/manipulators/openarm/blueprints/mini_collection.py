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

"""OpenArm Mini leader-follower collection for the bimanual OpenArm."""

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.module import WebcamModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionProfile
from dimos.imitation.collection.recorder import collect
from dimos.imitation.dataprep.schema import FeatureSpec
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openarm.blueprints.mini_teleop import (
    teleop_openarm_mini,
    teleop_openarm_mini_follower,
)
from dimos.robot.manipulators.openarm.connection import OpenArmConnection

_CAMERAS = ("left_wrist", "right_wrist")
# The coordinator reports the connection's joints: arm/<urdf joint>, arms then jaws.
_JOINTS = [f"arm/{joint}" for joint in OpenArmConnection.robot.joint_names]
_ARM_JOINTS = [f"arm/{joint}" for joint in OpenArmConnection.robot.arm_joint_names]

_state = FeatureSpec(
    stream="coordinator_joint_state",
    message_type=JointState,
    field="position",
    dtype="float32",
    shape=(len(_JOINTS),),
    names=_JOINTS,
)
# The leaders drive the 14 arm joints only; the grippers are observed, not commanded.
_profile = CollectionProfile(
    observations={
        **{
            f"observation.images.{name}": FeatureSpec(
                stream=f"{name}_image",
                message_type=Image,
                field="data",
                dtype="video",
                shape=(480, 640, 3),
                names=["height", "width", "channels"],
            )
            for name in _CAMERAS
        },
        "observation.state": _state,
    },
    actions={
        "action": _state.model_copy(
            update={
                "stream": "coordinator_joint_command",
                "source_kind": "joint_position_updates",
                "shape": (len(_ARM_JOINTS),),
                "names": _ARM_JOINTS,
            },
            deep=True,
        )
    },
    metadata={"name": "openarm-mini", "robot_type": "openarm"},
)


def _cameras() -> Blueprint:
    return autoconnect(
        *(
            WebcamModule.blueprint(
                instance_name=name,
                hardware=WebcamConfig(camera_index=index, fps=30, frame_id_prefix=f"{name}_image"),
                frame_id=f"{name}_camera_link",
            )
            for index, name in enumerate(_CAMERAS)
        )
    ).remappings(
        [
            remapping
            for name in _CAMERAS
            for remapping in (
                (name, "color_image", f"{name}_image"),
                (name, "camera_info", f"{name}_camera_info"),
                (name, "tf", f"{name}_tf"),
            )
        ]
    )


# Leaders, follower and cameras on one machine; the follower is in-memory
# unless both CAN ports are given.
openarm_mini_collection = autoconnect(
    collect(profile=_profile),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
    teleop_openarm_mini,
    _cameras(),
)

# Robot side of the split deployment; the operator runs teleop-openarm-mini-leader.
openarm_mini_follower_collection = autoconnect(
    collect(profile=_profile),
    EpisodeMonitorModule.blueprint(instance_name="episodes"),
    teleop_openarm_mini_follower,
    _cameras(),
)
