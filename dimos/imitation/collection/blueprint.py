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

"""Profile-based Rust recording for the xArm and Piper teleop blueprints."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import cast

from dimos.constants import RECORDINGS_DIR
from dimos.control.coordinator import ControlCoordinator
from dimos.core.coordination.blueprint_config.fields import module_config_cls
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.camera.spec import CameraConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.profile import CollectionFeature, CollectionProfile
from dimos.imitation.collection.recorder import collection_recorder
from dimos.imitation.dataprep.core import SyncConfig
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.stream.audio.tts.kokoro_module import KokoroTTSModule
from dimos.teleop.webxr.blueprints import (
    teleop_webxr_piper,
    teleop_webxr_xarm7,
)


def _camera_if_real() -> tuple[Blueprint, ...]:
    """Real RealSense only off-sim. In `--simulation` the teleop coordinator's
    MujocoSimModule already publishes color_image on /camera/color_image, so a
    real camera would be redundant (and fail with no device connected)."""
    if global_config.simulation:
        return ()
    return (RealSenseCamera.blueprint(enable_pointcloud=False),)


def _collection_components(robot: str, teleop: Blueprint) -> tuple[Blueprint, ...]:
    producers = autoconnect(teleop, *_camera_if_real())
    coordinator = next(
        atom for atom in producers.active_blueprints if issubclass(atom.module, ControlCoordinator)
    )
    joints = [joint for hardware in coordinator.kwargs["hardware"] for joint in hardware.joints]
    camera = next(
        atom
        for atom in producers.active_blueprints
        if any(
            stream.name == "color_image" and stream.direction == "out" for stream in atom.streams
        )
    )
    camera_config = cast("CameraConfig", module_config_cls(camera)(**camera.kwargs))
    state = CollectionFeature(
        stream="coordinator_joint_state",
        message_type=JointState,
        field="position",
        dtype="float32",
        shape=(len(joints),),
        names=joints,
    )
    profile = CollectionProfile(
        name=f"{robot}-webxr",
        robot_type=robot,
        observations={
            "camera": CollectionFeature(
                stream="color_image",
                message_type=Image,
                field="data",
                dtype="video",
                shape=(camera_config.height, camera_config.width, 3),
                names=["height", "width", "channels"],
            ),
            "state": state,
        },
        actions={"action": state.model_copy(deep=True)},
        sync=SyncConfig(anchor="camera", rate_hz=camera_config.fps, tolerance_ms=50),
    )
    directory: Path = RECORDINGS_DIR / f"session_{robot}_{datetime.now():%Y%m%d_%H%M%S_%f}"
    return (
        collection_recorder(profile=profile, recording=directory, format="sqlite"),
        EpisodeMonitorModule.blueprint(),
        KokoroTTSModule.blueprint(instance_name="tts"),
        producers,
    )


learning_collect_webxr_xarm7 = autoconnect(*_collection_components("xarm7", teleop_webxr_xarm7))
learning_collect_webxr_piper = autoconnect(*_collection_components("piper", teleop_webxr_piper))
