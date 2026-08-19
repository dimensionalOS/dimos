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

"""Recording blueprints.

`CollectionRecorder` (a memory2 Recorder) captures the obs/action/status
streams to a SQLite session DB during the run and flushes it durably on
shutdown. DataPrep reads that DB afterwards.
"""

from __future__ import annotations

from datetime import datetime
from functools import partial
from pathlib import Path

from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.global_config import global_config
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionRecorder
from dimos.robot.manipulators.openarm.blueprints.teleop import teleop_quest_openarm
from dimos.teleop.quest.blueprints import (
    teleop_quest_piper,
    teleop_quest_xarm7,
)


def _session_db(robot: str) -> str:
    """Timestamped session DB path under the state dir, namespaced by robot."""
    return str(STATE_DIR / "recordings" / f"session_{robot}_{datetime.now():%Y%m%d_%H%M%S}.db")


def _camera_if_real() -> tuple[Blueprint, ...]:
    """Real RealSense only off-sim. In `--simulation` the teleop coordinator's
    MujocoSimModule already publishes color_image on /camera/color_image, so a
    real camera would be redundant (and fail with no device connected)."""
    if global_config.simulation:
        return ()
    return (RealSenseCamera.blueprint(enable_pointcloud=False),)


# buttons / color_image / status are left to autoconnect — each name is unique
# across the composed blueprint, so it resolves to a stable /<name> topic shared
# by producer and recorder. The joint stream is remapped onto the coordinator's
# per-robot `arm_joints` output; the recorder still writes it to the db under its
# port name (`coordinator_joint_state`), which is what DataPrep reads.
_JOINTS_FROM_ARM = [(CollectionRecorder, "coordinator_joint_state", "arm_joints")]

learning_collect_quest_xarm7 = autoconnect(
    teleop_quest_xarm7,
    *_camera_if_real(),
    EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
    CollectionRecorder.blueprint(db_path=_session_db("xarm7")),
).remappings(_JOINTS_FROM_ARM)


learning_collect_quest_piper = autoconnect(
    teleop_quest_piper,
    *_camera_if_real(),
    EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
    CollectionRecorder.blueprint(db_path=_session_db("piper")),
).remappings(_JOINTS_FROM_ARM)


# The OpenArm coordinator publishes the combined coordinator_joint_state, so
# the recorder connects without a per-robot remapping.
learning_collect_quest_openarm = autoconnect(
    teleop_quest_openarm,
    *_camera_if_real(),
    EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
    CollectionRecorder.blueprint(db_path=_session_db("openarm")),
)


def _scene_webcam() -> Webcam:
    """First external UVC camera, falling back to device 0.

    Node indices shift across reboots, so pick by name instead of number.
    """
    index = 0
    for node in sorted(Path("/sys/class/video4linux").glob("video*")):
        name = (node / "name").read_text()
        if "Integrated" not in name:
            index = int(node.name.removeprefix("video"))
            break
    return Webcam(camera_index=index, width=1280, height=720, fps=30.0)


# Variant for a plain UVC scene camera instead of a RealSense.
learning_collect_quest_openarm_webcam = autoconnect(
    teleop_quest_openarm,
    CameraModule.blueprint(hardware=_scene_webcam),
    EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
    CollectionRecorder.blueprint(db_path=_session_db("openarm")),
)


# Physical USB port each rig camera is plugged into, identified by unplug
# testing on 2026-08-18 (see memory: openarm-act-4cam-mapping). The cameras
# report an identical fake serial number, so the USB port is the only way to
# tell them apart; this mapping breaks if a cable moves to a different port,
# and the ports themselves must stay labeled.
_MULTICAM_USB_PORTS = {
    "chest": "3-5.1",
    "left_hand": "3-5.2",
    "right_hand": "3-5.3",
    "waist": "3-4.1",
}


def _video_index_for_usb_port(port: str, sysfs_root: Path = Path("/sys/class/video4linux")) -> int:
    """Resolve the lowest /dev/videoN capture node at a physical USB port.

    Each UVC camera exposes a pair of adjacent nodes sharing one `device`
    symlink; the lower-numbered node is the one that actually streams
    frames. Raises if the port is empty, rather than silently falling back
    to a different physical camera under the wrong body label.
    """
    matches: list[int] = []
    for node in sysfs_root.glob("video*"):
        device_path = (node / "device").resolve()
        node_port = device_path.name.split(":")[0]
        if node_port == port:
            matches.append(int(node.name.removeprefix("video")))
    if not matches:
        raise RuntimeError(
            f"No camera found at USB port {port!r}. Check the cable is connected "
            "to its labeled port; ports drift only if a cable is moved."
        )
    return min(matches)


def _rig_camera(label: str) -> Webcam:
    port = _MULTICAM_USB_PORTS[label]
    index = _video_index_for_usb_port(port)
    return Webcam(camera_index=index, width=1280, height=720, fps=30.0)


def _rig_camera_blueprint(label: str) -> Blueprint:
    instance_name = f"{label}_camera"
    return CameraModule.blueprint(
        instance_name=instance_name,
        hardware=partial(_rig_camera, label),
    ).remappings([(instance_name, "color_image", f"{label}_image")])


# Multi-view variant recording all four labeled rig cameras (chest, both
# hands, waist) as separate observation streams.
learning_collect_quest_openarm_multicam = autoconnect(
    teleop_quest_openarm,
    *(_rig_camera_blueprint(label) for label in _MULTICAM_USB_PORTS),
    EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
    CollectionRecorder.blueprint(db_path=_session_db("openarm")),
)
