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

`CollectionRecorder` (a memory Recorder) captures the obs/action/status
streams to a SQLite session DB during the run and flushes it durably on
shutdown. DataPrep reads that DB afterwards.
"""

from __future__ import annotations

from datetime import datetime
from functools import partial

from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.stream import In
from dimos.hardware.sensors.camera.module import CameraModule, CameraModuleConfig
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.hardware.sensors.lidar.pointlio.pointlio_blueprints import mid360_pointlio
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionRecorder
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.quest_types import Buttons
from dimos.robot.manipulators.openarm.blueprints.teleop import teleop_quest_openarm_blueprint
from dimos.robot.manipulators.openarm.homing_module import OpenArmHomingModule
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


# buttons / color_image / coordinator_joint_state / status are left to
# autoconnect — each name is unique across the composed blueprint, so it
# resolves to a stable /<name> topic shared by producer and recorder. The
# recorder captures whatever joints are present, so the coordinator's aggregate
# stream is its intended input (see dimos/control/README.md).
learning_collect_quest_xarm7 = autoconnect(
    CollectionRecorder.blueprint(
        db_path=_session_db("xarm7"),
        poseless_streams=["color_image", "coordinator_joint_state", "status"],
        record_tf=False,
    ),
    EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
    teleop_quest_xarm7,
    *_camera_if_real(),
)


learning_collect_quest_piper = autoconnect(
    CollectionRecorder.blueprint(
        db_path=_session_db("piper"),
        poseless_streams=["color_image", "coordinator_joint_state", "status"],
        record_tf=False,
    ),
    EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
    teleop_quest_piper,
    *_camera_if_real(),
)


# Wrist camera USB port paths on the Alfred-mounted bimanual OpenArm rig
# (orin-nx-7837). The two 4K fisheyes report identical USB serials, so the
# physical port path is the only stable identity; it holds only while the
# cables stay in their ports. Override per run with
# --left-wrist-camera.device / --right-wrist-camera.device.
OPENARM_LEFT_WRIST_CAMERA_DEVICE = (
    "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1.4:1.0-video-index0"
)
OPENARM_RIGHT_WRIST_CAMERA_DEVICE = ""

# Front D455 on the Alfred rig; pinned so the scene camera never grabs the
# rig's other RealSense. Override with --real-sense-camera.serial-number.
OPENARM_SCENE_CAMERA_SERIAL = "260922302422"

OPENARM_COLLECT_IMAGE_WIDTH = 640
OPENARM_COLLECT_IMAGE_HEIGHT = 480
OPENARM_COLLECT_IMAGE_FPS = 30


def _wrist_webcam(device: int | str, width: int, height: int, fps: float) -> Webcam:
    return Webcam(camera_index=device, width=width, height=height, fps=fps)


class WristCameraConfig(CameraModuleConfig):
    device: str = ""
    width: int = OPENARM_COLLECT_IMAGE_WIDTH
    height: int = OPENARM_COLLECT_IMAGE_HEIGHT
    fps: float = OPENARM_COLLECT_IMAGE_FPS
    transform: Transform | None = None


class WristCameraModule(CameraModule):
    """A CameraModule whose webcam is selected by CLI-settable device path."""

    config: WristCameraConfig

    @rpc
    def start(self) -> None:
        if not self.config.device:
            raise RuntimeError(
                f"{self.config.instance_name}: no camera device set, pass "
                f"--{(self.config.instance_name or '').replace('_', '-')}.device"
            )
        self.config.hardware = partial(
            _wrist_webcam,
            self.config.device,
            self.config.width,
            self.config.height,
            self.config.fps,
        )
        super().start()


def _wrist_camera(side: str, device: str) -> Blueprint:
    name = f"{side}_wrist_camera"
    return WristCameraModule.blueprint(instance_name=name, device=device).remappings(
        [
            (name, "color_image", f"{side}_wrist_image"),
            (name, "camera_info", f"{side}_wrist_camera_info"),
        ]
    )


class OpenArmCollectionRecorder(CollectionRecorder):
    left_wrist_image: In[Image]
    right_wrist_image: In[Image]
    coordinator_joint_targets: In[JointState]  # action (commanded targets)
    # Raw operator inputs, for end effector space training and episode
    # forensics. Streams without a producer stay silently empty.
    left_cartesian_command: In[PoseStamped]
    right_cartesian_command: In[PoseStamped]
    teleop_buttons: In[Buttons]
    twist_command: In[Twist]
    # Intrinsics for every camera, 1 Hz.
    camera_info: In[CameraInfo]
    left_wrist_camera_info: In[CameraInfo]
    right_wrist_camera_info: In[CameraInfo]
    # World frame chassis pose when the lidar odometry blueprint is composed
    # in (the mobile variant below).
    pointlio_odometry: In[Odometry]


def _openarm_cameras_if_real() -> tuple[Blueprint, ...]:
    if global_config.simulation:
        return ()
    return (
        RealSenseCamera.blueprint(
            serial_number=OPENARM_SCENE_CAMERA_SERIAL,
            width=OPENARM_COLLECT_IMAGE_WIDTH,
            height=OPENARM_COLLECT_IMAGE_HEIGHT,
            fps=OPENARM_COLLECT_IMAGE_FPS,
            enable_depth=False,
            enable_pointcloud=False,
        ),
        _wrist_camera("left", OPENARM_LEFT_WRIST_CAMERA_DEVICE),
        _wrist_camera("right", OPENARM_RIGHT_WRIST_CAMERA_DEVICE),
    )


learning_collect_quest_openarm = autoconnect(
    OpenArmCollectionRecorder.blueprint(
        db_path=_session_db("openarm"),
        poseless_streams=[
            "color_image",
            "left_wrist_image",
            "right_wrist_image",
            "coordinator_joint_state",
            "coordinator_joint_targets",
            "left_cartesian_command",
            "right_cartesian_command",
            "teleop_buttons",
            "twist_command",
            "camera_info",
            "left_wrist_camera_info",
            "right_wrist_camera_info",
            "pointlio_odometry",
            "status",
        ],
        record_tf=False,
    ),
    EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
    OpenArmHomingModule.blueprint(),  # right thumbstick click, deadman released
    teleop_quest_openarm_blueprint(publish_joint_targets=True, enable_base=True),
    *_openarm_cameras_if_real(),
)


# Mobile manipulation variant: the lidar odometry stack joins, so the
# recorder's pointlio_odometry stream carries the world frame chassis pose.
# Needs the Mid-360 reachable and its DIMOS_POINTLIO_* addresses set.
learning_collect_quest_openarm_mobile = autoconnect(
    learning_collect_quest_openarm,
    mid360_pointlio,
)
