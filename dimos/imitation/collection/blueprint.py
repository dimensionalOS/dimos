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
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module
from dimos.core.stream import In, Out
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
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.diy.alfred.pillar_connection import (
    PILLAR_HARDWARE_ID,
    PILLAR_MAX_POSITION_M,
    PILLAR_MIN_POSITION_M,
    PillarConnection,
)
from dimos.robot.manipulators.openarm.blueprints.teleop import teleop_quest_openarm_blueprint
from dimos.robot.manipulators.openarm.homing_module import OpenArmHomingModule
from dimos.teleop.quest.blueprints import (
    teleop_quest_piper,
    teleop_quest_xarm7,
)
from dimos.teleop.quest.quest_extensions import ArmBaseTeleopConfig, ArmBaseTeleopModule
from dimos.teleop.quest.quest_types import Buttons, QuestControllerState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


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
OPENARM_RIGHT_WRIST_CAMERA_DEVICE = (
    "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.3:1.0-video-index0"
)
OPENARM_CHEST_CAMERA_DEVICE = (
    "/dev/v4l/by-path/platform-140c0000.pcie-pci-0009:01:00.0-usb-0:1:1.0-video-index0"
)

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


class ImageFlipModule(Module):
    """Republish a camera stream rotated 180 degrees.

    For cameras that mount upside down for cable routing; the flipped
    stream is what the recorder and any consumer should read.
    """

    image_in: In[Image]
    image_out: Out[Image]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.image_in.subscribe(self._flip)))

    def _flip(self, msg: Image) -> None:
        self.image_out.publish(
            Image(
                data=np.ascontiguousarray(msg.data[::-1, ::-1]),
                format=msg.format,
                frame_id=msg.frame_id,
                ts=msg.ts,
            )
        )


def _usb_camera(name: str, device: str) -> Blueprint:
    instance = f"{name}_camera"
    return WristCameraModule.blueprint(instance_name=instance, device=device).remappings(
        [
            (instance, "color_image", f"{name}_image"),
            (instance, "camera_info", f"{name}_camera_info"),
        ]
    )


class AlfredLiftTeleopConfig(ArmBaseTeleopConfig):
    lift_speed_m_s: float = 0.01
    lift_deadzone: float = 0.25
    lift_engage_center_tolerance: float = 0.15
    lift_command_hz: float = 5.0


class AlfredLiftTeleopModule(ArmBaseTeleopModule):
    """Base driving plus a click-gated pillar lift mode on the left stick.

    A left thumbstick click with both deadman grips released and both sticks
    centered toggles lift mode. In lift mode the base is frozen (zero twist
    every tick) and the left stick's vertical axis jogs the pillar slowly
    through its position RPC. A second click, or grabbing either deadman,
    exits the mode, stops the pillar, and drops every internal target.
    """

    config: AlfredLiftTeleopConfig

    _pillar: PillarConnection

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lift_mode = False
        self._lift_target: float | None = None
        self._lift_press_was_down = False
        self._last_lift_rpc_t = 0.0
        self._last_drive_t: float | None = None

    def _exit_lift_mode(self) -> None:
        self._lift_mode = False
        self._lift_target = None
        try:
            self._pillar.stop_motion()
        except Exception:
            logger.exception("Pillar stop failed on lift mode exit")
        logger.info("Lift mode off")

    def _try_enter_lift_mode(
        self,
        left: QuestControllerState,
        right: QuestControllerState | None,
    ) -> None:
        tol = self.config.lift_engage_center_tolerance
        sticks = [left.thumbstick.x, left.thumbstick.y]
        if right is not None:
            sticks += [right.thumbstick.x, right.thumbstick.y]
        if any(abs(v) > tol for v in sticks):
            logger.warning("Ignoring lift mode click while a stick is deflected")
            return
        try:
            status = self._pillar.get_status()
        except Exception:
            logger.exception("Pillar status unavailable, staying out of lift mode")
            return
        if not status.get("homed") or status.get("position_m") is None:
            logger.warning("Pillar is not homed, run PillarConnection.home() first")
            return
        self._lift_target = float(status["position_m"])
        self._lift_mode = True
        logger.info("Lift mode on at %.3f m", self._lift_target)

    def _handle_base_drive(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        now = time.monotonic()
        dt = 0.0 if self._last_drive_t is None else min(now - self._last_drive_t, 0.1)
        self._last_drive_t = now

        press = bool(left.thumbstick_press) if left is not None else False
        rising = press and not self._lift_press_was_down
        self._lift_press_was_down = press

        deadman_held = any(self._is_engaged.values())
        if self._lift_mode and deadman_held:
            self._exit_lift_mode()
        elif rising and not deadman_held:
            if self._lift_mode:
                self._exit_lift_mode()
            elif left is not None:
                self._try_enter_lift_mode(left, right)

        if not self._lift_mode:
            super()._handle_base_drive(left, right)
            return

        self._publish_safe_base_command()
        if left is None or self._lift_target is None:
            return
        y = left.thumbstick.y
        if abs(y) < self.config.lift_deadzone:
            return
        self._lift_target -= y * self.config.lift_speed_m_s * dt
        self._lift_target = min(
            max(self._lift_target, PILLAR_MIN_POSITION_M), PILLAR_MAX_POSITION_M
        )
        if now - self._last_lift_rpc_t < 1.0 / self.config.lift_command_hz:
            return
        self._last_lift_rpc_t = now
        try:
            self._pillar.set_position(self._lift_target)
        except Exception:
            logger.exception("Pillar position command failed")
            self._exit_lift_mode()


class OpenArmCollectionRecorder(CollectionRecorder):
    left_wrist_image: In[Image]
    right_wrist_image: In[Image]
    chest_image: In[Image]
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
    chest_camera_info: In[CameraInfo]
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
        _usb_camera("left_wrist", OPENARM_LEFT_WRIST_CAMERA_DEVICE),
        _usb_camera("right_wrist", OPENARM_RIGHT_WRIST_CAMERA_DEVICE),
        _usb_camera("chest", OPENARM_CHEST_CAMERA_DEVICE),
    )


learning_collect_quest_openarm = autoconnect(
    OpenArmCollectionRecorder.blueprint(
        db_path=_session_db("openarm"),
        poseless_streams=[
            "color_image",
            "left_wrist_image",
            "right_wrist_image",
            "chest_image",
            "coordinator_joint_state",
            "coordinator_joint_targets",
            "left_cartesian_command",
            "right_cartesian_command",
            "teleop_buttons",
            "twist_command",
            "camera_info",
            "left_wrist_camera_info",
            "right_wrist_camera_info",
            "chest_camera_info",
            "pointlio_odometry",
            "status",
        ],
        record_tf=False,
    ),
    EpisodeMonitorModule.blueprint(),  # default button_map: toggle=B, discard=Y
    OpenArmHomingModule.blueprint(),  # right thumbstick click, deadman released
    teleop_quest_openarm_blueprint(
        publish_joint_targets=True,
        enable_base=True,
        enable_pillar=True,
        teleop_module_cls=AlfredLiftTeleopModule,
    ),
    PillarConnection.blueprint(),
    # The D455 mounts upside down for cable routing; the recorder reads the
    # flipped stream while raw color_image stays on the bus for diagnosis.
    ImageFlipModule.blueprint(instance_name="scene_flip").remappings(
        [
            ("scene_flip", "image_in", "color_image"),
            ("scene_flip", "image_out", "scene_image"),
        ]
    ),
    *_openarm_cameras_if_real(),
).remappings([(OpenArmCollectionRecorder, "color_image", "scene_image")]).transports(
    {
        ("motor_command", MotorCommandArray): LCMTransport.spec(
            f"/{PILLAR_HARDWARE_ID}/motor_command", MotorCommandArray
        ),
        ("motor_states", JointState): LCMTransport.spec(
            f"/{PILLAR_HARDWARE_ID}/motor_states", JointState
        ),
    }
)


# Mobile manipulation variant: the lidar odometry stack joins, so the
# recorder's pointlio_odometry stream carries the world frame chassis pose.
# Needs the Mid-360 reachable and its DIMOS_POINTLIO_* addresses set.
learning_collect_quest_openarm_mobile = autoconnect(
    learning_collect_quest_openarm,
    mid360_pointlio,
)
