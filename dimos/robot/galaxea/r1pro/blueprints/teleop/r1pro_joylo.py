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

"""r1pro-openarm-joylo-real on the simulated robot; see docs/development/r1pro-joylo.md.

The hardware blueprint and this one are the same stack. Only the robot differs:
here a sim2 world stands behind CC's connection contract, and MuJoCo renders the
head and wrist cameras on the hardware's stream names, types, sizes and rates. The
operator devices, the headset page, the coordinator and the recorder are those
of the hardware run, so an app or a dataset tool cannot tell the two apart.
"""

import math
from pathlib import Path

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.assets.model import UsdModel
from dimos.robot.galaxea.r1pro.blueprints.teleop.r1pro_joylo_inputs import (
    _camera_transports,
    _control_blueprint,
    _operator_blueprint,
    _webxr_blueprint,
    _webxr_joystick_blueprint,
)
from dimos.robot.galaxea.r1pro.collection import r1pro_collection
from dimos.robot.galaxea.r1pro.connection import R1ProConnectionConfig
from dimos.robot.galaxea.r1pro.head_cameras import HEAD_JPEG_QUALITY
from dimos.robot.galaxea.r1pro.motion import motion_groups
from dimos.robot.galaxea.r1pro.sim.cameras import (
    WRIST_JPEG_QUALITY,
    head_color_camera,
    head_depth_camera,
    wrist_color_camera,
    wrist_depth_camera,
)
from dimos.robot.galaxea.r1pro.sim.config import (
    ARM_RATE,
    CONTROL_MODEL,
    JOINTS,
    MODEL,
    TORSO_RATE,
    UPPER_JOINTS,
    UPPER_RATES,
)
from dimos.robot.galaxea.r1pro.sim.connection import R1ProSimConnection
from dimos.robot.galaxea.r1pro.sim.control import R1ProSimControl
from dimos.robot.galaxea.r1pro.sim.model import TORSO_LINK_LENGTHS
from dimos.robot.galaxea.r1pro.sim.office import SCENE
from dimos.robot.usd_manager import UsdManager
from dimos.sim2.module import SimModule
from dimos.sim2.physics.mujoco import MujocoBackend
from dimos.sim2.sensors.camera.config import CameraConfig
from dimos.sim2.sensors.camera.module import SimColorCameraModule, SimDepthCameraModule
from dimos.sim2.sensors.camera.render_module import CameraRenderModule
from dimos.sim2.sensors.camera.renderers.mujoco import MujocoRenderer
from dimos.sim2.sensors.encoder.module import EncoderConfig, SimJointEncoderModule
from dimos.sim2.sensors.tf import SimTfModule

_declaration = R1ProConnectionConfig(model=CONTROL_MODEL)


def _world_blueprint(scene: Path = SCENE) -> Blueprint:
    """The simulated robot behind the connection: scene, physics, encoders and servo."""
    return autoconnect(
        UsdManager.blueprint(
            model=UsdModel.from_file(scene), render_model=MODEL, publish_joint_tf=False
        ),
        SimTfModule.blueprint(),
        SimModule.blueprint(
            scene_manager=UsdManager,
            backend=MujocoBackend,
            dt_ns=2_000_000,
            joint_ids=JOINTS,
            observation_hz=250,
        ),
        SimJointEncoderModule.blueprint(sensor=EncoderConfig(JOINTS, hz=250)),
        R1ProSimControl.blueprint(coordinator_torso=True),
        R1ProSimConnection.blueprint(model=CONTROL_MODEL),
    ).remappings(
        [
            (SimJointEncoderModule, "joint_state", "encoder"),
            (R1ProSimControl, "joint_state", "encoder"),
            (UsdManager, "coordinator_joint_state", "encoder"),
            (R1ProSimControl, "cmd_vel", "servo_velocity"),
            (R1ProSimControl, "left_gripper_command", "servo_left_gripper"),
            (R1ProSimControl, "right_gripper_command", "servo_right_gripper"),
        ]
    )


# A camera's color and its depth are two views from one frame, at two rates. Each
# set has a rendering worker: the color runs at the camera rate beside the depth.
_VIDEO_RENDERER = "video_render"
_VIDEO_CAMERAS: dict[str, tuple[CameraConfig, int]] = {
    **{
        f"head_{side}_color": (head_color_camera(side), HEAD_JPEG_QUALITY)
        for side in ("left", "right")
    },
    **{
        f"wrist_{side}_color": (wrist_color_camera(side), WRIST_JPEG_QUALITY)
        for side in ("left", "right")
    },
}
# Depth has the name of the camera it belongs to, as R1SpatialSensors names it.
_DEPTH_CAMERAS: dict[str, CameraConfig] = {
    "head_left": head_depth_camera(),
    "wrist_left": wrist_depth_camera("left"),
    "wrist_right": wrist_depth_camera("right"),
}


def _video_camera(stream: str, sensor: CameraConfig, jpeg_quality: int) -> Blueprint:
    """One camera's own color on the stream the hardware camera writes."""
    return SimColorCameraModule.blueprint(
        instance_name=f"{stream}_camera",
        sensor=sensor,
        jpeg_quality=jpeg_quality,
        renderer_name=_VIDEO_RENDERER,
    ).remappings(
        [
            (SimColorCameraModule, "color_image", stream),
            (SimColorCameraModule, "camera_info", f"{stream}_info"),
        ]
    )


def _depth_camera(camera: str, sensor: CameraConfig) -> Blueprint:
    """One camera's depth and its intrinsics on the streams R1SpatialSensors writes."""
    return SimDepthCameraModule.blueprint(
        instance_name=f"{camera}_depth_camera", sensor=sensor
    ).remappings(
        [
            (SimDepthCameraModule, "depth_image", f"{camera}_depth"),
            (SimDepthCameraModule, "camera_info", f"{camera}_info"),
        ]
    )


def _camera_blueprint() -> Blueprint:
    """Each camera's own color, and its depth as R1SpatialSensors publishes it."""
    return autoconnect(
        *(_video_camera(stream, *camera) for stream, camera in _VIDEO_CAMERAS.items()),
        CameraRenderModule.blueprint(
            instance_name=_VIDEO_RENDERER,
            scene_manager=UsdManager,
            cameras=tuple(f"{stream}_camera" for stream in _VIDEO_CAMERAS),
            renderer=MujocoRenderer,
        ),
        *(_depth_camera(camera, sensor) for camera, sensor in _DEPTH_CAMERAS.items()),
        CameraRenderModule.blueprint(
            scene_manager=UsdManager,
            cameras=tuple(f"{camera}_depth_camera" for camera in _DEPTH_CAMERAS),
            # Both groups use the same Filament RGB-D implementation; separate
            # workers preserve their independent acquisition rates.
            renderer=MujocoRenderer,
        ),
    ).transports(_camera_transports)


def _robot_blueprint(scene: Path = SCENE) -> Blueprint:
    """The simulated robot under JoyLo and the coordinator: no devices, cameras or recorder."""
    return _control_blueprint(
        _world_blueprint(scene),
        R1ProSimConnection,
        _declaration,
        joylo={
            "model": MODEL,
            "leader_angle_period": math.tau,
            "torso_link_lengths": TORSO_LINK_LENGTHS,
        },
        # The simulation's own tracking profile; the hardware declares slower rates.
        motion_limits=motion_groups(
            arm_rate=ARM_RATE,
            arm_ceiling=ARM_RATE,
            torso_rate=TORSO_RATE,
            torso_ceiling=TORSO_RATE,
            base_speed=0.1,
            yaw_speed=0.25,
        ),
        velocity_limits=dict(zip(UPPER_JOINTS, UPPER_RATES, strict=True)),
    )


# The recorder starts first and stops last, as on the robot.
# The launch requires --recorder.recording and --episodes.task.
# No leader serial port is not an error here: the headset or a browser can drive.
# macOS reads the Joy-Cons through HIDAPI; Linux keeps its hid-nintendo backend.
r1pro_openarm_joylo_sim2 = autoconnect(
    r1pro_collection(),
    _robot_blueprint(),
    _webxr_joystick_blueprint(),
    _camera_blueprint(),
    _operator_blueprint(require_leaders=False),
    _webxr_blueprint(),
).global_config(viewer="none", transport="zenoh", n_workers=10)
