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

"""Composable A1Z demonstration, replay, and learned-policy blueprints."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from dimos.control.coordinator import TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.manipulators.galaxea_a1z.config import (
    A1ZConfig,
    A1ZGripperConfig,
    A1ZTeachingConfig,
)
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import WebcamConfig
from dimos.imitation.collection.episode_monitor import EpisodeMonitorModule
from dimos.imitation.collection.recorder import CollectionRecorder
from dimos.memory.module import OnExisting
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.manipulators.a1z.config import A1Z_G1Z_MODEL_PATH, a1z_hardware
from dimos.robot.manipulators.a1z.learning import A1Z_LEARNING_PROFILE
from dimos.robot.manipulators.common.blueprints import coordinator

if TYPE_CHECKING:
    from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule

A1Z_REPLAY_TASK_NAME = "teach_replay_arm"
A1Z_POLICY_TASK_NAME = "lerobot_trajectory_arm"
A1Z_TEACH_CAMERA_WIDTH = A1Z_LEARNING_PROFILE.camera_width
A1Z_TEACH_CAMERA_HEIGHT = A1Z_LEARNING_PROFILE.camera_height
A1Z_TEACH_CAMERA_FPS = A1Z_LEARNING_PROFILE.fps


def _a1z_camera(camera_index: int) -> Blueprint:
    return CameraModule.blueprint(
        hardware=WebcamConfig(
            camera_index=camera_index,
            width=A1Z_TEACH_CAMERA_WIDTH,
            height=A1Z_TEACH_CAMERA_HEIGHT,
            fps=A1Z_TEACH_CAMERA_FPS,
        ),
        # Placeholder until the wrist-camera mount is calibrated. Learned
        # policies do not consume this transform, but recording needs a frame.
        transform=Transform(
            frame_id="coordinator",
            child_frame_id="camera_link",
        ),
    )


def make_a1z_teach_blueprint(
    db_path: Path,
    *,
    task_label: str,
    camera_index: int = 0,
    gripper_free_drive: bool = False,
) -> Blueprint:
    """Record camera and measured arm/gripper state while hand-drivable."""
    hardware = a1z_hardware(
        "arm",
        has_gripper=True,
        dynamics_urdf_path=A1Z_G1Z_MODEL_PATH,
        adapter_config=A1ZConfig(
            gripper=A1ZGripperConfig(),
            teaching=A1ZTeachingConfig(gripper_free_drive=gripper_free_drive),
        ),
    )
    return autoconnect(
        coordinator(hardware=[hardware], tasks=[]),
        EpisodeMonitorModule.blueprint(task=task_label),
        CollectionRecorder.blueprint(
            db_path=db_path,
            on_existing=OnExisting.ERROR,
            root_frame="coordinator",
            default_frame_id="coordinator",
            tf_tolerance=1.5,
            record_tf=False,
        ),
        _a1z_camera(camera_index),
    )


def make_a1z_replay_blueprint() -> Blueprint:
    """Run a validated seven-joint arm/gripper trajectory through the coordinator."""
    hardware = a1z_hardware(
        "arm",
        has_gripper=True,
        dynamics_urdf_path=A1Z_G1Z_MODEL_PATH,
    )
    return coordinator(
        hardware=[hardware],
        tasks=[
            TaskConfig(
                name=A1Z_REPLAY_TASK_NAME,
                type="trajectory",
                joint_names=hardware.joints,
                priority=10,
            )
        ],
    )


def make_a1z_policy_blueprint(
    policy_path: str,
    *,
    policy_module: type[LeRobotPolicyModule] | None = None,
    task: str = "",
    camera_index: int = 0,
    device: str | None = None,
    fps: float = A1Z_TEACH_CAMERA_FPS,
) -> Blueprint:
    """Run one trained LeRobot policy against the live A1Z camera and state."""
    if policy_module is None:
        # LeRobot is optional and intentionally imported only when this factory is used.
        from dimos.imitation.policy.lerobot.module import LeRobotPolicyModule

        policy_module = LeRobotPolicyModule
    hardware = a1z_hardware(
        "arm",
        has_gripper=True,
        dynamics_urdf_path=A1Z_G1Z_MODEL_PATH,
    )
    return autoconnect(
        coordinator(
            hardware=[hardware],
            tasks=[
                TaskConfig(
                    name=A1Z_POLICY_TASK_NAME,
                    type="trajectory",
                    joint_names=hardware.joints,
                    priority=10,
                )
            ],
        ),
        policy_module.blueprint(
            policy_path=policy_path,
            task=task,
            device=device,
            joint_names=hardware.joints,
            fps=fps,
            robot_type="galaxea_a1z",
        ),
        _a1z_camera(camera_index),
    )
