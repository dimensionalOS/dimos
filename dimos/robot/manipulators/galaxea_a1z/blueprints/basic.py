# Copyright 2025-2026 Dimensional Inc.
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

"""Basic Galaxea A1Z coordinator blueprint."""

from __future__ import annotations

from functools import partial
from pathlib import Path

from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.hardware.sensors.camera.module import CameraModule
from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.learning.collection.episode_monitor import EpisodeMonitorModule
from dimos.learning.collection.recorder import CollectionRecorder
from dimos.learning.lerobot_policy import LeRobotPolicyConfig, LeRobotPolicyModule
from dimos.memory2.module import OnExisting
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.robot.manipulators.a1z.config import A1Z_G1Z_MODEL_PATH
from dimos.robot.manipulators.galaxea_a1z.config import galaxea_a1z_hardware

A1Z_REPLAY_TASK_NAME = "teach_replay_arm"
A1Z_POLICY_TASK_NAME = "lerobot_servo_arm"
A1Z_TEACH_CAMERA_WIDTH = 640
A1Z_TEACH_CAMERA_HEIGHT = 480
A1Z_TEACH_CAMERA_FPS = 15.0

# The real arm has a G1Z gripper. Its mass must be present in the dynamics
# model, and the hardware component exposes its measured/commanded opening as
# arm/gripper alongside the six arm joints.
_A1Z_DYNAMICS_URDF = str(A1Z_G1Z_MODEL_PATH)
_a1z_hw = galaxea_a1z_hardware("arm", gripper=True, dynamics_urdf_path=_A1Z_DYNAMICS_URDF)

coordinator_galaxea_a1z = autoconnect(
    ControlCoordinator.blueprint(
        hardware=[_a1z_hw],
        tasks=[
            TaskConfig(
                name="traj_arm",
                type="trajectory",
                joint_names=_a1z_hw.joints,
                priority=10,
            )
        ],
    ),
)


def _a1z_camera(camera_index: int) -> Blueprint:
    return CameraModule.blueprint(
        hardware=partial(
            Webcam,
            camera_index=camera_index,
            width=A1Z_TEACH_CAMERA_WIDTH,
            height=A1Z_TEACH_CAMERA_HEIGHT,
            fps=A1Z_TEACH_CAMERA_FPS,
        ),
        # Placeholder until the hackathon camera mount is calibrated.
        # Learned image policies do not consume this transform, but the
        # recorder and Rerun still require a connected frame tree.
        transform=Transform(
            frame_id="coordinator",
            child_frame_id="camera_link",
        ),
    )


def make_a1z_teach_blueprint(
    db_path: Path,
    *,
    task_label: str | None = None,
    camera_index: int = 0,
    gripper_free_drive: bool = False,
) -> Blueprint:
    """Record camera and measured arm/gripper state while hand-drivable.

    The arm is always hand-drivable (zero gravity). The gripper defaults to
    powered position control so it can be opened/closed from the keyboard,
    keeping the operator's hand out of the wrist camera; pass
    gripper_free_drive=True for the legacy pinch-by-hand behavior.
    """
    hardware = galaxea_a1z_hardware(
        "arm",
        gripper=True,
        dynamics_urdf_path=_A1Z_DYNAMICS_URDF,
        adapter_kwargs={
            "zero_gravity": True,
            "gripper_free_drive": gripper_free_drive,
        },
    )
    return autoconnect(
        ControlCoordinator.blueprint(hardware=[hardware], tasks=[]),
        EpisodeMonitorModule.blueprint(default_task_label=task_label),
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
    hardware = galaxea_a1z_hardware(
        "arm",
        gripper=True,
        dynamics_urdf_path=_A1Z_DYNAMICS_URDF,
    )
    return autoconnect(
        ControlCoordinator.blueprint(
            hardware=[hardware],
            tasks=[
                TaskConfig(
                    name=A1Z_REPLAY_TASK_NAME,
                    type="trajectory",
                    joint_names=hardware.all_joints,
                    priority=10,
                )
            ],
        )
    )


def make_a1z_policy_blueprint(
    policy_path: str,
    *,
    task: str = "",
    camera_index: int = 0,
    device: str | None = None,
    fps: float = A1Z_TEACH_CAMERA_FPS,
) -> Blueprint:
    """Run one trained LeRobot policy against the live A1Z camera and state."""
    return make_a1z_learned_policy_blueprint(
        policies={
            "default": LeRobotPolicyConfig(
                policy_path=policy_path,
                task=task,
                device=device,
            )
        },
        camera_index=camera_index,
        fps=fps,
    )


def make_a1z_learned_policy_blueprint(
    policies: dict[str, LeRobotPolicyConfig],
    *,
    policy_module: type[LeRobotPolicyModule] = LeRobotPolicyModule,
    camera_index: int = 0,
    fps: float = A1Z_TEACH_CAMERA_FPS,
) -> Blueprint:
    """Compose an A1Z stack exposing a catalog of trained LeRobot policies.

    Pass a ``LeRobotPolicyModule`` subclass with named ``@skill`` methods to
    expose task-specific tools to an agent. The default module instead exposes
    the generic ``execute_learned_policy(policy_name, duration)`` skill.
    """
    hardware = galaxea_a1z_hardware(
        "arm",
        gripper=True,
        dynamics_urdf_path=_A1Z_DYNAMICS_URDF,
    )
    return autoconnect(
        ControlCoordinator.blueprint(
            hardware=[hardware],
            tasks=[
                TaskConfig(
                    name=A1Z_POLICY_TASK_NAME,
                    type="servo",
                    joint_names=hardware.all_joints,
                    priority=10,
                    params={"timeout": max(1.0, 3.0 / fps)},
                )
            ],
        ),
        policy_module.blueprint(
            policies=policies,
            joint_names=hardware.all_joints,
            fps=fps,
            robot_type="galaxea_a1z",
        ),
        _a1z_camera(camera_index),
    )
