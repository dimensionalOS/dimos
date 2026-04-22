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

"""Visual servoing blueprints: ArUco tracking + CartesianIK control.

Wires RealSense camera, ArUco tracker, and ControlCoordinator with a
CartesianIKTask for eye-in-hand visual servoing.

The servo loop:
    RealSense → ArucoTracker → PoseStamped → Coordinator.cartesian_command
                                              → CartesianIKTask (IK)
                                              → JointCommandOutput → hardware

TF tree (eye-in-hand):
    base_link → (coordinator joint_state FK) → ee_link → camera_link
    → camera_color_optical_frame → aruco_avg

Usage:
    # Camera-only tracking (no arm movement):
    dimos run aruco-tracker

    # Full visual servoing with mock arm (for testing):
    dimos run aruco-servo-mock

    # Full visual servoing with XArm6:
    dimos run aruco-servo-xarm6
"""

from __future__ import annotations

import math

import cv2

from dimos.control.components import HardwareComponent, HardwareType, make_joints
from dimos.control.coordinator import TaskConfig, control_coordinator
from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.realsense import realsense_camera
from dimos.manipulation.dynamic_tracking.aruco_tracker import aruco_tracker
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import CameraInfo, JointState
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.data import LfsPath

# =============================================================================
# Model paths for IK
# =============================================================================

_XARM6_MODEL_PATH = LfsPath("xarm_description/urdf/xarm6/xarm6.urdf")

# =============================================================================
# Camera calibration (EEF → camera transform)
# =============================================================================
# Default eye-in-hand calibration for xArm6 + RealSense D435.
# To re-calibrate, run:
#   python -m dimos.manipulation.dynamic_tracking.calibrate_hand_eye \
#       --arm-ip 192.168.1.210 --output calibration.json
# Then load with calibration_to_transform("calibration.json").

_DEFAULT_EEF_TO_CAMERA = Transform(
    translation=Vector3(0.06693724, -0.0309563, 0.00691482),
    rotation=Quaternion(0.70513398, 0.00535696, 0.70897578, -0.01052180),  # xyzw
)

# =============================================================================
# Camera-only: ArUco tracker with RealSense (no arm control)
# =============================================================================
# Detects markers and publishes TF transforms. No servo output.
# Useful for verifying detection, calibration, and TF tree.

aruco_tracker_realsense = (
    autoconnect(
        realsense_camera(
            width=848,
            height=480,
            fps=15,
            camera_name="camera",
            base_frame_id="ee_link",
            base_transform=_DEFAULT_EEF_TO_CAMERA,
            enable_depth=True,
            align_depth_to_color=False,
        ),
        aruco_tracker(
            marker_size=0.015,
            aruco_dict=cv2.aruco.DICT_4X4_50,
            expected_marker_count=4,
            camera_frame_id="camera_color_optical_frame",
            rate=15,
            enable_servo=False,
        ),
    )
    .transports(
        {
            ("color_image", Image): LCMTransport("/camera/color", Image),
            ("camera_info", CameraInfo): LCMTransport("/camera/color_info", CameraInfo),
            ("annotated_image", Image): LCMTransport("/aruco/annotated", Image),
        }
    )
    .global_config(viewer_backend="rerun-native")
)

# =============================================================================
# Visual servoing with mock arm (for testing without hardware)
# =============================================================================
# Full servo loop: ArUco detection → CartesianIK → mock arm.
# Camera is real (RealSense); arm is simulated.

aruco_servo_mock = (
    autoconnect(
        control_coordinator(
            tick_rate=100.0,
            publish_joint_state=True,
            joint_state_frame_id="coordinator",
            hardware=[
                HardwareComponent(
                    hardware_id="arm",
                    hardware_type=HardwareType.MANIPULATOR,
                    joints=make_joints("arm", 6),
                    adapter_type="mock",
                ),
            ],
            tasks=[
                TaskConfig(
                    name="cartesian_ik_arm",
                    type="cartesian_ik",
                    joint_names=[f"arm_joint{i + 1}" for i in range(6)],
                    priority=10,
                    model_path=_XARM6_MODEL_PATH,
                    ee_joint_id=6,
                ),
            ],
        ),
        realsense_camera(
            width=848,
            height=480,
            fps=15,
            camera_name="camera",
            base_frame_id="ee_link",
            base_transform=_DEFAULT_EEF_TO_CAMERA,
            enable_depth=True,
            align_depth_to_color=False,
        ),
        aruco_tracker(
            marker_size=0.015,
            aruco_dict=cv2.aruco.DICT_4X4_50,
            expected_marker_count=4,
            camera_frame_id="camera_color_optical_frame",
            rate=10,
            enable_servo=True,
            target_task_name="cartesian_ik_arm",
            reach_offset=[0.0, 0.0, 0.10],
            reach_orientation_rpy=[math.pi, 0.0, 0.0],
            min_move_distance_m=0.003,
        ),
    )
    .transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("cartesian_command", PoseStamped): LCMTransport(
                "/coordinator/cartesian_command", PoseStamped
            ),
            ("color_image", Image): LCMTransport("/camera/color", Image),
            ("camera_info", CameraInfo): LCMTransport("/camera/color_info", CameraInfo),
            ("annotated_image", Image): LCMTransport("/aruco/annotated", Image),
        }
    )
    .global_config(viewer_backend="rerun-native")
)


# =============================================================================
# Visual servoing with XArm6 (real hardware)
# =============================================================================
# Full servo loop: ArUco detection → CartesianIK → XArm6.
# Both camera and arm are real hardware.

aruco_servo_xarm6 = (
    autoconnect(
        control_coordinator(
            tick_rate=100.0,
            publish_joint_state=True,
            joint_state_frame_id="coordinator",
            hardware=[
                HardwareComponent(
                    hardware_id="arm",
                    hardware_type=HardwareType.MANIPULATOR,
                    joints=make_joints("arm", 6),
                    adapter_type="xarm",
                    address="192.168.1.210",
                    auto_enable=True,
                ),
            ],
            tasks=[
                TaskConfig(
                    name="cartesian_ik_arm",
                    type="cartesian_ik",
                    joint_names=[f"arm_joint{i + 1}" for i in range(6)],
                    priority=10,
                    model_path=_XARM6_MODEL_PATH,
                    ee_joint_id=6,
                ),
            ],
        ),
        realsense_camera(
            width=848,
            height=480,
            fps=15,
            camera_name="camera",
            base_frame_id="ee_link",
            base_transform=_DEFAULT_EEF_TO_CAMERA,
            enable_depth=True,
            align_depth_to_color=False,
        ),
        aruco_tracker(
            marker_size=0.015,
            aruco_dict=cv2.aruco.DICT_4X4_50,
            expected_marker_count=4,
            camera_frame_id="camera_color_optical_frame",
            rate=4,
            enable_servo=True,
            target_task_name="cartesian_ik_arm",
            reach_offset=[0.0, 0.0, 0.10],
            reach_orientation_rpy=[math.pi, 0.0, 0.0],
            min_move_distance_m=0.005,
        ),
    )
    .transports(
        {
            ("joint_state", JointState): LCMTransport("/coordinator/joint_state", JointState),
            ("cartesian_command", PoseStamped): LCMTransport(
                "/coordinator/cartesian_command", PoseStamped
            ),
            ("color_image", Image): LCMTransport("/camera/color", Image),
            ("camera_info", CameraInfo): LCMTransport("/camera/color_info", CameraInfo),
            ("annotated_image", Image): LCMTransport("/aruco/annotated", Image),
        }
    )
    .global_config(viewer_backend="rerun-native")
)


__all__ = [
    "aruco_servo_mock",
    "aruco_servo_xarm6",
    "aruco_tracker_realsense",
]
