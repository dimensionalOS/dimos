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


"""Explicit YAM observation and action boundaries; no driver or network access."""

import base64
from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS

ACTION_HZ = 20.0
HORIZON = 48
CAMERAS = ("overhead", "wrist_l", "wrist_r")


def state_vector(state: JointState) -> NDArray[np.float64]:
    if len(state.name) != len(state.position) or len(set(state.name)) != len(state.name):
        raise ValueError("ambiguous joint state")
    positions = dict(zip(state.name, state.position, strict=True))
    if not set(DUAL_OPENYAM_JOINTS) <= positions.keys():
        raise ValueError("joint state lacks dual YAM joints or grippers")
    vector = np.array([positions[name] for name in DUAL_OPENYAM_JOINTS], dtype=np.float64)
    if not np.isfinite(vector).all() or np.any(vector[12:] < 0) or np.any(vector[12:] > 1):
        raise ValueError("invalid joint state or normalized gripper")
    return vector


def observation(vector: NDArray[np.float64], prompt: str) -> dict[str, Any]:
    return {
        "state.left_arm": vector[:6].astype(np.float32),
        "state.right_arm": vector[6:12].astype(np.float32),
        "state.left_gripper": vector[12:13].astype(np.float32),
        "state.right_gripper": vector[13:14].astype(np.float32),
        "prompt": prompt,
    }


def encode_image(image: Image) -> str:
    if image.format != ImageFormat.RGB or image.data.dtype != np.uint8:
        raise ValueError("Dreamlabs requires uint8 RGB images")
    if image.data.ndim != 3 or image.data.shape[2] != 3 or not image.data.size:
        raise ValueError("invalid RGB image shape")
    rgb = cv2.resize(image.data, (320, 176), interpolation=cv2.INTER_AREA)
    ok, jpeg = cv2.imencode(
        ".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85]
    )
    if not ok:
        raise ValueError("JPEG encoding failed")
    return base64.b64encode(jpeg).decode("ascii")


def validate_times(stamps: list[float], now: float, max_age: float, max_skew: float) -> None:
    times = np.asarray(stamps)
    if not np.isfinite(times).all() or np.any(times > now):
        raise ValueError("non-finite or future observation timestamp")
    if now - min(stamps) > max_age or max(stamps) - min(stamps) > max_skew:
        raise ValueError("stale or unsynchronized observations")


def action_trajectory(
    reply: dict[str, Any],
    start: NDArray[np.float64],
    lower: NDArray[np.float64],
    upper: NDArray[np.float64],
    max_joint_speed: float,
    max_gripper_speed: float,
) -> JointTrajectory:
    left = np.asarray(reply.get("q_l_chunk"), dtype=np.float64)
    right = np.asarray(reply.get("q_r_chunk"), dtype=np.float64)
    if left.shape != (HORIZON, 7) or right.shape != (HORIZON, 7):
        raise ValueError("expected two 48 by 7 YAM action chunks")
    rows = np.column_stack((left[:, :6], right[:, :6], left[:, 6], right[:, 6]))
    if not np.isfinite(rows).all() or np.any(rows < lower) or np.any(rows > upper):
        raise ValueError("action outside finite joint or gripper bounds")
    points = np.vstack((start, rows))
    speed = np.abs(np.diff(points, axis=0)) * ACTION_HZ
    if np.any(speed[:, :12] > max_joint_speed) or np.any(speed[:, 12:] > max_gripper_speed):
        raise ValueError("action exceeds speed bound, including chunk seam")
    return JointTrajectory(
        joint_names=list(DUAL_OPENYAM_JOINTS),
        points=[
            TrajectoryPoint(
                positions=row.tolist(), velocities=[0.0] * 14, time_from_start=i / ACTION_HZ
            )
            for i, row in enumerate(points)
        ],
    )


def policy_config(checkpoint: str, timeout_ms: int) -> dict[str, Any]:
    parts = {
        "right_arm": [0, 6],
        "right_gripper": [6, 7],
        "left_arm": [7, 13],
        "left_gripper": [13, 14],
        "right_eef_xyz": [14, 17],
        "right_eef_rot6": [17, 23],
        "right_grip": [23, 24],
        "left_eef_xyz": [24, 27],
        "left_eef_rot6": [27, 33],
        "left_grip": [33, 34],
    }
    return {
        "checkpoint": checkpoint,
        "strict": True,
        "timeout_ms": timeout_ms,
        "action_hz": ACTION_HZ,
        "robot": "yam",
        "obs_window": 9,
        "obs_fps": 5.0,
        # SDK 1.0.1 otherwise sends a second, background recording to the platform.
        "_record": False,
        "embodiment": {
            "state": parts,
            # Joint chunks already contain the normalized grippers. A YAM model's
            # per_key uses either grip or gripper; validate both full chunks locally.
            "action": {"right_arm": [0, 6], "left_arm": [6, 12]},
            "video": {role: f"observation.images.{role}" for role in CAMERAS},
        },
    }
