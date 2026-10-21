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

from typing import Any

import cv2
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

ACTION_HZ = 20.0
HORIZON = 48
CAMERAS = ("overhead", "wrist_l", "wrist_r")
MAX_JOINT_OVERSHOOT_RAD = np.deg2rad(2.0)


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
    return image.resize(320, 176, interpolation=cv2.INTER_AREA).to_base64(quality=85)


def action_trajectory(
    reply: dict[str, Any],
    start: NDArray[np.float64],
    joint_bounds: dict[str, tuple[float, float]],
) -> JointTrajectory:
    left = np.asarray(reply.get("q_l_chunk"), dtype=np.float64)
    right = np.asarray(reply.get("q_r_chunk"), dtype=np.float64)
    if left.shape != (HORIZON, 7) or right.shape != (HORIZON, 7):
        raise ValueError("expected two 48 by 7 YAM action chunks")
    rows = np.column_stack((left[:, :6], right[:, :6], left[:, 6], right[:, 6]))
    for label, values in (("arm", rows[:, :12]), ("gripper", rows[:, 12:])):
        invalid = np.argwhere(~np.isfinite(values))
        if invalid.size:
            raise ValueError(f"action contains non-finite {label} targets at {invalid.tolist()}")
    # Bound model output only; the measured anchor must remain an actual observation.
    # Validate the whole chunk before applying any corrections.
    for column, name in enumerate(DUAL_OPENYAM_JOINTS[:12]):
        lower, upper = joint_bounds[name]
        if not np.isfinite([lower, upper]).all() or lower >= upper:
            raise ValueError(f"invalid model limits for {name}")
        values = rows[:, column]
        excess = np.maximum(lower - values, values - upper)
        if np.any(excess > MAX_JOINT_OVERSHOOT_RAD):
            raise ValueError(
                f"{name} targets exceed model limits by more than 2 degrees: "
                f"maximum excess {np.degrees(excess.max()):.6f} degrees"
            )
    for column, name in enumerate(DUAL_OPENYAM_JOINTS[:12]):
        lower, upper = joint_bounds[name]
        values = rows[:, column]
        indices = np.flatnonzero((values < lower) | (values > upper))
        if indices.size:
            corrected = np.clip(values, lower, upper)
            logger.warning(
                "Clipping small Dreamlabs joint overshoot to model limits",
                joint=name,
                count=int(indices.size),
                lower=lower,
                upper=upper,
                max_correction_rad=float(np.max(np.abs(corrected - values))),
                indices=indices.tolist(),
                raw_values=values[indices].tolist(),
            )
            rows[:, column] = corrected
    for side, column in (("left", 12), ("right", 13)):
        opening = rows[:, column]
        indices = np.flatnonzero((opening < 0) | (opening > 1))
        if indices.size:
            logger.warning(
                "Clipping finite Dreamlabs gripper output to [0, 1]",
                arm=side,
                count=int(indices.size),
                raw_min=float(opening.min()),
                raw_max=float(opening.max()),
                indices=indices.tolist(),
                raw_values=opening[indices].tolist(),
            )
        rows[:, column] = np.clip(opening, 0.0, 1.0)
    if np.any(rows[:, 12:] < 0) or np.any(rows[:, 12:] > 1):
        raise ValueError("action contains non-normalized grippers after conversion")
    points = np.vstack((start, rows))
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
