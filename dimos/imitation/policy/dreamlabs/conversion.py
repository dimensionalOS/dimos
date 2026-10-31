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

import numpy as np
from numpy.typing import NDArray

from dimos.imitation.policy.dreamlabs.constants import (
    ACTION_HZ,
    CAMERAS,
    HORIZON,
    MAX_JOINT_OVERSHOOT_RAD,
)
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_JOINTS
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def encode_image(image: Image) -> str:
    import cv2

    if image.format != ImageFormat.RGB or image.data.dtype != np.uint8:
        raise ValueError("Dreamlabs requires uint8 RGB images")
    if image.data.ndim != 3 or image.data.shape[2] != 3 or not image.data.size:
        raise ValueError("invalid RGB image shape")
    return image.resize(320, 176, interpolation=cv2.INTER_AREA).to_base64(quality=85)


def joint_actions(
    reply: dict[str, Any],
    joint_bounds: dict[str, tuple[float, float]],
) -> NDArray[np.float32]:
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
    return rows.astype(np.float32)


def policy_config(checkpoint: str, timeout_ms: int, mode: str = "joint") -> dict[str, Any]:
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
        "robot": "yam" if mode == "joint" else "other_embodiment",
        "obs_window": 9,
        "obs_fps": 5.0,
        # SDK 1.0.1 otherwise sends a second, background recording to the platform.
        "_record": False,
        "embodiment": {
            "state": parts,
            # Joint chunks already contain the normalized grippers. A YAM model's
            # per_key uses either grip or gripper; validate both full chunks locally.
            "action": (
                {"right_arm": [0, 6], "left_arm": [6, 12]}
                if mode == "joint"
                else {
                    "right_eef_xyz": [0, 3],
                    "right_eef_rot6": [3, 9],
                    "right_grip": [9, 10],
                    "left_eef_xyz": [10, 13],
                    "left_eef_rot6": [13, 19],
                    "left_grip": [19, 20],
                }
            ),
            "video": {role: f"observation.images.{role}" for role in CAMERAS},
        },
    }
