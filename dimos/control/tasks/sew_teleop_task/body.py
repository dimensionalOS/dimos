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

"""Explicit WebXR body convention adapter; no enable-time arm offsets."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.manipulation.planning.kinematics.sew_retargeting import SewArmTarget, unit
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot


@dataclass(frozen=True)
class BodyTarget:
    arms: dict[str, SewArmTarget]
    chest_rotation: np.ndarray
    height: float


def adapt_body(snapshot: BodyTrackingSnapshot) -> BodyTarget:
    if snapshot.frame_id not in ("local-floor", "bounded-floor") or not snapshot.joints:
        raise ValueError("Body source unavailable or unsupported reference space")
    joints = snapshot.joints
    required = ["hips"] + [
        f"{s}-{n}"
        for s in ("left", "right")
        for n in ("arm-upper", "arm-lower", "hand-wrist", "hand-palm")
    ]
    missing = [n for n in required if n not in joints]
    if missing:
        raise ValueError("Missing body joints: " + ", ".join(missing))
    p = {n: np.asarray(joints[n].position) for n in required}
    center = (p["left-arm-upper"] + p["right-arm-upper"]) / 2
    y = unit(p["left-arm-upper"] - p["right-arm-upper"])
    x = unit(np.cross(y, center - p["hips"]))
    z = unit(np.cross(x, y))
    body = np.column_stack([x, y, z])
    arms = {}
    # WebXR palm: -Z points distal, -Y points out of palm. Robot tool uses
    # +X forward, +Y left, +Z up; palm at neutral points down robot -Z.
    hand_axes = np.column_stack([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    for side in ("left", "right"):
        quat = np.asarray(joints[f"{side}-hand-palm"].orientation)
        if abs(np.linalg.norm(quat) - 1) > 0.01:
            raise ValueError("Invalid palm quaternion")
        hand = body.T @ Rotation.from_quat(quat).as_matrix() @ hand_axes
        points = [
            body.T @ (p[f"{side}-{n}"] - center) for n in ("arm-upper", "arm-lower", "hand-wrist")
        ]
        arms[side] = SewArmTarget(points[0], points[1], points[2], hand)
    webxr_to_world = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    return BodyTarget(arms, webxr_to_world @ body, float(center[1]))
