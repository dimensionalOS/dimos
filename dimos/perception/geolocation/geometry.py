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

"""Pixel -> camera ray -> gimbal -> vehicle/NED line of sight, and ground intersection.

Frames (all right-handed): camera x = optical axis, y = right in image, z = down in
image; NED x = North, y = East, z = Down. Euler convention is aerospace ZYX with no roll,
R = Rz(yaw) @ Ry(pitch), pitch positive = camera up, yaw positive = clockwise.
Angles are degrees throughout; callers convert at the edge.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.utils.transform_utils import normalize_angle


@dataclass(frozen=True)
class GimbalFrameConfig:
    """What the gimbal's yaw is measured from: the vehicle's nose (body) or north (earth)."""

    yaw_frame: Literal["body", "earth"] = "body"


class CameraModel:
    """Pinhole. Pixels must be those of the frames ``info`` describes; nothing here sees an
    image, so a stream resized under it goes unnoticed."""

    def __init__(self, info: CameraInfo) -> None:
        self.fx, self.fy = float(info.K[0]), float(info.K[4])
        self.cx, self.cy = float(info.K[2]), float(info.K[5])

    def pixel_to_ray(self, u: float, v: float) -> NDArray[np.float64]:
        """Unit ray in the camera frame (x forward, y right, z down)."""
        r = np.array([1.0, (u - self.cx) / self.fx, (v - self.cy) / self.fy])
        return np.asarray(r / np.linalg.norm(r))


def ned_to_az_el(v: NDArray[np.float64]) -> tuple[float, float]:
    """Azimuth (0=N, 90=E) and elevation (+ = above horizon) of a NED vector, degrees."""
    n, e, d = (float(x) for x in v)
    az = (math.degrees(math.atan2(e, n)) + 360.0) % 360.0
    el = math.degrees(math.atan2(-d, math.hypot(n, e)))
    return az, el


@dataclass(frozen=True)
class LosSolution:
    los_ned: tuple[float, float, float]
    azimuth_deg: float
    elevation_deg: float
    gimbal_yaw_body_deg: float


class LOSSolver:
    """Combines camera model, gimbal attitude and vehicle heading into a NED line of sight."""

    def __init__(self, camera: CameraModel, frame_cfg: GimbalFrameConfig | None = None) -> None:
        self.cam = camera
        self.cfg = frame_cfg or GimbalFrameConfig()

    def solve(
        self,
        u: float,
        v: float,
        gimbal_pitch_deg: float,
        gimbal_yaw_deg: float,
        vehicle_yaw_deg: float,
    ) -> LosSolution:
        """Pixel to line of sight. The gimbal is taken as roll-stabilised: no camera roll."""
        ray_cam = self.cam.pixel_to_ray(u, v)
        r_g = Rotation.from_euler(
            "ZY", [gimbal_yaw_deg, gimbal_pitch_deg], degrees=True
        ).as_matrix()
        if self.cfg.yaw_frame == "body":
            r_v = Rotation.from_euler("z", vehicle_yaw_deg, degrees=True).as_matrix()
            los = r_v @ r_g @ ray_cam
            yaw_body = gimbal_yaw_deg
        else:
            los = r_g @ ray_cam
            yaw_body = gimbal_yaw_deg - vehicle_yaw_deg
        az, el = ned_to_az_el(los)
        return LosSolution(
            los_ned=(float(los[0]), float(los[1]), float(los[2])),
            azimuth_deg=az,
            elevation_deg=el,
            gimbal_yaw_body_deg=math.degrees(normalize_angle(math.radians(yaw_body))),
        )


def intersect_ground(
    p_ned: tuple[float, float, float],
    los_ned: tuple[float, float, float],
    ground_d: float,
    min_depression_deg: float = 5.0,
) -> tuple[tuple[float, float, float] | None, float | str]:
    """Intersect ray p + s*los with the horizontal plane D = ground_d.

    Returns (point_ned, range_m) or (None, reason). Rejects rays that look up or are
    shallower than min_depression_deg (range explodes).
    """
    p = np.asarray(p_ned, dtype=float)
    r = np.asarray(los_ned, dtype=float)
    _, el = ned_to_az_el(r)
    if el > -min_depression_deg:
        return None, f"elevation {el:.1f} deg too shallow"
    if ground_d <= p[2]:
        return None, "camera at/below ground plane"
    s = (ground_d - p[2]) / r[2]
    pt = p + s * r
    return (float(pt[0]), float(pt[1]), float(pt[2])), float(s)
