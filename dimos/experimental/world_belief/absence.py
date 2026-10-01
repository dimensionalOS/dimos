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

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from dimos_generated.geometry_msgs.msg import TransformStamped, Vector3
    from dimos_generated.sensor_msgs.msg import CameraInfo
    from numpy.typing import NDArray

from dimos.msgs.camera_info import intrinsic_matrix
from dimos.msgs.geometry import inverse_transform, transform_matrix

OUT_OF_VIEW = "out_of_view"
PRESENT = "present"
ABSENT = "absent"
OCCLUDED = "occluded"

_NEAR_M = 0.05  # Sensor unreliable within 5cm of camera; too close to measure
_FREE_MARGIN_M = 0.03  # Tolerance for sensor noise and aiming error
_PATCH_PX = 2  # 5x5 median depth patch


def classify_visibility(
    center_world: Vector3,
    camera_info: CameraInfo,
    world_from_camera: TransformStamped,
    depth_m: NDArray[np.float32],
    *,
    near_extent_m: float = 0.0,
) -> str:
    """Classify a world point as PRESENT, ABSENT, OCCLUDED, or OUT_OF_VIEW.
    ``near_extent_m`` widens the expected PRESENT depth band."""
    cam_from_world = inverse_transform(world_from_camera)
    matrix = transform_matrix(cam_from_world.transform)
    x, y, z = (matrix @ np.array([center_world.x, center_world.y, center_world.z, 1.0]))[:3]
    if z <= _NEAR_M:  # Too close to camera; sensor is unreliable
        return OUT_OF_VIEW

    k = intrinsic_matrix(camera_info)
    u = k[0, 0] * x / z + k[0, 2]
    v = k[1, 1] * y / z + k[1, 2]
    h, w = depth_m.shape[:2]
    guard = _PATCH_PX + 3
    if not (guard <= u < w - guard and guard <= v < h - guard):
        return OUT_OF_VIEW  # Near image edge; patch would be outside frame

    ui = min(max(round(float(u)), 0), w - 1)
    vi = min(max(round(float(v)), 0), h - 1)
    u0, u1 = max(ui - _PATCH_PX, 0), min(ui + _PATCH_PX + 1, w)
    v0, v1 = max(vi - _PATCH_PX, 0), min(vi + _PATCH_PX + 1, h)
    patch = depth_m[v0:v1, u0:u1]
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return OCCLUDED  # No depth data; assume blocked (safer than guessing see-through)
    # Absence requires every valid sample to exceed the expected depth.
    if float(valid.min()) > z + _FREE_MARGIN_M:
        return ABSENT
    d = float(np.median(valid))
    if d < z - (near_extent_m + _FREE_MARGIN_M):
        return OCCLUDED  # Depth too close; something is blocking the object
    return PRESENT  # Depth is consistent with object being there
