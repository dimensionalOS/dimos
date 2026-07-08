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

"""Geometric evidence-of-absence: classify a believed point against a depth frame as
OUT_OF_VIEW (no evidence), ABSENT (seen through), OCCLUDED (blocked), or PRESENT."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

OUT_OF_VIEW = "out_of_view"
PRESENT = "present"
ABSENT = "absent"
OCCLUDED = "occluded"

_NEAR_M = 0.05  # Sensor unreliable within 5cm of camera; too close to measure
_FREE_MARGIN_M = 0.03  # Tolerance for sensor noise and aiming error
_PATCH_PX = 2  # Median over 5×5 patch to smooth sensor speckle and improve robustness


def classify_visibility(
    center_world: Vector3,
    camera_info: CameraInfo,
    world_from_camera: Transform,
    depth_m: NDArray[np.float32],
    *,
    near_extent_m: float = 0.0,
) -> str:
    """Project belief point to camera image, read median depth in a local patch, and
    classify as PRESENT, ABSENT (see-through), OCCLUDED (blocked), or OUT_OF_VIEW.
    ``near_extent_m``: tolerance for object's own depth extent (half-depth)."""
    cam_from_world = world_from_camera.inverse()
    p = cam_from_world.rotation.rotate_vector(center_world) + cam_from_world.translation
    x, y, z = p.x, p.y, p.z
    if z <= _NEAR_M:  # Too close to camera; sensor is unreliable
        return OUT_OF_VIEW

    k = camera_info.get_K_matrix()
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
    # ABSENT only if ALL depths are way beyond expected (by >3cm tolerance)
    if float(valid.min()) > z + _FREE_MARGIN_M:
        return ABSENT
    d = float(np.median(valid))
    if d < z - (near_extent_m + _FREE_MARGIN_M):
        return OCCLUDED  # Depth too close; something is blocking the object
    return PRESENT  # Depth is consistent with object being there
