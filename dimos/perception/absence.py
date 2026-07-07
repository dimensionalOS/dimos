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

"""Geometric evidence-of-absence: classify a believed object against a depth frame.

Given a believed position, the camera pose + intrinsics, and the measured depth, return
OUT_OF_VIEW (position not in frame — the frame carries no evidence; do not decay),
ABSENT (the depth ray reaches a *farther* surface — we see through its spot, it is
gone), OCCLUDED (a *nearer* surface or no reading blocks the view — inconclusive), or
PRESENT (a surface at roughly the expected depth). 
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

OUT_OF_VIEW = "out_of_view"
PRESENT = "present"
ABSENT = "absent"
OCCLUDED = "occluded"

_NEAR_M = 0.05  # z at or behind this is on/behind the image plane → no evidence
_FREE_MARGIN_M = 0.03  # need only clear sensor noise + center error (see docstring)
_PATCH_PX = 2  # median over a (2*_PATCH_PX+1)² patch — depth holes/speckle flip pixels


def classify_visibility(
    center_world: Vector3,
    camera_info: CameraInfo,
    world_from_camera: NDArray[np.float64],
    depth_m: NDArray[np.float32],
    *,
    near_extent_m: float = 0.0,
) -> str:
    """Classify a believed world-frame point against one depth frame.

    ``world_from_camera`` is the 4x4 that maps camera coords → world (i.e.
    ``camera_transform.to_matrix()``). ``depth_m`` is float32 metres, shape (H, W).
    The depth at the projected point is the **median of a ``(2*patch_px+1)`` square** of
    valid (finite, >0) readings — real depth sensors have holes and speckle, so a single
    pixel can flip the verdict. Returns one of OUT_OF_VIEW / ABSENT / OCCLUDED / PRESENT.

    ``free_margin_m`` need only clear sensor noise + center-estimation error, NOT the
    distance to the background: an opaque object's depth reading is its NEAR surface, so
    a present object always measures ``d <= expected + noise`` — any reading beyond that
    means the near surface is gone. On a real tabletop the support surface sits only
    4-13 cm behind a can's believed center (measured on the arm), so a generous margin
    silently classifies every removed object as PRESENT and resets its absence votes.

    Robustness (adversarial-probe hardened):
    - ABSENT requires EVERY valid patch pixel to see through (min > expected + margin) —
      a believed center sitting 1 px outside the silhouette must not read the background
      median and kill an object that is plainly in frame.
    - ``near_extent_m`` extends the PRESENT band toward the camera by the object's own
      half-depth, so a deep object's near surface (e.g. a bottle read 5 cm in front of
      its center) still counts as "surface present" and resets absence votes.
    - A ``border_guard_px`` band (default ``patch_px + 3``) around the image border is
      OUT_OF_VIEW: mid-pan, cm-level pose error can place the believed center fractionally
      inside the border while the object is fully outside — no evidence either way.
    """
    cam_from_world = np.linalg.inv(np.asarray(world_from_camera, dtype=np.float64))
    pw = np.array(
        [float(center_world.x), float(center_world.y), float(center_world.z), 1.0],
        dtype=np.float64,
    )
    x, y, z = (cam_from_world @ pw)[:3]
    if z <= _NEAR_M:  # behind the camera (or on the image plane) → no evidence
        return OUT_OF_VIEW

    k = camera_info.get_K_matrix()
    u = k[0, 0] * x / z + k[0, 2]
    v = k[1, 1] * y / z + k[1, 2]
    h, w = depth_m.shape[:2]
    guard = _PATCH_PX + 3
    if not (guard <= u < w - guard and guard <= v < h - guard):
        return OUT_OF_VIEW  # border band: object may be fully outside; no evidence

    ui = min(max(round(float(u)), 0), w - 1)
    vi = min(max(round(float(v)), 0), h - 1)
    u0, u1 = max(ui - _PATCH_PX, 0), min(ui + _PATCH_PX + 1, w)
    v0, v1 = max(vi - _PATCH_PX, 0), min(vi + _PATCH_PX + 1, h)
    patch = depth_m[v0:v1, u0:u1]
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size == 0:
        return OCCLUDED  # no valid reading → inconclusive, never conclude absence
    # See-through only when EVERY valid pixel is beyond the expected depth — a center
    # projected just outside the silhouette must not average the background into ABSENT.
    if float(valid.min()) > z + _FREE_MARGIN_M:
        return ABSENT
    d = float(np.median(valid))
    if d < z - (near_extent_m + _FREE_MARGIN_M):
        return OCCLUDED  # nearer than the object's own near surface could be → blocked
    return PRESENT  # a surface within the object's own extent → it (or its face) is there
