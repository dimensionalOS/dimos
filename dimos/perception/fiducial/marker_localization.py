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

"""Pure-function marker localization core: solvePnP run in reverse against a known marker-map pose."""

from __future__ import annotations

from pathlib import Path
from typing import Any, NamedTuple

import numpy as np
import yaml

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.perception.fiducial.marker_pose import (
    camera_info_to_cv_matrices,
    estimate_marker_pose_candidates,
    marker_reprojection_error,
    rvec_tvec_to_transform,
)

MAP_FRAME, OPTICAL_FRAME = (
    "map",
    "camera_optical",
)  # single static camera; multi-camera is a follow-up


class LocalizationConfig(NamedTuple):
    marker_length_m: float
    min_tags: int = 1
    max_reprojection_error_px: float = 3.0
    # IPPE mirror-ambiguity gate: the best PnP candidate is only trusted when it
    # beats the runner-up by this reprojection-error ratio. Near-tied candidates
    # mean the flipped mirror pose explains the pixels almost as well (weak
    # perspective: small/near-head-on tag) and neither can be trusted. 1.0
    # disables the gate.
    ambiguity_ratio_min: float = 2.0


def detect_markers(gray_image: np.ndarray, detector: Any) -> list[tuple[int, np.ndarray]]:
    corners, ids, _ = detector.detectMarkers(gray_image)
    if ids is None or len(ids) == 0:
        return []
    return [(int(mid[0]), c.reshape(4, 2)) for c, mid in zip(corners, ids, strict=True)]


def localize_from_detections(
    detections: list[tuple[int, np.ndarray]],
    marker_map: dict[int, Transform],
    camera_info: CameraInfo,
    config: LocalizationConfig,
    ts: float,
) -> Transform | None:
    """Per known tag: invert solvePnP (``pose_in_map = map_T_marker . (optical_T_marker)^-1``).

    Solves with ``solvePnPGeneric`` (both IPPE candidates) and gates each tag two
    ways: the best candidate's absolute reprojection error, AND the mirror-
    ambiguity ratio — at weak perspective the flipped IPPE solution can reproject
    as well as the correct one, so a near-tied runner-up means the view is
    geometrically ambiguous and the tag is skipped (``config.ambiguity_ratio_min``).
    When multiple tags clear both gates, the lowest-reprojection-error estimate
    wins (never worse than the single best tag) instead of an arbitrary
    detection-order pick.
    """
    k, d = camera_info_to_cv_matrices(camera_info)
    good: list[tuple[float, Transform]] = []
    for marker_id, corners_px in detections:
        if (map_T_marker := marker_map.get(marker_id)) is None:
            continue
        candidates = estimate_marker_pose_candidates(corners_px, config.marker_length_m, k, d)
        if not candidates:
            continue
        scored = sorted(
            (
                (
                    marker_reprojection_error(corners_px, config.marker_length_m, k, d, rvec, tvec),
                    rvec,
                    tvec,
                )
                for rvec, tvec in candidates
            ),
            key=lambda item: item[0],
        )
        error, rvec, tvec = scored[0]
        if len(scored) > 1 and error > 1e-12:
            if scored[1][0] / error < config.ambiguity_ratio_min:
                continue  # mirror pose explains the pixels almost as well: untrustworthy view
        if error > config.max_reprojection_error_px:
            continue
        optical_T_marker = rvec_tvec_to_transform(
            rvec, tvec, frame_id=OPTICAL_FRAME, child_frame_id=map_T_marker.child_frame_id, ts=ts
        )
        good.append((error, map_T_marker + optical_T_marker.inverse()))
    if not good or len(good) < config.min_tags:
        return None
    return min(good, key=lambda item: item[0])[1]


def _validated_entry(marker_id: int, entry: dict[str, Any]) -> Transform:
    """Fail loudly on malformed values: a short translation list would otherwise silently
    zero-fill (``Vector3(1.0)`` -> ``(1.0, 0.0, 0.0)``) and a zero-norm quaternion only
    crashes much later, in ``Quaternion.inverse()`` — or worse, corrupts a published pose."""
    translation, rotation = entry["translation"], entry["rotation"]
    if not isinstance(translation, (list, tuple)) or len(translation) != 3:
        raise ValueError(f"marker {marker_id}: translation must be [x, y, z], got {translation!r}")
    if not isinstance(rotation, (list, tuple)) or len(rotation) != 4:
        raise ValueError(f"marker {marker_id}: rotation must be [x, y, z, w], got {rotation!r}")
    norm = float(np.linalg.norm(np.asarray(rotation, dtype=np.float64)))
    if not np.isfinite(norm) or norm < 1e-6:
        raise ValueError(f"marker {marker_id}: rotation quaternion norm is {norm}, not usable")
    return Transform(
        translation=Vector3(*translation),
        rotation=Quaternion(*rotation),
        frame_id=MAP_FRAME,
        child_frame_id=f"marker_{marker_id}",
    )


def load_marker_map(path: str | Path) -> dict[int, Transform]:
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return {
        int(marker_id): _validated_entry(int(marker_id), entry)
        for marker_id, entry in (data.get("markers", {}) or {}).items()
    }
