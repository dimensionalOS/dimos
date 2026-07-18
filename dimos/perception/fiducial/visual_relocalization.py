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
    # 3.0 px is an engineering guess (untuned), not calibrated against real
    # footage: loose enough for a clean detection's residual, tight enough to
    # reject a grossly wrong solve.
    max_reprojection_error_px: float = 3.0
    # IPPE mirror-ambiguity gate: the best PnP candidate is only trusted when it
    # beats the runner-up by this reprojection-error ratio. Near-tied candidates
    # mean the flipped mirror pose explains the pixels almost as well (weak
    # perspective: small/near-head-on tag) and neither can be trusted; 1.0
    # disables the gate. 5.0 TESTED on the hk_village3 replay (n=112 live fixes,
    # 164 detections passing the old 2.0 gate): this ratio was the only live
    # signal tracking world-frame tag-orientation error (Spearman rho -0.57) —
    # the error a 31 m tag-to-map-origin lever arm amplifies into ~0.55 m of fix
    # translation per degree. At 5.0 the gate kept 41/112 fixes, median tag-
    # orientation deviation 2.4 deg kept vs 5.8 deg cut, fix translation error
    # 2.38 m kept vs 3.35 m cut, coverage retained in all 5 sighting bursts.
    # Tuned on that ONE recording; untuned beyond it.
    ambiguity_ratio_min: float = 5.0


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
    reject_counts: dict[str, int] | None = None,
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

    ``reject_counts``, when given, is filled with per-tag skip reasons
    (``unmapped_id`` / ``mirror_ambiguous`` / ``high_reprojection``) so the
    caller's log can say WHICH gate fired — each reason has a different
    operator fix (see VisualRelocalizationModule.handle_color_image).
    """
    rejects = reject_counts if reject_counts is not None else {}
    k, d = camera_info_to_cv_matrices(camera_info)
    # distortion_model must reach the PnP helpers: the Go2 camera is
    # equidistant fisheye, and without the model the 4 fisheye coefficients
    # get misread as radtan k1,k2,p1,p2 — poses solved against the wrong
    # lens. marker_detect.py passes it through for the same reason.
    model = camera_info.distortion_model
    good: list[tuple[float, Transform]] = []
    for marker_id, corners_px in detections:
        if (map_T_marker := marker_map.get(marker_id)) is None:
            rejects["unmapped_id"] = rejects.get("unmapped_id", 0) + 1
            continue
        candidates = estimate_marker_pose_candidates(
            corners_px, config.marker_length_m, k, d, distortion_model=model
        )
        if not candidates:
            continue
        scored = sorted(
            (
                (
                    marker_reprojection_error(
                        corners_px,
                        config.marker_length_m,
                        k,
                        d,
                        rvec,
                        tvec,
                        distortion_model=model,
                    ),
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
                # mirror pose explains the pixels almost as well: untrustworthy view
                rejects["mirror_ambiguous"] = rejects.get("mirror_ambiguous", 0) + 1
                continue
        if error > config.max_reprojection_error_px:
            rejects["high_reprojection"] = rejects.get("high_reprojection", 0) + 1
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
    if not np.all(np.isfinite(np.asarray(translation, dtype=np.float64))):
        raise ValueError(f"marker {marker_id}: translation must be finite, got {translation!r}")
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
