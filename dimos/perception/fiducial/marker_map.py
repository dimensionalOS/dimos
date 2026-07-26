#!/usr/bin/env python3
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

"""Survey marker map file I/O: ``map_T_tag`` per id plus the tag size it was solved at."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.fiducial.marker_aggregation import Pose7

MAP_FRAME = "map"
MARKER_MAP_SUFFIX = ".json"  # what write_marker_map emits and load_marker_map parses


def _validated_entry(marker_id: int, entry: dict[str, Any]) -> Transform:
    """Validate one survey entry into a ``map_T_marker`` Transform, failing loudly on a short translation or zero-norm quaternion."""
    translation, rotation = entry["translation"], entry["rotation"]
    if not isinstance(translation, (list, tuple)) or len(translation) != 3:
        raise ValueError(f"marker {marker_id}: translation must be [x, y, z], got {translation!r}")
    if not isinstance(rotation, (list, tuple)) or len(rotation) != 4:
        raise ValueError(f"marker {marker_id}: rotation must be [x, y, z, w], got {rotation!r}")
    if not np.all(np.isfinite(np.asarray(translation, dtype=np.float64))):
        raise ValueError(f"marker {marker_id}: translation must be finite, got {translation!r}")
    norm = float(np.linalg.norm(np.asarray(rotation, dtype=np.float64)))
    if (
        not np.isfinite(norm) or norm < 1e-6
    ):  # 1e-6: unnormalizable, floor below a unit quaternion's round-off
        raise ValueError(f"marker {marker_id}: rotation quaternion norm is {norm}, not usable")
    return Transform(
        translation=Vector3(*translation),
        rotation=Quaternion(*rotation),
        frame_id=MAP_FRAME,
        child_frame_id=f"marker_{marker_id}",
    )


def load_marker_map(path: str | Path) -> dict[int, Transform]:
    """``marker_id -> map_T_marker`` from a survey JSON (the format ``write_marker_map`` emits)."""
    data = json.loads(Path(path).read_text()) or {}
    return {
        int(marker_id): _validated_entry(int(marker_id), entry)
        for marker_id, entry in (data.get("markers", {}) or {}).items()
    }


def marker_length_m_from_map(path: str | Path) -> float | None:
    """The printed tag edge length the survey solved at, or ``None`` when the map carries none."""
    meta = (json.loads(Path(path).read_text()) or {}).get("meta") or {}
    size = meta.get("marker_length_m")
    if size is None:
        return None
    if float(size) <= 0:
        raise ValueError(f"{path}: marker_length_m must be > 0, got {size}")
    return float(size)


def write_marker_map(
    path: Path, aggregated: dict[int, tuple[Pose7, int]], *, source: str, marker_length_m: float
) -> None:
    """Serialize aggregated ``map_T_tag`` poses to the JSON ``load_marker_map`` reads."""
    doc = {
        "meta": {
            "schema": "map_T_tag",
            "source_recording": source,
            # every pose below is metric only at this tag size, so the live detector reads it back rather than being told twice
            "marker_length_m": marker_length_m,
            "n_detections_aggregated": {
                str(mid): n for mid, (_pose, n) in sorted(aggregated.items())
            },
        },
        "markers": {
            str(marker_id): {
                "translation": [pose[0], pose[1], pose[2]],
                "rotation": [pose[3], pose[4], pose[5], pose[6]],
            }
            for marker_id, (pose, _n) in sorted(aggregated.items())
        },
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2))
