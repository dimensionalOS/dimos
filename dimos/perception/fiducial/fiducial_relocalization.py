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

"""Load a surveyed marker map (``marker_id -> map_T_marker``) for the fiducial relocalization path."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3

MAP_FRAME = "map"


def _validated_entry(marker_id: int, entry: dict[str, Any]) -> Transform:
    """Validate one survey entry into a ``map_T_marker`` Transform, failing loudly: a short
    translation zero-fills and a zero-norm quaternion only crashes (or corrupts a pose) later."""
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
    """``marker_id -> map_T_marker`` from a survey JSON or YAML."""
    path = Path(path)
    text = path.read_text()
    data = (json.loads(text) if path.suffix == ".json" else yaml.safe_load(text)) or {}
    return {
        int(marker_id): _validated_entry(int(marker_id), entry)
        for marker_id, entry in (data.get("markers", {}) or {}).items()
    }
