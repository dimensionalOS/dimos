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

"""Surveyed marker-map loading for the fiducial relocalization path.

The ``map_T_marker`` poses (one per tag id) that ``FiducialPrior`` composes
with each fused ``world_T_marker`` sighting into a ``world->map`` candidate.
The per-glimpse pose solve + IPPE mirror-ambiguity gate live in
:func:`dimos.perception.fiducial.marker_pose.ambiguity_gated_pose`; the
robust multi-sighting fusion lives in
:mod:`dimos.perception.fiducial.apriltag_aggregation`.
"""

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
    if not np.isfinite(norm) or norm < 1e-6:  # 1e-6: unnormalizable, floor below a unit quaternion's round-off
        raise ValueError(f"marker {marker_id}: rotation quaternion norm is {norm}, not usable")
    return Transform(
        translation=Vector3(*translation),
        rotation=Quaternion(*rotation),
        frame_id=MAP_FRAME,
        child_frame_id=f"marker_{marker_id}",
    )


def load_marker_map(path: str | Path) -> dict[int, Transform]:
    """``marker_id -> map_T_marker`` from a survey JSON or YAML.

    Schema (``meta`` is provenance, ignored here)::

        {"meta": {...},
         "markers": {"<tag_id>": {"translation": [x, y, z],
                                  "rotation": [qx, qy, qz, qw]}}}  # map_T_tag

    A ``.yaml``/``.yml`` survey parses too (the eval overlay's loader
    eval_module.load_markers_xyz accepts both, and the marker-map derivation
    pipeline writes yaml); any other suffix goes through yaml.safe_load, a JSON
    superset, so a bare-name or extensionless survey still loads.
    """
    path = Path(path)
    text = path.read_text()
    data = (json.loads(text) if path.suffix == ".json" else yaml.safe_load(text)) or {}
    return {
        int(marker_id): _validated_entry(int(marker_id), entry)
        for marker_id, entry in (data.get("markers", {}) or {}).items()
    }
