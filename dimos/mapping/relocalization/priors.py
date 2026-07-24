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

"""Pluggable relocalization priors: candidate proposers feeding the shared fine-ICP judge in relocalize.py (``refine_candidates``)."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import threading
from typing import Annotated, Any, Literal, Protocol

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
from pydantic import Field
import yaml

from dimos.mapping.relocalization.relocalize import (
    GRAVITY_TILT_MAX_DEG,
    generate_ransac_candidates,
    refine_candidates,
)
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.fiducial.apriltag_aggregation import AggregationConfig, Pose7
from dimos.protocol.service.spec import BaseConfig

# s between RANSAC fires; one FPFH+RANSAC search costs seconds of CPU (4.4-23 s on the trial's go2/Orin recordings), so the sweep is paced, not per-frame.
DEFAULT_RANSAC_INTERVAL_S = 2.0


# One pydantic config per prior, keyed by a Literal ``type`` into a discriminated union. Pattern from dimos/manipulation/planning/kinematics/config.py:26-57.


class PriorConfigBase(BaseConfig):
    """Fields every prior shares: the on/off toggle plus its accept bar."""

    enabled: bool = True
    # Per-prior accept gate: min wall fitness (dimensionless, 0-1) this prior's fix must clear.
    fitness_threshold: float = Field(default=0.6, ge=0.0, le=1.0)


class RansacPriorConfig(PriorConfigBase):
    """Multi-scale FPFH+RANSAC global search (``RansacPrior``); search knobs live in relocalize.py, this entry owns the accept bar, cadence and geometry floor."""

    type: Literal["ransac"] = "ransac"
    # s between RANSAC fires; a global search waits on no event, so it paces itself.
    interval_s: float = Field(default=DEFAULT_RANSAC_INTERVAL_S, gt=0.0)
    # Min local-map points (post VoxelGridMapper) before this search fires; below this FPFH matching + the wall-only rerank have too little geometry, so the frame is skipped.
    min_local_points: int = Field(default=50_000, ge=0)


class FiducialPriorConfig(PriorConfigBase):
    """Marker sightings Huber-aggregated into one world->map candidate per tag (``FiducialPrior``). Owns the whole fiducial parameter surface."""

    type: Literal["fiducial"] = "fiducial"
    # Surveyed marker map (map_T_marker per id), a .json path resolved via resolve_named_path; required -- start() no-ops the prior without it.
    marker_map_file: str | None = None
    # Tag geometry/family/aggregation knobs the DETECTOR needs; none is read here.
    marker_length_m: float = Field(default=0.10, gt=0.0)  # physical tag edge, m
    aruco_dictionary: str = "DICT_APRILTAG_36h11"
    aggregation: AggregationConfig = Field(default_factory=AggregationConfig)


# Discriminated on ``type`` (kinematics/config.py:54 is the exemplar).
PriorConfig = Annotated[
    RansacPriorConfig | FiducialPriorConfig,
    Field(discriminator="type"),
]


@dataclass
class Candidate:
    """One proposed transform, pre-judging. ``T`` is the 4x4 mapping ``local_map`` into ``global_map``'s frame; no self-reported confidence (the judge ranks it)."""

    T: np.ndarray
    source: str


class RelocPrior(Protocol):
    """A relocalization candidate proposer; the module owns the trigger. A prior must not self-select a winner (``refine_candidates``'s job); zero candidates is a valid response."""

    name: str

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]: ...


class RansacPrior:
    """Wraps relocalize.py's FPFH+RANSAC global search; a pure candidate source the module polls on a paced interval (the pace bounds an eventless search's cost)."""

    name = "ransac"

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        transforms = generate_ransac_candidates(global_map, local_map)
        return [Candidate(T=T, source=self.name) for T in transforms]


MAP_FRAME = "map"


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
    """``marker_id -> map_T_marker`` from a survey JSON or YAML."""
    path = Path(path)
    text = path.read_text()
    data = (json.loads(text) if path.suffix == ".json" else yaml.safe_load(text)) or {}
    return {
        int(marker_id): _validated_entry(int(marker_id), entry)
        for marker_id, entry in (data.get("markers", {}) or {}).items()
    }


def write_marker_map(path: Path, aggregated: dict[int, tuple[Pose7, int]], *, source: str) -> None:
    """Serialize aggregated ``map_T_tag`` poses to the JSON ``load_marker_map`` reads."""
    doc = {
        "meta": {
            "schema": "map_T_tag",
            "source_recording": source,
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


class FiducialPrior:
    """Aggregated fiducial tag poses -> ONE map_T_world candidate per tag, ``map_T_marker @ inv(world_T_marker_aggregated)`` (local_map->global_map, as RANSAC)."""

    name = "fiducial"

    def __init__(self, marker_map: dict[int, np.ndarray]) -> None:
        # marker_id -> map_T_marker (4x4); the surveyed marker map.
        self._map_T_marker = marker_map
        # marker_id -> map_T_world (4x4) awaiting its ONE trip past the judge.
        self._pending: dict[int, np.ndarray] = {}
        # observe() and propose() run on different transport threads; the lock makes the read-modify-write of _pending one critical section -- see propose() for the tear.
        self._pending_lock = threading.Lock()

    def observe(self, marker_id: int, world_T_marker_aggregated: np.ndarray) -> str | None:
        """Compose this tag's map_T_world fix from one aggregated pose; returns ``unmapped_id`` or ``None``."""
        map_T_marker = self._map_T_marker.get(marker_id)
        if map_T_marker is None:
            return "unmapped_id"
        with self._pending_lock:
            self._pending[marker_id] = map_T_marker @ np.linalg.inv(world_T_marker_aggregated)
        return None

    @property
    def has_pending(self) -> bool:
        """A composed fix is waiting for the judge -- the module's fire signal."""
        return bool(self._pending)

    def propose(
        self,
        global_map: o3d.geometry.PointCloud,
        local_map: o3d.geometry.PointCloud,
    ) -> list[Candidate]:
        # Consume on use (re-offering a drained fix scores worse, world has drifted); swap under the lock or observe()'s read-modify-write tears here into a "dict changed size during iteration" dropped cycle.
        with self._pending_lock:
            pending, self._pending = self._pending, {}
        return [Candidate(T=fix_T, source=self.name) for fix_T in pending.values()]


class EmptyProposalError(ValueError):
    """No prior proposed any candidate this fire; a benign race artifact (the loser of two threads drains an empty _pending), treated as a no-op, not a crash."""


def relocalize_with_priors(
    global_map: o3d.geometry.PointCloud,
    local_map: o3d.geometry.PointCloud,
    priors: list[RelocPrior],
    gravity_tilt_max_deg: float = GRAVITY_TILT_MAX_DEG,
) -> tuple[np.ndarray, float, str]:
    """Gather candidates from the priors this fire runs, judge them through the shared fine-ICP tail, report the winning source; returns ``(T, fitness, winning_source)``, raises ``EmptyProposalError`` if every prior proposed zero candidates (a benign race)."""
    all_transforms: list[np.ndarray] = []
    sources: list[str] = []
    for prior in priors:
        for candidate in prior.propose(global_map, local_map):
            all_transforms.append(candidate.T)
            sources.append(candidate.source)

    if not all_transforms:
        raise EmptyProposalError("relocalize_with_priors: no prior proposed any candidate")

    T, fitness, winning_index = refine_candidates(
        global_map,
        local_map,
        all_transforms,
        gravity_tilt_max_deg=gravity_tilt_max_deg,
    )
    return T, fitness, sources[winning_index]
