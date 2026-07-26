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

from typing import Literal

from pydantic import Field

from dimos.mapping.relocalization.relocalize import generate_ransac_candidates
from dimos.protocol.service.spec import BaseConfig


class PriorConfigBase(BaseConfig):
    """Fields every prior shares: the on/off toggle plus its accept bar."""

    enabled: bool = True
    # Per-prior accept gate: min wall fitness (dimensionless, 0-1) this prior's fix must clear. 0.6 because the trial's office survey produced sub-0.6 fixes that were meters off while still scoring as "fit".
    fitness_threshold: float = Field(default=0.6, ge=0.0, le=1.0)


class RansacPriorConfig(PriorConfigBase):
    """Multi-scale FPFH+RANSAC global search (``RansacPrior``); search knobs live in relocalize.py, this entry owns the accept bar, cadence and geometry floor."""

    type: Literal["ransac"] = "ransac"
    # s between RANSAC fires; one FPFH+RANSAC search costs 4.4-23 s of CPU on the trial's go2/Orin recordings, so the sweep is paced, not per-frame.
    interval_s: float = Field(default=2.0, gt=0.0)
    # Min local-map points (post VoxelGridMapper) before this search fires; below this FPFH matching + the wall-only rerank have too little geometry, so the frame is skipped.
    min_local_points: int = Field(default=50_000, ge=0)


class RansacPrior:
    """relocalize.py's FPFH+RANSAC global search as a prior; the module owns its poll timer."""

    name = "ransac"
    propose = staticmethod(generate_ransac_candidates)
