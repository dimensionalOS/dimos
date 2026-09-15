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

"""Validated results produced by recorded-memory analysis."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BaseModel, Field

FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
Point3 = tuple[FiniteFloat, FiniteFloat, FiniteFloat]
Color = Annotated[str, Field(pattern=r"^#[0-9a-fA-F]{6}$")]


class HighlightPath(BaseModel):
    """A world-frame path rendered as a tube in VR."""

    points: list[Point3] = Field(min_length=2, max_length=20_000)  # a planned route: one per cell
    label: str = Field(default="", max_length=120)
    color: Color = "#ffd166"


class HighlightRegion(BaseModel):
    """A world-frame floor polygon rendered as a translucent region."""

    points: list[Point3] = Field(min_length=3, max_length=500)
    label: str = Field(default="", max_length=120)
    color: Color = "#f9e547"
    opacity: float = Field(default=0.35, ge=0.0, le=1.0)


# The most a single answer may repaint. The module's `object_radius_m` is what fills
# `HighlightPoint.radius`, so the two bounds have to be the same number: they were not,
# and an `object_radius_m` the config accepted (anything over 5) made every located
# answer raise a ValidationError here, which the user was shown as "The SigLIP index
# cannot answer" -- a query failure, reported at query time, for a setting.
MAX_HIGHLIGHT_RADIUS_M = 5.0

# The most places one answer may name. The module's `max_places` is what fills
# `MemoryQueryResult.clusters`, so the two bounds have to be the same number -- the same
# pairing, and the same bug, as `MAX_HIGHLIGHT_RADIUS_M` above: a `max_places` the config
# accepted made every answer raise here, and the user was shown "The SigLIP index cannot
# answer" for what was a setting.
MAX_ANSWER_PLACES = 64


class HighlightPoint(BaseModel):
    """A world-frame point of interest."""

    position: Point3
    label: str = Field(default="", max_length=120)
    color: Color = "#ff4d6d"
    # Metres around the point whose voxels the viewer repaints. Only set when
    # the point is an object, not a capture pose.
    radius: float | None = Field(default=None, gt=0.0, le=MAX_HIGHLIGHT_RADIUS_M)


class ClusterSummary(BaseModel):
    """One place an answer names: where it is, how big, how sure, how many pictures back it."""

    index: int = Field(ge=0)
    centre: Point3
    radius: Annotated[FiniteFloat, Field(gt=0.0)]
    # The engine scores a place by cosine similarity, which is in [-1, 1] and is usually
    # a small positive number -- so these are NOT bounded to [0, 1], and clamping would
    # print the same 0.00 for -0.8 and for 0.
    #
    # FINITE, like every other number on the wire. These three were the only floats in
    # this package that were not: `json.dumps` writes `NaN`, which is not JSON, so
    # `JSON.parse` in `protocol.js` throws, `decodeText` returns null and `main.js` drops
    # the WHOLE `query_result` on `if (!msg) return;`. The viewer shows nothing at all
    # and the server reports a success -- an answer that silently never arrives.
    score: FiniteFloat
    peak: FiniteFloat
    n_views: int = Field(default=0, ge=0)
    n_evidence: int = Field(default=0, ge=0)
    label: str = Field(default="", max_length=120)


class MemoryQueryResult(BaseModel):
    """Textual answer and spatial evidence for one memory query."""

    answer: str = Field(min_length=1, max_length=2_000)
    focus_point: Point3 | None = None
    regions: list[HighlightRegion] = Field(default_factory=list, max_length=32)
    evidence_paths: list[HighlightPath] = Field(default_factory=list, max_length=32)
    points: list[HighlightPoint] = Field(default_factory=list, max_length=128)
    observation_ids: list[int] = Field(default_factory=list, max_length=200)
    route: HighlightPath | None = None
    action: Literal["replace"] = "replace"
    # The places the viewer steps through with next/prev, best first.
    clusters: list[ClusterSummary] = Field(default_factory=list, max_length=MAX_ANSWER_PLACES)
    engine: Literal["siglip"] = "siglip"
    query_text: str = Field(default="", max_length=400)
