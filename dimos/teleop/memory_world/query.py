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

from pathlib import Path
from typing import Annotated, Literal

from pydantic import BaseModel, Field, ValidationError

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

# A side of a measured box, bounded by the same metre budget as a highlight radius.
BoxSide = Annotated[FiniteFloat, Field(gt=0.0, le=2 * MAX_HIGHLIGHT_RADIUS_M)]


class HighlightPoint(BaseModel):
    """A world-frame point of interest."""

    position: Point3
    label: str = Field(default="", max_length=120)
    color: Color = "#ff4d6d"
    # Metres around the point whose voxels the viewer repaints. Only set when
    # the point is an object, not a capture pose.
    radius: float | None = Field(default=None, gt=0.0, le=MAX_HIGHLIGHT_RADIUS_M)


class HighlightBox(BaseModel):
    """A world-frame, axis-aligned box around a thing whose size was MEASURED.

    Only drawn for a measurement. Hyperspace's item path backprojects the detector's
    2-D box through depth and merges the looks that agree, so `extent` is the thing's
    own size; a heatmap or area answer measured no size and gets a point and its
    configured radius instead. A box invented from a radius would look exactly like
    this one and mean nothing.
    """

    centre: Point3
    # FULL side lengths, not half-extents, in the same order as `centre`. Bounded like
    # `HighlightPoint.radius` and for the same reason -- a value the config accepts but
    # this rejects surfaces to the user as "the query failed" -- so a producer CLAMPS
    # rather than passes a bigger one through.
    extent: tuple[BoxSide, BoxSide, BoxSide]
    label: str = Field(default="", max_length=120)
    color: Color = "#22dd88"
    opacity: float = Field(default=0.12, ge=0.0, le=1.0)


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
    # The measured size of each thing an answer found, where anything measured one.
    boxes: list[HighlightBox] = Field(default_factory=list, max_length=MAX_ANSWER_PLACES)
    observation_ids: list[int] = Field(default_factory=list, max_length=200)
    route: HighlightPath | None = None
    action: Literal["replace"] = "replace"
    # The places the viewer steps through with next/prev, best first.
    clusters: list[ClusterSummary] = Field(default_factory=list, max_length=MAX_ANSWER_PLACES)
    # Which engine answered, and it is on the wire because the two mean different
    # things: "siglip" places are where a thing was SEEN FROM, "hyperspace" places
    # are the thing's own measured position. A viewer that described both the same
    # way would be wrong about one of them.
    engine: Literal["siglip", "hyperspace"] = "siglip"
    # What produced the numbers in `clusters`, because `engine` alone no longer says.
    # "hyperspace" now covers an item answer, whose score is OWLv2's calibrated box
    # confidence, AND a heatmap or area answer, whose score is a CELL'S -- two scales
    # under one engine name. So `(engine, kind)` is what says whether two scores may be
    # compared, and either alone is a guess.
    #
    # It is PROVENANCE, not units, and it sits on the ANSWER rather than on each cluster:
    # every cluster in one answer came from one engine and one kind, so a per-cluster tag
    # would be the same fact repeated N times, and repeated facts drift. Two quantities
    # can share a range and still not be comparable, which is exactly the [0, 1] case
    # here -- `ClusterSummary.score` has already meant two incomparable things once, and
    # the bounds written for the first rejected every answer from the second.
    kind: Literal["embedding", "item", "heatmap", "area"] = "embedding"
    query_text: str = Field(default="", max_length=400)


def validation_summary(error: ValidationError) -> str:
    """Every distinct complaint once, with the fields it applies to.

    From `andrew/feat/vr_demo`. Pydantic reports one line per offending item, so a
    result with sixty bad points hands the agent sixty identical sentences and no room
    for the one that differs; this says "points.*.position (60 of them): ..." instead.
    """
    groups: dict[tuple[str, str], list[str]] = {}
    for item in error.errors():
        loc = [str(part) for part in item["loc"]]
        shape = ".".join("*" if part.isdigit() else part for part in loc)
        groups.setdefault((shape, item["msg"]), []).append(".".join(loc))
    lines = []
    for (shape, msg), fields in groups.items():
        where = shape if len(fields) == 1 else f"{shape} ({len(fields)} of them)"
        lines.append(f"{where}: {msg}")
    return "; ".join(lines)


# ---- the analysis sandbox -----------------------------------------------------------
# `analyze_memory` runs the agent's program in a SEPARATE PROCESS, which is what makes a
# runaway loop or a segfault in it survivable: the module kills the child and answers the
# question with a failure instead of dying with it. The two ends talk over the child's
# own stdio, which is why both sentinels exist -- a program is free to print whatever it
# likes, so the result and the step reports have to be findable in the noise. The result
# is the LAST line matching its sentinel on stdout; the steps go to stderr so that
# ordinary prints cannot be mistaken for one.
RESULT_SENTINEL = "__DIMOS_MEMORY_RESULT__="
STEP_SENTINEL = "__DIMOS_MEMORY_STEP__="
# Loaded by path in the sandbox: importing the package would pull in the whole module.
STEPWISE_PATH = Path(__file__).with_name("stepwise.py")
# `open_recording` rather than `SqliteStore` (which is what andrew's branch opens): this
# build is pointed at a .mcap as often as a .db, and the mcap path needs the derived
# store beside it. The package's `__init__` is lazy precisely so a child like this one
# can import a sibling without paying for torch and FastAPI.
MEMORY_ANALYSIS_BOOTSTRAP = f"""
import json
import sys

import numpy as np

from dimos.teleop.memory_world.recording import open_recording

store = open_recording(sys.argv[1])
viewer_position = json.loads(sys.argv[2])
places = json.loads(sys.argv[3])
trail = json.loads(sys.argv[4])
world_frame = sys.argv[5]

def sample_pose_path(max_points=200):
    # The robot's own trajectory, world-frame xyz, thinned to at most `max_points`.
    # It is passed in already built rather than read here: on this build the poses come
    # from the tf tree, not from a pose stamped on an observation, and tf is the module's
    # to read. A recording whose tf could not place the camera gives an empty path.
    if not isinstance(max_points, int) or not 2 <= max_points <= 2000:
        raise ValueError("max_points must be an integer from 2 through 2000")
    if len(trail) <= max_points:
        return [list(point) for point in trail]
    stride = (len(trail) - 1) / (max_points - 1)
    return [list(trail[min(int(index * stride), len(trail) - 1)]) for index in range(max_points)]

namespace = {{
    "__name__": "__main__",
    "np": np,
    "store": store,
    "viewer_position": viewer_position,
    "places": places,
    "world_frame": world_frame,
    "sample_pose_path": sample_pose_path,
}}
import importlib.util

_spec = importlib.util.spec_from_file_location("stepwise", r"{STEPWISE_PATH}")
stepwise = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(stepwise)


def report_step(payload):
    sys.stderr.write("{STEP_SENTINEL}" + json.dumps(payload) + "\\n")
    sys.stderr.flush()


stepwise.run_stepwise(sys.stdin.read(), namespace, report_step)
result = namespace.get("result")
if not isinstance(result, dict):
    raise TypeError("analysis must assign a dictionary to `result`")


def json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"{{type(value).__name__}} is not JSON serializable")


print("{RESULT_SENTINEL}" + json.dumps(result, default=json_default, separators=(",", ":")))
"""
