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
    """One blob of a heat map: where it is, how big, how sure, how many pictures back it."""

    index: int = Field(ge=0)
    centre: Point3
    radius: float = Field(gt=0.0)
    # Hyperspace scores a voxel in [0, 1]; the embedding engine scores a place by cosine
    # similarity, which is in [-1, 1] and is usually a small positive number. Bounds that
    # only ever described the heat map rejected every embedding answer outright, and
    # clamping instead would have printed the same 0.00 for -0.8 and for 0.
    score: float
    peak: float
    # The embedding engine answers from frames, not from a voxel grid, so it has none.
    n_voxels: int = Field(default=0, ge=0)
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
    # Heat-map answers (Hyperspace): the clusters the viewer steps through with
    # next/prev, best first. The voxels themselves travel as MSG_HEATMAP.
    clusters: list[ClusterSummary] = Field(default_factory=list, max_length=MAX_ANSWER_PLACES)
    engine: Literal["hyperspace", "siglip", "agent"] = "agent"
    query_text: str = Field(default="", max_length=400)


def answer_positions(result: MemoryQueryResult) -> list[tuple[float, float, float]]:
    """Everywhere an answer points, focus point included: an answer can carry one and
    no points, and dropping it there loses every photograph it could have had."""
    near = [tuple(point.position) for point in result.points]
    if result.focus_point is not None:
        near.append(tuple(result.focus_point))
    return near  # type: ignore[return-value]


RESULT_SENTINEL = "__DIMOS_MEMORY_RESULT__="


MEMORY_ANALYSIS_BOOTSTRAP = f"""
import json
import math
import sys

import numpy as np

from dimos.teleop.memory_world.recording import open_recording

store = open_recording(sys.argv[1])
store.start()

_NO_WRITES = (
    "analyze_memory reads the recording and cannot write to it."
)

def _read_only(value):
    # A stream hands back MORE STREAMS: `limit`, `after`, `near`, `order_by` and a dozen
    # others each return another view of the same table, with the same `append` on it.
    # Forwarding them handed the analysis a writable object through a read-only wrapper --
    # measured, `store.streams["x"].limit(1).append(9.0, ts=2.0)` put a row in the
    # recording. So a stream that comes back out of one is wrapped again, and so is
    # anything a method of one returns.
    if isinstance(value, _ReadOnlyStream):
        return value
    if hasattr(value, "append") and hasattr(value, "data_type"):
        return _ReadOnlyStream(value)
    if isinstance(value, type):
        # A CLASS is callable, and `stream.data_type` is one: wrapping it in a function
        # left `np.array([...], dtype=stream.data_type)` raising "Cannot interpret
        # <function _read_only.<locals>.wrapped> as a data type" -- a read the analysis
        # skill's own examples make. A type is not a way into the store.
        return value
    if callable(value):
        def wrapped(*args, **kwargs):
            return _read_only(value(*args, **kwargs))

        return wrapped
    return value

class _ReadOnlyStream:
    # Analysis READS the recording. `open_recording` hands back a read-write store --
    # there is no read-only mode -- so a snippet that appended a stream left it in the
    # operator's recording for good, and `analyze_memory` reported success: measured, an
    # injected stream survived and the .db grew by 24 KB. The store's read surface is
    # spelt out rather than its writes blacklisted, because a blacklist misses the one
    # that matters -- the first attempt at this wrapped only objects that already had an
    # `append`, so `store.stream(name, type)` handed back the real thing and wrote.
    #
    # A stream's surface is far too wide to spell out that way, so here the writes are
    # named and everything else comes back through `_read_only`, which wraps whatever a
    # forwarded method returns rather than trusting it.
    #
    # This stops the mistake, which is the whole of the risk: the code is whoever asked
    # the question's own, and one determined to write could import sqlite3 itself.
    def __init__(self, inner):
        self._inner = inner

    def __getattr__(self, name):
        if name in ("append", "extend", "truncate", "delete"):
            raise PermissionError(_NO_WRITES)
        return _read_only(getattr(self._inner, name))

    def __iter__(self):
        return iter(self._inner)

    def __len__(self):
        return len(self._inner)

    def __getitem__(self, key):
        return _read_only(self._inner[key])

    # Special methods are looked up on the TYPE, so `__getattr__` never sees them: without
    # these, `with store.streams['x'].limit(1) as s:` raised "'_ReadOnlyStream' object does
    # not support the context manager protocol" on a stream that supports it perfectly
    # well. `__enter__` hands back the WRAPPER, not the inner stream, or the block body
    # would be holding the writable one.
    def __enter__(self):
        self._inner.__enter__()
        return self

    def __exit__(self, *exception):
        return self._inner.__exit__(*exception)

class _ReadOnlyStreams:
    def __init__(self, inner):
        self._inner = inner

    def __getitem__(self, key):
        return _ReadOnlyStream(self._inner[key])

    def __getattr__(self, name):
        return _ReadOnlyStream(getattr(self._inner, name))

    def __iter__(self):
        return iter(self._inner)

    def __contains__(self, key):
        return key in self._inner

class _ReadOnlyStore:
    def __init__(self, inner):
        self._inner = inner
        self.streams = _ReadOnlyStreams(inner.streams)

    def list_streams(self, *args, **kwargs):
        return self._inner.list_streams(*args, **kwargs)

    def summary(self, *args, **kwargs):
        return self._inner.summary(*args, **kwargs)

    def start(self, *args, **kwargs):
        return self._inner.start(*args, **kwargs)

    def stop(self, *args, **kwargs):
        # The bootstrap's own finally calls this. Everything else a read needs is named
        # above; anything not named is absent rather than forwarded, which is the point.
        return self._inner.stop(*args, **kwargs)

    def stream(self, *args, **kwargs):
        # The create-or-open entry point, and the one an accidental write goes through.
        raise PermissionError(_NO_WRITES)

    def delete_stream(self, *args, **kwargs):
        raise PermissionError(_NO_WRITES)

store = _ReadOnlyStore(store)
viewer_position = json.loads(sys.argv[2])

def sample_pose_path(stream_name="odom", max_points=200):
    # Return a bounded world-frame xyz path from a pose-bearing stream.
    if not isinstance(max_points, int) or not 2 <= max_points <= 2000:
        raise ValueError("max_points must be an integer from 2 through 2000")
    stream = store.streams[stream_name]
    stride = max(1, math.ceil(stream.count() / max_points))
    points = []
    for index, observation in enumerate(stream):
        pose = observation.pose_tuple
        if pose is not None and index % stride == 0:
            points.append([float(pose[0]), float(pose[1]), float(pose[2])])
    return points[:max_points]

namespace = {{
    "__name__": "__main__",
    "np": np,
    "store": store,
    "viewer_position": viewer_position,
    "sample_pose_path": sample_pose_path,
}}
try:
    exec(compile(sys.stdin.read(), "<analyze_memory>", "exec"), namespace)
    result = namespace.get("result")
    if not isinstance(result, dict):
        raise TypeError("analysis must assign a dictionary to `result`")

    def json_default(value):
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        raise TypeError(f"{{type(value).__name__}} is not JSON serializable")

    # The bare print() is load-bearing: the parent finds this marker by looking for a line
    # that STARTS with it, and analysis code is free to `print(..., end="")` right before
    # returning. Without a line break first, that output and this marker share a line, the
    # parent finds nothing, and a perfectly good answer comes back as EXECUTION_FAILED.
    # (A "\\n" prefix would be the obvious way to write this and is wrong here: the
    # bootstrap is an f-string, so the escape resolves when the TEMPLATE is built and puts
    # a real newline inside the child's string literal, which will not compile.)
    print()
    print("{RESULT_SENTINEL}" + json.dumps(result, default=json_default, separators=(",", ":")))
finally:
    store.stop()
"""
