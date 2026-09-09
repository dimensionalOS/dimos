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

    points: list[Point3] = Field(min_length=2, max_length=2_000)
    label: str = Field(default="", max_length=120)
    color: Color = "#ffd166"


class HighlightRegion(BaseModel):
    """A world-frame floor polygon rendered as a translucent region."""

    points: list[Point3] = Field(min_length=3, max_length=500)
    label: str = Field(default="", max_length=120)
    color: Color = "#f9e547"
    opacity: float = Field(default=0.35, ge=0.0, le=1.0)


class HighlightPoint(BaseModel):
    """A world-frame point of interest."""

    position: Point3
    label: str = Field(default="", max_length=120)
    color: Color = "#ff4d6d"
    # Metres around the point whose voxels the viewer repaints. Only set when
    # the point is an object, not a capture pose.
    radius: float | None = Field(default=None, gt=0.0, le=5.0)


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


RESULT_SENTINEL = "__DIMOS_MEMORY_RESULT__="


MEMORY_ANALYSIS_BOOTSTRAP = f"""
import json
import math
import sys

import numpy as np

from dimos.memory.store.sqlite import SqliteStore

store = SqliteStore(path=sys.argv[1], must_exist=True)
store.start()
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

    print("{RESULT_SENTINEL}" + json.dumps(result, default=json_default, separators=(",", ":")))
finally:
    store.stop()
"""
