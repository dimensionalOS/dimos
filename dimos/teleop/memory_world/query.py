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

from collections.abc import Sequence
from pathlib import Path
from typing import Annotated, Literal

from PIL import ImageColor
from pydantic import BaseModel, BeforeValidator, Field, ValidationError, field_validator

FiniteFloat = Annotated[float, Field(allow_inf_nan=False)]
Point3 = tuple[FiniteFloat, FiniteFloat, FiniteFloat]


def _to_hex(value: object) -> object:
    """A CSS color name, a hex string, or an RGB triple in 0-1 or 0-255, as #rrggbb."""
    if isinstance(value, str):
        try:
            r, g, b = ImageColor.getrgb(value.strip())[:3]
        except ValueError:
            return value
        return f"#{r:02x}{g:02x}{b:02x}"
    if isinstance(value, Sequence) and len(value) == 3:
        channels = [float(v) for v in value]
        scale = 255.0 if max(channels) > 1.0 else 1.0
        r, g, b = (round(min(max(v, 0.0), scale) / scale * 255) for v in channels)
        return f"#{r:02x}{g:02x}{b:02x}"
    return value


Color = Annotated[str, BeforeValidator(_to_hex), Field(pattern=r"^#[0-9a-fA-F]{6}$")]


def validation_summary(error: ValidationError) -> str:
    """Every distinct complaint once, with the fields it applies to, instead of one line per item."""
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


class HighlightBox(BaseModel):
    """An axis-aligned world-frame box rendered as a wireframe with a faint fill."""

    center: Point3
    # Full sizes in metres along the box's own axes.
    extent: Point3
    # Heading of the box's x axis, world radians about z.
    yaw: float = 0.0
    label: str = Field(default="", max_length=120)
    color: Color = "#22dd88"
    opacity: float = Field(default=0.12, ge=0.0, le=1.0)

    @field_validator("extent")
    @classmethod
    def _positive(cls, extent: Point3) -> Point3:
        if min(extent) <= 0.0:
            raise ValueError("extent must be positive on every axis")
        return extent


class HighlightPoint(BaseModel):
    """A world-frame point of interest."""

    position: Point3
    label: str = Field(default="", max_length=120)
    color: Color = "#ff4d6d"
    # Metres around the point whose voxels the viewer repaints. Only set when
    # the point is an object, not a capture pose.
    radius: float | None = Field(default=None, gt=0.0, le=5.0)
    # Full x, y, z size in metres of the object's box around the point; the viewer
    # repaints the voxels inside it instead of the ball.
    extent: Point3 | None = None
    yaw: float = 0.0


ANSWER_MAX_CHARS = 8_000


def _trim_answer(value: object) -> object:
    """Cut an overlong answer rather than reject the whole result for it."""
    if isinstance(value, str) and len(value) > ANSWER_MAX_CHARS:
        return value[: ANSWER_MAX_CHARS - 20].rstrip() + " [answer cut here]"
    return value


class MemoryQueryResult(BaseModel):
    """Textual answer and spatial evidence for one memory query."""

    answer: Annotated[str, BeforeValidator(_trim_answer)] = Field(
        min_length=1, max_length=ANSWER_MAX_CHARS
    )
    focus_point: Point3 | None = None
    regions: list[HighlightRegion] = Field(default_factory=list, max_length=128)
    boxes: list[HighlightBox] = Field(default_factory=list, max_length=64)
    evidence_paths: list[HighlightPath] = Field(default_factory=list, max_length=32)
    points: list[HighlightPoint] = Field(default_factory=list, max_length=128)
    observation_ids: list[int] = Field(default_factory=list, max_length=200)
    route: HighlightPath | None = None
    action: Literal["replace"] = "replace"


RESULT_SENTINEL = "__DIMOS_MEMORY_RESULT__="
STEP_SENTINEL = "__DIMOS_MEMORY_STEP__="
# Loaded by path in the sandbox: importing the package would pull in the whole module.
STEPWISE_PATH = Path(__file__).with_name("stepwise.py")
MEMORY_ANALYSIS_BOOTSTRAP = f"""
import json
import math
import sys

import numpy as np

from dimos.memory.store.sqlite import SqliteStore

store = SqliteStore(path=sys.argv[1], must_exist=True)
store.start()
viewer_position = json.loads(sys.argv[2])
route = json.loads(sys.argv[3])
objects = json.loads(sys.argv[4])

def sample_pose_path(stream_name="pointlio_lidar", max_points=200):
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
    "route": route,
    "objects": objects,
    "sample_pose_path": sample_pose_path,
}}
import importlib.util

_spec = importlib.util.spec_from_file_location("stepwise", r"{STEPWISE_PATH}")
stepwise = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(stepwise)
run_stepwise = stepwise.run_stepwise


def report_step(payload):
    sys.stderr.write("{STEP_SENTINEL}" + json.dumps(payload) + "\\n")
    sys.stderr.flush()


try:
    run_stepwise(sys.stdin.read(), namespace, report_step)
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
