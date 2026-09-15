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

"""Deterministic scale fixtures; analytic labels never consult agent_encode.

These are development cases, not a held-out benchmark. Surfaces are sampled
at the same relative density across scales. They test spatial extent and
precision, not acquisition noise or processing millions of points.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
from tempfile import TemporaryDirectory
from typing import Literal, TypeAlias

import numpy as np
from numpy.typing import NDArray

from dimos.constants import CACHE_DIR
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

Point: TypeAlias = tuple[float, float, float]
Box: TypeAlias = tuple[Point, Point]
Quantity: TypeAlias = Literal["span", "gap", "maximum"]

DATA_DIR = CACHE_DIR / "evals" / "pointcloud-scale-v1"
SAMPLES_PER_EDGE = 13
TIMESTAMP = 1000.0
AXES = "xyz"

# The two layouts have different answers, including different relative gaps.
WIDE: tuple[Box, ...] = (((-5, -2, 0), (-1, 2, 3)), ((1, -1, 0), (7, 3, 5)))
NARROW: tuple[Box, ...] = (((-4, -3, -1), (-1, 1, 2)), ((-0.25, -2, -1), (5, 2, 3)))
DISTANT: tuple[Box, ...] = (
    ((-500, -150, 0), (-450, -100, 60)),
    ((450, 100, 0), (500, 150, 40)),
)
LOCAL_REGION: Box = ((-8, -4, -2), (8, 4, 6))


def transform_boxes(
    boxes: tuple[Box, ...],
    *,
    scale: float = 1.0,
    origin: Point = (0, 0, 0),
    axes: tuple[int, int, int] = (0, 1, 2),
    signs: Point = (1, 1, 1),
) -> tuple[Box, ...]:
    """Transform fixture bounds by a uniform scale and signed axis permutation."""
    result: list[Box] = []
    for lo, hi in boxes:
        ends = [
            sorted((scale * signs[i] * lo[a] + origin[i], scale * signs[i] * hi[a] + origin[i]))
            for i, a in enumerate(axes)
        ]
        result.append(((ends[0][0], ends[1][0], ends[2][0]), (ends[0][1], ends[1][1], ends[2][1])))
    return tuple(result)


@dataclass(frozen=True)
class CloudSpec:
    id: str
    boxes: tuple[Box, ...]
    category: str
    frame_id: str = "map"
    gap_axis: int = 0
    height_axis: int = 2
    crop: Box | None = None

    @property
    def visible_boxes(self) -> tuple[Box, ...]:
        # Our prepared crops retain complete local boxes and exclude distant ones.
        # No boundary interpolation or geometric query implementation is needed.
        if self.crop is None:
            return self.boxes
        lo, hi = self.crop
        return tuple(
            b for b in self.boxes if all(lo[i] <= b[0][i] <= b[1][i] <= hi[i] for i in range(3))
        )

    def path(self, directory: Path = DATA_DIR) -> Path:
        # Content-address the recipe, including sampling and storage precision.
        recipe = {"cloud": asdict(self), "edge_samples": SAMPLES_PER_EDGE, "dtype": "float32"}
        digest = hashlib.sha256(json.dumps(recipe, sort_keys=True).encode()).hexdigest()[:20]
        return directory / f"{digest}.db"


def cloud_specs() -> tuple[CloudSpec, ...]:
    specs = [
        CloudSpec(f"{name}_{layout}", transform_boxes(boxes, scale=scale), name)
        for name, scale in (("tiny", 0.001), ("room", 1.0), ("block", 100.0))
        for layout, boxes in (("wide", WIDE), ("narrow", NARROW))
    ]
    specs.extend(
        (
            CloudSpec("elevated", transform_boxes(WIDE, origin=(11, -7, 40)), "translation"),
            CloudSpec(
                "large_offset",
                transform_boxes(NARROW, origin=(1_000_000, -2_000_000, 3_000_000)),
                "translation",
            ),
            # A proper rotation: input x -> output z, y -> -x, z -> -y.
            CloudSpec(
                "optical",
                transform_boxes(WIDE, axes=(1, 2, 0), signs=(-1, -1, 1), origin=(0, 0, 12)),
                "frame",
                frame_id="camera_optical",
                gap_axis=2,
                height_axis=1,
            ),
        )
    )
    for layout, boxes in (("wide", WIDE), ("narrow", NARROW)):
        specs.append(CloudSpec(f"overview_{layout}", boxes + DISTANT, "overview"))
        specs.append(CloudSpec(f"crop_{layout}", boxes + DISTANT, "crop", crop=LOCAL_REGION))
    return tuple(specs)


def surface_points(box: Box) -> NDArray[np.float64]:
    """Sample all six faces, including extrema; remove shared edge/corner points."""
    lo, hi = box
    coordinates = [np.linspace(lo[i], hi[i], SAMPLES_PER_EDGE) for i in range(3)]
    faces = []
    for axis in range(3):
        other = [i for i in range(3) if i != axis]
        u, v = np.meshgrid(coordinates[other[0]], coordinates[other[1]])
        for end in (lo[axis], hi[axis]):
            face = np.empty((u.size, 3))
            face[:, axis] = end
            face[:, other[0]], face[:, other[1]] = u.ravel(), v.ravel()
            faces.append(face)
    return np.unique(np.concatenate(faces), axis=0)


def points_for(spec: CloudSpec) -> NDArray[np.float32]:
    """The exact float32 points written to the eval recording, in canonical order."""
    points = np.concatenate([surface_points(box) for box in spec.boxes]).astype(np.float32)
    if spec.crop is not None:
        lo, hi = spec.crop
        points = points[np.all((points >= lo) & (points <= hi), axis=1)]
    return np.unique(points, axis=0)


@dataclass(frozen=True)
class GeometryQuestion:
    cloud: CloudSpec
    quantity: Quantity
    axis: int
    lower: float
    upper: float
    tolerance: float

    @property
    def id(self) -> str:
        return f"pointcloud_scale_{self.cloud.id}_{self.quantity}"

    @property
    def expected(self) -> float:
        return self.upper if self.quantity == "maximum" else self.upper - self.lower

    @property
    def inputs(self) -> str:
        axis = AXES[self.axis]
        if self.quantity == "span":
            question = (
                f"What is the full span of the stored points along {axis} in the source cloud's "
                "frame (maximum minus minimum)?"
            )
        elif self.quantity == "maximum":
            question = (
                f"What is the largest {axis} coordinate among the stored points in the source "
                "cloud's frame?"
            )
        else:
            region = ""
            if self.cloud.category in ("overview", "crop"):
                lo, hi = LOCAL_REGION
                region = (
                    "Consider only points inside this region, specified in meters in the source "
                    f"cloud's frame: x=[{lo[0]}, {hi[0]}], "
                    f"y=[{lo[1]}, {hi[1]}], z=[{lo[2]}, {hi[2]}]. "
                )
            question = (
                f"{region}What is the largest gap between consecutive distinct {axis} coordinates "
                "of the stored points in the source cloud's frame?"
            )
        return f"{question} Answer in meters with a single plain decimal number, without units or text."

    @property
    def evidence(self) -> str:
        axis = AXES[self.axis]
        if self.quantity == "maximum":
            return f"The largest declared {axis} endpoint is {self.upper:.12g} m."
        return f"{axis}: {self.upper:.12g} - ({self.lower:.12g}) = {self.expected:.12g} m."


def questions() -> tuple[GeometryQuestion, ...]:
    result = []
    for cloud in cloud_specs():
        a = cloud.gap_axis
        lo = min(b[0][a] for b in cloud.visible_boxes)
        hi = max(b[1][a] for b in cloud.visible_boxes)
        result.append(GeometryQuestion(cloud, "span", a, lo, hi, (hi - lo) * 0.01))
        # The first two boxes are always the local pair, also in the overview.
        pair = sorted(cloud.boxes[:2], key=lambda b: b[0][a])
        left, right = pair[0][1][a], pair[1][0][a]
        result.append(GeometryQuestion(cloud, "gap", a, left, right, (right - left) * 0.01))
        if cloud.category not in ("overview", "crop"):
            a = cloud.height_axis
            lo = min(b[0][a] for b in cloud.visible_boxes)
            hi = max(b[1][a] for b in cloud.visible_boxes)
            # Precision follows the local extent, never the absolute coordinate.
            result.append(GeometryQuestion(cloud, "maximum", a, lo, hi, (hi - lo) * 0.01))
    return tuple(result)


def parse_number(text: str) -> float:
    """Parse one plain decimal number; other answer formats score zero."""
    value = text.strip()
    if re.fullmatch(r"-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?", value) is None:
        return math.nan
    return float(value)


def score_number(expected: float, actual: float, *, tolerance: float) -> float:
    """Full credit within the documented absolute tolerance, zero outside it."""
    return float(math.isfinite(actual) and abs(expected - actual) <= tolerance)


def prepare_recordings(directory: Path = DATA_DIR) -> None:
    """Write one cloud per recording, atomically, without exposing labels as data.

    Preparation is explicit so importing/listing suites never writes files.
    Re-running replaces these generated recordings rather than appending frames.
    """
    directory.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=directory) as temporary:
        for spec in cloud_specs():
            path = spec.path(Path(temporary))
            with SqliteStore(path=str(path)) as store:
                cloud = PointCloud2.from_numpy(
                    points_for(spec), frame_id=spec.frame_id, timestamp=TIMESTAMP
                )
                store.stream("pointcloud", PointCloud2).append(cloud, ts=TIMESTAMP)
            path.replace(spec.path(directory))
