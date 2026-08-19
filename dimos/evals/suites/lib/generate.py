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

"""Eval-row generators.

Ground truth is computed analytically from a *privileged* modality (full-res
clouds, odom); the emitted case quizzes a different — or lossily encoded —
surface. Rows are pure data; :func:`cases` maps them onto
:class:`~dimos.evals.types.EvalCase` values over a :class:`Dataset`.

Row schema::

    {"id", "family", "type": "numeric"|"mcq", "q", "a",
     "band" (numeric) | "choices" (mcq),
     "context": [[stream, [t0, t1]], ...], "dataset"}
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Sequence
from contextlib import contextmanager
from typing import TYPE_CHECKING, Any, cast

import numpy as np

from dimos.evals.environments.dataset import Dataset
from dimos.evals.scorers import choice, exact, first_number, within
from dimos.evals.types import EvalCase, Outcome, Select
from dimos.memory.cli.dataset import open_dataset

if TYPE_CHECKING:
    from dimos.memory.store.base import Store

Row = dict[str, object]

COMPASS = ("east", "northeast", "north", "northwest", "west", "southwest", "south", "southeast")
BODY_Z = (0.15, 1.0)  # world-frame band: above floor returns, below robot cap
VOXEL = 0.2


@contextmanager
def _dataset(name: str) -> Iterator[Store]:
    store = open_dataset(name)
    try:
        yield store
    finally:
        store.stop()


def _compass_of(v: np.ndarray) -> str:
    return COMPASS[int(np.round(np.arctan2(v[1], v[0]) / (np.pi / 4))) % 8]


def _voxels2d(pts: np.ndarray, cell: float = VOXEL) -> set[tuple[int, int]]:
    return set(map(tuple, np.floor(pts[:, :2] / cell).astype(int)))


def _cloud_at(store: Store, t: float) -> tuple[np.ndarray, list[list[object]]]:
    """Points of the last cloud at or before ``t``, plus a context window selecting it."""
    lidar = store.streams.lidar
    obs = lidar.range_time(0, t).to_list()[-1]
    ts = obs.ts - lidar.first().ts
    return obs.data.points_f32(), [["lidar", [round(ts - 0.05, 2), round(ts + 0.05, 2)]]]


def displacement_rows(dataset: str, windows: Sequence[tuple[float, float]]) -> list[Row]:
    """Straight-line displacement over each window (odom is the privileged truth;
    the case quizzes the encoded odom summary). Sampling-invariant ground truth."""
    with _dataset(dataset) as store:
        rows: list[Row] = []
        for t1, t2 in windows:
            obs = store.streams.odom.range_time(t1, t2).to_list()
            if len(obs) < 2:
                continue
            d = (obs[-1].data.position - obs[0].data.position).length()
            rows.append(
                {
                    "id": f"{dataset}_disp_{t1:g}_{t2:g}",
                    "family": "displacement",
                    "type": "numeric",
                    "q": "How far in a straight line is your final position from your "
                    "position at the first shown observation, in meters? Answer with a single "
                    "number.",
                    "a": round(d, 1),
                    "band": max(1.0, d * 0.4),
                    "context": [["odom", [t1, t2]]],
                    "dataset": dataset,
                }
            )
        return rows


def path_length_rows(dataset: str, windows: Sequence[tuple[float, float]]) -> list[Row]:
    """Integrated path length per window. Deliberately hard on a downsampled
    encoding — expect partial credit; that gap is the finding."""
    with _dataset(dataset) as store:
        rows: list[Row] = []
        for t1, t2 in windows:
            path, prev = 0.0, None
            for obs in store.streams.odom.range_time(t1, t2):
                p = obs.data.position
                if prev is not None:
                    path += (p - prev).length()
                prev = p
            if prev is None:
                continue
            rows.append(
                {
                    "id": f"{dataset}_path_{t1:g}_{t2:g}",
                    "family": "path_length",
                    "type": "numeric",
                    "q": "Roughly how many meters did you travel in total over these "
                    "observations (path length, not displacement)? Answer with a single "
                    "number.",
                    "a": round(path, 1),
                    "band": max(2.0, path * 0.5),
                    "context": [["odom", [t1, t2]]],
                    "dataset": dataset,
                }
            )
        return rows


def extent_rows(dataset: str, timestamps: Sequence[float]) -> list[Row]:
    """Largest horizontal bbox side of a single cloud."""
    with _dataset(dataset) as store:
        rows: list[Row] = []
        for t in timestamps:
            pts, context = _cloud_at(store, t)
            a = round(float(max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1]))), 2)
            rows.append(
                {
                    "id": f"{dataset}_extent_t{t:g}",
                    "family": "extent",
                    "type": "numeric",
                    "q": "Consider the mapped point cloud shown. What is its largest "
                    "horizontal extent (the longer side of its axis-aligned "
                    "bounding box in the x-y plane), in meters? Answer with a single number.",
                    "a": a,
                    "band": round(max(0.5, 0.10 * a), 2),
                    "context": context,
                    "dataset": dataset,
                }
            )
        return rows


def zspan_rows(dataset: str, timestamps: Sequence[float]) -> list[Row]:
    """Vertical span of a single cloud."""
    with _dataset(dataset) as store:
        rows: list[Row] = []
        for t in timestamps:
            pts, context = _cloud_at(store, t)
            rows.append(
                {
                    "id": f"{dataset}_zspan_t{t:g}",
                    "family": "zspan",
                    "type": "numeric",
                    "q": "Consider the mapped point cloud shown. What is its vertical "
                    "span (max z minus min z), in meters? Answer with a single number.",
                    "a": round(float(np.ptp(pts[:, 2])), 2),
                    "band": 0.3,
                    "context": context,
                    "dataset": dataset,
                }
            )
        return rows


def footprint_rows(dataset: str, timestamps: Sequence[float], *, cell: float = VOXEL) -> list[Row]:
    """Mapped floor area of a single cloud (occupied x-y cells)."""
    with _dataset(dataset) as store:
        rows: list[Row] = []
        for t in timestamps:
            pts, context = _cloud_at(store, t)
            area = round(len(_voxels2d(pts, cell)) * cell * cell, 1)
            rows.append(
                {
                    "id": f"{dataset}_area_t{t:g}",
                    "family": "area",
                    "type": "numeric",
                    "q": "Consider the local map point cloud shown. Roughly how many "
                    "square meters of floor plan does it cover (footprint of the "
                    "mapped points projected onto the x-y plane)? Answer with a single number.",
                    "a": area,
                    "band": round(max(3.0, 0.25 * area), 1),
                    "context": context,
                    "dataset": dataset,
                }
            )
        return rows


def nearest_obstacle_rows(
    dataset: str, timestamps: Sequence[float], *, z_band: tuple[float, float] = BODY_Z
) -> list[Row]:
    """Horizontal clearance from the robot's own pose to the nearest body-height
    point — the only family that needs two streams (cloud + odom)."""
    with _dataset(dataset) as store:
        rows: list[Row] = []
        for t in timestamps:
            pts, context = _cloud_at(store, t)
            odom = store.streams.odom.range_time(0, t).to_list()[-1].data.position
            band = pts[(pts[:, 2] >= z_band[0]) & (pts[:, 2] <= z_band[1])]
            d = np.hypot(band[:, 0] - odom.x, band[:, 1] - odom.y)
            d = d[d > 0.15]  # the robot's own return
            a = round(float(d.min()), 2)
            rows.append(
                {
                    "id": f"{dataset}_nearest_t{t:g}",
                    "family": "nearest",
                    "type": "numeric",
                    "q": "You are the robot; your current pose is the odom observation "
                    f"shown. Using the mapped point cloud, how far away is the "
                    f"nearest obstacle point at body height (z between {z_band[0]} "
                    f"and {z_band[1]} m), horizontal distance in meters? Answer with a single "
                    "number.",
                    "a": a,
                    "band": round(max(0.25, 0.30 * a), 2),
                    "context": [
                        *context,
                        ["odom", [round(max(0.0, t - 0.5), 2), round(t + 0.1, 2)]],
                    ],
                    "dataset": dataset,
                }
            )
        return rows


def map_shift_rows(dataset: str, windows: Sequence[tuple[float, float]]) -> list[Row]:
    """How far the mapped region's center moved across a window. Meaningful only
    where the cloud is a rolling local map, not an accumulated one."""
    with _dataset(dataset) as store:
        rows: list[Row] = []
        for w0, w1 in windows:
            frames = store.streams.lidar.range_time(w0, w1).to_list()
            shift = frames[-1].data.points_f32()[:, :2].mean(axis=0) - frames[0].data.points_f32()[
                :, :2
            ].mean(axis=0)
            s = round(float(np.hypot(*shift)), 1)
            rows.append(
                {
                    "id": f"{dataset}_shift_{w0:g}_{w1:g}",
                    "family": "shift",
                    "type": "numeric",
                    "q": "You are shown a sequence of local map point clouds over "
                    "time (world coordinates). The local map follows the robot. "
                    "Roughly how far did the center of the mapped region move "
                    "from the first shown cloud to the last, in meters? Answer with a single "
                    "number.",
                    "a": s,
                    "band": round(max(1.0, 0.30 * s), 1),
                    "context": [["lidar", [w0, w1]]],
                    "dataset": dataset,
                }
            )
        return rows


def map_direction_rows(
    dataset: str, windows: Sequence[tuple[float, float]], *, min_shift: float = 1.0
) -> list[Row]:
    """Compass direction the mapped region's center travelled. Windows whose
    shift is under ``min_shift`` are skipped — the answer would be noise."""
    with _dataset(dataset) as store:
        rows: list[Row] = []
        for w0, w1 in windows:
            frames = store.streams.lidar.range_time(w0, w1).to_list()
            shift = frames[-1].data.points_f32()[:, :2].mean(axis=0) - frames[0].data.points_f32()[
                :, :2
            ].mean(axis=0)
            if np.hypot(*shift) < min_shift:
                continue
            rows.append(
                {
                    "id": f"{dataset}_direction_{w0:g}_{w1:g}",
                    "family": "direction",
                    "type": "mcq",
                    "q": "You are shown a sequence of local map point clouds over "
                    "time (world frame: +x is east, +y is north). The local map "
                    "follows the robot. In which compass direction did the "
                    "mapped region's center move overall? Answer with exactly "
                    "one of: " + ", ".join(COMPASS) + ".",
                    "a": _compass_of(shift),
                    "choices": list(COMPASS),
                    "context": [["lidar", [w0, w1]]],
                    "dataset": dataset,
                }
            )
        return rows


def area_trend_rows(dataset: str, windows: Sequence[tuple[float, float]]) -> list[Row]:
    """Grow / shrink / same over a window — a 3-way MCQ, so a blind guess floors
    at ~0.33 rather than the ~0.5 a yes/no would give away."""
    with _dataset(dataset) as store:
        rows: list[Row] = []
        for w0, w1 in windows:
            frames = store.streams.lidar.range_time(w0, w1).to_list()
            first = _voxels2d(frames[0].data.points_f32())
            last = _voxels2d(frames[-1].data.points_f32())
            ratio = len(last) / max(1, len(first))
            rows.append(
                {
                    "id": f"{dataset}_areatrend_{w0:g}_{w1:g}",
                    "family": "areatrend",
                    "type": "mcq",
                    "q": "You are shown a sequence of mapped point clouds over time. "
                    "From the first shown cloud to the last, does the mapped "
                    "floor area grow, shrink, or stay about the same? Answer "
                    "with exactly one word: grow, shrink, or same.",
                    "a": "grow" if ratio > 1.05 else ("shrink" if ratio < 0.95 else "same"),
                    "choices": ["grow", "shrink", "same"],
                    "context": [["lidar", [w0, w1]]],
                    "dataset": dataset,
                }
            )
        return rows


def coverage_direction_rows(
    dataset: str, windows: Sequence[tuple[float, float]], *, min_new_cells: int = 50
) -> list[Row]:
    """Compass direction of newly mapped coverage, for accumulated maps (where
    :func:`map_direction_rows` degenerates because the center barely moves)."""
    with _dataset(dataset) as store:
        rows: list[Row] = []
        for w0, w1 in windows:
            frames = store.streams.lidar.range_time(w0, w1).to_list()
            first = _voxels2d(frames[0].data.points_f32())
            new = np.array(sorted(_voxels2d(frames[-1].data.points_f32()) - first))
            if len(new) < min_new_cells:
                continue
            gained = new.mean(axis=0) * VOXEL - np.array(sorted(first)).mean(axis=0) * VOXEL
            rows.append(
                {
                    "id": f"{dataset}_coverage_{w0:g}_{w1:g}",
                    "family": "direction",
                    "type": "mcq",
                    "q": "You are shown a sequence of mapped point clouds over time "
                    "(world frame: +x is east, +y is north). In which compass "
                    "direction, relative to the center of the first shown cloud, "
                    "did the map gain most of its new coverage? Answer with "
                    "exactly one of: " + ", ".join(COMPASS) + ".",
                    "a": _compass_of(gained),
                    "choices": list(COMPASS),
                    "context": [["lidar", [w0, w1]]],
                    "dataset": dataset,
                }
            )
        return rows


def _select(name: str, window: tuple[float, ...]) -> Select:
    """One context entry -> the stream the case contains."""
    return lambda s: s.streams[name].range_time(*window)


def _grade(row: Row) -> Callable[[Outcome], float]:
    """The scorer a row's type implies. A reply the parser cannot read is a
    wrong answer, not a broken run: it scores 0."""
    if row["type"] == "numeric":
        expected, band = float(cast("float", row["a"])), float(cast("float", row["band"]))

        def numeric(o: Outcome) -> float:
            try:
                return within(band)(expected, first_number(o.trajectory.final_answer))
            except ValueError:
                return 0.0

        return numeric
    answer, parse = str(row["a"]), choice(cast("list[str]", row["choices"]))

    def mcq(o: Outcome) -> float:
        try:
            return exact(answer, parse(o.trajectory.final_answer))
        except ValueError:
            return 0.0

    return mcq


def cases(rows: Sequence[Row], *, tags: frozenset[str] = frozenset()) -> list[EvalCase]:
    """Rows -> cases. The mirror of the schema above: numeric rows score on a
    band, mcq rows on exact match against the named options."""
    return [
        EvalCase(
            id=str(row["id"]),
            inputs=str(row["q"]),
            environment=Dataset(
                str(row["dataset"]),
                select=tuple(
                    _select(str(entry[0]), tuple(cast("list[float]", entry[1])))
                    for entry in cast("list[list[Any]]", row["context"])
                ),
            ),
            grade=_grade(row),
            tags=tags | {str(row["family"]), str(row["type"])},
        )
        for row in rows
    ]
