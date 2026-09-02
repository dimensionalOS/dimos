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

Ground truth is computed analytically from a *privileged* modality (odom, or
full-res clouds); the emitted case quizzes a different — or lossily encoded —
surface. Rows are pure data; :func:`cases` maps them onto
:class:`~dimos.evals.types.EvalCase` values over a :class:`Dataset`.

Row schema::

    {"id", "family", "type": "numeric"|"mcq", "q", "a",
     "band" (numeric) | "choices" (mcq),
     "context": [[stream, [t0, t1]], ...], "dataset",
     "split" (optional): "train" | "holdout" | "spare"}

A suite family with its own row types or context shapes extends the two
hooks of :func:`cases`, ``grade_of`` and ``select_of``, and falls back to
the ones here for the rest — see ``suites/pointcloud/lib/generate.py``.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator, Sequence
from contextlib import contextmanager
from typing import TYPE_CHECKING, Any, cast

from dimos.evals.environments.dataset import Dataset
from dimos.evals.scorers import first_number, grade_choice, within
from dimos.evals.types import EvalCase, Outcome, Select
from dimos.memory.cli.dataset import open_dataset

if TYPE_CHECKING:
    from dimos.memory.store.base import Store

Row = dict[str, object]


@contextmanager
def _dataset(name: str) -> Iterator[Store]:
    store = open_dataset(name)
    try:
        yield store
    finally:
        store.stop()


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


def select_of(entry: Sequence[Any]) -> Select:
    """One context entry, ``[stream, [t0, t1]]`` -> the stream the case contains."""
    if len(entry) != 2:
        raise ValueError(f"context entry has fields this reader does not know: {entry!r}")
    name, window = str(entry[0]), tuple(cast("list[float]", entry[1]))
    return lambda s: s.streams[name].range_time(*window)


def grade_of(row: Row) -> Callable[[Outcome], float]:
    """The scorer a row's type implies: numeric rows score on a band, mcq rows
    on exact match against the named options. A reply the parser cannot read
    is a wrong answer, not a broken run: it scores 0."""
    if row["type"] == "numeric":
        expected, band = float(cast("float", row["a"])), float(cast("float", row["band"]))

        def numeric(o: Outcome) -> float:
            try:
                return within(band)(expected, first_number(o.trajectory.final_answer))
            except ValueError:
                return 0.0

        return numeric
    if row["type"] == "mcq":
        return grade_choice(cast("list[str]", row["choices"]), str(row["a"]))
    raise ValueError(f"row {row['id']!r} has a type this reader does not know: {row['type']!r}")


def cases(
    rows: Sequence[Row],
    *,
    tags: frozenset[str] = frozenset(),
    grade_of: Callable[[Row], Callable[[Outcome], float]] = grade_of,
    select_of: Callable[[Sequence[Any]], Select] = select_of,
) -> list[EvalCase]:
    """Rows -> cases, the mirror of the schema above. A row's family, type
    and split (when it has one) become tags."""
    return [
        EvalCase(
            id=str(row["id"]),
            inputs=str(row["q"]),
            environment=Dataset(
                str(row["dataset"]),
                select=tuple(select_of(entry) for entry in cast("list[list[Any]]", row["context"])),
            ),
            grade=grade_of(row),
            tags=tags | {str(row[key]) for key in ("family", "type", "split") if key in row},
        )
        for row in rows
    ]
