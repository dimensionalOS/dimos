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

"""Train / holdout slices over generated rows.

Two rows taken seconds apart, at the same bearing off the same wall, with the
same answer are one question asked twice. Paying a model for both is waste,
and putting one in train and its twin in holdout makes the holdout measure
nothing. So rows are grouped by the scene moment they came from — one glass
pane, or one 30 s block of one recording — and a whole group goes to one
slice.

Every row leaves with exactly one ``split``, which :func:`generate.cases`
carries onto the case as a tag:

``train``    what the optimizer sees (``--tags train``).
``holdout``  scored only at gate time; no group here appears in train.
``spare``    near-duplicate of a kept row; in neither slice, still runnable.

The geometry suite is not sliced — it is the frozen regression set, tagged
``frozen`` where it is built.

Print the table::

    python -m dimos.evals.temp.split
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
import re
from typing import Any, cast

from dimos.evals.generate import Row

NEAR_S: dict[str, float] = {
    # Below this gap, two rows in the same lane with the same answer are the
    # same question. Per family because sampling densities differ: the glass
    # probe walks a pane every 2 s and a 4 s move changes the viewpoint, while
    # a corridor 8 s later is usually the same corridor.
    "crossing": 4.0,
    "clearance": 10.0,
    "route": 10.0,
}
DEFAULT_NEAR_S = 10.0

BLOCK_S = 30.0  # a "scene moment" for the families sampled along a trajectory
HOLDOUT_STRIDE = 3  # every third group of each family, so ~1/3 of the groups

SLICES = ("train", "holdout", "spare")


def _time(row: Row) -> float:
    """Start of the row's first context window — when it was asked."""
    context = cast("Sequence[Sequence[Any]]", row["context"])
    return float(context[0][1][0])


def _lane(row: Row) -> str:
    """What varies inside a scene moment: the bearing asked about, or the surface.

    Crossing rows name their surface (``partition_a``, ``meeting_room``, or
    ``open`` for the gates the robot drove through); the rest end in a compass
    word.
    """
    ident = str(row["id"])
    if str(row["family"]) == "crossing":
        match = re.search(r"_crossing_(.+?)_t[\d.]+", ident)
        tag = match.group(1) if match else "open"
        return "open" if tag.startswith("open") else tag
    return ident.rsplit("_", 1)[-1]


def _group(row: Row) -> str:
    """The split unit. One glass pane is one group however long it is looked at;
    everything else blocks by time, because a recording moves on."""
    lane = _lane(row)
    if str(row["family"]) == "crossing" and lane != "open":
        return f"{row['dataset']}/{row['family']}/{lane}"
    return f"{row['dataset']}/{row['family']}/t{int(_time(row) // BLOCK_S)}"


def _order(row: Row) -> tuple[str, str, str, str, float]:
    return (
        str(row["dataset"]),
        str(row["family"]),
        _lane(row),
        str(row["a"]),
        _time(row),
    )


def _holdout_groups(rows: Iterable[Row]) -> set[str]:
    """Every third group of each family, ordered by when the group first appears.

    Per family, so no family loses its holdout to another family's group count,
    and by first appearance, so the answer is stable when rows are regenerated
    at different sampling densities.
    """
    first: dict[str, tuple[str, float]] = {}
    for row in rows:
        group, t = _group(row), _time(row)
        known = first.get(group)
        if known is None or t < known[1]:
            first[group] = (str(row["family"]), t)
    by_family: dict[str, list[tuple[float, str]]] = {}
    for group, (family, t) in first.items():
        by_family.setdefault(family, []).append((t, group))
    return {
        group
        for groups in by_family.values()
        for i, (_, group) in enumerate(sorted(groups))
        if i % HOLDOUT_STRIDE == HOLDOUT_STRIDE - 1
    }


def assign(rows: Sequence[Row]) -> list[Row]:
    """Rows in, the same rows out with ``split`` set. Input order preserved.

    Duplicates are dropped first, so a group is never held out on the strength
    of rows that are themselves redundant.
    """
    kept: list[Row] = []
    tagged: dict[str, Row] = {}
    last: dict[tuple[str, str, str, str], float] = {}
    for row in sorted(rows, key=_order):
        key = (str(row["dataset"]), str(row["family"]), _lane(row), str(row["a"]))
        near = NEAR_S.get(str(row["family"]), DEFAULT_NEAR_S)
        t = _time(row)
        if key in last and t - last[key] < near:
            tagged[str(row["id"])] = {**row, "split": "spare"}
        else:
            last[key] = t
            kept.append(row)
    holdout = _holdout_groups(kept)
    for row in kept:
        slice_ = "holdout" if _group(row) in holdout else "train"
        tagged[str(row["id"])] = {**row, "split": slice_}
    return [tagged[str(row["id"])] for row in rows]


def counts(rows: Sequence[Row]) -> dict[tuple[str, str], dict[str, int]]:
    """``(family, split) -> {answer: n}`` — the table the docs quote."""
    table: dict[tuple[str, str], dict[str, int]] = {}
    for row in rows:
        answers = table.setdefault((str(row["family"]), str(row["split"])), {})
        answers[str(row["a"])] = answers.get(str(row["a"]), 0) + 1
    return table


def main() -> None:
    """Print the slice table for the committed sliced suites."""
    import json
    from pathlib import Path

    rows: list[Row] = []
    for name in ("clearance", "route", "glass"):
        path = Path(__file__).parents[1] / "suites" / f"go2_pointcloud_{name}_vqa.json"
        rows += assign(json.loads(path.read_text()))
    for (family, slice_), answers in sorted(counts(rows).items()):
        total = sum(answers.values())
        breakdown = " ".join(f"{a}={n}" for a, n in sorted(answers.items()))
        print(f"{family:10} {slice_:8} n={total:3}  {breakdown}")


if __name__ == "__main__":
    main()
