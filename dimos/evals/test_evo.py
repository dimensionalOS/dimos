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

"""The autoresearch loop's own guardrails: slices, harness, gates.

Offline — reads the committed rows, imports the suites, and exercises the
gate helpers. No datasets, no model, no evo.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pytest

from dimos.evals import generate, split
from dimos.evals.generate import Row
from dimos.evals.tool_evo_bench import SLICE_TAGS, family_of, means, pool, select
from dimos.evals.tool_evo_gate import TARGET, _banned_reach, floors

SUITES_DIR = Path(__file__).parent / "suites"
SLICED = ("clearance", "route", "glass")


@pytest.fixture(scope="module")
def rows() -> list[Row]:
    """Every sliced row, as the suites build them."""
    out: list[Row] = []
    for name in SLICED:
        path = SUITES_DIR / f"go2_pointcloud_{name}_vqa.json"
        out += split.assign(json.loads(path.read_text()))
    return out


# -- slices ----------------------------------------------------------------------


def test_assign_is_total_and_order_preserving() -> None:
    raw = json.loads((SUITES_DIR / "go2_pointcloud_clearance_vqa.json").read_text())
    assigned = split.assign(raw)
    assert [r["id"] for r in assigned] == [r["id"] for r in raw]
    assert {str(r["split"]) for r in assigned} <= set(split.SLICES)
    assert all("split" in r for r in assigned)
    assert split.assign(raw) == assigned  # deterministic


def test_no_group_straddles_the_split(rows: list[Row]) -> None:
    """The whole point: a holdout group's twins are never in train."""
    seen: dict[str, str] = {}
    for row in rows:
        if row["split"] == "spare":
            continue
        group = split._group(row)
        assert seen.setdefault(group, str(row["split"])) == row["split"], group


def test_spares_are_near_duplicates(rows: list[Row]) -> None:
    kept = [r for r in rows if r["split"] != "spare"]
    for spare in (r for r in rows if r["split"] == "spare"):
        near = split.NEAR_S.get(str(spare["family"]), split.DEFAULT_NEAR_S)
        twins = [
            k
            for k in kept
            if (k["dataset"], k["family"], split._lane(k), k["a"])
            == (spare["dataset"], spare["family"], split._lane(spare), spare["a"])
            and abs(split._time(k) - split._time(spare)) < near
        ]
        assert twins, f"{spare['id']} was spared with no twin to stand in for it"


def test_both_slices_stay_answerable(rows: list[Row]) -> None:
    """Guards the constants: retuning NEAR_S or BLOCK_S must not empty a slice
    or leave one holding a single answer, which would make its mean unreadable."""
    table = split.counts(rows)
    families = {family for family, _ in table}
    for family in families:
        answers = {a for (f, _), counts in table.items() if f == family for a in counts}
        for slice_ in ("train", "holdout"):
            got = table.get((family, slice_), {})
            assert sum(got.values()) >= 4, f"{family}/{slice_} is too small: {got}"
            assert set(got) == answers, f"{family}/{slice_} is missing answers: {got}"


def test_split_tag_reaches_the_case() -> None:
    row: Row = {
        "id": "x_clearance_t1_east",
        "family": "clearance",
        "type": "mcq",
        "q": "?",
        "a": "clear",
        "choices": ["clear", "blocked"],
        "context": [["lidar", [1.0, 1.1]]],
        "dataset": "go2_short",
        "split": "holdout",
    }
    (case,) = generate.cases([row])
    assert {"clearance", "mcq", "holdout"} <= case.tags


# -- harness ---------------------------------------------------------------------


def test_every_case_carries_exactly_one_slice() -> None:
    slices = {"train", "holdout", "spare", "frozen"}
    for case in select("all"):
        assert len(case.tags & slices) == 1, case.id


def test_slices_partition_the_suites() -> None:
    everything = {c.id for c in select("all")}
    parts = [{c.id for c in select(s)} for s in ("train", "holdout", "spare", "frozen")]
    assert set().union(*parts) == everything
    assert sum(len(p) for p in parts) == len(everything)  # disjoint
    assert {c.id for c in select("bench")} == parts[0] | parts[3]


def test_limit_touches_every_family() -> None:
    families = {family_of(c) for c in select("bench")}
    assert {family_of(c) for c in select("bench", limit=len(families))} == families


def test_family_of_reads_the_leftover_tag() -> None:
    (case,) = [c for c in select("frozen") if c.id == "go2_short_extent_t5"]
    assert family_of(case) == "extent"


def test_pool_is_family_weighted_over_the_scored_families() -> None:
    families = means([("route", 1.0), ("route", 0.0), ("crossing", 1.0), ("extent", 0.0)])
    assert families == {"crossing": 1.0, "extent": 0.0, "route": 0.5}
    assert pool(families) == pytest.approx(0.75)  # extent rides along, cannot buy score
    assert pool({"extent": 0.9, "zspan": 0.7}) == pytest.approx(0.8)  # frozen-only run


def test_bench_slice_is_train_plus_frozen() -> None:
    assert SLICE_TAGS["bench"] == frozenset({"train", "frozen"})


# -- gates -----------------------------------------------------------------------


def test_the_encoder_reaches_nowhere_it_should_not() -> None:
    assert _banned_reach((Path(__file__).parents[2] / TARGET).read_text()) == []


def test_banned_reach_catches_importing_the_answer() -> None:
    source = (
        "from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar\n"
        "def agent_encode(self):\n"
        "    return open('/tmp/answers.json').read()\n"
    )
    found = _banned_reach(source)
    assert any("dimos.navigation" in f for f in found)
    assert any("open" in f for f in found)


def _floors_args(tmp_path: Path, recorded: dict[str, float], measured: dict[str, float]):
    (tmp_path / "floors.json").write_text(json.dumps({"families": recorded}))
    (tmp_path / "bench.json").write_text(json.dumps({"families": measured}))
    return argparse.Namespace(
        artifact=str(tmp_path / "bench.json"),
        floors=str(tmp_path / "floors.json"),
        tolerance=0.05,
    )


def test_floors_pass_within_tolerance(tmp_path: Path) -> None:
    args = _floors_args(tmp_path, {"extent": 0.9}, {"extent": 0.87, "route": 0.4})
    assert floors(args) == 0


def test_floors_fail_on_a_regression(tmp_path: Path) -> None:
    args = _floors_args(tmp_path, {"extent": 0.9}, {"extent": 0.6})
    assert floors(args) == 1


def test_floors_fail_when_a_family_vanishes(tmp_path: Path) -> None:
    args = _floors_args(tmp_path, {"extent": 0.9}, {"route": 0.9})
    assert floors(args) == 1


def test_floors_fail_without_a_baseline(tmp_path: Path) -> None:
    args = argparse.Namespace(
        artifact=str(tmp_path / "bench.json"), floors=str(tmp_path / "nope.json"), tolerance=0.05
    )
    assert floors(args) == 1
