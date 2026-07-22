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

"""Per-case pass/fail diff between two report JSONs.

The aggregate score can rise while individual cases flip from pass to fail.
Diffing two reports names every flip, so a change is judged case by case
rather than by the average alone. Stateless: which report counts as the
baseline is the caller's decision, typically the last kept run.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import cast

TESTS = ("inc", "fin")

Outcomes = dict[str, dict[str, dict[str, bool]]]


@dataclass
class Flip:
    """One case whose pass/fail state changed on one of the two tests."""

    key: str
    test: str


@dataclass
class ReportDiff:
    fixed: list[Flip]
    broke: list[Flip]
    # Case ids present in only one of the two reports.
    added: list[str]
    removed: list[str]


def outcomes(report: dict[str, object]) -> Outcomes:
    """Pass/fail of both tests for every case in a `run --json` report."""
    out: Outcomes = {}
    for dataset in cast("list[dict[str, object]]", report["datasets"]):
        cases: dict[str, dict[str, bool]] = {}
        for case in cast("list[dict[str, object]]", dataset["cases"]):
            online = cast("dict[str, object]", case["online"])
            final = cast("dict[str, object]", case["final"])
            cases[cast("str", case["id"])] = {
                "inc": bool(online["success"]),
                "fin": bool(final["success"]),
            }
        out[cast("str", dataset["dataset"])] = dict(sorted(cases.items()))
    return out


def diff(old_report: dict[str, object], new_report: dict[str, object]) -> ReportDiff:
    old, new = outcomes(old_report), outcomes(new_report)
    fixed: list[Flip] = []
    broke: list[Flip] = []
    added: list[str] = []
    for dataset, cases in new.items():
        old_cases = old.get(dataset, {})
        for case_id, tests in cases.items():
            key = f"{dataset}/{case_id}"
            if case_id not in old_cases:
                added.append(key)
                continue
            for test in TESTS:
                was, now = old_cases[case_id][test], tests[test]
                if was != now:
                    (fixed if now else broke).append(Flip(key, test))
    removed = [
        f"{dataset}/{case_id}"
        for dataset, cases in old.items()
        for case_id in cases
        if case_id not in new.get(dataset, {})
    ]
    return ReportDiff(fixed, broke, sorted(added), sorted(removed))


# Wall-clock fields legitimately differ between runs of identical code.
TIMING_KEYS = frozenset({"plan_ms", "map_update_ms", "map_build_ms", "add_frame_ms"})


def _strip_timing(value: object) -> object:
    if isinstance(value, dict):
        return {k: _strip_timing(v) for k, v in value.items() if k not in TIMING_KEYS}
    if isinstance(value, list):
        return [_strip_timing(v) for v in value]
    return value


def _walk(path: str, old: object, new: object, out: list[str]) -> None:
    if isinstance(old, dict) and isinstance(new, dict):
        for key in sorted(old.keys() | new.keys()):
            if key not in old or key not in new:
                out.append(f"{path}.{key}: only in {'old' if key in old else 'new'}")
            else:
                _walk(f"{path}.{key}", old[key], new[key], out)
    elif isinstance(old, list) and isinstance(new, list):
        if len(old) != len(new):
            out.append(f"{path}: length {len(old)} != {len(new)}")
            return
        for i, (o, n) in enumerate(zip(old, new, strict=True)):
            _walk(f"{path}[{i}]", o, n, out)
    elif old != new:
        out.append(f"{path}: {old!r} != {new!r}")


def perf_violations(report: dict[str, object]) -> list[str]:
    """Timing stats that exceed the budgets recorded in the report's config."""
    config = cast("dict[str, float]", report.get("config") or {})
    out: list[str] = []
    for stat_key, budget_key in (
        ("plan_ms", "plan_p95_budget_ms"),
        ("map_update_ms", "map_update_p95_budget_ms"),
    ):
        stats = cast("dict[str, float]", report.get(stat_key) or {})
        budget = config.get(budget_key)
        p95 = stats.get("p95")
        if budget is not None and p95 is not None and p95 > budget:
            out.append(f"{stat_key} p95 {p95:.1f}ms exceeds budget {budget:.0f}ms")
    return out


def exact_differences(old_report: dict[str, object], new_report: dict[str, object]) -> list[str]:
    """Every non-timing field that differs between two reports, at full precision.

    Two runs of identical code must produce an empty list. This is the
    determinism gate: it holds for any algorithm under test, present or
    future, because it checks the results rather than the implementation.
    """
    out: list[str] = []
    _walk("report", _strip_timing(old_report), _strip_timing(new_report), out)
    return out
