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

"""Editing a case manifest: add, update, and delete curated cases.

Every mutation snaps endpoints to the final map's standable surface and writes
the manifest, so the CLI and the browser picker share one implementation and
one set of rules.
"""

from __future__ import annotations

from dataclasses import dataclass
import itertools
from typing import TYPE_CHECKING

import numpy as np

from dimos.navigation.nav_3d.evaluator.cases import CASES_DIR, Case, load_suite, save_suite
from dimos.navigation.nav_3d.evaluator.config import EvalConfig
from dimos.navigation.nav_3d.evaluator.final_map import load_or_build_final_map
from dimos.navigation.nav_3d.evaluator.generate import snap_to_surface

if TYPE_CHECKING:
    from pathlib import Path

    from numpy.typing import NDArray

    from dimos.navigation.nav_3d.evaluator.cases import Suite
    from dimos.navigation.nav_3d.evaluator.final_map import FinalMap

Point = tuple[float, float, float]


class CurationError(Exception):
    """A curation request the manifest cannot accept."""


@dataclass
class CaseStore:
    """Mutable view of one dataset's manifest, saved after every change."""

    suite: Suite
    manifest: Path
    surface: NDArray[np.float32]
    cfg: EvalConfig

    def _snap(self, label: str, point: Point, *, required: bool = True) -> Point:
        snapped = snap_to_surface(
            np.asarray(point, dtype=np.float32), self.surface, self.cfg.snap_max_m
        )
        if snapped is None:
            if required:
                raise CurationError(
                    f"{label} {point} is more than {self.cfg.snap_max_m}m from a standable surface"
                )
            # An infeasible goal may sit on geometry with no standable surface.
            print(f"note: {label} {point} is off any standable surface; keeping it as picked")
            return point
        return (float(snapped[0]), float(snapped[1]), float(snapped[2]))

    def _next_id(self, prefix: str) -> str:
        existing = {c.id for c in self.suite.cases}
        return next(
            f"{prefix}_{n:02d}" for n in itertools.count() if f"{prefix}_{n:02d}" not in existing
        )

    def add(
        self,
        start: Point,
        goal: Point,
        tags: list[str],
        case_id: str | None = None,
        expect_fail: bool = False,
    ) -> Case:
        case = Case(
            id=case_id or self._next_id("neg" if expect_fail else "manual"),
            start=self._snap("start", start),
            goal=self._snap("goal", goal, required=not expect_fail),
            tags=_curated_tags(tags, expect_fail),
            expect_fail=expect_fail,
        )
        if any(c.id == case.id for c in self.suite.cases):
            raise CurationError(f"case id {case.id!r} already exists in {self.manifest}")
        self.suite.cases.append(case)
        self.save()
        kind = "negative (must refuse)" if expect_fail else "positive"
        print(f"added {kind} {case.id}: {case.start} -> {case.goal} to {self.manifest}")
        return case

    def update(self, case_id: str, new_id: str, tags: list[str], expect_fail: bool) -> Case:
        case = self.get(case_id)
        if new_id != case_id and any(c.id == new_id for c in self.suite.cases):
            raise CurationError(f"case id {new_id!r} already exists")
        case.id = new_id
        case.tags = _curated_tags(tags, expect_fail)
        case.expect_fail = expect_fail
        if expect_fail:
            case.expect_final_fail = False
        self.save()
        return case

    def delete(self, case_id: str) -> None:
        self.suite.cases.remove(self.get(case_id))
        self.save()

    def get(self, case_id: str) -> Case:
        case = next((c for c in self.suite.cases if c.id == case_id), None)
        if case is None:
            raise CurationError(f"case {case_id!r} not found in {self.manifest}")
        return case

    def save(self) -> None:
        save_suite(self.suite, self.manifest)


def _curated_tags(tags: list[str], expect_fail: bool) -> list[str]:
    """Curated cases always carry manual provenance. The negative tag tracks
    expect_fail rather than being editable text, so the two cannot drift."""
    keep = [t for t in tags if t not in ("manual", "negative")]
    return ["manual", *(["negative"] if expect_fail else []), *keep]


def load_store(dataset: str) -> tuple[CaseStore, FinalMap]:
    """Open a dataset's manifest with the final map and surface it snaps to."""
    manifest = CASES_DIR / f"{dataset}.yaml"
    if not manifest.exists():
        raise CurationError(f"no manifest {manifest}; run ingest first")
    suite = load_suite(manifest)
    cfg = EvalConfig()
    final = load_or_build_final_map(suite.db_path(), suite, cfg)
    planner = cfg.make_planner()
    planner.update_global_map(final.occupied)
    return CaseStore(suite, manifest, planner.surface_map(), cfg), final
