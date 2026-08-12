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

"""Editing a case manifest, shared by the CLI and the browser picker."""

from __future__ import annotations

from dataclasses import dataclass
import itertools
from typing import TYPE_CHECKING

from dimos.navigation.nav_3d.evaluator.cases import (
    Case,
    load_suite,
    manifest_path,
    save_suite,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.navigation.nav_3d.evaluator.cases import Suite

logger = setup_logger()

Point = tuple[float, float, float]


class CurationError(Exception):
    """A curation request the manifest cannot accept."""


def _provenance(tags: list[str]) -> str:
    return "auto" if "auto" in tags else "manual"


@dataclass
class CaseStore:
    """Mutable view of one dataset's manifest, saved after every change."""

    suite: Suite

    @property
    def path(self) -> Path:
        """The manifest this store saves to, set when the suite was loaded."""
        assert self.suite.path is not None, "a curated suite is always loaded from a file"
        return self.suite.path

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
        expect_final_fail: bool = False,
    ) -> Case:
        if expect_fail and expect_final_fail:
            raise CurationError("a case cannot be both expect_fail and expect_final_fail")
        case = Case(
            id=case_id or self._next_id("neg" if expect_fail else "manual"),
            start=start,
            goal=goal,
            tags=_curated_tags(tags, expect_fail, expect_final_fail),
            expect_fail=expect_fail,
            expect_final_fail=expect_final_fail,
        )
        if any(c.id == case.id for c in self.suite.cases):
            raise CurationError(f"case id {case.id!r} already exists in {self.path}")
        self.suite.cases.append(case)
        self.save()
        kind = "negative (should fail planning)" if expect_fail else "positive"
        logger.info(
            "added case",
            kind=kind,
            case=case.id,
            start=case.start,
            goal=case.goal,
            manifest=self.path,
        )
        return case

    def update(
        self,
        case_id: str,
        new_id: str,
        tags: list[str],
        *,
        expect_fail: bool,
        expect_final_fail: bool = False,
    ) -> Case:
        if expect_fail and expect_final_fail:
            raise CurationError("a case cannot be both expect_fail and expect_final_fail")
        case = self.get(case_id)
        if new_id != case_id and any(c.id == new_id for c in self.suite.cases):
            raise CurationError(f"case id {new_id!r} already exists")
        case.id = new_id
        case.tags = _curated_tags(tags, expect_fail, expect_final_fail, _provenance(case.tags))
        case.expect_fail = expect_fail
        case.expect_final_fail = expect_final_fail
        self.save()
        return case

    def delete(self, case_id: str) -> None:
        self.suite.cases.remove(self.get(case_id))
        self.save()

    def get(self, case_id: str) -> Case:
        case = next((c for c in self.suite.cases if c.id == case_id), None)
        if case is None:
            raise CurationError(f"case {case_id!r} not found in {self.path}")
        return case

    def save(self) -> None:
        save_suite(self.suite, self.path)


PROVENANCE_TAGS = ("auto", "manual")


def _curated_tags(
    tags: list[str],
    expect_fail: bool,
    expect_final_fail: bool = False,
    provenance: str = "manual",
) -> list[str]:
    """Rewrite a case's tags around exactly one provenance tag."""
    keep = [t for t in tags if t not in (*PROVENANCE_TAGS, "negative", "dynamic")]
    extra = []
    if expect_fail:
        extra.append("negative")
    if expect_final_fail:
        extra.append("dynamic")
    return [provenance, *extra, *keep]


def load_store(dataset: str) -> CaseStore:
    """Open a dataset's manifest for editing."""
    manifest = manifest_path(dataset)
    if not manifest.exists():
        raise CurationError(f"no manifest {manifest}; run ingest first")
    return CaseStore(load_suite(manifest))
