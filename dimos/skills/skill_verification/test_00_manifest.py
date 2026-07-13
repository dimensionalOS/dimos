# Copyright 2025-2026 Dimensional Inc.
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

"""Keep manifest.yaml and the test modules in sync.

If you add a skill test, add its manifest entry (and vice versa). This guards
against a skill silently losing coverage or a manifest entry going stale.
"""

from __future__ import annotations

from pathlib import Path
import re

from dimos.skills.skill_verification._review import load_manifest

HERE = Path(__file__).parent
_TEST_RE = re.compile(r"^test_(\d{2})_.*\.py$")
_REQUIRED_FIELDS = {
    "id", "wishlist_item", "title", "status", "mcp_skills", "module",
    "test_file", "tier", "fixture", "how", "why", "outputs", "review_summary",
}


def _skill_test_files() -> set[str]:
    """test_NN_*.py modules that back a skill (excludes this meta-test, NN==00)."""
    return {
        p.name
        for p in HERE.glob("test_*.py")
        if (m := _TEST_RE.match(p.name)) and m.group(1) != "00"
    }


def test_every_manifest_entry_has_its_test_file() -> None:
    manifest = load_manifest()
    for skill in manifest["skills"]:
        test_file = skill["test_file"]
        assert (HERE / test_file).is_file(), (
            f"manifest entry {skill['id']!r} names {test_file}, which does not exist"
        )


def test_every_skill_test_file_has_a_manifest_entry() -> None:
    manifest = load_manifest()
    declared = {skill["test_file"] for skill in manifest["skills"]}
    for test_file in _skill_test_files():
        assert test_file in declared, (
            f"{test_file} has no manifest.yaml entry — add one so it appears in REVIEW.md"
        )


def test_manifest_entries_are_well_formed() -> None:
    manifest = load_manifest()
    seen_ids: set[str] = set()
    for skill in manifest["skills"]:
        missing = _REQUIRED_FIELDS - set(skill)
        assert not missing, f"skill {skill.get('id')!r} missing manifest fields: {missing}"
        assert skill["id"] not in seen_ids, f"duplicate skill id {skill['id']!r}"
        seen_ids.add(skill["id"])
        assert skill["tier"] in {"headless", "real-data"}, skill["tier"]
        for art in skill["outputs"]:
            assert {"file", "review"} <= set(art), f"{skill['id']} output entry needs file+review"
