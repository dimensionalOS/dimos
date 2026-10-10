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

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from dimos.navigation.bench.ground_truth import GO2, GroundTruth
from dimos.navigation.bench.suite import (
    MINED,
    NARROW_DOOR_M,
    OBSTRUCTED,
    STRESSORS,
    FreezeConfig,
    Manifest,
    freeze,
)

SMALL = FreezeConfig(seeds=(1, 3), stressor_samples=60, samples_per_scene=25, cases_per_bin=1)


STEPS: list[tuple[int, int, str]] = []


@pytest.fixture(scope="module")
def manifest() -> Manifest:
    return freeze(SMALL, lambda *step: STEPS.append(step))


def test_every_case_is_routable_and_pinned_to_its_scene(manifest: Manifest) -> None:
    assert manifest.cases
    assert len({c.id for c in manifest.cases}) == len(manifest.cases)
    for case in manifest.cases:
        assert case.route_length >= SMALL.min_route_m
        assert case.difficulty.min_clearance >= GO2.radius
        assert case.scene().digest() == case.scene_digest
    case = next(c for c in manifest.cases if c.tag == "narrow_door")
    gt = GroundTruth(case.scene())
    assert gt.stands(case.start)
    route = gt.route(case.start, case.goal)
    assert route is not None
    assert route.length == pytest.approx(case.route_length)


def test_stressors_match_their_templates(manifest: Manifest) -> None:
    by_tag: dict[str, list] = {}
    for case in manifest.cases:
        by_tag.setdefault(case.tag, []).append(case)
    for case in by_tag["narrow_door"]:
        assert case.params == {"door_width": NARROW_DOOR_M}
        assert case.difficulty.doors >= 1
        assert case.difficulty.min_clearance <= NARROW_DOOR_M / 2 + 0.05
    for case in by_tag["doorway_clutter"]:
        assert case.params == {"door_clutter": True}
        assert case.difficulty.doors >= 1
    for case in by_tag["behind_clutter"]:
        assert case.difficulty.doors == 0
        assert case.difficulty.detour >= OBSTRUCTED
        assert case.difficulty.clutter_near >= 1
    for case in by_tag.get("against_wall", []):
        assert case.difficulty.doors >= 1
    for case in by_tag["around_table"]:
        assert case.difficulty.doors == 0
        assert case.difficulty.detour >= OBSTRUCTED
    for case in by_tag.get("dead_end", []):
        assert case.difficulty.detour >= 1.4
    for case in by_tag.get("three_doors", []):
        assert case.difficulty.doors >= 3
    assert by_tag[MINED]
    assert all(c.params == {} and c.start[:2] != (1.0, 1.0) for c in by_tag[MINED])


def test_every_stressor_is_a_case_or_a_rejection_per_scene(manifest: Manifest) -> None:
    seen = {(c.seed, c.tag) for c in manifest.cases if c.tag != MINED}
    rejected = {(r.seed, r.tag) for r in manifest.rejections}
    assert not seen & rejected
    assert seen | rejected == {(seed, s.name) for seed in SMALL.seeds for s in STRESSORS}
    assert sum(manifest.mined_rejected.values()) > 0


def test_manifest_round_trips_through_json(manifest: Manifest, tmp_path: Path) -> None:
    path = tmp_path / "suite.json"
    manifest.save(path)
    loaded = Manifest.load(path)
    assert loaded == manifest
    loaded.check_drift()


def test_drift_check_catches_a_changed_scene(manifest: Manifest) -> None:
    drifted = replace(manifest, cases=[replace(manifest.cases[0], scene_digest="0" * 16)])
    with pytest.raises(RuntimeError, match=manifest.cases[0].id):
        drifted.check_drift()


def test_freeze_is_deterministic_and_reports_every_scene(manifest: Manifest) -> None:
    assert freeze(SMALL) == manifest
    assert [done for done, _, _ in STEPS] == [0, 1, 2, 3]
    assert all(total == 3 for _, total, _ in STEPS)
    assert {STEPS[1][2], STEPS[2][2]} == {"scene 1 done", "scene 3 done"}
    assert STEPS[-1][2] == "done"
