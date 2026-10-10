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

from pathlib import Path

import numpy as np
import pytest

from dimos.navigation.bench.ground_truth import GO2, GroundTruth
from dimos.navigation.bench.suite import MINED, NARROW_DOOR_M, FreezeConfig, Manifest, freeze
from dimos.simulation.scenes.procedural import office

SMALL = FreezeConfig(seeds=(1, 3), stressor_samples=60, samples_per_scene=25, cases_per_bin=1)


def test_ground_truth_finds_floor_walls_routes_and_what_the_lidar_sees() -> None:
    gt = GroundTruth(office(1))
    z0 = gt.scene.params["z0"]
    assert gt.height[gt.index((1.0, 1.0))] == pytest.approx(z0, abs=1e-6)
    assert np.isnan(gt.height[gt.index((-0.05, 3.0))])
    assert gt.stands((1.0, 1.0)) and not gt.stands((0.1, 3.0))
    reach = np.where(gt.walkable, np.add.outer(*map(np.arange, gt.shape)), -1)
    goal = gt.center(*np.unravel_index(int(np.argmax(reach)), gt.shape))
    route = gt.route(gt.scene.start, goal, centered=True)
    assert route is not None and all(gt.stands(p) for p in route.points)
    assert np.allclose(route.points[-1, :2], goal[:2])
    difficulty = gt.difficulty(route)
    assert difficulty.min_clearance >= GO2.radius and difficulty.doors >= 1
    cloud = gt.premap_cloud(route.points)
    lo, hi = gt.scene.bounds()
    assert len(cloud) > 20_000 and np.all(cloud >= lo - gt.cell) and np.all(cloud <= hi + gt.cell)
    assert np.mean(np.abs(cloud[:, 2] - z0) < 0.05) > 0.1 and np.any(cloud[:, 2] > z0 + 2.5)


def test_freeze_bins_routable_cases_that_round_trip(tmp_path: Path) -> None:
    steps: list[tuple[int, int, str]] = []
    manifest = freeze(SMALL, lambda *step: steps.append(step))
    assert len({c.id for c in manifest.cases}) == len(manifest.cases) > 0
    for case in manifest.cases:
        assert case.route_length >= SMALL.min_route_m
        assert case.scene().digest() == case.scene_digest
    assert {"narrow_door", "behind_clutter", MINED} <= {c.tag for c in manifest.cases}
    narrow = next(c for c in manifest.cases if c.tag == "narrow_door")
    assert narrow.params == {"door_width": NARROW_DOOR_M} and narrow.difficulty.doors >= 1
    assert [done for done, _, _ in steps] == [0, 1, 2, 3] and steps[-1][2] == "done"
    manifest.save(tmp_path / "suite.json")
    loaded = Manifest.load(tmp_path / "suite.json")
    assert loaded == manifest
    loaded.check_drift()
