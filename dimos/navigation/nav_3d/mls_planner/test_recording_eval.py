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

"""Recorded-dataset false-negative regression test for the MLS planner (issue #2996).

The robot traversed the whole recording in one continuous run, so every pair of
points on its odometry trajectory is connected by a real feasible path -- free
ground truth, no manual labelling. We replay the recording through the online
pipeline (RayTraceMap -> MLSPlanner.update_region) to build the accumulated map,
then ask plan(A, B) for a floor x floor matrix of trajectory pairs. plan()->None
on a feasible pair is a FALSE NEGATIVE.

This is a REGRESSION BASELINE, not a pass/fail gate: the current planner produces
many false negatives on this hard dataset (sparse 45-degree-lidar stair points +
noise shatter the traversability graph). The objective is to drive that count
DOWN -- typically by improving the ray-tracing mapper; this test fails if it
regresses UP past the recorded baseline. Lower BASELINE_FALSE_NEG as it improves.

Marked self_hosted: needs the native extensions + the LFS dataset and does a full
replay (~5 min), so it is deselected from the default CI run.
"""
import os
from pathlib import Path

import pytest

pytest.importorskip("dimos_mls_planner")
pytest.importorskip("dimos_voxel_ray_tracing")

from dimos.navigation.nav_3d.mls_planner.recording_eval import (  # noqa: E402
    MapperConfig,
    build_map,
    evaluate,
    print_scorecard,
)
from dimos.utils.data import get_data  # noqa: E402

pytestmark = [pytest.mark.self_hosted]

DATASET = "mid360_athens_stairs.db"
PER_FLOOR = 4

# Baseline measured on the incremental (production) map with the default
# MapperConfig: 112/132 false negatives across 3 auto-detected floors.
# Deterministic (Dijkstra planner + deterministic sampling). This test guards
# against regression; lower it as the mapper/planner improves.
BASELINE_FALSE_NEG = 112
MIN_FLOORS = 2


@pytest.fixture(scope="module")
def scorecard():
    # DIMOS_RECORDING_DB lets a dev point at a local .db (e.g. a host without
    # git-lfs); otherwise fetch the LFS dataset the canonical way.
    override = os.environ.get("DIMOS_RECORDING_DB")
    if override and Path(override).exists():
        path = Path(override)
    else:
        try:
            path = get_data(DATASET)
        except Exception as exc:  # e.g. git-lfs missing on this host
            pytest.skip(f"dataset unavailable ({DATASET}): {exc}")
        if not path or not path.exists():
            pytest.skip(f"dataset not found: {DATASET}")
    build = build_map(str(path), MapperConfig(), progress=False)
    sc = evaluate(build, per_floor=PER_FLOOR)
    print_scorecard(sc)  # visible with -s; this is the metric issue #2996 tracks
    return sc


def test_eval_produces_multifloor_scorecard(scorecard):
    """Smoke + generalization: floors are auto-detected and pairs are tested."""
    assert len(scorecard.floor_levels) >= MIN_FLOORS, scorecard.floor_levels
    assert scorecard.total > 0


def test_false_negatives_do_not_regress(scorecard):
    """The headline metric: feasible pairs the planner fails to route."""
    assert scorecard.false_neg <= BASELINE_FALSE_NEG, (
        f"false negatives {scorecard.false_neg}/{scorecard.total} regressed past "
        f"baseline {BASELINE_FALSE_NEG} -- routing of feasible pairs got WORSE"
    )


def test_every_false_negative_is_a_real_disconnect(scorecard):
    """Structural invariant: the union-find disconnect diagnostic can only explain
    false negatives it actually found, never more -- so the eval measures genuine
    'separate connected surface components' failures, not artifacts."""
    for (i, j), (n_pairs, n_fn, n_disc, _unsafe) in scorecard.cells.items():
        assert n_disc <= n_fn, f"cell {i}->{j}: {n_disc} disconnects > {n_fn} false negatives"


def test_same_floor_no_harder_than_cross_floor(scorecard):
    """Dataset-agnostic sanity: same-floor routes should be no harder to plan than
    cross-floor ones. If this inverts, the eval or map is broken."""
    diag = [c for (i, j), c in scorecard.cells.items() if i == j and c[0] > 0]
    off = [c for (i, j), c in scorecard.cells.items() if i != j and c[0] > 0]
    if not diag or not off:
        pytest.skip("need both same-floor and cross-floor pairs")
    diag_rate = sum(c[1] for c in diag) / sum(c[0] for c in diag)
    off_rate = sum(c[1] for c in off) / sum(c[0] for c in off)
    assert diag_rate <= off_rate + 1e-9, f"same-floor FN {diag_rate:.2f} > cross-floor {off_rate:.2f}"


def test_produced_paths_reported_for_safety(scorecard):
    """The safety guardrail (box-slide vs voxels) is reported, not gated: it is a
    coarse 2.5D check that over-flags on stair steps, so it is informational until
    calibrated. Collision checking proper is a separate downstream module."""
    assert scorecard.unsafe >= 0  # executed; value surfaced in the printed scorecard
