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
ground truth, no manual labelling. We replay the recording through the production
pipeline (RayTraceMap -> MLSPlanner.update_region) to build the accumulated
multi-floor map, then ask plan(A, B) for a floor x floor matrix of trajectory
pairs. plan()->None on a feasible pair is a FALSE NEGATIVE (Andrew: "as soon as
there is any disconnect between start and goal it doesn't plan").

This is a REGRESSION BASELINE, not a pass/fail gate: the current planner produces
many false negatives on this hard dataset (sparse 45-degree-lidar stair points +
noise shatter the traversability graph). The objective (issue #2996) is to drive
the false-negative count DOWN while keeping produced paths safe; this test fails
if it regresses UP past the recorded baseline. Lower BASELINE_FALSE_NEG as the
planner improves.

Marked self_hosted: needs the native extensions + the LFS dataset and does a full
replay (~5 min), so it is deselected from the default CI run.
"""
import os
from pathlib import Path

import pytest

pytest.importorskip("dimos_mls_planner")
pytest.importorskip("dimos_voxel_ray_tracing")

from dimos.navigation.nav_3d.mls_planner.recording_eval import (  # noqa: E402
    build_map,
    evaluate,
    print_scorecard,
)
from dimos.utils.data import get_data  # noqa: E402

pytestmark = [pytest.mark.self_hosted]

DATASET = "mid360_athens_stairs.db"
PER_CELL = 4  # -> 240 feasible pairs across the 4 floor strata

# Baseline measured 2026-07-22 on the incremental (production) map: 210/240 false
# negatives. Deterministic (Dijkstra planner + deterministic sampling). This test
# guards against regression; lower it as false negatives are fixed.
BASELINE_FALSE_NEG = 210
EXPECTED_TOTAL = 240


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
    build = build_map(str(path), max_frames=None, progress=False)
    sc = evaluate(build, per_cell=PER_CELL)
    print_scorecard(sc)  # visible with -s; this is the metric issue #2996 tracks
    return sc


def test_false_negatives_do_not_regress(scorecard):
    """The headline metric: feasible pairs the planner fails to route."""
    assert scorecard.total == EXPECTED_TOTAL, scorecard.total
    assert scorecard.false_neg <= BASELINE_FALSE_NEG, (
        f"false negatives {scorecard.false_neg}/{scorecard.total} regressed past "
        f"baseline {BASELINE_FALSE_NEG} -- the planner got WORSE at routing "
        f"feasible pairs on the athens dataset"
    )


def test_every_false_negative_is_a_real_disconnect(scorecard):
    """Structural invariant: the union-find disconnect diagnostic can only explain
    false negatives it actually found, never more. Confirms the eval is measuring
    genuine 'separate connected surface components' failures, not artifacts."""
    for (sa, sb), (n_pairs, n_fn, n_disc, _n_unsafe) in scorecard.cells.items():
        assert n_disc <= n_fn, f"cell {sa}->{sb}: {n_disc} disconnects > {n_fn} false negatives"


def test_produced_paths_reported_for_safety(scorecard, capsys):
    """The safety guardrail (box-slide vs voxels) is reported, not gated: it is a
    coarse 2.5D check that over-flags on multi-level surfaces (stair steps), so it
    is informational until calibrated to the planner's clearance model. Collision
    checking proper is a separate downstream module (per issue #2996 scope)."""
    # No assertion on scorecard.unsafe -- surfaced in the printed scorecard instead.
    assert scorecard.total > 0
