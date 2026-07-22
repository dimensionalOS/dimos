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

"""Tests for eval_module.py's PURE analysis -- the code the offline held-out driver
and the in-process RelocEval Module SHARE, exercised with
no LCM bus, no replay, no DB. Two properties:

  * umeyama_alignment recovers a KNOWN rigid map_B_T_map_A (and scale) from
    constructed corresponding point sets to ~1e-9, always a proper rotation.
  * compute_stats + format_report + stats_to_dict aggregate a small synthetic Fix
    list correctly: per-source won counts, %traj (over covered odom), medians, and
    the success gate tally.

Deterministic: numpy default_rng + fixed scipy Rotations; no wall-clock, no random.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.mapping.relocalization.eval_module import (
    Fix,
    aligned_err_t_m,
    compute_stats,
    format_report,
    resolve_heldout_alignment,
    stats_to_dict,
    umeyama_alignment,
)


def _rt(rot: np.ndarray, t: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = t
    return m


# --------------------------------------------------------------------------- #
# umeyama_alignment: recover a known rigid transform (and scale) exactly
# --------------------------------------------------------------------------- #
def test_umeyama_recovers_known_rigid_transform() -> None:
    """dst = R @ src + t for a known R, t -> umeyama returns exactly that (R, t),
    scale 1.0, and det(R) == +1 (a PROPER rotation, no reflection)."""
    rng = np.random.default_rng(0)
    src = rng.uniform(-5.0, 5.0, size=(8, 3))
    rot = Rotation.from_euler("xyz", [10.0, -25.0, 40.0], degrees=True).as_matrix()
    trans = np.array([1.5, -2.0, 3.25])
    dst = (rot @ src.T).T + trans

    r_out, t_out, scale = umeyama_alignment(src, dst)  # with_scale defaults False
    np.testing.assert_allclose(r_out, rot, atol=1e-9)
    np.testing.assert_allclose(t_out, trans, atol=1e-9)
    assert scale == 1.0
    assert abs(np.linalg.det(r_out) - 1.0) < 1e-9


def test_umeyama_recovers_scale_when_requested() -> None:
    """dst = 2.0 * R @ src + t with with_scale=True -> scale ~2.0 and R still exact.
    Scale is the health signal that flags wrong correspondences."""
    rng = np.random.default_rng(1)
    src = rng.uniform(-3.0, 3.0, size=(10, 3))
    rot = Rotation.from_euler("z", 33.0, degrees=True).as_matrix()
    trans = np.array([0.5, 0.5, -1.0])
    dst = 2.0 * (rot @ src.T).T + trans

    r_out, t_out, scale = umeyama_alignment(src, dst, with_scale=True)
    assert abs(scale - 2.0) < 1e-9
    np.testing.assert_allclose(r_out, rot, atol=1e-9)
    np.testing.assert_allclose(t_out, trans, atol=1e-9)


def test_umeyama_too_few_points_raises() -> None:
    """A rigid frame needs >= 3 correspondences; 2 must raise ValueError, not
    silently return a degenerate transform."""
    two = np.zeros((2, 3))
    try:
        umeyama_alignment(two, two)
    except ValueError as e:
        assert "3 correspondences" in str(e)
    else:
        raise AssertionError("expected ValueError for < 3 correspondences")


def test_umeyama_shape_mismatch_raises() -> None:
    try:
        umeyama_alignment(np.zeros((4, 3)), np.zeros((5, 3)))
    except ValueError as e:
        assert "matching (N,3)" in str(e)
    else:
        raise AssertionError("expected ValueError for mismatched shapes")


# --------------------------------------------------------------------------- #
# resolve_heldout_alignment: markers in both frames -> map_B_T_map_A, else `-`
# --------------------------------------------------------------------------- #
def test_resolve_alignment_from_shared_markers() -> None:
    """>= 3 shared tags surveyed in both frames -> the recovered map_B_T_map_A maps
    each map_A marker onto its map_B twin."""
    rng = np.random.default_rng(2)
    rot = Rotation.from_euler("xyz", [5.0, 10.0, -8.0], degrees=True).as_matrix()
    trans = np.array([2.0, -1.0, 0.5])
    markers_a = {i: tuple(rng.uniform(-4.0, 4.0, 3)) for i in (1, 2, 3, 4)}
    markers_b = {i: tuple(rot @ np.array(p) + trans) for i, p in markers_a.items()}

    mat, reason = resolve_heldout_alignment(markers_a, markers_b)
    assert mat is not None
    np.testing.assert_allclose(mat, _rt(rot, trans), atol=1e-9)
    assert "Umeyama over 4 shared tags" in reason


def test_resolve_alignment_no_run_b_is_held_out() -> None:
    mat, reason = resolve_heldout_alignment({1: (0, 0, 0)}, None)
    assert mat is None and "no run-B marker survey" in reason


def test_resolve_alignment_too_few_shared_is_held_out() -> None:
    """Only 2 shared ids -> None with a reason naming the shortfall, never a fake
    number."""
    mat, reason = resolve_heldout_alignment(
        {1: (0, 0, 0), 2: (1, 0, 0)}, {1: (0, 0, 0), 2: (1, 0, 0)}
    )
    assert mat is None and "2 tag id(s) shared" in reason


def test_aligned_err_t_m_zero_when_fix_matches_truth() -> None:
    """A fix that IS B's truth (carried through the alignment) gives ~0 m error."""
    rot = Rotation.from_euler("z", 12.0, degrees=True).as_matrix()
    trans = np.array([1.0, 2.0, 0.0])
    map_b_t_map_a = _rt(rot, trans)
    est_map_a = _rt(np.eye(3), np.array([3.0, -1.0, 0.0]))  # map_A_T_world_B
    world_map_fix = np.linalg.inv(est_map_a)  # world_T_map_A
    truth_mat = map_b_t_map_a @ est_map_a  # map_B_T_world_B, exactly consistent
    assert aligned_err_t_m(world_map_fix, truth_mat, map_b_t_map_a) < 1e-9


# --------------------------------------------------------------------------- #
# compute_stats + rendering: per-source won / %traj / medians / success tally
# --------------------------------------------------------------------------- #
def _fixture_fixes() -> list[Fix]:
    """ransac wins twice (one success, one fail), fiducial once (success); the
    err/fitness/success values are hand-picked so the medians are checkable."""
    return [
        Fix(ts=1.0, world_map_fix=None, source="ransac", fitness=0.9, err_t_m=0.2, success=True),
        Fix(ts=2.0, world_map_fix=None, source="fiducial", fitness=0.8, err_t_m=0.5, success=True),
        Fix(ts=3.0, world_map_fix=None, source="ransac", fitness=0.7, err_t_m=1.5, success=False),
    ]


def test_compute_stats_per_source_aggregation() -> None:
    """odom samples at 0.5/1.5/2.5/3.5 s -> sample 0.5 is uncovered (before the first
    fix), the rest map to ransac/fiducial/ransac. Assert won counts, %traj over the
    3 covered samples, medians, coverage, first-fix, and the success tally."""
    fixes = _fixture_fixes()
    odom_ts = np.array([0.5, 1.5, 2.5, 3.5])
    census = [{"ransac": 2, "fiducial": 1}, {"ransac": 1}]  # fiducial in 1 of 2 cycles

    stats = compute_stats(
        fixes, odom_ts, census, n_rejects=4, mode="held_out", held_out_note="note"
    )
    by = {r.source: r for r in stats.rows}

    assert by["ransac"].won == 2 and by["fiducial"].won == 1 and by["last_pose"].won == 0
    # covered = 3 samples; ransac active for 2 of them, fiducial for 1
    assert abs(by["ransac"].pct_traj - 200.0 / 3.0) < 1e-9
    assert abs(by["fiducial"].pct_traj - 100.0 / 3.0) < 1e-9
    assert by["last_pose"].pct_traj == 0.0
    # medians over each source's own fixes
    assert by["ransac"].med_err_m == 0.85 and by["ransac"].med_fit == 0.8
    assert by["fiducial"].med_err_m == 0.5
    # success gate tally: ransac 1 of 2 judged, fiducial 1 of 1, last_pose untouched
    assert by["ransac"].n_success == 1 and by["ransac"].n_judged == 2
    assert by["fiducial"].n_success == 1 and by["fiducial"].n_judged == 1
    assert by["last_pose"].n_success is None and by["last_pose"].n_judged is None

    assert stats.accepts == 3 and stats.rejects == 4
    assert abs(stats.coverage_pct - 75.0) < 1e-9  # 3 of 4 odom samples covered
    assert abs(stats.first_fix_s - 0.5) < 1e-9  # 1.0 - 0.5
    assert stats.fiducial_won == 1
    assert stats.fiducial_proposed_cycles == 1 and stats.census_cycles == 2


def test_compute_stats_no_odom_zero_coverage() -> None:
    """Empty odom -> 0% coverage, 0% per-source traj, first-fix None; no divide-by-
    zero."""
    stats = compute_stats(
        _fixture_fixes(), np.empty((0,)), [], n_rejects=None,
        mode="held_out", held_out_note="n",
    )
    assert stats.coverage_pct == 0.0
    assert stats.first_fix_s is None
    assert all(r.pct_traj == 0.0 for r in stats.rows)
    assert stats.census_cycles is None and stats.rejects is None


def test_stats_to_dict_rounding_and_keys() -> None:
    """stats_to_dict rounds pct_traj to 2 and med_err to 4 dp and preserves the
    per-source success counts."""
    stats = compute_stats(
        _fixture_fixes(), np.array([0.5, 1.5, 2.5, 3.5]), [], n_rejects=0,
        mode="held_out", held_out_note="n",
    )
    d = stats_to_dict(stats, title="t")
    ransac = next(r for r in d["per_source"] if r["source"] == "ransac")
    assert ransac["won"] == 2
    assert ransac["pct_traj"] == 66.67  # round(200/3, 2)
    assert ransac["med_err_m"] == 0.85
    assert ransac["n_success"] == 1 and ransac["n_judged"] == 2
    assert d["coverage_pct"] == 75.0 and d["first_fix_s"] == 0.5


def test_format_report_held_out_shows_columns_live_hides_them() -> None:
    """held_out mode renders med_err + success columns; live mode drops both (no
    truth in-process). The won/%traj columns appear in both."""
    fixes = _fixture_fixes()
    odom_ts = np.array([0.5, 1.5, 2.5, 3.5])

    held = compute_stats(fixes, odom_ts, [], None, mode="held_out", held_out_note="hn")
    held_txt = format_report(held, title="demo")
    assert "med_err" in held_txt and "success" in held_txt
    assert "1/2" in held_txt  # ransac success tally rendered
    assert "0.850m" in held_txt  # ransac med_err
    assert "66.7%" in held_txt  # ransac %traj, 1 dp in the table

    live = compute_stats(fixes, odom_ts, [], None, mode="live", held_out_note="ln")
    live_txt = format_report(live, title="demo")
    assert "med_err" not in live_txt and "success" not in live_txt
    assert "%traj" in live_txt and "ransac" in live_txt
