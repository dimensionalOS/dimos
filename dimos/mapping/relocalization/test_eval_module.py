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
no LCM bus, no replay, no DB. Four properties:

  * umeyama_alignment recovers a KNOWN rigid map_B_T_map_A (and scale) from
    constructed corresponding point sets to ~1e-9, always a proper rotation.
  * compute_stats + format_report + stats_to_dict aggregate a small synthetic Fix
    list correctly: per-source won counts, %traj (over covered odom), medians, and
    the success gate tally.
  * the accept key and the log join stay honest: a yaw-only fix is its own accept, a
    health line labels exactly one fix, and degenerate shared tags hold med_err out
    with a reason instead of reporting an arbitrary Umeyama transform.
  * accepted_points_in_map (the live rerun overlay) puts one point per accepted fix
    at the robot's position IN THE MAP FRAME, 1:1 with the deduped accepts, coloured
    by the winning prior -- with `unknown` (a join failure) visibly its own colour.
  * the log parsers read what the PIPELINE ACTUALLY EMITS. Those tests never write a
    log line by hand -- they run the real module.py / priors.py call sites through
    the real dimos logger and parse the bytes it rendered, in BOTH renderings
    (console + main.jsonl). A hand-written fixture is exactly how the parsers rotted
    into silently returning [] once the emitters moved to structlog kwargs.

Deterministic: numpy default_rng + fixed scipy Rotations; no wall-clock, no random.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field
import io
import json
import logging
from pathlib import Path
import re
import time
from types import ModuleType
from typing import Any

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.constants import DIMOS_PROJECT_ROOT
import dimos.mapping.relocalization.eval_module as eval_mod
from dimos.mapping.relocalization.eval_module import (
    SOURCE_COLORS,
    SOURCES,
    EvalConfig,
    Fix,
    HealthLine,
    RelocEval,
    accepted_points_in_map,
    aligned_err_t_m,
    compute_correction_stats,
    compute_stats,
    correction_at_robot,
    corrections_to_dict,
    dedup_tf_fixes,
    format_report,
    label_fixes_from_log,
    parse_census,
    parse_health_lines,
    parse_reject_lines,
    parse_run_log,
    resolve_heldout_alignment,
    stats_to_dict,
    umeyama_alignment,
    write_report,
)
import dimos.mapping.relocalization.module as module_mod
from dimos.mapping.relocalization.module import Config, RelocalizationModule
import dimos.mapping.relocalization.priors as priors_mod
from dimos.mapping.relocalization.priors import (
    Candidate,
    FiducialPriorConfig,
    LastPosePrior,
    LastPosePriorConfig,
    RansacPrior,
    RansacPriorConfig,
    relocalize_with_priors,
)
from dimos.mapping.relocalization.relocalize import JudgeReport


def _rt(rot: np.ndarray, t: np.ndarray) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = t
    return m


def _parked_odom(ts: list[float]) -> np.ndarray:
    """(N,4) [ts,x,y,z] odom samples, all at the world origin -- for the tests that
    exercise only the timestamp-driven stats (coverage, %traj, first-fix)."""
    return np.array([[t, 0.0, 0.0, 0.0] for t in ts], dtype=float)


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
    with pytest.raises(ValueError, match="3 correspondences"):
        umeyama_alignment(two, two)


def test_umeyama_shape_mismatch_raises() -> None:
    with pytest.raises(ValueError, match=r"matching \(N,3\)"):
        umeyama_alignment(np.zeros((4, 3)), np.zeros((5, 3)))


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


def test_resolve_alignment_collinear_tags_held_out() -> None:
    """>=3 shared tags that lie on a LINE leave the rotation about that line
    unconstrained -- Umeyama's SVD would return an arbitrary transform. Guard must
    return None with a collinearity reason, never a fabricated med_err."""
    # four tags on the x-axis in map_A; map_B is any rigid image (still a line)
    rot = Rotation.from_euler("xyz", [15.0, -20.0, 35.0], degrees=True).as_matrix()
    trans = np.array([1.0, -2.0, 0.5])
    markers_a = {i: (float(i), 0.0, 0.0) for i in (1, 2, 3, 4)}
    markers_b = {i: tuple(rot @ np.array(p) + trans) for i, p in markers_a.items()}
    mat, reason = resolve_heldout_alignment(markers_a, markers_b)
    assert mat is None and "collinear" in reason
    # a well-spread (non-collinear) set of the SAME count still resolves
    markers_a2 = {1: (0.0, 0.0, 0.0), 2: (1.0, 0.0, 0.0), 3: (0.0, 1.0, 0.0), 4: (1.0, 1.0, 0.4)}
    markers_b2 = {i: tuple(rot @ np.array(p) + trans) for i, p in markers_a2.items()}
    mat2, _ = resolve_heldout_alignment(markers_a2, markers_b2)
    assert mat2 is not None
    np.testing.assert_allclose(mat2, _rt(rot, trans), atol=1e-9)


def test_resolve_alignment_degenerate_run_b_survey_held_out() -> None:
    """map_A well spread but the run-B survey collapsed onto a LINE: cov goes
    rank-deficient and Umeyama returns an arbitrary rotation. Both sides are gated,
    so this is held out naming map_B."""
    markers_a = {1: (0.0, 0.0, 0.0), 2: (1.0, 0.0, 0.0), 3: (0.0, 1.0, 0.0), 4: (1.0, 1.0, 0.4)}
    markers_b = {1: (0.0, 0.0, 0.0), 2: (1.0, 0.0, 0.0), 3: (2.0, 0.0, 0.0), 4: (3.0, 0.0, 0.0)}
    mat, reason = resolve_heldout_alignment(markers_a, markers_b)
    assert mat is None
    assert "collinear in map_B" in reason and "med_err held out" in reason


def test_resolve_alignment_coincident_tags_held_out() -> None:
    """Shared tags that all sit at one point have no extent at all -- held out with
    the coincide reason, never a fabricated number."""
    markers_a = {i: (1.0, 2.0, 3.0) for i in (1, 2, 3)}
    markers_b = {i: (0.0, 0.0, 0.0) for i in (1, 2, 3)}
    mat, reason = resolve_heldout_alignment(markers_a, markers_b)
    assert mat is None and "coincide in map_A" in reason


# --------------------------------------------------------------------------- #
# dedup_tf_fixes: rotation is part of the accept key (yaw-only fix not dropped)
# --------------------------------------------------------------------------- #
def test_dedup_keeps_yaw_only_fix() -> None:
    """A second accept at the SAME origin but a corrected yaw (symmetric-scene flip)
    is a distinct fix -- translation-only dedup would silently drop it."""
    origin = np.array([2.0, 3.0, 0.0])
    pose_a = _rt(Rotation.from_euler("z", 0.0, degrees=True).as_matrix(), origin)
    pose_b = _rt(Rotation.from_euler("z", 180.0, degrees=True).as_matrix(), origin)
    # republished twice each (every PUBLISH_INTERVAL_S) before the flip lands
    samples = [(0.0, pose_a), (0.1, pose_a), (0.2, pose_b), (0.3, pose_b)]
    fixes = dedup_tf_fixes(samples)
    assert len(fixes) == 2
    np.testing.assert_allclose(fixes[0][1], pose_a)
    np.testing.assert_allclose(fixes[1][1], pose_b)


def test_dedup_collapses_repeats() -> None:
    """Identical republished poses collapse to one accept (rotation guard does not
    over-split on the repeats)."""
    pose = _rt(np.eye(3), np.array([1.0, 0.0, 0.0]))
    assert len(dedup_tf_fixes([(0.0, pose), (0.1, pose), (0.2, pose)])) == 1


# --------------------------------------------------------------------------- #
# label_fixes_from_log: nearby accepts from different priors keep their own source
# --------------------------------------------------------------------------- #
def test_label_nearby_fixes_keep_distinct_sources() -> None:
    """Two accepts 1.5 cm apart from different priors: each TF fix must get its OWN
    health line (source+fitness), not the nearest-overwritten one. Consume-once
    prevents both fixes collapsing onto a single health entry."""
    t_a = np.array([0.000, 0.0, 0.0])
    t_b = np.array([0.015, 0.0, 0.0])  # 1.5 cm away, inside the 2 cm join tolerance
    fix_a = _rt(np.eye(3), t_a)
    fix_b = _rt(np.eye(3), t_b)
    health = [
        HealthLine(source="ransac", fitness=0.9, published_t_m=tuple(t_a)),
        HealthLine(source="fiducial", fitness=0.7, published_t_m=tuple(t_b)),
    ]
    fixes = label_fixes_from_log([(0.0, fix_a), (0.1, fix_b)], health)
    assert [f.source for f in fixes] == ["ransac", "fiducial"]
    assert [f.fitness for f in fixes] == [0.9, 0.7]


def test_label_second_nearby_fix_never_reuses_a_claimed_line() -> None:
    """Two accepts 1.5 cm apart with only ONE health line near them: the line belongs
    to the fix that owns it, and the other stays ``unknown``. Reusing it would stamp
    a second accept with a source and fitness that are not its own."""
    health = [
        HealthLine(source="ransac", fitness=0.9, published_t_m=(0.0, 0.0, 0.0)),
        HealthLine(source="fiducial", fitness=0.7, published_t_m=(5.0, 0.0, 0.0)),  # far away
    ]
    tf_fixes = [
        (0.0, _rt(np.eye(3), np.array([0.000, 0.0, 0.0]))),
        (0.1, _rt(np.eye(3), np.array([0.015, 0.0, 0.0]))),
    ]
    fixes = label_fixes_from_log(tf_fixes, health)
    assert [f.source for f in fixes] == ["ransac", "unknown"]
    assert fixes[0].fitness == 0.9 and np.isnan(fixes[1].fitness)


def test_label_equal_distance_tie_is_stable() -> None:
    """A fix equidistant from two health lines takes the EARLIER (log order); a
    later equally-close line never overwrites it."""
    fix = _rt(np.eye(3), np.array([0.010, 0.0, 0.0]))  # 1 cm from each
    health = [
        HealthLine(source="ransac", fitness=0.9, published_t_m=(0.0, 0.0, 0.0)),
        HealthLine(source="fiducial", fitness=0.7, published_t_m=(0.020, 0.0, 0.0)),
    ]
    fixes = label_fixes_from_log([(0.0, fix)], health)
    assert fixes[0].source == "ransac" and fixes[0].fitness == 0.9


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
    odom = _parked_odom([0.5, 1.5, 2.5, 3.5])
    census = [{"ransac": 2, "fiducial": 1}, {"ransac": 1}]  # fiducial in 1 of 2 cycles

    stats = compute_stats(
        fixes, odom, census, n_rejects=4, mode="held_out", held_out_note="note"
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
        _fixture_fixes(), _parked_odom([0.5, 1.5, 2.5, 3.5]), [], n_rejects=0,
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
    """held_out mode renders the false + med_err columns; live mode drops both (no
    truth in-process). prop/acc/rej and %traj/med_fit appear in both."""
    fixes = _fixture_fixes()
    odom = _parked_odom([0.5, 1.5, 2.5, 3.5])

    held = compute_stats(fixes, odom, [], None, mode="held_out", held_out_note="hn")
    held_txt = format_report(held, title="demo")
    assert re.search(r"source +prop +acc +rej +false +%traj +med_err +med_fit", held_txt), held_txt
    assert "0.850m" in held_txt  # ransac med_err
    assert "66.7%" in held_txt  # ransac %traj, 1 dp in the table

    live = compute_stats(fixes, odom, [], None, mode="live", held_out_note="ln")
    live_txt = format_report(live, title="demo")
    assert "med_err" not in live_txt and "false" not in live_txt
    assert re.search(r"source +prop +acc +rej +%traj +med_fit", live_txt), live_txt
    assert "ransac" in live_txt


def _activity_fixes() -> list[Fix]:
    """Three held-out accepts: ransac wins twice (one beyond SUCCESS_T_M -> a FALSE
    accept), fiducial once (within gate). Hand-picked so the false tally is checkable."""
    return [
        Fix(ts=1.0, world_map_fix=None, source="ransac", fitness=0.9, err_t_m=0.2, success=True),
        Fix(ts=2.0, world_map_fix=None, source="ransac", fitness=0.7, err_t_m=1.5, success=False),
        Fix(ts=3.0, world_map_fix=None, source="fiducial", fitness=0.8, err_t_m=0.4, success=True),
    ]


def test_per_source_activity_proposed_accepted_rejected_false() -> None:
    """The full per-source stats print on constructed inputs. Three census cycles
    (ransac in all 3, fiducial in 2), three accepts (ransac x2, fiducial x1), three
    rejects (ransac, fiducial, and one source-less -> the `unknown` bucket a verbose
    --eval reject falls into), and one held-out FALSE accept (ransac, err 1.5 m >
    SUCCESS_T_M). Every per-source count and the TOTAL row are exact by hand."""
    census = [{"ransac": 3, "fiducial": 1}, {"ransac": 2}, {"ransac": 1, "fiducial": 2}]
    accept_sources = ["ransac", "ransac", "fiducial"]  # the `relocalize accepted` sources
    reject_sources = ["ransac", "fiducial", "unknown"]  # last is a source-less reject
    fixes = _activity_fixes()

    stats = compute_stats(
        fixes, _parked_odom([0.5, 1.5, 2.5, 3.5]), census, n_rejects=3,
        mode="held_out", held_out_note="n",
        accept_sources=accept_sources, reject_sources=reject_sources,
    )
    by = {r.source: r for r in stats.rows}

    # PROPOSED = cycles the source offered >=1 candidate (per-cycle presence).
    assert by["ransac"].proposed == 3 and by["fiducial"].proposed == 2
    assert by["last_pose"].proposed == 0
    # ACCEPTED = `relocalize accepted` log lines for that source (from accept_sources).
    assert by["ransac"].accepted == 2 and by["fiducial"].accepted == 1
    assert by["last_pose"].accepted == 0
    # REJECTED = `relocalize rejected` lines for that source; a source-less reject
    # becomes an `unknown` row rather than being dropped or fabricated onto a prior.
    assert by["ransac"].rejected == 1 and by["fiducial"].rejected == 1
    assert by["last_pose"].rejected == 0
    assert "unknown" in by and by["unknown"].rejected == 1
    assert by["unknown"].accepted == 0 and by["unknown"].proposed == 0
    # FALSE = accepts beyond SUCCESS_T_M; ransac has one (err 1.5 m), fiducial none.
    assert by["ransac"].n_false == 1 and by["fiducial"].n_false == 0
    assert by["last_pose"].n_false is None  # no judged fix -> held out, never a fake 0

    # The TOTAL row sums the count columns: prop 3+2=5, acc 2+1=3, rej 1+1+1=3, false 1.
    txt = format_report(stats, title="demo")
    assert re.search(r"TOTAL +5 +3 +3 +1", txt), txt
    assert re.search(r"unknown +0 +0 +1", txt), txt  # prop acc rej for the unknown row

    # ...and every count survives into the machine-readable json (won/success kept too).
    d = stats_to_dict(stats, title="demo")
    ransac = next(r for r in d["per_source"] if r["source"] == "ransac")
    assert (ransac["proposed"], ransac["accepted"], ransac["rejected"], ransac["n_false"]) == (
        3, 2, 1, 1,
    )
    assert ransac["won"] == 2 and ransac["n_success"] == 1 and ransac["n_judged"] == 2


def test_per_source_rejected_and_false_held_out_when_not_derivable() -> None:
    """No reject sources parsed -> REJECTED is `-` for every row and the TOTAL, never a
    fabricated 0. No held-out truth -> FALSE is `-`. Both degrade honestly rather than
    reading as 'zero rejects / zero false'."""
    fixes = _fixture_fixes()  # carry success, but pass reject_sources=None
    stats = compute_stats(
        fixes, _parked_odom([0.5, 1.5, 2.5, 3.5]), [{"ransac": 1}], n_rejects=2,
        mode="held_out", held_out_note="n",  # reject_sources omitted -> None
    )
    by = {r.source: r for r in stats.rows}
    assert by["ransac"].rejected is None and by["fiducial"].rejected is None
    txt = format_report(stats, title="demo")
    # the rej column renders `-` (held out), while the overall line still counts 2.
    assert re.search(r"ransac +\d+ +\d+ +-", txt), txt
    assert "rejects=2" in txt

    # live-mode has no in-process truth -> the false column is dropped entirely.
    live = compute_stats(
        fixes, _parked_odom([0.5, 1.5, 2.5, 3.5]), [], n_rejects=None,
        mode="live", held_out_note="n",
    )
    assert "false" not in format_report(live, title="demo")


# --------------------------------------------------------------------------- #
# Correction magnitude + correction per metre: how much reloc actually moves the
# estimate. Every number below is hand-computable from the constructed track.
# --------------------------------------------------------------------------- #
_R_YAW_90 = Rotation.from_euler("z", 90.0, degrees=True).as_matrix()


def test_correction_is_measured_at_the_robot_through_the_inverted_fix() -> None:
    """FRAME DIRECTION, pinned to a case where the three candidate answers differ.

    Robot at world (2,0,0). Old fix world_T_map = translate(+1,0,0) -> the robot
    believes it is at map (1,0,0). New fix world_T_map = yaw(+90 deg) -> map_T_world
    = yaw(-90), so the belief jumps to (0,-2,0). The correction is therefore
    ||(0,-2,0) - (1,0,0)|| = sqrt(5).

    The two ways to get it wrong both return a plausible number:
      * applying the fix instead of its inverse -> sqrt(13),
      * differencing the fixes' own translations (the MAP ORIGIN) -> 1.0.
    """
    world_map_old = _rt(np.eye(3), np.array([1.0, 0.0, 0.0]))
    world_map_new = _rt(_R_YAW_90, np.zeros(3))
    p_world = np.array([2.0, 0.0, 0.0])

    magnitude_m, dyaw_deg = correction_at_robot(world_map_old, world_map_new, p_world)
    assert abs(magnitude_m - np.sqrt(5.0)) < 1e-12
    assert abs(dyaw_deg - 90.0) < 1e-9

    inverted = np.linalg.norm(
        (world_map_new @ np.array([*p_world, 1.0]))[:3]
        - (world_map_old @ np.array([*p_world, 1.0]))[:3]
    )
    at_map_origin = np.linalg.norm(world_map_new[:3, 3] - world_map_old[:3, 3])
    assert abs(inverted - np.sqrt(13.0)) < 1e-12  # the un-inverted answer...
    assert at_map_origin == 1.0  # ...and the map-origin answer
    assert abs(magnitude_m - inverted) > 1.0 and abs(magnitude_m - at_map_origin) > 1.0


def test_first_fix_acquisition_is_measured_against_no_correction() -> None:
    """No world->map had been published before the first accept, so the robot had no
    map-frame belief; identity is the "no correction yet" convention. Robot at world
    (3,0,0), fix world_T_map = translate(1,0,0) -> believed map (2,0,0) -> 1.0 m."""
    magnitude_m, dyaw_deg = correction_at_robot(
        None, _rt(np.eye(3), np.array([1.0, 0.0, 0.0])), np.array([3.0, 0.0, 0.0])
    )
    assert magnitude_m == 1.0 and dyaw_deg == 0.0


def _known_track() -> tuple[list[Fix], np.ndarray]:
    """Robot drives +x at 1 m/s, sampled at 1 Hz for 5 s, plus three pure-translation
    fixes whose corrections are exact by hand:

      f0 t=0 world_T_map=(1,0,0)      first fix: robot at world 0 -> map (-1,0,0)  1.0 m
      f1 t=2 world_T_map=(1.6,0,0)    robot at world 2: (1,0,0) -> (0.4,0,0)       0.6 m
      f2 t=5 world_T_map=(1.6,.8,0)   robot at world 5: (3.4,0,0) -> (3.4,-.8,0)   0.8 m
    """
    odom = np.array([[t, t, 0.0, 0.0] for t in range(6)], dtype=float)
    fixes = [
        Fix(ts=0.0, world_map_fix=_rt(np.eye(3), np.array([1.0, 0.0, 0.0])), source="ransac",
            fitness=0.9),
        Fix(ts=2.0, world_map_fix=_rt(np.eye(3), np.array([1.6, 0.0, 0.0])), source="ransac",
            fitness=0.8),
        Fix(ts=5.0, world_map_fix=_rt(np.eye(3), np.array([1.6, 0.8, 0.0])), source="fiducial",
            fitness=0.7),
    ]
    return fixes, odom


def test_correction_magnitude_and_rate_from_a_known_track() -> None:
    """The whole metric on the constructed track: two drift corrections of 0.6 m over
    2 m driven and 0.8 m over 3 m driven, the acquisition fix held out of every
    aggregate, and the per-source split attributing each to the prior that won it."""
    fixes, odom = _known_track()
    cs = compute_correction_stats(fixes, odom)
    assert cs is not None

    assert cs.first is not None and cs.first.magnitude_m == 1.0 and cs.first.source == "ransac"
    assert [round(c.magnitude_m, 12) for c in cs.per_fix] == [0.6, 0.8]
    assert [c.dist_travelled_m for c in cs.per_fix] == [2.0, 3.0]
    assert abs(cs.per_fix[0].rate_m_per_m - 0.3) < 1e-12  # 0.6 m corrected over 2 m
    assert abs(cs.per_fix[1].rate_m_per_m - 0.8 / 3.0) < 1e-12
    assert [c.source for c in cs.per_fix] == ["ransac", "fiducial"]
    assert cs.first not in cs.per_fix  # acquisition never enters the aggregates

    assert abs(cs.med_magnitude_m - 0.7) < 1e-12
    assert abs(cs.p90_magnitude_m - 0.78) < 1e-12  # 0.6 + 0.9 * (0.8 - 0.6)
    assert abs(cs.med_rate_m_per_m - (0.3 + 0.8 / 3.0) / 2.0) < 1e-12
    assert abs(cs.total_correction_m - 1.4) < 1e-12
    assert cs.total_distance_m == 5.0  # first fix -> last fix, integrated over odom
    assert abs(cs.rate_m_per_m - 0.28) < 1e-12  # 1.4 m corrected over 5 m driven

    by = {r.source: r for r in cs.rows}
    assert by["ransac"].n == 1 and abs(by["ransac"].med_magnitude_m - 0.6) < 1e-12
    assert abs(by["ransac"].med_rate_m_per_m - 0.3) < 1e-12
    assert by["fiducial"].n == 1 and abs(by["fiducial"].med_magnitude_m - 0.8) < 1e-12
    assert "last_pose" not in by  # a prior that won nothing is not a row of zeros


def test_correction_distance_integrates_the_path_not_the_displacement() -> None:
    """The robot walks +2 m then back -2 m between two accepts: displacement 0, path
    4 m. LIO drift accumulates over DRIVEN distance, so the denominator is 4 m."""
    odom = np.array(
        [[0.0, 0, 0, 0], [1.0, 1, 0, 0], [2.0, 2, 0, 0], [3.0, 1, 0, 0], [4.0, 0, 0, 0]],
        dtype=float,
    )
    fixes = [
        Fix(ts=0.0, world_map_fix=np.eye(4), source="ransac", fitness=0.9),
        Fix(ts=4.0, world_map_fix=_rt(np.eye(3), np.array([0.0, 0.4, 0.0])), source="ransac",
            fitness=0.9),
    ]
    cs = compute_correction_stats(fixes, odom)
    assert cs is not None
    assert cs.per_fix[0].dist_travelled_m == 4.0
    assert abs(cs.per_fix[0].magnitude_m - 0.4) < 1e-12
    assert abs(cs.per_fix[0].rate_m_per_m - 0.1) < 1e-12


def test_correction_rate_is_omitted_while_the_robot_is_stationary() -> None:
    """Standing still between two accepts: report the magnitude, no rate. The
    division must never reach the report as inf -- json.dumps would write a bare
    `Infinity`, which is not valid JSON and poisons every downstream trend reader."""
    odom = _parked_odom([0.0, 1.0, 2.0, 3.0])  # never moves
    fixes = [
        Fix(ts=0.0, world_map_fix=np.eye(4), source="ransac", fitness=0.9),
        Fix(ts=2.0, world_map_fix=_rt(np.eye(3), np.array([0.5, 0.0, 0.0])), source="ransac",
            fitness=0.9),
    ]
    cs = compute_correction_stats(fixes, odom)
    assert cs is not None
    assert cs.per_fix[0].dist_travelled_m == 0.0
    assert abs(cs.per_fix[0].magnitude_m - 0.5) < 1e-12
    assert cs.per_fix[0].rate_m_per_m is None
    assert cs.med_rate_m_per_m is None and cs.rate_m_per_m is None
    assert "Infinity" not in json.dumps(corrections_to_dict(cs))


def test_correction_unavailable_without_odom_or_fixes() -> None:
    """No odom (or no fix carrying a pose) -> None, not a zero. There is no robot
    position to evaluate the jump at, and 0.0 would read as "reloc corrected
    nothing"."""
    fixes, odom = _known_track()
    assert compute_correction_stats(fixes, np.empty((0, 4))) is None
    assert compute_correction_stats([], odom) is None
    unposed = [Fix(ts=1.0, world_map_fix=None, source="ransac", fitness=0.5)]
    assert compute_correction_stats(unposed, odom) is None


def test_compute_stats_rejects_a_timestamp_only_odom_array() -> None:
    """Correction magnitude is evaluated at the robot, so compute_stats needs the
    POSITIONS. A ts-only (or [ts,x,y]) array raises instead of being indexed as if
    its columns were [ts,x,y,z] -- a silent column mix-up is how a metric becomes
    wrong but plausible."""
    with pytest.raises(ValueError, match=r"odom must be \(N,4\)"):
        compute_stats(
            [], np.array([0.5, 1.5]), [], None, mode="live", held_out_note="n"
        )
    with pytest.raises(ValueError, match=r"odom must be \(N,4\)"):
        compute_stats(
            [], np.zeros((3, 3)), [], None, mode="live", held_out_note="n"
        )


def test_correction_reaches_the_report_and_the_json() -> None:
    """The operator reads the printed block; a trend tracker reads the json. Both
    carry the same numbers, and the printed form stays to three lines."""
    fixes, odom = _known_track()
    stats = compute_stats(fixes, odom, [], None, mode="live", held_out_note="n")
    text = format_report(stats, title="demo")
    assert "correction (at the robot, n=2): med=0.700m p90=0.780m" in text
    assert "total 1.400m over 5.00m driven (0.2800m/m)" in text
    assert "first fix: 1.000m 0.0deg (ransac, acquisition -- excluded above)" in text
    assert "by source: ransac 0.600m 0.3000m/m (n=1)  fiducial 0.800m 0.2667m/m (n=1)" in text

    d = stats_to_dict(stats, title="demo")["correction"]
    assert d["n"] == 2
    assert d["med_magnitude_m"] == 0.7 and d["p90_magnitude_m"] == 0.78
    assert d["total_correction_m"] == 1.4 and d["total_distance_m"] == 5.0
    assert d["rate_m_per_m"] == 0.28
    assert d["first_fix"]["magnitude_m"] == 1.0 and d["first_fix"]["source"] == "ransac"
    assert [r["source"] for r in d["per_source"]] == ["ransac", "fiducial"]
    assert [c["magnitude_m"] for c in d["per_fix"]] == [0.6, 0.8]
    assert json.loads(json.dumps(d))["per_fix"][0]["dist_travelled_m"] == 2.0


# --------------------------------------------------------------------------- #
# Log parsers vs the REAL emitters: every line below is rendered by the real
# dimos logger from the real module.py / priors.py call sites -- never typed out
# here. The previous parsers matched a format the pipeline had stopped emitting
# ("relocalize:", "published_t="), so they returned [] on every real run and every
# fix fell back to source="unknown"; only real bytes can catch that again.
# --------------------------------------------------------------------------- #
@dataclass
class _RunLog:
    """The two files a dimos run leaves behind, in memory: the console capture a
    harness tees to `.replay_run.log`, and the run's `main.jsonl`."""

    console_buf: io.StringIO = field(default_factory=io.StringIO)
    jsonl_buf: io.StringIO = field(default_factory=io.StringIO)

    @property
    def console(self) -> str:
        return self.console_buf.getvalue()

    @property
    def jsonl(self) -> str:
        return self.jsonl_buf.getvalue()

    def both(self) -> list[tuple[str, str]]:
        """[(rendering name, text)] -- one parser must read both."""
        return [("console", self.console), ("jsonl", self.jsonl)]


@contextmanager
def _capture_run_log(*modules: ModuleType) -> Iterator[_RunLog]:
    """Everything ``modules`` log, in BOTH renderings, from the REAL logger.

    setup_logger gives each dimos module a console StreamHandler and a main.jsonl
    RotatingFileHandler (logging_config.py); this swaps ONLY their streams for
    in-memory buffers, so the structlog processors, the formatters and the call
    sites stay exactly what production runs. One shared buffer per rendering, since
    a real run has one stdout and one main.jsonl for all modules."""
    log = _RunLog()
    swapped: list[tuple[logging.StreamHandler[Any], Any]] = []
    try:
        for mod in modules:
            # setup_logger names the stdlib logger by the caller's path relative to
            # the project root (logging_config.py); mirror that to find it.
            name = str(Path(str(mod.__file__)).relative_to(DIMOS_PROJECT_ROOT))
            handlers = logging.getLogger(name).handlers
            assert handlers, f"{name} has no handlers -- setup_logger wiring changed"
            for handler in handlers:
                assert isinstance(handler, logging.StreamHandler), (
                    f"{name} handler {handler!r} is not stream-backed; "
                    "the capture below cannot see it -- setup_logger wiring changed"
                )
                buf = log.jsonl_buf if isinstance(handler, logging.FileHandler) else log.console_buf
                swapped.append((handler, handler.setStream(buf)))
        yield log
    finally:
        for handler, stream in swapped:
            handler.setStream(stream)


def _bare_module(config: Config) -> RelocalizationModule:
    """A RelocalizationModule shell with just the attributes _try_relocalize reads
    -- no coordinator, no start() wiring (same shell test_relocalize.py uses)."""
    m = object.__new__(RelocalizationModule)
    m.config = config
    m._premap = _StubCloud(10)  # type: ignore[assignment]
    m._last_skip_log = 0.0
    m._last_pose_prior = LastPosePrior()
    ransac_entry = next(
        (p for p in config.priors if isinstance(p, RansacPriorConfig)), RansacPriorConfig()
    )
    m._ransac_prior = RansacPrior(interval_s=ransac_entry.interval_s)
    # Mirror module.__init__: the accept gate is per-prior now (fitness_threshold moved
    # onto each prior entry), keyed by the entry's source discriminator.
    m._accept_threshold = {p.type: p.fitness_threshold for p in config.priors}
    m._ransac_min_local_points = ransac_entry.min_local_points
    m._fiducial_prior = None
    m._now_fn = time.monotonic
    m._last_fix_map_T_world = None  # no previous fix -> the jump guard is in acquisition
    m._last_fix_ts_s = 0.0
    return m


class _StubCloud:
    """Minimal PointCloud2 stand-in: __len__ (the n_pts log field) and a
    `.pointcloud` sentinel handed to the monkeypatched relocalize()."""

    def __init__(self, n: int) -> None:
        self._n = n
        self.pointcloud = object()

    def __len__(self) -> int:
        return self._n


class _StubPrior:
    """Proposes ``n`` candidates under one source name. The census counts proposals,
    so only the count and the name are load-bearing."""

    def __init__(self, name: str, n: int) -> None:
        self.name = name
        self._n = n

    def propose(self, global_map: object, local_map: object) -> list[Candidate]:
        return [Candidate(T=np.eye(4), source=self.name) for _ in range(self._n)]


def _multi_source_module() -> RelocalizationModule:
    """ransac + last_pose (enabled) -> the judge path, which tags the accept with
    source=. A fiducial entry is configured but inert (_bare_module leaves
    _fiducial_prior None, so it is not among the enabled objects): it supplies the
    per-source accept gate's ``fiducial`` threshold key, so a monkeypatched fiducial
    win is gated by its own bar exactly as a real fiducial-configured run would be.

    verbose_eval_logging is ON because that is the log these parsers exist to read:
    the operating default emits one bare accept line, while `--eval` turns on the
    full per-cycle trace (census, judge finalists, every accept kwarg)."""
    return _bare_module(
        Config(
            priors=[
                RansacPriorConfig(fitness_threshold=0.45),
                LastPosePriorConfig(fitness_threshold=0.45),
                FiducialPriorConfig(fitness_threshold=0.45),
            ],
            verbose_eval_logging=True,
        )
    )


def test_parse_health_lines_reads_a_real_accept_in_both_renderings(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The real accept the module emits (event `relocalize accepted`, kwargs
    source/fitness/published_t_m) parses to one HealthLine carrying that source,
    that fitness, and the published world_T_map translation -- from the console
    capture AND from main.jsonl."""
    map_t_world = _rt(Rotation.from_euler("z", 25.0, degrees=True).as_matrix(),
                      np.array([1.5, -0.8, 0.05]))
    published_t_m = np.linalg.inv(map_t_world)[:3, 3]  # what the module publishes
    m = _multi_source_module()
    monkeypatch.setattr(
        module_mod, "_relocalize_with_priors", lambda *a, **k: (map_t_world, 0.873, "last_pose")
    )

    with _capture_run_log(module_mod) as log:
        assert m._try_relocalize(_StubCloud(55828), m._enabled_prior_objects()) is not None  # type: ignore[arg-type]

    for rendering, text in log.both():
        health = parse_health_lines(text)
        assert len(health) == 1, f"{rendering}: parsed {health} from {text!r}"
        assert health[0].source == "last_pose", rendering
        assert health[0].fitness == 0.873, rendering
        # the verbose accept is the one that carries a position at all -- the join key
        parsed_t_m = health[0].published_t_m
        assert parsed_t_m is not None, rendering
        # the log rounds the translation to 3 dp; that is the join key's precision
        np.testing.assert_allclose(parsed_t_m, published_t_m, atol=5e-4)


def test_parse_health_lines_defaults_to_ransac_on_the_single_source_path(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A ransac-only pool accepts through plain relocalize(), whose log carries NO
    source= (module.py tags a winner only when the judge ran). Both renderings must
    still yield a labelled fix -- ransac, not unknown."""
    m = _bare_module(
        Config(priors=[RansacPriorConfig(fitness_threshold=0.45)], verbose_eval_logging=True)
    )
    monkeypatch.setattr(
        module_mod, "_relocalize", lambda *a, **k: (_rt(np.eye(3), np.array([2.0, 1.0, 0.0])), 0.95)
    )

    with _capture_run_log(module_mod) as log:
        assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is not None  # type: ignore[arg-type]

    for rendering, text in log.both():
        health = parse_health_lines(text)
        assert len(health) == 1, f"{rendering}: parsed {health} from {text!r}"
        assert health[0].source == "ransac", rendering
        assert health[0].fitness == 0.95, rendering


def test_quiet_accept_parses_to_a_positionless_health_line(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """An operating run (verbose_eval_logging OFF, the default) logs ONE accept line:
    source, fitness, the cross-source margin, time_cost_s -- no published_t_m. It
    still parses to a HealthLine carrying those three real numbers, with
    published_t_m None. Dropping it instead would make a quiet log look exactly like
    a log this parser cannot read: empty, silent, every count zero.

    Positionless means unjoinable, so the fix it belongs to stays `unknown` -- which
    is why `--eval` must turn the verbose trace on."""
    m = _bare_module(
        Config(
            priors=[
                RansacPriorConfig(fitness_threshold=0.45),
                LastPosePriorConfig(fitness_threshold=0.45),
                # inert (no _fiducial_prior) -- supplies the per-source gate's fiducial key
                FiducialPriorConfig(fitness_threshold=0.45),
            ]
        )
    )
    assert m.config.verbose_eval_logging is False, "the operating default must be quiet"

    def _judged(*_a: Any, report: JudgeReport, **_k: Any) -> tuple[np.ndarray, float, str]:
        report.finalists = [("fiducial", 0.873), ("ransac", 0.695)]  # margin 0.178
        report.winner = "fiducial"
        return _rt(np.eye(3), np.array([1.5, -0.8, 0.05])), 0.873, "fiducial"

    monkeypatch.setattr(module_mod, "_relocalize_with_priors", _judged)

    with _capture_run_log(module_mod) as log:
        assert m._try_relocalize(_StubCloud(55828), m._enabled_prior_objects()) is not None  # type: ignore[arg-type]

    for rendering, text in log.both():
        health = parse_health_lines(text)
        assert len(health) == 1, f"{rendering}: parsed {health} from {text!r}"
        assert health[0].source == "fiducial", rendering
        assert health[0].fitness == 0.873, rendering
        assert health[0].margin == 0.178, rendering
        assert health[0].published_t_m is None, rendering
        # ...and with no join key it labels nothing, rather than claiming a fix.
        fixes = label_fixes_from_log([(0.0, np.eye(4))], health)
        assert [f.source for f in fixes] == ["unknown"], rendering


def test_parse_reject_lines_counts_real_rejects_and_never_scores_them_as_accepts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Two sub-threshold fixes emit two real `relocalize rejected` warnings: rejects
    count 2 in both renderings, and parse_health_lines stays empty -- a reject has no
    published_t_m and must never enter the per-source table."""
    m = _bare_module(Config(priors=[RansacPriorConfig(fitness_threshold=0.45)]))
    monkeypatch.setattr(module_mod, "_relocalize", lambda *a, **k: (np.eye(4), 0.30))

    with _capture_run_log(module_mod) as log:
        assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is None  # type: ignore[arg-type]
        assert m._try_relocalize(_StubCloud(1234), m._enabled_prior_objects()) is None  # type: ignore[arg-type]

    for rendering, text in log.both():
        assert len(parse_reject_lines(text)) == 2, f"{rendering}: {text!r}"
        assert parse_health_lines(text) == [], rendering


def test_parse_reject_lines_reads_the_source_on_every_live_branch(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Every live reject names the prior whose bar refused it, so ``--eval`` per-source
    reject counts are real. Parsed from REAL bytes on all three branches: quiet
    multi-prior, verbose ``--eval``, and the solo RANSAC path (which names ``ransac``
    even though its ACCEPT line omits source=). Only the legacy f-string, written
    before the field existed, buckets to ``unknown``."""
    monkeypatch.setattr(
        module_mod, "_relocalize_with_priors", lambda *a, **k: (np.eye(4), 0.30, "fiducial")
    )

    # QUIET multi-prior reject (operating default): the judge named the loser.
    quiet = _bare_module(
        Config(
            priors=[
                RansacPriorConfig(fitness_threshold=0.45),
                LastPosePriorConfig(fitness_threshold=0.45),
                # inert (no _fiducial_prior) -- supplies the per-source gate's fiducial key
                FiducialPriorConfig(fitness_threshold=0.45),
            ]
        )
    )
    assert quiet.config.verbose_eval_logging is False
    with _capture_run_log(module_mod) as qlog:
        assert quiet._try_relocalize(_StubCloud(1234), quiet._enabled_prior_objects()) is None  # type: ignore[arg-type]
    for rendering, text in qlog.both():
        assert parse_reject_lines(text) == ["fiducial"], f"{rendering}: {text!r}"

    # VERBOSE --eval reject: the branch --eval forces on, attributed the same way.
    verbose = _multi_source_module()
    with _capture_run_log(module_mod) as vlog:
        assert verbose._try_relocalize(_StubCloud(1234), verbose._enabled_prior_objects()) is None  # type: ignore[arg-type]
    for rendering, text in vlog.both():
        assert parse_reject_lines(text) == ["fiducial"], f"{rendering}: {text!r}"

    # SOLO ransac path: no winning source from the judge, still attributed to the
    # entry whose threshold= the line prints.
    solo = _bare_module(Config(priors=[RansacPriorConfig(fitness_threshold=0.45)]))
    monkeypatch.setattr(module_mod, "_relocalize", lambda *a, **k: (np.eye(4), 0.30))
    with _capture_run_log(module_mod) as slog:
        assert solo._try_relocalize(_StubCloud(1234), solo._enabled_prior_objects()) is None  # type: ignore[arg-type]
    for rendering, text in slog.both():
        assert parse_reject_lines(text) == ["ransac"], f"{rendering}: {text!r}"

    # legacy sourceless reject -> unknown, and it still counts as one reject.
    assert parse_reject_lines(_LEGACY_CONSOLE_LOG) == ["unknown"]
    assert len(parse_reject_lines(_LEGACY_CONSOLE_LOG)) == 1


def test_parse_census_reads_the_real_counts_dict_in_both_renderings(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """priors.py logs the proposal census as a dict kwarg (`counts={'ransac': 2,
    ...}` on the console, a JSON object in main.jsonl). One cycle -> one dict with
    each proposer's count; the fiducial entry is what drives 'proposed but lost'."""
    monkeypatch.setattr(priors_mod, "refine_candidates", lambda *a, **k: (np.eye(4), 0.9, 0))

    with _capture_run_log(priors_mod) as log:
        relocalize_with_priors(
            None,  # type: ignore[arg-type]
            None,  # type: ignore[arg-type]
            [_StubPrior("ransac", 2), _StubPrior("fiducial", 1)],  # type: ignore[list-item]
            verbose_eval_logging=True,  # the census is part of the --eval trace
        )

    for rendering, text in log.both():
        census = parse_census(text)
        assert census == [{"ransac": 2, "fiducial": 1}], f"{rendering}: {text!r}"


def test_real_log_labels_stream_fixes_end_to_end(monkeypatch: pytest.MonkeyPatch) -> None:
    """The whole supplement path on real bytes: two accepts from DIFFERENT priors,
    joined to the TFs the module published, give a per-source table with zero
    `unknown` -- the failure the dead parsers produced on every run."""
    first = _rt(Rotation.from_euler("z", 10.0, degrees=True).as_matrix(), np.array([1.0, 0.0, 0.0]))
    second = _rt(Rotation.from_euler("z", 40.0, degrees=True).as_matrix(), np.array([4.0, 2.0, 0.0]))
    m = _multi_source_module()

    with _capture_run_log(module_mod, priors_mod) as log:
        monkeypatch.setattr(
            module_mod, "_relocalize_with_priors", lambda *a, **k: (first, 0.9, "ransac")
        )
        tf_a = m._try_relocalize(_StubCloud(1000), m._enabled_prior_objects())  # type: ignore[arg-type]
        monkeypatch.setattr(
            module_mod, "_relocalize_with_priors", lambda *a, **k: (second, 0.7, "fiducial")
        )
        tf_b = m._try_relocalize(_StubCloud(1000), m._enabled_prior_objects())  # type: ignore[arg-type]
    assert tf_a is not None and tf_b is not None

    for rendering, text in log.both():
        # /tf republishes each accept until the next one; dedup recovers the two.
        tf_samples = [(0.0, tf_a.to_matrix()), (0.5, tf_a.to_matrix()), (1.0, tf_b.to_matrix())]
        fixes = label_fixes_from_log(dedup_tf_fixes(tf_samples), parse_health_lines(text))
        assert [f.source for f in fixes] == ["ransac", "fiducial"], f"{rendering}: {text!r}"
        stats = compute_stats(
            fixes, _parked_odom([0.25, 1.25]), parse_census(text), len(parse_reject_lines(text)),
            mode="live", held_out_note="n",
        )
        by = {r.source: r for r in stats.rows}
        assert by["ransac"].won == 1 and by["fiducial"].won == 1, rendering
        assert by["fiducial"].med_fit == 0.7, rendering
        assert stats.rejects == 0, rendering


# --------------------------------------------------------------------------- #
# The LEGACY wire format. These lines cannot be re-emitted -- the f-string call
# sites are gone -- so unlike every other parser test they are a FIXTURE, copied
# verbatim (paths shortened) out of a real archived capture,
# trial/harness/out/results_dimos/survey2_heldout.replay_run.log. That archive is
# the trial's evidence: when the parser stopped understanding this format it
# returned [] and the run's 24/28 fiducial-proposal census silently read as null.
# --------------------------------------------------------------------------- #
_LEGACY_CONSOLE_LOG = "\n".join([
    "22:11:37.254 [inf][pping/relocalization/module.py] relocalize: fiducial prior enabled"
    " marker_map_file='/tmp/survey1.marker_map.yaml' n_markers=5",
    "22:11:49.281 [inf][pping/relocalization/priors.py] relocalize candidates: ransac=34",
    "22:11:50.589 [inf][pping/relocalization/module.py] relocalize: fitness=0.756"
    " time_cost=13.0s n_pts=89003 reloc_t=[-2.467, -8.645, -0.086] TF 'world' -> 'map'"
    " published_t=[-8.849, -1.587, 0.073] source=ransac",
    "22:12:06.431 [inf][pping/relocalization/priors.py] relocalize candidates:"
    " fiducial=1 ransac=34",
    "22:12:08.022 [inf][pping/relocalization/module.py] relocalize: fitness=0.568"
    " time_cost=17.4s n_pts=122958 reloc_t=[0.145, -11.99, -0.079] TF 'world' -> 'map'"
    " published_t=[-9.046, 7.87, -0.105] source=fiducial",
    "22:19:52.114 [war][pping/relocalization/module.py] relocalize rejected:"
    " fitness=0.449 < threshold=0.45 time_cost=35.1s n_pts=269597",
])


def test_parsers_read_the_legacy_f_string_format() -> None:
    """The archived format parses to the same records the current one does: two
    accepts with their OWN source (the legacy line puts `source=` last, which is
    how a positional regex silently labelled every accept ransac), the published
    translation off bare `published_t=`, two census cycles with fiducial in one,
    and one reject. The `fiducial prior enabled` line carries no fitness and must
    not be counted as an accept."""
    health = parse_health_lines(_LEGACY_CONSOLE_LOG)
    assert [h.source for h in health] == ["ransac", "fiducial"]
    assert [h.fitness for h in health] == [0.756, 0.568]
    assert health[0].published_t_m == (-8.849, -1.587, 0.073)

    census = parse_census(_LEGACY_CONSOLE_LOG)
    assert census == [{"ransac": 34}, {"fiducial": 1, "ransac": 34}]
    assert len(parse_reject_lines(_LEGACY_CONSOLE_LOG)) == 1


def test_legacy_census_drives_the_prior_activity_line() -> None:
    """End of the chain the regression broke: legacy census -> compute_stats ->
    the prior-activity line. `1/2 cycles` is the evidence that vanished as null."""
    stats = compute_stats(
        [], np.empty((0,)), parse_census(_LEGACY_CONSOLE_LOG), 1,
        mode="held_out", held_out_note="n",
    )
    assert stats.fiducial_proposed_cycles == 1 and stats.census_cycles == 2
    assert "fiducial proposed 1/2 cycles" in format_report(stats, title="legacy")


def test_current_format_parses_through_the_same_path(monkeypatch: pytest.MonkeyPatch) -> None:
    """The legacy support is ADDITIVE: bytes the emitters render today still parse
    to the same records, in both renderings, through one parser."""
    m = _multi_source_module()
    monkeypatch.setattr(
        module_mod, "_relocalize_with_priors",
        lambda *a, **k: (_rt(np.eye(3), np.array([1.0, 2.0, 0.0])), 0.61, "fiducial"),
    )
    monkeypatch.setattr(priors_mod, "refine_candidates", lambda *a, **k: (np.eye(4), 0.9, 0))

    with _capture_run_log(module_mod, priors_mod) as log:
        relocalize_with_priors(
            None,  # type: ignore[arg-type]
            None,  # type: ignore[arg-type]
            [_StubPrior("ransac", 34), _StubPrior("fiducial", 1)],  # type: ignore[list-item]
            verbose_eval_logging=True,  # the census is part of the --eval trace
        )
        assert m._try_relocalize(_StubCloud(89003), m._enabled_prior_objects()) is not None  # type: ignore[arg-type]

    for rendering, text in log.both():
        health = parse_health_lines(text)
        assert [h.source for h in health] == ["fiducial"], f"{rendering}: {text!r}"
        assert health[0].fitness == 0.61, rendering
        assert parse_census(text) == [{"ransac": 34, "fiducial": 1}], rendering


def test_parse_run_log_warns_when_a_log_yields_nothing(tmp_path: Path) -> None:
    """A log the parser reads NOTHING out of must say so, naming the file. Silence
    here is indistinguishable from a real 'the fiducial prior never proposed', and
    that is exactly how the legacy captures' census went missing unnoticed."""
    empty = tmp_path / "nothing.replay_run.log"
    empty.write_text("22:11:37.254 [inf][core/module.py] some other module started\n")

    with _capture_run_log(eval_mod) as log:
        assert parse_run_log(empty) == ([], [], [])

    for rendering, text in log.both():
        assert "no relocalize records" in text, f"{rendering}: {text!r}"
        assert str(empty) in text, rendering

    # ...and a log that DOES parse stays quiet.
    real = tmp_path / "legacy.replay_run.log"
    real.write_text(_LEGACY_CONSOLE_LOG)
    with _capture_run_log(eval_mod) as quiet:
        health, census, reject_sources = parse_run_log(real)
    assert (len(health), len(census), len(reject_sources)) == (2, 2, 1)
    assert reject_sources == ["unknown"]  # the legacy reject named no source
    assert "no relocalize records" not in quiet.console


# --------------------------------------------------------------------------- #
# accepted_points_in_map: the live rerun overlay. Colours are asserted as exact
# literals, not recomputed from SOURCE_COLORS -- the overlay's whole job is that a
# colour means one prior, so a palette edit must fail here and be deliberate.
# --------------------------------------------------------------------------- #
_EXPECTED_RGB: dict[str, tuple[int, int, int]] = {
    "ransac": (43, 108, 176),  # #2b6cb0 blue
    "fiducial": (221, 107, 32),  # #dd6b20 orange
    "last_pose": (113, 128, 150),  # #718096 grey
    "unknown": (155, 44, 44),  # #9b2c2c dim red
}


def test_overlay_colours_each_winning_prior_distinctly() -> None:
    """One point per accept, coloured by the prior that won it, labelled with that
    source and its fitness. Every source keeps its own colour and no two collide --
    reading ransac wins from fiducial wins at a glance is the point of the overlay."""
    odom = np.array([[float(i), 0.0, 0.0, 0.0] for i in range(len(SOURCES))])
    fixes = [
        Fix(ts=float(i), world_map_fix=np.eye(4), source=s, fitness=0.5)
        for i, s in enumerate(SOURCES)
    ]

    points = accepted_points_in_map(fixes, odom)

    assert points.colors_rgb == [_EXPECTED_RGB[s] for s in SOURCES]
    assert len(set(points.colors_rgb)) == len(SOURCES), "two priors share a colour"
    assert points.labels == [f"{s} fit=0.50" for s in SOURCES]


def test_overlay_marks_an_unknown_source_visibly_not_as_a_second_grey() -> None:
    """A fix no health line claims stays `unknown` -- a JOIN FAILURE, not a prior.
    It draws in dim red, unmistakably not the last_pose grey it used to share a
    colour family with, and its label states that no fitness was recovered."""
    odom = np.array([[0.0, 0.0, 0.0, 0.0]])
    fixes = [Fix(ts=0.0, world_map_fix=np.eye(4), source="unknown", fitness=float("nan"))]

    points = accepted_points_in_map(fixes, odom)

    red, green, blue = points.colors_rgb[0]
    assert (red, green, blue) == _EXPECTED_RGB["unknown"]
    assert red - max(green, blue) > 100, "unknown must read as RED, not another grey"
    assert all(points.colors_rgb[0] != _EXPECTED_RGB[s] for s in SOURCES)
    assert points.labels == ["unknown fit=-"]
    # the palette an unlisted future source falls back to is that same dim red
    assert SOURCE_COLORS["unknown"] == "#9b2c2c"


def test_overlay_points_match_the_deduped_accepts_one_to_one_in_the_map_frame() -> None:
    """The whole overlay path on constructed known truth: /tf republishes each accept
    until the next, dedup recovers two accepts, and the overlay draws exactly two
    points -- each at map_T_world @ robot_world for THAT accept's own fix, the same
    frame the premap is drawn in. A point at the fix's own translation (the map
    origin) instead would fail this."""
    map_t_world_a = _rt(Rotation.from_euler("z", 90.0, degrees=True).as_matrix(),
                        np.array([1.0, 0.0, 0.0]))
    map_t_world_b = _rt(Rotation.from_euler("z", -30.0, degrees=True).as_matrix(),
                        np.array([-2.0, 0.5, 0.0]))
    # module.py publishes the INVERSE of relocalize()'s map_T_world on /tf
    world_map_a = np.linalg.inv(map_t_world_a)
    world_map_b = np.linalg.inv(map_t_world_b)

    robot_a = np.array([2.0, 3.0, 0.5])
    robot_b = np.array([4.0, -1.0, 0.5])
    odom = np.array([
        [0.0, *robot_a],
        [1.0, 9.9, 9.9, 9.9],  # a sample far from either accept must not be picked
        [2.0, *robot_b],
    ])
    tf_samples = [
        (0.0, world_map_a), (0.5, world_map_a), (1.0, world_map_a),  # republished
        (2.0, world_map_b), (2.5, world_map_b),
    ]
    health = [
        HealthLine("ransac", 0.91, tuple(world_map_a[:3, 3])),
        HealthLine("fiducial", 0.72, tuple(world_map_b[:3, 3])),
    ]

    accepts = dedup_tf_fixes(tf_samples)
    points = accepted_points_in_map(label_fixes_from_log(accepts, health), odom)

    assert len(accepts) == 2
    assert len(points) == len(accepts), "overlay must be 1:1 with the deduped accepts"
    expected = [
        (map_t_world_a @ np.array([*robot_a, 1.0]))[:3],
        (map_t_world_b @ np.array([*robot_b, 1.0]))[:3],
    ]
    np.testing.assert_allclose(points.positions_map_m, expected, atol=1e-9)
    assert points.colors_rgb == [_EXPECTED_RGB["ransac"], _EXPECTED_RGB["fiducial"]]
    assert points.labels == ["ransac fit=0.91", "fiducial fit=0.72"]


def test_overlay_draws_nothing_without_odom() -> None:
    """No odom captured -> no robot position to draw a fix at. The overlay stays
    empty rather than falling back to the fix's translation, which is the map origin
    and would put every accept in one blob."""
    fixes = [Fix(ts=0.0, world_map_fix=np.eye(4), source="ransac", fitness=0.9)]
    assert len(accepted_points_in_map(fixes, np.empty((0, 4)))) == 0


def test_write_report_prints_the_table_and_the_corrections(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """What an --eval run leaves an operator with: the per-source table, the
    correction block, and two artifact paths -- json and trajectory png -- both on
    disk."""
    fixes, odom = _known_track()
    stats = compute_stats(fixes, odom, [], 0, mode="live", held_out_note="n")

    paths = write_report(stats, odom, fixes, {}, tmp_path, "releval", title="live")
    printed = capsys.readouterr().out

    assert set(paths) == {"json", "png"}
    assert all(p.exists() for p in paths.values())
    # the per-source table (columns are width-padded, so match the names in order)
    assert re.search(r"source +prop +acc +rej +%traj +med_fit", printed), printed
    assert "correction (at the robot, n=2): med=0.700m" in printed  # the correction block


def _bare_eval(out_dir: Path) -> RelocEval:
    """A RelocEval shell holding just what _finalize reads -- one accepted /tf fix
    and the odom it landed on -- with no bus and no coordinator (the same shell
    _bare_module uses). run_log_file points at a file that does not exist, so the
    log supplement degrades to empty the way a live run with no trace does."""
    m = object.__new__(RelocEval)
    m.config = EvalConfig(
        out_dir=str(out_dir), tag="releval", run_log_file=str(out_dir / "absent.jsonl")
    )
    m._odom = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0)]
    m._world_map = [(0.5, _rt(np.eye(3), np.array([0.2, 0.0, 0.0])))]
    m._finalized = False
    return m


def test_the_exit_hook_writes_the_report_when_stop_never_runs(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """Ctrl+C and a crashing run tear the module down without ever running stop()
    here, so the atexit hook start() arms is what leaves the operator a report: the
    same table and the same two artifacts a clean stop writes."""
    m = _bare_eval(tmp_path)

    m._finalize()  # what atexit.register(self.stop) reaches at process exit

    printed = capsys.readouterr().out
    assert re.search(r"source +prop +acc +rej +%traj +med_fit", printed), printed
    assert (tmp_path / "releval.eval.json").exists()
    assert (tmp_path / "releval.trajectory.png").exists()


def test_the_report_is_written_once_however_often_teardown_arrives(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """Teardown reaches this module up to three times on one shutdown (the
    coordinator's RPC, the worker's own teardown, the exit hook). The report is
    written on the first and never again -- a second pass would overwrite the
    artifacts and print the table twice."""
    m = _bare_eval(tmp_path)

    m._finalize()
    m._finalize()

    printed = capsys.readouterr().out
    assert printed.count(f"[releval] wrote {tmp_path / 'releval.eval.json'}") == 1
