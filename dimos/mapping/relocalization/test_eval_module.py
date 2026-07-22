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
import logging
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.mapping.relocalization.eval_module import (
    SOURCE_COLORS,
    SOURCES,
    Fix,
    HealthLine,
    accepted_points_in_map,
    aligned_err_t_m,
    compute_stats,
    count_rejects,
    dedup_tf_fixes,
    format_report,
    label_fixes_from_log,
    parse_census,
    parse_health_lines,
    resolve_heldout_alignment,
    stats_to_dict,
    umeyama_alignment,
)
import dimos.mapping.relocalization.module as module_mod
from dimos.mapping.relocalization.module import Config, RelocalizationModule
import dimos.mapping.relocalization.priors as priors_mod
from dimos.mapping.relocalization.priors import (
    Candidate,
    LastPosePrior,
    LastPosePriorConfig,
    RansacPriorConfig,
    relocalize_with_priors,
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
    m._fiducial_prior = None
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
    """ransac + last_pose -> the judge path, which tags the accept with source=."""
    return _bare_module(
        Config(priors=[RansacPriorConfig(), LastPosePriorConfig()], fitness_threshold=0.45)
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
        assert m._try_relocalize(_StubCloud(55828)) is not None  # type: ignore[arg-type]

    for rendering, text in log.both():
        health = parse_health_lines(text)
        assert len(health) == 1, f"{rendering}: parsed {health} from {text!r}"
        assert health[0].source == "last_pose", rendering
        assert health[0].fitness == 0.873, rendering
        # the log rounds the translation to 3 dp; that is the join key's precision
        np.testing.assert_allclose(health[0].published_t_m, published_t_m, atol=5e-4)


def test_parse_health_lines_defaults_to_ransac_on_the_single_source_path(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A ransac-only pool accepts through plain relocalize(), whose log carries NO
    source= (module.py tags a winner only when the judge ran). Both renderings must
    still yield a labelled fix -- ransac, not unknown."""
    m = _bare_module(Config(priors=[RansacPriorConfig()], fitness_threshold=0.45))
    monkeypatch.setattr(
        module_mod, "_relocalize", lambda *a, **k: (_rt(np.eye(3), np.array([2.0, 1.0, 0.0])), 0.95)
    )

    with _capture_run_log(module_mod) as log:
        assert m._try_relocalize(_StubCloud(1234)) is not None  # type: ignore[arg-type]

    for rendering, text in log.both():
        health = parse_health_lines(text)
        assert len(health) == 1, f"{rendering}: parsed {health} from {text!r}"
        assert health[0].source == "ransac", rendering
        assert health[0].fitness == 0.95, rendering


def test_count_rejects_counts_real_rejects_and_never_scores_them_as_accepts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Two sub-threshold fixes emit two real `relocalize rejected` warnings: rejects
    count 2 in both renderings, and parse_health_lines stays empty -- a reject has no
    published_t_m and must never enter the per-source table."""
    m = _bare_module(Config(priors=[RansacPriorConfig()], fitness_threshold=0.45))
    monkeypatch.setattr(module_mod, "_relocalize", lambda *a, **k: (np.eye(4), 0.30))

    with _capture_run_log(module_mod) as log:
        assert m._try_relocalize(_StubCloud(1234)) is None  # type: ignore[arg-type]
        assert m._try_relocalize(_StubCloud(1234)) is None  # type: ignore[arg-type]

    for rendering, text in log.both():
        assert count_rejects(text) == 2, f"{rendering}: {text!r}"
        assert parse_health_lines(text) == [], rendering


def test_parse_census_reads_the_real_counts_dict_in_both_renderings(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """priors.py logs the proposal census as a dict kwarg (`counts={'ransac': 2,
    ...}` on the console, a JSON object in main.jsonl). One cycle -> one dict with
    each proposer's count; the fiducial entry is what drives 'proposed but lost'."""
    monkeypatch.setattr(priors_mod, "refine_candidates", lambda *a, **k: (np.eye(4), 0.9, 0))

    with _capture_run_log(priors_mod) as log:
        relocalize_with_priors(
            None, None, [_StubPrior("ransac", 2), _StubPrior("fiducial", 1)]  # type: ignore[arg-type]
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
        tf_a = m._try_relocalize(_StubCloud(1000))  # type: ignore[arg-type]
        monkeypatch.setattr(
            module_mod, "_relocalize_with_priors", lambda *a, **k: (second, 0.7, "fiducial")
        )
        tf_b = m._try_relocalize(_StubCloud(1000))  # type: ignore[arg-type]
    assert tf_a is not None and tf_b is not None

    for rendering, text in log.both():
        # /tf republishes each accept until the next one; dedup recovers the two.
        tf_samples = [(0.0, tf_a.to_matrix()), (0.5, tf_a.to_matrix()), (1.0, tf_b.to_matrix())]
        fixes = label_fixes_from_log(dedup_tf_fixes(tf_samples), parse_health_lines(text))
        assert [f.source for f in fixes] == ["ransac", "fiducial"], f"{rendering}: {text!r}"
        stats = compute_stats(
            fixes, np.array([0.25, 1.25]), parse_census(text), count_rejects(text),
            mode="live", held_out_note="n",
        )
        by = {r.source: r for r in stats.rows}
        assert by["ransac"].won == 1 and by["fiducial"].won == 1, rendering
        assert by["fiducial"].med_fit == 0.7, rendering
        assert stats.rejects == 0, rendering


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
