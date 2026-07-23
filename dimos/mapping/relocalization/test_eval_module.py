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

"""Crucial-invariant tests for eval_module.py's PURE analysis -- no LCM bus, no
replay, no DB. Deterministic: constructed Fix lists + a RelocEval shell."""

from __future__ import annotations

from pathlib import Path
import re

import numpy as np
import pytest

from dimos.mapping.relocalization.eval_module import (
    EvalConfig,
    Fix,
    RelocEval,
    compute_stats,
    format_report,
)


def _fixture_fixes() -> list[Fix]:
    """ransac wins twice (one success, one fail), fiducial once (success); the
    err/fitness values are hand-picked so the per-source medians are checkable."""
    return [
        Fix(ts=1.0, world_map_fix=None, source="ransac", fitness=0.9, err_t_m=0.2, success=True),
        Fix(ts=2.0, world_map_fix=None, source="fiducial", fitness=0.8, err_t_m=0.5, success=True),
        Fix(ts=3.0, world_map_fix=None, source="ransac", fitness=0.7, err_t_m=1.5, success=False),
    ]


def test_compute_stats_aggregates_per_source_and_format_report_renders_them() -> None:
    """Invariant 11 (per-source stats): compute_stats aggregates each source's own
    fixes -- won counts, %traj over the COVERED odom samples (0.5 s is before the first
    fix, uncovered; the rest map ransac/fiducial/ransac), and per-source medians -- and
    format_report renders a row per source. A single shared module gate could not
    attribute any of these."""
    odom = np.array([[t, 0.0, 0.0, 0.0] for t in (0.5, 1.5, 2.5, 3.5)], dtype=float)
    stats = compute_stats(
        _fixture_fixes(), odom, [], n_rejects=4, mode="held_out", held_out_note="n"
    )
    by = {r.source: r for r in stats.rows}

    assert by["ransac"].won == 2 and by["fiducial"].won == 1
    assert abs(by["ransac"].pct_traj - 200.0 / 3.0) < 1e-9  # active for 2 of 3 covered samples
    assert abs(by["fiducial"].pct_traj - 100.0 / 3.0) < 1e-9
    assert by["ransac"].med_err_m == 0.85 and by["fiducial"].med_err_m == 0.5
    assert stats.accepts == 3 and abs(stats.coverage_pct - 75.0) < 1e-9  # 3 of 4 samples covered

    report = format_report(stats, title="demo")
    assert re.search(r"source +prop +acc +rej +false +%traj +med_err +med_fit", report), report
    assert "ransac" in report and "fiducial" in report


def _bare_eval(out_dir: Path) -> RelocEval:
    """A RelocEval shell holding just what _finalize reads -- one accepted /tf fix and
    the odom it landed on -- with no bus and no coordinator. run_log_file points at a
    missing file, so the census supplement degrades to empty as a live run with no
    trace does."""
    m = object.__new__(RelocEval)
    m.config = EvalConfig(
        out_dir=str(out_dir), tag="releval", run_log_file=str(out_dir / "absent.jsonl")
    )
    m._odom = [(0.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0)]
    m._world_map = [(0.5, np.eye(4))]
    m._finalized = False
    return m


def test_atexit_hook_writes_report_when_stop_never_runs(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """Invariant 11 (Ctrl+C survival): a crash or Ctrl+C tears the module down without
    stop(), so the atexit hook start() arms is what leaves the operator a report -- the
    same table and the same two artifacts a clean stop writes. Idempotent: teardown can
    reach this module several times on one shutdown, and it writes exactly once."""
    m = _bare_eval(tmp_path)

    m._finalize()  # what atexit.register(self.stop) reaches at process exit
    m._finalize()  # a second teardown must not re-write or re-print

    printed = capsys.readouterr().out
    assert re.search(r"source +prop +acc +rej +%traj +med_fit", printed), printed
    assert (tmp_path / "releval.eval.json").exists()
    assert (tmp_path / "releval.trajectory.png").exists()
    assert printed.count(f"[releval] wrote {tmp_path / 'releval.eval.json'}") == 1
