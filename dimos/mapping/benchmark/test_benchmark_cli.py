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

"""Fast, no-robot tests for `dimos benchmark` (dimos/mapping/benchmark/cli.py).

Two things get exercised: CLI arg parsing (via `dimos.robot.cli.dimos.main` +
typer's CliRunner, argument-shaped errors only -- none of these reach a
transport/camera, so no robot is needed) and a fixture-log scoring roundtrip
(write a JSONL fixture -> load_jsonl -> compute_run_metrics -> append_csv_row
-> regenerate_report -> assert the CSV/RESULTS.md come out right), plus the
start/end tag adoption logic (`TagAdopter`) in isolation. `BenchmarkLogger`
itself needs a live transport/camera and is intentionally NOT constructed
here.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
import re

import numpy as np
import pytest
from typer.testing import CliRunner

from dimos.mapping.benchmark.cli import (
    RESULTS_CSV_NAME,
    RESULTS_MD_NAME,
    TagAdopter,
    append_csv_row,
    effective_route,
    make_run_id,
    normalize_mode,
    regenerate_report,
)
from dimos.mapping.benchmark.testdata import testcases
from dimos.mapping.benchmark.tool_benchmark import compute_run_metrics, load_jsonl
from dimos.mapping.benchmark.type import RunRecord
from dimos.robot.cli.dimos import main

# -- pure helpers -----------------------------------------------------------


@pytest.mark.parametrize(
    ("mode", "expected"),
    [("odom", "odom"), ("lidar", "lidar"), ("visual", "visual"), ("marker", "visual")],
)
def test_normalize_mode_canonical_and_hidden_alias(mode: str, expected: str) -> None:
    assert normalize_mode(mode) == expected


def test_normalize_mode_rejects_unknown_mode() -> None:
    with pytest.raises(ValueError, match="unknown benchmark mode"):
        normalize_mode("fused")


def test_effective_route_folds_variant_only_when_given() -> None:
    assert effective_route("drift-recovery", None) == "drift-recovery"
    assert effective_route("drift-recovery", "kidnap") == "drift-recovery-kidnap"


def test_make_run_id_shape_with_and_without_variant() -> None:
    run_id = make_run_id("Drift Recovery", "kidnap", "visual")
    assert run_id.startswith("drift-recovery-kidnap__visual__")
    run_id_no_variant = make_run_id("drift-recovery", None, "odom")
    assert run_id_no_variant.startswith("drift-recovery__odom__")


# -- TagAdopter (ported holdout_overlay.py adoption logic) -------------------


def _square(x: float, y: float, side: float) -> np.ndarray:
    return np.array([[x, y], [x + side, y], [x + side, y + side], [x, y + side]], dtype=np.float32)


def test_tag_adopter_adopts_prominent_unmapped_stable_tag() -> None:
    """Mirrors holdout_overlay.py's own --selftest guard: an unmapped tag (7)
    and a mapped tag (0) both stably visible -- adoption must pick the
    unmapped one, matching SELFTEST_TAG_ID/SELFTEST_MAPPED_ID there."""
    adopter = TagAdopter(map_ids={0})
    adopted = None
    for _ in range(10):
        adopted = adopter.update({0: _square(0, 0, 50), 7: _square(100, 100, 20)})
    assert adopted == 7
    assert adopter.adopted_id == 7
    assert adopter.locked is True
    assert adopter.mapped_only is False


def test_tag_adopter_any_stable_tag_adopts_with_no_marker_map() -> None:
    """odom/lidar modes with no --marker-map: map_ids is empty, so a tag that
    WOULD be excluded if mapped (id 0) is a valid candidate on its own."""
    adopter = TagAdopter(map_ids=set())
    adopted = None
    for _ in range(10):
        adopted = adopter.update({0: _square(0, 0, 30)})
    assert adopted == 0
    assert adopter.locked is True


def test_tag_adopter_mapped_only_blocks_adoption() -> None:
    """Every stable visible tag is in the map -- nothing should adopt, and
    mapped_only must flag it (the CLI prints a guard message on this)."""
    adopter = TagAdopter(map_ids={0})
    for _ in range(10):
        adopted = adopter.update({0: _square(0, 0, 30)})
    assert adopted is None
    assert adopter.adopted_id is None
    assert adopter.mapped_only is True
    assert adopter.locked is False


def test_tag_adopter_locks_and_ignores_later_frames() -> None:
    """Once adopted, a headless run must NOT flip to a different tag even if
    a bigger/more-stable candidate shows up later -- flipping mid-run would
    compare two different physical tags across the start/end window."""
    adopter = TagAdopter(map_ids=set())
    for _ in range(10):
        adopter.update({7: _square(0, 0, 20)})
    assert adopter.adopted_id == 7

    for _ in range(10):
        adopted = adopter.update({9: _square(50, 50, 200)})  # bigger, different tag
    assert adopted == 7
    assert adopter.adopted_id == 7


def test_tag_adopter_needs_min_seen_frames_before_adopting() -> None:
    """Fewer than ADOPT_MIN_SEEN (8) sightings in the last 10 frames must not
    adopt yet -- adoption is a stability gate, not a first-sight grab."""
    adopter = TagAdopter(map_ids=set())
    adopted = None
    for _ in range(7):
        adopted = adopter.update({3: _square(0, 0, 20)})
    assert adopted is None
    assert adopter.adopted_id is None
    assert adopter.locked is False


# -- fixture-log scoring roundtrip (no robot) --------------------------------


def _dump_records(records: list[RunRecord], path: Path) -> None:
    """Serialize RunRecords back to the raw JSONL shape metrics_logger.py /
    this tool's BenchmarkLogger writes, so load_jsonl can reparse them."""
    with path.open("w") as f:
        for r in records:
            d: dict[str, object] = {"type": r.type, "ts": r.ts}
            for key, value in (
                ("logged_at", r.logged_at),
                ("frame_id", r.frame_id),
                ("child_frame_id", r.child_frame_id),
                ("magnitude_m", r.magnitude_m),
                ("level", r.level),
                ("logger", r.logger),
                ("event", r.event),
                ("marker_id", r.marker_id),
                ("reprojection_error_px", r.reprojection_error_px),
            ):
                if value is not None:
                    d[key] = value
            if r.translation is not None:
                d["translation"] = list(r.translation)
            if r.rotation is not None:
                d["rotation"] = list(r.rotation)
            f.write(json.dumps(d) + "\n")


def test_fixture_log_roundtrip_writes_csv_and_results_md(tmp_path: Path) -> None:
    case = testcases[1]  # "marker-loop-drift-corrected" -- has real correction/reject data
    log_path = tmp_path / "fixture_run.jsonl"
    _dump_records(case.records, log_path)

    records = load_jsonl(log_path)
    metrics = compute_run_metrics(
        records,
        run_id="fixture-run",
        route=case.route,
        mode=case.mode,
        duration_s=case.duration_s,
        notes="fixture roundtrip",
        log_path=str(log_path),
        start_end_tag=case.start_end_tag,
    )
    for field_name, expected_value in case.expected.items():
        assert getattr(metrics, field_name) == expected_value

    results_dir = tmp_path / "results"
    append_csv_row(results_dir, metrics)
    n_rows = regenerate_report(results_dir)
    assert n_rows == 1

    csv_path = results_dir / RESULTS_CSV_NAME
    assert csv_path.exists()
    with csv_path.open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 1
    assert rows[0]["route"] == case.route
    assert rows[0]["mode"] == case.mode
    assert rows[0]["notes"] == "fixture roundtrip"

    md_path = results_dir / RESULTS_MD_NAME
    md = md_path.read_text()
    assert case.route in md
    assert "Start/End closure" in md
    assert "Holdout" not in md  # wording straggler must not resurface in the new tool


def test_report_renders_head_to_head_and_start_end_referee_footnote(tmp_path: Path) -> None:
    """testcases[3]/[4] share route 'holdout-referee' across odom/marker modes
    and both carry a start_end_tag -- exercises the head-to-head grouping and
    the referee footnote in one pass."""
    results_dir = tmp_path / "results"
    for case in (testcases[3], testcases[4]):
        metrics = compute_run_metrics(
            case.records,
            run_id=case.name,
            route=case.route,
            mode=case.mode,
            duration_s=case.duration_s,
            start_end_tag=case.start_end_tag,
        )
        append_csv_row(results_dir, metrics)

    n_rows = regenerate_report(results_dir)
    assert n_rows == 2

    md = (results_dir / RESULTS_MD_NAME).read_text()
    assert "## Head-to-head (by route)" in md
    assert "### holdout-referee" in md
    assert "start/end tag referee: tag 42" in md


# -- CLI arg parsing (no robot -- these all fail before any transport is
# touched: mode/variant/marker-map validation happens first in
# run_benchmark()) -------------------------------------------------------


def _strip_ansi(s: str) -> str:
    return re.sub(r"\x1b\[[0-9;]*m", "", s)


def test_cli_help_lists_benchmark_run_flags() -> None:
    result = CliRunner().invoke(main, ["benchmark", "run", "--help"])
    assert result.exit_code == 0, result.output
    output_plain = _strip_ansi(result.output)
    for flag in [
        "--mode",
        "--route",
        "--variant",
        "--notes",
        "--marker-map",
        "--start-end-tag",
        "--results-dir",
    ]:
        assert flag in output_plain


def test_cli_help_lists_benchmark_report_flags() -> None:
    result = CliRunner().invoke(main, ["benchmark", "report", "--help"])
    assert result.exit_code == 0, result.output
    assert "--results-dir" in _strip_ansi(result.output)


def test_cli_run_rejects_unknown_mode(tmp_path: Path) -> None:
    result = CliRunner().invoke(
        main,
        [
            "benchmark",
            "run",
            "--mode",
            "fused",
            "--route",
            "drift-recovery",
            "--results-dir",
            str(tmp_path),
        ],
    )
    assert result.exit_code == 2, result.output
    assert "unknown benchmark mode" in result.output


def test_cli_run_rejects_unknown_variant(tmp_path: Path) -> None:
    result = CliRunner().invoke(
        main,
        [
            "benchmark",
            "run",
            "--mode",
            "odom",
            "--route",
            "drift-recovery",
            "--variant",
            "bogus",
            "--results-dir",
            str(tmp_path),
        ],
    )
    assert result.exit_code == 2, result.output
    assert "unknown --variant" in result.output


def test_cli_run_rejects_missing_marker_map(tmp_path: Path) -> None:
    missing = tmp_path / "nope.yaml"
    result = CliRunner().invoke(
        main,
        [
            "benchmark",
            "run",
            "--mode",
            "visual",
            "--route",
            "drift-recovery",
            "--marker-map",
            str(missing),
            "--results-dir",
            str(tmp_path),
        ],
    )
    assert result.exit_code == 2, result.output
    assert "marker map not found" in result.output
