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

"""Typed results for the mapping-localization benchmark devtool.

Mirrors dimos/protocol/pubsub/benchmark/type.py's shape (a dataclass per
computed result + a collection wrapper with a rich-table summary), applied to
marker/lidar localization runs instead of pubsub transports. Field names and
the metrics they hold port the trial/scripts/bench.py CSV row
(CSV_FIELDS + compute_run_metrics) and trial/scripts/report.py's
detection_stats()/compute_ate() output -- see tool_benchmark.py for the
computations themselves.
"""

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any


def _opt_float(v: Any) -> float | None:
    return None if v is None else float(v)


@dataclass(frozen=True)
class RunRecord:
    """One parsed line from a metrics_logger.py run JSONL.

    Mirrors the record shapes ../../../trial/scripts/metrics_logger.py's
    transform_record() and log_event branch write: pose ticks
    (odom_pose/corrected_pose/correction_new/correction_hold) carry
    translation/rotation (+ magnitude_m for the correction_* kinds); log_event
    carries level/logger/event instead; tag_sighting (only logged when a
    start/end referee tag is set -- see RunMetrics.start_end_tag_id) carries
    marker_id/reprojection_error_px alongside translation/rotation, one record
    per frame the withheld tag was seen.
    """

    type: str
    ts: float
    logged_at: float | None = None
    frame_id: str | None = None
    child_frame_id: str | None = None
    translation: tuple[float, float, float] | None = None
    rotation: tuple[float, float, float, float] | None = None
    magnitude_m: float | None = None
    level: str | None = None
    logger: str | None = None
    event: str | None = None
    marker_id: int | None = None
    reprojection_error_px: float | None = None

    @staticmethod
    def from_dict(d: Mapping[str, Any]) -> "RunRecord":
        translation = d.get("translation")
        rotation = d.get("rotation")
        xyz = (
            (float(translation[0]), float(translation[1]), float(translation[2]))
            if translation is not None
            else None
        )
        wxyz = (
            (float(rotation[0]), float(rotation[1]), float(rotation[2]), float(rotation[3]))
            if rotation is not None
            else None
        )
        marker_id = d.get("marker_id")
        return RunRecord(
            type=str(d["type"]),
            ts=float(d["ts"]),
            logged_at=_opt_float(d.get("logged_at")),
            frame_id=d.get("frame_id"),
            child_frame_id=d.get("child_frame_id"),
            translation=xyz,
            rotation=wxyz,
            magnitude_m=_opt_float(d.get("magnitude_m")),
            level=d.get("level"),
            logger=d.get("logger"),
            event=d.get("event"),
            marker_id=int(marker_id) if marker_id is not None else None,
            reprojection_error_px=_opt_float(d.get("reprojection_error_px")),
        )

    @property
    def xyz(self) -> tuple[float, float, float]:
        if self.translation is None:
            raise ValueError(f"RunRecord of type={self.type!r} carries no translation")
        return self.translation


@dataclass
class RunMetrics:
    """Aggregate metrics for one benchmark run, computed from its RunRecords.

    Ports trial/scripts/report.py's detection_stats()/compute_ate() and
    trial/scripts/bench.py's compute_run_metrics() (loop-closure error,
    correction-magnitude stats, marker/lidar log_event source breakdown).

    Naming note (benchmark-metrics.md's ranked honest-metric research, folded
    in here): this rig only has ground truth at discrete start/end-tag
    sightings, never continuous per-frame -- so nothing here is named ATE or
    RPE outright. The **primary** metric is `start_end_closure_error_m/_deg`
    below ("return error vs fiducial GT" in that research's terms); the
    self-consistency fields right below this comment measure odom-vs-
    corrected agreement only, no external reference at all, and are named
    accordingly (not "ate_proxy" -- see the 2026-07-15 rename note there).
    """

    run_id: str
    route: str
    mode: str
    duration_s: float
    odom_ticks: int
    corrected_ticks: int
    corrected_pose_coverage: float | None
    corrections_accepted: int
    corrections_held: int
    corrections_rejected: int
    correction_mag_mean_m: float | None
    correction_mag_max_m: float | None
    loop_closure_error_odom_m: float | None
    loop_closure_error_corrected_m: float | None
    # Renamed 2026-07-15 from ate_proxy_rmse_m/ate_proxy_n_matched: this is
    # RMSE between one run's own raw-odom stream and its own corrected
    # stream -- self-consistency (how much correction moved the estimate),
    # with no external ground truth anywhere in the computation. "ATE" (even
    # hedged as "proxy") implies a ground-truth-referenced error, which this
    # is not; benchmark-metrics.md's research is explicit that a real
    # ATE-RMSE isn't claimable at all from start/end-tag-only GT. The
    # ground-truth-referenced number is start_end_closure_error_m/_deg below.
    self_consistency_rmse_m: float | None = None
    self_consistency_n_matched: int = 0
    marker_log_events: int = 0
    lidar_log_events: int = 0
    notes: str = ""
    log_path: str = ""

    # Start/end referee fields -- the PRIMARY metric (benchmark-metrics.md's
    # ranked research, #1: "return error vs fiducial GT"). A marker
    # deliberately withheld from every localization map under test, scored
    # against real ground truth instead of the self-consistency fields above.
    # Ported from trial/scripts/bench.py's holdout-referee metrics
    # (_median_pose / _quat_angle_deg / _holdout_referee_metrics), named
    # start_end_* here since this typed field is the port's lasting home --
    # trial/scripts' internals (metrics_logger.py's --holdout-tag,
    # RunRecord.marker_id) keep the historical "holdout" name. tag_id/
    # closure_error_* are nullable: None when the run didn't set a start/end
    # referee tag; readings_* stay 0 in that case (a real count of zero, not
    # "unknown"). closure_error_m/_deg report a magnitude-only difference
    # (claimed displacement distance/rotation-angle vs measured), not a full
    # 6-DOF pose delta -- the referee's readings live in the tag's own fixed
    # frame while the claim lives in world/map frame, two frames related by
    # an uncalibrated rotation, so only frame-invariant scalars (a distance,
    # a rotation angle) are honestly comparable across them; see
    # _start_end_referee_metrics's docstring in tool_benchmark.py.
    start_end_tag_id: int | None = None
    start_end_closure_error_m: float | None = None
    start_end_closure_error_deg: float | None = None
    start_end_readings_start: int = 0
    start_end_readings_end: int = 0

    # Added 2026-07-15 (benchmark-metrics.md research items #3/#4): max/RMS
    # position error across every GT-visible instant the start/end tag was
    # seen, not just the first/last window used for the primary return-error
    # above -- see tool_benchmark.py's _checkpoint_position_errors(). Same
    # magnitude-only reasoning as start_end_closure_error_m applies.
    # checkpoint_count is the number of distinct visits to the tag found
    # (clustered by time gap, see CHECKPOINT_GAP_S) -- checkpoint_error_rms_m
    # is only populated when checkpoint_count > 2, matching the research's
    # own call that RMS is redundant with max/return error at N<=2 (i.e. the
    # ordinary case of a tag seen only at drill-start and drill-end).
    max_checkpoint_error_m: float | None = None
    checkpoint_error_rms_m: float | None = None
    checkpoint_count: int = 0

    # Added 2026-07-15 (benchmark-metrics.md research item #5): wall-clock
    # delta from run-start to the first accepted correction, for a `kidnap`
    # variant run only -- see tool_benchmark.py's _recovery_time_s() for the
    # cold-start-displaced assumption this requires (a mid-run carry-and-drop
    # kidnap has no logged instant to measure from and stays None).
    recovery_time_s: float | None = None


def _fmt_loop_closure(r: RunMetrics) -> str:
    o, c = r.loop_closure_error_odom_m, r.loop_closure_error_corrected_m
    if o is not None and c is not None:
        return f"{o:.3f}m -> {c:.3f}m"
    if c is not None:
        return f"{c:.3f}m (corrected)"
    if o is not None:
        return f"{o:.3f}m (odom)"
    return "n/a"


def _fmt_self_consistency(r: RunMetrics) -> str:
    """Renamed from _fmt_ate_proxy (2026-07-15) -- see RunMetrics'
    self_consistency_rmse_m docstring for why "ATE" doesn't belong in this
    label."""
    if r.self_consistency_rmse_m is None:
        return "n/a"
    return f"{r.self_consistency_rmse_m:.3f}m (n={r.self_consistency_n_matched})"


def _fmt_start_end(r: RunMetrics) -> str:
    """Formats the PRIMARY metric -- return error vs fiducial GT."""
    if r.start_end_tag_id is None:
        return "n/a"
    if r.start_end_closure_error_m is None:
        return f"n/a (tag {r.start_end_tag_id})"
    deg = (
        f" / {r.start_end_closure_error_deg:.1f}deg"
        if r.start_end_closure_error_deg is not None
        else ""
    )
    return f"{r.start_end_closure_error_m:.3f}m{deg}"


def _fmt_max_checkpoint(r: RunMetrics) -> str:
    if r.max_checkpoint_error_m is None:
        return "n/a"
    return f"{r.max_checkpoint_error_m:.3f}m (n={r.checkpoint_count})"


def _fmt_checkpoint_rms(r: RunMetrics) -> str:
    if r.checkpoint_error_rms_m is None:
        return "n/a"
    return f"{r.checkpoint_error_rms_m:.3f}m"


def _fmt_recovery_time(r: RunMetrics) -> str:
    if r.recovery_time_s is None:
        return "n/a"
    return f"{r.recovery_time_s:.1f}s"


@dataclass
class BenchmarkResults:
    results: list[RunMetrics] = field(default_factory=list)

    def add(self, result: RunMetrics) -> None:
        self.results.append(result)

    def print_summary(self) -> None:
        if not self.results:
            return

        from rich.console import Console
        from rich.table import Table

        console = Console()

        table = Table(title="Mapping Localization Benchmark Results")
        table.add_column("Route", style="cyan")
        table.add_column("Mode")
        table.add_column("Loop-closure (odom -> corrected)", justify="right")
        table.add_column("Return error (vs fiducial GT)", justify="right", style="bold")
        table.add_column("Max checkpoint err", justify="right")
        table.add_column("Checkpoint RMS", justify="right")
        table.add_column("Recovery (kidnap)", justify="right")
        table.add_column("Self-consistency", justify="right", style="green")
        table.add_column("Accepted", justify="right")
        table.add_column("Rejected", justify="right")
        table.add_column("Held", justify="right")
        table.add_column("Coverage", justify="right")

        for r in sorted(self.results, key=lambda x: (x.route, x.mode)):
            coverage = (
                f"{r.corrected_pose_coverage:.1%}"
                if r.corrected_pose_coverage is not None
                else "n/a"
            )
            table.add_row(
                r.route,
                r.mode,
                _fmt_loop_closure(r),
                _fmt_start_end(r),
                _fmt_max_checkpoint(r),
                _fmt_checkpoint_rms(r),
                _fmt_recovery_time(r),
                _fmt_self_consistency(r),
                str(r.corrections_accepted),
                str(r.corrections_rejected),
                str(r.corrections_held),
                coverage,
            )

        console.print()
        console.print(table)
