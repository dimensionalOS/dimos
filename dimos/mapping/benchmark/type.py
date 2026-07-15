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
    ate_proxy_rmse_m: float | None
    ate_proxy_n_matched: int
    marker_log_events: int
    lidar_log_events: int
    notes: str = ""
    log_path: str = ""

    # Start/end referee fields: a marker deliberately withheld from every
    # localization map under test, scored against real ground truth instead
    # of the ATE-proxy above (which only measures odom-vs-corrected
    # self-consistency, no external reference). Ported from
    # trial/scripts/bench.py's holdout-referee metrics (_median_pose /
    # _quat_angle_deg / _holdout_referee_metrics), named start_end_* here
    # since this typed field is the port's lasting home -- trial/scripts'
    # internals (metrics_logger.py's --holdout-tag, RunRecord.marker_id) keep
    # the historical "holdout" name. tag_id/closure_error_* are nullable:
    # None when the run didn't set a start/end referee tag; readings_* stay 0
    # in that case (a real count of zero, not "unknown").
    start_end_tag_id: int | None = None
    start_end_closure_error_m: float | None = None
    start_end_closure_error_deg: float | None = None
    start_end_readings_start: int = 0
    start_end_readings_end: int = 0


def _fmt_loop_closure(r: RunMetrics) -> str:
    o, c = r.loop_closure_error_odom_m, r.loop_closure_error_corrected_m
    if o is not None and c is not None:
        return f"{o:.3f}m -> {c:.3f}m"
    if c is not None:
        return f"{c:.3f}m (corrected)"
    if o is not None:
        return f"{o:.3f}m (odom)"
    return "n/a"


def _fmt_ate_proxy(r: RunMetrics) -> str:
    if r.ate_proxy_rmse_m is None:
        return "n/a"
    return f"{r.ate_proxy_rmse_m:.3f}m (n={r.ate_proxy_n_matched})"


def _fmt_start_end(r: RunMetrics) -> str:
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
        table.add_column("Start/End closure", justify="right")
        table.add_column("ATE-proxy", justify="right", style="green")
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
                _fmt_ate_proxy(r),
                str(r.corrections_accepted),
                str(r.corrections_rejected),
                str(r.corrections_held),
                coverage,
            )

        console.print()
        console.print(table)
