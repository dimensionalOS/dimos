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

"""Implementation of the `dimos benchmark` subcommand (run + report).

Attaches to an already-running DimOS stack (same TF/camera transports every
process publishes on -- picked automatically from global_config, same as
whatever `--transport` the target run used) and records a marker/lidar
localization benchmark run, exactly like the trial-week tooling did from
outside the repo:

  - TF firehose + correction logging: ported from
    `../../../trial/scripts/metrics_logger.py`'s MetricsLogger
    (`_on_transform` / `run` / `close`).
  - Start/end tag runtime adoption: ported from
    `../../../trial/scripts/holdout_overlay.py`'s OverlayApp
    (`_update_adoption` -- stable = seen in >=8 of the last 10 frames; a
    map, if given via --marker-map, excludes its ids from candidacy; with
    no map, any stable tag is a candidate). Unlike the interactive overlay
    (which keeps re-evaluating candidates until a human clicks START), a
    headless run has no "waiting" screen to keep showing -- see
    `TagAdopter` below for why this port locks on first adoption instead.
  - Scoring: `dimos.mapping.benchmark.tool_benchmark.compute_run_metrics`
    (itself ported from `trial/scripts/report.py` + `bench.py`), CSV
    append + RESULTS.md regeneration mirroring `bench.py`'s own
    append_csv_row/regenerate_report, renamed to this tool's own
    `--results-dir` (default `./benchmark_results`) instead of the trial
    week's `trial/results/`.

`run_benchmark`/`report_benchmark` are the two verbs `dimos.py` wires up
(the `dimos benchmark run|report` subcommands) -- kept typer-agnostic where
practical (see `normalize_mode`) so they're unit-testable without a robot,
but CLI-boundary argument errors (bad --mode, missing --marker-map file)
raise `typer.Exit` directly, the same convention
`dimos/learning/dataprep/cli.py`'s `build()`/`inspect()` use.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Mapping
import csv
from dataclasses import asdict, fields
from datetime import datetime
import json
import math
from pathlib import Path
import time
from typing import TYPE_CHECKING

import cv2
import cv2.aruco
import numpy as np
import typer

from dimos.mapping.benchmark.tool_benchmark import compute_run_metrics, load_jsonl
from dimos.mapping.benchmark.type import (
    RunMetrics,
    _fmt_ate_proxy,
    _fmt_loop_closure,
    _fmt_start_end,
)
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.perception.fiducial.marker_localization import load_marker_map
from dimos.perception.fiducial.marker_pose import (
    camera_info_to_cv_matrices,
    camera_optical_frame_id,
    create_aruco_detector,
    estimate_marker_pose_candidates,
    marker_reprojection_error,
    rvec_tvec_to_transform,
)

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image

# -- modes --------------------------------------------------------------

# "visual" is the canonical name for the marker-localization mode in this
# devtool; "marker" (bench.py/metrics_logger.py's original name) is accepted
# as a hidden alias so trial-week muscle memory / notes still work.
MODES = ("odom", "lidar", "visual")
_MODE_ALIASES = {"marker": "visual"}
_MODE_ORDER = {m: i for i, m in enumerate(MODES)}

_VALID_VARIANTS = ("clean", "kidnap")

# -- start/end tag runtime adoption (ported from holdout_overlay.py) ------

ADOPT_WINDOW_FRAMES = 10  # a tag is "stable" when seen in >=ADOPT_MIN_SEEN of the last N frames
ADOPT_MIN_SEEN = 8

# Same reprojection gate TagTracker.solve applies: a frame whose best-fit pose
# still reprojects badly has bad corners -- treat it as "tag not in view"
# rather than logging a garbage sighting.
REPROJ_GATE_PX = 3.0

DEFAULT_MARKER_LENGTH_M = 0.10  # matches the standard 100mm survey tags
DEFAULT_ARUCO_DICTIONARY = "DICT_APRILTAG_36h11"

DEFAULT_RESULTS_DIR = Path("benchmark_results")
RESULTS_CSV_NAME = "benchmarks.csv"
RESULTS_MD_NAME = "RESULTS.md"

# -- TF logging constants (ported from metrics_logger.py) -----------------

WORLD_FRAME = "world"
MAP_FRAME = "map"
BASE_FRAME = "base_link"

# Below this translation jump, a fresh world->map publish is a periodic
# "hold" republish (RelocalizationModule-style) rather than a new correction.
CORRECTION_EPS_M = 0.01

# How far back the map->base_link lookup may search for a matching world->map
# sample -- MarkerLocalizationModule only republishes on a fresh accept, not
# on a timer, so a correction can be many seconds old and still current.
CORRECTED_POSE_LOOKUP_TOLERANCE_S = 30.0

CSV_FIELDS = ["timestamp", *(f.name for f in fields(RunMetrics))]


def normalize_mode(mode: str) -> str:
    """Resolve a `--mode` value to one of MODES, honoring the 'marker' alias."""
    resolved = _MODE_ALIASES.get(mode, mode)
    if resolved not in MODES:
        raise ValueError(
            f"unknown benchmark mode {mode!r} -- choose one of {MODES} (or alias 'marker')"
        )
    return resolved


def _slug(s: str) -> str:
    return "".join(c if c.isalnum() or c in "-_" else "-" for c in s.strip().lower())


def make_run_id(route: str, variant: str | None, mode: str) -> str:
    parts = [_slug(route)]
    if variant:
        parts.append(_slug(variant))
    return f"{'-'.join(parts)}__{mode}__{time.strftime('%Y%m%d-%H%M%S')}"


def effective_route(route: str, variant: str | None) -> str:
    """Fold --variant into the scored route name (no RunMetrics schema change
    needed) so a clean run and a kidnap run of the same route stay distinct
    rows/head-to-head groups instead of one overwriting the other."""
    return f"{route}-{variant}" if variant else route


# -- start/end tag runtime adoption ---------------------------------------


class TagAdopter:
    """Ported from `trial/scripts/holdout_overlay.py`'s
    `OverlayApp._update_adoption`: adopts the most prominent (largest image
    area) tag stably visible (seen in >=ADOPT_MIN_SEEN of the last
    ADOPT_WINDOW_FRAMES frames) whose id is NOT in `map_ids`. With no map
    (`map_ids` empty -- the odom/lidar-mode, no --marker-map case) any stable
    tag is a candidate.

    Unlike the interactive overlay -- which keeps re-evaluating candidates
    for as long as a human is looking at the WAITING screen, and only freezes
    the id once they click START -- a headless run has no such screen: it
    starts recording the instant it attaches. So this port LOCKS on the first
    successful adoption instead of continuing to re-evaluate for the whole
    run; flipping the adopted id mid-run would silently compare two different
    physical tags across the start/end window, which is worse than adopting
    slightly early.
    """

    def __init__(self, map_ids: set[int]) -> None:
        self.map_ids = map_ids
        self._seen_window: deque[dict[int, float]] = deque(maxlen=ADOPT_WINDOW_FRAMES)
        self.adopted_id: int | None = None
        self.mapped_only = False
        self.locked = False

    def update(self, detections: Mapping[int, np.ndarray]) -> int | None:
        """Feed one frame's detections (id -> (4,2) corners). Returns the
        adopted id the frame it locks in, else None."""
        if self.locked:
            return self.adopted_id

        self._seen_window.append(
            {mid: float(cv2.contourArea(cs)) for mid, cs in detections.items()}
        )
        counts: dict[int, int] = {}
        for frame_seen in self._seen_window:
            for mid in frame_seen:
                counts[mid] = counts.get(mid, 0) + 1
        stable = {mid for mid, n in counts.items() if n >= ADOPT_MIN_SEEN}
        candidates = stable - self.map_ids
        self.mapped_only = bool(stable) and not candidates
        if not candidates:
            return None

        latest_area = self._seen_window[-1]
        self.adopted_id = max(candidates, key=lambda mid: latest_area.get(mid, 0.0))
        self.locked = True
        return self.adopted_id


def _t2v(t: Transform) -> tuple[float, float, float]:
    return (t.translation.x, t.translation.y, t.translation.z)


def _pose_dist(a: Transform, b: Transform) -> float:
    return math.dist(_t2v(a), _t2v(b))


def _transform_record(t: Transform, kind: str, **extra: object) -> dict[str, object]:
    """Ported from metrics_logger.py's `transform_record()`."""
    return {
        "type": kind,
        "ts": t.ts,
        "logged_at": time.time(),
        "frame_id": t.frame_id,
        "child_frame_id": t.child_frame_id,
        "translation": list(_t2v(t)),
        "rotation": [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w],
        **extra,
    }


def _solve_tag_pose(
    corners_2d: np.ndarray,
    tag_id: int,
    ts: float,
    *,
    cam_mtx: np.ndarray,
    dist: np.ndarray,
    distortion_model: str | None,
    marker_length_m: float,
    optical_frame: str,
    reproj_gate_px: float = REPROJ_GATE_PX,
) -> tuple[Transform, float] | None:
    """tag_T_camera + reprojection error (px) for one detected tag corner
    set, or None if the fit fails the gate.

    Ported from `holdout_overlay.py`'s `TagTracker.solve`: tries every IPPE
    candidate and keeps the min-reprojection one (the same planar
    mirror-pose-ambiguity disambiguation MarkerLocalizationModule itself
    applies) rather than trusting `estimate_marker_pose`'s single best guess,
    which can pick the flipped pose near the ambiguity.
    """
    candidates = estimate_marker_pose_candidates(
        corners_2d, marker_length_m, cam_mtx, dist, distortion_model=distortion_model
    )
    best: tuple[np.ndarray, np.ndarray] | None = None
    best_err = math.inf
    for rvec, tvec in candidates:
        err = marker_reprojection_error(
            corners_2d,
            marker_length_m,
            cam_mtx,
            dist,
            rvec,
            tvec,
            distortion_model=distortion_model,
        )
        if err < best_err:
            best, best_err = (rvec, tvec), err
    if best is None or best_err > reproj_gate_px:
        return None

    rvec, tvec = best
    optical_t_marker = rvec_tvec_to_transform(
        rvec, tvec, frame_id=optical_frame, child_frame_id=f"marker_{tag_id}", ts=ts
    )
    return optical_t_marker.inverse(), float(best_err)


# -- benchmark logger (ported from metrics_logger.py) ---------------------


class BenchmarkLogger:
    """Ported from `../../../trial/scripts/metrics_logger.py`'s
    MetricsLogger, merged with `holdout_overlay.py`'s runtime start/end tag
    adoption (`TagAdopter`) so a pre-agreed --start-end-tag id isn't
    required: the first stable unmapped (or, with no --marker-map, any
    stable) tag seen after attach is adopted, printed, and locked for the
    run.

    Does NOT run any detection/localization itself and does NOT touch the
    module under test -- a pure observer, attachable to any blueprint after
    the fact, same as the trial-week tool it ports.
    """

    def __init__(
        self,
        out_path: str | Path,
        *,
        map_ids: set[int],
        override_tag: int | None,
        marker_length_m: float = DEFAULT_MARKER_LENGTH_M,
        aruco_dictionary: str = DEFAULT_ARUCO_DICTIONARY,
    ) -> None:
        # Deferred (not module-level) so importing this module -- e.g. for
        # TagAdopter/CSV unit tests -- never needs a transport config, same
        # rationale holdout_overlay.py's own run_live() gives for the
        # identical imports.
        from dimos.core.transport_factory import make_transport, tf_backend
        from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
        from dimos.msgs.sensor_msgs.Image import Image

        self.out_path = Path(out_path)
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        self._out_f = self.out_path.open("w")
        import threading

        self._write_lock = threading.Lock()
        self._stop_flag = False

        self._last_correction: Transform | None = None
        self.n_odom_ticks = 0
        self.n_corrected_ticks = 0
        self.n_corrections_new = 0
        self.n_corrections_hold = 0
        self.n_log_events = 0
        self.n_tag_sightings = 0

        tf_class = tf_backend()
        outer = self

        class _LoggingTF(tf_class):  # type: ignore[misc, valid-type]
            def receive_transform(self, *args: Transform) -> None:
                for t in args:
                    outer._on_transform(t)
                super().receive_transform(*args)

        self.tf = _LoggingTF()

        self.override_tag = override_tag
        self.adopter: TagAdopter | None = TagAdopter(map_ids) if override_tag is None else None
        self.active_tag_id: int | None = override_tag
        self.marker_length_m = marker_length_m
        self._detector: cv2.aruco.ArucoDetector = create_aruco_detector(aruco_dictionary)
        self._cam_mtx: np.ndarray | None = None
        self._dist: np.ndarray | None = None
        self._camera_info: CameraInfo | None = None

        self._dimos_log_path: Path | None = None

        camera_info_t = make_transport("camera_info", CameraInfo)
        camera_info_t.subscribe(self._on_camera_info)
        color_image_t = make_transport("color_image", Image)
        color_image_t.subscribe(self._on_color_image)
        self._vision_transports = [camera_info_t, color_image_t]

    def attach_dimos_log(self, path: Path) -> None:
        """Tail the target run's main.jsonl for gate-rejected/relocalize log
        lines, same opt-in metrics_logger.py's --dimos-log provides -- here
        auto-discovered by `run_benchmark` via the run registry instead of a
        manual flag, since this tool already attaches to a *running* stack."""
        self._dimos_log_path = path

    # -- TF firehose (ported from metrics_logger.py's _on_transform) ------
    def _write(self, record: Mapping[str, object]) -> None:
        with self._write_lock:
            self._out_f.write(json.dumps(record) + "\n")
            self._out_f.flush()

    def _on_transform(self, t: Transform) -> None:
        if t.frame_id == WORLD_FRAME and t.child_frame_id == BASE_FRAME:
            self.n_odom_ticks += 1
            self._write(_transform_record(t, "odom_pose"))
            if self._last_correction is not None:
                corrected = self.tf.get(
                    MAP_FRAME,
                    BASE_FRAME,
                    time_point=t.ts,
                    time_tolerance=CORRECTED_POSE_LOOKUP_TOLERANCE_S,
                )
                if corrected is not None:
                    self.n_corrected_ticks += 1
                    self._write(_transform_record(corrected, "corrected_pose"))
        elif t.frame_id == WORLD_FRAME and t.child_frame_id == MAP_FRAME:
            magnitude = _pose_dist(t, self._last_correction) if self._last_correction else None
            is_new = magnitude is None or magnitude > CORRECTION_EPS_M
            if is_new:
                self.n_corrections_new += 1
            else:
                self.n_corrections_hold += 1
            self._write(
                _transform_record(
                    t, "correction_new" if is_new else "correction_hold", magnitude_m=magnitude
                )
            )
            self._last_correction = t

    # -- start/end tag: adoption + sighting (ported from holdout_overlay.py
    # TagTracker/OverlayApp + metrics_logger.py's holdout referee) --------
    def _on_camera_info(self, info: CameraInfo) -> None:
        self._camera_info = info
        self._cam_mtx, self._dist = camera_info_to_cv_matrices(info)

    def _on_color_image(self, image: Image) -> None:
        if self._camera_info is None or self._cam_mtx is None or self._dist is None:
            return
        info = self._camera_info
        if (
            info.width
            and info.height
            and (image.width != info.width or image.height != info.height)
        ):
            return

        gray = image.to_grayscale().as_numpy()
        corners, ids, _rejected = self._detector.detectMarkers(gray)
        detections: dict[int, np.ndarray] = (
            {}
            if ids is None
            else {
                int(mid[0]): cs.reshape(4, 2).astype(np.float32)
                for cs, mid in zip(corners, ids, strict=True)
            }
        )

        if self.adopter is not None and not self.adopter.locked:
            adopted = self.adopter.update(detections)
            if adopted is not None:
                self.active_tag_id = adopted
                print(
                    f"benchmark: adopted start/end tag ID {adopted} "
                    f"(stable {ADOPT_MIN_SEEN}/{ADOPT_WINDOW_FRAMES} frames)"
                )

        tag_id = self.active_tag_id
        if tag_id is None or tag_id not in detections:
            return

        optical_frame = camera_optical_frame_id(image, info)
        solved = _solve_tag_pose(
            detections[tag_id],
            tag_id,
            image.ts,
            cam_mtx=self._cam_mtx,
            dist=self._dist,
            distortion_model=info.distortion_model,
            marker_length_m=self.marker_length_m,
            optical_frame=optical_frame,
        )
        if solved is None:
            return
        marker_t_camera, reproj = solved
        self.n_tag_sightings += 1
        self._write(
            _transform_record(
                marker_t_camera,
                "tag_sighting",
                marker_id=tag_id,
                reprojection_error_px=round(reproj, 4),
            )
        )

    # -- dimos structured-log tail (ported from metrics_logger.py) --------
    _WATCH_SUBSTRINGS = ("gate rejected", "relocalize", "MarkerLocalizationModule")

    def _tail_dimos_log(self) -> None:
        from dimos.core.log_viewer import follow_log

        path = self._dimos_log_path
        if path is None:
            return
        if not path.exists():
            print(f"benchmark: dimos log {path} not found yet, waiting...")
            deadline = time.time() + 60.0
            while not path.exists() and time.time() < deadline and not self._stop_flag:
                time.sleep(0.5)
            if not path.exists():
                print(f"benchmark: gave up waiting for {path}")
                return
        for line in follow_log(path, stop=lambda: self._stop_flag):
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            event = str(rec.get("event", ""))
            logger_name = str(rec.get("logger", ""))
            if any(s in event or s in logger_name for s in self._WATCH_SUBSTRINGS):
                self.n_log_events += 1
                self._write(
                    {
                        "type": "log_event",
                        "ts": rec.get("timestamp"),
                        "logged_at": time.time(),
                        "level": rec.get("level"),
                        "logger": logger_name,
                        "event": event,
                    }
                )

    # -- lifecycle (ported from metrics_logger.py) ------------------------
    def run(self, duration: float | None = None) -> None:
        import threading

        tail_thread: threading.Thread | None = None
        if self._dimos_log_path is not None:
            tail_thread = threading.Thread(target=self._tail_dimos_log, daemon=True)
            tail_thread.start()
        start = time.monotonic()
        try:
            while duration is None or (time.monotonic() - start) < duration:
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("benchmark: Ctrl+C, stopping...")
        finally:
            self._stop_flag = True
            if tail_thread is not None:
                tail_thread.join(timeout=2.0)
            self.close()

    def close(self) -> None:
        self._out_f.close()
        try:
            self.tf.stop()
        except Exception:
            pass
        for t in self._vision_transports:
            try:
                t.stop()
            except Exception:
                pass


# -- CSV / RESULTS.md (mirrors bench.py's append_csv_row/render_results_md,
# reusing type.py's RunMetrics-based formatters instead of re-deriving
# CSV-string formatting) ---------------------------------------------------


def _opt_float(row: Mapping[str, str], key: str) -> float | None:
    v = row.get(key)
    return None if v in (None, "", "None") else float(v)  # type: ignore[arg-type]


def _opt_int(row: Mapping[str, str], key: str) -> int | None:
    v = row.get(key)
    return None if v in (None, "", "None") else int(float(v))  # type: ignore[arg-type]


def _int(row: Mapping[str, str], key: str, default: int = 0) -> int:
    v = row.get(key)
    return default if v in (None, "") else int(float(v))  # type: ignore[arg-type]


def _row_to_metrics(row: Mapping[str, str]) -> RunMetrics:
    """Reconstruct a typed RunMetrics from one CSV-parsed (all-string) row."""
    return RunMetrics(
        run_id=row.get("run_id", ""),
        route=row.get("route", ""),
        mode=row.get("mode", ""),
        duration_s=float(row.get("duration_s") or 0.0),
        odom_ticks=_int(row, "odom_ticks"),
        corrected_ticks=_int(row, "corrected_ticks"),
        corrected_pose_coverage=_opt_float(row, "corrected_pose_coverage"),
        corrections_accepted=_int(row, "corrections_accepted"),
        corrections_held=_int(row, "corrections_held"),
        corrections_rejected=_int(row, "corrections_rejected"),
        correction_mag_mean_m=_opt_float(row, "correction_mag_mean_m"),
        correction_mag_max_m=_opt_float(row, "correction_mag_max_m"),
        loop_closure_error_odom_m=_opt_float(row, "loop_closure_error_odom_m"),
        loop_closure_error_corrected_m=_opt_float(row, "loop_closure_error_corrected_m"),
        ate_proxy_rmse_m=_opt_float(row, "ate_proxy_rmse_m"),
        ate_proxy_n_matched=_int(row, "ate_proxy_n_matched"),
        marker_log_events=_int(row, "marker_log_events"),
        lidar_log_events=_int(row, "lidar_log_events"),
        notes=row.get("notes") or "",
        log_path=row.get("log_path") or "",
        start_end_tag_id=_opt_int(row, "start_end_tag_id"),
        start_end_closure_error_m=_opt_float(row, "start_end_closure_error_m"),
        start_end_closure_error_deg=_opt_float(row, "start_end_closure_error_deg"),
        start_end_readings_start=_int(row, "start_end_readings_start"),
        start_end_readings_end=_int(row, "start_end_readings_end"),
    )


def append_csv_row(results_dir: Path, metrics: RunMetrics) -> None:
    results_dir.mkdir(parents=True, exist_ok=True)
    csv_path = results_dir / RESULTS_CSV_NAME
    row = {"timestamp": datetime.now().isoformat(timespec="seconds"), **asdict(metrics)}
    is_new = not csv_path.exists() or csv_path.stat().st_size == 0
    with csv_path.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if is_new:
            w.writeheader()
        w.writerow(row)


def render_results_md(entries: list[tuple[str, RunMetrics]], csv_path: Path) -> str:
    lines = [
        "# Benchmark results",
        "",
        f"Generated by `dimos benchmark report` from `{csv_path}` -- {len(entries)} run(s).",
        "",
        "## All runs",
        "",
        "| Route | Mode | Loop-closure error | Start/End closure | ATE-proxy | Corrections | "
        "Notes | Timestamp |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for ts, m in sorted(entries, key=lambda e: e[0], reverse=True):
        corr = f"{m.corrections_accepted} acc / {m.corrections_rejected} rej / {m.corrections_held} held"
        notes = (m.notes or "").replace("|", "/") or "--"
        lines.append(
            f"| {m.route} | {m.mode} | {_fmt_loop_closure(m)} | {_fmt_start_end(m)} | "
            f"{_fmt_ate_proxy(m)} | {corr} | {notes} | {ts} |"
        )
    lines.append("")

    tags = sorted({m.start_end_tag_id for _, m in entries if m.start_end_tag_id is not None})
    for tag in tags:
        lines.append(
            f"*start/end tag referee: tag {tag}, excluded from every localization map under test.*"
        )
    if tags:
        lines.append("")

    by_route: dict[str, list[tuple[str, RunMetrics]]] = {}
    for ts, m in entries:
        by_route.setdefault(m.route, []).append((ts, m))
    h2h = {r: es for r, es in by_route.items() if len({m.mode for _, m in es}) >= 2}
    if h2h:
        lines += ["## Head-to-head (by route)", ""]
        for route in sorted(h2h):
            latest: dict[str, tuple[str, RunMetrics]] = {}
            for ts, m in sorted(h2h[route], key=lambda e: e[0]):
                latest[m.mode] = (ts, m)
            lines.append(f"### {route}")
            lines.append("")
            lines.append(
                "| Mode | Loop-closure error | Start/End closure | ATE-proxy | Corrections |"
            )
            lines.append("|---|---|---|---|---|")
            for mode in sorted(latest, key=lambda mo: (_MODE_ORDER.get(mo, len(_MODE_ORDER)), mo)):
                _, rm = latest[mode]
                corr = (
                    f"{rm.corrections_accepted} acc / {rm.corrections_rejected} rej / "
                    f"{rm.corrections_held} held"
                )
                lines.append(
                    f"| {mode} | {_fmt_loop_closure(rm)} | {_fmt_start_end(rm)} | "
                    f"{_fmt_ate_proxy(rm)} | {corr} |"
                )
            lines.append("")

    return "\n".join(lines)


def regenerate_report(results_dir: Path) -> int:
    results_dir.mkdir(parents=True, exist_ok=True)
    csv_path = results_dir / RESULTS_CSV_NAME
    md_path = results_dir / RESULTS_MD_NAME
    if not csv_path.exists():
        md_path.write_text(
            "# Benchmark results\n\nNo runs yet -- `dimos benchmark run ...` then Ctrl+C.\n"
        )
        return 0
    with csv_path.open() as f:
        rows = list(csv.DictReader(f))
    entries = [(row.get("timestamp", ""), _row_to_metrics(row)) for row in rows]
    md_path.write_text(render_results_md(entries, csv_path))
    return len(rows)


# -- verbs (called by dimos.py's thin `dimos benchmark run|report` wiring) --


def run_benchmark(
    *,
    mode: str,
    route: str,
    variant: str | None = None,
    notes: str = "",
    marker_map: Path | None = None,
    start_end_tag: int | None = None,
    results_dir: Path = DEFAULT_RESULTS_DIR,
    marker_length_m: float = DEFAULT_MARKER_LENGTH_M,
    aruco_dictionary: str = DEFAULT_ARUCO_DICTIONARY,
    duration: float | None = None,
) -> RunMetrics:
    """Attach to a running DimOS stack, record a benchmark run, score it, and
    append/regenerate this tool's own CSV + RESULTS.md under `results_dir`.

    Blocks until Ctrl+C (or `duration` elapses). CLI-boundary argument
    errors (bad --mode/--variant, missing --marker-map file) raise
    `typer.Exit(2)` with a message -- same convention
    `dimos/learning/dataprep/cli.py`'s `build()` uses.
    """
    try:
        resolved_mode = normalize_mode(mode)
    except ValueError as e:
        typer.echo(f"benchmark run: {e}", err=True)
        raise typer.Exit(2) from e

    if variant is not None and variant not in _VALID_VARIANTS:
        typer.echo(
            f"benchmark run: unknown --variant {variant!r} -- choose one of {_VALID_VARIANTS}",
            err=True,
        )
        raise typer.Exit(2)

    map_ids: set[int] = set()
    if marker_map is not None:
        if not marker_map.exists():
            typer.echo(f"benchmark run: marker map not found: {marker_map}", err=True)
            raise typer.Exit(2)
        map_ids = set(load_marker_map(marker_map))

    if start_end_tag is not None:
        print(f"benchmark: start/end tag pinned to ID {start_end_tag} (no adoption)")
    elif map_ids:
        print(
            f"benchmark: auto-adopting the start/end tag -- {len(map_ids)} mapped id(s) "
            f"excluded ({sorted(map_ids)}) per {marker_map}"
        )
    else:
        print(
            "benchmark: no --marker-map given -- any stable tag will be adopted as the start/end tag"
        )

    run_id = make_run_id(route, variant, resolved_mode)
    log_path = results_dir / "runs" / f"{run_id}.jsonl"

    logger = BenchmarkLogger(
        log_path,
        map_ids=map_ids,
        override_tag=start_end_tag,
        marker_length_m=marker_length_m,
        aruco_dictionary=aruco_dictionary,
    )

    from dimos.core.run_registry import get_most_recent

    entry = get_most_recent(alive_only=True)
    if entry is not None:
        dimos_log_path = Path(entry.log_dir) / "main.jsonl"
        logger.attach_dimos_log(dimos_log_path)
        print(f"benchmark: tailing {dimos_log_path} for gate-reject/relocalize log events")
    else:
        print(
            "benchmark: no running DimOS instance found via the run registry -- TF/tag "
            "logging only (no gate-reject log_event capture); start the stack with "
            "`dimos run ... --daemon` first for the log_event breakdown"
        )

    print(f"benchmark: run_id={run_id} mode={resolved_mode} route={route} -> {log_path}")
    print("benchmark: Ctrl+C to stop, score, and append the row.")

    start_ts = time.time()
    logger.run(duration=duration)
    duration_s = time.time() - start_ts

    print(
        f"benchmark: logger stopped -- odom_ticks={logger.n_odom_ticks} "
        f"corrected_ticks={logger.n_corrected_ticks} tag_sightings={logger.n_tag_sightings} "
        f"log_events={logger.n_log_events}"
    )
    print("benchmark: scoring run...")

    records = load_jsonl(log_path)
    metrics = compute_run_metrics(
        records,
        run_id=run_id,
        route=effective_route(route, variant),
        mode=resolved_mode,
        duration_s=duration_s,
        notes=notes,
        log_path=str(log_path),
        start_end_tag=logger.active_tag_id,
    )
    append_csv_row(results_dir, metrics)
    n_rows = regenerate_report(results_dir)

    print(f"benchmark: appended row to {results_dir / RESULTS_CSV_NAME} ({n_rows} total)")
    print(f"benchmark: regenerated {results_dir / RESULTS_MD_NAME}")
    print(
        f"benchmark: loop-closure {_fmt_loop_closure(metrics)} | ATE-proxy {_fmt_ate_proxy(metrics)} | "
        f"corrections {metrics.corrections_accepted} acc / {metrics.corrections_rejected} rej"
        + (
            f" | start/end closure {_fmt_start_end(metrics)}"
            if metrics.start_end_tag_id is not None
            else ""
        )
    )
    return metrics


def report_benchmark(results_dir: Path = DEFAULT_RESULTS_DIR) -> int:
    """Regenerate RESULTS.md from this tool's own benchmark CSV."""
    n_rows = regenerate_report(results_dir)
    print(
        f"benchmark: regenerated {results_dir / RESULTS_MD_NAME} from {n_rows} row(s) "
        f"in {results_dir / RESULTS_CSV_NAME}"
    )
    return n_rows
