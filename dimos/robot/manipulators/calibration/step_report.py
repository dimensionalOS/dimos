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

"""Offline step-response report from a ``dual_openyam_step_test`` recording.

    python -m dimos.robot.manipulators.calibration.step_report recordings/<run-id>

Reads ``memory.db``, cuts the joint stream at each ``step_events`` marker and
writes ``step_report/`` next to it: ``steps.csv``, ``report.md``, one PNG per
joint (a panel per gain set, the steps overlaid) and two summary PNGs (settle
time and overshoot against kp, a panel per joint).
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Callable
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any

import matplotlib
import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from dimos.robot.manipulators.calibration.progress import ProgressBar
from dimos.robot.manipulators.calibration.step_analysis import (
    OVERSHOOT_LIMIT_PCT,
    SETTLE_TIME_LIMIT_S,
    StepEvent,
    StepMetrics,
    analyze_step,
    format_table,
    gain_label,
    segment_steps,
)
from dimos.robot.manipulators.dual_openyam.joints import DUAL_OPENYAM_ARM_JOINTS

matplotlib.use("Agg")
import matplotlib.pyplot as plt

JOINT_STATE_STREAM = "joint_state"
JOINT_COMMAND_STREAM = "joint_command"
STEP_EVENTS_STREAM = "step_events"
REPORT_DIR_NAME = "step_report"

# Fixed categorical order for the four sweep steps; never cycled.
_STEP_COLORS = {1.0: "#2a78d6", -1.0: "#eb6834", 3.0: "#1baf7a", -3.0: "#eda100"}
_EXTRA_COLOR = "#e87ba4"
_INK = "#52514e"
_GRID = "#e5e4df"
_SURFACE = "#fcfcfb"


@dataclass(frozen=True)
class JointTrace:
    """Time series of one joint in degrees and degrees per second."""

    t: np.ndarray
    q_deg: np.ndarray
    dq_deg_s: np.ndarray


@dataclass(frozen=True)
class Recording:
    traces: dict[str, JointTrace]
    events: list[StepEvent]
    t_end: float


@dataclass(frozen=True)
class StepRecord:
    event: StepEvent
    t: np.ndarray  # seconds since the step command
    q_deg: np.ndarray  # measured, relative to the target before the step
    target_deg: np.ndarray  # commanded, relative to the target before the step
    metrics: StepMetrics


def load_recording(recording_dir: Path) -> Recording:
    """Read the joint and event streams of a step-test recording."""
    path = recording_dir / "memory.db"
    if not path.exists():
        raise FileNotFoundError(f"no memory.db in {recording_dir}")
    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    try:
        names = set(store.list_streams())
        missing = {JOINT_STATE_STREAM, STEP_EVENTS_STREAM} - names
        if missing:
            raise ValueError(f"{path} lacks streams {sorted(missing)}; has {sorted(names)}")
        stamps: list[float] = []
        positions: list[list[float]] = []
        velocities: list[list[float]] = []
        joint_names: list[str] | None = None
        for obs in store.stream(JOINT_STATE_STREAM):
            msg = obs.data
            if joint_names is None:
                joint_names = list(msg.name)
            stamps.append(float(msg.ts or obs.ts))
            positions.append(list(msg.position))
            velocities.append(list(msg.velocity))
        if joint_names is None:
            raise ValueError(f"{JOINT_STATE_STREAM} is empty")
        t = np.asarray(stamps)
        order = np.argsort(t, kind="stable")
        q = np.degrees(np.asarray(positions))[order]
        dq = np.degrees(np.asarray(velocities))[order]
        t = t[order]
        traces = {
            name: JointTrace(t=t, q_deg=q[:, i], dq_deg_s=dq[:, i])
            for i, name in enumerate(joint_names)
            if name in DUAL_OPENYAM_ARM_JOINTS
        }
        events = [StepEvent.from_json(obs.data.data) for obs in store.stream(STEP_EVENTS_STREAM)]
    finally:
        store.stop()
    return Recording(traces=traces, events=events, t_end=float(t[-1]))


def extract_steps(
    recording: Recording, on_step: Callable[[StepEvent], None] | None = None
) -> list[StepRecord]:
    """One record per step marker, measured from the sample before the command."""
    records: list[StepRecord] = []
    for event, start, stop in segment_steps(recording.events, recording.t_end):
        if on_step is not None:
            on_step(event)
        trace = recording.traces.get(event.joint)
        if trace is None:
            continue
        mask = (trace.t >= start) & (trace.t <= stop)
        if not mask.any():
            continue
        before = np.flatnonzero(trace.t < start)
        y0 = float(trace.q_deg[before[-1]]) if before.size else event.target_before_deg
        t = trace.t[mask] - start
        q = trace.q_deg[mask]
        metrics = analyze_step(event, t, q, trace.dq_deg_s[mask], y0_deg=y0)
        records.append(
            StepRecord(
                event=event,
                t=t,
                q_deg=q - event.target_before_deg,
                target_deg=np.full(t.shape, event.target_after_deg - event.target_before_deg),
                metrics=metrics,
            )
        )
    return records


def write_tables(records: list[StepRecord], out_dir: Path) -> tuple[Path, Path]:
    metrics = [record.metrics for record in records]
    csv_path = out_dir / "steps.csv"
    rows = [m.as_row() for m in metrics]
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["joint"])
        writer.writeheader()
        writer.writerows(rows)
    md_path = out_dir / "report.md"
    md_path.write_text(
        "# Dual OpenYAM joint step response\n\n"
        f"{len(records)} steps over {len({m.joint for m in metrics})} joints.\n\n"
        + format_table(metrics)
        + "\n"
    )
    return csv_path, md_path


def _by_joint_and_gains(
    records: list[StepRecord],
) -> dict[str, dict[tuple[float, float], list[StepRecord]]]:
    grouped: dict[str, dict[tuple[float, float], list[StepRecord]]] = defaultdict(
        lambda: defaultdict(list)
    )
    for record in records:
        grouped[record.event.joint][(record.event.kp, record.event.kd)].append(record)
    return grouped


def _style(ax: Any) -> None:
    ax.set_facecolor(_SURFACE)
    ax.grid(True, color=_GRID, linewidth=0.6)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(_GRID)
    ax.tick_params(colors=_INK, labelsize=8)


def _step_color(delta_deg: float) -> str:
    return _STEP_COLORS.get(float(delta_deg), _EXTRA_COLOR)


def _step_handles(deltas: set[float], marker: str = "") -> list[Any]:
    """Legend entries in the fixed step order, one per step size seen."""
    return [
        plt.Line2D(
            [],
            [],
            marker=marker or None,
            linestyle="" if marker else "-",
            linewidth=2.0,
            color=_step_color(d),
            label=f"{d:+g} deg",
        )
        for d in sorted(deltas, key=lambda d: (abs(d), -d))
    ]


def plot_joint(
    joint: str, by_gains: dict[tuple[float, float], list[StepRecord]], path: Path
) -> None:
    gains = sorted(by_gains)
    fig, axes = plt.subplots(
        1, len(gains), figsize=(max(7.0, 4.2 * len(gains)), 3.8), sharey=True, squeeze=False
    )
    fig.patch.set_facecolor(_SURFACE)
    deltas: set[float] = set()
    for ax, key in zip(axes[0], gains, strict=True):
        _style(ax)
        for record in sorted(
            by_gains[key], key=lambda r: (-abs(r.event.delta_deg), r.event.delta_deg)
        ):
            color = _step_color(record.event.delta_deg)
            deltas.add(float(record.event.delta_deg))
            ax.plot(record.t, record.target_deg, color=color, linewidth=1.0, linestyle="--")
            ax.plot(record.t, record.q_deg, color=color, linewidth=2.0)
        ax.axvline(SETTLE_TIME_LIMIT_S, color=_INK, linewidth=0.8, linestyle=":")
        ax.set_title(gain_label(*key), fontsize=9, color=_INK)
        ax.set_xlabel("time from step (s)", fontsize=8, color=_INK)
    axes[0][0].set_ylabel("angle from start (deg)", fontsize=8, color=_INK)
    fig.legend(
        handles=_step_handles(deltas), loc="upper right", fontsize=8, frameon=False, title="step"
    )
    fig.suptitle(
        f"{joint}: measured (solid) vs target (dashed)",
        fontsize=10,
        color=_INK,
        x=0.02,
        ha="left",
    )
    fig.tight_layout(rect=(0, 0, 0.92, 0.95))
    fig.savefig(path, dpi=120)
    plt.close(fig)


def plot_summary(
    records: list[StepRecord], path: Path, *, metric: str, label: str, limit: float
) -> None:
    joints = [j for j in DUAL_OPENYAM_ARM_JOINTS if any(r.event.joint == j for r in records)]
    if not joints:
        return
    cols = min(6, len(joints))
    rows = math.ceil(len(joints) / cols)
    fig, axes = plt.subplots(
        rows, cols, figsize=(max(7.0, 3.0 * cols), 2.6 * rows + 0.5), squeeze=False
    )
    fig.patch.set_facecolor(_SURFACE)
    flat = axes.ravel()
    seen_deltas: set[float] = set()
    for ax, joint in zip(flat, joints, strict=False):
        _style(ax)
        ax.axhline(limit, color=_INK, linewidth=0.8, linestyle=":")
        for record in records:
            if record.event.joint != joint:
                continue
            value = getattr(record.metrics, metric)
            if value is None:
                continue
            seen_deltas.add(float(record.event.delta_deg))
            ax.scatter(
                record.event.kp,
                value,
                s=28,
                color=_step_color(record.event.delta_deg),
                edgecolors=_SURFACE,
                linewidths=1.0,
                zorder=3,
            )
        ax.set_title(joint, fontsize=9, color=_INK)
        ax.set_xlabel("kp", fontsize=8, color=_INK)
    for ax in flat[len(joints) :]:
        ax.set_visible(False)
    axes[0][0].set_ylabel(label, fontsize=8, color=_INK)
    fig.legend(
        handles=_step_handles(seen_deltas, marker="o"),
        loc="upper right",
        fontsize=8,
        frameon=False,
        title="step",
    )
    fig.suptitle(f"{label} vs kp (dotted: pass limit)", fontsize=10, color=_INK, x=0.02, ha="left")
    fig.tight_layout(rect=(0, 0, 0.92, 0.95))
    fig.savefig(path, dpi=120)
    plt.close(fig)


def generate_report(recording_dir: Path) -> Path:
    """Analyse a recording directory and write ``step_report/`` inside it."""
    print(f"reading {recording_dir / 'memory.db'} ...", flush=True)
    recording = load_recording(recording_dir)
    joints = sorted({event.joint for event in recording.events})
    bar = ProgressBar(len(recording.events) + len(joints) + 2, "step report")
    records = extract_steps(
        recording,
        on_step=lambda event: bar.update(text=f"analyse {event.joint} {event.delta_deg:+g} deg"),
    )
    if not records:
        bar.close()
        raise ValueError(f"no steps found in {recording_dir}")
    out_dir = recording_dir / REPORT_DIR_NAME
    out_dir.mkdir(exist_ok=True)
    write_tables(records, out_dir)
    for joint, by_gains in _by_joint_and_gains(records).items():
        bar.update(text=f"plot {joint}")
        plot_joint(joint, by_gains, out_dir / f"{joint}.png")
    bar.update(text="plot settle time summary")
    plot_summary(
        records,
        out_dir / "summary_settle_time.png",
        metric="settle_time_s",
        label="settle time (s)",
        limit=SETTLE_TIME_LIMIT_S,
    )
    bar.update(text="plot overshoot summary")
    plot_summary(
        records,
        out_dir / "summary_overshoot.png",
        metric="overshoot_pct",
        label="overshoot (%)",
        limit=OVERSHOOT_LIMIT_PCT,
    )
    bar.close()
    return out_dir


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        print(__doc__, file=sys.stderr)
        return 2
    out_dir = generate_report(Path(args[0]))
    print((out_dir / "report.md").read_text())
    print(f"Wrote {out_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
