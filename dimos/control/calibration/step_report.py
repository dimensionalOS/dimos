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

"""Offline step-response report from a joint step-test recording of any arm.

    python -m dimos.control.calibration.step_report recordings/<run-id>

Reads ``memory.db``, cuts the joint stream at each ``step_events`` marker, from
the first recorded command (``arm_position_command``) that carries the step's
target when there is one, and writes ``step_report/`` next to it:
``steps.csv``, ``report.md`` (with the gains
``propose_gains`` picks per joint), one PNG per joint (a panel per gain set,
the steps overlaid) and two summary PNGs (settle time and overshoot against kp,
a panel per joint). Joints are taken by name from the recorded joint states.
Each joint's friction, which sets its rest limits, comes from its rests at the
first gains it was swept with: half the spread of its largest steps' rests, or
its largest rest error over the steps that moved, if that is more
(``sweep_friction_nm``).
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Callable, Iterable, Mapping
import csv
from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
from typing import Any

import matplotlib
from matplotlib.lines import Line2D
import numpy as np

from dimos.control.calibration.progress import ProgressBar
from dimos.control.calibration.step_analysis import (
    OVERSHOOT_LIMIT_PCT,
    SETTLE_TIME_LIMIT_S,
    StepEvent,
    StepMetrics,
    analyze_step,
    format_proposals,
    format_table,
    gain_label,
    propose_gains,
    segment_steps,
    sweep_friction_nm,
)
from dimos.memory.store.sqlite import SqliteStore

matplotlib.use("Agg")
import matplotlib.pyplot as plt

JOINT_STATE_STREAM = "joint_state"
JOINT_COMMAND_STREAM = "arm_position_command"
STEP_EVENTS_STREAM = "step_events"
REPORT_DIR_NAME = "step_report"
# A recorded command this close to a step's target carries it.
COMMAND_MATCH_RAD = 1e-6

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
class CommandTrace:
    """Commanded positions of one joint over time, in degrees."""

    t: np.ndarray
    q_deg: np.ndarray


@dataclass(frozen=True)
class Recording:
    traces: dict[str, JointTrace]  # by recorded joint name, in joint-state order
    events: list[StepEvent]
    t_end: float
    commands: dict[str, CommandTrace] = field(default_factory=dict)  # by recorded joint name


@dataclass(frozen=True)
class StepRecord:
    event: StepEvent
    t: np.ndarray  # seconds since the step command
    q_deg: np.ndarray  # measured, relative to the target before the step
    target_deg: np.ndarray  # commanded, relative to the target before the step
    metrics: StepMetrics


def load_recording(recording_dir: Path, window: tuple[float, float] | None = None) -> Recording:
    """Read the joint and event streams of a step-test recording (every recorded joint).

    Args:
        recording_dir: The directory holding ``memory.db``.
        window: Only the joint samples and step markers between these Unix
            times, e.g. those of one calibration step in a longer run.
    """
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
        state_stream: Any = store.stream(JOINT_STATE_STREAM)
        if window is not None:
            state_stream = state_stream.time_range(*window)
        for obs in state_stream:
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
        }
        event_stream: Any = store.stream(STEP_EVENTS_STREAM)
        events = [StepEvent.from_json(obs.data.data) for obs in event_stream]
        commands: dict[str, CommandTrace] = {}
        if JOINT_COMMAND_STREAM in names:
            command_stream: Any = store.stream(JOINT_COMMAND_STREAM)
            if window is not None:
                command_stream = command_stream.time_range(*window)
            commands = command_traces(command_stream)
    finally:
        store.stop()
    if window is not None:
        events = [event for event in events if window[0] <= event.t0 <= window[1]]
    return Recording(traces=traces, events=events, t_end=float(t[-1]), commands=commands)


def command_traces(observations: Iterable[Any]) -> dict[str, CommandTrace]:
    """Each joint's recorded commands, in time order, from ``JointState`` observations."""
    times: dict[str, list[float]] = defaultdict(list)
    positions: dict[str, list[float]] = defaultdict(list)
    for obs in observations:
        msg = obs.data
        if len(msg.name) != len(msg.position):
            continue
        stamp = float(msg.ts or obs.ts)
        for name, position in zip(msg.name, msg.position, strict=True):
            times[name].append(stamp)
            positions[name].append(float(position))
    traces: dict[str, CommandTrace] = {}
    for name, stamps in times.items():
        t = np.asarray(stamps)
        order = np.argsort(t, kind="stable")
        q_deg = np.degrees(np.asarray(positions[name]))
        traces[name] = CommandTrace(t=t[order], q_deg=q_deg[order])
    return traces


def recorded_trace(recording: Recording, joint: str) -> JointTrace | None:
    """The trace of ``joint``: by its name, else the only one with a ``robot/`` prefix on it."""
    trace = recording.traces.get(joint)
    if trace is not None:
        return trace
    prefixed = [name for name in recording.traces if name.rsplit("/", 1)[-1] == joint]
    return recording.traces[prefixed[0]] if len(prefixed) == 1 else None


def recorded_command(recording: Recording, joint: str) -> CommandTrace | None:
    """The commands of ``joint``: by its name, else the only one with a ``robot/`` prefix on it."""
    trace = recording.commands.get(joint)
    if trace is not None:
        return trace
    prefixed = [name for name in recording.commands if name.rsplit("/", 1)[-1] == joint]
    return recording.commands[prefixed[0]] if len(prefixed) == 1 else None


def step_start(recording: Recording, event: StepEvent, stop: float) -> float:
    """When the step's target went out: its first recorded command from ``t0`` to ``stop``.

    That is the first within ``COMMAND_MATCH_RAD`` of ``target_after_deg``;
    ``t0`` when none was recorded.
    """
    trace = recorded_command(recording, event.joint)
    if trace is None:
        return event.t0
    tolerance = math.degrees(COMMAND_MATCH_RAD)
    hits = np.flatnonzero(
        (trace.t >= event.t0)
        & (trace.t <= stop)
        & (np.abs(trace.q_deg - event.target_after_deg) <= tolerance)
    )
    return float(trace.t[hits[0]]) if hits.size else event.t0


def extract_steps(
    recording: Recording,
    on_step: Callable[[StepEvent], None] | None = None,
    dither_friction_nm: Mapping[str, float] | None = None,
) -> list[StepRecord]:
    """One record per step marker, from when its target went out (``step_start``).

    A marker with no recorded samples keeps its record, with none, and fails.
    Each joint's friction is the larger of what its sweep shows
    (``sweep_friction_nm``) and ``dither_friction_nm``, by joint name.
    """
    unrecorded = JointTrace(t=np.empty(0), q_deg=np.empty(0), dq_deg_s=np.empty(0))
    windows = []
    for event, _, stop in segment_steps(recording.events, recording.t_end):
        if on_step is not None:
            on_step(event)
        start = step_start(recording, event, stop)
        trace = recorded_trace(recording, event.joint) or unrecorded
        mask = (trace.t >= start) & (trace.t <= stop)
        before = np.flatnonzero(trace.t < start)
        y0 = float(trace.q_deg[before[-1]]) if before.size else event.target_before_deg
        windows.append((event, trace.t[mask] - start, trace.q_deg[mask], trace.dq_deg_s[mask], y0))
    friction = dict(dither_friction_nm or {})
    swept = sweep_friction_nm(analyze_step(e, t, q, dq, y0_deg=y0) for e, t, q, dq, y0 in windows)
    for joint, value in swept.items():
        friction[joint] = max(value, friction.get(joint, 0.0))
    return [
        StepRecord(
            event=event,
            t=t,
            q_deg=q - event.target_before_deg,
            target_deg=np.full(t.shape, event.target_after_deg - event.target_before_deg),
            metrics=analyze_step(
                event, t, q, dq, y0_deg=y0, friction_nm=friction.get(event.joint, 0.0)
            ),
        )
        for event, t, q, dq, y0 in windows
    ]


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
        "# Joint step response\n\n"
        f"{len(records)} steps over {len({m.joint for m in metrics})} joints.\n\n"
        + format_table(metrics)
        + "\n\n## Proposed gains\n\n"
        + format_proposals(propose_gains(metrics))
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
        Line2D(
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
    joints = list(dict.fromkeys(record.event.joint for record in records))
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
