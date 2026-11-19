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

"""Offline float-test report from a recording of any known robot.

    python -m dimos.control.calibration.float_report recordings/<run-id> \
        --robot dual-openyam [--plot]

Reads ``memory.db``, cuts the joint stream at each ``float_events`` marker and
prints, per pose, the fingertip drift over the window (mm, pass below 2 mm),
the largest joint drift (deg) and which joint it was. Each marker's ``side``
names the arm (``robots.arm_spec(robot, side)``). Writes ``float_report/``
next to the database with ``poses.csv`` and ``report.md``; ``--plot`` adds one
PNG per arm of fingertip drift against time, one line per pose.
"""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Callable
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np
import typer

from dimos.control.calibration.float_analysis import (
    DRIFT_LIMIT_MM,
    DriftMetrics,
    FingertipModel,
    FloatEvent,
    analyze_window,
    fingertip_drift_mm_series,
    format_metrics,
    format_table,
    segment_windows,
)
from dimos.control.calibration.progress import ProgressBar
from dimos.control.calibration.robots import arm_spec
from dimos.control.calibration.spec import ArmCalibrationSpec
from dimos.memory.store.sqlite import SqliteStore

matplotlib.use("Agg")
import matplotlib.pyplot as plt

JOINT_STATE_STREAM = "joint_state"
FLOAT_EVENTS_STREAM = "float_events"
REPORT_DIR_NAME = "float_report"

_POSE_COLORS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#7d6fd6", "#52514e")
_INK = "#52514e"
_GRID = "#e5e4df"
_SURFACE = "#fcfcfb"

app = typer.Typer(add_completion=False)


@dataclass(frozen=True)
class FloatRecording:
    t: np.ndarray  # sorted sample times
    q_rad: dict[str, np.ndarray]  # joint name -> angles, one per sample
    events: list[FloatEvent]


@dataclass(frozen=True)
class WindowRecord:
    event: FloatEvent
    t: np.ndarray  # seconds since the marker
    q_rad: np.ndarray  # (n, dof) in the arm's joint order
    metrics: DriftMetrics


def load_recording(recording_dir: Path) -> FloatRecording:
    path = recording_dir / "memory.db"
    if not path.exists():
        raise FileNotFoundError(f"no memory.db in {recording_dir}")
    store = SqliteStore(path=str(path), must_exist=True)
    store.start()
    try:
        names = set(store.list_streams())
        missing = {JOINT_STATE_STREAM, FLOAT_EVENTS_STREAM} - names
        if missing:
            raise ValueError(f"{path} lacks streams {sorted(missing)}; has {sorted(names)}")
        stamps: list[float] = []
        positions: list[list[float]] = []
        joint_names: list[str] | None = None
        state_stream: Any = store.stream(JOINT_STATE_STREAM)
        for obs in state_stream:
            msg = obs.data
            if joint_names is None:
                joint_names = list(msg.name)
            stamps.append(float(msg.ts or obs.ts))
            positions.append(list(msg.position))
        if joint_names is None:
            raise ValueError(f"{JOINT_STATE_STREAM} is empty")
        t = np.asarray(stamps)
        order = np.argsort(t, kind="stable")
        q = np.asarray(positions)[order]
        event_stream: Any = store.stream(FLOAT_EVENTS_STREAM)
        events = [FloatEvent.from_json(obs.data.data) for obs in event_stream]
    finally:
        store.stop()
    return FloatRecording(
        t=t[order], q_rad={name: q[:, i] for i, name in enumerate(joint_names)}, events=events
    )


def extract_windows(
    recording: FloatRecording,
    models: dict[str, FingertipModel] | None = None,
    resolve: Callable[[str], ArmCalibrationSpec] | None = None,
) -> list[WindowRecord]:
    """One record per marker that has at least two samples inside its window.

    ``models`` holds a fingertip model per marker ``side``; one missing is built
    from ``resolve(side)``.
    """
    models = {} if models is None else models
    records: list[WindowRecord] = []
    for event, start, stop in segment_windows(recording.events, float(recording.t[-1])):
        mask = (recording.t >= start) & (recording.t <= stop)
        if mask.sum() < 2:
            continue
        model = models.get(event.side)
        if model is None:
            if resolve is None:
                raise ValueError(f"no arm model for float markers on {event.side!r}")
            model = models[event.side] = FingertipModel(resolve(event.side))
        names = _recorded_names(recording, model.joint_names)
        q = np.stack([recording.q_rad[name][mask] for name in names], axis=1)
        t = recording.t[mask]
        metrics = analyze_window(model, event, t, q)
        records.append(WindowRecord(event=event, t=t - start, q_rad=q, metrics=metrics))
    return records


def _recorded_names(recording: FloatRecording, joint_names: list[str]) -> list[str]:
    """Recorded joint per arm joint: the same name, or one ``robot/`` prefix on it."""
    names = []
    for name in joint_names:
        if name in recording.q_rad:
            names.append(name)
            continue
        prefixed = [key for key in recording.q_rad if key.rsplit("/", 1)[-1] == name]
        if len(prefixed) != 1:
            raise ValueError(f"the recording has no single joint for {name!r}")
        names.append(prefixed[0])
    return names


def write_tables(records: list[WindowRecord], out_dir: Path) -> tuple[Path, Path]:
    rows = [record.metrics.as_row() for record in records]
    csv_path = out_dir / "poses.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["side"])
        writer.writeheader()
        writer.writerows(rows)
    metrics = [record.metrics for record in records]
    passed = sum(m.passed for m in metrics)
    md_path = out_dir / "report.md"
    md_path.write_text(
        "# Float (hand-teaching) test\n\n"
        f"{len(metrics)} window(s); {passed} within {DRIFT_LIMIT_MM:g} mm at the fingertip.\n\n"
        + format_table(metrics)
        + "\n"
    )
    return csv_path, md_path


def _style(ax: Any) -> None:
    ax.set_facecolor(_SURFACE)
    ax.grid(True, color=_GRID, linewidth=0.6)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(_GRID)
    ax.tick_params(colors=_INK, labelsize=8)


def plot_side(side: str, records: list[WindowRecord], model: FingertipModel, path: Path) -> None:
    """Fingertip drift against time for every window on ``side``, one line per pose."""
    fig, ax = plt.subplots(figsize=(7.0, 3.8))
    fig.patch.set_facecolor(_SURFACE)
    _style(ax)
    for position, record in enumerate(sorted(records, key=lambda r: r.event.index)):
        series = fingertip_drift_mm_series(model, record.t, record.q_rad)
        ax.plot(
            record.t,
            series,
            color=_POSE_COLORS[position % len(_POSE_COLORS)],
            linewidth=1.6,
            label=f"pose {record.event.index} ({record.metrics.fingertip_drift_mm:.2f} mm)",
        )
    ax.axhline(DRIFT_LIMIT_MM, color=_INK, linewidth=0.8, linestyle=":")
    ax.set_xlabel("time in window (s)", fontsize=8, color=_INK)
    ax.set_ylabel("fingertip drift from start (mm)", fontsize=8, color=_INK)
    ax.legend(fontsize=8, frameon=False, loc="upper left")
    fig.suptitle(
        f"{model.arm_id} floating: fingertip drift (dotted: {DRIFT_LIMIT_MM:g} mm limit)",
        fontsize=10,
        color=_INK,
        x=0.02,
        ha="left",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(path, dpi=120)
    plt.close(fig)


def generate_report(recording_dir: Path, *, robot: str, plot: bool = False) -> Path:
    """Analyse a recording directory and write ``float_report/`` inside it.

    Each marker's ``side`` is an arm of ``robot`` (a ``robots.KNOWN_ROBOTS`` key).
    """
    print(f"reading {recording_dir / 'memory.db'} ...", flush=True)
    recording = load_recording(recording_dir)
    if not recording.events:
        raise ValueError(f"no float_events markers in {recording_dir}")
    sides = sorted({event.side for event in recording.events})
    bar = ProgressBar(len(sides) * (2 if plot else 1) + 1, "float report")
    models: dict[str, FingertipModel] = {}
    for side in sides:
        bar.update(text=f"load {side} arm model")
        models[side] = FingertipModel(arm_spec(robot, side))
    records = extract_windows(recording, models)
    if not records:
        bar.close()
        raise ValueError(f"no float window in {recording_dir} has joint samples")
    out_dir = recording_dir / REPORT_DIR_NAME
    out_dir.mkdir(exist_ok=True)
    write_tables(records, out_dir)
    if plot:
        by_side: dict[str, list[WindowRecord]] = defaultdict(list)
        for record in records:
            by_side[record.event.side].append(record)
        for side, side_records in by_side.items():
            bar.update(text=f"plot {side}")
            plot_side(side, side_records, models[side], out_dir / f"{side}_fingertip_drift.png")
    bar.update(text="tables")
    bar.close()
    for record in records:
        print(format_metrics(record.metrics))
    return out_dir


@app.command()
def main(
    recording_dir: Path = typer.Argument(..., help="recordings/<run-id> with a memory.db"),
    robot: str = typer.Option(..., help="The robot that floated, e.g. dual-openyam"),
    plot: bool = typer.Option(False, "--plot", help="Also write a drift-vs-time PNG per arm"),
) -> None:
    """Report the fingertip drift of each float window in a recording."""
    out_dir = generate_report(recording_dir, robot=robot, plot=plot)
    typer.echo((out_dir / "report.md").read_text())
    typer.echo(f"Wrote {out_dir}")


if __name__ == "__main__":
    app()
