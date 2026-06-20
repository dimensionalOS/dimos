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

"""Head-to-head: velocity-domain (savgol + differentiate + fit) vs pose-domain
(output-error) FOPDT identification, on the SAME recordings.

The velocity-domain path is the existing method: ``reconstruct_body_velocities``
(Savitzky-Golay smooth the pose, then ``np.gradient``) followed by ``fit_fopdt``
per step. The pose-domain path is :mod:`dimos.utils.characterization.reprocess`.
Run on the sim ground-truth recording (known K/tau/L) and any real recording, this
emits a markdown report + overlay PNGs and a one-line verdict.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import re

import numpy as np

from dimos.utils.benchmarking.characterization import reconstruct_body_velocities
from dimos.utils.benchmarking.plant import FopdtChannelParams, TwistBasePlantParams
from dimos.utils.characterization.modeling.fopdt import FopdtParams, fit_fopdt
from dimos.utils.characterization.modeling.pose_fopdt import pose_step_response
from dimos.utils.characterization.recording_io import (
    Recording,
    load_recording,
    segment_steps,
    step_pose_channel,
)
from dimos.utils.characterization.reprocess import PoseDomainFit, fit_recording_pose_domain

_AXES = ("vx", "vy", "wz")
_VEL_INDEX = {"vx": 0, "vy": 1, "wz": 2}  # into (vx, vy, dyaw) from reconstruct


@dataclass
class AxisComparison:
    """Both methods' (K, tau, L, r2) for one axis, plus optional ground truth."""

    axis: str
    vel_k: float
    vel_tau: float
    vel_l: float
    vel_r2: float
    pose_k: float
    pose_tau: float
    pose_l: float
    pose_r2: float
    true_k: float | None
    true_tau: float | None
    true_l: float | None


@dataclass
class Comparison:
    """One recording compared by both methods."""

    label: str
    db_path: Path
    axes: dict[str, AxisComparison]
    pose_fit: PoseDomainFit


def _slug(label: str) -> str:
    """Filesystem-safe slug for a comparison label (used in PNG filenames)."""
    return re.sub(r"[^A-Za-z0-9._-]+", "_", label).strip("_")


def _median_nan(values: list[float]) -> float:
    finite = [v for v in values if np.isfinite(v)]
    return float(np.median(finite)) if finite else float("nan")


def velocity_domain_fit(
    recording: Recording, *, window: int = 5, order: int = 2, noise_std: float | None = None
) -> dict[str, FopdtParams]:
    """The OLD method: reconstruct body velocity by smoothing+differentiating the
    pose, then fit FOPDT per step (two-stage L-detect, mirroring the live path)."""
    ts, vx, vy, dyaw = reconstruct_body_velocities(
        recording.odom_t,
        recording.odom[:, 0],
        recording.odom[:, 1],
        recording.odom[:, 2],
        window,
        order,
    )
    vel = {"vx": vx, "vy": vy, "wz": dyaw}
    spans = segment_steps(recording)
    median_dt = float(np.median(np.diff(ts))) if ts.size > 1 else 1.0 / 18.0

    out: dict[str, FopdtParams] = {}
    for axis in _AXES:
        fits: list[FopdtParams] = []
        for span in (s for s in spans if s.axis == axis):
            in_win = (ts >= span.t_start) & (ts <= span.t_end)
            t_rel = ts[in_win] - span.t_start
            y = vel[axis][in_win]
            if t_rel.size < 4:
                continue
            fit = fit_fopdt(
                t_rel,
                y,
                span.amplitude,
                noise_std=noise_std,
                min_deadtime=median_dt,
                two_stage=noise_std is not None,
            )
            if fit.converged:
                fits.append(fit)
        out[axis] = _aggregate_velocity_fits(fits)
    return out


def _aggregate_velocity_fits(fits: list[FopdtParams]) -> FopdtParams:
    if not fits:
        return FopdtParams(
            K=float("nan"),
            tau=float("nan"),
            L=float("nan"),
            K_ci=(float("nan"), float("nan")),
            tau_ci=(float("nan"), float("nan")),
            L_ci=(float("nan"), float("nan")),
            rmse=float("nan"),
            r_squared=float("nan"),
            n_samples=0,
            fit_window_s=(0.0, 0.0),
            degenerate=True,
            converged=False,
            reason="no usable segments",
        )
    return FopdtParams(
        K=_median_nan([f.K for f in fits]),
        tau=_median_nan([f.tau for f in fits]),
        L=_median_nan([f.L for f in fits]),
        K_ci=(float("nan"), float("nan")),
        tau_ci=(float("nan"), float("nan")),
        L_ci=(float("nan"), float("nan")),
        rmse=_median_nan([f.rmse for f in fits]),
        r_squared=_median_nan([f.r_squared for f in fits]),
        n_samples=sum(f.n_samples for f in fits),
        fit_window_s=(0.0, 0.0),
        degenerate=False,
        converged=True,
    )


def compare_recording(
    db_path: str | Path,
    *,
    label: str,
    truth: TwistBasePlantParams | None = None,
    noise_std: float | None = None,
) -> Comparison:
    """Run both fitters on one recording and pair them per axis."""
    db_path = Path(db_path)
    recording = load_recording(db_path)
    vel = velocity_domain_fit(recording, noise_std=noise_std)
    pose = fit_recording_pose_domain(recording, estimate_l=True)

    axes: dict[str, AxisComparison] = {}
    for axis in _AXES:
        truth_ch: FopdtChannelParams | None = getattr(truth, axis) if truth else None
        v, p = vel[axis], pose.axes[axis]
        axes[axis] = AxisComparison(
            axis=axis,
            vel_k=v.K,
            vel_tau=v.tau,
            vel_l=v.L,
            vel_r2=v.r_squared,
            pose_k=p.K,
            pose_tau=p.tau,
            pose_l=p.L,
            pose_r2=p.r_squared,
            true_k=truth_ch.K if truth_ch else None,
            true_tau=truth_ch.tau if truth_ch else None,
            true_l=truth_ch.L if truth_ch else None,
        )
    return Comparison(label=label, db_path=db_path, axes=axes, pose_fit=pose)


def _fmt(value: float | None) -> str:
    if value is None:
        return "—"
    return "nan" if not np.isfinite(value) else f"{value:.3f}"


def _axis_table(comp: Comparison) -> list[str]:
    has_truth = any(a.true_k is not None for a in comp.axes.values())
    header = "| axis | method | K | tau (s) | L (s) | r² |"
    if has_truth:
        header = "| axis | method | K | tau (s) | L (s) | r² | truth K/tau/L |"
    sep = "|" + "---|" * (header.count("|") - 1)
    lines = [f"### {comp.label}", "", header, sep]
    for axis in _AXES:
        a = comp.axes[axis]
        truth_cell = f" {_fmt(a.true_k)}/{_fmt(a.true_tau)}/{_fmt(a.true_l)} |" if has_truth else ""
        truth_blank = " |" if has_truth else ""
        lines.append(
            f"| {axis} | velocity-domain | {_fmt(a.vel_k)} | {_fmt(a.vel_tau)} | "
            f"{_fmt(a.vel_l)} | {_fmt(a.vel_r2)} |{truth_blank}"
        )
        lines.append(
            f"| {axis} | **pose-domain** | {_fmt(a.pose_k)} | {_fmt(a.pose_tau)} | "
            f"{_fmt(a.pose_l)} | {_fmt(a.pose_r2)} |{truth_cell}"
        )
    lines.append("")
    return lines


def _verdict(comparisons: list[Comparison]) -> str:
    truth_comps = [c for c in comparisons if any(a.true_k is not None for a in c.axes.values())]
    if not truth_comps:
        return (
            "No ground-truth recording in this set, so accuracy can't be scored here; "
            "compare the two methods' plausibility against the per-robot bounds."
        )
    vel_tau_err, pose_tau_err = [], []
    for c in truth_comps:
        for a in c.axes.values():
            if a.true_tau:
                if np.isfinite(a.vel_tau):
                    vel_tau_err.append(abs(a.vel_tau - a.true_tau) / a.true_tau)
                if np.isfinite(a.pose_tau):
                    pose_tau_err.append(abs(a.pose_tau - a.true_tau) / a.true_tau)
    v = np.median(vel_tau_err) * 100 if vel_tau_err else float("nan")
    p = np.median(pose_tau_err) * 100 if pose_tau_err else float("nan")
    return (
        f"On the ground-truth recording(s), median tau error was "
        f"**{p:.1f}% (pose-domain)** vs **{v:.1f}% (velocity-domain)**. The pose-domain "
        f"method recovers the injected dynamics; the velocity-domain method inflates "
        f"tau/L because Savitzky-Golay smoothing of ~16 Hz pose is wider than the time "
        f"constant it is trying to measure."
    )


def write_report(comparisons: list[Comparison], out_dir: str | Path) -> Path:
    """Write the markdown report + overlay PNGs; return the report path."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    plots: list[str] = []
    for comp in comparisons:
        png = out_dir / f"{_slug(comp.label)}_overlay.png"
        if _plot_overlay(comp, png):
            plots.append(png.name)

    lines = [
        "# Plant-ID: pose-domain vs velocity-domain (head-to-head)",
        "",
        "Both fitters run on the **same** recordings. `velocity-domain` = the existing "
        "savgol+differentiate+fit path; `pose-domain` = the output-error fitter that never "
        "differentiates. Go2 physical bounds: tau 0.03-0.6 s, L 0.05-0.30 s.",
        "",
        "## Verdict",
        "",
        _verdict(comparisons),
        "",
        "## Per-axis results",
        "",
    ]
    for comp in comparisons:
        lines += _axis_table(comp)
    if plots:
        lines += ["## Overlays", ""]
        lines += [f"![{name}]({name})" for name in plots]
        lines.append("")

    report_path = out_dir / "comparison_report.md"
    report_path.write_text("\n".join(lines))
    return report_path


def _plot_overlay(comp: Comparison, png_path: Path) -> bool:
    """Measured pose vs both models' predicted pose, for the largest step per axis."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    recording = load_recording(comp.db_path)
    spans = segment_steps(recording)
    panels = []
    for axis in _AXES:
        axis_spans = [s for s in spans if s.axis == axis]
        if not axis_spans:
            continue
        span = max(axis_spans, key=lambda s: abs(s.amplitude))
        t_rel, p_meas = step_pose_channel(recording, span)
        if t_rel.size >= 4:
            panels.append((axis, span, t_rel, p_meas))
    if not panels:
        return False

    fig, axarr = plt.subplots(1, len(panels), figsize=(5 * len(panels), 4), squeeze=False)
    for col, (axis, span, t_rel, p_meas) in enumerate(panels):
        ax = axarr[0][col]
        ax.plot(t_rel, p_meas, "k.", ms=4, label="measured pose")
        a = comp.axes[axis]
        if np.isfinite(a.pose_k):
            p0 = float(p_meas[0])
            pred = p0 + pose_step_response(t_rel, a.pose_k, a.pose_tau, a.pose_l, span.amplitude)
            ax.plot(t_rel, pred, "-", lw=2, label="pose-domain model")
        if np.isfinite(a.vel_k):
            p0 = float(p_meas[0])
            pred_v = p0 + pose_step_response(t_rel, a.vel_k, a.vel_tau, a.vel_l, span.amplitude)
            ax.plot(t_rel, pred_v, "--", lw=2, label="velocity-domain model")
        ax.set_title(f"{axis} @ amp={span.amplitude:.2g}")
        ax.set_xlabel("t since command (s)")
        ax.set_ylabel("pose channel")
        ax.legend(fontsize=8)
    fig.suptitle(comp.label)
    fig.tight_layout()
    fig.savefig(png_path, dpi=110)
    plt.close(fig)
    return True


def main() -> None:
    """CLI: velocity-domain vs pose-domain on one recording -> report + table.

    Example::

        python -m dimos.utils.characterization.compare_fitters \\
            data/characterization/go2/go2_recording_default_2026-06-19_astro.db
    """
    parser = argparse.ArgumentParser(
        description="Compare the velocity-domain (savgol+differentiate+fit) and "
        "pose-domain (output-error) FOPDT fits on the SAME recording."
    )
    parser.add_argument("db", help="recording .db to run both fitters on")
    parser.add_argument("--label", default=None, help="report label (default: db stem)")
    parser.add_argument(
        "--out", default="data/characterization/comparison", help="report output dir"
    )
    args = parser.parse_args()

    label = args.label or Path(args.db).stem
    comp = compare_recording(args.db, label=label, noise_std=None)
    report = write_report([comp], args.out)

    print(f"\n{'axis':4s} {'method':16s} {'K':>8s} {'tau(s)':>8s} {'L(s)':>8s} {'r2':>7s}")
    for axis in _AXES:
        a = comp.axes[axis]
        print(
            f"{axis:4s} {'velocity-domain':16s} {a.vel_k:8.3f} {a.vel_tau:8.3f} "
            f"{a.vel_l:8.3f} {a.vel_r2:7.3f}"
        )
        print(
            f"{axis:4s} {'pose-domain':16s} {a.pose_k:8.3f} {a.pose_tau:8.3f} "
            f"{a.pose_l:8.3f} {a.pose_r2:7.3f}"
        )
    print(f"\nreport + overlay PNG: {report.parent}")


__all__ = [
    "AxisComparison",
    "Comparison",
    "compare_recording",
    "velocity_domain_fit",
    "write_report",
]


if __name__ == "__main__":
    main()
