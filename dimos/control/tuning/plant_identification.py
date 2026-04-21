#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Plant identification from step-response data.

Reads the CSV from step_response_test.py (raw odom + phase markers),
computes velocity from position via smoothed differentiation, and
plots per-amplitude step response curves.

Optionally fits a First-Order Plus Dead Time (FOPDT) model.

Usage:
    .venv/bin/python -m dimos.control.tuning.plant_identification --data step_response_data.csv
    .venv/bin/python -m dimos.control.tuning.plant_identification --data step_response_data.csv --channel vx
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray


def _zero_phase_lowpass(signal: NDArray, dt_arr: NDArray, cutoff_hz: float = 2.0) -> NDArray:
    """Apply a zero-phase (no lag) Butterworth low-pass filter.

    Uses scipy.signal.filtfilt which runs the filter forward then backward,
    canceling phase distortion. Safe for system identification — the filtered
    signal is smoothed but NOT time-shifted.

    Args:
        signal: Raw velocity signal.
        dt_arr: Array of time deltas between samples.
        cutoff_hz: Filter cutoff frequency (Hz). Set below gait frequency
                   (~3-4 Hz for Go2) to remove leg oscillation noise.

    Returns:
        Filtered signal (same length as input).
    """
    from scipy.signal import butter, filtfilt

    if len(signal) < 12:
        return signal

    # Estimate sample rate from median dt
    fs = 1.0 / float(np.median(dt_arr))
    nyquist = fs / 2.0

    if cutoff_hz >= nyquist:
        cutoff_hz = nyquist * 0.8

    b, a = butter(2, cutoff_hz / nyquist, btype="low")
    return filtfilt(b, a, signal)


def fopdt_step_response(
    t: NDArray, K: float, tau: float, theta: float, amplitude: float
) -> NDArray:
    """Analytical FOPDT step response.

    y(t) = K * amplitude * (1 - exp(-(t - theta) / tau))  for t >= theta
    y(t) = 0                                                for t < theta
    """
    y = np.zeros_like(t)
    mask = t >= theta
    if tau > 1e-6:
        y[mask] = K * amplitude * (1.0 - np.exp(-(t[mask] - theta) / tau))
    else:
        y[mask] = K * amplitude
    return y


@dataclass
class FOPDTParams:
    """First-Order Plus Dead Time model parameters."""

    K: float  # steady-state gain (actual / commanded)
    tau: float  # time constant (s) — time to reach 63.2%
    theta: float  # dead time (s) — pure delay before response
    channel: str = ""
    amplitude: float = 0.0

    def __repr__(self) -> str:
        return (
            f"FOPDT({self.channel} @ {self.amplitude:+.1f}: "
            f"K={self.K:.3f}, τ={self.tau:.3f}s, θ={self.theta:.3f}s)"
        )


def remove_outlier_trials(trials: list[TrialData]) -> list[TrialData]:
    """Drop outlier trials using both FOPDT params and steady-state velocity.

    Two-stage rejection:
    1. FOPDT-based: drop if K or tau deviates >30% from median.
    2. Steady-state: drop if mean velocity during last 50% of step phase
       deviates >25% from median across trials.
    """
    groups: dict[tuple[str, float], list[TrialData]] = {}
    for t in trials:
        groups.setdefault((t.channel, t.amplitude), []).append(t)

    clean: list[TrialData] = []
    for (_channel, _amplitude), group in groups.items():
        if len(group) < 2:
            clean.extend(group)
            continue

        # Stage 1: FOPDT param check
        fits = [(t, identify_fopdt(t)) for t in group]
        Ks = [p.K for _, p in fits]
        taus = [p.tau for _, p in fits]
        K_med = float(np.median(Ks))
        tau_med = float(np.median(taus))

        stage1 = []
        for t, p in fits:
            K_dev = abs(p.K - K_med) / max(K_med, 1e-6)
            tau_dev = abs(p.tau - tau_med) / max(tau_med, 1e-6)
            if K_dev <= 0.25 and tau_dev <= 0.25:
                stage1.append(t)

        # Stage 2: steady-state velocity check on survivors
        # Skip percentage-based rejection when steady-state is near zero
        # (e.g. vy=0.2 where the robot genuinely can't move laterally)
        if len(stage1) >= 2:
            ss_values = []
            for t in stage1:
                step_mask = np.array([p == "step" for p in t.phase])
                step_vel = t.actual[step_mask]
                n_tail = max(1, len(step_vel) // 2)
                ss_values.append(float(np.mean(step_vel[-n_tail:])))

            ss_med = float(np.median(ss_values))
            # Only apply percentage rejection if steady-state is large enough
            # to make percentages meaningful (>0.05 m/s or rad/s)
            if abs(ss_med) > 0.05:
                for t, ss in zip(stage1, ss_values, strict=False):
                    ss_dev = abs(ss - ss_med) / abs(ss_med)
                    if ss_dev <= 0.25:
                        clean.append(t)
            else:
                clean.extend(stage1)
        else:
            clean.extend(stage1)

    return clean


@dataclass
class TrialData:
    """Processed data for one trial."""

    channel: str
    amplitude: float
    trial: int
    time: NDArray  # relative time (s), zeroed at step onset
    cmd: NDArray  # commanded velocity for the channel
    actual: NDArray  # actual velocity (from odom differentiation)
    phase: list[str]  # phase labels per sample


def load_trials(csv_path: str, channel_filter: str | None = None) -> list[TrialData]:
    """Load CSV and split into per-trial TrialData with velocity computation."""
    # Group rows by (channel, amplitude, trial)
    groups: dict[tuple[str, float, int], list[dict]] = {}
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            key = (row["channel"], float(row["amplitude"]), int(row["trial"]))
            if channel_filter and key[0] != channel_filter:
                continue
            groups.setdefault(key, []).append(row)

    trials = []
    for (channel, amplitude, trial_num), rows in sorted(groups.items()):
        ts = np.array([float(r["timestamp"]) for r in rows])
        x = np.array([float(r["odom_x"]) for r in rows])
        y = np.array([float(r["odom_y"]) for r in rows])
        yaw = np.array([float(r["odom_yaw"]) for r in rows])
        phases = [r["phase"] for r in rows]

        # Deduplicate: odom arrives at ~20Hz but we sample at 50Hz.
        # Keep only rows where odom actually changed (new timestamp).
        unique_mask = np.concatenate([[True], np.diff(ts) > 1e-6])
        ts = ts[unique_mask]
        x = x[unique_mask]
        y = y[unique_mask]
        yaw = yaw[unique_mask]
        phases = [p for p, m in zip(phases, unique_mask, strict=False) if m]

        # Compute body-frame velocity from world-frame odom.
        # World displacement (dx, dy) must be rotated into the robot's
        # heading frame to get body vx (forward) and vy (lateral).
        dt = np.diff(ts)
        dt[dt < 1e-6] = 1e-6
        dx = np.diff(x)
        dy = np.diff(y)
        dyaw = np.diff(yaw)
        dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi

        # Rotation into body frame using midpoint heading
        mid_yaw = yaw[:-1] + dyaw / 2.0
        cos_y = np.cos(mid_yaw)
        sin_y = np.sin(mid_yaw)

        # body_vx = (dx*cos + dy*sin) / dt  (forward in robot frame)
        # body_vy = (-dx*sin + dy*cos) / dt  (left in robot frame)
        body_vx_raw = (dx * cos_y + dy * sin_y) / dt
        body_vy_raw = (-dx * sin_y + dy * cos_y) / dt
        body_wz_raw = dyaw / dt

        if channel == "vx":
            vel_raw = body_vx_raw
        elif channel == "vy":
            vel_raw = body_vy_raw
        else:
            vel_raw = body_wz_raw

        # Zero-phase Butterworth low-pass filter (no phase lag)
        vel_smooth = _zero_phase_lowpass(vel_raw, dt_arr=dt, cutoff_hz=2.0)

        # Pad to match original length (diff loses one sample)
        vel = np.concatenate([[0.0], vel_smooth])

        # Commanded velocity for this channel
        if channel == "vx":
            cmd = np.array([float(r["cmd_vx"]) for r in rows])
        elif channel == "vy":
            cmd = np.array([float(r["cmd_vy"]) for r in rows])
        else:
            cmd = np.array([float(r["cmd_wz"]) for r in rows])

        # Zero time at step onset
        step_indices = [i for i, p in enumerate(phases) if p == "step"]
        t0 = ts[step_indices[0]] if step_indices else ts[0]
        t_rel = ts - t0

        trials.append(
            TrialData(
                channel=channel,
                amplitude=amplitude,
                trial=trial_num,
                time=t_rel,
                cmd=cmd,
                actual=vel,
                phase=phases,
            )
        )

    return trials


def identify_fopdt(trial: TrialData) -> FOPDTParams:
    """Fit FOPDT model to a single trial's step phase."""
    from scipy.optimize import curve_fit

    # Extract step phase only
    step_mask = np.array([p == "step" for p in trial.phase])
    t = trial.time[step_mask]
    y = trial.actual[step_mask]
    amplitude = abs(trial.amplitude)

    if trial.amplitude < 0:
        y = -y

    if len(t) < 10 or amplitude < 1e-6:
        return FOPDTParams(
            K=1.0, tau=0.1, theta=0.0, channel=trial.channel, amplitude=trial.amplitude
        )

    t = t - t[0]  # zero at step start

    # Estimate steady-state from last 20% of step
    n_tail = max(1, len(y) // 5)
    y_ss = float(np.mean(y[-n_tail:]))
    K_est = y_ss / amplitude if amplitude > 1e-6 else 1.0
    K_est = max(abs(K_est), 0.01)

    # Dead time: first time response exceeds 10% of steady state
    threshold = 0.1 * abs(y_ss)
    above = np.abs(y) > threshold
    theta_est = float(t[np.argmax(above)]) if np.any(above) else 0.0

    # Time constant: time from theta to 63.2%
    target = 0.632 * abs(y_ss)
    post_theta = t >= theta_est
    above_63 = np.abs(y) >= target
    both = post_theta & above_63
    tau_est = float(t[np.argmax(both)] - theta_est) if np.any(both) else 0.1
    tau_est = max(tau_est, 0.01)

    def model(t_arr: NDArray, K: float, tau: float, theta: float) -> NDArray:
        out = np.zeros_like(t_arr)
        mask = t_arr >= theta
        out[mask] = K * amplitude * (1.0 - np.exp(-(t_arr[mask] - theta) / tau))
        return out

    try:
        popt, _ = curve_fit(
            model,
            t,
            y,
            p0=[K_est, tau_est, theta_est],
            bounds=([0.001, 0.001, 0.0], [10.0, 5.0, 2.0]),
            maxfev=5000,
        )
        return FOPDTParams(
            K=popt[0], tau=popt[1], theta=popt[2], channel=trial.channel, amplitude=trial.amplitude
        )
    except Exception:
        return FOPDTParams(
            K=K_est, tau=tau_est, theta=theta_est, channel=trial.channel, amplitude=trial.amplitude
        )


def _get_ylim(channel: str, amplitude: float) -> tuple[float, float]:
    """Consistent y-axis limits per channel and sign."""
    if channel == "wz":
        return (-0.1, 3.5) if amplitude > 0 else (-3.5, 0.1)
    else:
        return (-0.1, 1.5) if amplitude > 0 else (-1.5, 0.1)


def plot_step_responses(
    trials: list[TrialData],
    output: str = "step_response_curves.png",
    title_suffix: str = "",
) -> None:
    """Plot step response curves grouped by (channel, amplitude)."""
    import matplotlib.pyplot as plt

    _UNITS = {"vx": "m/s", "vy": "m/s", "wz": "rad/s"}
    _LABELS = {
        "vx": "Forward velocity (vx)",
        "vy": "Lateral velocity (vy)",
        "wz": "Yaw rate (ωz)",
    }

    # Group by (channel, amplitude)
    groups: dict[tuple[str, float], list[TrialData]] = {}
    for t in trials:
        groups.setdefault((t.channel, t.amplitude), []).append(t)

    n_plots = len(groups)
    if n_plots == 0:
        print("No data to plot")
        return

    cols = min(3, n_plots)
    rows_count = math.ceil(n_plots / cols)
    fig, axes = plt.subplots(rows_count, cols, figsize=(6 * cols, 4 * rows_count), squeeze=False)

    for idx, ((channel, amplitude), group_trials) in enumerate(sorted(groups.items())):
        ax = axes[idx // cols][idx % cols]
        unit = _UNITS.get(channel, "")
        label = _LABELS.get(channel, channel)

        for td in group_trials:
            color = "C0" if td.trial == 1 else ("C1" if td.trial == 2 else "C2")
            ax.plot(
                td.time, td.actual, color=color, alpha=0.6, linewidth=0.8, label=f"Trial {td.trial}"
            )

        # Command level
        ax.axhline(
            y=amplitude,
            color="red",
            linestyle="--",
            alpha=0.5,
            label=f"Commanded: {amplitude} {unit}",
        )
        ax.axhline(y=0, color="gray", linestyle=":", alpha=0.3)

        # Phase boundary
        ax.axvline(x=0, color="green", linestyle="-", alpha=0.4, label="Step onset")

        direction = "+" if amplitude > 0 else ""
        ax.set_title(f"{label} @ {direction}{amplitude} {unit}", fontsize=10)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"Velocity ({unit})")
        ax.legend(fontsize=7, loc="lower right")
        ax.grid(True, alpha=0.3)
        ax.set_xlim(-2.5, 13.5)
        ax.set_ylim(_get_ylim(channel, amplitude))

    # Hide unused subplots
    for idx in range(n_plots, rows_count * cols):
        axes[idx // cols][idx % cols].set_visible(False)

    suffix = f" — {title_suffix}" if title_suffix else ""
    fig.suptitle(f"Go2 Step Response{suffix}", fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(output, dpi=150)
    print(f"Saved plot to {output}")


def plot_averaged_responses(
    trials: list[TrialData],
    output: str = "step_response_averaged.png",
    title_suffix: str = "",
) -> None:
    """Plot trial-averaged step response per (channel, amplitude)."""
    import matplotlib.pyplot as plt

    _UNITS = {"vx": "m/s", "vy": "m/s", "wz": "rad/s"}
    _LABELS = {
        "vx": "Forward velocity (vx)",
        "vy": "Lateral velocity (vy)",
        "wz": "Yaw rate (ωz)",
    }

    groups: dict[tuple[str, float], list[TrialData]] = {}
    for t in trials:
        groups.setdefault((t.channel, t.amplitude), []).append(t)

    n_plots = len(groups)
    if n_plots == 0:
        return

    cols = min(3, n_plots)
    rows_count = math.ceil(n_plots / cols)
    fig, axes = plt.subplots(rows_count, cols, figsize=(6 * cols, 4 * rows_count), squeeze=False)

    for idx, ((channel, amplitude), group_trials) in enumerate(sorted(groups.items())):
        ax = axes[idx // cols][idx % cols]
        unit = _UNITS.get(channel, "")
        label = _LABELS.get(channel, channel)

        # Interpolate all trials onto common time grid and average
        t_min = max(td.time[0] for td in group_trials)
        t_max = min(td.time[-1] for td in group_trials)
        t_common = np.linspace(t_min, t_max, 500)

        all_interp = []
        for td in group_trials:
            interped = np.interp(t_common, td.time, td.actual)
            all_interp.append(interped)

        mean_vel = np.mean(all_interp, axis=0)
        std_vel = np.std(all_interp, axis=0) if len(all_interp) > 1 else np.zeros_like(mean_vel)

        ax.plot(t_common, mean_vel, "b-", linewidth=1.5, label="Mean (averaged)")
        ax.fill_between(
            t_common,
            mean_vel - std_vel,
            mean_vel + std_vel,
            color="blue",
            alpha=0.15,
            label="±1 std",
        )
        ax.axhline(
            y=amplitude,
            color="red",
            linestyle="--",
            alpha=0.5,
            label=f"Commanded: {amplitude} {unit}",
        )
        ax.axvline(x=0, color="green", linestyle="-", alpha=0.4, label="Step onset")
        ax.axhline(y=0, color="gray", linestyle=":", alpha=0.3)

        direction = "+" if amplitude > 0 else ""
        ax.set_title(f"{label} @ {direction}{amplitude} {unit}", fontsize=10)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"Velocity ({unit})")
        ax.legend(fontsize=7, loc="lower right")
        ax.grid(True, alpha=0.3)
        ax.set_xlim(-2.5, 13.5)
        ax.set_ylim(_get_ylim(channel, amplitude))

    for idx in range(n_plots, rows_count * cols):
        axes[idx // cols][idx % cols].set_visible(False)

    suffix = f" — {title_suffix}" if title_suffix else ""
    fig.suptitle(f"Go2 Step Response — Trial-Averaged{suffix}", fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(output, dpi=150)
    print(f"Saved plot to {output}")


def plot_fopdt_overlay(trials: list[TrialData], output: str = "step_response_fopdt.png") -> None:
    """Plot trial-averaged actual response with FOPDT model overlay.

    Shows how well the identified model fits the measured data.
    """
    import matplotlib.pyplot as plt

    _UNITS = {"vx": "m/s", "vy": "m/s", "wz": "rad/s"}
    _LABELS = {
        "vx": "Forward velocity (vx)",
        "vy": "Lateral velocity (vy)",
        "wz": "Yaw rate (ωz)",
    }

    groups: dict[tuple[str, float], list[TrialData]] = {}
    for t in trials:
        groups.setdefault((t.channel, t.amplitude), []).append(t)

    n_plots = len(groups)
    if n_plots == 0:
        return

    cols = min(3, n_plots)
    rows_count = math.ceil(n_plots / cols)
    fig, axes = plt.subplots(rows_count, cols, figsize=(6 * cols, 4 * rows_count), squeeze=False)

    for idx, ((channel, amplitude), group_trials) in enumerate(sorted(groups.items())):
        ax = axes[idx // cols][idx % cols]
        unit = _UNITS.get(channel, "")
        label = _LABELS.get(channel, channel)

        # Average the trials onto common time grid
        t_min = max(td.time[0] for td in group_trials)
        t_max = min(td.time[-1] for td in group_trials)
        t_common = np.linspace(t_min, t_max, 500)

        all_interp = []
        for td in group_trials:
            all_interp.append(np.interp(t_common, td.time, td.actual))

        mean_vel = np.mean(all_interp, axis=0)

        # Fit FOPDT on each trial, average the params
        fits = [identify_fopdt(td) for td in group_trials]
        K_avg = float(np.mean([f.K for f in fits]))
        tau_avg = float(np.mean([f.tau for f in fits]))
        theta_avg = float(np.mean([f.theta for f in fits]))

        # Generate model response on the same time grid
        # t_common is relative to step onset (t=0)
        amp_abs = abs(amplitude)
        model_vel = fopdt_step_response(t_common, K_avg, tau_avg, theta_avg, amp_abs)
        if amplitude < 0:
            model_vel = -model_vel

        # Plot
        ax.plot(t_common, mean_vel, "b-", linewidth=1.5, label="Actual (averaged)")
        ax.plot(
            t_common,
            model_vel,
            "r--",
            linewidth=1.5,
            label=f"FOPDT (K={K_avg:.2f}, τ={tau_avg:.2f}s, θ={theta_avg:.2f}s)",
        )
        ax.axhline(
            y=amplitude,
            color="green",
            linestyle=":",
            alpha=0.5,
            label=f"Commanded: {amplitude} {unit}",
        )
        ax.axvline(x=0, color="gray", linestyle="-", alpha=0.3, label="Step onset")
        ax.axhline(y=0, color="gray", linestyle=":", alpha=0.2)

        direction = "+" if amplitude > 0 else ""
        ax.set_title(f"{label} @ {direction}{amplitude} {unit}", fontsize=10)
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"Velocity ({unit})")
        ax.legend(fontsize=7, loc="lower right")
        ax.grid(True, alpha=0.3)
        ax.set_xlim(-2.5, 13.5)
        ax.set_ylim(_get_ylim(channel, amplitude))

    for idx in range(n_plots, rows_count * cols):
        axes[idx // cols][idx % cols].set_visible(False)

    fig.suptitle("Go2 Step Response — FOPDT Model Fit", fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(output, dpi=150)
    print(f"Saved plot to {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plant identification from step-response CSV")
    parser.add_argument("--data", required=True, help="Path to step_response_data.csv")
    parser.add_argument(
        "--channel", type=str, default=None, help="Filter to single channel (vx/vy/wz)"
    )
    parser.add_argument("--fopdt", action="store_true", help="Also fit FOPDT models")
    args = parser.parse_args()

    trials_raw = load_trials(args.data, channel_filter=args.channel)
    print(f"Loaded {len(trials_raw)} trials")

    if not trials_raw:
        print("No data found")
        return

    trials = remove_outlier_trials(trials_raw)
    dropped = len(trials_raw) - len(trials)
    if dropped:
        print(f"Removed {dropped} outlier trial(s), {len(trials)} remaining")

    # Always plot (with outliers removed)
    try:
        import matplotlib

        matplotlib.use("Agg")
        plot_step_responses(trials)
        plot_averaged_responses(trials)
        plot_fopdt_overlay(trials)
    except ImportError:
        print("matplotlib not available — skipping plots")

    # Optional FOPDT fitting (print numbers)
    if args.fopdt:
        print("\n" + "=" * 65)
        print("FOPDT Plant Identification Results")
        print("=" * 65)
        for trial in trials:
            params = identify_fopdt(trial)
            print(f"  {params}")
        print("=" * 65)


if __name__ == "__main__":
    main()
