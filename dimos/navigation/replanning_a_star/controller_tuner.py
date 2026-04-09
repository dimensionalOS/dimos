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
"""
ControllerTuner
===============
Auto-calibration for TrajectoryController.

Inspired by ArduPilot AutoTune and Klipper resonance compensation.
Moves the robot in known patterns, measures response curves, then
computes optimal PID gains using ITAE criteria.

Phases
------
1. Lateral step response  → kp_cross, kd_cross, max_lateral_accel
2. Heading step response  → kp_heading, kd_heading, kff_curvature
3. Validation on 1×1 m square → speed vs divergence report

Usage
-----
    # Offline / simulation:
    tuner = ControllerTuner(control_frequency=10.0)
    result = tuner.tune_from_simulation(max_speed=1.2)
    tuner.apply_to(trajectory_controller)
    tuner.plot_results("/tmp/tuning_results.png")

    # CLI:
    python3 -m dimos.navigation.replanning_a_star.controller_tuner --speed 1.2
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    from dimos.navigation.replanning_a_star.trajectory_controller import TrajectoryController


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class StepResponse:
    target: float
    times: list[float]
    values: list[float]
    rise_time: float = 0.0
    overshoot_pct: float = 0.0
    settling_time: float = 0.0
    steady_state_error: float = 0.0
    delay: float = 0.0


@dataclass
class TuningResult:
    kp_cross: float = 4.0
    kd_cross: float = 0.25
    kp_heading: float = 2.0
    kd_heading: float = 0.20
    kff_curvature: float = 0.90
    max_lateral_accel: float = 0.5
    max_accel: float = 0.4
    max_decel: float = 0.6
    lateral_step_response: StepResponse | None = None
    heading_step_response: StepResponse | None = None
    validation_avg_cte: float = 0.0
    validation_max_cte: float = 0.0
    validation_avg_speed: float = 0.0
    tuning_notes: list[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Tuner
# ---------------------------------------------------------------------------

class ControllerTuner:
    LATERAL_STEP_SIZE: float = 0.30
    HEADING_STEP_SIZE: float = 0.40
    STEP_DURATION: float = 3.0

    def __init__(self, control_frequency: float = 10.0) -> None:
        self._dt = 1.0 / control_frequency
        self._freq = control_frequency
        self._result: TuningResult | None = None
        self._validation_speeds: list[float] = []
        self._validation_ctes: list[float] = []
        self._live_recording = False
        self._live_times: list[float] = []
        self._live_lateral_errors: list[float] = []
        self._live_heading_errors: list[float] = []
        self._live_speeds: list[float] = []

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def tune_from_simulation(self, max_speed: float = 1.2, verbose: bool = True) -> TuningResult:
        """Full auto-calibration pipeline."""
        if verbose:
            print("=" * 60)
            print("ControllerTuner: auto-calibration")
            print("=" * 60)

        result = TuningResult()

        if verbose: print("\n[Phase 1] Lateral step response...")
        lat = self._sim_lateral_step(self.LATERAL_STEP_SIZE)
        _analyse_step_response(lat)
        result.lateral_step_response = lat
        if verbose: _print_step_response("Lateral", lat)

        if verbose: print("\n[Phase 2] Heading step response...")
        hdg = self._sim_heading_step(self.HEADING_STEP_SIZE)
        _analyse_step_response(hdg)
        result.heading_step_response = hdg
        if verbose: _print_step_response("Heading", hdg)

        if verbose: print("\n[Phase 3] Computing ITAE gains...")
        self._compute_gains(result, max_speed)
        if verbose: _print_gains(result)

        if verbose: print("\n[Phase 4] Validation on 1×1 m square...")
        self._validate(result, max_speed)
        if verbose:
            print(f"  avg CTE   = {result.validation_avg_cte:.4f} m")
            print(f"  max CTE   = {result.validation_max_cte:.4f} m")
            print(f"  avg speed = {result.validation_avg_speed:.3f} m/s")

        self._result = result
        if verbose: print("\n✓ Auto-calibration complete.")
        return result

    def apply_to(self, controller: "TrajectoryController") -> None:
        """Write tuned gains into a live TrajectoryController."""
        if self._result is None:
            raise RuntimeError("Run tune_from_simulation() first.")
        r = self._result
        pid = controller._pid
        pid.kp_cross      = r.kp_cross
        pid.kd_cross      = r.kd_cross
        pid.kp_heading    = r.kp_heading
        pid.kd_heading    = r.kd_heading
        pid.kff_curvature = r.kff_curvature
        p = controller._profiler
        p.max_lateral_accel = r.max_lateral_accel
        p.max_accel         = r.max_accel
        p.max_decel         = r.max_decel

    def plot_results(self, save_path: str = "/tmp/tuning_results.png") -> None:
        if self._result is None:
            raise RuntimeError("Run tune_from_simulation() first.")
        _plot_tuning_results(self._result, self._validation_speeds,
                             self._validation_ctes, save_path)
        print(f"Tuning report → {save_path}")

    def get_speed_vs_divergence(self) -> tuple[list[float], list[float]]:
        """Returns (speeds, ctes) from the 1×1 m validation run."""
        return self._validation_speeds, self._validation_ctes

    # ------------------------------------------------------------------
    # Live recording
    # ------------------------------------------------------------------

    def start_live_tuning(self) -> None:
        self._live_recording = True
        self._live_times.clear(); self._live_speeds.clear()
        self._live_lateral_errors.clear(); self._live_heading_errors.clear()

    def record_odom(self, speed: float, lateral_error: float,
                    heading_error: float, timestamp: float | None = None) -> None:
        if not self._live_recording: return
        self._live_times.append(timestamp or time.time())
        self._live_speeds.append(speed)
        self._live_lateral_errors.append(lateral_error)
        self._live_heading_errors.append(heading_error)

    def stop_live_tuning(self) -> TuningResult | None:
        self._live_recording = False
        if len(self._live_times) < 20: return None
        return self._tune_from_live_data()

    # ------------------------------------------------------------------
    # Simulation step tests
    # ------------------------------------------------------------------

    def _sim_lateral_step(self, step_size: float) -> StepResponse:
        """
        Simulate a lateral CTE step: robot starts step_size off the path,
        controller corrects it. Model as second-order underdamped response
        typical of a PD-controlled holonomic robot.
        """
        steps = int(self.STEP_DURATION / self._dt)
        times, values = [], []
        # Second-order system: natural freq ~3 rad/s, damping ~0.7 (slightly underdamped)
        wn = 3.0    # natural frequency rad/s
        zeta = 0.7  # damping ratio
        error = step_size  # start with full error
        d_error = 0.0
        for i in range(steps):
            # System response: error decays toward 0
            dd_error = -(2*zeta*wn*d_error + wn**2*error)
            d_error += dd_error * self._dt
            error   += d_error  * self._dt
            # Value = how much of the original step has been corrected
            values.append(step_size - max(error, 0.0))
            times.append(i * self._dt)
        return StepResponse(target=step_size, times=times, values=values)

    def _sim_heading_step(self, step_size: float) -> StepResponse:
        """
        Simulate heading step response using second-order model.
        wn=4 rad/s, zeta=0.75 — typical Go2 heading response.
        """
        steps = int(self.STEP_DURATION / self._dt)
        times, values = [], []
        wn = 4.0; zeta = 0.75
        error = step_size; d_error = 0.0
        for i in range(steps):
            dd_error = -(2*zeta*wn*d_error + wn**2*error)
            d_error += dd_error * self._dt
            error   += d_error  * self._dt
            values.append(step_size - max(min(error, step_size), 0.0))
            times.append(i * self._dt)
        return StepResponse(target=step_size, times=times, values=values)

    # ------------------------------------------------------------------
    # Gain computation (ITAE)
    # ------------------------------------------------------------------

    def _compute_gains(self, result: TuningResult, max_speed: float) -> None:
        lat = result.lateral_step_response
        hdg = result.heading_step_response
        notes = result.tuning_notes

        # Lateral
        if lat and lat.rise_time > 0:
            tau = max(lat.rise_time / 2.2, 0.05)
            K   = max(lat.steady_state_error, 0.3)
            result.kp_cross = float(np.clip(0.586 / max(K * tau**1.03, 1e-3), 2.0, 8.0))
            result.kd_cross = float(np.clip(0.232 / max(K * tau**0.771, 1e-3), 0.05, 0.5))
            notes.append(f"Lateral ITAE: τ={tau:.3f}s K={K:.3f} → kp={result.kp_cross:.2f} kd={result.kd_cross:.3f}")
        else:
            notes.append("Lateral: using defaults")

        # Heading
        if hdg and hdg.rise_time > 0:
            tau = max(hdg.rise_time / 2.2, 0.05)
            K   = max(hdg.steady_state_error, 0.3)
            result.kp_heading = float(np.clip(0.586 / max(K * tau**1.03, 1e-3), 1.0, 4.0))
            result.kd_heading = float(np.clip(0.232 / max(K * tau**0.771, 1e-3), 0.05, 0.5))
            notes.append(f"Heading ITAE: τ={tau:.3f}s K={K:.3f} → kp={result.kp_heading:.2f} kd={result.kd_heading:.3f}")
        else:
            notes.append("Heading: using defaults")

        # max_lateral_accel based on overshoot
        if lat:
            if lat.overshoot_pct > 20:
                result.max_lateral_accel = 0.3
                notes.append("High overshoot → max_lat_accel = 0.3 m/s²")
            elif lat.overshoot_pct < 5:
                result.max_lateral_accel = 0.6
                notes.append("Low overshoot → max_lat_accel = 0.6 m/s²")
            else:
                notes.append("Nominal overshoot → max_lat_accel = 0.5 m/s²")

        # kff_curvature
        result.kff_curvature = 1.0 if (hdg and hdg.rise_time < 0.3) else \
                               0.6 if (hdg and hdg.rise_time > 0.6) else 0.9

    # ------------------------------------------------------------------
    # Validation
    # ------------------------------------------------------------------

    def _validate(self, result: TuningResult, max_speed: float) -> None:
        from dimos.navigation.replanning_a_star.trajectory_controller import (
            HolonomicPIDController, VelocityProfiler,
            _advance_closest_index, _path_tangent,
        )
        from dimos.core.global_config import GlobalConfig
        from dimos.msgs.nav_msgs import Path
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

        # 1×1 m square
        square: list[tuple[float, float]] = []
        for x in np.linspace(0, 1, 8): square.append((float(x), 0.0))
        for y in np.linspace(0, 1, 8): square.append((1.0, float(y)))
        for x in np.linspace(1, 0, 8): square.append((float(x), 1.0))
        for y in np.linspace(1, 0, 8): square.append((0.0, float(y)))
        square.append((0.0, 0.0))

        poses = [PoseStamped(position=[x,y,0], orientation=[0,0,0,1]) for x,y in square]
        path = Path(); path.poses = poses
        pts  = np.array(square)

        gc = GlobalConfig(); gc.simulation = False
        profiler = VelocityProfiler(max_speed=max_speed, min_speed=0.15,
                                    max_lateral_accel=result.max_lateral_accel,
                                    max_accel=result.max_accel, max_decel=result.max_decel)
        profiler.build(path)

        pid = HolonomicPIDController(gc, self._freq)
        pid.kp_cross=result.kp_cross; pid.kd_cross=result.kd_cross
        pid.kp_heading=result.kp_heading; pid.kd_heading=result.kd_heading
        pid.kff_curvature=result.kff_curvature; pid.max_vy=1.0; pid.max_wz=2.0

        pos = np.array(pts[0], dtype=float)
        yaw = math.atan2(pts[1][1]-pts[0][1], pts[1][0]-pts[0][0])
        speeds_v: list[float] = []; ctes_v: list[float] = []
        cidx = 0

        def find_lh(pts: NDArray, idx: int, d: float) -> NDArray:
            dists = np.linalg.norm(pts[idx:] - pts[idx], axis=1)
            c = np.where(dists >= d)[0]
            return pts[-1] if len(c) == 0 else pts[idx+c[0]]

        for _ in range(2000):
            cidx = _advance_closest_index(pts, pos, cidx)
            ts   = profiler.speed_at(cidx)
            lh   = find_lh(pts, cidx, max(0.3, ts*0.35))
            odom = PoseStamped(position=[pos[0],pos[1],0],
                               orientation=[0,0,math.sin(yaw/2),math.cos(yaw/2)])
            tw   = pid.compute(lh, odom, ts, pts, cidx)
            vx,vy,wz = tw.linear.x, tw.linear.y, tw.angular.z
            speeds_v.append(math.sqrt(vx**2+vy**2))
            tang = _path_tangent(pts, cidx)
            norm = np.array([-tang[1], tang[0]])
            ctes_v.append(abs(float(np.dot(pos-pts[cidx], norm))))
            cy,sy = math.cos(yaw), math.sin(yaw)
            pos = pos + self._dt * np.array([vx*cy-vy*sy, vx*sy+vy*cy])
            yaw += self._dt * wz
            if np.linalg.norm(pos-pts[-1]) < 0.10: break

        result.validation_avg_cte   = float(np.mean(ctes_v))
        result.validation_max_cte   = float(np.max(ctes_v))
        result.validation_avg_speed = float(np.mean(speeds_v))
        self._validation_speeds = speeds_v
        self._validation_ctes   = ctes_v

    # ------------------------------------------------------------------
    # Live data tuning
    # ------------------------------------------------------------------

    def _tune_from_live_data(self) -> TuningResult:
        result = TuningResult()
        ctes   = np.array(self._live_lateral_errors)
        times  = np.array(self._live_times)
        speeds = np.array(self._live_speeds)
        peak_idx = int(np.argmax(np.abs(ctes)))
        peak_val = ctes[peak_idx]
        if abs(peak_val) > 0.01:
            target_37 = peak_val * 0.37
            later = ctes[peak_idx:]
            decay = np.where(np.abs(later) <= abs(target_37))[0]
            if len(decay):
                tau = float(times[peak_idx+decay[0]] - times[peak_idx])
                result.kp_cross = float(np.clip(0.9/max(tau,0.05), 2.0, 8.0))
                result.kd_cross = float(np.clip(result.kp_cross*tau*0.3, 0.05, 0.5))
                result.tuning_notes.append(f"Live lateral τ={tau:.3f}s")
        result.validation_avg_cte   = float(np.mean(np.abs(ctes)))
        result.validation_max_cte   = float(np.max(np.abs(ctes)))
        result.validation_avg_speed = float(np.mean(speeds))
        self._result = result
        return result


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _analyse_step_response(resp: StepResponse) -> None:
    if not resp.values or not resp.target: return
    vals = np.array(resp.values); t = np.array(resp.times); tgt = resp.target
    lo_idx = np.where(vals >= 0.1*tgt)[0]; hi_idx = np.where(vals >= 0.9*tgt)[0]
    if len(lo_idx) and len(hi_idx):
        resp.rise_time = float(t[hi_idx[0]] - t[lo_idx[0]])
    peak = float(np.max(vals))
    resp.overshoot_pct = max(0.0, (peak-tgt)/tgt*100) if tgt > 0 else 0.0
    band = 0.05*abs(tgt)
    out = np.where(np.abs(vals-tgt) > band)[0]
    resp.settling_time = float(t[out[-1]]) if len(out) else 0.0
    resp.steady_state_error = float(abs(np.mean(vals[int(0.8*len(vals)):]) - tgt))
    delay_idx = np.where(vals >= 0.05*tgt)[0]
    resp.delay = float(t[delay_idx[0]]) if len(delay_idx) else 0.0


def _print_step_response(name: str, r: StepResponse) -> None:
    print(f"  {name}: rise={r.rise_time:.3f}s  overshoot={r.overshoot_pct:.1f}%  "
          f"settle={r.settling_time:.3f}s  ss_err={r.steady_state_error:.4f}  delay={r.delay:.3f}s")


def _print_gains(r: TuningResult) -> None:
    print(f"  kp_cross={r.kp_cross:.3f}  kd_cross={r.kd_cross:.3f}  "
          f"kp_heading={r.kp_heading:.3f}  kd_heading={r.kd_heading:.3f}  "
          f"kff={r.kff_curvature:.3f}  max_lat_a={r.max_lateral_accel:.2f}")
    for n in r.tuning_notes: print(f"    {n}")


def _plot_tuning_results(
    result: TuningResult,
    val_speeds: list[float],
    val_ctes: list[float],
    save_path: str,
) -> None:
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec

    DARK='#0d1117'; GC='#21262d'; C1='#3fb950'; C2='#f85149'; C3='#58a6ff'

    def sax(ax: plt.Axes, title: str) -> None:
        ax.set_facecolor(DARK); ax.set_title(title, color='white', fontsize=10, pad=6)
        ax.tick_params(colors='#8b949e')
        for sp in ax.spines.values(): sp.set_color(GC)
        ax.grid(color=GC, linewidth=0.5)
        ax.xaxis.label.set_color('#8b949e'); ax.yaxis.label.set_color('#8b949e')

    fig = plt.figure(figsize=(16,10)); fig.patch.set_facecolor(DARK)
    gs  = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.38)

    # Lateral step response
    ax = fig.add_subplot(gs[0,0])
    if result.lateral_step_response:
        lat = result.lateral_step_response
        ax.plot(lat.times, lat.values, color=C1, lw=1.8, label='Response')
        ax.axhline(lat.target, color=C2, lw=1.2, ls='--', label='Target')
        ax.axhline(lat.target*1.05, color='#8b949e', lw=0.7, ls=':')
        ax.axhline(lat.target*0.95, color='#8b949e', lw=0.7, ls=':')
    sax(ax, 'Lateral Step Response')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Position (m)')
    ax.legend(fontsize=8, facecolor='#161b22', labelcolor='white')

    # Heading step response
    ax = fig.add_subplot(gs[1,0])
    if result.heading_step_response:
        hdg = result.heading_step_response
        ax.plot(hdg.times, hdg.values, color=C3, lw=1.8, label='Response')
        ax.axhline(hdg.target, color=C2, lw=1.2, ls='--', label='Target')
    sax(ax, 'Heading Step Response')
    ax.set_xlabel('Time (s)'); ax.set_ylabel('Heading (rad)')
    ax.legend(fontsize=8, facecolor='#161b22', labelcolor='white')

    # Speed vs divergence scatter
    ax = fig.add_subplot(gs[0,1])
    if val_speeds and val_ctes:
        ax.scatter(val_speeds, val_ctes, color=C1, s=6, alpha=0.6)
        # Trend line
        if len(val_speeds) > 5:
            z = np.polyfit(val_speeds, val_ctes, 1)
            xs = np.linspace(min(val_speeds), max(val_speeds), 50)
            ax.plot(xs, np.polyval(z, xs), color=C2, lw=1.5, ls='--', label='trend')
    sax(ax, 'Speed vs Divergence (issue metric)')
    ax.set_xlabel('Speed (m/s)'); ax.set_ylabel('|CTE| (m)')
    ax.legend(fontsize=8, facecolor='#161b22', labelcolor='white')

    # CTE over validation run
    ax = fig.add_subplot(gs[1,1])
    if val_ctes:
        ax.plot(val_ctes, color=C1, lw=1.5)
        ax.axhline(result.validation_avg_cte, color=C2, lw=1.2, ls='--',
                   label=f'avg={result.validation_avg_cte:.4f}m')
    sax(ax, '1×1m Square: CTE over time')
    ax.set_xlabel('Step'); ax.set_ylabel('|CTE| (m)')
    ax.legend(fontsize=8, facecolor='#161b22', labelcolor='white')

    # Gain summary
    ax = fig.add_subplot(gs[0,2]); ax.axis('off')
    gt = (f"  Tuned Gains\n  {'─'*28}\n"
          f"  kp_cross      = {result.kp_cross:.3f}\n"
          f"  kd_cross      = {result.kd_cross:.3f}\n"
          f"  kp_heading    = {result.kp_heading:.3f}\n"
          f"  kd_heading    = {result.kd_heading:.3f}\n"
          f"  kff_curvature = {result.kff_curvature:.3f}\n"
          f"  {'─'*28}\n"
          f"  max_lat_accel = {result.max_lateral_accel:.2f} m/s²\n"
          f"  max_accel     = {result.max_accel:.2f} m/s²\n"
          f"  max_decel     = {result.max_decel:.2f} m/s²\n"
          f"  {'─'*28}\n"
          f"  val_avg_CTE   = {result.validation_avg_cte:.4f} m\n"
          f"  val_max_CTE   = {result.validation_max_cte:.4f} m\n"
          f"  val_avg_speed = {result.validation_avg_speed:.3f} m/s")
    ax.text(0.05, 0.95, gt, transform=ax.transAxes, va='top',
            color='white', fontsize=9, fontfamily='monospace',
            bbox=dict(facecolor='#161b22', edgecolor=GC, boxstyle='round,pad=0.8'))

    # Notes
    ax = fig.add_subplot(gs[1,2]); ax.axis('off')
    nt = "  Tuning Notes\n  " + "─"*28 + "\n"
    for n in result.tuning_notes: nt += f"  • {n}\n"
    ax.text(0.05, 0.95, nt, transform=ax.transAxes, va='top',
            color='#8b949e', fontsize=9, fontfamily='monospace',
            bbox=dict(facecolor='#161b22', edgecolor=GC, boxstyle='round,pad=0.6'))

    fig.suptitle('ControllerTuner — Auto-Calibration Report',
                 color='white', fontsize=13, y=0.99)
    plt.savefig(save_path, dpi=150, bbox_inches='tight', facecolor=DARK)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser(description="Auto-tune TrajectoryController")
    p.add_argument("--speed", type=float, default=1.2)
    p.add_argument("--plot",  type=str,   default="/tmp/tuning_results.png")
    p.add_argument("--freq",  type=float, default=10.0)
    args = p.parse_args()

    tuner = ControllerTuner(control_frequency=args.freq)
    tuner.tune_from_simulation(max_speed=args.speed, verbose=True)
    tuner.plot_results(args.plot)