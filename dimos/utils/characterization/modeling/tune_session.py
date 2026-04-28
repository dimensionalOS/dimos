# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Session-level tuning orchestration.

Loads ``model_summary.json`` from a session, picks the pooled K/τ/L per
channel, runs the multiplier sweep + robustness checks, and writes the
tuning artifacts under ``<session>/modeling/tuning/``.

Per the Session 3 spec:

  - Tune against pooled K (single nominal point per channel).
  - Use rise model for the controller (closed-loop masks small plant
    asymmetries, so we don't switch models mid-trajectory).
  - Validate against ±15% K spread (run-to-run variance from Session 2).
  - 50 Hz control rate (matches harness scheduler).
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

from dimos.utils.characterization.modeling._io import atomic_write_json
from dimos.utils.characterization.modeling.closed_loop import (
    PIController, simulate_closed_loop,
)
from dimos.utils.characterization.modeling.fopdt import FopdtParams
from dimos.utils.characterization.modeling import references as refs
from dimos.utils.characterization.modeling.robustness import (
    gain_sweep, param_sweep,
)
from dimos.utils.characterization.modeling.tune import (
    TuningResult, default_scenarios, tune_channel,
)


# Default saturation limits (cmd-space) per channel. wz is firmware-limited
# at the actual rate output, but the cmd we send is bounded by the cmd
# range we used in characterization (E2 collected up to ±1.5).
_DEFAULT_SAT_LIMITS = {
    "vx": (-1.0, 1.0),
    "vy": (-1.0, 1.0),
    "wz": (-1.5, 1.5),
}

# Default control rate (50 Hz matches the harness scheduler).
_DEFAULT_CONTROL_DT_S = 0.02

# Default K-sweep range — Session 2's measured run-to-run variance.
_DEFAULT_K_SWEEP = (0.85, 1.15)

# Default τ / L uncertainty band for the model-uncertainty sweep.
_DEFAULT_PARAM_SWEEP = (0.80, 1.20)


def _plant_from_summary(
    summary: dict[str, Any], channel: str,
) -> FopdtParams | None:
    """Build a FopdtParams from the pooled rise model for one channel.

    Falls back to ``rise.channels[channel].pooled`` when present (newer
    summaries split rise/fall); otherwise reads ``channels[channel].pooled``
    directly (older summaries with rise as the headline).
    """
    rise = summary.get("rise") or summary
    ch = (rise.get("channels") or {}).get(channel)
    if not ch:
        return None
    pooled = ch.get("pooled") or {}
    K = (pooled.get("K") or {}).get("mean")
    tau = (pooled.get("tau") or {}).get("mean")
    L = (pooled.get("L") or {}).get("mean")
    if not all(v is not None and np.isfinite(float(v)) for v in (K, tau, L)):
        return None
    return FopdtParams(
        K=float(K), tau=float(tau), L=float(L),
        K_ci=(float(K), float(K)),
        tau_ci=(float(tau), float(tau)),
        L_ci=(float(L), float(L)),
        rmse=0.0, r_squared=1.0, n_samples=0,
        fit_window_s=(0.0, 0.0),
        degenerate=False, converged=True,
    )


def _validation_reference(plant: FopdtParams, sat_limits: tuple[float, float]):
    """One canonical reference signal used for the robustness sweeps."""
    return refs.step(amplitude=0.7 * sat_limits[1], t_start=0.5)


def tune_session(
    session_dir: Path,
    *,
    write_plots_enabled: bool = True,
    saturation_limits: dict[str, tuple[float, float]] | None = None,
    control_dt_s: float = _DEFAULT_CONTROL_DT_S,
) -> dict[str, Any]:
    """End-to-end per-channel tuning. Writes JSON + markdown + plots.

    Returns the in-memory tuning summary.
    """
    session_dir = Path(session_dir).expanduser().resolve()
    summary_path = session_dir / "modeling" / "model_summary.json"
    if not summary_path.exists():
        raise FileNotFoundError(
            f"{summary_path} not found — run `process_session fit` first.")
    summary = json.loads(summary_path.read_text())

    sat_limits = dict(_DEFAULT_SAT_LIMITS)
    if saturation_limits:
        sat_limits.update(saturation_limits)

    out_dir = session_dir / "modeling" / "tuning"
    out_dir.mkdir(parents=True, exist_ok=True)
    plots_dir = out_dir / "plots"

    rise_summary = summary.get("rise") or summary
    channels_summary = rise_summary.get("channels") or {}

    channel_results: dict[str, Any] = {}
    for channel in sorted(channels_summary):
        plant = _plant_from_summary(summary, channel)
        if plant is None:
            channel_results[channel] = {"skip_reason": "no pooled FOPDT params"}
            continue
        sat = sat_limits.get(channel, (-1.0, 1.0))

        # 1) Multiplier sweep.
        tr = tune_channel(
            plant, saturation_limits=sat,
            control_dt_s=control_dt_s,
        )
        winner = tr.best

        # 2) Robustness sweeps with the winning controller.
        ctrl = PIController(
            Kp=winner.gains.Kp, Ki=winner.gains.Ki, Kt=winner.gains.Kt,
            u_min=sat[0], u_max=sat[1],
        )
        ref_fn = _validation_reference(plant, sat)
        gs = gain_sweep(
            plant, ctrl, ref_fn,
            k_range=_DEFAULT_K_SWEEP,
            duration_s=4.0, control_dt_s=control_dt_s,
        )
        ctrl.reset()
        ts = param_sweep(
            plant, ctrl, ref_fn,
            which="tau",
            factor_range=_DEFAULT_PARAM_SWEEP,
            duration_s=4.0, control_dt_s=control_dt_s,
        )
        ctrl.reset()
        ls = param_sweep(
            plant, ctrl, ref_fn,
            which="L",
            factor_range=_DEFAULT_PARAM_SWEEP,
            duration_s=4.0, control_dt_s=control_dt_s,
        )

        channel_results[channel] = {
            "plant": {"K": plant.K, "tau": plant.tau, "L": plant.L},
            "saturation_limits": list(sat),
            "tuning": tr.asdict(),
            "robustness": {
                "K_sweep": gs.asdict(),
                "tau_sweep": ts.asdict(),
                "L_sweep": ls.asdict(),
            },
            "verdict": _channel_verdict(winner.cost_breakdown, gs, ts, ls),
        }

        if write_plots_enabled:
            try:
                _plot_channel(plots_dir, channel, plant, winner, sat,
                              gs, ts, ls, control_dt_s)
            except Exception as e:  # noqa: BLE001 — plot failure must not block
                channel_results[channel]["plot_error"] = (
                    f"{type(e).__name__}: {e}"
                )

    out = {
        "session_dir": str(session_dir),
        "control_dt_s": control_dt_s,
        "channels": channel_results,
        "thresholds": {
            "k_sweep_range": list(_DEFAULT_K_SWEEP),
            "tau_sweep_range": list(_DEFAULT_PARAM_SWEEP),
            "L_sweep_range": list(_DEFAULT_PARAM_SWEEP),
        },
    }

    atomic_write_json(out_dir / "tuning_summary.json", out)
    (out_dir / "tuning_report.md").write_text(_render_markdown(out))
    return out


def _channel_verdict(cost_breakdown, gs, ts, ls) -> str:
    """Pass/marginal/fail per channel based on robustness sweeps.

    pass:     all sweeps stable, mean overshoot ≤ 5%, max saturation ≤ 50%
    marginal: some non-zero saturation but still stable everywhere
    fail:     any sweep point unstable
    """
    if not (gs.all_stable and ts.all_stable and ls.all_stable):
        return "fail"
    overshoot = float(cost_breakdown.get("overshoot", 0.0))
    saturated = float(cost_breakdown.get("saturation_fraction", 0.0))
    if not np.isfinite(overshoot):
        overshoot = 0.0
    if overshoot <= 0.05 and saturated <= 0.50:
        return "pass"
    return "marginal"


# --------------------------------------------------------------------------- markdown

def _render_markdown(out: dict[str, Any]) -> str:
    lines: list[str] = []
    lines.append("# Tuning report")
    lines.append("")
    lines.append(f"- Session: `{out['session_dir']}`")
    lines.append(f"- Control rate: {1.0 / out['control_dt_s']:.0f} Hz "
                 f"(dt = {out['control_dt_s']} s)")
    lines.append("")
    lines.append("## Per channel")
    lines.append("")
    for channel, ch in sorted((out.get("channels") or {}).items()):
        lines.append(f"### {channel}")
        lines.append("")
        if "skip_reason" in ch:
            lines.append(f"- **skipped**: {ch['skip_reason']}")
            lines.append("")
            continue
        plant = ch["plant"]
        lines.append(f"- Plant: K={plant['K']:.4g}, τ={plant['tau']:.4g} s, "
                     f"L={plant['L']:.4g} s")
        lines.append(f"- Saturation: u ∈ [{ch['saturation_limits'][0]}, "
                     f"{ch['saturation_limits'][1]}]")
        winner = (ch.get("tuning") or {}).get("best_gains") or {}
        lines.append(f"- **Chosen gains**: Kp={winner.get('Kp', 0.0):.4f}, "
                     f"Ki={winner.get('Ki', 0.0):.4f}, "
                     f"Kt={winner.get('Kt', 0.0):.4f} (λ-multiplier "
                     f"= {winner.get('multiplier', 0.0):.2f})")
        lines.append(f"- Robustness verdict: **{ch.get('verdict', '?').upper()}**")
        lines.append("")
        lines.append("#### Multiplier sweep")
        lines.append("")
        lines.append("| multiplier | λ (s) | Kp | Ki | total cost | mean IAE | mean overshoot | mean settle |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for c in (ch.get("tuning") or {}).get("candidates") or []:
            g = c.get("gains") or {}
            br = c.get("cost_breakdown") or {}
            lines.append(
                "| {m:.2f} | {lam:.3f} | {Kp:.3f} | {Ki:.3f} | {cost:.3f} | "
                "{iae:.3f} | {ovr:.1%} | {set:.2f} s |".format(
                    m=g.get("multiplier", 0.0), lam=g.get("lambda_s", 0.0),
                    Kp=g.get("Kp", 0.0), Ki=g.get("Ki", 0.0),
                    cost=c.get("cost", 0.0),
                    iae=br.get("iae", 0.0),
                    ovr=br.get("overshoot", 0.0),
                    **{"set": br.get("settle_time_s", 0.0)},
                )
            )
        lines.append("")
        lines.append("#### Robustness — K sweep (run-to-run variance)")
        lines.append("")
        gs = (ch.get("robustness") or {}).get("K_sweep") or {}
        lines.append(f"- All stable: **{gs.get('all_stable')}**")
        lines.append(f"- Worst IAE: {gs.get('worst_iae', 0.0):.3f}")
        lines.append("")
        lines.append("| label | K | overshoot | settle | sat fraction | stable |")
        lines.append("|---|---|---|---|---|---|")
        for p in gs.get("points") or []:
            m = p.get("metrics") or {}
            lines.append(
                "| {l} | {K:.3f} | {ovr:.1%} | {st:.2f} s | {sat:.0%} | {stab} |".format(
                    l=p.get("label", ""), K=(p.get("plant") or {}).get("K", 0.0),
                    ovr=m.get("overshoot", 0.0),
                    st=m.get("settle_time_s", 0.0),
                    sat=m.get("saturation_fraction", 0.0),
                    stab=p.get("stable", "?"),
                )
            )
        lines.append("")
        for which, label in (("tau_sweep", "τ"), ("L_sweep", "L")):
            sw = (ch.get("robustness") or {}).get(which) or {}
            lines.append(f"#### Robustness — {label} sweep (model uncertainty)")
            lines.append("")
            lines.append(f"- All stable: **{sw.get('all_stable')}**")
            lines.append(f"- Worst IAE: {sw.get('worst_iae', 0.0):.3f}")
            lines.append("")
    return "\n".join(lines) + "\n"


# --------------------------------------------------------------------------- plots

def _plot_channel(
    plots_dir: Path, channel: str, plant: FopdtParams, winner,
    sat: tuple[float, float], gs, ts, ls, control_dt_s: float,
) -> None:
    """Step + staircase + K-sweep plots for one channel."""
    plots_dir.mkdir(parents=True, exist_ok=True)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # Re-run scenarios with the winner so we can plot trajectories.
    scenarios = default_scenarios(sat)
    fig, axes = plt.subplots(len(scenarios), 1, figsize=(12, 2.6 * len(scenarios)),
                              sharex=False)
    if len(scenarios) == 1:
        axes = [axes]
    for ax, (label, ref_fn, dur) in zip(axes, scenarios):
        ctrl = PIController(
            Kp=winner.gains.Kp, Ki=winner.gains.Ki, Kt=winner.gains.Kt,
            u_min=sat[0], u_max=sat[1],
        )
        sim = simulate_closed_loop(
            plant, ctrl, ref_fn,
            duration_s=dur, control_dt_s=control_dt_s,
        )
        ax.plot(sim.t, sim.r, "k-", lw=1.0, alpha=0.55, label="reference")
        ax.plot(sim.t, sim.y, color="tab:blue", lw=1.5, label="output y")
        ax.plot(sim.t, sim.u, color="tab:red", lw=1.0, alpha=0.7, label="control u")
        ax.axhline(sat[0], color="grey", lw=0.5, ls=":")
        ax.axhline(sat[1], color="grey", lw=0.5, ls=":")
        m = sim.metrics
        ax.set_title(
            f"{channel}  -  {label}  -  IAE={m.get('iae', 0):.3f}, "
            f"overshoot={m.get('overshoot', 0):.1%}, "
            f"settle={m.get('settle_time_s', 0):.2f}s",
            fontsize=10,
        )
        ax.set_xlabel("t (s)")
        ax.legend(loc="best", fontsize=8)
        ax.grid(alpha=0.3)
    fig.suptitle(
        f"{channel}: closed-loop response with chosen gains "
        f"(Kp={winner.gains.Kp:.3f}, Ki={winner.gains.Ki:.3f}, "
        f"λ-mult={winner.gains.multiplier:.2f})",
        fontsize=11,
    )
    fig.tight_layout()
    fig.savefig(plots_dir / f"{channel}__step_responses.svg")
    plt.close(fig)

    # K-sweep plot: tracking error vs K and overshoot vs K.
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    Ks = [p.plant["K"] for p in gs.points]
    iaes = [p.metrics.get("iae", 0.0) for p in gs.points]
    overs = [p.metrics.get("overshoot", 0.0) for p in gs.points]
    axes[0].plot(Ks, iaes, "o-", color="tab:blue")
    axes[0].set_xlabel("plant K")
    axes[0].set_ylabel("IAE")
    axes[0].set_title(f"{channel}: K-sweep IAE (controller fixed)")
    axes[0].grid(alpha=0.3)
    axes[1].plot(Ks, overs, "o-", color="tab:orange")
    axes[1].axhline(0.05, color="red", lw=1, ls="--", label="5% target")
    axes[1].set_xlabel("plant K")
    axes[1].set_ylabel("overshoot")
    axes[1].yaxis.set_major_formatter(plt.matplotlib.ticker.PercentFormatter(1.0))
    axes[1].set_title(f"{channel}: K-sweep overshoot")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(plots_dir / f"{channel}__K_sweep.svg")
    plt.close(fig)


__all__ = ["tune_session"]
