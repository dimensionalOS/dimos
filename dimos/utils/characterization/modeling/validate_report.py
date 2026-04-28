# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Markdown + plots for the Session 2 validation report."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from dimos.utils.characterization.modeling.validate_run import ValidationResult


# --------------------------------------------------------------------------- markdown

def render_validation_markdown(
    *,
    session_dir: Path | None,
    mode: str,
    summary: dict[str, Any],
    diagnosis: dict[str, Any] | None,
    n_train: int,
    n_validate: int,
) -> str:
    """Human-readable validation report.

    Sections:
      - Headline verdict (per channel)
      - Summary table
      - Per (channel, amplitude, direction) breakdown
      - Diagnosis findings (if any)
      - Recommendation for Session 3
    """
    lines: list[str] = []
    lines.append("# Validation report")
    lines.append("")
    if session_dir is not None:
        lines.append(f"- Session: `{session_dir}`")
    lines.append(f"- Mode: `{mode}`")
    lines.append(f"- Training runs: {n_train} (forward direction)")
    lines.append(f"- Validation runs: {n_validate} (held out — reverse direction)")
    lines.append(f"- Total runs validated: {summary.get('n_runs_total', 0)}")
    lines.append(f"  - pass: {summary.get('n_runs_pass', 0)}")
    lines.append(f"  - marginal: {summary.get('n_runs_marginal', 0)}")
    lines.append(f"  - fail: {summary.get('n_runs_fail', 0)}")
    lines.append(f"  - skip: {summary.get('n_runs_skip', 0)}")
    lines.append("")

    # Headline.
    lines.append("## Verdict")
    lines.append("")
    channels = summary.get("channels") or {}
    overall = _overall_verdict(channels)
    lines.append(f"**Overall: {overall.upper()}**")
    lines.append("")
    for ch, cs in sorted(channels.items()):
        verdict = cs.get("verdict", "skip")
        n = cs.get("n_total", 0)
        pr = cs.get("pass_rate", float("nan"))
        rise_med = (cs.get("rise_norm_rmse") or {}).get("median", float("nan"))
        lines.append(
            f"- **{ch}**: {verdict.upper()} — {cs.get('n_pass', 0)}/{n} pass "
            f"({_pct(pr)}), median rise nRMSE = {_pct(rise_med)}"
        )
    lines.append("")

    # Summary table — verdict-driving metric (smoothed nRMSE) headlines,
    # raw nRMSE shown alongside for traceability, residual-over-noise
    # tells you whether the model is at the channel's noise ceiling.
    lines.append("## Summary table")
    lines.append("")
    lines.append("Verdict driver: **smoothed** nRMSE = "
                 "RMSE(Sav-Gol(meas-pred)) / |amp|. "
                 "`raw` is the un-smoothed equivalent. "
                 "`r/σ` is rmse_smoothed / noise_floor — values ≲ 2 mean "
                 "the model is at the noise ceiling for that channel.")
    lines.append("")
    lines.append("| channel | verdict | pass | marginal | fail | skip | "
                 "rise nRMSE (smooth, med) | rise nRMSE (raw, med) | "
                 "fall nRMSE (smooth, med) | rise r/σ (med) |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|")
    for ch, cs in sorted(channels.items()):
        rs = cs.get("rise_norm_rmse") or {}
        fs = cs.get("fall_norm_rmse") or {}
        rs_raw = cs.get("rise_norm_rmse_raw") or {}
        ron = cs.get("rise_residual_over_noise") or {}
        lines.append(
            "| {ch} | {v} | {p} | {m} | {f} | {s} | {rm} | {rmr} | {fm} | {ron} |".format(
                ch=ch, v=cs.get("verdict", "skip"),
                p=cs.get("n_pass", 0), m=cs.get("n_marginal", 0),
                f=cs.get("n_fail", 0), s=cs.get("n_skip", 0),
                rm=_pct(rs.get("median")),
                rmr=_pct(rs_raw.get("median")) if rs_raw else "—",
                fm=_pct(fs.get("median")) if fs else "—",
                ron=_num(ron.get("median")) if ron else "—",
            )
        )
    lines.append("")

    # Per (channel, amp, direction).
    lines.append("## Per (channel, amplitude, direction)")
    lines.append("")
    for ch, cs in sorted(channels.items()):
        lines.append(f"### {ch}")
        lines.append("")
        lines.append("| amplitude | direction | n | pass | rise nRMSE (med) | fall nRMSE (med) | worst |")
        lines.append("|---|---|---|---|---|---|---|")
        for g in cs.get("by_amp_direction") or []:
            key = g.get("key") or {}
            rn = g.get("rise_norm_rmse") or {}
            fn = g.get("fall_norm_rmse") or {}
            worst = ", ".join(g.get("worst_run_ids") or [])
            lines.append(
                "| {amp:+.2f} | {dir} | {n} | {p}/{n} | {rm} | {fm} | {w} |".format(
                    amp=float(key.get("amplitude", 0.0)),
                    dir=key.get("direction", ""),
                    n=g.get("n_total", 0),
                    p=g.get("n_pass", 0),
                    rm=_pct(rn.get("median")),
                    fm=_pct(fn.get("median")) if fn else "—",
                    w=worst,
                )
            )
        lines.append("")

    # Diagnosis — only render the section if at least one channel
    # produced at least one finding.
    diag_channels = (diagnosis or {}).get("channels") or {}
    has_any_finding = any(
        (d.get("findings") or []) for d in diag_channels.values()
    )
    if has_any_finding:
        lines.append("## Diagnosis (marginal/failing channels only)")
        lines.append("")
        for ch, d in sorted(diag_channels.items()):
            findings = d.get("findings") or []
            if not findings:
                continue
            lines.append(f"### {ch}")
            lines.append("")
            for f in findings:
                lines.append(f"- **{f.get('pattern')}** ({f.get('severity')})")
                lines.append(f"  - {f.get('recommendation', '')}")
                ev = f.get("evidence") or {}
                if ev:
                    parts = [f"{k}={_fmt(v)}" for k, v in ev.items()]
                    lines.append(f"  - Evidence: {', '.join(parts)}")
            lines.append("")

    # Recommendation.
    lines.append("## Ready for Session 3?")
    lines.append("")
    lines.append(_recommendation(overall, channels))
    lines.append("")
    lines.append("## Thresholds used")
    lines.append("")
    th = summary.get("thresholds") or {}
    for k, v in th.items():
        lines.append(f"- `{k}`: {v}")
    lines.append("")

    return "\n".join(lines) + "\n"


def _overall_verdict(channels: dict[str, Any]) -> str:
    """Worst per-channel verdict wins."""
    order = {"pass": 0, "marginal": 1, "fail": 2, "skip": 3}
    worst = "pass"
    for cs in channels.values():
        v = cs.get("verdict", "skip")
        if order.get(v, 3) > order.get(worst, 0):
            worst = v
    return worst


def _recommendation(overall: str, channels: dict[str, Any]) -> str:
    if overall == "pass":
        return ("FOPDT generalizes within thresholds across direction. The "
                "fitted model is ready to feed Session 3 (closed-loop "
                "simulator + lambda-tuning).")
    if overall == "marginal":
        return ("FOPDT works but with caveats — see the diagnosis section "
                "for which channels need attention. Either narrow the "
                "operating envelope (e.g. cap amplitude) or upgrade the "
                "model along the highest-severity finding before Session 3.")
    return ("FOPDT does not generalize within thresholds. See the diagnosis "
            "section for the upgrade path. Do not proceed to Session 3 with "
            "the current model.")


def _pct(v: Any) -> str:
    try:
        f = float(v)
        if not np.isfinite(f):
            return "—"
        return f"{f:.1%}"
    except (TypeError, ValueError):
        return "—"


def _num(v: Any) -> str:
    try:
        f = float(v)
        if not np.isfinite(f):
            return "—"
        return f"{f:.2f}"
    except (TypeError, ValueError):
        return "—"


def _fmt(v: Any) -> str:
    if isinstance(v, float):
        if not np.isfinite(v):
            return "nan"
        return f"{v:.4g}"
    if isinstance(v, (tuple, list)):
        return "(" + ", ".join(_fmt(x) for x in v) + ")"
    return str(v)


# --------------------------------------------------------------------------- plots

def write_validation_plots(
    *,
    plots_dir: Path,
    results: list[ValidationResult],
    summary: dict[str, Any],
    n_per_channel: int = 6,
) -> list[Path]:
    """Per-channel best/median/worst overlay plots + nRMSE distribution.

    Best-effort: failures don't block the rest of the pipeline.
    """
    plots_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return written

    by_channel: dict[str, list[ValidationResult]] = {}
    for r in results:
        if r.channel and r._t_meas is not None:
            by_channel.setdefault(r.channel, []).append(r)

    for channel, ch_results in by_channel.items():
        # Pick best, median, worst by rise nRMSE.
        scored: list[tuple[float, ValidationResult]] = []
        for r in ch_results:
            nr = (r.rise_metrics or {}).get("norm_rmse")
            if nr is None or not np.isfinite(nr):
                continue
            scored.append((float(nr), r))
        if not scored:
            continue
        scored.sort(key=lambda x: x[0])
        picks = _pick_best_median_worst(scored, n_per_channel)
        for label, (nrmse, r) in picks:
            try:
                p = plots_dir / f"{channel}__{label}__{r.run_id}__overlay.svg"
                _plot_overlay(p, r, label=label, plt=plt)
                written.append(p)
            except Exception:
                continue

        # Distribution plot.
        try:
            p = plots_dir / f"{channel}__norm_rmse_distribution.svg"
            _plot_distribution(p, channel, ch_results, plt=plt)
            written.append(p)
        except Exception:
            pass

    return written


def _pick_best_median_worst(
    scored: list[tuple[float, ValidationResult]], n: int
) -> list[tuple[str, tuple[float, ValidationResult]]]:
    """Best, median, and worst-N from a sorted-ascending list."""
    out: list[tuple[str, tuple[float, ValidationResult]]] = []
    if not scored:
        return out
    out.append(("best", scored[0]))
    mid_idx = len(scored) // 2
    out.append(("median", scored[mid_idx]))
    # Take up to (n-2) worst, but skip duplicates of best/median.
    remaining = max(0, n - 2)
    for nrmse, r in reversed(scored):
        if (nrmse, r) in (out[0][1], out[1][1]):
            continue
        out.append((f"worst_{r.run_id}", (nrmse, r)))
        remaining -= 1
        if remaining <= 0:
            break
    return out


def _plot_overlay(path: Path, r: ValidationResult, *, label: str, plt) -> None:
    fig, axes = plt.subplots(
        3, 1, figsize=(9, 7), sharex=True,
        gridspec_kw={"height_ratios": [1, 2.5, 1]},
    )
    ax_cmd, ax_y, ax_res = axes
    t = r._t_meas
    cmd = r._cmd_at_meas if r._cmd_at_meas is not None else np.zeros_like(t)
    y_meas = r._y_meas
    y_pred = r._y_pred

    ax_cmd.plot(t, cmd, "k-", lw=1.0)
    ax_cmd.set_ylabel("cmd")
    ax_cmd.grid(alpha=0.3)
    ax_cmd.set_title(
        f"{r.recipe} — {r.run_id} ({label}) — verdict: {r.verdict}"
    )

    ax_y.plot(t, y_meas, "b.", ms=2.5, alpha=0.55, label="measured")
    ax_y.plot(t, y_pred, "r-", lw=1.5, label="predicted (FOPDT)")
    ax_y.set_ylabel("y (baseline-subtracted)")
    ax_y.legend(loc="best", fontsize=8)
    ax_y.grid(alpha=0.3)

    res = y_meas - y_pred
    ax_res.plot(t, res, "g-", lw=0.8)
    ax_res.axhline(0, color="k", lw=0.5)
    ax_res.set_ylabel("residual")
    ax_res.set_xlabel("t (s)")
    ax_res.grid(alpha=0.3)

    rise = r.rise_metrics or {}
    fall = r.fall_metrics or {}
    info = []
    if rise:
        info.append(f"rise nRMSE = {rise.get('norm_rmse', float('nan')):.2%}")
    if fall:
        info.append(f"fall nRMSE = {fall.get('norm_rmse', float('nan')):.2%}")
    info.append(f"K={r.used_K:.3g}, τ={r.used_tau:.3g}, L={r.used_L:.3g}")
    ax_res.text(0.02, 0.85, "  ".join(info), transform=ax_res.transAxes,
                fontsize=9, family="monospace",
                bbox={"facecolor": "white", "alpha": 0.7, "lw": 0})

    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _plot_distribution(
    path: Path, channel: str, results: list[ValidationResult], *, plt
) -> None:
    fig, ax = plt.subplots(1, 1, figsize=(9, 5))
    by_dir: dict[str, list[float]] = {"forward": [], "reverse": []}
    for r in results:
        nr = (r.rise_metrics or {}).get("norm_rmse")
        if nr is None or not np.isfinite(nr) or r.direction is None:
            continue
        by_dir.setdefault(r.direction, []).append(float(nr))

    bins = np.linspace(0, max(0.3, max((max(v) if v else 0) for v in by_dir.values()) * 1.05), 25)
    for direction, color in (("forward", "tab:blue"), ("reverse", "tab:orange")):
        v = by_dir.get(direction) or []
        if not v:
            continue
        ax.hist(v, bins=bins, alpha=0.55, label=f"{direction} (n={len(v)})", color=color)

    ax.axvline(0.10, color="green", linestyle="--", lw=1, label="pass (10%)")
    ax.axvline(0.20, color="orange", linestyle="--", lw=1, label="marginal (20%)")
    ax.set_xlabel("rise normalized RMSE = RMSE / |amplitude|")
    ax.set_ylabel("count")
    ax.set_title(f"{channel}: validation rise-nRMSE distribution")
    ax.legend(loc="best", fontsize=9)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


__all__ = ["render_validation_markdown", "write_validation_plots"]
