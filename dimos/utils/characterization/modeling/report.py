# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Markdown report + matplotlib overlay plots for FOPDT modeling.

Two plot families:
  - per-group overlay: cmd step + measured response + fitted FOPDT curve
    + residuals subplot. One SVG per recipe group.
  - per-channel parameter-vs-amplitude: K/τ/L vs |amplitude|, with CI
    error bars and forward/reverse colored separately.

Markdown report has four sections: per-channel summary, per-cell table,
pooling decisions, diagnostics.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from dimos.utils.characterization.modeling.aggregate import GroupFit
from dimos.utils.characterization.modeling.per_run import RunFit


# --------------------------------------------------------------------------- markdown

def render_markdown(
    *,
    session_dir: Path,
    mode: str,
    summary: dict[str, Any],
    group_fits: list[GroupFit],
    run_fits: list[RunFit],
    fall_groups: list[GroupFit] | None = None,
) -> str:
    """Build the modeling report markdown."""
    lines: list[str] = []
    lines.append(f"# FOPDT Plant Model — {session_dir.name}")
    lines.append("")
    lines.append(f"- Session: `{session_dir}`")
    lines.append(f"- Mode: **{mode}**")
    lines.append(f"- Total runs: {len(run_fits)}")
    skipped = sum(1 for r in run_fits if r.skip_reason is not None)
    failed_fit = sum(
        1 for r in run_fits
        if r.skip_reason is None and (r.params is None or not r.params.converged)
    )
    degenerate = sum(
        1 for r in run_fits
        if r.params is not None and r.params.converged and r.params.degenerate
    )
    lines.append(f"- Skipped (non-step / unparseable): {skipped}")
    lines.append(f"- Failed fit: {failed_fit}")
    lines.append(f"- Degenerate (singular covariance): {degenerate}")
    lines.append("")

    # Per-channel summary.
    lines.append("## Per-channel summary")
    lines.append("")
    for channel, ch in sorted(summary.get("channels", {}).items()):
        lines.append(f"### `{channel}`")
        pooled = ch.get("pooled", {})
        for p in ("K", "tau", "L"):
            stats = pooled.get(p) or {}
            mean = _fmt(stats.get("mean"))
            lo = _fmt(stats.get("ci_low"))
            hi = _fmt(stats.get("ci_high"))
            n = stats.get("n_groups")
            lines.append(f"- **{p}** = {mean}  (95% CI [{lo}, {hi}]; n_groups={n})")
        if ch.get("direction_asymmetric"):
            lines.append("- Direction asymmetric — forward and reverse fits disagree at one or more amplitudes.")
        else:
            lines.append("- Forward / reverse pooled (CIs overlapped at every amplitude).")
        lin = ch.get("linear_in_amplitude") or {}
        nonlinear = [p for p, v in lin.items() if not v]
        if nonlinear:
            lines.append(
                f"- Gain-scheduled (slope CI excludes zero) for: {', '.join(nonlinear)}. "
                f"See `gain_schedule` in `model_summary.json`."
            )
        else:
            lines.append("- Linear-in-amplitude on all parameters (slope CIs include zero).")
        lines.append("")

    # Per-cell table.
    lines.append("## Per-cell results")
    lines.append("")
    lines.append("| recipe | channel | amp | direction | K (95% CI) | τ (95% CI) | L (95% CI) | kept / input | rejected (2σ) |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for g in sorted(group_fits, key=lambda gg: (gg.key.get("channel") or "", gg.key.get("amplitude") or 0)):
        recipe = g.key.get("recipe", "?")
        ch = g.key.get("channel", "?")
        amp = _fmt(g.key.get("amplitude"))
        direction = g.key.get("direction", "?")
        K_cell = _fmt_param_cell(g.K)
        tau_cell = _fmt_param_cell(g.tau)
        L_cell = _fmt_param_cell(g.L)
        rejected = len(g.rejected_run_ids)
        lines.append(
            f"| `{recipe}` | {ch} | {amp} | {direction} | {K_cell} | {tau_cell} | {L_cell} | "
            f"{g.n_runs_kept}/{g.n_runs_input} | {rejected} |"
        )
    lines.append("")

    # Pooling decisions.
    lines.append("## Pooling decisions")
    lines.append("")
    for channel, ch in sorted(summary.get("channels", {}).items()):
        lines.append(f"- `{channel}`: direction_asymmetric={ch.get('direction_asymmetric')}, "
                     f"linear_in_amplitude={ch.get('linear_in_amplitude')}")
    lines.append("")

    # Diagnostics.
    diag = summary.get("diagnostics", {})
    lines.append("## Diagnostics")
    lines.append("")
    for k, v in diag.items():
        lines.append(f"- {k}: {v}")
    lines.append("")

    # Step-down dynamics. Two parts:
    #   1. Per-channel pooled fall fit summary.
    #   2. Rise vs fall comparison (asymmetry verdict per param).
    fall_summary = summary.get("fall") or {}
    rvf = summary.get("rise_vs_fall") or {}
    if fall_summary or rvf:
        lines.append("## Step-down (decel) dynamics")
        lines.append("")
        for channel, fch in sorted((fall_summary.get("channels") or {}).items()):
            pooled = fch.get("pooled", {})
            lines.append(f"### `{channel}` — fall fit")
            for p in ("K", "tau", "L"):
                stats = pooled.get(p) or {}
                lines.append(
                    f"- **{p}** = {_fmt(stats.get('mean'))}  "
                    f"(95% CI [{_fmt(stats.get('ci_low'))}, {_fmt(stats.get('ci_high'))}]; "
                    f"n_groups={stats.get('n_groups')})"
                )
            lines.append("")

        if rvf.get("channels"):
            lines.append("### Rise vs fall comparison")
            lines.append("")
            lines.append("| channel | param | rise mean | fall mean | ratio fall/rise | CI overlap | verdict |")
            lines.append("|---|---|---|---|---|---|---|")
            for channel, cv in sorted(rvf["channels"].items()):
                for p in ("K", "tau", "L"):
                    pv = cv.get("params", {}).get(p, {})
                    rise = pv.get("rise") or {}
                    fall = pv.get("fall") or {}
                    lines.append(
                        f"| {channel} | {p} | {_fmt(rise.get('mean'))} | {_fmt(fall.get('mean'))} | "
                        f"{_fmt(pv.get('ratio_fall_over_rise'))} | {pv.get('ci_overlap')} | "
                        f"{pv.get('verdict')} |"
                    )
            lines.append("")
            lines.append(
                "Verdicts: `identical` (<5% of mean), `equivalent` (CIs overlap or "
                "<20% of mean), `differs` (otherwise). A `differs` row on **τ** means "
                "the plant decelerates at a different rate than it accelerates — "
                "common on legged plants and worth knowing for controller design."
            )
            lines.append("")

    return "\n".join(lines) + "\n"


def _fmt(v: float | None, digits: int = 4) -> str:
    if v is None:
        return "—"
    try:
        if not np.isfinite(v):
            return "nan"
    except TypeError:
        return str(v)
    return f"{v:.{digits}g}"


def _fmt_param_cell(stats: dict[str, Any] | None) -> str:
    if stats is None:
        return "—"
    return f"{_fmt(stats.get('mean'))} [{_fmt(stats.get('ci_low'))}, {_fmt(stats.get('ci_high'))}]"


# --------------------------------------------------------------------------- plots

def write_plots(
    *,
    plots_dir: Path,
    summary: dict[str, Any],
    group_fits: list[GroupFit],
    run_fits: list[RunFit],
    fall_groups: list[GroupFit] | None = None,
) -> list[Path]:
    """Write overlay + parameter plots. Returns the list of paths written.

    Plots are best-effort — failure to render any single plot doesn't
    block the rest of the pipeline.
    """
    plots_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return written

    # Group RunFits by recipe so the overlay can show every repeat at once.
    by_recipe: dict[str, list[RunFit]] = {}
    for rf in run_fits:
        if rf.recipe == "<unknown>" or rf.skip_reason is not None:
            continue
        by_recipe.setdefault(rf.recipe, []).append(rf)

    by_recipe_group = {g.key.get("recipe"): g for g in group_fits}
    by_recipe_fall = {g.key.get("recipe"): g for g in (fall_groups or [])}

    for recipe, fits in by_recipe.items():
        try:
            path = plots_dir / f"{_safe(recipe)}__overlay.svg"
            _plot_overlay(path, recipe, fits, by_recipe_group.get(recipe), plt, edge="rise")
            written.append(path)
        except Exception:
            continue
        if fall_groups is not None:
            try:
                path = plots_dir / f"{_safe(recipe)}__overlay_down.svg"
                _plot_overlay(path, recipe, fits, by_recipe_fall.get(recipe), plt, edge="fall")
                written.append(path)
            except Exception:
                continue

    for channel, ch in summary.get("channels", {}).items():
        try:
            path = plots_dir / f"{_safe(channel)}__params_vs_amp.svg"
            _plot_params_vs_amp(path, channel, ch, plt)
            written.append(path)
        except Exception:
            continue

    return written


def _safe(s: Any) -> str:
    return "".join(c if (c.isalnum() or c in "+-._") else "_" for c in str(s))


def _plot_overlay(
    path: Path, recipe: str, fits: list[RunFit], group: GroupFit | None, plt,
    *, edge: str = "rise",
) -> None:
    """Per-recipe overlay: measured trace + group-mean FOPDT model + RMSE bars.

    ``edge`` selects which fit to draw — ``"rise"`` (cmd 0 → amplitude) or
    ``"fall"`` (cmd amplitude → 0). The fall variant uses ``params_down``
    and ``extra_down`` so we share one plotting function for both phases.
    """
    from dimos.utils.characterization.modeling.fopdt import fopdt_step_response
    from dimos.utils.characterization.scripts.analyze_run import (
        _channel_arrays,
        load_run,
    )

    fig, (ax_main, ax_resid) = plt.subplots(
        2, 1, figsize=(8, 5.5), gridspec_kw={"height_ratios": [3, 1]}, sharex=False
    )

    K_mean = (group.K or {}).get("mean") if group else None
    tau_mean = (group.tau or {}).get("mean") if group else None
    L_mean = (group.L or {}).get("mean") if group else None

    u_step = None
    channel = None
    plotted_meas = 0
    for rf in fits:
        if rf.amplitude is not None:
            u_step = rf.amplitude if edge == "rise" else -rf.amplitude
        if rf.channel is not None:
            channel = rf.channel
        edge_params = rf.params if edge == "rise" else rf.params_down
        edge_extra = rf.extra if edge == "rise" else rf.extra_down
        if rf.skip_reason is not None or edge_params is None or rf.channel is None:
            continue
        try:
            run = load_run(Path(rf.run_dir))
        except Exception:
            continue
        try:
            cmd_arr, meas_arr = _channel_arrays(run, rf.channel)
        except Exception:
            continue
        if edge == "rise":
            edge_t = float(edge_extra.get("step_t", 0.0))
            baseline = float(edge_extra.get("baseline", 0.0))
            window_end = float(edge_extra.get("active_end_t", edge_t + 4.0))
        else:
            edge_t = float(edge_extra.get("active_end_t", 0.0))
            baseline = float(edge_extra.get("baseline_down", 0.0))
            window_end = float(edge_extra.get("fall_end_t", edge_t + 1.0))
        meas_ts = run.meas_ts_rel
        mask = (meas_ts >= edge_t - 0.3) & (meas_ts <= window_end)
        if int(mask.sum()) >= 2:
            ax_main.plot(
                meas_ts[mask] - edge_t,
                meas_arr[mask] - baseline,
                color="#888", alpha=0.35, linewidth=0.8,
                label=("measured (per-repeat)" if plotted_meas == 0 else None),
            )
            plotted_meas += 1

    if K_mean is not None and tau_mean is not None and L_mean is not None and u_step is not None:
        t_max = 4.0 if edge == "rise" else 1.5
        t_grid = np.linspace(-0.3, t_max, 400)
        y_model = fopdt_step_response(t_grid, K_mean, tau_mean, L_mean, u_step)
        ax_main.plot(t_grid, y_model, color="#000", linewidth=2.0, label="FOPDT (group mean)")
        ax_main.axhline(K_mean * u_step, color="#888", linestyle="--", linewidth=0.6, label="K·u_step")

    for rf in fits:
        edge_params = rf.params if edge == "rise" else rf.params_down
        if rf.skip_reason is not None or edge_params is None or not edge_params.converged:
            continue
        u = (rf.amplitude or 0.0) if edge == "rise" else -(rf.amplitude or 0.0)
        t_max = 4.0 if edge == "rise" else 1.5
        t_grid = np.linspace(0.0, t_max, 400)
        y = fopdt_step_response(t_grid, edge_params.K, edge_params.tau, edge_params.L, u)
        ax_main.plot(
            t_grid, y, alpha=0.35, linewidth=0.6,
            color=("#1f77b4" if (rf.amplitude or 0) > 0 else "#d62728"),
        )

    title = f"{recipe} ({edge})"
    if channel:
        title += f"  (channel={channel}"
        if u_step is not None:
            title += f", u_step={u_step}"
        title += ")"
    ax_main.set_title(title)
    ax_main.set_xlabel(f"t − {'step_t' if edge == 'rise' else 'active_end_t'} [s]")
    ax_main.set_ylabel(f"meas_{channel or '?'} − baseline")
    ax_main.legend(loc="best", fontsize=8)
    ax_main.grid(True, alpha=0.3)

    edge_params_list = [
        rf.params if edge == "rise" else rf.params_down for rf in fits
    ]
    rmses = [p.rmse for p in edge_params_list if p is not None and np.isfinite(p.rmse)]
    if rmses:
        ax_resid.bar(range(len(rmses)), rmses, color="#888")
    ax_resid.set_ylabel("per-fit RMSE")
    ax_resid.set_xlabel("repeat index")
    ax_resid.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, format="svg")
    plt.close(fig)


def _plot_params_vs_amp(
    path: Path, channel: str, channel_summary: dict[str, Any], plt
) -> None:
    """Per-channel K/τ/L vs |amplitude| with CI error bars."""
    entries = channel_summary.get("per_amplitude", []) or []
    if not entries:
        return
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.2))
    for ax, p in zip(axes, ("K", "tau", "L")):
        for direction, color in (("forward", "#1f77b4"), ("reverse", "#d62728"), ("pooled", "#000")):
            xs = []
            ys = []
            yerr_lo = []
            yerr_hi = []
            for e in entries:
                if e.get("direction") != direction:
                    continue
                stats = e.get(p) or {}
                m = stats.get("mean")
                lo = stats.get("ci_low")
                hi = stats.get("ci_high")
                if m is None:
                    continue
                amp = abs(float(e.get("amplitude") or 0.0))
                xs.append(amp)
                ys.append(m)
                yerr_lo.append((m - lo) if lo is not None else 0.0)
                yerr_hi.append((hi - m) if hi is not None else 0.0)
            if xs:
                order = np.argsort(xs)
                xs_a = np.asarray(xs)[order]
                ys_a = np.asarray(ys)[order]
                lo_a = np.asarray(yerr_lo)[order]
                hi_a = np.asarray(yerr_hi)[order]
                ax.errorbar(xs_a, ys_a, yerr=[lo_a, hi_a], fmt="o-", color=color, label=direction)
        ax.set_xlabel("|amplitude|")
        ax.set_ylabel(p)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    fig.suptitle(f"channel = {channel}")
    fig.tight_layout()
    fig.savefig(path, format="svg")
    plt.close(fig)


def render_compare_markdown(verdict: dict[str, Any]) -> str:
    """Render the cross-mode comparison verdict to markdown."""
    lines: list[str] = []
    lines.append("# FOPDT Plant Model — Default vs Rage")
    lines.append("")
    for channel in sorted(verdict.get("channels", {})):
        cv = verdict["channels"][channel]
        lines.append(f"## `{channel}` — overall: **{cv.get('verdict')}**")
        lines.append("")
        lines.append("| param | default mean | rage mean | ratio | CI overlap | verdict |")
        lines.append("|---|---|---|---|---|---|")
        for p in ("K", "tau", "L"):
            v = cv.get("params", {}).get(p, {})
            d = v.get("default") or {}
            r = v.get("rage") or {}
            d_mean = _fmt(d.get("mean"))
            r_mean = _fmt(r.get("mean"))
            ratio = _fmt(v.get("ratio"))
            overlap = v.get("ci_overlap")
            lines.append(
                f"| {p} | {d_mean} | {r_mean} | {ratio} | {overlap} | {v.get('verdict')} |"
            )
        lines.append("")
    return "\n".join(lines) + "\n"


__all__ = ["render_compare_markdown", "render_markdown", "write_plots"]
