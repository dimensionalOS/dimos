# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Default-vs-rage comparison.

Takes two sets of session dirs (one labelled "default", one "rage")
and emits a side-by-side markdown table of per-metric ratios. Decides
whether rage is a "scaled plant" (constants change but shapes match —
one model with mode parameter suffices) or a "different plant"
(separate models needed).
"""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path
from typing import Any


# Verdict thresholds (relative to ratio = 1.0):
#   < 5%  → "identical"  (within run-to-run noise; modes indistinguishable)
#   5-20% → "scaled"     (consistent multiplicative offset; one model with mode param)
#   > 20% → "differs"    (qualitatively different; separate models)
_RATIO_THRESHOLD_IDENTICAL = 0.05
_RATIO_THRESHOLD_SCALED = 0.20


def compare_modes(
    default_sessions: list[Path],
    rage_sessions: list[Path],
    *,
    out_path: Path | None = None,
) -> str:
    """Emit a markdown comparison of metrics across two modes."""
    default_metrics = _collect(default_sessions)
    rage_metrics = _collect(rage_sessions)

    common = sorted(set(default_metrics) & set(rage_metrics))

    lines: list[str] = []
    lines.append("# Default vs Rage — per-recipe comparison")
    lines.append("")
    lines.append(f"Default sessions: {len(default_sessions)}")
    lines.append(f"Rage sessions:    {len(rage_sessions)}")
    lines.append(f"Recipes in both:  {len(common)}")
    lines.append("")
    lines.append("| Recipe | metric | default mean | rage mean | ratio (rage/default) | verdict |")
    lines.append("|--|--|--|--|--|--|")

    overall_ratios: list[float] = []
    for recipe in common:
        d = default_metrics[recipe]
        r = rage_metrics[recipe]
        for key in ("steady_state", "rise_10_90_s", "settle_s", "overshoot"):
            dv = d.get(key)
            rv = r.get(key)
            if not isinstance(dv, (int, float)) or not isinstance(rv, (int, float)):
                continue
            if abs(dv) < 1e-6:
                continue
            ratio = rv / dv
            dev = abs(ratio - 1.0)
            if dev < _RATIO_THRESHOLD_IDENTICAL:
                verdict = "identical"
            elif dev <= _RATIO_THRESHOLD_SCALED:
                verdict = "scaled"
            else:
                verdict = "differs"
            overall_ratios.append(ratio)
            lines.append(
                f"| {recipe} | {key} | {_fmt(dv)} | {_fmt(rv)} | {ratio:+.3f} | {verdict} |"
            )

    lines.append("")
    if overall_ratios:
        n_identical = sum(
            1 for r in overall_ratios if abs(r - 1.0) < _RATIO_THRESHOLD_IDENTICAL
        )
        n_scaled = sum(
            1 for r in overall_ratios
            if _RATIO_THRESHOLD_IDENTICAL <= abs(r - 1.0) <= _RATIO_THRESHOLD_SCALED
        )
        n_differs = len(overall_ratios) - n_identical - n_scaled
        total = len(overall_ratios)
        identical_pct = 100.0 * n_identical / total
        lines.append(
            f"**Identical** (|ratio−1| < 5%): {n_identical}/{total} ({identical_pct:.1f}%)"
        )
        lines.append(
            f"**Scaled**    (5–20% offset):    {n_scaled}/{total} ({100.0 * n_scaled / total:.1f}%)"
        )
        lines.append(
            f"**Differs**   (> 20% offset):    {n_differs}/{total} ({100.0 * n_differs / total:.1f}%)"
        )
        lines.append("")
        if identical_pct >= 70.0:
            lines.append(
                "**Verdict**: rage and default appear *indistinguishable* on these "
                "metrics. Either rage isn't activating, or the recipe envelope "
                "doesn't push hard enough to expose a difference. Check "
                "session.json's ``rage`` flag and consider higher-amplitude recipes."
            )
        elif (n_identical + n_scaled) / total >= 0.70:
            lines.append(
                "**Verdict**: rage is a *scaled plant* — one FOPDT family with "
                "mode parameter."
            )
        elif (n_identical + n_scaled) / total >= 0.40:
            lines.append(
                "**Verdict**: rage is *partially scaled* — model some metrics "
                "shared, others separately."
            )
        else:
            lines.append(
                "**Verdict**: rage is a *different plant* — model the two modes "
                "separately."
            )
    lines.append("")

    text = "\n".join(lines) + "\n"
    if out_path is not None:
        Path(out_path).write_text(text)
    return text


def _collect(session_dirs: list[Path]) -> dict[str, dict[str, float | None]]:
    """For each recipe, collect mean of each metric across all sessions."""
    by_recipe: dict[str, dict[str, list[float]]] = defaultdict(lambda: defaultdict(list))
    for s in session_dirs:
        sp = Path(s).expanduser().resolve() / "session_summary.json"
        if not sp.exists():
            continue
        summary = json.loads(sp.read_text())
        for g in summary.get("groups", []):
            recipe = g["recipe"]
            for key, v in (g.get("metrics") or {}).items():
                if isinstance(v, dict):
                    mean = v.get("mean")
                    if isinstance(mean, (int, float)):
                        by_recipe[recipe][key].append(mean)

    out: dict[str, dict[str, float | None]] = {}
    for recipe, metrics in by_recipe.items():
        out[recipe] = {}
        for key, vals in metrics.items():
            out[recipe][key] = (sum(vals) / len(vals)) if vals else None
    return out


def _fmt(v: Any) -> str:
    if v is None:
        return "—"
    return f"{v:+.4f}"


__all__ = ["compare_modes"]
