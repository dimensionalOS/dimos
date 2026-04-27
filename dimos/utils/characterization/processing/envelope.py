# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Per-mode operational envelope summary.

Takes a list of session directories (presumed to be the same mode —
either all default or all rage), pulls aggregated metrics from each,
and emits one human-readable markdown summary covering:

  - Linear (vx) gain curve from E1 step matrix
  - Angular (wz) gain curve from E2 step matrix
  - Saturation points from E3/E4 ramps
  - Cross-coupling decision from E7
  - Deadtime statistics from E8
  - Battery / surface metadata snapshot

Pre-requisites: each input session must have already been processed:
``python -m dimos.utils.characterization.scripts.process_session validate``, ``python -m dimos.utils.characterization.scripts.process_session aggregate``, optionally
``python -m dimos.utils.characterization.scripts.process_session deadtime`` and ``python -m dimos.utils.characterization.scripts.process_session coupling``.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def envelope_report(
    session_dirs: list[Path], *, mode_label: str = "default", out_path: Path | None = None
) -> str:
    """Build a markdown envelope summary; returns the markdown text.

    Writes to ``out_path`` if given.
    """
    sessions = [Path(p).expanduser().resolve() for p in session_dirs]
    bundles = [_load_session(s) for s in sessions]

    lines: list[str] = []
    lines.append(f"# Operational envelope — {mode_label}")
    lines.append("")
    lines.append(f"Sessions analyzed: {len(sessions)}")
    for s in sessions:
        lines.append(f"  - `{s}`")
    lines.append("")

    # E1 / E2 step gain
    lines.append("## E1 — vx step gain")
    lines.append("")
    lines.append(_step_gain_table(bundles, channel="vx", recipe_prefix="e1_vx"))
    lines.append("")
    lines.append("## E2 — wz step gain")
    lines.append("")
    lines.append(_step_gain_table(bundles, channel="wz", recipe_prefix="e2_wz"))
    lines.append("")

    # E3 / E4 saturation
    lines.append("## E3 — vx saturation ramp")
    lines.append("")
    lines.append(_ramp_summary(bundles, recipe_prefix="e3_vx_ramp"))
    lines.append("")
    lines.append("## E4 — wz saturation ramp")
    lines.append("")
    lines.append(_ramp_summary(bundles, recipe_prefix="e4_wz_ramp"))
    lines.append("")

    # E7 coupling
    lines.append("## E7 — cross-coupling")
    lines.append("")
    lines.append(_coupling_summary(bundles))
    lines.append("")

    # E8 deadtime
    lines.append("## E8 — deadtime")
    lines.append("")
    lines.append(_deadtime_summary(bundles))
    lines.append("")

    # Operational metadata
    lines.append("## Operational metadata")
    lines.append("")
    lines.append(_metadata_summary(bundles))
    lines.append("")

    text = "\n".join(lines) + "\n"
    if out_path is not None:
        Path(out_path).write_text(text)
    return text


# ---------------------------------------------------------------------------- internal

def _load_session(session_dir: Path) -> dict[str, Any]:
    """Load all derived artifacts from one session into a dict."""
    bundle: dict[str, Any] = {"dir": session_dir}
    for fname, key in [
        ("session.json", "session"),
        ("session_summary.json", "summary"),
        ("validation_summary.json", "validation"),
        ("deadtime_stats.json", "deadtime"),
        ("coupling_stats.json", "coupling"),
    ]:
        p = session_dir / fname
        if p.exists():
            bundle[key] = json.loads(p.read_text())
    return bundle


def _step_gain_table(bundles: list[dict[str, Any]], *, channel: str, recipe_prefix: str) -> str:
    """One row per (recipe, repetition) showing measured/commanded ratio."""
    rows: list[tuple[str, dict[str, Any]]] = []
    for b in bundles:
        for g in (b.get("summary") or {}).get("groups", []):
            if g["recipe"].startswith(recipe_prefix):
                rows.append((g["recipe"], g))
    if not rows:
        return f"_no `{recipe_prefix}` data found_"

    lines = [
        "| Recipe | n | target | meas mean | meas std | gain K = meas/cmd | rise (s) | settle (s) | overshoot |",
        "|--|--|--|--|--|--|--|--|--|",
    ]
    for recipe, g in sorted(rows, key=lambda kv: kv[0]):
        m = g["metrics"]
        target = m["target"]["mean"]
        ss_mean = m["steady_state"]["mean"]
        ss_std = m["steady_state"]["std"]
        rise = m["rise_10_90_s"]["mean"]
        settle = m["settle_s"]["mean"]
        ovs = m["overshoot"]["mean"]
        gain = (ss_mean / target) if (target and ss_mean is not None and abs(target) > 1e-6) else None
        lines.append(
            f"| {recipe} | {g['n_runs_kept']} | {_fmt(target)} | {_fmt(ss_mean)} | "
            f"{_fmt(ss_std)} | {_fmt(gain)} | {_fmt(rise)} | {_fmt(settle)} | {_fmt(ovs)} |"
        )
    return "\n".join(lines)


def _ramp_summary(bundles: list[dict[str, Any]], *, recipe_prefix: str) -> str:
    rows: list[tuple[str, dict[str, Any]]] = []
    for b in bundles:
        for g in (b.get("summary") or {}).get("groups", []):
            if g["recipe"].startswith(recipe_prefix):
                rows.append((g["recipe"], g))
    if not rows:
        return f"_no `{recipe_prefix}` data found_"

    lines = ["| Recipe | n | cmd_max | cmd_min |", "|--|--|--|--|"]
    for recipe, g in sorted(rows, key=lambda kv: kv[0]):
        m = g["metrics"]
        lines.append(
            f"| {recipe} | {g['n_runs_kept']} | {_fmt(m['cmd_max']['mean'])} | "
            f"{_fmt(m['cmd_min']['mean'])} |"
        )
    return (
        "\n".join(lines)
        + "\n\n_Saturation point detection requires running the parametric "
        "(cmd vs meas) analysis — see plot.svg for each ramp run._"
    )


def _coupling_summary(bundles: list[dict[str, Any]]) -> str:
    chunks: list[str] = []
    for b in bundles:
        c = b.get("coupling")
        if not c:
            continue
        chunks.append(f"**Session `{b['dir'].name}`**: overall = `{c.get('overall_decision')}`")
        for g in c.get("groups", []):
            chunks.append(
                f"- `{g['recipe']}` (n={g['n_runs']}): "
                f"decision={g['decision']}, "
                f"leak% means={g.get('leak_pct_mean')}"
            )
    return "\n".join(chunks) if chunks else "_no E7 coupling data found_"


def _deadtime_summary(bundles: list[dict[str, Any]]) -> str:
    chunks: list[str] = []
    for b in bundles:
        d = b.get("deadtime")
        if not d:
            continue
        s = d.get("summary") or {}
        chunks.append(
            f"**Session `{b['dir'].name}`**: n={s.get('n')}, "
            f"mean={s.get('mean_s')}s, median={s.get('median_s')}s, "
            f"p95={s.get('p95_s')}s, jitter σ={s.get('std_s')}s"
        )
    return "\n".join(chunks) if chunks else "_no E8 deadtime data found_"


def _metadata_summary(bundles: list[dict[str, Any]]) -> str:
    chunks: list[str] = []
    for b in bundles:
        sess = b.get("session") or {}
        op = sess.get("operator") or {}
        chunks.append(
            f"- `{b['dir'].name}`  notes='{op.get('notes')}'  surface='{op.get('surface')}'  "
            f"rage={sess.get('rage')}  runs={len(sess.get('runs') or [])}"
        )
    return "\n".join(chunks) if chunks else "_no session.json found_"


def _fmt(v: Any) -> str:
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:+.3f}"
    return str(v)


__all__ = ["envelope_report"]
