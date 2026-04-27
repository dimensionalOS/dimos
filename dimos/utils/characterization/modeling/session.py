# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Session-level orchestration: discover runs, fit each, aggregate, pool, write.

Three entry points used by the scripts layer:

  - ``fit_session(session_dir)``        — fit one session, write artifacts.
  - ``fit_all_sessions(parent_dir)``    — discover and fit every session
                                          under a parent directory.
  - ``compare_pooled(default, rage)``   — pool RunFits across multiple
                                          sessions per mode, then compare.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from dimos.utils.characterization.modeling._io import atomic_write_json
from dimos.utils.characterization.modeling.aggregate import (
    aggregate_session as aggregate_session_groups,
)
from dimos.utils.characterization.modeling.per_run import RunFit, fit_run, select_edge
from dimos.utils.characterization.modeling.pool import (
    compare_models,
    compare_rise_fall,
    pool_session,
)
from dimos.utils.characterization.modeling.report import (
    render_compare_markdown,
    render_markdown,
    write_plots,
)


def _read_mode(session_dir: Path) -> str:
    """Read mode from ``session.json``; defaults to "default" when absent."""
    sj = session_dir / "session.json"
    if not sj.exists():
        return "default"
    try:
        meta = json.loads(sj.read_text())
    except Exception:
        return "default"
    return "rage" if bool(meta.get("rage")) else "default"


def _discover_runs(session_dir: Path) -> list[Path]:
    """Same convention used by the rest of the pipeline (aggregate.py:47)."""
    return sorted(
        p for p in session_dir.iterdir()
        if p.is_dir() and p.name and p.name[0].isdigit()
    )


def fit_session_runs(session_dir: Path, mode: str | None = None) -> list[RunFit]:
    """Per-run fitting only; does not aggregate or write artifacts.

    Useful for cross-session pooling where we need RunFits from many
    sessions concatenated before any aggregation happens.
    """
    session_dir = Path(session_dir).expanduser().resolve()
    if not session_dir.is_dir():
        raise FileNotFoundError(f"not a directory: {session_dir}")
    if mode is None:
        mode = _read_mode(session_dir)
    out: list[RunFit] = []
    for rd in _discover_runs(session_dir):
        try:
            out.append(fit_run(rd, mode=mode))
        except Exception as e:
            out.append(
                RunFit(
                    run_id=rd.name, run_dir=str(rd), recipe="<unknown>",
                    channel=None, amplitude=None, direction=None,
                    mode=mode, split=None, params=None,
                    skip_reason=f"fit_run raised: {type(e).__name__}: {e}",
                )
            )
    return out


def fit_session(session_dir: Path, *, write_plots_enabled: bool = True) -> dict[str, Any]:
    """Run the full per-session FOPDT pipeline.

    Writes:
      - ``<session>/modeling/fits_per_run.json``
      - ``<session>/modeling/fits_per_group.json``
      - ``<session>/modeling/model_summary.json``
      - ``<session>/modeling/model_report.md``
      - ``<session>/modeling/plots/*.svg``  (best-effort)

    Returns the in-memory ``model_summary`` dict.
    """
    session_dir = Path(session_dir).expanduser().resolve()
    mode = _read_mode(session_dir)
    out_dir = session_dir / "modeling"
    out_dir.mkdir(parents=True, exist_ok=True)
    plots_dir = out_dir / "plots"

    run_fits = fit_session_runs(session_dir, mode=mode)
    summary = _build_summary(
        run_fits, mode=mode,
        provenance={"session_dir": str(session_dir)},
    )

    atomic_write_json(out_dir / "fits_per_run.json", {
        "session_dir": str(session_dir),
        "mode": mode,
        "n_runs": len(run_fits),
        "runs": [rf.asdict() for rf in run_fits],
    })
    atomic_write_json(out_dir / "fits_per_group.json", {
        "session_dir": str(session_dir),
        "mode": mode,
        "n_groups_rise": len(summary["_rise_groups_in_memory"]),
        "n_groups_fall": len(summary["_fall_groups_in_memory"]),
        "rise_groups": [g.asdict() for g in summary["_rise_groups_in_memory"]],
        "fall_groups": [g.asdict() for g in summary["_fall_groups_in_memory"]],
    })
    rise_groups = summary.pop("_rise_groups_in_memory")
    fall_groups = summary.pop("_fall_groups_in_memory")
    atomic_write_json(out_dir / "model_summary.json", summary)

    md = render_markdown(
        session_dir=session_dir, mode=mode,
        summary=summary,
        group_fits=rise_groups, run_fits=run_fits,
        fall_groups=fall_groups,
    )
    (out_dir / "model_report.md").write_text(md)

    if write_plots_enabled:
        write_plots(
            plots_dir=plots_dir,
            summary=summary,
            group_fits=rise_groups,
            run_fits=run_fits,
            fall_groups=fall_groups,
        )

    return summary


def _build_summary(
    run_fits: list[RunFit], *, mode: str, provenance: dict[str, Any]
) -> dict[str, Any]:
    """Aggregate + pool rise and fall edges, return combined summary.

    Sticks the in-memory GroupFits onto the dict under ``_*_in_memory`` so
    the caller can serialize them; the caller is expected to pop those
    before writing the summary JSON.
    """
    rise_fits = select_edge(run_fits, "rise")
    fall_fits = select_edge(run_fits, "fall")
    rise_groups = aggregate_session_groups(rise_fits)
    fall_groups = aggregate_session_groups(fall_fits)
    rise_summary = pool_session(rise_groups, mode=mode)
    fall_summary = pool_session(fall_groups, mode=mode)
    rise_vs_fall = compare_rise_fall(rise_summary, fall_summary)

    summary: dict[str, Any] = {
        **provenance,
        "mode": mode,
        "channels": rise_summary["channels"],
        "diagnostics": rise_summary["diagnostics"],
        "rise": rise_summary,
        "fall": fall_summary,
        "rise_vs_fall": rise_vs_fall,
        "_rise_groups_in_memory": rise_groups,
        "_fall_groups_in_memory": fall_groups,
    }
    return summary


# ---------------------------------------------------------------- batch fit

def fit_all_sessions(
    parent_dir: Path, *, force: bool = False, write_plots_enabled: bool = True
) -> dict[str, Any]:
    """Discover ``session_*`` dirs under ``parent_dir`` and fit each.

    Skips sessions whose ``modeling/model_summary.json`` already exists
    unless ``force=True``. Writes a combined index at
    ``<parent>/models_index.json`` listing every session and its mode,
    n_runs, and n_step_runs (so it's easy to see at a glance which
    sessions had E1/E2/E8 step recipes worth pooling).
    """
    parent_dir = Path(parent_dir).expanduser().resolve()
    if not parent_dir.is_dir():
        raise FileNotFoundError(f"not a directory: {parent_dir}")

    sessions = sorted(
        p for p in parent_dir.iterdir()
        if p.is_dir() and p.name.startswith("session_")
    )
    index_rows: list[dict[str, Any]] = []
    for s in sessions:
        ms = s / "modeling" / "model_summary.json"
        status = "ok"
        if ms.exists() and not force:
            try:
                summary = json.loads(ms.read_text())
                status = "cached"
            except Exception:
                summary = None
                status = "cached_read_failed"
        else:
            try:
                summary = fit_session(s, write_plots_enabled=write_plots_enabled)
            except Exception as e:
                summary = None
                status = f"failed: {type(e).__name__}: {e}"

        n_runs = sum(1 for _ in _discover_runs(s)) if s.is_dir() else 0
        diag = (summary or {}).get("diagnostics") or {}
        n_groups_with_fit = int(diag.get("n_groups_with_fit") or 0)
        index_rows.append({
            "session": s.name,
            "session_dir": str(s),
            "mode": (summary or {}).get("mode") or _read_mode(s),
            "status": status,
            "n_runs": n_runs,
            "n_groups_with_fit": n_groups_with_fit,
            "model_summary": str(ms) if ms.exists() else None,
        })

    index = {
        "parent_dir": str(parent_dir),
        "n_sessions": len(sessions),
        "sessions": index_rows,
    }
    atomic_write_json(parent_dir / "models_index.json", index)
    return index


# -------------------------------------------------------- cross-session pool

def pool_runs_across_sessions(
    session_dirs: list[Path], *, mode_label: str
) -> tuple[dict[str, Any], list[RunFit]]:
    """Concatenate RunFits from multiple sessions, then aggregate + pool.

    Returns ``(summary, run_fits)``. ``summary`` has the same shape as
    ``fit_session``'s output but with ``session_dirs`` provenance instead
    of a single ``session_dir``.
    """
    all_runs: list[RunFit] = []
    for s in session_dirs:
        all_runs.extend(fit_session_runs(Path(s), mode=mode_label))
    summary = _build_summary(
        all_runs, mode=mode_label,
        provenance={
            "session_dirs": [str(Path(s).expanduser().resolve()) for s in session_dirs],
            "n_sessions": len(session_dirs),
        },
    )
    summary.pop("_rise_groups_in_memory", None)
    summary.pop("_fall_groups_in_memory", None)
    return summary, all_runs


# -------------------------------------------------------- compare (multi)

def compare_pooled(
    default_sessions: list[Path],
    rage_sessions: list[Path],
    *,
    out_path: Path,
) -> dict[str, Any]:
    """Pool RunFits across all default-mode sessions and all rage-mode
    sessions, then compare modes. Writes:

      - ``<out_path>``                       — markdown verdict report
      - ``<out_path stem>_pooled.json``      — full pooled summaries + verdict

    ``default_sessions`` and ``rage_sessions`` may each contain multiple
    paths. Only step-recipe runs contribute to fits; other recipes are
    skipped automatically.
    """
    out_path = Path(out_path).expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    default_pooled, _d_runs = pool_runs_across_sessions(default_sessions, mode_label="default")
    rage_pooled, _r_runs = pool_runs_across_sessions(rage_sessions, mode_label="rage")

    verdict = compare_models(default_pooled, rage_pooled)

    md_lines = [render_compare_markdown(verdict).rstrip("\n")]
    md_lines.append("")
    md_lines.append("## Pooled provenance")
    md_lines.append("")
    md_lines.append(f"- default mode: {len(default_sessions)} session(s), "
                    f"{default_pooled.get('diagnostics', {}).get('n_groups_total', 0)} groups")
    for s in default_sessions:
        md_lines.append(f"  - `{s}`")
    md_lines.append(f"- rage mode: {len(rage_sessions)} session(s), "
                    f"{rage_pooled.get('diagnostics', {}).get('n_groups_total', 0)} groups")
    for s in rage_sessions:
        md_lines.append(f"  - `{s}`")
    md_lines.append("")
    md_lines.append("## Pooled K/τ/L per channel")
    md_lines.append("")
    md_lines.append("| mode | channel | K (95% CI) | τ (95% CI) | L (95% CI) |")
    md_lines.append("|---|---|---|---|---|")
    for label, summary in (("default", default_pooled), ("rage", rage_pooled)):
        for channel, ch in sorted((summary.get("channels") or {}).items()):
            pooled = ch.get("pooled") or {}
            row = [label, channel]
            for p in ("K", "tau", "L"):
                stats = pooled.get(p) or {}
                row.append(_fmt_stats(stats))
            md_lines.append("| " + " | ".join(row) + " |")
    md_lines.append("")

    out_path.write_text("\n".join(md_lines) + "\n")
    pooled_path = out_path.with_name(out_path.stem + "_pooled.json")
    atomic_write_json(pooled_path, {
        "default_pooled": default_pooled,
        "rage_pooled": rage_pooled,
        "verdict": verdict,
    })
    return verdict


def _fmt_stats(stats: dict[str, Any]) -> str:
    mean = stats.get("mean")
    lo = stats.get("ci_low")
    hi = stats.get("ci_high")
    def f(v):
        try:
            return f"{float(v):.4g}"
        except Exception:
            return "—"
    return f"{f(mean)} [{f(lo)}, {f(hi)}]"


# ----------------------------------------------- legacy single-pair compare

def compare_two_sessions(
    default_session: Path, rage_session: Path, *, out_path: Path
) -> dict[str, Any]:
    """Compare two pre-fit sessions via their ``model_summary.json``.

    Kept for backwards-compatibility. Prefer ``compare_pooled`` when you
    have multiple sessions per mode.
    """
    default_session = Path(default_session).expanduser().resolve()
    rage_session = Path(rage_session).expanduser().resolve()

    def _load(s: Path) -> dict[str, Any]:
        ms = s / "modeling" / "model_summary.json"
        if not ms.exists():
            raise FileNotFoundError(
                f"{ms} not found — run `process_session fit {s}` first."
            )
        return json.loads(ms.read_text())

    d_summary = _load(default_session)
    r_summary = _load(rage_session)
    verdict = compare_models(d_summary, r_summary)
    verdict["default_session"] = str(default_session)
    verdict["rage_session"] = str(rage_session)

    out_path = Path(out_path).expanduser().resolve()
    md = render_compare_markdown(verdict)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(md)
    pooled_path = out_path.with_name(out_path.stem + "_pooled.json")
    atomic_write_json(pooled_path, verdict)
    return verdict


__all__ = [
    "compare_pooled",
    "compare_two_sessions",
    "fit_all_sessions",
    "fit_session",
    "fit_session_runs",
    "pool_runs_across_sessions",
]
