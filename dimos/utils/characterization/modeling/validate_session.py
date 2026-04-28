# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Session-level validation orchestration.

Direction holdout (the only regime in scope per the Session 2 plan):

  1. Read existing per-run fits (or compute fresh) for the session.
  2. Filter to forward-direction runs only — that's the training set.
  3. Re-aggregate + re-pool on the forward subset to build a
     forward-only model (rise + fall).
  4. For each reverse-direction step run, predict the response using
     the forward-only model and compute residual metrics.
  5. Aggregate per channel, run diagnosis if marginal/failing.
  6. Write JSON artifacts, markdown report, plots.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from dimos.utils.characterization.modeling._io import atomic_write_json
from dimos.utils.characterization.modeling.aggregate import (
    aggregate_session as aggregate_session_groups,
)
from dimos.utils.characterization.modeling.diagnose import diagnose_validation
from dimos.utils.characterization.modeling.per_run import RunFit, select_edge
from dimos.utils.characterization.modeling.pool import pool_session
from dimos.utils.characterization.modeling.session import (
    _read_mode,
    _discover_runs,
    fit_session_runs,
)
from dimos.utils.characterization.modeling.validate_aggregate import (
    aggregate_validation,
)
from dimos.utils.characterization.modeling.validate_report import (
    render_validation_markdown,
    write_validation_plots,
)
from dimos.utils.characterization.modeling.validate_run import (
    ValidationResult,
    validate_run,
)


def _pool_forward_only(
    run_fits: list[RunFit], *, mode: str
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Build (rise_summary, fall_summary) from the forward-direction subset.

    This is the model the validator will use to predict held-out reverse
    runs. Skipped runs and reverse runs are dropped before aggregation.
    """
    fwd = [r for r in run_fits if r.direction == "forward" and r.skip_reason is None]
    rise_groups = aggregate_session_groups(select_edge(fwd, "rise"))
    fall_groups = aggregate_session_groups(select_edge(fwd, "fall"))
    rise_summary = pool_session(rise_groups, mode=mode)
    fall_summary = pool_session(fall_groups, mode=mode)
    return rise_summary, fall_summary


def validate_session_direction_holdout(
    session_dir: Path,
    *,
    write_plots_enabled: bool = True,
) -> dict[str, Any]:
    """Run direction-holdout validation on one session and write artifacts.

    Outputs (under ``<session_dir>/modeling/validation/``):
      - ``validation_per_run.json`` — every held-out run's prediction +
        metrics + verdict.
      - ``validation_summary.json`` — per-channel aggregate verdict.
      - ``diagnosis.json`` — only when at least one channel is
        marginal/failing.
      - ``validation_report.md`` — human-readable.
      - ``plots/`` — best/median/worst overlays + distribution histogram.

    Returns the in-memory ``validation_summary`` dict.
    """
    session_dir = Path(session_dir).expanduser().resolve()
    if not session_dir.is_dir():
        raise FileNotFoundError(f"not a directory: {session_dir}")
    mode = _read_mode(session_dir)

    out_dir = session_dir / "modeling" / "validation"
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1-3: refit on forward-only.
    run_fits = fit_session_runs(session_dir, mode=mode)
    rise_summary, fall_summary = _pool_forward_only(run_fits, mode=mode)

    # 4: validate each reverse-direction step run.
    reverse_runs = [
        r for r in run_fits
        if r.direction == "reverse" and r.skip_reason is None
    ]
    n_train = sum(1 for r in run_fits if r.direction == "forward" and r.skip_reason is None)
    n_validate_runs = len(reverse_runs)

    results: list[ValidationResult] = []
    for rf in reverse_runs:
        try:
            v = validate_run(
                Path(rf.run_dir),
                rise_summary=rise_summary,
                fall_summary=fall_summary,
                mode=mode,
                keep_traces=True,
            )
        except Exception as e:
            v = ValidationResult(
                run_id=rf.run_id, run_dir=rf.run_dir, recipe=rf.recipe,
                channel=rf.channel, amplitude=rf.amplitude,
                direction=rf.direction, mode=mode, split=rf.split,
                used_K=float("nan"), used_tau=float("nan"), used_L=float("nan"),
                skip_reason=f"validate_run raised: {type(e).__name__}: {e}",
            )
        results.append(v)

    # 5: aggregate + diagnose.
    summary = aggregate_validation(results, mode=mode)
    summary["session_dir"] = str(session_dir)
    summary["n_train_runs_forward"] = n_train
    summary["n_validate_runs_reverse"] = n_validate_runs

    diagnosis: dict[str, Any] | None = diagnose_validation(results, summary)
    if not (diagnosis.get("channels") or {}):
        diagnosis = None

    # 6: write artifacts.
    atomic_write_json(out_dir / "validation_per_run.json", {
        "session_dir": str(session_dir),
        "mode": mode,
        "n_runs": len(results),
        "results": [r.asdict() for r in results],
    })
    atomic_write_json(out_dir / "validation_summary.json", summary)
    if diagnosis is not None:
        atomic_write_json(out_dir / "diagnosis.json", diagnosis)

    md = render_validation_markdown(
        session_dir=session_dir, mode=mode, summary=summary,
        diagnosis=diagnosis,
        n_train=n_train, n_validate=n_validate_runs,
    )
    (out_dir / "validation_report.md").write_text(md)

    if write_plots_enabled:
        plots_dir = out_dir / "plots"
        write_validation_plots(
            plots_dir=plots_dir, results=results, summary=summary,
        )

    return summary


__all__ = ["validate_session_direction_holdout"]
