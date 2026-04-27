# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Per-group aggregation of FOPDT fits.

A "group" is the set of repeats for a single recipe — i.e. one
(channel, amplitude, direction) cell within one session/mode. The
aggregator drops degenerate / failed fits, performs 2σ outlier
rejection on the median of K/τ/L, then computes inverse-variance
weighted means with pooled CIs.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

from dimos.utils.characterization.modeling.per_run import RunFit


@dataclass
class _ParamStats:
    mean: float
    std: float
    ci_low: float
    ci_high: float
    n: int

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class GroupFit:
    key: dict[str, Any]              # {channel, amplitude, direction, recipe, mode}
    n_runs_input: int                # how many RunFits the aggregator saw
    n_runs_kept: int                 # converged + non-degenerate + not-2σ-rejected
    n_runs_rejected_2sigma: int
    n_runs_failed: int               # converged=False or skip_reason
    n_runs_degenerate: int           # converged but singular covariance
    rejected_run_ids: list[dict[str, Any]]   # [{"run_id", "param", "delta_sigma"}]
    failed_run_ids: list[str]
    K: dict[str, Any] | None         # _ParamStats.asdict() or None when empty
    tau: dict[str, Any] | None
    L: dict[str, Any] | None
    rmse_median: float | None
    notes: list[str] = field(default_factory=list)

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


def aggregate_group(
    run_fits: list[RunFit], *, sigma_reject: float = 2.0
) -> GroupFit:
    """Aggregate per-run FOPDT fits within one recipe group."""
    if not run_fits:
        raise ValueError("aggregate_group requires at least one RunFit")

    # All RunFits in a group must share the same key. Use the first as canonical.
    head = run_fits[0]
    key = {
        "channel": head.channel,
        "amplitude": head.amplitude,
        "direction": head.direction,
        "recipe": head.recipe,
        "mode": head.mode,
    }

    failed_ids: list[str] = []
    degenerate_count = 0
    usable: list[RunFit] = []
    for rf in run_fits:
        if rf.skip_reason is not None:
            failed_ids.append(rf.run_id)
            continue
        if rf.params is None or not rf.params.converged:
            failed_ids.append(rf.run_id)
            continue
        if rf.params.degenerate:
            degenerate_count += 1
            failed_ids.append(rf.run_id)
            continue
        usable.append(rf)

    notes: list[str] = []
    if degenerate_count:
        notes.append(f"{degenerate_count} run(s) had singular covariance (degenerate fit)")

    rejected: list[dict[str, Any]] = []
    if len(usable) >= 3:
        # 2σ outlier rejection on median: a fit is rejected if any of K/τ/L
        # is more than ``sigma_reject`` standard deviations from the group
        # median (using std with ddof=1 over the parameter vector).
        kept: list[RunFit] = []
        for param_name in ("K", "tau", "L"):
            arr = np.asarray([_p(rf, param_name) for rf in usable], dtype=float)
            med = float(np.median(arr))
            sd = float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0
            if sd <= 0:
                continue
            for rf, val in zip(usable, arr):
                delta = abs(val - med) / sd
                if delta > sigma_reject and rf.run_id not in {r["run_id"] for r in rejected}:
                    rejected.append({
                        "run_id": rf.run_id,
                        "param": param_name,
                        "value": float(val),
                        "median": med,
                        "delta_sigma": float(delta),
                    })
        rejected_ids = {r["run_id"] for r in rejected}
        kept = [rf for rf in usable if rf.run_id not in rejected_ids]
    else:
        kept = list(usable)
        if len(usable) > 0 and len(usable) < 3:
            notes.append(f"only {len(usable)} usable fit(s); skipped 2σ rejection")

    K_stats = _inverse_variance_combine(kept, "K")
    tau_stats = _inverse_variance_combine(kept, "tau")
    L_stats = _inverse_variance_combine(kept, "L")

    rmses = [float(rf.params.rmse) for rf in kept if rf.params is not None and np.isfinite(rf.params.rmse)]
    rmse_median = float(np.median(rmses)) if rmses else None

    return GroupFit(
        key=key,
        n_runs_input=len(run_fits),
        n_runs_kept=len(kept),
        n_runs_rejected_2sigma=len(rejected),
        n_runs_failed=len(failed_ids) - degenerate_count,
        n_runs_degenerate=degenerate_count,
        rejected_run_ids=rejected,
        failed_run_ids=failed_ids,
        K=K_stats.asdict() if K_stats else None,
        tau=tau_stats.asdict() if tau_stats else None,
        L=L_stats.asdict() if L_stats else None,
        rmse_median=rmse_median,
        notes=notes,
    )


def aggregate_session(
    run_fits: list[RunFit], *, sigma_reject: float = 2.0
) -> list[GroupFit]:
    """Bucket per-run fits by recipe and aggregate each group.

    Runs with ``skip_reason`` set are bucketed under their recipe (when
    present) so failures are visible in the per-group output rather than
    silently dropped.
    """
    by_recipe: dict[str, list[RunFit]] = {}
    for rf in run_fits:
        # Skip runs that don't even have a parseable recipe — there's no
        # group key to bucket them under. They'll already be recorded in
        # fits_per_run.json with a skip_reason.
        if rf.recipe == "<unknown>" or rf.channel is None:
            continue
        by_recipe.setdefault(rf.recipe, []).append(rf)
    return [aggregate_group(fits, sigma_reject=sigma_reject) for fits in by_recipe.values()]


# --------------------------------------------------------------------------- internal

def _p(rf: RunFit, name: str) -> float:
    assert rf.params is not None
    return float(getattr(rf.params, name))


def _ci_width(rf: RunFit, name: str) -> float:
    assert rf.params is not None
    lo, hi = getattr(rf.params, f"{name}_ci")
    return float(hi - lo)


def _inverse_variance_combine(
    kept: list[RunFit], param_name: str
) -> _ParamStats | None:
    """Inverse-variance weighted mean + pooled CI for one parameter.

    Sigma per fit is half the 95% CI width / 1.96. When CIs are NaN
    (degenerate, but those should already be filtered) or zero-width,
    fall back to equal weighting.
    """
    if not kept:
        return None
    vals = np.asarray([_p(rf, param_name) for rf in kept], dtype=float)
    widths = np.asarray([_ci_width(rf, param_name) for rf in kept], dtype=float)

    finite = np.isfinite(vals) & np.isfinite(widths) & (widths > 0)
    if not finite.any():
        # No usable per-fit sigmas — equal-weight mean, std across fits.
        if not np.isfinite(vals).any():
            return None
        v = vals[np.isfinite(vals)]
        mean = float(np.mean(v))
        std = float(np.std(v, ddof=1)) if v.size > 1 else 0.0
        ci_half = 1.96 * (std / np.sqrt(v.size)) if v.size > 1 else 0.0
        return _ParamStats(
            mean=mean, std=std,
            ci_low=mean - ci_half, ci_high=mean + ci_half,
            n=int(v.size),
        )

    v = vals[finite]
    sigmas = (widths[finite] / 2.0) / 1.96
    weights = 1.0 / (sigmas ** 2)
    mean = float(np.sum(weights * v) / np.sum(weights))
    pooled_var = 1.0 / float(np.sum(weights))
    pooled_sigma = float(np.sqrt(pooled_var))
    # Group std reported as the inter-fit std (descriptive), separate from
    # the pooled inferential CI.
    std = float(np.std(v, ddof=1)) if v.size > 1 else 0.0
    return _ParamStats(
        mean=mean, std=std,
        ci_low=mean - 1.96 * pooled_sigma,
        ci_high=mean + 1.96 * pooled_sigma,
        n=int(v.size),
    )


__all__ = ["GroupFit", "aggregate_group", "aggregate_session"]
