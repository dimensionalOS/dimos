# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Aggregation of per-run validation results into per-channel verdicts.

Per-run validation gives one ``ValidationResult`` per held-out run.
This module groups those by (channel, amplitude, direction) and per
channel, then produces a top-line pass/marginal/fail verdict per
channel for the report.

Pass-rate thresholds (validation-set, not per-run):

  - pass:     ≥ 80% of runs pass per-run thresholds
  - marginal: 60-80% pass rate
  - fail:     < 60% pass rate

These mirror the Session 2 plan's defaults. Same caveat as the per-run
thresholds: numbers are starting points; the report should surface the
distribution so they can be revised with evidence.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

from dimos.utils.characterization.modeling.validate_run import ValidationResult


_PASS_RATE_PASS = 0.80
_PASS_RATE_MARGINAL = 0.60


@dataclass
class GroupSummary:
    key: dict[str, Any]      # {channel, amplitude (signed), direction}
    n_total: int
    n_pass: int
    n_marginal: int
    n_fail: int
    n_skip: int
    # ``rise_norm_rmse`` carries the *smoothed* distribution (verdict-driver).
    # ``_raw`` carries the original raw-residual distribution for reference.
    # ``residual_over_noise`` is the noise-floor diagnostic (rmse_smoothed/σ).
    rise_norm_rmse: dict[str, Any]
    fall_norm_rmse: dict[str, Any] | None
    rise_norm_rmse_raw: dict[str, Any] | None = None
    fall_norm_rmse_raw: dict[str, Any] | None = None
    rise_residual_over_noise: dict[str, Any] | None = None
    fall_residual_over_noise: dict[str, Any] | None = None
    worst_run_ids: list[str] = field(default_factory=list)

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ChannelSummary:
    channel: str
    mode: str
    n_total: int
    n_pass: int
    n_marginal: int
    n_fail: int
    n_skip: int
    pass_rate: float
    verdict: str
    rise_norm_rmse: dict[str, Any]
    fall_norm_rmse: dict[str, Any] | None
    rise_norm_rmse_raw: dict[str, Any] | None = None
    fall_norm_rmse_raw: dict[str, Any] | None = None
    rise_residual_over_noise: dict[str, Any] | None = None
    fall_residual_over_noise: dict[str, Any] | None = None
    by_amp_direction: list[GroupSummary] = field(default_factory=list)
    worst_run_ids: list[str] = field(default_factory=list)

    def asdict(self) -> dict[str, Any]:
        d = asdict(self)
        return d


# --------------------------------------------------------------------------- helpers

def _quantiles(values: list[float]) -> dict[str, Any]:
    """Median / p25 / p75 / max of a list of finite floats."""
    finite = [v for v in values if v is not None and np.isfinite(v)]
    if not finite:
        return {"median": float("nan"), "p25": float("nan"),
                "p75": float("nan"), "max": float("nan"), "n": 0}
    arr = np.asarray(finite, dtype=float)
    return {
        "median": float(np.median(arr)),
        "p25": float(np.percentile(arr, 25)),
        "p75": float(np.percentile(arr, 75)),
        "max": float(np.max(arr)),
        "n": int(arr.size),
    }


def _channel_verdict(pass_rate: float) -> str:
    if pass_rate >= _PASS_RATE_PASS:
        return "pass"
    if pass_rate >= _PASS_RATE_MARGINAL:
        return "marginal"
    return "fail"


def _metric_value(metrics: dict[str, Any] | None, key: str) -> float | None:
    """Pull a finite metric value or None."""
    if metrics is None:
        return None
    v = metrics.get(key)
    return v if (v is not None and np.isfinite(v)) else None


def _norm_rmse_of(metrics: dict[str, Any] | None) -> float | None:
    """Smoothed nRMSE — the verdict-driving metric."""
    return _metric_value(metrics, "norm_rmse_smoothed")


# --------------------------------------------------------------------------- aggregation

def aggregate_validation(
    results: list[ValidationResult],
    *,
    mode: str = "default",
    worst_n: int = 4,
) -> dict[str, Any]:
    """Aggregate a flat list of ValidationResults into a structured summary.

    Returns a dict shaped for ``validation_summary.json``::

        {
          "mode": "default",
          "n_runs_total": ...,
          "n_runs_pass": ..., "n_runs_marginal": ...,
          "n_runs_fail": ...,  "n_runs_skip": ...,
          "channels": {
            "vx": {ChannelSummary fields...},
            "wz": {...}
          },
          "thresholds": {...}
        }
    """
    by_channel: dict[str, list[ValidationResult]] = {}
    for r in results:
        if r.channel is None:
            continue
        by_channel.setdefault(r.channel, []).append(r)

    n_pass = sum(1 for r in results if r.verdict == "pass")
    n_marg = sum(1 for r in results if r.verdict == "marginal")
    n_fail = sum(1 for r in results if r.verdict == "fail")
    n_skip = sum(1 for r in results if r.verdict == "skip")

    channels_out: dict[str, Any] = {}
    for channel in sorted(by_channel):
        ch_summary = _aggregate_channel(
            by_channel[channel], channel=channel, mode=mode, worst_n=worst_n,
        )
        channels_out[channel] = ch_summary.asdict()

    return {
        "mode": mode,
        "n_runs_total": len(results),
        "n_runs_pass": n_pass,
        "n_runs_marginal": n_marg,
        "n_runs_fail": n_fail,
        "n_runs_skip": n_skip,
        "channels": channels_out,
        "thresholds": {
            "metric": "norm_rmse_smoothed",
            "rise_pass_norm_rmse": 0.10,
            "rise_marginal_norm_rmse": 0.20,
            "fall_pass_norm_rmse": 0.15,
            "fall_marginal_norm_rmse": 0.25,
            "channel_pass_rate_pass": _PASS_RATE_PASS,
            "channel_pass_rate_marginal": _PASS_RATE_MARGINAL,
        },
    }


def _aggregate_channel(
    results: list[ValidationResult], *, channel: str, mode: str, worst_n: int
) -> ChannelSummary:
    # Bucket by (signed amplitude, direction).
    by_amp_dir: dict[tuple[float, str], list[ValidationResult]] = {}
    for r in results:
        if r.amplitude is None or r.direction is None:
            continue
        by_amp_dir.setdefault((float(r.amplitude), r.direction), []).append(r)

    groups: list[GroupSummary] = []
    for (amp, direction), rs in sorted(by_amp_dir.items()):
        groups.append(_aggregate_group(
            rs, key={"channel": channel, "amplitude": amp, "direction": direction},
            worst_n=worst_n,
        ))

    rise_vals = [
        v for v in (_norm_rmse_of(r.rise_metrics) for r in results) if v is not None
    ]
    fall_vals = [
        v for v in (_norm_rmse_of(r.fall_metrics) for r in results) if v is not None
    ]
    rise_vals_raw = [
        v for v in (_metric_value(r.rise_metrics, "norm_rmse") for r in results)
        if v is not None
    ]
    fall_vals_raw = [
        v for v in (_metric_value(r.fall_metrics, "norm_rmse") for r in results)
        if v is not None
    ]
    rise_ron = [
        v for v in (_metric_value(r.rise_metrics, "residual_over_noise") for r in results)
        if v is not None
    ]
    fall_ron = [
        v for v in (_metric_value(r.fall_metrics, "residual_over_noise") for r in results)
        if v is not None
    ]

    n_pass = sum(1 for r in results if r.verdict == "pass")
    n_marg = sum(1 for r in results if r.verdict == "marginal")
    n_fail = sum(1 for r in results if r.verdict == "fail")
    n_skip = sum(1 for r in results if r.verdict == "skip")
    n_scored = n_pass + n_marg + n_fail
    pass_rate = (n_pass / n_scored) if n_scored > 0 else float("nan")

    # Worst runs by smoothed rise nRMSE (verdict-driving metric).
    scored = [
        (r, _norm_rmse_of(r.rise_metrics))
        for r in results
    ]
    scored = [(r, v) for r, v in scored if v is not None]
    scored.sort(key=lambda rv: rv[1], reverse=True)
    worst_run_ids = [r.run_id for r, _ in scored[:worst_n]]

    return ChannelSummary(
        channel=channel,
        mode=mode,
        n_total=len(results),
        n_pass=n_pass, n_marginal=n_marg, n_fail=n_fail, n_skip=n_skip,
        pass_rate=pass_rate,
        verdict=(_channel_verdict(pass_rate) if np.isfinite(pass_rate) else "skip"),
        rise_norm_rmse=_quantiles(rise_vals),
        fall_norm_rmse=(_quantiles(fall_vals) if fall_vals else None),
        rise_norm_rmse_raw=(_quantiles(rise_vals_raw) if rise_vals_raw else None),
        fall_norm_rmse_raw=(_quantiles(fall_vals_raw) if fall_vals_raw else None),
        rise_residual_over_noise=(_quantiles(rise_ron) if rise_ron else None),
        fall_residual_over_noise=(_quantiles(fall_ron) if fall_ron else None),
        by_amp_direction=groups,
        worst_run_ids=worst_run_ids,
    )


def _aggregate_group(
    results: list[ValidationResult], *, key: dict[str, Any], worst_n: int
) -> GroupSummary:
    rise_vals = [
        v for v in (_norm_rmse_of(r.rise_metrics) for r in results) if v is not None
    ]
    fall_vals = [
        v for v in (_norm_rmse_of(r.fall_metrics) for r in results) if v is not None
    ]
    rise_vals_raw = [
        v for v in (_metric_value(r.rise_metrics, "norm_rmse") for r in results)
        if v is not None
    ]
    fall_vals_raw = [
        v for v in (_metric_value(r.fall_metrics, "norm_rmse") for r in results)
        if v is not None
    ]
    rise_ron = [
        v for v in (_metric_value(r.rise_metrics, "residual_over_noise") for r in results)
        if v is not None
    ]
    fall_ron = [
        v for v in (_metric_value(r.fall_metrics, "residual_over_noise") for r in results)
        if v is not None
    ]
    n_pass = sum(1 for r in results if r.verdict == "pass")
    n_marg = sum(1 for r in results if r.verdict == "marginal")
    n_fail = sum(1 for r in results if r.verdict == "fail")
    n_skip = sum(1 for r in results if r.verdict == "skip")

    scored = [(r, _norm_rmse_of(r.rise_metrics)) for r in results]
    scored = [(r, v) for r, v in scored if v is not None]
    scored.sort(key=lambda rv: rv[1], reverse=True)
    worst_run_ids = [r.run_id for r, _ in scored[:worst_n]]

    return GroupSummary(
        key=key,
        n_total=len(results),
        n_pass=n_pass, n_marginal=n_marg, n_fail=n_fail, n_skip=n_skip,
        rise_norm_rmse=_quantiles(rise_vals),
        fall_norm_rmse=(_quantiles(fall_vals) if fall_vals else None),
        rise_norm_rmse_raw=(_quantiles(rise_vals_raw) if rise_vals_raw else None),
        fall_norm_rmse_raw=(_quantiles(fall_vals_raw) if fall_vals_raw else None),
        rise_residual_over_noise=(_quantiles(rise_ron) if rise_ron else None),
        fall_residual_over_noise=(_quantiles(fall_ron) if fall_ron else None),
        worst_run_ids=worst_run_ids,
    )


__all__ = ["GroupSummary", "ChannelSummary", "aggregate_validation"]
