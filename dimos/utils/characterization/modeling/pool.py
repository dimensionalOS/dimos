# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Within-session pooling (direction + amplitude) and cross-mode comparison.

For each (channel, mode), this module decides:

  - **Direction pooling**: do forward and reverse fits agree at every
    amplitude? If yes, pool. If no, mark direction-asymmetric and keep
    separate.
  - **Amplitude pooling / gain schedule**: across amplitudes, is each of
    K/τ/L roughly constant (linear-in-amplitude plant), or does it follow
    a trend? Constant ⇒ pool to one value. Trend ⇒ report a linear gain
    schedule.

Mode pooling lives in ``compare_models`` since it's inherently
cross-session (modes are recorded per-session, not per-run).
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from dimos.utils.characterization.modeling.aggregate import GroupFit


# --------------------------------------------------------------------------- helpers

def _ci_overlap(a: dict[str, Any] | None, b: dict[str, Any] | None) -> bool:
    """True when two CI intervals overlap. Returns True conservatively when
    either is missing (we don't have evidence they disagree)."""
    if a is None or b is None:
        return True
    return not (a["ci_high"] < b["ci_low"] or b["ci_high"] < a["ci_low"])


def _combine_ivw(
    means: list[float], sigmas: list[float]
) -> tuple[float, float, float, float]:
    """Inverse-variance combine. Returns (mean, std, ci_low, ci_high).

    sigmas are 1σ values (half the 95% CI width / 1.96 from upstream).
    """
    means_a = np.asarray(means, dtype=float)
    sigmas_a = np.asarray(sigmas, dtype=float)
    finite = np.isfinite(means_a) & np.isfinite(sigmas_a) & (sigmas_a > 0)
    if not finite.any():
        if np.isfinite(means_a).any():
            v = means_a[np.isfinite(means_a)]
            m = float(np.mean(v))
            s = float(np.std(v, ddof=1)) if v.size > 1 else 0.0
            return (m, s, m - 1.96 * s, m + 1.96 * s)
        return (float("nan"), float("nan"), float("nan"), float("nan"))
    m = means_a[finite]
    sig = sigmas_a[finite]
    w = 1.0 / (sig ** 2)
    mean = float(np.sum(w * m) / np.sum(w))
    pooled_sigma = float(math.sqrt(1.0 / float(np.sum(w))))
    std = float(np.std(m, ddof=1)) if m.size > 1 else 0.0
    return (mean, std, mean - 1.96 * pooled_sigma, mean + 1.96 * pooled_sigma)


def _stats_to_pair(stats: dict[str, Any] | None) -> tuple[float, float] | None:
    """Convert a stats dict to (mean, sigma) for IVW combination."""
    if stats is None:
        return None
    mean = stats.get("mean")
    lo = stats.get("ci_low")
    hi = stats.get("ci_high")
    if mean is None or lo is None or hi is None:
        return None
    width = hi - lo
    sigma = (width / 2.0) / 1.96 if width > 0 else 0.0
    return (float(mean), float(sigma))


def _linear_regression_with_ci(
    x: np.ndarray, y: np.ndarray, sigma_y: np.ndarray | None = None
) -> dict[str, Any]:
    """OLS y = a + b*x with 95% CI on slope. Returns {slope, intercept,
    slope_ci, intercept_ci, n}. Falls back to NaN for n < 3.
    """
    n = int(x.size)
    if n < 3:
        return {
            "slope": float("nan"),
            "intercept": float("nan"),
            "slope_ci": [float("nan"), float("nan")],
            "intercept_ci": [float("nan"), float("nan")],
            "n": n,
        }
    if sigma_y is not None and np.all(np.isfinite(sigma_y)) and np.all(sigma_y > 0):
        w = 1.0 / (sigma_y ** 2)
    else:
        w = np.ones_like(x)
    sw = float(np.sum(w))
    swx = float(np.sum(w * x))
    swy = float(np.sum(w * y))
    swxx = float(np.sum(w * x * x))
    swxy = float(np.sum(w * x * y))
    denom = sw * swxx - swx ** 2
    if denom <= 0:
        return {
            "slope": float("nan"),
            "intercept": float("nan"),
            "slope_ci": [float("nan"), float("nan")],
            "intercept_ci": [float("nan"), float("nan")],
            "n": n,
        }
    slope = (sw * swxy - swx * swy) / denom
    intercept = (swy - slope * swx) / sw
    resid = y - (intercept + slope * x)
    dof = max(1, n - 2)
    s2 = float(np.sum(w * resid ** 2)) / dof
    var_slope = s2 * sw / denom
    var_intercept = s2 * swxx / denom
    se_slope = math.sqrt(max(var_slope, 0.0))
    se_intercept = math.sqrt(max(var_intercept, 0.0))
    return {
        "slope": float(slope),
        "intercept": float(intercept),
        "slope_ci": [float(slope - 1.96 * se_slope), float(slope + 1.96 * se_slope)],
        "intercept_ci": [float(intercept - 1.96 * se_intercept), float(intercept + 1.96 * se_intercept)],
        "n": n,
    }


# --------------------------------------------------------------------------- per-session pool

def pool_session(group_fits: list[GroupFit], *, mode: str) -> dict[str, Any]:
    """Pool a session's per-group fits across direction and amplitude.

    Returns a dict with the ``model_summary.json`` shape.
    """
    by_channel: dict[str, list[GroupFit]] = {}
    for g in group_fits:
        ch = g.key.get("channel")
        if not isinstance(ch, str):
            continue
        by_channel.setdefault(ch, []).append(g)

    channels_out: dict[str, Any] = {}
    diag = {
        "n_groups_total": len(group_fits),
        "n_groups_with_fit": 0,
        "n_groups_without_fit": 0,
    }
    for g in group_fits:
        if g.K is not None and g.tau is not None and g.L is not None:
            diag["n_groups_with_fit"] += 1
        else:
            diag["n_groups_without_fit"] += 1

    for channel, groups in sorted(by_channel.items()):
        channels_out[channel] = _pool_channel(groups)

    return {
        "mode": mode,
        "channels": channels_out,
        "diagnostics": diag,
    }


def _pool_channel(groups: list[GroupFit]) -> dict[str, Any]:
    """Pool one channel's groups across direction + amplitude."""
    # Bucket by (|amplitude|, direction).
    by_amp_dir: dict[tuple[float, str], GroupFit] = {}
    for g in groups:
        amp = g.key.get("amplitude")
        direction = g.key.get("direction")
        if amp is None or direction is None:
            continue
        by_amp_dir[(abs(float(amp)), direction)] = g

    # Direction symmetry: at each amplitude, do forward and reverse CIs
    # overlap on K/τ/L? If any amplitude shows non-overlap on any param,
    # mark direction_asymmetric=True.
    amps = sorted({a for (a, _d) in by_amp_dir})
    direction_asymmetric = False
    for amp in amps:
        fwd = by_amp_dir.get((amp, "forward"))
        rev = by_amp_dir.get((amp, "reverse"))
        if fwd is None or rev is None:
            continue
        for p in ("K", "tau", "L"):
            if not _ci_overlap(getattr(fwd, p), getattr(rev, p)):
                direction_asymmetric = True
                break
        if direction_asymmetric:
            break

    # Build a flat list of (amplitude_signed, direction, GroupFit) for
    # downstream pooling.
    per_amp_entries: list[dict[str, Any]] = []

    if direction_asymmetric:
        # Keep per-amplitude per-direction entries.
        for (abs_amp, direction), g in sorted(by_amp_dir.items()):
            per_amp_entries.append({
                "amplitude": float(g.key["amplitude"]),
                "direction": direction,
                "K": g.K, "tau": g.tau, "L": g.L,
                "n_runs_kept": g.n_runs_kept,
            })
        pool_input = [
            (abs(float(g.key["amplitude"])), g) for g in groups
        ]
    else:
        # Pool fwd+rev at each amplitude.
        for amp in amps:
            fwd = by_amp_dir.get((amp, "forward"))
            rev = by_amp_dir.get((amp, "reverse"))
            entry = {
                "amplitude": float(amp),
                "direction": "pooled",
                "K": _pool_two(fwd, rev, "K"),
                "tau": _pool_two(fwd, rev, "tau"),
                "L": _pool_two(fwd, rev, "L"),
                "n_runs_kept": (fwd.n_runs_kept if fwd else 0)
                               + (rev.n_runs_kept if rev else 0),
            }
            per_amp_entries.append(entry)
        pool_input = [(amp, _entry_to_fakegroup(e)) for amp, e in zip(amps, per_amp_entries)]

    # Amplitude pooling: linear regression of param ~ |amplitude|. If the
    # slope CI contains 0, treat the param as constant in amplitude;
    # otherwise report a gain schedule.
    linear_in_amp: dict[str, bool] = {}
    gain_schedule: dict[str, Any] = {}
    pooled: dict[str, Any] = {}
    for p in ("K", "tau", "L"):
        amps_arr = np.asarray([a for a, _g in pool_input], dtype=float)
        means = []
        sigmas = []
        for _a, g in pool_input:
            pair = _stats_to_pair(getattr(g, p))
            if pair is None:
                means.append(float("nan"))
                sigmas.append(float("nan"))
            else:
                means.append(pair[0])
                sigmas.append(pair[1])
        means_a = np.asarray(means, dtype=float)
        sigmas_a = np.asarray(sigmas, dtype=float)
        finite = np.isfinite(means_a)
        if int(finite.sum()) >= 3:
            reg = _linear_regression_with_ci(
                amps_arr[finite], means_a[finite],
                sigma_y=(sigmas_a[finite] if np.all(np.isfinite(sigmas_a[finite])) else None),
            )
            slope_ci = reg["slope_ci"]
            ci_includes_zero = slope_ci[0] <= 0.0 <= slope_ci[1]
            linear_in_amp[p] = bool(ci_includes_zero)
            if not ci_includes_zero:
                gain_schedule[p] = reg
            else:
                gain_schedule[p] = None
        else:
            linear_in_amp[p] = True  # not enough evidence to reject constant
            gain_schedule[p] = None

        # Single pooled value (always reported, even if gain-scheduled —
        # the gain_schedule field tells you whether to use it or the schedule).
        mean, std, lo, hi = _combine_ivw(means, sigmas)
        n_groups = int(np.isfinite(means_a).sum())
        pooled[p] = {
            "mean": mean, "std": std, "ci_low": lo, "ci_high": hi,
            "n_groups": n_groups,
        }

    return {
        "direction_asymmetric": direction_asymmetric,
        "linear_in_amplitude": linear_in_amp,
        "pooled": pooled,
        "gain_schedule": gain_schedule,
        "per_amplitude": per_amp_entries,
    }


def _pool_two(
    a: GroupFit | None, b: GroupFit | None, p: str
) -> dict[str, Any] | None:
    """Inverse-variance combine two GroupFit stats dicts."""
    pairs = []
    for g in (a, b):
        if g is None:
            continue
        pair = _stats_to_pair(getattr(g, p))
        if pair is not None:
            pairs.append(pair)
    if not pairs:
        return None
    means = [pp[0] for pp in pairs]
    sigmas = [pp[1] for pp in pairs]
    mean, std, lo, hi = _combine_ivw(means, sigmas)
    return {
        "mean": mean, "std": std, "ci_low": lo, "ci_high": hi,
        "n": len(pairs),
    }


class _FakeGroup:
    """Lightweight stand-in so amplitude pooling can treat
    direction-pooled and per-direction entries uniformly."""

    def __init__(self, K, tau, L) -> None:
        self.K = K
        self.tau = tau
        self.L = L


def _entry_to_fakegroup(entry: dict[str, Any]) -> _FakeGroup:
    return _FakeGroup(K=entry["K"], tau=entry["tau"], L=entry["L"])


# --------------------------------------------------------------------------- cross-session

def compare_models(
    default_summary: dict[str, Any], rage_summary: dict[str, Any]
) -> dict[str, Any]:
    """Compare default-mode and rage-mode ``model_summary.json`` dicts.

    Verdict per (channel, parameter):
      - identical:  |ratio - 1| < 0.05
      - equivalent: 0.05 ≤ |ratio - 1| ≤ 0.20 OR CIs overlap
      - differs:    otherwise

    Channel-level verdict is the worst per-parameter verdict.
    """
    out: dict[str, Any] = {
        "default_mode": default_summary.get("mode"),
        "rage_mode": rage_summary.get("mode"),
        "channels": {},
    }
    default_channels = default_summary.get("channels", {})
    rage_channels = rage_summary.get("channels", {})
    for channel in sorted(set(default_channels) | set(rage_channels)):
        d = default_channels.get(channel) or {}
        r = rage_channels.get(channel) or {}
        d_pool = d.get("pooled") or {}
        r_pool = r.get("pooled") or {}

        per_param: dict[str, Any] = {}
        worst = "identical"
        for p in ("K", "tau", "L"):
            ds = d_pool.get(p)
            rs = r_pool.get(p)
            verdict = _compare_one_param(ds, rs)
            per_param[p] = verdict
            worst = _worst_verdict(worst, verdict["verdict"])

        out["channels"][channel] = {
            "verdict": worst,
            "params": per_param,
            "default_pooled": d_pool,
            "rage_pooled": r_pool,
        }
    return out


def _compare_one_param(
    ds: dict[str, Any] | None, rs: dict[str, Any] | None
) -> dict[str, Any]:
    if ds is None or rs is None:
        return {
            "verdict": "missing",
            "default": ds, "rage": rs,
            "ratio": None, "ci_overlap": None,
        }
    d_mean = ds.get("mean")
    r_mean = rs.get("mean")
    if d_mean is None or r_mean is None or not (math.isfinite(d_mean) and math.isfinite(r_mean)):
        return {
            "verdict": "missing",
            "default": ds, "rage": rs,
            "ratio": None, "ci_overlap": None,
        }
    if abs(d_mean) < 1e-9:
        ratio = float("inf") if abs(r_mean) > 1e-9 else 1.0
    else:
        ratio = r_mean / d_mean
    overlap = _ci_overlap(ds, rs)
    deviation = abs(ratio - 1.0) if math.isfinite(ratio) else float("inf")
    if deviation < 0.05:
        verdict = "identical"
    elif overlap or deviation <= 0.20:
        verdict = "equivalent"
    else:
        verdict = "differs"
    return {
        "verdict": verdict,
        "default": ds, "rage": rs,
        "ratio": ratio, "ci_overlap": overlap,
    }


def _worst_verdict(a: str, b: str) -> str:
    order = {"identical": 0, "equivalent": 1, "differs": 2, "missing": 3}
    return a if order.get(a, 0) >= order.get(b, 0) else b


def compare_rise_fall(
    rise_summary: dict[str, Any], fall_summary: dict[str, Any]
) -> dict[str, Any]:
    """Compare rise vs fall pooled FOPDT params per channel.

    Same verdict scheme as ``compare_models`` (identical / equivalent /
    differs / missing) — surfaces accel-vs-decel asymmetry. Use case:
    if the plant decelerates much slower than it accelerates, a single
    FOPDT can't represent both phases and the controller may need
    direction-aware feedforward.
    """
    out: dict[str, Any] = {"channels": {}}
    rise_channels = rise_summary.get("channels", {})
    fall_channels = fall_summary.get("channels", {})
    for channel in sorted(set(rise_channels) | set(fall_channels)):
        rch = rise_channels.get(channel) or {}
        fch = fall_channels.get(channel) or {}
        rp = rch.get("pooled") or {}
        fp = fch.get("pooled") or {}

        per_param: dict[str, Any] = {}
        worst = "identical"
        for p in ("K", "tau", "L"):
            verdict = _compare_one_param(rp.get(p), fp.get(p))
            # Reuse the verdict shape but rename keys for clarity.
            per_param[p] = {
                "verdict": verdict["verdict"],
                "rise": verdict["default"],
                "fall": verdict["rage"],
                "ratio_fall_over_rise": verdict["ratio"],
                "ci_overlap": verdict["ci_overlap"],
            }
            worst = _worst_verdict(worst, verdict["verdict"])

        out["channels"][channel] = {
            "verdict": worst,
            "params": per_param,
        }
    return out


__all__ = ["compare_models", "compare_rise_fall", "pool_session"]
