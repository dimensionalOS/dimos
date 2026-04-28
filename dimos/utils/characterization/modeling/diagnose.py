# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Residual diagnostics — only runs when validation is marginal or failing.

Examines residual patterns and produces a structured diagnosis: a ranked
list of model-upgrade recommendations with evidence for each. Pure
analysis, no commitment to act on the recommendations.

Patterns mapped to upgrades:

  - DC-offset residuals          → bias / wrong K
  - Residual scales with |amp|   → nonlinearity in amplitude
  - Persistent forward/reverse   → directional asymmetry
    sign difference
  - Spike at step edge           → wrong L (deadtime mis-estimated)
  - Strong autocorrelation /     → unmodeled second-order dynamics
    oscillation in residual
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

from dimos.utils.characterization.modeling.validate_run import ValidationResult


# Magnitude thresholds for each pattern. These are intentionally
# coarse — diagnosis is a ranking, not a hypothesis test. The
# magnitudes tell you "this effect is bigger than that effect," not
# "this is statistically significant."
_DC_OFFSET_FRACTION = 0.05         # |mean(resid)| / |amp| > 5% → flag bias
_AMP_TREND_R = 0.6                  # Pearson r > 0.6 between |amp| and norm_rmse
_DIR_DIFF_FRACTION = 0.30           # |fwd_norm_rmse - rev_norm_rmse| / max > 30%
_EDGE_SPIKE_RATIO = 1.8             # max-abs near-edge / median |resid| > 1.8
_AUTOCORR_LAG1 = 0.5                # lag-1 autocorrelation > 0.5


@dataclass
class Finding:
    """One diagnostic finding."""
    pattern: str           # short ID (e.g. "amp_nonlinearity")
    severity: str          # "high" | "medium" | "low"
    evidence: dict[str, Any]
    recommendation: str

    def asdict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class Diagnosis:
    channel: str
    findings: list[Finding] = field(default_factory=list)

    def asdict(self) -> dict[str, Any]:
        return {"channel": self.channel,
                "findings": [f.asdict() for f in self.findings]}


# --------------------------------------------------------------------------- pattern detectors

def _residuals_for(r: ValidationResult) -> np.ndarray | None:
    """Reconstruct (y_meas - y_pred) on the rise-edge window for one run.
    Returns None when traces weren't kept or are too short.
    """
    if r._t_meas is None or r._y_meas is None or r._y_pred is None:
        return None
    step_t = (r.extra or {}).get("step_t")
    active_end_t = (r.extra or {}).get("active_end_t")
    if step_t is None or active_end_t is None:
        return None
    mask = (r._t_meas >= float(step_t)) & (r._t_meas <= float(active_end_t))
    if int(mask.sum()) < 8:
        return None
    return (r._y_meas - r._y_pred)[mask]


def _check_dc_offset(results: list[ValidationResult]) -> Finding | None:
    """Persistent residual mean (across runs) → bias / K error."""
    means = []
    for r in results:
        resid = _residuals_for(r)
        if resid is None or r.amplitude is None or abs(r.amplitude) < 1e-9:
            continue
        means.append(float(np.mean(resid) / r.amplitude))
    if len(means) < 3:
        return None
    arr = np.asarray(means)
    avg_offset = float(np.mean(arr))
    consistent = float(np.std(arr)) < 0.5 * abs(avg_offset) if abs(avg_offset) > 1e-6 else False
    if abs(avg_offset) > _DC_OFFSET_FRACTION and consistent:
        sev = "high" if abs(avg_offset) > 0.10 else "medium"
        return Finding(
            pattern="dc_offset",
            severity=sev,
            evidence={
                "mean_normalized_residual": avg_offset,
                "std_across_runs": float(np.std(arr)),
                "n_runs": len(means),
            },
            recommendation=(
                "Residual mean is consistently nonzero across runs — K (steady-state "
                "gain) is mis-estimated by ~{:.1%}. Consider re-fitting K with a "
                "longer steady-state window or check for biased baseline."
            ).format(abs(avg_offset)),
        )
    return None


def _check_amplitude_trend(results: list[ValidationResult]) -> Finding | None:
    """Residual magnitude scaling with |amplitude| → nonlinearity."""
    pairs = []
    for r in results:
        if r.amplitude is None:
            continue
        nr = (r.rise_metrics or {}).get("norm_rmse")
        if nr is None or not np.isfinite(nr):
            continue
        pairs.append((abs(float(r.amplitude)), float(nr)))
    if len(pairs) < 6:
        return None
    amps = np.asarray([a for a, _ in pairs])
    nrm = np.asarray([n for _, n in pairs])
    if np.std(amps) < 1e-9 or np.std(nrm) < 1e-9:
        return None
    r_pearson = float(np.corrcoef(amps, nrm)[0, 1])
    if abs(r_pearson) > _AMP_TREND_R:
        sev = "high" if abs(r_pearson) > 0.85 else "medium"
        return Finding(
            pattern="amplitude_nonlinearity",
            severity=sev,
            evidence={
                "pearson_r_amp_vs_norm_rmse": r_pearson,
                "n_runs": len(pairs),
                "amp_range": (float(amps.min()), float(amps.max())),
                "norm_rmse_range": (float(nrm.min()), float(nrm.max())),
            },
            recommendation=(
                "Normalized RMSE correlates with |amplitude| (Pearson r={:.2f}); "
                "the plant is nonlinear in amplitude. Consider gain-scheduling "
                "(if not already enabled), saturation modelling, or refitting "
                "per amplitude band."
            ).format(r_pearson),
        )
    return None


def _check_direction_asymmetry(results: list[ValidationResult]) -> Finding | None:
    """Forward-vs-reverse divergence in normalized RMSE."""
    fwd, rev = [], []
    for r in results:
        nr = (r.rise_metrics or {}).get("norm_rmse")
        if nr is None or not np.isfinite(nr):
            continue
        if r.direction == "forward":
            fwd.append(float(nr))
        elif r.direction == "reverse":
            rev.append(float(nr))
    if len(fwd) < 3 or len(rev) < 3:
        return None
    f_med, r_med = float(np.median(fwd)), float(np.median(rev))
    denom = max(f_med, r_med, 1e-9)
    diff_frac = abs(f_med - r_med) / denom
    if diff_frac > _DIR_DIFF_FRACTION:
        sev = "high" if diff_frac > 0.6 else "medium"
        return Finding(
            pattern="direction_asymmetry",
            severity=sev,
            evidence={
                "forward_median_norm_rmse": f_med,
                "reverse_median_norm_rmse": r_med,
                "diff_fraction": diff_frac,
                "n_forward": len(fwd), "n_reverse": len(rev),
            },
            recommendation=(
                "Forward and reverse runs show systematically different residuals "
                "(forward median nRMSE={:.2%}, reverse median nRMSE={:.2%}). "
                "The plant is direction-asymmetric — controller may need direction-"
                "aware feedforward or per-direction gain schedule."
            ).format(f_med, r_med),
        )
    return None


def _check_edge_spike(results: list[ValidationResult]) -> Finding | None:
    """Residual concentrated near step edge → L mis-estimated."""
    ratios = []
    for r in results:
        resid = _residuals_for(r)
        if resid is None or resid.size < 16:
            continue
        # Use first 20% of rise window as "near-edge".
        n_edge = max(4, int(0.2 * resid.size))
        near_edge = float(np.max(np.abs(resid[:n_edge])))
        median_abs = float(np.median(np.abs(resid)))
        if median_abs < 1e-9:
            continue
        ratios.append(near_edge / median_abs)
    if len(ratios) < 3:
        return None
    median_ratio = float(np.median(ratios))
    if median_ratio > _EDGE_SPIKE_RATIO:
        sev = "high" if median_ratio > 3.0 else "medium"
        return Finding(
            pattern="edge_spike",
            severity=sev,
            evidence={
                "median_ratio_edge_to_median": median_ratio,
                "n_runs": len(ratios),
            },
            recommendation=(
                "Residual is concentrated near the step edge (median ratio "
                "{:.2f}× the rest-of-window median). L (deadtime) is likely "
                "mis-estimated. Re-check deadtime extraction or use a finer "
                "L grid in the fitter."
            ).format(median_ratio),
        )
    return None


def _check_oscillation(results: list[ValidationResult]) -> Finding | None:
    """Lag-1 residual autocorrelation → unmodeled higher-order dynamics."""
    autocorrs = []
    for r in results:
        resid = _residuals_for(r)
        if resid is None or resid.size < 16:
            continue
        a = resid - float(np.mean(resid))
        denom = float(np.sum(a * a))
        if denom < 1e-12:
            continue
        ac1 = float(np.sum(a[1:] * a[:-1]) / denom)
        autocorrs.append(ac1)
    if len(autocorrs) < 3:
        return None
    median_ac = float(np.median(autocorrs))
    if abs(median_ac) > _AUTOCORR_LAG1:
        sev = "high" if abs(median_ac) > 0.75 else "medium"
        return Finding(
            pattern="oscillation",
            severity=sev,
            evidence={
                "median_lag1_autocorr": median_ac,
                "n_runs": len(autocorrs),
            },
            recommendation=(
                "Residuals show strong lag-1 autocorrelation (median {:.2f}); "
                "the plant has dynamics beyond a single first-order pole. "
                "Consider upgrading to a second-order model (FOPDT2 or "
                "underdamped second-order)."
            ).format(median_ac),
        )
    return None


# --------------------------------------------------------------------------- main

def diagnose_channel(
    results: list[ValidationResult], *, channel: str
) -> Diagnosis:
    """Run all pattern checks for one channel; return ranked findings."""
    diag = Diagnosis(channel=channel)
    for check in (_check_dc_offset, _check_amplitude_trend,
                  _check_direction_asymmetry, _check_edge_spike,
                  _check_oscillation):
        f = check(results)
        if f is not None:
            diag.findings.append(f)
    sev_rank = {"high": 0, "medium": 1, "low": 2}
    diag.findings.sort(key=lambda f: sev_rank.get(f.severity, 3))
    return diag


def diagnose_validation(
    results: list[ValidationResult], summary: dict[str, Any]
) -> dict[str, Any]:
    """Top-level diagnosis: run per-channel checks for any channel that's
    marginal or failing in the aggregated summary.

    Returns a dict shaped for ``diagnosis.json``::

        {
          "channels": {
            "vx": {"channel": ..., "findings": [...]},
            "wz": {"channel": ..., "findings": [...]}
          }
        }
    """
    channels = (summary.get("channels") or {})
    per_channel: dict[str, list[ValidationResult]] = {}
    for r in results:
        if r.channel:
            per_channel.setdefault(r.channel, []).append(r)
    out: dict[str, Any] = {"channels": {}}
    for ch, ch_results in sorted(per_channel.items()):
        verdict = (channels.get(ch) or {}).get("verdict")
        if verdict not in ("marginal", "fail"):
            continue
        d = diagnose_channel(ch_results, channel=ch)
        out["channels"][ch] = d.asdict()
    return out


__all__ = ["Diagnosis", "Finding", "diagnose_channel", "diagnose_validation"]
