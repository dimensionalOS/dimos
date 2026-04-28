# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Per-run FOPDT validation.

Predicts one held-out run's response using a fitted model summary,
compares prediction to measurement, returns metrics and a pass/fail
verdict.

Pipeline parallels ``per_run.fit_run``:

  - ``analyze_run.load_run`` for the trace
  - ``parse_recipe_name`` (with ``_detect_channel_and_amplitude`` fallback)
  - ZOH-resample cmd onto meas timestamps
  - ``simulate_fopdt`` for the prediction
  - residual metrics on rise / fall edges and overall

Pass/marginal/fail thresholds are starting points (per the Session 2
plan); the report is expected to surface the actual nRMSE distribution
so they can be revised with evidence.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from dimos.utils.characterization.modeling.fopdt import FopdtParams
from dimos.utils.characterization.modeling.per_run import (
    _detect_channel_and_amplitude,
    parse_recipe_name,
)
from dimos.utils.characterization.modeling.simulate import simulate_fopdt


# Thresholds for pass / marginal / fail verdict on a single run. These
# apply to *smoothed* normalized RMSE = SmoothedRMSE / |amplitude|.
# Smoothing is Sav-Gol over the residual to suppress leg-jitter the
# (deterministic) FOPDT model cannot predict. Raw nRMSE is still
# computed and surfaced alongside for traceability — it just isn't the
# verdict driver.
_RISE_PASS = 0.10
_RISE_MARGINAL = 0.20
_FALL_PASS = 0.15
_FALL_MARGINAL = 0.25

# Sav-Gol residual smoothing (window in samples, polynomial order). At
# ~50 Hz this gives ~220 ms window — suppresses gait-cadence noise
# (~2-5 Hz on the Go2 trot) without distorting the FOPDT-scale dynamics
# (~τ ≈ 0.25 s). Starting point per the Pre-Session-3 plan; revise once
# we see the new distribution.
_RESID_SMOOTH_WINDOW = 11
_RESID_SMOOTH_POLY = 3


@dataclass
class ValidationResult:
    run_id: str
    run_dir: str
    recipe: str
    channel: str | None
    amplitude: float | None
    direction: str | None
    mode: str
    split: str | None

    # Params actually used (after gain-schedule lookup). NaN when no
    # model was available for this channel.
    used_K: float
    used_tau: float
    used_L: float
    used_K_fall: float | None = None
    used_tau_fall: float | None = None
    used_L_fall: float | None = None

    # Per-edge and overall metrics. Each is {"rmse", "max_abs",
    # "norm_rmse", "n_samples", "r2_oos"} or None when the edge had
    # too few samples.
    rise_metrics: dict[str, Any] | None = None
    fall_metrics: dict[str, Any] | None = None
    overall_metrics: dict[str, Any] | None = None

    verdict: str = "skip"           # "pass" | "marginal" | "fail" | "skip"
    skip_reason: str | None = None

    extra: dict[str, Any] = field(default_factory=dict)

    # In-memory only — used by the report's plot stage. Excluded from JSON.
    _t_meas: np.ndarray | None = field(default=None, repr=False, compare=False)
    _y_meas: np.ndarray | None = field(default=None, repr=False, compare=False)
    _y_pred: np.ndarray | None = field(default=None, repr=False, compare=False)
    _cmd_at_meas: np.ndarray | None = field(default=None, repr=False, compare=False)

    def asdict(self) -> dict[str, Any]:
        d = asdict(self)
        # Strip private numpy-array fields from the JSON view.
        for k in ("_t_meas", "_y_meas", "_y_pred", "_cmd_at_meas"):
            d.pop(k, None)
        return d


# --------------------------------------------------------------------------- model lookup

def _eval_param(channel_summary: dict[str, Any], param: str, abs_amp: float) -> float:
    """Evaluate K/τ/L at this run's |amplitude|, honoring the gain schedule.

    Falls back to the pooled mean when the schedule is absent or when the
    parameter was found linear-in-amplitude.
    """
    if not channel_summary:
        return float("nan")
    linear_map = channel_summary.get("linear_in_amplitude") or {}
    is_linear = bool(linear_map.get(param, True))
    if not is_linear:
        sched = (channel_summary.get("gain_schedule") or {}).get(param)
        if sched and sched.get("slope") is not None and sched.get("intercept") is not None:
            slope = float(sched["slope"])
            intercept = float(sched["intercept"])
            if np.isfinite(slope) and np.isfinite(intercept):
                return float(intercept + slope * abs_amp)
    pooled = (channel_summary.get("pooled") or {}).get(param) or {}
    mean = pooled.get("mean")
    return float(mean) if mean is not None else float("nan")


def _fopdt_from_summary(
    summary: dict[str, Any] | None, channel: str, abs_amp: float
) -> FopdtParams | None:
    """Build a FopdtParams from a pooled summary at the given amplitude.
    Returns None when the channel isn't present or all params are NaN."""
    if summary is None:
        return None
    ch = (summary.get("channels") or {}).get(channel)
    if not ch:
        return None
    K = _eval_param(ch, "K", abs_amp)
    tau = _eval_param(ch, "tau", abs_amp)
    L = _eval_param(ch, "L", abs_amp)
    if not (np.isfinite(K) and np.isfinite(tau) and np.isfinite(L)):
        return None
    return FopdtParams(
        K=K, tau=tau, L=L,
        K_ci=(K, K), tau_ci=(tau, tau), L_ci=(L, L),
        rmse=0.0, r_squared=1.0, n_samples=0,
        fit_window_s=(0.0, 0.0), degenerate=False, converged=True,
    )


# --------------------------------------------------------------------------- ZOH cmd resample

def _zoh_resample(
    t_query: np.ndarray, t_grid: np.ndarray, vals: np.ndarray, *, left: float
) -> np.ndarray:
    """Same ZOH lookup as ``simulate._zoh_at`` — duplicated here so this
    module doesn't reach into a private helper."""
    idx = np.searchsorted(t_grid, t_query, side="right") - 1
    out = np.empty_like(t_query, dtype=float)
    pre = idx < 0
    valid = ~pre
    out[pre] = left
    if valid.any():
        out[valid] = vals[idx[valid]]
    return out


# --------------------------------------------------------------------------- metrics

def _smooth_residual(resid: np.ndarray) -> np.ndarray:
    """Sav-Gol smooth the residual. Falls back to a centered moving
    average when there are too few samples for the configured window.
    """
    n = int(resid.size)
    if n < 4:
        return resid.astype(float, copy=True)
    if n >= _RESID_SMOOTH_WINDOW and _RESID_SMOOTH_POLY < _RESID_SMOOTH_WINDOW:
        from scipy.signal import savgol_filter
        return savgol_filter(resid, _RESID_SMOOTH_WINDOW, _RESID_SMOOTH_POLY,
                             mode="nearest")
    # Tiny window: odd, < n, polyorder ≤ window-1.
    w = max(3, n if n % 2 == 1 else n - 1)
    p = min(2, w - 1)
    from scipy.signal import savgol_filter
    return savgol_filter(resid, w, p, mode="nearest")


def _compute_metrics(
    y_meas: np.ndarray, y_pred: np.ndarray,
    *, amp: float, noise_std: float | None = None,
) -> dict[str, Any] | None:
    """Residual metrics, raw and smoothed.

    Output keys:
      - ``rmse``, ``max_abs``, ``norm_rmse`` — raw residual stats.
      - ``rmse_smoothed``, ``norm_rmse_smoothed`` — same on Sav-Gol
        smoothed residual. Smoothed nRMSE drives the verdict.
      - ``residual_over_noise`` — ``rmse_smoothed / noise_std``. When
        this is ≲ 2, the model is at the channel's noise ceiling.
        ``None`` when the noise floor wasn't available for this run.
      - ``r2_oos`` — out-of-sample R² on the raw residual.
      - ``n_samples`` — fit-window sample count.
    """
    if y_meas.size == 0 or y_pred.size == 0 or y_meas.size != y_pred.size:
        return None
    resid = y_meas - y_pred
    rmse = float(np.sqrt(np.mean(resid ** 2)))
    max_abs = float(np.max(np.abs(resid)))
    norm_rmse = rmse / abs(amp) if abs(amp) > 1e-9 else float("nan")

    resid_s = _smooth_residual(resid)
    rmse_s = float(np.sqrt(np.mean(resid_s ** 2)))
    norm_rmse_s = rmse_s / abs(amp) if abs(amp) > 1e-9 else float("nan")

    if noise_std is not None and noise_std > 0:
        residual_over_noise = float(rmse_s / noise_std)
    else:
        residual_over_noise = None

    y_mean = float(np.mean(y_meas))
    ss_tot = float(np.sum((y_meas - y_mean) ** 2))
    ss_res = float(np.sum(resid ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    return {
        "rmse": rmse,
        "max_abs": max_abs,
        "norm_rmse": norm_rmse,
        "rmse_smoothed": rmse_s,
        "norm_rmse_smoothed": norm_rmse_s,
        "residual_over_noise": residual_over_noise,
        "noise_std": (float(noise_std) if noise_std is not None else None),
        "n_samples": int(y_meas.size),
        "r2_oos": r2,
    }


def _verdict(rise: dict[str, Any] | None, fall: dict[str, Any] | None) -> str:
    """Apply pass/marginal/fail thresholds to per-edge metrics.

    Driven by the *smoothed* nRMSE — see Pre-Session-3 metric note.
    """
    if rise is None:
        return "skip"
    rise_n = rise.get("norm_rmse_smoothed", float("nan"))
    if not np.isfinite(rise_n):
        return "skip"
    fall_n = (fall or {}).get("norm_rmse_smoothed")
    fall_finite = fall_n is not None and np.isfinite(fall_n)

    rise_pass = rise_n <= _RISE_PASS
    rise_marginal = rise_n <= _RISE_MARGINAL
    fall_pass = (not fall_finite) or fall_n <= _FALL_PASS
    fall_marginal = (not fall_finite) or fall_n <= _FALL_MARGINAL

    if rise_pass and fall_pass:
        return "pass"
    if rise_marginal and fall_marginal:
        return "marginal"
    return "fail"


# --------------------------------------------------------------------------- main entry

def validate_run(
    run_dir: Path,
    *,
    rise_summary: dict[str, Any],
    fall_summary: dict[str, Any] | None = None,
    mode: str = "default",
    keep_traces: bool = True,
) -> ValidationResult:
    """Validate one run against a fitted model.

    Parameters
    ----------
    run_dir : Path
        Run directory (one of ``session_<ts>/0NN_<recipe>``).
    rise_summary : dict
        ``rise`` sub-dict of ``model_summary.json`` (or the top-level
        summary when only one set of params is provided).
    fall_summary : dict, optional
        ``fall`` sub-dict. If provided, the simulator runs with separate
        rise / fall dynamics; otherwise it's symmetric.
    mode : str
        Mode label (default / rage), threaded through into the result.
    keep_traces : bool
        When True, the result keeps ``_t_meas`` / ``_y_meas`` / ``_y_pred``
        in memory for the report's plot stage. Set False when validating
        many runs and only the JSON metrics are needed.
    """
    from dimos.utils.characterization.scripts.analyze_run import (
        _channel_arrays,
        load_run,
    )

    run_dir = Path(run_dir).expanduser().resolve()
    run_id = run_dir.name

    # ----- load + parse ----------------------------------------------------
    try:
        run = load_run(run_dir)
    except Exception as e:
        return ValidationResult(
            run_id=run_id, run_dir=str(run_dir), recipe="<unknown>",
            channel=None, amplitude=None, direction=None,
            mode=mode, split=None,
            used_K=float("nan"), used_tau=float("nan"), used_L=float("nan"),
            skip_reason=f"load_run failed: {type(e).__name__}: {e}",
        )

    recipe = run.metadata["recipe"]["name"]
    test_type = run.metadata["recipe"]["test_type"]
    if test_type != "step":
        return ValidationResult(
            run_id=run_id, run_dir=str(run_dir), recipe=recipe,
            channel=None, amplitude=None, direction=None,
            mode=mode, split=None,
            used_K=float("nan"), used_tau=float("nan"), used_L=float("nan"),
            skip_reason=f"not a step recipe (test_type={test_type})",
        )

    parsed = parse_recipe_name(recipe) or _detect_channel_and_amplitude(run)
    if parsed is None:
        return ValidationResult(
            run_id=run_id, run_dir=str(run_dir), recipe=recipe,
            channel=None, amplitude=None, direction=None,
            mode=mode, split=None,
            used_K=float("nan"), used_tau=float("nan"), used_L=float("nan"),
            skip_reason=f"could not infer channel/amplitude for recipe {recipe!r}",
        )
    channel, amplitude = parsed
    direction = "forward" if amplitude > 0 else "reverse"
    split = "train" if direction == "forward" else "validate"
    abs_amp = abs(amplitude)

    # ----- model lookup ----------------------------------------------------
    rise_params = _fopdt_from_summary(rise_summary, channel, abs_amp)
    if rise_params is None:
        return ValidationResult(
            run_id=run_id, run_dir=str(run_dir), recipe=recipe,
            channel=channel, amplitude=amplitude, direction=direction,
            mode=mode, split=split,
            used_K=float("nan"), used_tau=float("nan"), used_L=float("nan"),
            skip_reason=f"no rise model for channel {channel}",
        )
    fall_params = _fopdt_from_summary(fall_summary, channel, abs_amp) if fall_summary else None

    # ----- pull data, find step edge --------------------------------------
    cmd_arr, meas_arr = _channel_arrays(run, channel)
    cmd_ts = run.cmd_ts_rel
    meas_ts = run.meas_ts_rel

    if meas_ts.size < 4:
        return ValidationResult(
            run_id=run_id, run_dir=str(run_dir), recipe=recipe,
            channel=channel, amplitude=amplitude, direction=direction,
            mode=mode, split=split,
            used_K=rise_params.K, used_tau=rise_params.tau, used_L=rise_params.L,
            skip_reason="fewer than 4 measured samples",
        )

    nonzero = np.flatnonzero(np.abs(cmd_arr) > 1e-6)
    if nonzero.size == 0:
        return ValidationResult(
            run_id=run_id, run_dir=str(run_dir), recipe=recipe,
            channel=channel, amplitude=amplitude, direction=direction,
            mode=mode, split=split,
            used_K=rise_params.K, used_tau=rise_params.tau, used_L=rise_params.L,
            skip_reason="no nonzero command on parsed channel",
        )
    step_t = float(cmd_ts[nonzero[0]])
    duration = float(run.metadata["recipe"]["duration_s"])
    active_end_t = step_t + duration
    post_roll_s = float(run.metadata["recipe"].get("post_roll_s", 1.0))
    fall_end_t = active_end_t + post_roll_s

    # Pre-step baseline (consistent with per_run.fit_run).
    pre_mask = meas_ts < step_t
    baseline = float(np.mean(meas_arr[pre_mask])) if pre_mask.any() else 0.0
    y_meas = meas_arr - baseline

    # Resample cmd onto meas timestamps (ZOH). Pre-step times default to 0.
    cmd_at_meas = _zoh_resample(meas_ts, cmd_ts, cmd_arr, left=0.0)

    # Predict.
    y_pred = simulate_fopdt(
        meas_ts, cmd_at_meas, rise_params,
        fall_params=fall_params, initial=0.0, pre_cmd=0.0,
    )

    # ----- metrics per edge ------------------------------------------------
    rise_mask = (meas_ts >= step_t) & (meas_ts <= active_end_t)
    fall_mask = (meas_ts > active_end_t) & (meas_ts <= fall_end_t)
    overall_mask = (meas_ts >= step_t) & (meas_ts <= fall_end_t)

    # Per-channel noise floor (1σ) from Rung 1 processing — used by the
    # noise-floor-relative diagnostic. None when unavailable.
    from dimos.utils.characterization.modeling.per_run import (
        _noise_std_for_channel,
    )
    noise_std = _noise_std_for_channel(run.metadata.get("noise_floor"), channel)

    rise_metrics = _compute_metrics(
        y_meas[rise_mask], y_pred[rise_mask], amp=amplitude, noise_std=noise_std,
    )
    fall_metrics = (
        _compute_metrics(
            y_meas[fall_mask], y_pred[fall_mask], amp=amplitude, noise_std=noise_std,
        )
        if int(fall_mask.sum()) >= 4 else None
    )
    overall_metrics = _compute_metrics(
        y_meas[overall_mask], y_pred[overall_mask], amp=amplitude, noise_std=noise_std,
    )

    verdict = _verdict(rise_metrics, fall_metrics)

    return ValidationResult(
        run_id=run_id, run_dir=str(run_dir), recipe=recipe,
        channel=channel, amplitude=amplitude, direction=direction,
        mode=mode, split=split,
        used_K=rise_params.K, used_tau=rise_params.tau, used_L=rise_params.L,
        used_K_fall=(fall_params.K if fall_params else None),
        used_tau_fall=(fall_params.tau if fall_params else None),
        used_L_fall=(fall_params.L if fall_params else None),
        rise_metrics=rise_metrics, fall_metrics=fall_metrics,
        overall_metrics=overall_metrics,
        verdict=verdict, skip_reason=None,
        extra={
            "step_t": step_t,
            "active_end_t": active_end_t,
            "fall_end_t": fall_end_t,
            "baseline": baseline,
            "noise_std": (float(noise_std) if noise_std is not None else None),
            "n_meas_total": int(meas_ts.size),
            "thresholds": {
                "rise_pass": _RISE_PASS, "rise_marginal": _RISE_MARGINAL,
                "fall_pass": _FALL_PASS, "fall_marginal": _FALL_MARGINAL,
                "metric": "norm_rmse_smoothed",
                "smoothing_window": _RESID_SMOOTH_WINDOW,
                "smoothing_polyorder": _RESID_SMOOTH_POLY,
            },
        },
        _t_meas=(meas_ts.copy() if keep_traces else None),
        _y_meas=(y_meas.copy() if keep_traces else None),
        _y_pred=(y_pred.copy() if keep_traces else None),
        _cmd_at_meas=(cmd_at_meas.copy() if keep_traces else None),
    )


__all__ = ["ValidationResult", "validate_run"]
