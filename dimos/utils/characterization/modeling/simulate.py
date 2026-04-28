# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""FOPDT forward simulator.

Predicts the output ``y(t)`` of a FOPDT plant driven by an arbitrary
commanded waveform ``cmd(t)``. The fitter (``fopdt.fit_fopdt``) goes the
other direction — data + cmd in, params out. The simulator goes
params + cmd in, predicted data out.

For Rung 2 Session 2 (validation), this is what lets us take params
fitted on forward-direction runs and predict the response on held-out
reverse-direction runs.

Discrete-time first-order with deadtime, integrated on the input
timestamps directly (no resampling — variable-step exact ZOH):

    cmd_delayed(t) = cmd(t - L)              (zero-order hold)
    target(t)     = K * cmd_delayed(t)
    y[k]          = alpha * y[k-1] + (1-alpha) * target[k-1]
    alpha         = exp(-dt / tau)

Using ``target[k-1]`` (the value held over the previous interval) rather
than ``target[k]`` is the exact ZOH discretization of a first-order ODE
under a piecewise-constant input — it matches the analytic step response
at sample times within numerical roundoff.

When ``fall_params`` is provided, ``tau`` (and ``K``, ``L``) switch based
on whether the system is accelerating (``|target| >= |y|``) or braking
(``|target| < |y|``). This mirrors how ``per_run.fit_run`` fits two
FOPDTs per run (one rise = accel, one fall = brake) — the simulator can
replay both edges with their own dynamics. The accel/brake formulation
(rather than ``target >= y``) is what makes this work for reverse-direction
steps where ``target`` is negative throughout.
"""

from __future__ import annotations

import numpy as np

from dimos.utils.characterization.modeling.fopdt import FopdtParams


def _zoh_at(t_query: np.ndarray, t_grid: np.ndarray, vals: np.ndarray,
            *, left: float) -> np.ndarray:
    """Zero-order hold sample of (t_grid, vals) at t_query.

    Times before t_grid[0] return ``left`` (the pre-trace value — usually
    the pre-step cmd, which is 0). Times within or beyond t_grid pick the
    most recent sample at or before t_query.
    """
    idx = np.searchsorted(t_grid, t_query, side="right") - 1
    out = np.empty_like(t_query, dtype=float)
    pre = idx < 0
    valid = ~pre
    out[pre] = left
    if valid.any():
        out[valid] = vals[idx[valid]]
    return out


def simulate_fopdt(
    t: np.ndarray,
    cmd: np.ndarray,
    params: FopdtParams,
    *,
    fall_params: FopdtParams | None = None,
    initial: float = 0.0,
    pre_cmd: float = 0.0,
) -> np.ndarray:
    """Simulate FOPDT response to a commanded waveform.

    Parameters
    ----------
    t : (N,) array
        Timestamps. Need not be uniform; the integrator uses each pair's
        ``dt = t[k] - t[k-1]`` directly.
    cmd : (N,) array
        Commanded value at each timestamp.
    params : FopdtParams
        Model parameters. Used unconditionally when ``fall_params`` is
        None; otherwise used when the system is "rising" (target > y).
    fall_params : FopdtParams, optional
        Parameters for the "falling" regime (target < y). When provided,
        the simulator switches per-step based on the sign of
        ``target - y``.
    initial : float, default 0.0
        Initial output ``y[0]``. Set to a non-zero pre-step baseline if
        you're predicting from a non-zero starting point.
    pre_cmd : float, default 0.0
        Cmd value to use for ``cmd(t - L)`` when ``t - L`` falls before
        ``t[0]``. Defaults to 0 (the system was at rest before the trace).

    Returns
    -------
    y : (N,) array
        Predicted output at each ``t[k]``.
    """
    t = np.asarray(t, dtype=float)
    cmd = np.asarray(cmd, dtype=float)
    if t.shape != cmd.shape:
        raise ValueError(f"t and cmd must have the same shape; got {t.shape} vs {cmd.shape}")
    if t.size == 0:
        return np.zeros(0, dtype=float)
    if t.size == 1:
        return np.full(1, initial, dtype=float)

    K_r, tau_r, L_r = float(params.K), float(params.tau), float(params.L)
    if fall_params is None:
        K_f, tau_f, L_f = K_r, tau_r, L_r
    else:
        K_f = float(fall_params.K)
        tau_f = float(fall_params.tau)
        L_f = float(fall_params.L)

    # Pre-compute delayed cmd for both regimes (cheap, vectorized) so the
    # inner loop just picks one.
    cmd_delayed_r = _zoh_at(t - L_r, t, cmd, left=pre_cmd)
    cmd_delayed_f = _zoh_at(t - L_f, t, cmd, left=pre_cmd)

    y = np.empty_like(t)
    y[0] = initial
    for k in range(1, t.size):
        dt = t[k] - t[k - 1]
        if dt <= 0:
            # Duplicate or out-of-order timestamps — keep previous y.
            y[k] = y[k - 1]
            continue
        # Decide regime from current state and the input held over
        # the previous interval (exact ZOH). Accel = rise_params,
        # brake = fall_params. Using |target| vs |y| (rather than
        # signed comparison) makes this correct for reverse steps too.
        target_r_prev = K_r * cmd_delayed_r[k - 1]
        accelerating = abs(target_r_prev) >= abs(y[k - 1])
        if accelerating or fall_params is None:
            target = target_r_prev
            tau_eff = tau_r
        else:
            target = K_f * cmd_delayed_f[k - 1]
            tau_eff = tau_f
        if tau_eff <= 0:
            y[k] = target
        else:
            alpha = float(np.exp(-dt / tau_eff))
            y[k] = alpha * y[k - 1] + (1.0 - alpha) * target
    return y


__all__ = ["simulate_fopdt"]
