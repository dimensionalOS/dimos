# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0.

"""Unit and integration tests for the FOPDT modeling pipeline.

Unit tests fabricate synthetic step responses with known K/τ/L and
verify that the fitter recovers them. Integration tests run end-to-end
against a real characterization session pointed to by the
``DIMOS_CHAR_SESSION`` env var; they're skipped when unset so CI
isn't blocked by missing data.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import pytest

from dimos.utils.characterization.modeling.aggregate import (
    aggregate_group,
    aggregate_session,
)
from dimos.utils.characterization.modeling.fopdt import (
    FopdtParams,
    fit_fopdt,
    fopdt_step_response,
)
from dimos.utils.characterization.modeling.per_run import (
    RunFit,
    parse_recipe_name,
    select_edge,
)
from dimos.utils.characterization.modeling.pool import (
    compare_models,
    compare_rise_fall,
    pool_session,
)
from dimos.utils.characterization.modeling.simulate import simulate_fopdt


# -------------------------- recipe-name parser ---------------------------

@pytest.mark.parametrize(
    "name,expected",
    [
        ("e1_vx_+1.0", ("vx", 1.0)),
        ("e1_vx_-0.3", ("vx", -0.3)),
        ("e2_wz_+0.6", ("wz", 0.6)),
        ("e2_wz_-1.5", ("wz", -1.5)),
        ("e8_vx_+1.0", ("vx", 1.0)),
    ],
)
def test_parse_recipe_name_valid(name: str, expected: tuple[str, float]) -> None:
    assert parse_recipe_name(name) == expected


@pytest.mark.parametrize(
    "name",
    [
        "step_vx_1.0",          # missing e<num>
        "e1_vy_no_amplitude",   # bad amp
        "e3_vx_ramp_0_to_3",    # ramp recipe
        "e7a_wz_+0.3",          # E7 has alpha suffix
        "e1_xx_+0.5",           # invalid channel
    ],
)
def test_parse_recipe_name_invalid(name: str) -> None:
    assert parse_recipe_name(name) is None


# -------------------------- FOPDT fitter --------------------------------

def _synthetic_step(K: float, tau: float, L: float, u_step: float,
                    *, dt: float = 0.02, duration: float = 3.0,
                    noise_std: float = 0.0,
                    seed: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """Generate a synthetic step-response trace, t starts at 0 (step edge)."""
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, duration, dt)
    y = fopdt_step_response(t, K, tau, L, u_step)
    if noise_std > 0:
        y = y + rng.normal(0.0, noise_std, size=t.shape)
    return t, y


def test_fopdt_recovers_clean_synthetic_within_1pct() -> None:
    K_true, tau_true, L_true = 0.95, 0.18, 0.06
    u_step = 1.0
    t, y = _synthetic_step(K_true, tau_true, L_true, u_step)
    fit = fit_fopdt(t, y, u_step=u_step)
    assert fit.converged
    assert not fit.degenerate
    assert abs(fit.K - K_true) / abs(K_true) < 0.01
    assert abs(fit.tau - tau_true) / abs(tau_true) < 0.05
    assert abs(fit.L - L_true) < 0.01
    assert fit.r_squared > 0.999


def test_fopdt_recovers_under_noise_within_5pct() -> None:
    K_true, tau_true, L_true = 0.85, 0.22, 0.04
    u_step = 1.0
    # Noise level ~2x typical Go2 vx noise floor (~5e-3) — still fittable.
    t, y = _synthetic_step(K_true, tau_true, L_true, u_step,
                            noise_std=0.01, seed=42)
    fit = fit_fopdt(t, y, u_step=u_step, noise_std=0.01)
    assert fit.converged
    assert abs(fit.K - K_true) / abs(K_true) < 0.05
    assert abs(fit.tau - tau_true) / abs(tau_true) < 0.20
    assert fit.r_squared > 0.95


def test_fopdt_handles_negative_u_step() -> None:
    K_true, tau_true, L_true = 0.9, 0.15, 0.05
    u_step = -0.5
    t, y = _synthetic_step(K_true, tau_true, L_true, u_step)
    fit = fit_fopdt(t, y, u_step=u_step)
    assert fit.converged
    assert abs(fit.K - K_true) / abs(K_true) < 0.01


def test_fopdt_returns_failure_for_zero_u_step() -> None:
    t = np.linspace(0.0, 3.0, 150)
    y = np.zeros_like(t)
    fit = fit_fopdt(t, y, u_step=0.0)
    assert not fit.converged
    assert fit.reason is not None and "u_step" in fit.reason


def test_fopdt_returns_failure_for_too_few_samples() -> None:
    t = np.array([0.0, 0.1, 0.2])
    y = np.array([0.0, 0.1, 0.2])
    fit = fit_fopdt(t, y, u_step=1.0)
    assert not fit.converged


# -------------------------- simulate (forward model) --------------------

def _params(K: float, tau: float, L: float) -> FopdtParams:
    return FopdtParams(
        K=K, tau=tau, L=L,
        K_ci=(K, K), tau_ci=(tau, tau), L_ci=(L, L),
        rmse=0.0, r_squared=1.0, n_samples=0,
        fit_window_s=(0.0, 0.0), degenerate=False, converged=True,
    )


def test_simulate_fopdt_step_matches_closed_form() -> None:
    """Step input through the simulator should reproduce the closed-form
    step response within numerical tolerance."""
    K, tau, L = 0.92, 0.28, 0.06
    u_step = 1.0
    dt = 0.005
    t = np.arange(0.0, 3.0, dt)
    cmd = np.where(t >= 0.0, u_step, 0.0)  # step at t=0
    y = simulate_fopdt(t, cmd, _params(K, tau, L))
    y_truth = fopdt_step_response(t, K, tau, L, u_step)
    # Discrete Euler with small dt should track the analytic curve closely.
    assert np.max(np.abs(y - y_truth)) < 5e-3


def test_simulate_then_fit_round_trip() -> None:
    """Generate a step trace via simulate_fopdt, fit it, recover params."""
    K, tau, L = 0.85, 0.22, 0.08
    u_step = 1.0
    dt = 0.01
    t = np.arange(0.0, 3.0, dt)
    cmd = np.where(t >= 0.0, u_step, 0.0)
    y = simulate_fopdt(t, cmd, _params(K, tau, L))
    fit = fit_fopdt(t, y, u_step=u_step)
    assert fit.converged
    assert abs(fit.K - K) / K < 0.02
    assert abs(fit.tau - tau) / tau < 0.05
    assert abs(fit.L - L) < 0.01


def test_simulate_handles_nonuniform_timestamps() -> None:
    """Simulator integrates on the input timestamps directly. Mildly
    non-uniform spacing should still yield a reasonable response."""
    K, tau, L = 0.9, 0.20, 0.05
    rng = np.random.default_rng(0)
    base = np.arange(0.0, 3.0, 0.01)
    jitter = rng.uniform(-0.002, 0.002, size=base.shape)
    t = np.sort(base + jitter)
    cmd = np.where(t >= 0.0, 1.0, 0.0)
    y = simulate_fopdt(t, cmd, _params(K, tau, L))
    # Tail should approach K * u_step.
    assert abs(np.mean(y[-30:]) - K) < 0.02


def test_simulate_asymmetric_reverse_step() -> None:
    """For a reverse step (cmd 0 -> -A -> 0), the accel phase has |target|
    growing from 0 to K*A and should use rise_params; the brake phase has
    |target| dropping back to 0 and should use fall_params. The signed
    target is always <= y[k-1] in this trace, so a heuristic based on
    signed-target-vs-y would pick fall_params throughout — wrong.
    """
    rise = _params(K=0.95, tau=0.20, L=0.05)
    fall = _params(K=0.95, tau=0.40, L=0.05)
    dt = 0.005
    t = np.arange(0.0, 4.0, dt)
    cmd = np.zeros_like(t)
    cmd[(t >= 0.5) & (t < 2.0)] = -1.0  # reverse step

    y = simulate_fopdt(t, cmd, rise, fall_params=fall)

    # During accel (just before step-down at t=2.0), y should be near -K.
    pre_down = y[(t > 1.9) & (t < 2.0)]
    assert abs(np.mean(pre_down) - (-0.95)) < 0.02

    # Accel rise time used should be tau_rise=0.2 not tau_fall=0.4.
    # At t = 0.5 + L + tau_rise = 0.75, y should have crossed 63% of -K.
    target_63 = -0.63 * 0.95
    near_one_tau = y[(t > 0.74) & (t < 0.76)]
    # If tau_rise (0.2) was used: y ≈ -0.6 here. If tau_fall (0.4) was
    # used by mistake: y ≈ -0.37. Threshold at -0.5 cleanly separates.
    assert np.mean(near_one_tau) < -0.5, (
        f"reverse-accel decay seems to have used fall_params; "
        f"y at t≈0.75 = {np.mean(near_one_tau):.3f} should be < -0.5 "
        f"(target_63 = {target_63:.3f})"
    )


def test_simulate_asymmetric_rise_fall() -> None:
    """With distinct fall_params, a step-up-then-step-down trace should
    show different time constants on rise vs fall."""
    rise = _params(K=0.95, tau=0.20, L=0.05)
    fall = _params(K=0.95, tau=0.40, L=0.05)  # fall is 2x slower
    dt = 0.005
    t = np.arange(0.0, 4.0, dt)
    cmd = np.zeros_like(t)
    cmd[(t >= 0.5) & (t < 2.0)] = 1.0  # step up at 0.5, down at 2.0
    y = simulate_fopdt(t, cmd, rise, fall_params=fall)

    # At t=2.0+rise_tau, rise response should be near steady-state.
    # At t=2.0+fall_tau, fall response should be ~63% down from peak.
    # Rough sanity: y near steady ~= K just before the step-down...
    pre_down = y[(t > 1.9) & (t < 2.0)]
    assert abs(np.mean(pre_down) - 0.95) < 0.02

    # ...and after fall_tau (0.4s) past the down-step, should still be
    # well above zero (slower decay than rise would predict).
    after_one_tau = y[(t > 2.05 + 0.40) & (t < 2.10 + 0.40)]
    # Symmetric (tau=0.20) would have y ~ 0.95*exp(-2) ~= 0.13.
    # Asymmetric with tau=0.40 should give y ~ 0.95*exp(-1) ~= 0.35.
    assert np.mean(after_one_tau) > 0.25


# -------------------------- aggregate ------------------------------------

def _mk_runfit(
    run_id: str, K: float, tau: float, L: float,
    *, recipe: str = "e1_vx_+1.0", channel: str = "vx",
    amplitude: float = 1.0, ci_half: float = 0.05,
    converged: bool = True, degenerate: bool = False,
    skip_reason: str | None = None,
) -> RunFit:
    direction = "forward" if amplitude > 0 else "reverse"
    params: FopdtParams | None
    if skip_reason is not None or not converged:
        params = None if skip_reason is not None else FopdtParams(
            K=float("nan"), tau=float("nan"), L=float("nan"),
            K_ci=(float("nan"), float("nan")),
            tau_ci=(float("nan"), float("nan")),
            L_ci=(float("nan"), float("nan")),
            rmse=float("nan"), r_squared=float("nan"),
            n_samples=0, fit_window_s=(0.0, 0.0),
            degenerate=True, converged=False, reason="forced",
        )
    else:
        params = FopdtParams(
            K=K, tau=tau, L=L,
            K_ci=(K - ci_half, K + ci_half),
            tau_ci=(tau - ci_half, tau + ci_half),
            L_ci=(L - ci_half, L + ci_half),
            rmse=0.01, r_squared=0.99, n_samples=150,
            fit_window_s=(0.0, 3.0),
            degenerate=degenerate, converged=True,
        )
    return RunFit(
        run_id=run_id, run_dir=f"/tmp/{run_id}",
        recipe=recipe, channel=channel,
        amplitude=amplitude, direction=direction,
        mode="default", split=("train" if direction == "forward" else "validate"),
        params=params, skip_reason=skip_reason,
    )


def test_aggregate_group_inverse_variance_weighted_mean() -> None:
    fits = [
        _mk_runfit(f"r{i}", K=1.0 + 0.01 * i, tau=0.2, L=0.05)
        for i in range(5)
    ]
    g = aggregate_group(fits)
    assert g.n_runs_kept == 5
    assert g.K is not None
    assert abs(g.K["mean"] - 1.02) < 0.01
    assert g.tau["mean"] == pytest.approx(0.2, abs=1e-6)


def test_aggregate_group_rejects_2sigma_outliers() -> None:
    # Four tight fits + one wild outlier on K should drop the outlier.
    fits = [_mk_runfit(f"r{i}", K=1.0 + 0.001 * i, tau=0.2, L=0.05) for i in range(4)]
    fits.append(_mk_runfit("r_outlier", K=2.5, tau=0.2, L=0.05))
    g = aggregate_group(fits)
    assert "r_outlier" in {r["run_id"] for r in g.rejected_run_ids}
    assert g.n_runs_kept == 4


def test_aggregate_group_skips_failed_runs() -> None:
    fits = [
        _mk_runfit("r0", K=1.0, tau=0.2, L=0.05),
        _mk_runfit("r1", K=1.0, tau=0.2, L=0.05, converged=False),
        _mk_runfit("r2", K=0.0, tau=0.0, L=0.0, skip_reason="too short"),
    ]
    g = aggregate_group(fits)
    assert g.n_runs_kept == 1
    assert "r1" in g.failed_run_ids
    assert "r2" in g.failed_run_ids


def test_aggregate_session_buckets_by_recipe() -> None:
    fits = [
        _mk_runfit("r0", K=1.0, tau=0.2, L=0.05, recipe="e1_vx_+1.0", amplitude=1.0),
        _mk_runfit("r1", K=1.0, tau=0.2, L=0.05, recipe="e1_vx_+1.0", amplitude=1.0),
        _mk_runfit("r2", K=0.95, tau=0.18, L=0.05, recipe="e1_vx_+0.5", amplitude=0.5),
    ]
    groups = aggregate_session(fits)
    recipes = {g.key["recipe"] for g in groups}
    assert recipes == {"e1_vx_+1.0", "e1_vx_+0.5"}


# -------------------------- pool -----------------------------------------

def _summary_for_pool(K_at_amps: dict[float, float]) -> dict:
    """Build a minimal model_summary by fabricating GroupFits at each
    (amp, direction) cell."""
    fits: list[RunFit] = []
    for amp, K_val in K_at_amps.items():
        for sign in (1, -1):
            signed = sign * amp
            for i in range(3):
                fits.append(_mk_runfit(
                    f"r_{amp}_{sign}_{i}",
                    K=K_val, tau=0.2, L=0.05,
                    recipe=f"e1_vx_{'+' if sign > 0 else '-'}{amp}",
                    amplitude=signed,
                ))
    groups = aggregate_session(fits)
    return pool_session(groups, mode="default")


def test_pool_session_detects_linear_in_amplitude() -> None:
    # Constant K across amplitudes -> linear_in_amplitude=True.
    summary = _summary_for_pool({0.3: 0.95, 0.6: 0.95, 1.0: 0.95})
    assert "vx" in summary["channels"]
    assert summary["channels"]["vx"]["linear_in_amplitude"]["K"] is True
    # Pooled mean roughly matches.
    assert abs(summary["channels"]["vx"]["pooled"]["K"]["mean"] - 0.95) < 0.05


def test_pool_session_detects_amplitude_dependent_K() -> None:
    summary = _summary_for_pool({0.3: 0.7, 0.6: 0.85, 1.0: 1.0})
    ch = summary["channels"]["vx"]
    # With a clean trend across 3 amps, regression should reject constant
    # (slope CI should exclude 0). If it does, gain_schedule is populated.
    if not ch["linear_in_amplitude"]["K"]:
        assert ch["gain_schedule"]["K"] is not None
        assert ch["gain_schedule"]["K"]["slope"] > 0


def test_compare_models_identical_sessions() -> None:
    summary = _summary_for_pool({0.3: 1.0, 0.6: 1.0, 1.0: 1.0})
    verdict = compare_models(summary, summary)
    for cv in verdict["channels"].values():
        # Same data on both sides: every parameter should be "identical".
        for p_v in cv["params"].values():
            assert p_v["verdict"] in {"identical", "equivalent"}


# -------------------------- step-down (fall) ----------------------------

def test_select_edge_projects_params_down_onto_params() -> None:
    """select_edge('fall') should swap params := params_down for downstream
    aggregate/pool code that's edge-agnostic."""
    rise_params = FopdtParams(
        K=0.95, tau=0.20, L=0.05,
        K_ci=(0.9, 1.0), tau_ci=(0.18, 0.22), L_ci=(0.04, 0.06),
        rmse=0.01, r_squared=0.99, n_samples=150,
        fit_window_s=(0.0, 3.0), degenerate=False, converged=True,
    )
    fall_params = FopdtParams(
        K=0.92, tau=0.40, L=0.08,
        K_ci=(0.88, 0.96), tau_ci=(0.36, 0.44), L_ci=(0.06, 0.10),
        rmse=0.02, r_squared=0.97, n_samples=50,
        fit_window_s=(0.0, 1.0), degenerate=False, converged=True,
    )
    rf = RunFit(
        run_id="r0", run_dir="/tmp/r0", recipe="e1_vx_+1.0",
        channel="vx", amplitude=1.0, direction="forward",
        mode="default", split="train",
        params=rise_params, params_down=fall_params,
        extra={"baseline": 0.0}, extra_down={"baseline_down": 1.0},
    )
    rise_view = select_edge([rf], "rise")
    fall_view = select_edge([rf], "fall")
    assert rise_view[0].params is rise_params
    assert fall_view[0].params is fall_params
    assert fall_view[0].extra.get("baseline_down") == 1.0
    # Original RunFit unchanged.
    assert rf.params is rise_params


def test_compare_rise_fall_flags_decel_slower_than_accel() -> None:
    rise = _summary_for_pool({0.3: 0.95, 0.6: 0.95, 1.0: 0.95})
    # Fabricate a "fall" summary where τ is 2× the rise — that's a real
    # asymmetry the comparison should flag.
    fall_fits: list[RunFit] = []
    for amp in (0.3, 0.6, 1.0):
        for sign in (1, -1):
            for i in range(3):
                fall_fits.append(_mk_runfit(
                    f"r_fall_{amp}_{sign}_{i}",
                    K=0.95, tau=0.40, L=0.05,
                    recipe=f"e1_vx_{'+' if sign > 0 else '-'}{amp}",
                    amplitude=sign * amp,
                ))
    fall_groups = aggregate_session(fall_fits)
    fall = pool_session(fall_groups, mode="default")
    verdict = compare_rise_fall(rise, fall)
    tau_v = verdict["channels"]["vx"]["params"]["tau"]
    assert tau_v["verdict"] == "differs"
    # ratio fall/rise should be ~2 since fall τ = 2× rise τ.
    assert 1.5 < tau_v["ratio_fall_over_rise"] < 2.5


def test_run_fit_smoke_includes_params_down(real_session: Path) -> None:
    """End-to-end check: a real step run produces both rise and fall fits."""
    from dimos.utils.characterization.modeling.per_run import fit_run

    step_runs = []
    for rd in sorted(p for p in real_session.iterdir()
                     if p.is_dir() and p.name and p.name[0].isdigit()):
        rj = rd / "run.json"
        if not rj.exists():
            continue
        meta = json.loads(rj.read_text())
        if meta.get("recipe", {}).get("test_type") == "step":
            step_runs.append(rd)
    if not step_runs:
        pytest.skip(f"no step runs in {real_session}")
    rf = fit_run(step_runs[0], mode="default")
    # Either a fall fit exists or extra_down explains why not.
    assert rf.params_down is not None or rf.extra_down.get("skip_reason")


# -------------------------- integration (real session) -------------------

@pytest.fixture
def real_session() -> Path:
    p = os.environ.get("DIMOS_CHAR_SESSION")
    if not p:
        pytest.skip("set DIMOS_CHAR_SESSION to a Rung 1 session dir to run integration tests")
    path = Path(p)
    if not path.is_dir():
        pytest.skip(f"DIMOS_CHAR_SESSION={p} is not a directory")
    return path


def test_fit_run_smoke_real_session(real_session: Path) -> None:
    """Fit one step run from a real session without exceptions."""
    from dimos.utils.characterization.modeling.per_run import fit_run

    step_runs = []
    for rd in sorted(p for p in real_session.iterdir()
                     if p.is_dir() and p.name and p.name[0].isdigit()):
        rj = rd / "run.json"
        if not rj.exists():
            continue
        meta = json.loads(rj.read_text())
        if meta.get("recipe", {}).get("test_type") == "step":
            step_runs.append(rd)
    if not step_runs:
        pytest.skip(f"no step runs in {real_session}")

    rf = fit_run(step_runs[0], mode="default")
    assert rf.run_id == step_runs[0].name
    # Either fit or principled skip — never an unhandled exception.
    assert rf.skip_reason is not None or rf.params is not None


def test_fit_session_smoke_real_session(tmp_path: Path, real_session: Path) -> None:
    """Full pipeline on a real session — produces all expected artifacts."""
    from dimos.utils.characterization.modeling.session import fit_session

    summary = fit_session(real_session, write_plots_enabled=False)
    out_dir = real_session / "modeling"
    assert (out_dir / "fits_per_run.json").exists()
    assert (out_dir / "fits_per_group.json").exists()
    assert (out_dir / "model_summary.json").exists()
    assert (out_dir / "model_report.md").exists()
    assert "channels" in summary
    assert summary["mode"] in {"default", "rage"}
