# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Loop 2: floors, SNRs, and the closed-loop rollout against a recording."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.sysid.ground import (
    DIVERGENCE_FIT_S,
    DIVERGENCE_TERMS,
    ObsNoise,
    PolicyRun,
    Report,
    aggregate_divergence,
    tracking_curves,
    tracking_of,
    usable_floor,
    window_curves,
)
from dimos.robot.unitree.go2.sim.sysid.stats import Summary

FREEWALK = Path.home() / "recordings/hard_floor/20260816-194142_policy-freewalk-hard_vive.mcap"
FREEWALK_BIN = Path(__file__).parents[6] / "data/ml-trajectory-research/freewalk_mcf.bin"


def _summary(**over: float) -> Summary:
    base = dict.fromkeys(Summary.__dataclass_fields__, 0.0)
    del base["source"]  # provenance, not a statistic
    base.update(speed=0.5, speed_gain=0.9, gait_hz=1.7, height_mean=0.3)
    base.update(over)
    return Summary(**base)  # type: ignore[arg-type]


def test_usable_floor_lifts_a_collapsed_noise_floor():
    """A well-stabilised policy can drive a floor to ~0, sending its SNR to
    infinity and letting one term dominate — clamp it."""
    floor = usable_floor({"speed": 1e-9, "gait_hz": 0.5}, {"speed": 0.5, "gait_hz": 1.7})
    assert floor["speed"] == pytest.approx(0.025)  # 5% of the real value
    assert floor["gait_hz"] == 0.5  # an honest floor is untouched


def test_usable_floor_cross_clamps_against_another_recording():
    floor = usable_floor({"speed": 1e-9}, {"speed": 0.0}, {"speed": 0.03})
    assert floor["speed"] == 0.03


def test_snr_leaves_out_statistics_that_are_nan_on_either_side():
    """NaN means NOT COMPARABLE on this recording — neither scored nor counted."""
    nan = float("nan")
    rep = _report(
        _summary(speed=0.6),
        _summary(speed=nan, speed_gain=nan, speed_lag=nan, height_std=nan, gait_hz=nan),
        dict.fromkeys(Summary.__dataclass_fields__, 0.05),
    )
    snr = rep.snr()
    assert len(snr) == 7  # 11 comparable - 4 NaN pairs (gait_hz was never scored)
    assert "speed" not in snr and "roll_std" in snr
    assert np.isfinite(rep.loss())
    _n, of = rep.n_matched()
    assert of == 7


def test_obs_noise_scales_every_level_together():
    n = ObsNoise().scaled(2.0)
    assert (n.gyro, n.gravity, n.q, n.dq) == (0.4, 0.1, 0.02, 3.0)
    assert ObsNoise().scaled(0.0) == ObsNoise(0.0, 0.0, 0.0, 0.0)


def _report(sim: Summary, real: Summary, noise: dict[str, float]) -> Report:
    return Report(
        preset="test", sim=sim, real=real, noise=noise, floor_source="test", start=0.0, seconds=1.0
    )


def test_snr_is_the_difference_over_the_floor_and_height_mean_is_excluded():
    rep = _report(
        _summary(speed=0.6, height_mean=0.5),
        _summary(speed=0.5, height_mean=0.2),
        dict.fromkeys(Summary.__dataclass_fields__, 0.05),
    )
    snr = rep.snr()
    assert snr["speed"] == pytest.approx(2.0)
    assert "height_mean" not in snr  # room frame: no honest comparison exists
    assert "gait_hz" not in snr  # retired: it measured its own estimator (README 6)
    assert len(snr) == 11


def test_loss_is_rms_and_n_matched_counts_within_floor():
    noise = dict.fromkeys(Summary.__dataclass_fields__, 0.1)
    rep = _report(_summary(speed=0.7), _summary(speed=0.5), noise)  # speed SNR 2, rest 0
    assert rep.loss() == pytest.approx(np.sqrt(4.0 / 11.0))
    n, of = rep.n_matched()
    assert (n, of) == (10, 11)
    assert "10 of 11" in rep.table()
    # no replicates: the table refuses to let one draw look like a verdict
    assert "SINGLE ROLLOUT" in rep.table()


def test_a_replicated_report_is_the_median_with_the_draw_spread_beside_it():
    """The verdict is the per-statistic median over replicates (README 4a on
    loop 2); the spread of what single draws would have read prints with it,
    because a single chaotic rollout resamples across ~20% of the loss."""
    from dimos.robot.unitree.go2.sim.sysid.stats import median_summary

    noise = dict.fromkeys(Summary.__dataclass_fields__, 0.1)
    sims = [_summary(speed=0.5), _summary(speed=0.7), _summary(speed=0.9)]
    rep = Report(
        preset="test",
        sim=median_summary(sims),
        real=_summary(speed=0.5),
        noise=noise,
        floor_source="test",
        start=0.0,
        seconds=1.0,
        sims=sims,
    )
    assert rep.sim.speed == pytest.approx(0.7)  # the median draw, not any single one
    lo, hi = rep.loss_range()
    assert lo == pytest.approx(0.0)  # the speed=0.5 draw matches the real side exactly
    assert hi == pytest.approx(np.sqrt(16.0 / 11.0))  # the speed=0.9 draw, SNR 4
    klo, khi = rep.matched_range()
    assert (klo, khi) == (10, 11)
    assert "single draws read" in rep.table() and "SINGLE ROLLOUT" not in rep.table()


def test_median_summary_keeps_nan_not_comparable():
    from dimos.robot.unitree.go2.sim.sysid.stats import median_summary

    nan = float("nan")
    med = median_summary([_summary(speed=nan), _summary(speed=nan)])
    assert np.isnan(med.speed)  # a statistic no replicate measures stays out
    assert med.speed_gain == 0.9


# ----------------------------------------------------- the divergence rate


def _static_streams(seconds: float = 12.0) -> Streams:
    """A synthetic recording: robot parked at the origin, identity attitude.

    The tracker frame carries a 30-degree room yaw the IMU does not see, so
    the constant tracker-IMU yaw offset the window anchor must absorb is
    genuinely non-zero here.
    """
    from dimos.robot.unitree.go2.sim.sysid.ingest import TRACKER_Z, mount_matrix
    from dimos.simulation.sysid.recording import Streams
    from dimos.simulation.sysid.rotations import mat_to_quat

    lt = np.arange(0.0, seconds, 0.01)
    n = len(lt)
    ident = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
    yaw = np.pi / 6
    rz = np.array(
        [[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0.0, 0.0, 1.0]]
    )
    vq = np.tile(mat_to_quat(rz @ mount_matrix()), (n, 1))  # derived base attitude = Rz(30 deg)
    # vp such that the derived base position is exactly the origin
    vp = np.tile([0.0, 0.0, TRACKER_Z], (n, 1))
    return Streams(
        lt=lt,
        lq=np.zeros((n, 12)),
        ldq=np.zeros((n, 12)),
        ltau=np.zeros((n, 12)),
        lquat=ident,
        lgyro=np.zeros((n, 3)),
        lacc=np.tile([0.0, 0.0, 9.81], (n, 1)),
        ct=lt,
        cq=np.zeros((n, 12)),
        ckp=np.zeros((n, 12)),
        ckd=np.zeros((n, 12)),
        ctau=np.zeros((n, 12)),
        cdq=np.zeros((n, 12)),
        vt=lt.copy(),
        vp=vp,
        vq=vq,
        wt=lt.copy(),
        wcmd=np.zeros((n, 3)),
    )


def _drifting_run(v_err: float, window: float = 2.0, seconds: float = 10.0) -> PolicyRun:
    """A sim that drifts +x at ``v_err`` m/s within each re-init window."""
    t = np.arange(0.0, seconds, 0.02)
    reinit_t = np.arange(0.5, seconds - 1e-9, window)
    pos = np.zeros((len(t), 3))
    for t0 in reinit_t:
        m = t >= t0 - 1e-9
        tau = t[m] - t0
        pos[m, 0] = v_err * tau  # resets to 0 at each snap, then drifts
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1))
    return PolicyRun(
        t=t,
        pos=pos,
        quat=quat,
        cmd=np.zeros((len(t), 3)),
        target=np.zeros((len(t), 12)),
        q=np.zeros((len(t), 12)),
        reinit_t=reinit_t,
        reinit_pos=np.zeros((len(reinit_t), 3)),
        reinit_quat=np.tile([1.0, 0.0, 0.0, 0.0], (len(reinit_t), 1)),
    )


def test_the_divergence_rate_reads_a_known_drift():
    """End to end through the anchor math: a sim drifting 4 cm/s along the
    robot's heading must read an along rate of 4 cm/s, zero cross, zero
    attitude — including through the constant tracker-IMU yaw offset."""
    st = _static_streams()
    run = _drifting_run(v_err=0.04)
    curves = window_curves(run, st, start=0.0)
    assert set(curves) == set(DIVERGENCE_TERMS)
    div = aggregate_divergence(curves, 2.0)
    assert div.n_windows == 4  # 5 snaps in 10 s -> 4 closed windows
    along = div.terms["along"]
    assert along.rate == pytest.approx(0.04, abs=1e-6)
    assert along.bias_rate == pytest.approx(0.04, abs=1e-6)  # signed: ahead, not behind
    # a perfectly linear drift is interval-stable: all three rates agree
    assert along.rates == pytest.approx((0.04,) * len(DIVERGENCE_FIT_S), abs=1e-6)
    assert div.terms["cross"].rate == pytest.approx(0.0, abs=1e-9)
    for term in ("yaw_trk", "yaw_imu", "pitch_trk", "roll_imu"):
        assert div.terms[term].rate == pytest.approx(0.0, abs=1e-9)


def test_snapped_rates_are_a_diagnostic_and_never_enter_the_loss():
    st = _static_streams()
    div = aggregate_divergence(window_curves(_drifting_run(0.04), st, start=0.0), 2.0)
    noise = dict.fromkeys(Summary.__dataclass_fields__, 0.1)
    base = _report(_summary(), _summary(), noise)
    with_div = Report(
        preset="test",
        sim=_summary(),
        real=_summary(),
        noise=noise,
        floor_source="test",
        start=0.0,
        seconds=1.0,
        divergence=div,
    )
    assert not any(k.startswith("div_") for k in with_div.snr())
    assert with_div.loss() == base.loss()  # the diagnostic moves nothing
    text = with_div.table()
    assert "LOCAL RESPONSE (diagnostic, not scored)" in text
    assert "signed bias" in text  # reported with its own SE


def _free_drifting_run(v_err: float, seconds: float = 10.0) -> PolicyRun:
    """A free (never-snapped) sim drifting +x at ``v_err`` m/s."""
    t = np.arange(0.0, seconds, 0.02)
    pos = np.zeros((len(t), 3))
    pos[:, 0] = v_err * t
    return PolicyRun(
        t=t,
        pos=pos,
        quat=np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1)),
        cmd=np.zeros((len(t), 3)),
        target=np.zeros((len(t), 12)),
        q=np.zeros((len(t), 12)),
    )


def test_tracking_areas_score_the_free_rollouts():
    """The headline: mean |error| per component from the free rollouts,
    chaos floor from the pairwise sim-sim area, median + draws + SNR."""
    st = _static_streams()
    fast = tracking_curves(_free_drifting_run(0.04), st, start=0.0)
    still = tracking_curves(_free_drifting_run(0.0), st, start=0.0)
    trk = tracking_of([fast, still], 10.0)
    assert trk is not None
    # anchored at 0.5 s: |e|(t) = 0.04 (t - 0.5), mean over [0.5, 10)
    expect = 0.04 * float(np.mean(np.arange(0.5, 10.0, 0.02) - 0.5))
    lo, hi = trk.draws("along")
    assert lo == pytest.approx(0.0, abs=1e-9)
    assert hi == pytest.approx(expect, rel=1e-3)
    # the pairwise floor is the same separation / sqrt(2)
    assert trk.floors["along"] == pytest.approx(expect / np.sqrt(2), rel=1e-3)
    # identical attitude -> zero chaos floor, so the instrument floor clamps
    assert trk.floor_of("yaw") == pytest.approx(0.053)
    rep = Report(
        preset="t",
        sim=_summary(),
        real=_summary(),
        noise=dict.fromkeys(Summary.__dataclass_fields__, 0.1),
        floor_source="test",
        start=0.0,
        seconds=10.0,
        tracking=trk,
    )
    snr = rep.snr()
    assert snr["trk_along"] == pytest.approx((expect / 2) / (expect / np.sqrt(2)), rel=1e-3)
    assert snr["trk_yaw"] == pytest.approx(0.0)  # no error, honest instrument floor
    # pitch/roll are instrument-floored: reported in the table, never scored
    assert "trk_pitch" not in snr and "trk_roll" not in snr
    assert "reported, not scored" in rep.table()


# ------------------------------------------------- against the real recording


@pytest.mark.go2sim
@pytest.mark.skipif(not FREEWALK.is_file(), reason="needs the freewalk recording")
@pytest.mark.skipif(not FREEWALK_BIN.is_file(), reason="needs the freewalk blob")
def test_the_closed_loop_rollout_is_deterministic_and_summarisable():
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.policy import FreePolicy
    from dimos.robot.unitree.go2.sim.ranges import load_preset
    from dimos.robot.unitree.go2.sim.sysid.ground import rollout_policy, sim_summary
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    st = read_streams(FREEWALK)
    policy = FreePolicy.load(FREEWALK_BIN)
    preset = load_preset("measured")
    backend = MujocoBackend()
    a = rollout_policy(st, policy, preset, backend, start=6.0, seconds=4.0)
    b = rollout_policy(st, policy, preset, backend, start=6.0, seconds=4.0)
    assert np.array_equal(a.pos, b.pos) and np.array_equal(a.target, b.target)
    assert len(a.t) == pytest.approx(4.0 / 0.02, abs=2)
    # it walks rather than falls: the summary is computable and upright.
    # height_mean is the VIRTUAL TRACKER's height (base + 0.207 m lever arm)
    # — sensor space, matching the raw tracker height on the real side.
    s = sim_summary(a)
    assert 0.35 < s.height_mean < 0.65
    assert s.tilt_p99 < 1.0


@pytest.mark.go2sim
@pytest.mark.skipif(not FREEWALK.is_file(), reason="needs the freewalk recording")
@pytest.mark.skipif(not FREEWALK_BIN.is_file(), reason="needs the freewalk blob")
def test_loop_mechanisms_are_off_by_default_and_deterministic_when_on():
    """Latency and obs noise must change the physics when on, change NOTHING
    when off (every existing grounding number reproduces), and be exactly
    repeatable so a probe result is a measurement, not a draw."""
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.policy import FreePolicy
    from dimos.robot.unitree.go2.sim.ranges import load_preset
    from dimos.robot.unitree.go2.sim.sysid.ground import rollout_policy
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    st = read_streams(FREEWALK)
    policy = FreePolicy.load(FREEWALK_BIN)
    preset = load_preset("measured")
    be = MujocoBackend()
    kw: dict[str, float] = {"start": 6.0, "seconds": 3.0}
    base = rollout_policy(st, policy, preset, be, **kw)  # type: ignore[arg-type]
    explicit_off = rollout_policy(
        st,
        policy,
        preset,
        be,
        action_latency=0.0,
        obs_noise=None,
        **kw,  # type: ignore[arg-type]
    )
    assert np.array_equal(base.pos, explicit_off.pos)

    lat = rollout_policy(st, policy, preset, be, action_latency=0.01, **kw)  # type: ignore[arg-type]
    lat2 = rollout_policy(st, policy, preset, be, action_latency=0.01, **kw)  # type: ignore[arg-type]
    assert not np.array_equal(base.pos, lat.pos)
    assert np.array_equal(lat.pos, lat2.pos)

    nz = rollout_policy(st, policy, preset, be, obs_noise=ObsNoise(), **kw)  # type: ignore[arg-type]
    nz2 = rollout_policy(st, policy, preset, be, obs_noise=ObsNoise(), **kw)  # type: ignore[arg-type]
    assert not np.array_equal(base.pos, nz.pos)
    assert np.array_equal(nz.pos, nz2.pos)


@pytest.mark.go2sim
@pytest.mark.skipif(not FREEWALK.is_file(), reason="needs the freewalk recording")
@pytest.mark.skipif(not FREEWALK_BIN.is_file(), reason="needs the freewalk blob")
def test_a_perturbed_start_diverges_but_the_statistics_survive():
    """The whole premise of loop 2 in one assertion pair: position separates,
    the summary statistics stay within a sensible band."""
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.policy import FreePolicy
    from dimos.robot.unitree.go2.sim.ranges import load_preset
    from dimos.robot.unitree.go2.sim.sysid.ground import rollout_policy, sim_summary
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    st = read_streams(FREEWALK)
    policy = FreePolicy.load(FREEWALK_BIN)
    preset = load_preset("measured")
    backend = MujocoBackend()
    a = rollout_policy(st, policy, preset, backend, start=6.0, seconds=8.0)
    b = rollout_policy(
        st, policy, preset, backend, start=6.0, seconds=8.0, perturb=np.full(12, 0.02)
    )
    # chaos: the trajectories separate ...
    assert np.abs(a.pos - b.pos).max() > 0.01
    # ... the distributional statistics do not
    sa, sb = sim_summary(a), sim_summary(b)
    assert abs(sa.speed - sb.speed) < 0.15
    # stride_hz, not gait_hz: the retired estimator is bimodal under chaos
    # (locks onto a bob harmonic — README 6), which is why it was retired.
    # The stride instrument is the cadence claim, and it holds to ~0.015.
    assert abs(sa.stride_hz - sb.stride_hz) < 0.3


@pytest.mark.go2sim
@pytest.mark.skipif(not FREEWALK.is_file(), reason="needs the freewalk recording")
@pytest.mark.skipif(not FREEWALK_BIN.is_file(), reason="needs the freewalk blob")
def test_the_snapped_rollout_reinitialises_and_measures_a_finite_rate():
    """The closed loop's re-init path: snaps fire on schedule, the run is
    deterministic, and the divergence rates come out finite with the sim
    world's yaw/xy preserved across snaps (no teleports)."""
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.policy import FreePolicy
    from dimos.robot.unitree.go2.sim.ranges import load_preset
    from dimos.robot.unitree.go2.sim.sysid.ground import (
        DIVERGENCE_SCORED,
        reinit_schedule,
        rollout_policy,
    )
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams

    st = read_streams(FREEWALK)
    policy = FreePolicy.load(FREEWALK_BIN)
    preset = load_preset("measured")
    backend = MujocoBackend()
    states = reinit_schedule(st, start=6.0, seconds=8.0, T=2.0)
    kw: dict[str, float] = {"start": 6.0, "seconds": 8.0}
    a = rollout_policy(st, policy, preset, backend, reinit=states, **kw)  # type: ignore[arg-type]
    b = rollout_policy(st, policy, preset, backend, reinit=states, **kw)  # type: ignore[arg-type]
    assert len(a.reinit_t) == len(states) == 4  # 0.5, 2.5, 4.5, 6.5
    assert np.array_equal(a.pos, b.pos) and np.array_equal(a.reinit_pos, b.reinit_pos)
    # no teleport: each snap keeps the base within a step of where it was
    for k, t0 in enumerate(a.reinit_t):
        i_prev = int(np.searchsorted(a.t, t0 - 1e-9, "left")) - 1
        if i_prev >= 0:
            assert float(np.linalg.norm(a.reinit_pos[k][:2] - a.pos[i_prev][:2])) < 0.15
    curves = window_curves(a, st, start=6.0)
    div = aggregate_divergence(curves, 2.0)
    assert div.n_windows == 3
    for term in DIVERGENCE_SCORED.values():
        assert np.isfinite(div.terms[term].rate)
