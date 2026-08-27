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

"""The drive instrument on synthetic drives whose ground truth is known."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("scipy")

from dimos.robot.unitree.go2.sim.plant import TORQUE_LIMITS
from dimos.robot.unitree.go2.sim.sysid.drive import (
    DriveTF,
    amplitude_gain,
    demanded_torque,
    residual_shift_sweep,
)
from dimos.simulation.sysid.recording import Streams

FS = 500.0


def synthetic(ltau_from, seconds=40.0, kp=40.0, kd=1.0, seed=0):
    """A Streams whose demand is known and whose ltau is ltau_from(demand).

    Commands step at 50 Hz with broadband content; q/dq carry independent
    broadband content so the PD demand excites every band.
    """
    rng = np.random.default_rng(seed)
    lt = np.arange(0.0, seconds, 1.0 / FS)
    ct = np.arange(0.0, seconds, 0.02)
    n, m = len(lt), len(ct)

    def band(shape, scale):
        from scipy import signal

        x = rng.normal(0.0, scale, shape)
        fs = FS if shape[0] == n else 50.0
        sos = signal.butter(2, min(30.0, 0.4 * fs), "lowpass", fs=fs, output="sos")
        return signal.sosfiltfilt(sos, x, axis=0)

    st = Streams(
        lt=lt,
        lq=band((n, 12), 0.3),
        ldq=band((n, 12), 1.0),
        ltau=np.zeros((n, 12)),
        lquat=np.tile([1.0, 0, 0, 0], (n, 1)),
        lgyro=np.zeros((n, 3)),
        lacc=np.zeros((n, 3)),
        ct=ct,
        cq=band((m, 12), 0.3),
        ckp=np.full((m, 12), kp),
        ckd=np.full((m, 12), kd),
        ctau=np.zeros((m, 12)),
        cdq=np.zeros((m, 12)),
    )
    st.ltau = ltau_from(demanded_torque(st))
    return st


def test_demanded_torque_is_the_pd_law_clipped():
    st = synthetic(lambda d: d)
    idx = np.clip(np.searchsorted(st.ct, st.lt, "right") - 1, 0, len(st.ct) - 1)
    expect = np.clip(
        st.ckp[idx] * (st.cq[idx] - st.lq) + st.ckd[idx] * (st.cdq[idx] - st.ldq),
        -TORQUE_LIMITS,
        TORQUE_LIMITS,
    )
    np.testing.assert_allclose(demanded_torque(st), expect)


def test_ideal_drive_reads_flat_unity():
    st = synthetic(lambda d: d)
    tf = DriveTF.measure(st, t0=1.0, t1=39.0)
    m = (tf.f >= 1.0) & (tf.f <= 20.0)
    assert np.all(np.abs(np.abs(tf.H[m]) - 1.0) < 0.02)
    assert np.all(tf.coherence[m] > 0.98)
    fits = tf.model_fits()
    assert all(f.lag_s <= 0.001 and f.delay_s <= 0.001 for f in fits)


def test_first_order_drive_is_named_first_order():
    tau = 0.008

    def lagged(d):
        out = np.zeros_like(d)
        alpha = (1.0 / FS) / (tau + 1.0 / FS)
        for i in range(1, len(d)):
            out[i] = out[i - 1] + alpha * (d[i] - out[i - 1])
        return out

    st = synthetic(lagged)
    fits = DriveTF.measure(st, t0=1.0, t1=39.0).model_fits()
    for f in fits:
        assert abs(f.lag_s - tau) < 0.003, f
        # No resonance invented: the free 2nd-order fit must not claim an
        # in-band underdamped peak.
        assert not (f.fn_hz < 20.0 and f.zeta < 0.7), f


def test_shift_sweep_finds_an_injected_delay():
    k = int(0.010 * FS)  # 10 ms

    def delayed(d):
        out = np.zeros_like(d)
        out[k:] = d[:-k]
        return out

    st = synthetic(delayed)
    pts = residual_shift_sweep(st, t0=1.0, t1=39.0, shifts_ms=(0, 5, 10, 15, 20))
    assert min(pts, key=lambda p: p[1])[0] == 10.0


def test_deadband_shows_as_low_gain_at_small_demand_only():
    def deadband(d):
        return np.sign(d) * np.maximum(np.abs(d) - 1.5, 0.0)

    st = synthetic(deadband)
    cells = {(c.demand_lo, c.fast): c.gain for c in amplitude_gain(st, t0=1.0, t1=39.0)}
    assert cells[(0.5, False)] < 0.45
    assert cells[(8.0, False)] > 0.75


def test_healthy_drive_has_flat_amplitude_gain():
    st = synthetic(lambda d: d)
    for c in amplitude_gain(st, t0=1.0, t1=39.0):
        assert abs(c.gain - 1.0) < 0.02, c
