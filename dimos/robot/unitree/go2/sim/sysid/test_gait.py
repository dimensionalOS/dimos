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

"""The stride instrument: FK against MuJoCo, and a synthetic gait it must read."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.sysid.gait import LEGS, Strides, foot_base, strides, touchdowns


def test_fk_matches_mujoco_exactly() -> None:
    """The hand FK is the model's own kinematics, not an approximation of it."""
    import mujoco

    from dimos.robot.unitree.go2.sim.engines.model import load

    model, data = load(None)
    rng = np.random.default_rng(7)
    q = rng.uniform(-0.9, 0.9, (5, 12))
    fk = foot_base(q)
    for n in range(len(q)):
        data.qpos[0:3] = 0
        data.qpos[3:7] = [1, 0, 0, 0]
        data.qpos[7:19] = q[n]
        mujoco.mj_forward(model, data)
        for i, leg in enumerate(LEGS):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, leg)
            assert np.linalg.norm(fk[n, i] - data.geom_xpos[gid]) < 1e-9


def _synthetic_gait(f_hz: float, speed: float, seconds: float = 20.0, rate: float = 500.0):
    """A crude trot: thigh/calf sinusoids at ``f_hz``, body advancing at ``speed``."""
    t = np.arange(0.0, seconds, 1.0 / rate)
    q = np.zeros((len(t), 12))
    for i, phase in enumerate((0.0, np.pi, np.pi, 0.0)):  # FL/FR/RL/RR diagonal pairs
        w = 2 * np.pi * f_hz * t + phase
        q[:, 3 * i + 1] = 0.9 + 0.25 * np.sin(w)
        q[:, 3 * i + 2] = -1.8 + 0.35 * np.clip(np.sin(w + np.pi / 2), 0, None)
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1))
    planar = np.stack([speed * t, np.zeros_like(t)], 1)
    return t, q, quat, planar


def test_reads_a_synthetic_gait() -> None:
    f, v = 1.8, 0.5
    t, q, quat, planar = _synthetic_gait(f, v)
    s = strides(t, q, quat, planar, np.ones(len(t), dtype=bool))
    assert isinstance(s, Strides)
    assert s.n_strides > 50
    assert s.stride_hz == pytest.approx(f, rel=0.03)
    assert s.stride_len == pytest.approx(v / f, rel=0.03)


def test_pauses_do_not_read_as_strides() -> None:
    """A stationary robot has no touchdown cycle and yields NaN, not garbage."""
    t = np.arange(0.0, 10.0, 0.002)
    q = np.tile(np.array([0.0, 0.9, -1.8] * 4), (len(t), 1))
    quat = np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1))
    planar = np.zeros((len(t), 2))
    s = strides(t, q, quat, planar, np.zeros(len(t), dtype=bool))
    assert np.isnan(s.stride_hz) and np.isnan(s.stride_len)
    assert s.n_strides == 0


def test_real_summary_scores_the_stride_pair() -> None:
    """The referee's cadence claim comes from the legs, end to end."""
    from dimos.robot.unitree.go2.sim.sysid.real import real_summary
    from dimos.simulation.sysid.recording import Streams

    f, v = 1.8, 0.5
    t, q, quat, planar = _synthetic_gait(f, v, seconds=30.0)
    n = len(t)
    vt, ct = t[::2], t[::10]
    m = len(ct)
    st = Streams(
        lt=t,
        lq=q,
        ldq=np.zeros((n, 12)),
        ltau=np.zeros((n, 12)),
        lquat=quat,
        lgyro=np.zeros((n, 3)),
        lacc=np.zeros((n, 3)),
        ct=ct,
        cq=np.zeros((m, 12)),
        ckp=np.zeros((m, 12)),
        ckd=np.zeros((m, 12)),
        ctau=np.zeros((m, 12)),
        cdq=np.zeros((m, 12)),
        vt=vt,
        vp=np.concatenate([planar[::2], np.full((len(vt), 1), 0.3)], axis=1),
        vq=np.tile([1.0, 0.0, 0.0, 0.0], (len(vt), 1)),
        wt=np.array([0.0]),
        wcmd=np.array([[v, 0.0, 0.0]]),
    )
    s = real_summary(st, start=1.0, seconds=25.0)
    assert s.stride_hz == pytest.approx(f, rel=0.05)
    assert np.isfinite(s.stride_len)
    assert "stride_hz" in s.as_dict()


def test_touchdowns_hysteresis() -> None:
    t = np.arange(0.0, 10.0, 0.002)
    h = -0.3 + 0.04 * np.sin(2 * np.pi * 2.0 * t)
    ev = touchdowns(t, h)
    assert len(ev) == pytest.approx(20, abs=1)
    assert np.allclose(np.diff(ev), 0.5, atol=0.02)
