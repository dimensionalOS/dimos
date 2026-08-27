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

"""ACCEPTANCE: serial and parallel are BIT-IDENTICAL.

Parallelism that changes answers is a bug that looks like a speedup. Both
paths run the same pure function; these tests prove the plumbing preserves
that, on the real recording, to full float precision.

Needs the recordings under ``~/recordings`` and a menagerie checkout, so these
run locally, not in CI.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("mujoco")
pytest.importorskip("mcap")


# The go2 assets are vendored (data/go2_menagerie): a missing scene is a real
# failure now, never a skip. Only the recording skip below remains.
pytestmark = [pytest.mark.go2sim]

HARD = Path.home() / "recordings/hard_floor/20260816-194142_policy-freewalk-hard_vive.mcap"
needs_hard = pytest.mark.skipif(not HARD.is_file(), reason=f"no recording at {HARD}")


def _equal_results(a, b) -> None:
    for name in ("t", "q", "dq", "body_pos", "body_rot", "at", "imu_accel", "imu_gyro", "tau"):
        assert np.array_equal(getattr(a.prediction, name), getattr(b.prediction, name)), name
    assert np.array_equal(a.q_real, b.q_real)
    assert np.array_equal(a.a_real, b.a_real)
    assert np.array_equal(a.dq_real, b.dq_real)
    assert (a.p_real is None) == (b.p_real is None)
    if a.p_real is not None:
        assert np.array_equal(a.p_real, b.p_real)


@needs_hard
def test_worker_processes_reproduce_the_serial_rollouts_bit_for_bit():
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.ranges import MEASURED
    from dimos.robot.unitree.go2.sim.sysid.rollouts import Rollouts, RolloutSpec

    values = {**MEASURED.physics, "actuator_tau": MEASURED.actuator_tau}

    def make_specs(rollouts):
        s = rollouts.streams
        t0 = max(float(s.lt[0]), float(s.ct[0])) + 5.0
        return [
            RolloutSpec(values=values, t0=t0 + 8.0 * i, duration=2.0, window=0.4, seed=i)
            for i in range(3)
        ]

    with Rollouts(HARD, MujocoBackend(), workers=1) as serial:
        specs = make_specs(serial)
        got_serial = serial.run(specs)
    with Rollouts(HARD, MujocoBackend(), workers=3) as parallel:
        got_parallel = parallel.run(specs)
    assert len(got_serial) == len(got_parallel) == 3
    for a, b in zip(got_serial, got_parallel, strict=True):
        _equal_results(a, b)


@needs_hard
def test_the_parallel_jacobian_is_the_serial_jacobian():
    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
    from dimos.robot.unitree.go2.sim.ranges import MEASURED
    from dimos.robot.unitree.go2.sim.sysid.identify import jacobian
    from dimos.robot.unitree.go2.sim.sysid.rollouts import Rollouts
    from dimos.simulation.sysid.regimes import sample_segments

    values = {**MEASURED.physics, "actuator_tau": MEASURED.actuator_tau}
    with Rollouts(HARD, MujocoBackend(), workers=3) as rollouts:
        st = rollouts.streams
        t_lo = max(float(st.lt[0]), float(st.ct[0]))
        segs = sample_segments(t_lo, t_lo + 12.0, n=2, length=(1.5, 2.5), seed=0)
        kw = dict(frac=0.05, window=0.4, seed=0, params=("armature", "actuator_tau"))
        a = jacobian(st, segs, MujocoBackend(), values, channel="accel", **kw)
        b = jacobian(st, segs, MujocoBackend(), values, channel="accel", rollouts=rollouts, **kw)
    assert np.array_equal(a.J, b.J)
    assert np.array_equal(a.residual, b.residual)


@needs_hard
def test_a_parallel_fit_is_the_serial_fit_bit_for_bit():
    """The whole stack: calibration, every trial, the harvest and the median
    agree exactly between one process and a worker pool."""
    pytest.importorskip("optuna")
    from dimos.robot.unitree.go2.sim.ranges import KNOBS
    from dimos.robot.unitree.go2.sim.sysid.fit import Objective, base_values, default_plan, fit
    from dimos.robot.unitree.go2.sim.sysid.rollouts import Rollouts
    from dimos.simulation.sysid.recording import read_declarations
    from dimos.simulation.sysid.regimes import regimes, sample_segments

    plan = default_plan(KNOBS, search=("armature", "actuator_tau"))
    base = base_values("measured")

    from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend

    def run_fit(workers: int):
        with Rollouts(HARD, MujocoBackend(), workers=workers) as rollouts:
            st = rollouts.streams
            spans = regimes(st, read_declarations(HARD))
            t_lo = max(float(st.lt[0]), float(st.ct[0]))
            segments = sample_segments(t_lo, t_lo + 20.0, n=2, length=(1.5, 2.5), seed=0)
            objective = Objective(
                rollouts,
                segments=segments,
                spans=spans,
                weights={("accel", "floor"): 1.0},
                backend_channels=frozenset({"accel"}),
                window=(0.05, 0.8),
                schedule_seed=0,
            )
            res = fit(objective, plan, base, trials=6, min_studies=1, max_studies=1, batch=1)
            return res, dict(objective.scales or {})

    res1, scales1 = run_fit(1)
    res3, scales3 = run_fit(3)
    assert scales1 == scales3  # calibration parity, to full precision
    assert res1.baseline.total == res3.baseline.total
    assert res1.point == res3.point
    assert res1.studies[0].best_total == res3.studies[0].best_total
    assert res1.studies[0].best_params == res3.studies[0].best_params
    for k in res1.cloud:
        assert np.array_equal(res1.cloud[k], res3.cloud[k])
