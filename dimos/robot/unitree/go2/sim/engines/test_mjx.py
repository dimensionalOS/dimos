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

"""The masking must stay free: MJX may accept the plant, but not by changing it."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from dimos.robot.unitree.go2.sim.backend import BaseCondition, Commands, RolloutPlan, State
from dimos.robot.unitree.go2.sim.engines import mjx as go2_mjx, model as go2_model
from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend
from dimos.robot.unitree.go2.sim.plant import TORQUE_ENVELOPES
from dimos.robot.unitree.go2.sim.ranges import MEASURED

pytestmark = [pytest.mark.go2sim]

COLLAPSE_STEPS = 2000
DROP_HEIGHT = 0.30  # stand height: it lands, folds, and rests on its own legs


def _measured() -> tuple[mujoco.MjModel, mujoco.MjData]:
    model, data = go2_model.load()
    go2_model.apply_physics(model, MEASURED.physics)
    return model, data


def _collapse(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    mujoco.mj_resetData(model, data)
    data.qpos[2] = DROP_HEIGHT
    data.ctrl[:] = 0.0
    return go2_mjx.qpos_after(model, data, COLLAPSE_STEPS)


def test_the_shipped_scene_is_not_mjx_acceptable_as_built():
    """The reason this module exists. If menagerie ever ships a go2 without
    cylinder-vs-box pairs, `prepare` becomes a no-op and this test says so."""
    model, _ = _measured()
    bad = go2_mjx.unsupported_pairs(model)
    assert bad, "no unsupported pairs left — prepare() may no longer be needed"
    types = {tuple(sorted(int(t) for t in model.geom_type[[a, b]])) for a, b in bad}
    assert types == {tuple(sorted(int(t) for t in go2_mjx.UNSUPPORTED_PAIR))}, (
        f"a NEW kind of unsupported pair appeared: {sorted(types)}. Masking "
        "self-collision is licensed for cylinder-vs-box only."
    )


def test_preparing_the_plant_leaves_no_unsupported_pair():
    prepared = go2_mjx.load(MEASURED.physics)
    assert go2_mjx.unsupported_pairs(prepared.model) == []
    assert prepared.masked_geoms, "nothing was masked, yet the scene needed it"


def test_masking_keeps_the_floor_and_only_drops_robot_against_robot():
    prepared = go2_mjx.load(MEASURED.physics)
    m = prepared.model
    floor = [g for g in range(m.ngeom) if m.geom_bodyid[g] == go2_mjx.WORLD_BODY]
    assert floor, "the scene must still own a world geom to stand on"
    assert all(m.geom_conaffinity[g] for g in floor), "the floor lost its affinity"
    shipped, _ = _measured()
    assert all(m.geom_contype[g] == shipped.geom_contype[g] for g in range(m.ngeom)), (
        "contype must survive untouched: it is what still pairs the robot with the floor"
    )
    assert all(shipped.geom_conaffinity[g] for g in prepared.masked_geoms), (
        "masked_geoms must report what actually changed, not every robot geom"
    )


def test_the_masked_plant_moves_IDENTICALLY_through_a_collapse():
    """The measurement the masking rests on, as a lock.

    A drop from stand height folds the robot onto its own legs — the motion
    most likely to need self-collision. Bit-identical means the shipped plant
    never used it, so MJX is not being handed a different robot."""
    shipped = _collapse(*_measured())
    masked_model, masked_data = _measured()
    go2_mjx.prepare(masked_model)
    masked = _collapse(masked_model, masked_data)
    assert np.array_equal(shipped, masked), (
        f"masking self-collision moved the plant (max |dqpos| "
        f"{np.abs(shipped - masked).max():.3e}) — it is no longer free, and the "
        "MJX model is no longer the plant this package fitted"
    )


def test_a_collapse_generates_no_self_contact_to_begin_with():
    """The same fact from the contact side, and the instrument a training env
    is told to re-run when it starts rewarding falls."""
    model, data = _measured()
    mujoco.mj_resetData(model, data)
    data.qpos[2] = DROP_HEIGHT
    data.ctrl[:] = 0.0
    seen = 0
    for _ in range(COLLAPSE_STEPS):
        mujoco.mj_step(model, data)
        seen += go2_mjx.self_contacts(model, data)
    assert seen == 0, f"{seen} self-contacts in a collapse: masking is NOT free here"


def test_mjx_accepts_the_prepared_plant():
    pytest.importorskip("jax")
    prepared = go2_mjx.load(MEASURED.physics)
    assert go2_mjx.put_model(prepared) is not None


STAND_Q = np.tile([0.0, 0.9, -1.8], 4)
PARITY_SECONDS = 0.2


def _plan(duration: float = PARITY_SECONDS, base: BaseCondition = BaseCondition.FREE):
    return RolloutPlan(
        t0=0.0,
        duration=duration,
        commands=Commands(
            t=np.array([0.0]),
            q=STAND_Q[None],
            dq=np.zeros((1, 12)),
            kp=np.full((1, 12), 40.0),
            kd=np.full((1, 12), 2.0),
            tau_ff=np.zeros((1, 12)),
        ),
        reinit=[State(t=0.0, q=STAND_Q.copy(), dq=np.zeros(12), rot=np.eye(3), gyro=np.zeros(3))],
        base=base,
    )


def _configured(backend):
    backend.apply({**MEASURED.physics, "actuator_tau": MEASURED.actuator_tau})
    return backend


def test_the_envelope_ports_to_jax_exactly():
    """The one piece of the actuator chain rewritten rather than reused: hold
    it to the original on the speeds that matter, both quadrants."""
    jnp = pytest.importorskip("jax.numpy")
    go2_mjx.enable_x64()  # the precision the backend runs in; float32 costs ~7e-6 N.m
    envelope = TORQUE_ENVELOPES[MEASURED.envelope]
    rng = np.random.default_rng(0)
    dq = rng.uniform(-20.0, 20.0, size=(64, 12))
    requested = rng.uniform(-40.0, 40.0, size=(64, 12))
    want = np.array([envelope.deliverable(r, v) for r, v in zip(requested, dq, strict=True)])
    got = np.asarray(go2_mjx.deliverable_jax(envelope, jnp.asarray(requested), jnp.asarray(dq)))
    assert np.allclose(want, got, rtol=0, atol=1e-12), np.abs(want - got).max()


def test_a_pinned_plan_is_refused_rather_than_faked():
    plan = _plan(base=BaseCondition.PINNED)
    with pytest.raises(NotImplementedError, match="FREE only"):
        go2_mjx.MjxBackend().rollout(plan)


def test_the_declared_channels_exclude_the_virtual_imu():
    """`accel` is absent BY DECLARATION, which is how the seam says so — and
    the fit is unaffected because score.DEFAULT_WEIGHTS puts accel at zero."""
    channels = go2_mjx.MjxBackend().channels()
    assert "accel" not in channels
    assert {"joint", "dq", "tau", "gyro", "pos", "rot"} <= channels


@pytest.mark.parametrize("envelope_name", [None, MEASURED.envelope])
def test_mjx_reproduces_the_cpu_plant_on_the_same_plan(envelope_name):
    """THE cross-engine check: one plan, two engines, the same measured plant.

    Agreement is to float64 accumulation noise, not to a tolerance chosen to
    pass — a real solver disagreement lands orders of magnitude above this."""
    pytest.importorskip("jax")
    envelope = TORQUE_ENVELOPES[envelope_name] if envelope_name else None
    plan = _plan()
    cpu = _configured(MujocoBackend(envelope=envelope)).rollout(plan)
    gpu = _configured(go2_mjx.MjxBackend(envelope=envelope)).rollout(plan)

    assert gpu.q.shape == cpu.q.shape and gpu.tau.shape == cpu.tau.shape
    for field, tol in (("q", 1e-8), ("dq", 1e-6), ("body_pos", 1e-8), ("tau", 1e-5)):
        a, b = getattr(cpu, field), getattr(gpu, field)
        worst = np.abs(a - b).max()
        assert worst < tol, f"{field}: engines disagree by {worst:.3e} (> {tol:.0e})"


def test_mjx_hosts_the_closed_loop_and_snaps_where_the_cpu_does():
    """Mode B under MJX exists so the referee can grade the plant that
    TRAINS: at the fast preset's 1/5 solver the engines truncate differently,
    so a CPU verdict only BOUNDS the MJX plant. The session's contract is the
    CPU session's — this holds the piece with its own arithmetic (snap), which
    must place the robot identically, since it is measured state and forward
    kinematics with no engine in it."""
    pytest.importorskip("jax")
    from dimos.robot.unitree.go2.sim.ranges import FAST

    state = State(
        t=0.0,
        q=STAND_Q.copy(),
        dq=np.zeros(12),
        rot=np.eye(3),
        gyro=np.zeros(3),
    )
    cpu = _configured(MujocoBackend()).session(STAND_Q)
    gpu = _configured(go2_mjx.MjxBackend()).session(STAND_Q)
    try:
        cpu.snap(state)
        gpu.snap(state)
        a, b = cpu.state(), gpu.state()
        assert np.allclose(a.pos, b.pos, atol=1e-12), (a.pos, b.pos)
        assert np.allclose(a.quat, b.quat, atol=1e-12)
        assert np.allclose(a.q, b.q, atol=1e-12)
        assert gpu.timestep == cpu.timestep
        # and it steps: torques in, state out, no exception
        assert gpu.step(np.zeros(12)) is True
    finally:
        cpu.close()
        gpu.close()


def test_the_mjx_session_refuses_a_viewer_rather_than_silently_dropping_it():
    """A viewer that silently does nothing is how someone ends up believing
    they watched the plant they graded."""
    pytest.importorskip("jax")
    with pytest.raises(NotImplementedError, match="no viewer"):
        go2_mjx.MjxBackend().session(STAND_Q, view=True)
    with pytest.raises(NotImplementedError, match="ghost"):
        go2_mjx.MjxBackend().session(STAND_Q, ghost=True)
