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

"""The batched engine: the measured plant, made acceptable to MJX.

MJX compiles the same :class:`mujoco.MjModel` that
:mod:`~dimos.robot.unitree.go2.sim.engines.model` builds, so the fitted knobs
carry over untouched — but it implements a subset of MuJoCo's collision
functions, and the go2's ``(CYLINDER, BOX)`` pairs are outside it. That is the
whole gap: every other type pair the scene needs (plane against sphere,
capsule, cylinder and box) has an implementation.

:func:`prepare` closes it, and the choice was measured rather than argued.
Two candidates:

* drop the robot's self-collision (mask ``conaffinity`` on every geom the
  robot owns, leaving the world's floor at its default so foot contact is
  untouched);
* convert the 13 cylinders to capsules, shortening each half-length by its
  radius to hold the tip-to-tip extent.

Graded on the CPU plant — 2000 steps of a drop-and-collapse from stand height,
a motion that folds the robot onto itself — masking is **bit-identical** to the
shipped model (max |dqpos| 0.0) while capsules diverge by 5.3e-2 rad: the caps
manufacture contacts the cylinders never made. So masking it is. Independent
evidence from the other end: a 40 s closed-loop freewalk in the measured plant
(20000 physics steps) generates 62938 contacts, every one of them a foot
against the floor, and ZERO on any of the 48 unsupported pairs.

What that licenses, precisely: the shipped plant does not USE robot
self-collision in any regime this package has measured — walking, and folding
up under gravity. It is not a proof that no configuration ever would. A
training env that rewards falling, crossing legs, or climbing over itself is
outside that evidence and must re-measure (:func:`self_contacts` counts it).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import mujoco
import numpy as np

from dimos.robot.unitree.go2.sim.engines import model as go2_model
from dimos.robot.unitree.go2.sim.plant import TORQUE_LIMITS
from dimos.robot.unitree.go2.sim.ranges import KNOBS, PHYSICS_KEYS, Knob
from dimos.simulation.sysid.backend import (
    CHANNELS,
    BaseCondition,
    LoopState,
    Prediction,
    RolloutPlan,
    State,
)
from dimos.simulation.sysid.plant import TorqueEnvelope
from dimos.simulation.sysid.rotations import mat_to_quat, quat_to_mat

if TYPE_CHECKING:
    from collections.abc import Mapping
    from pathlib import Path

# The one type pair the go2 scene needs and MJX does not implement. Named, not
# discovered, so a NEW unsupported pair is a loud failure in `prepare` rather
# than something silently masked away with the rest.
UNSUPPORTED_PAIR = (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_BOX)

WORLD_BODY = 0


@dataclass(frozen=True)
class Prepared:
    """An MJX-acceptable model, and what had to change to make it one."""

    model: mujoco.MjModel
    masked_geoms: tuple[int, ...]
    """Geoms whose ``conaffinity`` was cleared — the robot's, never the floor's."""


def unsupported_pairs(model: mujoco.MjModel) -> list[tuple[int, int]]:
    """Geom pairs MJX has no collision function for, after its own filtering."""
    # Private API, deliberately: mjx exposes no public list of what it can
    # collide, and asking `put_model` means catching an exception instead of
    # enumerating. test_mjx pins the answer, so a move here fails loudly.
    from mujoco.mjx._src import collision_driver  # type: ignore[import-untyped]

    return [
        (int(g1), int(g2))
        for g1, g2, _ip in collision_driver.geom_pairs(model)
        if not collision_driver.has_collision_fn(*model.geom_type[[g1, g2]])
    ]


def prepare(model: mujoco.MjModel) -> Prepared:
    """Mask the robot's self-collision so MJX accepts the plant.

    Contact against the world (the floor, and anything else the world owns) is
    untouched: MuJoCo pairs geoms when ``contype & conaffinity`` is non-zero in
    EITHER direction, so clearing the robot's ``conaffinity`` while the floor
    keeps its own leaves every floor contact intact and removes only
    robot-against-robot.
    """
    masked = tuple(
        g
        for g in range(model.ngeom)
        if model.geom_bodyid[g] != WORLD_BODY and model.geom_conaffinity[g]
    )
    for g in masked:
        model.geom_conaffinity[g] = 0

    left = unsupported_pairs(model)
    if left:
        types = {tuple(sorted(int(t) for t in model.geom_type[[a, b]])) for a, b in left}
        raise NotImplementedError(
            f"{len(left)} geom pairs remain unsupported by MJX after masking "
            f"self-collision (types {sorted(types)}). Masking only removes "
            "robot-against-robot; a pair involving the world is a real modelling "
            "question, not something to widen this mask for."
        )
    return Prepared(model=model, masked_geoms=masked)


def self_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> int:
    """Contacts in the current state that :func:`prepare` would remove.

    The instrument the docstring's caveat asks for: run it inside a rollout to
    find out whether a regime this package never measured actually leans on
    robot self-collision.
    """
    return sum(
        1
        for i in range(data.ncon)
        if model.geom_bodyid[data.contact[i].geom1] != WORLD_BODY
        and model.geom_bodyid[data.contact[i].geom2] != WORLD_BODY
    )


def load(preset_physics: dict[str, float] | None = None) -> Prepared:
    """The measured plant, prepared for MJX. The knobs apply first, as always."""
    model, _data = go2_model.load()
    if preset_physics:
        go2_model.apply_physics(model, preset_physics)
    return prepare(model)


def put_model(prepared: Prepared):  # type: ignore[no-untyped-def]
    """Hand the prepared model to MJX. Imported late: jax is a heavy dependency."""
    from mujoco import mjx  # type: ignore[attr-defined]

    return mjx.put_model(prepared.model)


def qpos_after(model: mujoco.MjModel, data: mujoco.MjData, steps: int) -> np.ndarray:
    """``qpos`` at every step of a passive rollout — the cross-engine yardstick."""
    out = np.empty((steps, model.nq))
    for i in range(steps):
        mujoco.mj_step(model, data)
        out[i] = data.qpos
    return out


def snap_state(
    model: mujoco.MjModel, data: mujoco.MjData, state: State, feet: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """``(qpos, qvel)`` for one measured state, FREE base — on the CPU, on purpose.

    A snap depends only on the MEASURED state: the pose, the yaw-stripped
    attitude, and the forward kinematics that drop the lowest foot onto the
    floor. Nothing about the rollout's own trajectory enters it, which is the
    seam's "decided before any physics" promise paying out — every snap can be
    computed once, up front, by exactly the code the CPU backend uses, and the
    batched loop just indexes the result.
    """
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[3:7] = mat_to_quat(state.rot)
    data.qpos[7:19] = state.q
    data.qpos[2] = 0.30
    mujoco.mj_forward(model, data)
    data.qpos[2] -= float(np.min(data.geom_xpos[feet, 2])) - go2_model.FOOT_RADIUS
    data.qvel[6:18] = state.dq
    data.qvel[3:6] = state.rot @ state.gyro
    if state.v_body is not None:
        data.qvel[0:3] = state.rot @ state.v_body
    mujoco.mj_forward(model, data)
    return data.qpos.copy(), data.qvel.copy()


def enable_x64() -> None:
    """Put jax in float64, which is the only precision this plant is defined in.

    The CPU plant is float64 and the knobs were fitted there; MJX defaults to
    float32, where a cross-engine comparison measures the dtype rather than the
    solver (the envelope alone moves by ~7e-6 N.m). jax exposes this as global
    config and nowhere else, so it is a call, made in one place, named.
    """
    import jax

    jax.config.update("jax_enable_x64", True)  # type: ignore[no-untyped-call]


def deliverable_jax(envelope: TorqueEnvelope, requested: Any, dq: Any) -> Any:
    """:meth:`TorqueEnvelope.deliverable`, in jax. Held to the original by test."""
    import jax.numpy as jnp

    speed = jnp.abs(dq)
    gain = jnp.interp(speed, jnp.asarray(envelope.gain_speed), jnp.asarray(envelope.gain))
    if envelope.brake_gain is not None:
        braking = jnp.interp(
            speed, jnp.asarray(envelope.gain_speed), jnp.asarray(envelope.brake_gain)
        )
        gain = jnp.where(requested * dq < 0.0, braking, gain)
    limit = jnp.interp(speed, jnp.asarray(envelope.ceiling_speed), jnp.asarray(envelope.ceiling))
    return jnp.clip(requested * gain, -limit, limit)


_STEP_CACHE: dict[bool, Any] = {}


def _jitted_step() -> Any:
    """One process-wide jitted ``mjx.step(mx, d)`` per x64 mode."""
    import jax
    from mujoco import mjx  # type: ignore[attr-defined]

    key = bool(jax.config.jax_enable_x64)
    if key not in _STEP_CACHE:
        _STEP_CACHE[key] = jax.jit(mjx.step)
    return _STEP_CACHE[key]


class MjxSession:
    """One closed-loop episode in MJX: Mode B's stepping primitive, batched-engine flavour.

    Exists so the referee can grade the plant that actually TRAINS. At a
    converged solver the CPU and MJX plants are the same object (1e-9 N.m
    per step) and this session is redundant; at the cheap solver the two
    engines truncate DIFFERENTLY, the truncation is plant behaviour, and a
    CPU verdict only bounds the MJX plant instead of grading it. This
    session closes that gap: the generic driver
    (:func:`~dimos.robot.unitree.go2.sim.sysid.ground.rollout_policy`) runs
    unchanged, one jitted ``mjx.step`` per tick, state read back each step.

    One env, one step per call — a per-step host/device round trip, priced
    for GRADING, not training: a 40 s rollout takes ~5.7 min, host-bound at
    ~25% GPU. Two operational consequences, both measured: replicates must
    run SERIAL (``ground --workers 1``; eight worker processes autotuning
    cuBLAS on one card fails outright with a JaxRuntimeError), and a
    verdict is better bought by PAIRING seeds across engines than by
    replicating each engine to the MDD — the same perturbation under both
    engines resolves a 0.4% difference that unpaired medians would call a
    tie. Snap
    placement is computed on a CPU twin of the same compiled model — the
    identical arithmetic :class:`~...engines.mujoco.MujocoSession.snap`
    does, preserving the sim's current world yaw and x/y — and uploaded.
    No viewer and no ghost: MJX has no CPU-side scene to attach one to.
    """

    def __init__(self, model: mujoco.MjModel, dt: float, x64: bool) -> None:
        import jax
        import jax.numpy as jnp
        from mujoco import mjx  # type: ignore[attr-defined]

        jax.config.update("jax_enable_x64", x64)  # type: ignore[no-untyped-call]
        self._model = model
        self._twin = mujoco.MjData(model)  # CPU scratch for snap kinematics
        self._feet = go2_model.foot_geom_ids(model)
        self._dt = dt
        self._mx = mjx.put_model(model)
        self._dx = mjx.make_data(self._mx)
        self._jnp = jnp
        # The model rides as an ARGUMENT, not a closure: jax caches traces by
        # function identity + abstract shapes, so every session of every
        # same-shaped plant reuses one compilation instead of re-jitting the
        # ~40 s step per episode (16 replicates would pay it 16 times).
        self._step_fn = _jitted_step()

    @property
    def timestep(self) -> float:
        return self._dt

    def state(self) -> LoopState:
        qpos = np.asarray(self._dx.qpos, dtype=float)
        qvel = np.asarray(self._dx.qvel, dtype=float)
        return LoopState(
            pos=qpos[0:3].copy(),
            quat=qpos[3:7].copy(),
            q=qpos[7:19].copy(),
            dq=qvel[6:18].copy(),
            gyro=qvel[3:6].copy(),
        )

    def step(self, ctrl: np.ndarray) -> bool:
        jnp = self._jnp
        self._dx = self._step_fn(
            self._mx, self._dx.replace(ctrl=jnp.asarray(ctrl, dtype=self._dx.ctrl.dtype))
        )
        return True

    def snap(self, state: State) -> None:
        """MujocoSession.snap's arithmetic on the CPU twin, then uploaded."""
        d, m = self._twin, self._model
        cur = np.asarray(self._dx.qpos, dtype=float)
        xy = cur[0:2].copy()
        w, x, y, z = cur[3:7]
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        cy, sy = np.cos(yaw), np.sin(yaw)
        rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        rot = rz @ state.rot
        d.qpos[:] = 0.0
        d.qvel[:] = 0.0
        d.qpos[0:2] = xy
        d.qpos[3:7] = mat_to_quat(rot)
        d.qpos[7:19] = state.q
        d.qpos[2] = 0.30
        mujoco.mj_forward(m, d)
        low = float(np.min(d.geom_xpos[self._feet, 2])) - go2_model.FOOT_RADIUS
        d.qpos[2] -= low  # lowest foot exactly on the floor
        d.qvel[6:18] = state.dq
        d.qvel[3:6] = rot @ state.gyro  # body rates -> world
        if state.v_body is not None:
            d.qvel[0:3] = rot @ state.v_body
        jnp = self._jnp
        self._dx = self._dx.replace(
            qpos=jnp.asarray(d.qpos, dtype=self._dx.qpos.dtype),
            qvel=jnp.asarray(d.qvel, dtype=self._dx.qvel.dtype),
        )

    def show_ghost(self, pos: np.ndarray, quat: np.ndarray) -> None:
        pass  # visual only, and there is nothing here to draw on

    def close(self) -> None:
        pass


class MjxBackend:
    """The measured plant under MJX: same model, same plan, same prediction.

    Every fitted knob reaches this engine through
    :func:`~dimos.robot.unitree.go2.sim.engines.model.apply_physics`, the same
    call the CPU backend makes, because MJX compiles the same
    :class:`mujoco.MjModel` — so a disagreement between the two backends is
    the SOLVER, never the plant. :func:`prepare` is the one documented
    exception, and it is measured free (module docstring).

    Two declared gaps, both visible through the protocol rather than hidden:

    * ``accel`` is absent from :meth:`channels`. The virtual IMU needs
      ``mj_objectAcceleration`` at the ``imu`` site, which MJX does not expose;
      re-deriving it from ``rne_postconstraint`` is a port of its own. It costs
      the fit nothing today — ``score.DEFAULT_WEIGHTS`` puts ``accel`` at zero
      — and :meth:`channels` is exactly how a backend says so.
    * ``BaseCondition.PINNED`` raises. The suspended regime needs a weld driven
      per-step from a mocap body; the walking regime is what a batched engine
      is for, and a wrong hanging plant is worse than an absent one.
    """

    name = "mjx"

    def __init__(
        self,
        menagerie: Path | None = None,
        *,
        envelope: TorqueEnvelope | None = None,
        x64: bool = True,
    ) -> None:
        self._menagerie = menagerie
        self._envelope = envelope
        # float64 is the plant's native precision (enable_x64); x64=False runs
        # the TRAINING dtype instead — consumer GPUs do fp64 at 1/32-1/64 of
        # fp32 — so what fp32 costs is measurable rather than argued.
        self._x64 = x64
        self._values: dict[str, float] = {}
        self._dt: float | None = None

    @property
    def timestep(self) -> float:
        if self._dt is None:
            model = mujoco.MjModel.from_xml_path(str(go2_model.scene_path(self._menagerie)))
            self._dt = float(model.opt.timestep)
        return self._dt

    def knobs(self) -> Mapping[str, Knob]:
        return KNOBS

    def channels(self) -> frozenset[str]:
        return frozenset(CHANNELS) - {"accel"}

    def apply(self, values: Mapping[str, float]) -> None:
        unknown = set(values) - set(KNOBS)
        if unknown:
            raise ValueError(f"unknown knob(s) for {self.name}: {sorted(unknown)}")
        self._values = dict(values)

    def session(
        self,
        pose: np.ndarray,
        *,
        ghost: bool = False,
        view: bool = False,
        view_speed: float = 1.0,
    ) -> MjxSession:
        """A closed-loop episode: applied physics, keyframe home, joints at ``pose``.

        The MuJoCo session's contract, minus what MJX cannot host: a viewer
        or a ghost is a refusal, not a silent downgrade — watch the same
        preset under the CPU engine instead (the compiled model is shared,
        so at a converged solver it IS the same plant; at a truncated one
        the difference is exactly what this session exists to measure).
        """
        if view or ghost:
            raise NotImplementedError(
                f"{self.name} has no viewer to attach and no scene to draw a ghost in; "
                "use the mujoco backend to watch"
            )
        model, data = go2_model.load(self._menagerie)
        physics = {k: v for k, v in self._values.items() if k in PHYSICS_KEYS}
        if physics:
            go2_model.apply_physics(model, physics)
        prepare(model)
        kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")  # type: ignore[attr-defined]
        if kid >= 0:
            mujoco.mj_resetDataKeyframe(model, data, kid)
        data.qpos[7:19] = pose
        mujoco.mj_forward(model, data)
        session = MjxSession(model, float(model.opt.timestep), self._x64)
        session._dx = session._dx.replace(
            qpos=session._jnp.asarray(data.qpos, dtype=session._dx.qpos.dtype),
            qvel=session._jnp.asarray(data.qvel, dtype=session._dx.qvel.dtype),
        )
        return session

    def rollout(self, plan: RolloutPlan) -> Prediction:
        if plan.base is not BaseCondition.FREE:
            raise NotImplementedError(
                f"{self.name} implements BaseCondition.FREE only; {plan.base.name} needs "
                "the mocap weld driven inside the stepped loop. Use the mujoco backend "
                "for the suspended regime."
            )
        import jax
        import jax.numpy as jnp
        from mujoco import mjx  # type: ignore[attr-defined]

        jax.config.update("jax_enable_x64", self._x64)  # type: ignore[no-untyped-call]

        model, data = go2_model.load(self._menagerie)
        physics = {k: v for k, v in self._values.items() if k in PHYSICS_KEYS}
        if physics:
            go2_model.apply_physics(model, physics)
        prepare(model)
        dt = float(model.opt.timestep)
        feet = go2_model.foot_geom_ids(model)
        actuator_tau = float(self._values.get("actuator_tau", 0.0))
        alpha = dt / (actuator_tau + dt) if actuator_tau > 0.0 else 1.0

        reinit = list(plan.reinit)
        if not reinit or reinit[0].t > plan.t0:
            raise ValueError("plan.reinit must start with the state at t0")

        # Everything the plan already determined, resolved before any physics:
        # which command row each step reads, and which step each snap fires on.
        n = int(plan.duration / dt)
        step_t = plan.t0 + np.arange(n) * dt
        cmds = plan.commands
        ci = np.clip(np.searchsorted(cmds.t, step_t, "right") - 1, 0, len(cmds.t) - 1)
        snap_qpos, snap_qvel = zip(*(snap_state(model, data, s, feet) for s in reinit), strict=True)
        snap_step = np.searchsorted(step_t, [s.t for s in reinit], "left")
        snap_step[0] = 0
        # -1 where no snap fires; the row of `snap_*` to load where one does.
        snap_at = np.full(n, -1, dtype=int)
        for row, st in enumerate(snap_step):
            if 0 <= st < n:
                snap_at[st] = row

        mx = mjx.put_model(model)
        dx = mjx.make_data(mx)

        j_snap_qpos = jnp.asarray(np.array(snap_qpos))
        j_snap_qvel = jnp.asarray(np.array(snap_qvel))
        j_snap_at = jnp.asarray(snap_at)
        j_kp, j_kd = jnp.asarray(cmds.kp[ci]), jnp.asarray(cmds.kd[ci])
        j_q, j_dq = jnp.asarray(cmds.q[ci]), jnp.asarray(cmds.dq[ci])
        j_ff = jnp.asarray(cmds.tau_ff[ci])
        j_limits = jnp.asarray(TORQUE_LIMITS)
        envelope = self._envelope

        def one_step(carry, inp):  # type: ignore[no-untyped-def]
            d, applied = carry
            row, kp, kd, q_des, dq_des, ff = inp
            fired = row >= 0
            d = d.replace(
                qpos=jnp.where(fired, j_snap_qpos[row], d.qpos),
                qvel=jnp.where(fired, j_snap_qvel[row], d.qvel),
            )
            applied = jnp.where(fired, jnp.zeros_like(applied), applied)

            q, dq = d.qpos[7:19], d.qvel[6:18]
            tau = jnp.clip(kp * (q_des - q) + kd * (dq_des - dq) + ff, -j_limits, j_limits)
            if envelope is not None:
                tau = deliverable_jax(envelope, tau, dq)
            applied = applied + alpha * (tau - applied)
            d = mjx.step(mx, d.replace(ctrl=applied))
            return (d, applied), (d.qpos, d.qvel, applied)

        (_, _), (qpos, qvel, tau_out) = jax.lax.scan(
            one_step,
            (dx, jnp.zeros(12)),
            (j_snap_at, j_kp, j_kd, j_q, j_dq, j_ff),
        )
        qpos, qvel, tau_out = np.asarray(qpos), np.asarray(qvel), np.asarray(tau_out)

        rot = np.array([quat_to_mat(w) for w in qpos[:, 3:7]])
        gyro = np.einsum("nji,nj->ni", rot, qvel[:, 3:6])  # world rates -> trunk frame
        pose = slice(None, None, 5)  # the 100 Hz pose-rate log
        return Prediction(
            t=step_t[pose],
            q=qpos[pose, 7:19],
            dq=qvel[pose, 6:18],
            body_pos=qpos[pose, 0:3],
            body_rot=rot[pose],
            at=step_t,
            imu_accel=np.zeros((0, 3)),  # not a channel this backend declares
            imu_gyro=gyro,
            tau=tau_out,
            reinit_t=np.asarray([s.t for s in reinit], dtype=float),
            reinit_pos=np.array(snap_qpos)[:, 0:3],
            reinit_rot=np.array([quat_to_mat(qp[3:7]) for qp in snap_qpos]),
        )
