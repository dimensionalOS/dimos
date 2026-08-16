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

"""The backend seam: what a simulator must provide to be identified.

Everything above this seam — ingest, regimes, clip scheduling, channels, the
fit — is simulator-agnostic. The backend never sees a regime, a channel or a
fit: it is handed a :class:`RolloutPlan` and returns a :class:`Prediction`.
MuJoCo implements it today; IsaacLab is the next backend, and PhysX shares no
contact model with MuJoCo, so fitted VALUES will not transfer — the
recordings, anchors, and method are what transfer.

Four expensive lessons are encoded in the types rather than left to comments:
``Knob.log`` because a bound must be judged in the parameter's own metric;
``Knob.why`` because a range without provenance is a guess; ``BaseCondition``
because a suspended robot needs a pinned trunk and no floor; and
``Prediction.imu_accel`` naming the sensor mount because reading it at the
body frame is the single easiest thing here to get silently wrong.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
import enum
from pathlib import Path
from typing import Protocol

import mujoco
import numpy as np

from dimos.robot.unitree.go2.sim import model as go2_model
from dimos.robot.unitree.go2.sim.plant import TORQUE_LIMITS, TorqueEnvelope, actuator_step
from dimos.robot.unitree.go2.sim.ranges import KNOBS, PHYSICS_KEYS, Knob
from dimos.robot.unitree.go2.sim.rotations import mat_to_quat, quat_to_mat


@dataclass(frozen=True)
class Commands:
    """Recorded joint-space commands, held zero-order between samples.

    The board's law is ``kp*(q_des-q) + kd*(dq_des-dq) + tau_ff``, and
    ``dq_des`` is NOT always zero: Unitree's built-in controller commands up
    to 20.7 rad/s. All arrays are in MuJoCo actuator order (FL FR RL RR).
    """

    t: np.ndarray  # (m,)
    q: np.ndarray  # (m, 12) target angles
    dq: np.ndarray  # (m, 12) target speeds
    kp: np.ndarray  # (m, 12)
    kd: np.ndarray  # (m, 12)
    tau_ff: np.ndarray  # (m, 12)


@dataclass(frozen=True)
class State:
    """One measured robot state, everything a backend needs to snap to it.

    The base HEIGHT is deliberately absent: the room frame's floor is unknown,
    so the backend places the lowest foot on its own floor (or clears the
    floor entirely under a pinned base). ``rot`` is yaw-stripped — flat ground
    is yaw-symmetric and the room's yaw is arbitrary.
    """

    t: float
    q: np.ndarray  # (12,) joint angles
    dq: np.ndarray  # (12,) joint speeds
    rot: np.ndarray  # (3, 3) gravity-referenced attitude, yaw stripped
    gyro: np.ndarray  # (3,) body angular rate
    v_body: np.ndarray | None = None  # (3,) base velocity in body frame; tracker only


class BaseCondition(enum.Enum):
    """What holds the trunk — the boundary condition the sim must impose."""

    FREE = "free"
    # The robot hung from a rope: pin the trunk to the MEASURED pose each clip
    # (it hangs 70-85 deg off level; holding it level points gravity the wrong
    # way through every leg), clear of a floor the real robot never met.
    PINNED = "pinned"


@dataclass(frozen=True)
class RolloutPlan:
    """One open-loop drive of the plant, fully determined before any physics.

    ``reinit`` is the multiple-shooting schedule: measured states the sim is
    snapped to, the first at ``t0``. The schedule is computed above the seam
    (seeded, shared across candidate plants — see ``sysid.regimes``) so every
    candidate is scored on identical clips.
    """

    t0: float
    duration: float
    commands: Commands
    reinit: Sequence[State]
    base: BaseCondition = BaseCondition.FREE


@dataclass(frozen=True)
class Prediction:
    """What the sim predicts of the signals the real robot also measures.

    Nothing here is a modelling choice: every array corresponds to a signal in
    ``rt/lowstate`` or the tracker. Two time bases on purpose — an impact is
    30-50 ms wide with a ~30 ms rise, so ``imu_accel``/``imu_gyro``/``tau``
    are sampled at the full physics rate (``at``) while the pose-rate arrays
    (``t``) match the 100 Hz the comparison has always used.

    ``reinit_*`` are the sim poses at each snap. They are bookkeeping, not
    prediction: open-loop body drift is only defined relative to the pose the
    clip started from, and the scorer needs it to place the recorded track in
    this rollout's world frame.
    """

    t: np.ndarray  # (n,) pose-rate sample times
    q: np.ndarray  # (n, 12) joint angles
    dq: np.ndarray  # (n, 12) joint speeds
    body_pos: np.ndarray  # (n, 3)
    body_rot: np.ndarray  # (n, 3, 3)
    at: np.ndarray  # (k,) physics-rate sample times
    imu_accel: np.ndarray  # (k, 3) SPECIFIC FORCE AT THE SENSOR MOUNT — the
    # model's `imu` site, 49 mm off the trunk frame. Off-axis it reads
    # alpha x r + omega x (omega x r) on top of the frame's acceleration:
    # 5-25 m/s2 during a landing, the same order as the impact itself.
    imu_gyro: np.ndarray  # (k, 3) angular rate, trunk frame
    tau: np.ndarray  # (k, 12) delivered joint torques
    reinit_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    reinit_pos: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    reinit_rot: np.ndarray = field(default_factory=lambda: np.zeros((0, 3, 3)))


class Backend(Protocol):
    """What a simulator must provide to be identified against a recording."""

    @property
    def name(self) -> str: ...

    @property
    def timestep(self) -> float:
        """The engine's physics step, s. Reinit times are quantized to it:
        a snap can only happen on a step boundary, and the measured state must
        be sampled at the exact time the snap will fire."""
        ...

    def knobs(self) -> Mapping[str, Knob]:
        """The parameters THIS engine exposes. The set is data, not code."""
        ...

    def apply(self, values: Mapping[str, float]) -> None:
        """Push values into the engine. Absent keys keep engine defaults."""
        ...

    def rollout(self, plan: RolloutPlan) -> Prediction:
        """Drive the plan and return what the real robot also measures."""
        ...


class MujocoBackend:
    """The menagerie Go2 behind the seam.

    Deterministic by construction: a fresh model is compiled per rollout with
    the currently applied knob values, so no state leaks between rollouts and
    the same plan always yields the same prediction.
    """

    name = "mujoco"

    def __init__(
        self,
        menagerie: Path | None = None,
        *,
        envelope: TorqueEnvelope | None = None,
    ) -> None:
        self._menagerie = menagerie
        self._envelope = envelope
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

    def apply(self, values: Mapping[str, float]) -> None:
        unknown = set(values) - set(KNOBS)
        if unknown:
            raise ValueError(f"unknown knob(s) for {self.name}: {sorted(unknown)}")
        self._values = dict(values)

    def rollout(self, plan: RolloutPlan) -> Prediction:
        model, data = go2_model.load(self._menagerie)
        physics = {k: v for k, v in self._values.items() if k in PHYSICS_KEYS}
        if physics:
            go2_model.apply_physics(model, physics)
        actuator_tau = self._values.get("actuator_tau", 0.0)
        dt = float(model.opt.timestep)
        feet = go2_model.foot_geom_ids(model)
        pinned = plan.base is BaseCondition.PINNED

        reinit = list(plan.reinit)
        if not reinit or reinit[0].t > plan.t0:
            raise ValueError("plan.reinit must start with the state at t0")

        def snap(state: State) -> None:
            data.qpos[:] = 0.0
            data.qvel[:] = 0.0
            data.qpos[3:7] = mat_to_quat(state.rot)
            data.qpos[7:19] = state.q
            data.qpos[2] = 0.30
            mujoco.mj_forward(model, data)
            low = float(np.min(data.geom_xpos[feet, 2])) - go2_model.FOOT_RADIUS
            data.qpos[2] -= low  # lowest foot exactly on the floor
            data.qvel[6:18] = state.dq
            data.qvel[3:6] = state.rot @ state.gyro  # body rates -> world
            if state.v_body is not None:
                data.qvel[0:3] = state.rot @ state.v_body
            mujoco.mj_forward(model, data)
            if pinned:
                data.qpos[2] = max(float(data.qpos[2]), 2.0)  # clear of the floor

        snap(reinit[0])
        pinned_qpos = data.qpos[0:7].copy()

        # The virtual IMU: `mj_objectAcceleration(..., flg_local=1)` returns
        # [angular, linear] in local coordinates and MuJoCo seeds the world
        # body with -gravity, so the linear half IS specific force — ~9.8
        # standing, ~0 in free fall — the same convention as rt/lowstate's
        # accelerometer. Read AT THE `imu` SITE: 49 mm of lever arm is worth
        # 1.8x on the landing residual (15.33 vs 8.61 RMS against a 9.01
        # signal — read at the frame it is worse than predicting zero).
        imu_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, go2_model.IMU_SITE)
        imu_objtype = mujoco.mjtObj.mjOBJ_SITE
        if imu_id < 0:  # a model without the sensor's mount point: fall back
            imu_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
            imu_objtype = mujoco.mjtObj.mjOBJ_BODY
        acc6 = np.zeros(6)

        cmds = plan.commands
        applied = np.zeros(12)
        ts: list[float] = []
        qs: list[np.ndarray] = []
        dqs: list[np.ndarray] = []
        ps: list[np.ndarray] = []
        rs: list[np.ndarray] = []
        ats: list[float] = []
        asim: list[np.ndarray] = []
        wsim: list[np.ndarray] = []
        tsim: list[np.ndarray] = []
        reinit_t: list[float] = [plan.t0]
        reinit_pos: list[np.ndarray] = [data.qpos[0:3].copy()]
        reinit_rot: list[np.ndarray] = [quat_to_mat(data.qpos[3:7].copy())]
        si = 1

        n = int(plan.duration / dt)
        for step in range(n):
            t = plan.t0 + step * dt
            if si < len(reinit) and t >= reinit[si].t:
                snap(reinit[si])
                if pinned:
                    pinned_qpos = data.qpos[0:7].copy()
                applied[:] = 0.0
                reinit_t.append(t)
                reinit_pos.append(data.qpos[0:3].copy())
                reinit_rot.append(quat_to_mat(data.qpos[3:7].copy()))
                si += 1

            k = int(np.clip(np.searchsorted(cmds.t, t, "right") - 1, 0, len(cmds.t) - 1))
            tau = (
                cmds.kp[k] * (cmds.q[k] - data.qpos[7:19])
                + cmds.kd[k] * (cmds.dq[k] - data.qvel[6:18])
                + cmds.tau_ff[k]
            )
            tau = np.clip(tau, -TORQUE_LIMITS, TORQUE_LIMITS)
            applied = actuator_step(
                applied, tau, dt, actuator_tau, dq=data.qvel[6:18], envelope=self._envelope
            )
            data.ctrl[:] = applied
            mujoco.mj_step(model, data)
            if pinned:
                # The rope holds the trunk, the feet never touch anything. A
                # free base would drop it onto a floor the real robot never
                # met and score the impact as plant error.
                data.qpos[0:7] = pinned_qpos
                data.qvel[0:6] = 0.0

            # Every step, not every fifth: an impact is ~20 ms wide.
            # Both exist at runtime; the bundled mujoco stubs omit them.
            mujoco.mj_rnePostConstraint(model, data)  # type: ignore[attr-defined]
            mujoco.mj_objectAcceleration(  # type: ignore[attr-defined]
                model, data, imu_objtype, imu_id, acc6, 1
            )
            ats.append(t)
            asim.append(acc6[3:].copy())
            # Angular rate in the trunk's own frame — the convention
            # rt/lowstate's gyroscope reports in.
            wsim.append(quat_to_mat(data.qpos[3:7]).T @ data.qvel[3:6])
            tsim.append(applied.copy())

            if step % 5 == 0:  # pose-rate log at 100 Hz
                ts.append(t)
                qs.append(data.qpos[7:19].copy())
                dqs.append(data.qvel[6:18].copy())
                ps.append(data.qpos[0:3].copy())
                rs.append(quat_to_mat(data.qpos[3:7].copy()))

        return Prediction(
            t=np.array(ts),
            q=np.array(qs).reshape(-1, 12),
            dq=np.array(dqs).reshape(-1, 12),
            body_pos=np.array(ps).reshape(-1, 3),
            body_rot=np.array(rs).reshape(-1, 3, 3),
            at=np.array(ats),
            imu_accel=np.array(asim).reshape(-1, 3),
            imu_gyro=np.array(wsim).reshape(-1, 3),
            tau=np.array(tsim).reshape(-1, 12),
            reinit_t=np.array(reinit_t),
            reinit_pos=np.array(reinit_pos).reshape(-1, 3),
            reinit_rot=np.array(reinit_rot).reshape(-1, 3, 3),
        )
