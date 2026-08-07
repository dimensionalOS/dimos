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

"""Replay recorded Go2 ``lowcmd`` into flat-ground MuJoCo.

The robot's joint controller is PD-plus-feedforward, evaluated on the motor
board::

    tau = kp * (q_des - q) + kd * (dq_des - dq) + tau_ff

so replaying means re-running that law against the *simulated* joint state,
not playing back recorded torques. Recorded ``lowstate`` is the reference the
result is scored against.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from dimos.navigation.motion.simulation import model as go2_model


@dataclass
class Command:
    """One LowCmd sample, reduced to the 12 leg motors in Unitree order."""

    ts: float
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray
    kp: np.ndarray
    kd: np.ndarray


@dataclass
class State:
    """One LowState sample, reduced to the 12 leg motors in Unitree order."""

    ts: float
    q: np.ndarray
    dq: np.ndarray
    tau: np.ndarray


@dataclass
class Rollout:
    """Simulated trajectory, sampled once per applied command."""

    ts: np.ndarray
    qpos: np.ndarray = field(repr=False)  # (n, nq) full state incl. free joint
    joint_q: np.ndarray = field(repr=False)  # (n, 12) MuJoCo actuator order
    joint_dq: np.ndarray = field(repr=False)
    tau: np.ndarray = field(repr=False)  # (n, 12) applied torque
    base_pos: np.ndarray = field(repr=False)  # (n, 3)
    base_quat: np.ndarray = field(repr=False)  # (n, 4) wxyz


def _motors(msg: Any, attr: str) -> list[Any]:
    return list(getattr(msg, attr))[:12]


def read_commands(dataset: str | Path, *, limit: int | None = None) -> list[Command]:
    from dimos.memory2.cli.dataset import open_dataset

    store = open_dataset(str(dataset))
    out: list[Command] = []
    with store:
        stream: Any = store.stream("lowcmd")
        if limit is not None:
            stream = stream.limit(limit)
        for obs in stream:
            m = _motors(obs.data, "motor_cmd")
            out.append(
                Command(
                    ts=obs.ts,
                    q=np.array([x.q for x in m]),
                    dq=np.array([x.dq for x in m]),
                    tau=np.array([x.tau for x in m]),
                    kp=np.array([x.kp for x in m]),
                    kd=np.array([x.kd for x in m]),
                )
            )
    return out


def read_states(dataset: str | Path, *, limit: int | None = None) -> list[State]:
    from dimos.memory2.cli.dataset import open_dataset

    store = open_dataset(str(dataset))
    out: list[State] = []
    with store:
        stream: Any = store.stream("lowstate")
        if limit is not None:
            stream = stream.limit(limit)
        for obs in stream:
            m = _motors(obs.data, "motor_state")
            out.append(
                State(
                    ts=obs.ts,
                    q=np.array([x.q for x in m]),
                    dq=np.array([x.dq for x in m]),
                    tau=np.array([x.tau_est for x in m]),
                )
            )
    return out


def replay(
    commands: list[Command],
    *,
    init_state: State | None = None,
    menagerie: Path | None = None,
    keyframe: str | None = "home",
) -> Rollout:
    """Step MuJoCo once per command, applying the on-board PD law.

    ``init_state`` seeds the leg joints from a recorded LowState so the rollout
    starts where the robot actually was; the base pose is not recorded in
    ``lowstate`` and falls back to ``keyframe``.
    """
    model, data = go2_model.load(menagerie)
    perm = go2_model.unitree_to_mujoco(model)
    qadr = go2_model.joint_qpos_adr(model)
    vadr = go2_model.joint_dof_adr(model)

    if keyframe is not None:
        # mjOBJ_KEY exists at runtime; the bundled mujoco stubs omit it.
        kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe)  # type: ignore[attr-defined]
        if kid >= 0:
            mujoco.mj_resetDataKeyframe(model, data, kid)
    if init_state is not None:
        data.qpos[qadr] = init_state.q[perm]
        data.qvel[vadr] = init_state.dq[perm]
    mujoco.mj_forward(model, data)

    n = len(commands)
    ts = np.empty(n)
    qpos = np.empty((n, model.nq))
    jq = np.empty((n, 12))
    jdq = np.empty((n, 12))
    taus = np.empty((n, 12))

    lo = model.actuator_ctrlrange[:, 0]
    hi = model.actuator_ctrlrange[:, 1]

    for i, cmd in enumerate(commands):
        q = data.qpos[qadr]
        dq = data.qvel[vadr]
        tau = cmd.kp[perm] * (cmd.q[perm] - q) + cmd.kd[perm] * (cmd.dq[perm] - dq) + cmd.tau[perm]
        tau = np.clip(tau, lo, hi)
        data.ctrl[:] = tau
        mujoco.mj_step(model, data)

        ts[i] = cmd.ts
        qpos[i] = data.qpos
        jq[i] = data.qpos[qadr]
        jdq[i] = data.qvel[vadr]
        taus[i] = tau

    return Rollout(
        ts=ts,
        qpos=qpos,
        joint_q=jq,
        joint_dq=jdq,
        tau=taus,
        base_pos=qpos[:, 0:3].copy(),
        base_quat=qpos[:, 3:7].copy(),
    )


def tracking_error(rollout: Rollout, states: list[State], model: mujoco.MjModel) -> dict[str, Any]:
    """Per-joint RMS error between simulated and recorded joint angles.

    This is the check that catches a wrong motor permutation: a mismatched
    ordering shows up as large, structured error on most joints rather than
    the small drift you get from dynamics mismatch alone.
    """
    perm = go2_model.unitree_to_mujoco(model)
    s_ts = np.array([s.ts for s in states])
    s_q = np.stack([s.q for s in states])[:, perm]

    idx = np.searchsorted(s_ts, rollout.ts).clip(0, len(s_ts) - 1)
    ref = s_q[idx]
    err = rollout.joint_q - ref
    return {
        "rms_per_joint": np.sqrt((err**2).mean(axis=0)),
        "rms_overall": float(np.sqrt((err**2).mean())),
        "max_abs": float(np.abs(err).max()),
        "n": len(rollout.ts),
    }


def view(
    commands: list[Command],
    *,
    init_state: State | None = None,
    menagerie: Path | None = None,
    keyframe: str | None = "home",
    speed: float = 1.0,
) -> None:
    """Same loop as :func:`replay`, but in an interactive MuJoCo window.

    Paced to wall-clock so the motion reads at ``speed`` x real time.
    """
    import time

    import mujoco.viewer

    model, data = go2_model.load(menagerie)
    perm = go2_model.unitree_to_mujoco(model)
    qadr = go2_model.joint_qpos_adr(model)
    vadr = go2_model.joint_dof_adr(model)

    if keyframe is not None:
        # mjOBJ_KEY exists at runtime; the bundled mujoco stubs omit it.
        kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, keyframe)  # type: ignore[attr-defined]
        if kid >= 0:
            mujoco.mj_resetDataKeyframe(model, data, kid)
    if init_state is not None:
        data.qpos[qadr] = init_state.q[perm]
        data.qvel[vadr] = init_state.dq[perm]
    mujoco.mj_forward(model, data)

    lo = model.actuator_ctrlrange[:, 0]
    hi = model.actuator_ctrlrange[:, 1]
    dt = model.opt.timestep / max(speed, 1e-6)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        wall = time.perf_counter()
        for cmd in commands:
            if not viewer.is_running():
                break
            q = data.qpos[qadr]
            dq = data.qvel[vadr]
            tau = (
                cmd.kp[perm] * (cmd.q[perm] - q)
                + cmd.kd[perm] * (cmd.dq[perm] - dq)
                + cmd.tau[perm]
            )
            data.ctrl[:] = np.clip(tau, lo, hi)
            mujoco.mj_step(model, data)
            viewer.sync()

            wall += dt
            lag = wall - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
            else:
                wall = time.perf_counter()
