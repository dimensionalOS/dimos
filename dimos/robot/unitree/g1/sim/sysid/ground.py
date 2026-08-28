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

"""Loop 2 on the G1: GR00T drives the plant, Point-LIO is the yardstick, and the fit runs HERE.

Each window starts from the robot's MEASURED mid-walk state (joints, attitude,
base velocity, the policy's own six-frame history teacher-forced from the
recording) and replays the recorded velocity commands through the real
policy. The verdict per window is the tracking area: mean |error| over the
horizon of the pelvis position, split along/cross in the real trajectory's
heading frame, and of yaw (go2sim's headline statistic). The loss a fit
minimises is the mean over windows and terms of area / stock area.

Three kinds of knob are searched here. Plant knobs (``ranges.KNOBS``) land on
the compiled model. LOOP knobs live in the stepping loop: a speed-torque
envelope (``envelope_gain`` at and beyond ``envelope_speed`` rad/s, 1.0 at
rest, linear between) and ``action_delay`` physics steps between inference
and the PD seeing the target. RIG knobs (``rig_dx/dy/dz``) are a pelvis-frame
lever-arm correction on where Point-LIO says the pelvis is: they move the
YARDSTICK, not the plant, and are reported apart so a fit that improves by
moving the ruler is seen doing it.

This is fitting THROUGH the controller, which go2sim measured to anti-transfer
on the Go2 twice; it is used here by decision, with that on record.

    python -m dimos.robot.unitree.g1.sim.sysid.ground REC.db --preset measured
    python -m dimos.robot.unitree.g1.sim.sysid.ground REC.db --fit 60 --studies 3 --workers 8 --out presets/candidate
    python -m dimos.robot.unitree.g1.sim.sysid.ground REC.db --preset measured --view
"""

from __future__ import annotations

import argparse
from collections import deque
import concurrent.futures
from dataclasses import dataclass
import json
import multiprocessing
from pathlib import Path
import time
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from dimos.robot.unitree.g1.sim import model as g1_model
from dimos.robot.unitree.g1.sim.plant import KD, KP, TORQUE_LIMITS
from dimos.robot.unitree.g1.sim.policy import (
    CONTROL_DT,
    DEFAULT_29,
    NUM_ACTIONS,
    OBS_HISTORY,
    GrootPolicy,
    action_of_target,
    observation,
)
from dimos.robot.unitree.g1.sim.ranges import ENGINE_DEFAULTS, KNOBS, PHYSICS_KEYS, load_preset
from dimos.robot.unitree.g1.sim.sysid.ingest import read_streams
from dimos.simulation.sysid.plant import TorqueEnvelope, actuator_step
from dimos.simulation.sysid.presets import Knob, Preset
from dimos.simulation.sysid.recording import Streams
from dimos.simulation.sysid.replay import measured_state
from dimos.simulation.sysid.rotations import (
    mat_to_quat,
    pitch_roll_of,
    quat_to_mat,
    yaw_anchor,
    yaw_of,
)
from dimos.utils.data import LfsPath

# along/cross/yaw are tracking areas; cadence is the relative error of the
# sway frequency (the roll spectrum's peak in 0.5-4 Hz): a plant can buy
# tracking area by striding slower, and only cadence sees it.
TERMS = ("along", "cross", "yaw", "cadence")
CADENCE_SCALE = 0.05  # a stock plant that already matches cadence must not make the term explode
PERTURB_RAD = 0.005  # sim-perturb replicates: +-0.3 deg on the 12 seeded leg joints
TRACKING_ANCHOR_S = 0.5  # where the free rollout is anchored to the room

# Knobs that live in the loop or on the yardstick, not on the compiled model.
LOOP_KNOBS: dict[str, Knob] = {
    "envelope_gain": Knob(
        0.3,
        1.0,
        why="torque delivered at high joint speed as a fraction of the demand; the Go2 "
        "measured ~0.5 above 3 rad/s and it closed half its speed deficit; 1.0 = no envelope",
    ),
    "envelope_speed": Knob(
        2.0,
        20.0,
        log=True,
        unit="rad/s",
        why="joint speed at which the derate reaches envelope_gain",
    ),
    "action_delay": Knob(
        0,
        4,
        unit="steps",
        why="physics steps (5 ms) between inference and the PD seeing the "
        "target; the real coordinator ticks at 100 Hz with transport on top; rounded",
    ),
    "rig_dx": Knob(-0.1, 0.1, unit="m", why="Point-LIO -> pelvis lever-arm correction, pelvis x"),
    "rig_dy": Knob(-0.1, 0.1, unit="m", why="Point-LIO -> pelvis lever-arm correction, pelvis y"),
    "rig_dz": Knob(-0.1, 0.1, unit="m", why="Point-LIO -> pelvis lever-arm correction, pelvis z"),
    "rig_dt": Knob(
        -0.15,
        0.15,
        unit="s",
        why="Point-LIO track latency vs the store clock. MEASURED 2026-08-28 by "
        "cross-correlating IMU yaw/roll/pitch rate with the track's: +10 ms on every "
        "axis (corr 0.7-0.84); pin it, do not search it",
    ),
    "cmd_dt": Knob(
        -0.15,
        0.15,
        unit="s",
        why="velocity-command clock vs the store clock: the payload stamps run ~50 ms "
        "behind, so when the policy actually saw a command is unknown to that order",
    ),
}
LOOP_DEFAULTS: dict[str, float] = {
    "envelope_gain": 1.0,
    "envelope_speed": 8.0,
    "action_delay": 0.0,
    "rig_dx": 0.0,
    "rig_dy": 0.0,
    "rig_dz": 0.0,
    "rig_dt": 0.01,
    "cmd_dt": 0.0,
}
ALL_KNOBS: dict[str, Knob] = {**KNOBS, **LOOP_KNOBS}
DEFAULT_SEARCH = (
    "armature",
    "frictionloss",
    "actuator_tau",
    "foot_solref_time",
    "foot_solref_damp",
    "trunk_mass_scale",
    "trunk_com_x",
    "leg_mass_scale",
    "foot_friction",
    "foot_solimp_dmin",
    "foot_solimp_width",
    "envelope_gain",
    "envelope_speed",
    "action_delay",
    "rig_dx",
    "rig_dy",
    "rig_dz",
    "cmd_dt",
)

_POLICY: GrootPolicy | None = None


def _policy() -> GrootPolicy:
    global _POLICY
    if _POLICY is None:
        _POLICY = GrootPolicy(Path(str(LfsPath("groot"))))
    return _POLICY


@dataclass
class PolicyRun:
    """One closed-loop rollout at the physics rate; ``t`` = 0 at ``start``."""

    t: NDArray[Any]
    pos: NDArray[Any]
    quat: NDArray[Any]


def _cmd_at(st: Streams, t_abs: float, cmd_dt: float = 0.0) -> NDArray[Any]:
    """The velocity command in force at ``t_abs``; ``cmd_dt`` shifts the command clock."""
    i = int(np.searchsorted(st.wt + cmd_dt, t_abs, "right")) - 1
    return st.wcmd[i] if i >= 0 else np.zeros(3)


def _envelope(values: dict[str, float]) -> TorqueEnvelope | None:
    g = float(values.get("envelope_gain", 1.0))
    if g >= 1.0:
        return None
    s = float(values.get("envelope_speed", 8.0))
    top = float(TORQUE_LIMITS.max())
    return TorqueEnvelope("fit", (0.0, s), (1.0, g), (0.0, 1.0), (top, top))


def base_pose(st: Streams, values: dict[str, float]) -> tuple[NDArray[Any], NDArray[Any]]:
    """Point-LIO's pelvis track with the rig knobs' lever-arm correction applied."""
    base_p, base_r = st.base_pose_room()
    d = np.array([values.get(k, 0.0) for k in ("rig_dx", "rig_dy", "rig_dz")])
    if d.any():
        base_p = base_p + base_r @ d
    # A track that lags the store clock by rig_dt is moved EARLIER by that
    # much: whole grid samples, edge-held, so the grid itself stays put.
    k = round(float(values.get("rig_dt", 0.0)) / float(np.median(np.diff(st.vt))))
    if k:
        idx = np.clip(np.arange(len(st.vt)) + k, 0, len(st.vt) - 1)
        base_p, base_r = base_p[idx], base_r[idx]
    return base_p, base_r


def rollout(
    st: Streams,
    values: dict[str, float],
    start: float,
    seconds: float,
    *,
    seed: int = 0,
    view: bool = False,
    speed: float = 1.0,
) -> PolicyRun:
    """GR00T on the plant ``values`` from the measured state at ``start``."""
    model, data = g1_model.load(ghost=view)
    g1_model.apply_physics(model, {k: v for k, v in values.items() if k in PHYSICS_KEYS})
    tau_lag = float(values.get("actuator_tau", 0.0))
    cmd_dt = float(values.get("cmd_dt", 0.0))
    envelope = _envelope(values)
    delay = round(float(values.get("action_delay", 0.0)))
    dt = float(model.opt.timestep)
    decim = max(1, round(CONTROL_DT / dt))
    feet = g1_model.foot_geom_ids(model)
    sensor = int(mujoco.mjtObj.mjOBJ_SENSOR)  # type: ignore[attr-defined]  # absent from the bundled stubs
    gyro_adr = model.sensor_adr[
        mujoco.mj_name2id(model, sensor, g1_model.IMU_SITE + "-angular-velocity")
    ]
    base_p, base_r = base_pose(st, values)

    # Seed: the measured state, the same placement as loop 1's snap.
    s0 = measured_state(st, start, base_p=base_p, base_r=base_r)
    rng = np.random.default_rng(seed)
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.qpos[3:7] = mat_to_quat(s0.rot)
    data.qpos[7:] = s0.q
    data.qpos[7:19] += rng.normal(0.0, PERTURB_RAD, 12) if seed else 0.0
    data.qpos[2] = 0.8
    mujoco.mj_forward(model, data)
    data.qpos[2] -= float(np.min(data.geom_xpos[feet, 2])) - g1_model.FOOT_RADIUS
    data.qvel[6:] = s0.dq
    data.qvel[3:6] = s0.rot @ s0.gyro
    if s0.v_body is not None:
        data.qvel[0:3] = s0.rot @ s0.v_body
    mujoco.mj_forward(model, data)

    # The policy's history, teacher-forced from what the robot actually saw.
    policy = _policy()
    policy._first = True
    frames = []
    for k in range(OBS_HISTORY, 0, -1):
        t_abs = start - k * CONTROL_DT
        li = int(np.clip(np.searchsorted(st.lt, t_abs, "right") - 1, 0, len(st.lt) - 1))
        ci = int(np.clip(np.searchsorted(st.ct, t_abs, "right") - 1, 0, len(st.ct) - 1))
        frames.append(
            observation(
                _cmd_at(st, t_abs, cmd_dt),
                st.lgyro[li],
                st.lquat[li],
                st.lq[li],
                st.ldq[li],
                action_of_target(st.cq[ci][:NUM_ACTIONS]),
            )
        )
    ci = int(np.clip(np.searchsorted(st.ct, start, "right") - 1, 0, len(st.ct) - 1))
    policy.warm_start(frames, action_of_target(st.cq[ci][:NUM_ACTIONS]))

    ghost_id = -1
    if view:
        from dimos.simulation.sysid.engines.model import GHOST_BODY, mocap_index

        ghost_id = mocap_index(model, GHOST_BODY)
        j0 = int(np.searchsorted(st.vt, start))
        a_r = yaw_anchor(quat_to_mat(data.qpos[3:7].copy()), base_r[j0])
        a_p = data.qpos[0:3].copy() - a_r @ base_p[j0]

    target = DEFAULT_29[:NUM_ACTIONS].copy()
    pending: deque[NDArray[Any]] = deque([target] * delay, maxlen=delay) if delay else deque()
    desired = DEFAULT_29.astype(np.float64).copy()
    applied = np.zeros(model.nu)
    n = int(seconds / dt)
    ts = np.empty(n)
    pos = np.empty((n, 3))
    quat = np.empty((n, 4))
    viewer = None
    if view:
        from mujoco import viewer as mj_viewer

        viewer = mj_viewer.launch_passive(model, data)
        wall = time.perf_counter()
    try:
        for i in range(n):
            t = i * dt
            if i % decim == 0:
                fresh = policy.step(
                    _cmd_at(st, start + t, cmd_dt),
                    data.sensordata[gyro_adr : gyro_adr + 3],
                    data.qpos[3:7],
                    data.qpos[7:],
                    data.qvel[6:],
                )
                if delay:
                    pending.append(fresh)
                    target = pending[0]
                else:
                    target = fresh
            elif delay:
                pending.append(pending[-1])
                target = pending[0]
            desired[:NUM_ACTIONS] = target
            tau = np.clip(
                KP * (desired - data.qpos[7:]) - KD * data.qvel[6:], -TORQUE_LIMITS, TORQUE_LIMITS
            )
            applied = actuator_step(applied, tau, dt, tau_lag, dq=data.qvel[6:], envelope=envelope)
            data.ctrl[:] = applied
            mujoco.mj_step(model, data)
            ts[i] = t + dt
            pos[i] = data.qpos[0:3]
            quat[i] = data.qpos[3:7]
            if viewer is not None:
                j = int(np.clip(np.searchsorted(st.vt, start + t, "right") - 1, 0, len(st.vt) - 1))
                data.mocap_pos[ghost_id] = a_r @ base_p[j] + a_p
                data.mocap_quat[ghost_id] = mat_to_quat(a_r @ base_r[j])
                if not viewer.is_running():
                    return PolicyRun(ts[:i], pos[:i], quat[:i])
                viewer.sync()
                wall += dt / max(speed, 1e-6)
                lag = wall - time.perf_counter()
                if lag > 0:
                    time.sleep(lag)
                else:
                    wall = time.perf_counter()
    finally:
        if viewer is not None:
            viewer.close()
    return PolicyRun(ts, pos, quat)


def curves(
    run: PolicyRun, st: Streams, start: float, values: dict[str, float] | None = None
) -> dict[str, NDArray[Any]]:
    """Signed tracking error vs time: along/cross in the real heading frame, yaw."""
    base_p, base_r = base_pose(st, values or {})
    yaw_room = np.unwrap(yaw_of(np.stack([mat_to_quat(r) for r in base_r])))
    yaw_s = np.unwrap(yaw_of(run.quat))
    i0 = int(np.searchsorted(run.t, TRACKING_ANCHOR_S - 1e-9, "left"))
    a0 = float(run.t[i0]) + start
    a_yaw = float(yaw_s[i0]) - float(np.interp(a0, st.vt, yaw_room))
    c, s = np.cos(a_yaw), np.sin(a_yaw)
    rot2 = np.array([[c, -s], [s, c]])
    p0 = np.array([np.interp(a0, st.vt, base_p[:, 0]), np.interp(a0, st.vt, base_p[:, 1])])
    a_p = run.pos[i0][:2] - rot2 @ p0
    a = run.t[i0:] + start
    p_room = np.stack([np.interp(a, st.vt, base_p[:, 0]), np.interp(a, st.vt, base_p[:, 1])], 1)
    e = run.pos[i0:, :2] - (p_room @ rot2.T + a_p)
    yr = np.interp(a, st.vt, yaw_room)
    h = yr + a_yaw
    u = np.stack([np.cos(h), np.sin(h)], 1)
    nrm = np.stack([-np.sin(h), np.cos(h)], 1)
    _, roll_real = pitch_roll_of(np.stack([mat_to_quat(r) for r in base_r]))
    m = (st.vt >= start) & (st.vt <= start + float(run.t[-1]))
    _, roll_sim = pitch_roll_of(run.quat)
    f_real = sway_hz(st.vt[m], roll_real[m])
    f_sim = sway_hz(run.t, roll_sim)
    return {
        "along": np.sum(e * u, 1),
        "cross": np.sum(e * nrm, 1),
        "yaw": (yaw_s[i0:] - float(yaw_s[i0])) - (yr - float(np.interp(a0, st.vt, yaw_room))),
        "cadence": np.array([(f_sim - f_real) / f_real]),
    }


def sway_hz(t: NDArray[Any], roll: NDArray[Any]) -> float:
    """The roll spectrum's peak in 0.5-4 Hz: the lateral sway, one per stride pair."""
    x = roll - roll.mean()
    dt = float(np.median(np.diff(t)))
    f = np.fft.rfftfreq(len(x), dt)
    p = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2
    band = (f > 0.5) & (f < 4.0)
    return float(f[band][np.argmax(p[band])])


def areas(c: dict[str, NDArray[Any]]) -> dict[str, float]:
    return {k: float(np.mean(np.abs(v))) for k, v in c.items()}


def windows(st: Streams, n: int, seconds: float, *, seed: int = 0) -> list[float]:
    """``n`` start times across the commanded span, each window mostly walking.

    The walk segments are short (the longest is ~9 s), so a window may cross a
    stop and a restart, exactly as the robot did; a window is accepted when
    the policy was in walk for at least half of it.
    """
    rng = np.random.default_rng(seed)
    walks = [(a, b) for _, m, a, b in st.segments() if m == "walk"]
    if not walks:
        raise ValueError("no walk segment in this recording")
    lo, hi = walks[0][0], float(st.wt[-1]) - seconds
    out: list[float] = []
    for _ in range(10_000):
        t0 = float(rng.uniform(lo, hi))
        walking = sum(max(0.0, min(b, t0 + seconds) - max(a, t0)) for a, b in walks)
        if walking >= 0.5 * seconds:
            out.append(t0)
            if len(out) == n:
                break
    if len(out) < n:
        raise ValueError(f"only {len(out)} of {n} windows of {seconds:.0f} s are half walking")
    return sorted(out)


# ------------------------------------------------------------------ objective

_W: dict[str, Any] = {}


def _init(recording: str) -> None:
    _W["st"] = read_streams(recording)


def _eval_window(args: tuple[dict[str, float], float, float, int]) -> dict[str, float]:
    values, start, seconds, seed = args
    st = _W["st"]
    return areas(curves(rollout(st, values, start, seconds, seed=seed), st, start, values))


class Objective:
    """Mean over windows and terms of area / stock area."""

    def __init__(self, recording: str, starts: list[float], seconds: float, workers: int) -> None:
        self.starts, self.seconds = starts, seconds
        self._pool = concurrent.futures.ProcessPoolExecutor(
            max_workers=max(1, workers),
            mp_context=multiprocessing.get_context("spawn"),
            initializer=_init,
            initargs=(recording,),
        )
        self.scale: dict[str, float] | None = None

    def rollouts(self, values: dict[str, float], *, seed: int = 0) -> list[dict[str, float]]:
        jobs = [(values, s, self.seconds, seed) for s in self.starts]
        return list(self._pool.map(_eval_window, jobs))

    def calibrate(self, values: dict[str, float]) -> list[dict[str, float]]:
        base = self.rollouts(values)
        self.scale = {t: float(np.mean([b[t] for b in base])) for t in TERMS}
        self.scale["cadence"] = max(self.scale["cadence"], CADENCE_SCALE)
        return base

    def evaluate(self, values: dict[str, float]) -> float:
        assert self.scale is not None
        per = self.rollouts(values)
        return float(np.mean([[p[t] / self.scale[t] for t in TERMS] for p in per]))

    def close(self) -> None:
        self._pool.shutdown()


def table(name: str, per: list[dict[str, float]]) -> str:
    m = {t: float(np.mean([p[t] for p in per])) for t in TERMS}
    return (
        f"  {name:<10s} along {m['along']:.4f} m  cross {m['cross']:.4f} m  "
        f"yaw {m['yaw']:.4f} rad  cadence {100 * m['cadence']:.1f}%"
    )


def load_values(preset_arg: str) -> tuple[Preset, dict[str, float]]:
    """A preset's complete loop-2 values: plant knobs, plus the loop/rig knobs
    from the ``.loop.json`` beside its plant JSON (a built-in name looks in
    ``presets/``); defaults otherwise."""
    preset = load_preset(preset_arg)
    values = {
        **ENGINE_DEFAULTS,
        **LOOP_DEFAULTS,
        **preset.physics,
        "actuator_tau": preset.actuator_tau,
    }
    if preset_arg.endswith(".plant.json"):
        loop = Path(preset_arg[: -len(".plant.json")] + ".loop.json")
    else:
        loop = Path(__file__).resolve().parents[1] / "presets" / f"{preset_arg}.loop.json"
    if loop.is_file():
        values.update(json.loads(loop.read_text()))
    return preset, values


def main() -> None:
    ap = argparse.ArgumentParser(prog="g1.sim.sysid.ground", description=__doc__)
    ap.add_argument("recording")
    ap.add_argument("--preset", default="stock", help="plant to grade (name or .plant.json)")
    ap.add_argument("--windows", type=int, default=6)
    ap.add_argument(
        "--seconds",
        type=float,
        default=None,
        help="window length: 15 s for grading and fitting; --view runs to the end of the recording",
    )
    ap.add_argument("--seed", type=int, default=0, help="window placement seed")
    ap.add_argument(
        "--replicates", type=int, default=1, help="perturbed repeats for the chaos floor"
    )
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument(
        "--fit", type=int, default=0, metavar="TRIALS", help="search the knobs on this loss"
    )
    ap.add_argument("--studies", type=int, default=3)
    ap.add_argument("--search", default=",".join(DEFAULT_SEARCH), help="plant, loop and rig knobs")
    ap.add_argument(
        "--out", default=None, help="prefix: writes .plant.json, .loop.json, .trials.json"
    )
    ap.add_argument("--name", default="candidate")
    ap.add_argument("--view", action="store_true", help="watch from --start")
    ap.add_argument("--start", type=float, default=None)
    ap.add_argument("--speed", type=float, default=1.0)
    args = ap.parse_args()

    st = read_streams(args.recording)
    preset, values = load_values(args.preset)

    if args.view:
        start = args.start if args.start is not None else float(st.segments()[1][2])
        seconds = args.seconds if args.seconds is not None else float(st.wt[-1]) - start
        run = rollout(st, values, start, seconds, view=True, speed=args.speed)
        print(table(preset.name, [areas(curves(run, st, start, values))]))
        return

    seconds = 15.0 if args.seconds is None else args.seconds
    starts = windows(st, args.windows, seconds, seed=args.seed)
    print(f"{len(starts)} windows of {seconds:.0f} s at " + ", ".join(f"{s:.1f}" for s in starts))
    obj = Objective(args.recording, starts, seconds, args.workers)
    try:
        stock = {**ENGINE_DEFAULTS, **LOOP_DEFAULTS, "actuator_tau": 0.0}
        base = obj.calibrate(stock)
        print("TRACKING AREAS  mean |error| over the horizon, mean over windows")
        print(table("stock", base))
        if preset.name != "stock":
            print(table(preset.name, obj.rollouts(values)))
        if args.replicates > 1:
            reps = [obj.rollouts(values, seed=k) for k in range(1, args.replicates + 1)]
            floor = {}
            for t in TERMS:
                pairs = [
                    abs(a[t] - b[t])
                    for i, ra in enumerate(reps)
                    for rb in reps[i + 1 :]
                    for a, b in zip(ra, rb, strict=True)
                ]
                floor[t] = float(np.median(pairs)) / np.sqrt(2)
            print(
                "  chaos floor (median pairwise sim-vs-sim / sqrt 2): "
                + "  ".join(f"{t} {floor[t]:.4f}" for t in TERMS)
            )

        if args.fit:
            import optuna

            optuna.logging.set_verbosity(optuna.logging.WARNING)
            names = tuple(args.search.split(","))
            unknown = [n for n in names if n not in ALL_KNOBS]
            if unknown:
                raise KeyError(
                    f"unknown knob(s) {unknown}; plant {sorted(KNOBS)} loop {sorted(LOOP_KNOBS)}"
                )
            searched = {n: ALL_KNOBS[n] for n in names}
            best: list[tuple[float, dict[str, float]]] = []
            for seed in range(args.studies):

                def fn(trial: optuna.Trial) -> float:
                    params = {
                        n: trial.suggest_float(n, k.lo, k.hi, log=k.log)
                        for n, k in searched.items()
                    }
                    loss = obj.evaluate({**values, **params})
                    best.append((loss, params))
                    return loss

                study = optuna.create_study(
                    direction="minimize",
                    sampler=optuna.samplers.CmaEsSampler(
                        seed=seed, independent_sampler=optuna.samplers.RandomSampler(seed=seed)
                    ),
                )
                study.enqueue_trial(
                    {n: float(min(k.hi, max(k.lo, values[n]))) for n, k in searched.items()}
                )
                study.optimize(fn, n_trials=args.fit)
                print(f"  study {seed}: best {study.best_value:.4f}")
            best.sort(key=lambda r: r[0])
            top = best[: max(3, len(best) // 10)]
            point = {n: float(np.median([p[n] for _, p in top])) for n in names}
            final = {**values, **point}
            print(
                f"\nLOOP-2 FIT  stock 1.0000 -> point {obj.evaluate(final):.4f}"
                f"  (median of the top {len(top)} of {len(best)} trials)"
            )
            for n in names:
                lo, hi = min(p[n] for _, p in top), max(p[n] for _, p in top)
                kind = "rig " if n.startswith("rig_") else ("loop" if n in LOOP_KNOBS else "plant")
                print(f"  {kind} {n:<20s} {point[n]:.6g}   top spread {lo:.4g} .. {hi:.4g}")
            print(table("loop2", obj.rollouts(final)))
            if args.out:
                phys = {k: v for k, v in final.items() if k in PHYSICS_KEYS}
                out = Path(args.out + ".plant.json")
                Preset(
                    name=args.name,
                    physics=phys,
                    actuator_tau=float(final["actuator_tau"]),
                    provenance={
                        n: "fitted on loop 2 (tracking areas vs Point-LIO)"
                        for n in names
                        if n in phys
                    },
                ).save(out)
                Path(args.out + ".loop.json").write_text(
                    json.dumps({k: final[k] for k in LOOP_KNOBS}, indent=2) + "\n"
                )
                Path(args.out + ".trials.json").write_text(
                    json.dumps([{"loss": loss, **p} for loss, p in best], indent=1) + "\n"
                )
                print(f"wrote {out}, .loop.json, .trials.json")
    finally:
        obj.close()


if __name__ == "__main__":
    main()
