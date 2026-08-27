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

This is fitting THROUGH the controller, which go2sim measured to anti-transfer
on the Go2 twice; it is used here by decision, with that on record.

    python -m dimos.robot.unitree.g1.sim.sysid.ground REC.db --preset measured
    python -m dimos.robot.unitree.g1.sim.sysid.ground REC.db --fit 40 --studies 3 --workers 8 --out presets/loop2
    python -m dimos.robot.unitree.g1.sim.sysid.ground REC.db --preset measured --view --start 30
"""

from __future__ import annotations

import argparse
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
from dimos.robot.unitree.g1.sim.ranges import ENGINE_DEFAULTS, KNOBS, load_preset
from dimos.robot.unitree.g1.sim.sysid.ingest import read_streams
from dimos.simulation.sysid.plant import actuator_step
from dimos.simulation.sysid.presets import Preset
from dimos.simulation.sysid.recording import Streams
from dimos.simulation.sysid.replay import measured_state
from dimos.simulation.sysid.rotations import mat_to_quat, quat_to_mat, yaw_of
from dimos.utils.data import LfsPath

TERMS = ("along", "cross", "yaw")
PERTURB_RAD = 0.005  # sim-perturb replicates: +-0.3 deg on the 12 seeded leg joints
TRACKING_ANCHOR_S = 0.5  # where the free rollout is anchored to the room
DEFAULT_SEARCH = (
    "armature",
    "frictionloss",
    "actuator_tau",
    "foot_solref_time",
    "foot_solref_damp",
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


def _cmd_at(st: Streams, t_abs: float) -> NDArray[Any]:
    i = int(np.searchsorted(st.wt, t_abs, "right")) - 1
    return st.wcmd[i] if i >= 0 else np.zeros(3)


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
    physics = {k: v for k, v in values.items() if k != "actuator_tau"}
    g1_model.apply_physics(model, physics)
    tau_lag = float(values.get("actuator_tau", 0.0))
    dt = float(model.opt.timestep)
    decim = max(1, round(CONTROL_DT / dt))
    feet = g1_model.foot_geom_ids(model)
    sensor = int(mujoco.mjtObj.mjOBJ_SENSOR)  # type: ignore[attr-defined]  # absent from the bundled stubs
    gyro_adr = model.sensor_adr[
        mujoco.mj_name2id(model, sensor, g1_model.IMU_SITE + "-angular-velocity")
    ]
    base_p, base_r = st.base_pose_room()

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
                _cmd_at(st, t_abs),
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
        a_r = quat_to_mat(data.qpos[3:7].copy()) @ base_r[int(np.searchsorted(st.vt, start))].T
        a_p = data.qpos[0:3].copy() - a_r @ base_p[int(np.searchsorted(st.vt, start))]

    target = DEFAULT_29[:NUM_ACTIONS].copy()
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
                target = policy.step(
                    _cmd_at(st, start + t),
                    data.sensordata[gyro_adr : gyro_adr + 3],
                    data.qpos[3:7],
                    data.qpos[7:],
                    data.qvel[6:],
                )
            desired[:NUM_ACTIONS] = target
            tau = np.clip(
                KP * (desired - data.qpos[7:]) - KD * data.qvel[6:], -TORQUE_LIMITS, TORQUE_LIMITS
            )
            applied = actuator_step(applied, tau, dt, tau_lag, dq=data.qvel[6:])
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


def curves(run: PolicyRun, st: Streams, start: float) -> dict[str, NDArray[Any]]:
    """Signed tracking error vs time: along/cross in the real heading frame, yaw."""
    base_p, base_r = st.base_pose_room()
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
    return {
        "along": np.sum(e * u, 1),
        "cross": np.sum(e * nrm, 1),
        "yaw": (yaw_s[i0:] - float(yaw_s[i0])) - (yr - float(np.interp(a0, st.vt, yaw_room))),
    }


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
    return areas(curves(rollout(st, values, start, seconds, seed=seed), st, start))


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
        f"  {name:<10s} along {m['along']:.4f} m  cross {m['cross']:.4f} m  yaw {m['yaw']:.4f} rad"
    )


def main() -> None:
    ap = argparse.ArgumentParser(prog="g1.sim.sysid.ground", description=__doc__)
    ap.add_argument("recording")
    ap.add_argument("--preset", default="stock", help="plant to grade (name or JSON)")
    ap.add_argument("--windows", type=int, default=6)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0, help="window placement seed")
    ap.add_argument(
        "--replicates", type=int, default=1, help="perturbed repeats for the chaos floor"
    )
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument(
        "--fit", type=int, default=0, metavar="TRIALS", help="search the knobs on this loss"
    )
    ap.add_argument("--studies", type=int, default=3)
    ap.add_argument("--search", default=",".join(DEFAULT_SEARCH))
    ap.add_argument("--out", default=None, help="prefix for .plant.json")
    ap.add_argument("--name", default="loop2")
    ap.add_argument("--view", action="store_true", help="watch one window from --start")
    ap.add_argument("--start", type=float, default=None)
    ap.add_argument("--speed", type=float, default=1.0)
    args = ap.parse_args()

    st = read_streams(args.recording)
    preset = load_preset(args.preset)
    values = {**ENGINE_DEFAULTS, **preset.physics, "actuator_tau": preset.actuator_tau}

    if args.view:
        start = (
            args.start
            if args.start is not None
            else windows(st, 1, args.seconds, seed=args.seed)[0]
        )
        run = rollout(st, values, start, args.seconds, view=True, speed=args.speed)
        print(table(preset.name, [areas(curves(run, st, start))]))
        return

    starts = windows(st, args.windows, args.seconds, seed=args.seed)
    print(
        f"{len(starts)} windows of {args.seconds:.0f} s at " + ", ".join(f"{s:.1f}" for s in starts)
    )
    obj = Objective(args.recording, starts, args.seconds, args.workers)
    try:
        stock = {**ENGINE_DEFAULTS, "actuator_tau": 0.0}
        base = obj.calibrate(stock)
        print("TRACKING AREAS  mean |error| over the horizon, mean over windows")
        print(table("stock", base))
        if preset.name != "stock":
            print(table(preset.name, obj.rollouts(values)))
        if args.replicates > 1:
            reps = [obj.rollouts(values, seed=k) for k in range(1, args.replicates + 1)]
            floor = {
                t: float(
                    np.median(
                        [
                            abs(a[t] - b[t])
                            for i, ra in enumerate(reps)
                            for rb in reps[i + 1 :]
                            for a, b in zip(ra, rb, strict=True)
                        ]
                    )
                )
                / np.sqrt(2)
                for t in TERMS
            }
            print(
                "  chaos floor (median pairwise sim-vs-sim / sqrt 2): "
                + "  ".join(f"{t} {floor[t]:.4f}" for t in TERMS)
            )

        if args.fit:
            import optuna

            optuna.logging.set_verbosity(optuna.logging.WARNING)
            names = tuple(args.search.split(","))
            searched = {n: KNOBS[n] for n in names}
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
                print(f"  study {seed}: best {study.best_value:.4f}  {study.best_params}")
            best.sort(key=lambda r: r[0])
            top = best[: max(3, len(best) // 10)]
            point = {n: float(np.median([p[n] for _, p in top])) for n in names}
            print(
                f"\nLOOP-2 FIT  stock 1.0000 -> point {obj.evaluate({**values, **point}):.4f}  (median of the top {len(top)} of {len(best)} trials)"
            )
            for n in names:
                lo, hi = min(p[n] for _, p in top), max(p[n] for _, p in top)
                print(f"  {n:<20s} {point[n]:.6g}   top spread {lo:.4g} .. {hi:.4g}")
            print(table("loop2", obj.rollouts({**values, **point})))
            if args.out:
                phys = {**values, **point}
                tau = phys.pop("actuator_tau")
                out = Path(args.out + ".plant.json")
                Preset(
                    name=args.name,
                    physics=phys,
                    actuator_tau=tau,
                    provenance={n: "fitted on loop 2 (tracking areas vs Point-LIO)" for n in names},
                ).save(out)
                out.with_suffix(".trials.json").write_text(
                    json.dumps([{"loss": l, **p} for l, p in best], indent=1)
                )
                print(f"wrote {out}")
    finally:
        obj.close()


if __name__ == "__main__":
    main()
