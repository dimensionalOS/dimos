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

"""Mode A: open-loop plant replay. Recorded joint targets in, residuals out.

Replaying a recording *through a policy* only identifies the plant if the
recorded and simulated policies are the same net; replaying the policy's
OUTPUT (the joint targets it actually sent) removes the policy from the loop,
so every span is plant-excitation data whatever was driving. The cost is
feedback — the replay diverges — hence multiple shooting with short clips
(:func:`~dimos.simulation.sysid.regimes.clip_schedule`).

Everything here is simulator-agnostic: it builds a :class:`RolloutPlan` from
measured streams, hands it to a :class:`Backend`, and differences the
:class:`Prediction` against the recording where the two overlap.
"""

from __future__ import annotations

from dataclasses import dataclass, field as dataclasses_field

import numpy as np

from dimos.robot.unitree.go2.sim.ranges import Preset
from dimos.robot.unitree.go2.sim.sysid.ingest import TRACKER_Z, mount_matrix
from dimos.simulation.sysid.backend import (
    Backend,
    BaseCondition,
    BaseTrack,
    Commands,
    GhostTrack,
    Prediction,
    RolloutPlan,
    State,
)
from dimos.simulation.sysid.recording import Streams
from dimos.simulation.sysid.regimes import clip_schedule
from dimos.simulation.sysid.rotations import quat_to_mat, rotation_angle, strip_yaw


def _step_quantized(t0: float, duration: float, schedule: np.ndarray, dt: float) -> list[float]:
    """Each scheduled snap moved to the first step time at or past it.

    Mirrors the stepping loop exactly, including its one-snap-per-step rule,
    so a plan built here fires bit-identically inside the backend.
    """
    out: list[float] = []
    si = 0
    for step in range(int(duration / dt)):
        t = t0 + step * dt
        if si < len(schedule) and t >= schedule[si]:
            out.append(t)
            si += 1
    return out


def measured_state(
    st: Streams,
    t: float,
    *,
    base_p: np.ndarray | None = None,
    base_r: np.ndarray | None = None,
) -> State:
    """The robot's MEASURED state at ``t``, ready for a backend to snap to.

    Joints from ``rt/lowstate``; attitude from the IMU, yaw-stripped. Base
    linear velocity needs the tracker (central difference on the room-frame
    base track); without one the clip starts at rest.
    """
    i = int(np.clip(int(np.searchsorted(st.lt, t, "right")) - 1, 0, len(st.lt) - 1))
    rot = strip_yaw(quat_to_mat(st.lquat[i]))
    v_body: np.ndarray | None = None
    if st.has_markers and base_p is not None and base_r is not None:
        j = int(np.clip(np.searchsorted(st.vt, t, "right") - 1, 1, len(st.vt) - 2))
        dt = st.vt[j + 1] - st.vt[j - 1]
        v_room = (base_p[j + 1] - base_p[j - 1]) / max(dt, 1e-6)
        v_body = base_r[j].T @ v_room  # room -> body
    return State(t=t, q=st.lq[i], dq=st.ldq[i], rot=rot, gyro=st.lgyro[i], v_body=v_body)


def base_track(st: Streams, t0: float, duration: float) -> BaseTrack:
    """The measured trunk attitude over ``[t0, t0+duration]``, for a PINNED base.

    Raw IMU attitude, NOT yaw-stripped: the backend anchors the track to the
    (yaw-stripped) snap pose at each re-initialisation, which cancels the
    arbitrary yaw as a constant world rotation while keeping every measured
    attitude increment — including real yaw motion, which per-sample stripping
    would corrupt near the 70-85 deg hang. One sample of margin either side so
    zero-order-hold sampling inside the backend never runs off the end.
    """
    i0 = max(int(np.searchsorted(st.lt, t0, "right")) - 1, 0)
    i1 = min(int(np.searchsorted(st.lt, t0 + duration, "right")) + 1, len(st.lt))
    rots = np.stack([quat_to_mat(q) for q in st.lquat[i0:i1]])
    return BaseTrack(t=st.lt[i0:i1].copy(), rot=rots)


def build_plan(
    st: Streams,
    t0: float,
    duration: float,
    *,
    schedule: np.ndarray,
    dt: float,
    suspended: bool = False,
    mount: np.ndarray | None = None,
) -> RolloutPlan:
    """A fully determined rollout: commands, reinit states, base condition.

    ``schedule`` are the mid-run snap times (absolute, from
    :func:`~dimos.simulation.sysid.regimes.clip_schedule`); the
    state at ``t0`` is always prepended.

    Snap times are quantized to the backend's step grid (``dt``) FIRST, and
    the measured state is sampled at the quantized time: a snap can only fire
    on a step boundary, and sampling at the scheduled time instead can land
    one 500 Hz sample off — a silent nondeterminism between backends with
    different steps.
    """
    mount = mount_matrix() if mount is None else mount
    base_p = base_r = None
    if st.has_markers:
        base_p, base_r = st.base_pose_room(mount, TRACKER_Z)
    times = _step_quantized(t0, duration, schedule, dt)
    reinit = [measured_state(st, t, base_p=base_p, base_r=base_r) for t in [t0, *times]]
    return RolloutPlan(
        t0=t0,
        duration=duration,
        commands=Commands(t=st.ct, q=st.cq, dq=st.cdq, kp=st.ckp, kd=st.ckd, tau_ff=st.ctau),
        reinit=reinit,
        base=BaseCondition.PINNED if suspended else BaseCondition.FREE,
        base_track=base_track(st, t0, duration) if suspended else None,
    )


@dataclass
class ReplayResult:
    """Sim prediction and recording, aligned on the prediction's time bases."""

    prediction: Prediction
    q_real: np.ndarray  # (n,12) at prediction.t
    p_real: np.ndarray | None  # (n,3) recorded base track in the SIM world frame
    r_real: np.ndarray | None  # (n,3,3)
    a_real: np.ndarray  # (k,3) at prediction.at
    w_real: np.ndarray  # (k,3)
    tau_real: np.ndarray  # (k,12)
    dq_real: np.ndarray = dataclasses_field(default_factory=lambda: np.zeros((0, 12)))
    # (n,12) at prediction.t — measured joint speeds, the `dq` channel

    def joint_err(self) -> np.ndarray:
        out: np.ndarray = np.abs(self.prediction.q - self.q_real)
        return out

    def body_err(self) -> tuple[np.ndarray, np.ndarray]:
        """Planar position error (m) and orientation error (rad) per sample."""
        if self.p_real is None or self.r_real is None:
            n = len(self.prediction.t)
            return np.zeros(n), np.zeros(n)
        dp = np.linalg.norm(self.prediction.body_pos[:, :2] - self.p_real[:, :2], axis=1)
        return dp, rotation_angle(self.prediction.body_rot, self.r_real)


def _real_track_in_sim_world(
    pred: Prediction, st: Streams, base_p: np.ndarray, base_r: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """The recorded base track mapped into the rollout's world frame.

    Open-loop drift is only defined relative to the pose a clip started from,
    so the rigid room->sim anchor is recomputed at EVERY re-initialisation
    from the sim pose the backend reported — a fixed anchor reads the snap
    itself as an 81 deg orientation error that is pure bookkeeping.
    """
    n = len(pred.t)
    p_real = np.empty((n, 3))
    r_real = np.empty((n, 3, 3))
    # anchors per reinit
    anchors_r = []
    anchors_p = []
    for r in range(len(pred.reinit_t)):
        j = int(np.clip(np.searchsorted(st.vt, pred.reinit_t[r], "right") - 1, 0, len(st.vt) - 1))
        a_r = pred.reinit_rot[r] @ base_r[j].T
        a_p = pred.reinit_pos[r] - a_r @ base_p[j]
        anchors_r.append(a_r)
        anchors_p.append(a_p)
    for i, t in enumerate(pred.t):
        r = int(np.clip(np.searchsorted(pred.reinit_t, t, "right") - 1, 0, len(pred.reinit_t) - 1))
        j = int(np.clip(np.searchsorted(st.vt, t, "right") - 1, 0, len(st.vt) - 1))
        p_real[i] = anchors_r[r] @ base_p[j] + anchors_p[r]
        r_real[i] = anchors_r[r] @ base_r[j]
    return p_real, r_real


def score(pred: Prediction, st: Streams, *, mount: np.ndarray | None = None) -> ReplayResult:
    """Difference the prediction against the recording on their intersection.

    A recording without a tracker simply has no body channels — no separate
    code path, no "VR mode": adding a tracker adds a channel and nothing else
    changes.
    """
    mount = mount_matrix() if mount is None else mount
    idx = np.clip(np.searchsorted(st.lt, pred.t, "right") - 1, 0, len(st.lt) - 1)
    q_real = st.lq[idx]
    aidx = np.clip(np.searchsorted(st.lt, pred.at, "right") - 1, 0, len(st.lt) - 1)
    p_real = r_real = None
    if st.has_markers:
        base_p, base_r = st.base_pose_room(mount, TRACKER_Z)
        p_real, r_real = _real_track_in_sim_world(pred, st, base_p, base_r)
    return ReplayResult(
        prediction=pred,
        q_real=q_real,
        p_real=p_real,
        r_real=r_real,
        a_real=st.lacc[aidx] if len(st.lacc) else np.zeros((len(pred.at), 3)),
        w_real=st.lgyro[aidx] if len(st.lgyro) else np.zeros((len(pred.at), 3)),
        tau_real=st.ltau[aidx] if len(st.ltau) else np.zeros((len(pred.at), 12)),
        dq_real=st.ldq[idx] if len(st.ldq) else np.zeros((len(pred.t), 12)),
    )


def ghost_track(st: Streams, *, mount: np.ndarray | None = None) -> GhostTrack | None:
    """The recorded base pose for a viewer's ghost; ``None`` without a tracker.

    Where no tracker exists the ghost is ABSENT rather than faked — an IMU
    dead-reckoned stand-in would draw a confident picture of something never
    measured.
    """
    if not st.has_markers:
        return None
    mount = mount_matrix() if mount is None else mount
    base_p, base_r = st.base_pose_room(mount, TRACKER_Z)
    return GhostTrack(t=st.vt, pos=base_p, rot=base_r)


def replay(
    st: Streams,
    t0: float,
    duration: float,
    backend: Backend,
    *,
    preset: Preset | None = None,
    window: float | tuple[float, float] | None = 0.4,
    seed: int = 0,
    protect: np.ndarray | None = None,
    suspended: bool = False,
    mount: np.ndarray | None = None,
) -> ReplayResult:
    """THE Mode-A code path: schedule, plan, rollout, score.

    ``preset`` applies a named plant to the backend first; ``None`` leaves
    whatever the caller last applied. Everything that varies between candidate
    plants is in the backend; everything here is a pure function of the
    recording and ``(window, seed, protect)``, so every candidate is scored on
    identical clips.
    """
    if preset is not None:
        backend.apply({**preset.physics, "actuator_tau": preset.actuator_tau})
    sched = clip_schedule(t0, duration, window, seed=seed, protect=protect)
    plan = build_plan(
        st, t0, duration, schedule=sched, dt=backend.timestep, suspended=suspended, mount=mount
    )
    return score(backend.rollout(plan), st, mount=mount)


def _window_arg(vals: list[float]) -> float | tuple[float, float] | None:
    if len(vals) == 1:
        return None if vals[0] <= 0 else float(vals[0])
    return (float(vals[0]), float(vals[1]))


def main() -> None:
    """Watch (or just score) a Mode-A replay.

        python -m dimos.robot.unitree.go2.sim.sysid.replay REC.mcap --view
        python -m ... REC.mcap --view --no-reinit --speed 0.25   # free divergence
        python -m ... REC.mcap                                   # numbers only

    The viewer and the headless run are the same function: ``--view`` only
    attaches a viewer to the backend and paces to wall clock. The ghost is the
    recorded tracker pose where a tracker exists, absent where none does. A
    suspended recording shows the held, swinging trunk, because the viewer
    watches the same weld the physics imposes.
    """
    import argparse

    ap = argparse.ArgumentParser(prog="go2.sim.sysid.replay", description=main.__doc__)
    ap.add_argument("recording")
    ap.add_argument("--preset", default=None, help="plant preset name or JSON path")
    ap.add_argument("--segment", type=int, default=None, help="policy-mode segment; --list first")
    ap.add_argument("--list", action="store_true", help="list policy segments and exit")
    ap.add_argument("--start", type=float, default=None)
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument(
        "--window",
        type=float,
        nargs="+",
        default=[0.05, 0.8],
        metavar="S",
        help="clip length, s: one value = fixed (0 = never re-initialise), two = U(lo, hi)",
    )
    ap.add_argument("--seed", type=int, default=0, help="clip-schedule seed")
    ap.add_argument(
        "--no-reinit",
        action="store_true",
        help="free open-loop divergence — the only way to SEE what open loop costs",
    )
    ap.add_argument(
        "--engine",
        choices=("mujoco", "mjx"),
        default="mujoco",
        help="which backend drives the plan; the plants are the same model, so a "
        "difference between them is the SOLVER (mjx: FREE base only, no viewer)",
    )
    ap.add_argument(
        "--f32",
        action="store_true",
        help="mjx only: run the TRAINING dtype instead of the plant's float64 — "
        "the cost of fp32 is a measurement, not an argument",
    )
    ap.add_argument("--view", action="store_true", help="attach the MuJoCo viewer")
    ap.add_argument("--speed", type=float, default=1.0, help="viewer playback rate")
    ap.add_argument("--no-ghost", action="store_true", help="hide the recorded-pose ghost")
    args = ap.parse_args()

    from dimos.robot.unitree.go2.sim.ranges import load_preset
    from dimos.robot.unitree.go2.sim.sysid.ingest import read_streams
    from dimos.simulation.sysid.recording import read_declarations
    from dimos.simulation.sysid.regimes import protected, regimes

    st = read_streams(args.recording)
    if args.list or args.segment is not None:
        print(f"{'idx':>4s} {'policy mode':>12s} {'start':>8s} {'end':>8s} {'dur':>7s}")
        for i, mode, a, b in st.segments():
            print(f"{i:4d} {mode:>12s} {a:8.2f} {b:8.2f} {b - a:7.2f}")
        if args.list:
            return

    declared = read_declarations(args.recording)
    suspended = bool(declared.suspended)
    spans = regimes(st, declared)
    if args.segment is not None:
        _i, mode, a, b = st.segments()[args.segment]
        t0 = a + 0.2 if args.start is None else args.start
        dur = (b - t0 - 0.2) if args.seconds is None else args.seconds
        print(f"replaying segment {args.segment} ({mode}) from {t0:.2f}s for {dur:.2f}s")
    else:
        t0 = max(float(st.lt[0]), float(st.ct[0])) + 0.5 if args.start is None else args.start
        dur = 10.0 if args.seconds is None else args.seconds

    window = None if args.no_reinit else _window_arg(args.window)
    ghost = None if args.no_ghost else ghost_track(st)
    preset = load_preset(args.preset)
    # The backend is built WITH the preset's envelope: a plant that carries one
    # must never silently run bare (the two are one claim — see ranges.Preset).
    from dimos.robot.unitree.go2.sim.plant import TORQUE_ENVELOPES

    envelope = TORQUE_ENVELOPES[preset.envelope] if preset.envelope else None
    if args.engine == "mjx":
        if args.view:
            ap.error("--view is a MuJoCo viewer; mjx has no CPU MjData to attach it to")
        if suspended:
            ap.error("mjx implements BaseCondition.FREE only; this recording is suspended")
        from dimos.robot.unitree.go2.sim.engines.mjx import MjxBackend

        backend: Backend = MjxBackend(envelope=envelope, x64=not args.f32)
    else:
        if args.f32:
            ap.error("--f32 is an mjx dtype choice; the CPU engine is float64 only")
        from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend

        backend = MujocoBackend(
            view=args.view,
            view_speed=args.speed,
            ghost=ghost,
            envelope=envelope,
        )
    print(
        f"engine {backend.name} | preset {preset.name} | re-init "
        f"{'OFF (free divergence)' if window is None else window} | "
        f"ghost {'tracker' if ghost is not None and args.view else 'absent'}"
        f"{' | SUSPENDED (trunk welded to the measured attitude)' if suspended else ''}"
    )
    r = replay(
        st,
        t0,
        dur,
        backend,
        preset=preset,
        window=window,
        seed=args.seed,
        protect=protected(spans),
        suspended=suspended,
    )
    je = r.joint_err()
    print(
        f"\n  joint |error|  mean {je.mean():.4f}  p50 {np.percentile(je, 50):.4f}"
        f"  p90 {np.percentile(je, 90):.4f}  max {je.max():.4f} rad"
    )
    if r.p_real is not None:
        dp, da = r.body_err()
        print(f"  body xy error  mean {dp.mean():.4f}  p90 {np.percentile(dp, 90):.4f} m")
        print(f"  body rot error mean {np.degrees(da.mean()):.2f} deg")
    print(f"  re-initialisations: {len(r.prediction.reinit_t) - 1}")


if __name__ == "__main__":
    main()
