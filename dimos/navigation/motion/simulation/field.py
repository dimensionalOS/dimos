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

"""Score the fitted sim against a field navigation run instead of a VR session.

    python -m dimos.navigation.motion.simulation.field \\
        ml-trajectory-research/20260806-063428.zenoh.mcap \\
        --policy data/ml-trajectory-research/freewalk_mcf.bin --fitted

A field run arrives as a PAIR of recordings on the robot's clock: the zenoh
side carries what the nav stack commanded (``cmd_vel``, 11 Hz) and where
Point-LIO says the body went (``odometry``, 30 Hz, stamped on the *sensor*
frame and resolved to ``base_link`` through the recorded tf); the sidecar
carries ``sportmodestate`` (300 Hz on-board body-velocity estimate) and
``lowstate`` (500 Hz real joint angles, velocities and torques -- but only once
the LowState CDR frame is right; :func:`joint_fault` is the guard). There is no
``control_log`` with velocities and no ``vive_pose``, so :mod:`evaluate` cannot
read one of these.

The LIO stream is logged when the recorder *received* it, which is later than
the pose it describes. Two ways to recover the difference, and the loader
sniffs which one the recording supports (:func:`diagnose.stamp_dialect`): a
recording whose odometry speaks sensor time carries the answer in its own
stamps, so the track is timed by them directly; an older one has the stamps on
the lidar's boot clock, and :func:`clock_offset` recovers the lag instead by
cross-correlating LIO-derived speed against the robot's own estimate. Where
both are available they cross-check each other (:data:`LAG_DISAGREE_S`), and
the correlation wins a disagreement because it is tied to physical motion.
Skipping this is not cosmetic: on the 063428 pair it is 170 ms, and during a
1.3 rad/s spin an uncorrected LIO track puts the body a quarter of a metre and
0.2 rad from where it was, which reads as the robot failing to hold its
commanded path.

Accumulated sim-vs-real divergence is not the headline here -- a legged gait
decorrelates within seconds no matter how good the physics is (``FINDINGS.md``).
What survives is *tracking error*: how far each body -- simulated and real --
misses the twist it was told to hold. Binned by how aggressive that twist was
(:data:`REGIME_BINS`), it answers the only question a refit decision needs: does
the fitted sim miss the same way reality misses, in the regime that hurt?
"""

from __future__ import annotations

from dataclasses import dataclass, field as dc_field
import math
from pathlib import Path
from typing import Any

import numpy as np

from dimos.navigation.motion.adapter.diagnose import (
    ODOMETRY,
    TF,
    Dialect,
    Instant,
    Window,
    open_lcm_mcap,
    parse_instant,
    payload_ts,
    stamp_dialect,
)
from dimos.navigation.motion.simulation import metrics, model as go2_model, walk as walk_mod
from dimos.navigation.motion.simulation.policy import FreePolicy
from dimos.navigation.tf_pose import OdomBasePose

# The nav stack's output as the zenoh recorder slugs it. This is the twist the
# on-board mcf policy consumed, not the follower's pre-arbitration request
# (`nav_cmd_vel`) and not the operator's override (`tele_cmd_vel`).
CMD_VEL = "dimos_cmd_vel_geometry_msgs.Twist"

# Maneuver regimes, by the magnitude of the commanded axis. Upper bounds, so a
# sample lands in the first band it fits. Constants, not configuration: the
# point of the bins is that two runs compare band for band.
REGIME_BINS: tuple[tuple[str, float], ...] = (
    ("gentle", 0.4),
    ("moderate", 0.9),
    ("aggressive", math.inf),
)

# |cmd| below this on every axis is a stand command, scored on its own line.
STANDING = 1e-3

# Longest LIO pipeline lag :func:`clock_offset` will look for, seconds. The
# streams are on one clock by construction; anything past this is a broken pair
# rather than a latency to correct.
MAX_CLOCK_LAG_S = 1.0

# Reported alignment is only corrected past this. Below it the correction is
# inside the resample grid's own resolution.
LAG_TOLERANCE_S = 0.05

# How far the stamp-derived lag and the cross-correlated one may disagree before
# one of them is wrong rather than merely noisy. Past it both get printed and the
# correlation is used: it is measured against the robot's own motion, while a
# stamp is only as honest as whatever wrote it.
LAG_DISAGREE_S = 0.030

# Fraction of recorded joint angles allowed outside the model's own joint
# travel before the stream is called junk rather than mis-mapped.
OUT_OF_RANGE_MAX = 0.02

# Base height the real and simulated tracks are both anchored to. Only the
# anchor matters -- LIO z and MuJoCo z live in unrelated frames, so z is
# reported as drift-from-anchor and never compared in absolute terms.
ANCHOR_HEIGHT = 0.32

# Sim-over-real tracking-error ratio that calls for a refit. A ratio of 1 is a
# sim that misses its commands exactly as much as the robot does; the gentle
# and moderate bands sit near 1.15 on a run the preset was never fitted to, so
# 2 is "different in kind" rather than "different in detail".
REFIT_RATIO = 2.0

# Simulated base height below which the robot is on the floor, not walking.
# A standing Go2 holds ~0.32 m; the trunk is 0.114 m tall, so 0.15 m is well
# past any crouch the policy commands.
FALLEN_Z = 0.15

GRID_HZ = 50.0


# ------------------------------------------------------------------ loading --


def _resolve(path: str | Path) -> Path:
    """A recording on disk, or a name inside the LFS data archive."""
    from dimos.utils.data import get_data

    p = Path(path)
    return p if p.is_file() else Path(get_data(str(path)))


def sidecar_for(zenoh: str | Path) -> Path:
    """The sidecar recording that pairs with a ``*.zenoh.mcap``."""
    p = Path(zenoh)
    name = p.name
    if not name.endswith(".zenoh.mcap"):
        raise ValueError(f"{zenoh}: not a *.zenoh.mcap, pass --sidecar explicitly")
    return p.with_name(name[: -len(".zenoh.mcap")] + ".mcap")


@dataclass
class FieldPair:
    """One field run: commands, LIO ground truth, and the sidecar's joints.

    Times are seconds from the first ``cmd_vel`` message, the same "first
    command" epoch :func:`walk.read_control_log` uses. Streams stay WHOLE
    however ``window`` is set -- a rollout has to start where the robot was
    standing, so trimming the inputs would simulate a run that never happened.
    The window only decides what gets reported.
    """

    zenoh: str
    sidecar: str
    t0: float  # unix second of t=0
    cmd_t: np.ndarray
    cmd: np.ndarray  # (n, 3) vx, vy, wz
    real_t: np.ndarray
    real_pos: np.ndarray  # (m, 3) base_link, anchored
    real_quat: np.ndarray  # (m, 4) wxyz, anchored
    joint_t: np.ndarray
    joint_q: np.ndarray  # (k, 12) MuJoCo actuator order
    lag: float  # LIO lag measured by cross-correlation, seconds
    applied_lag: float = 0.0  # how much of it was actually taken off real_t
    direct_lag: float | None = None  # the same lag off the odometry's own stamps
    lag_source: str = "correlation"  # which of them timed real_t
    onboard_response: float = 0.0  # cmd_vel -> on-board yaw rate, seconds
    lio_response: float = 0.0  # cmd_vel -> raw LIO yaw rate, seconds
    joint_fault: str | None = None  # set when lowstate is not real leg state
    window: Window = dc_field(default_factory=Window)

    @property
    def schedule(self) -> tuple[np.ndarray, np.ndarray]:
        """The ``(t, cmd)`` pair :func:`walk.walk` holds zero-order between samples."""
        return self.cmd_t, self.cmd

    @property
    def span(self) -> float:
        return float(self.cmd_t[-1])


def read_cmd_vel(store: Any) -> tuple[np.ndarray, np.ndarray]:
    """``(t, cmd)`` in unix seconds and (n, 3) vx/vy/wz."""
    rows = [
        (o.ts, o.data.linear.x, o.data.linear.y, o.data.angular.z) for o in store.stream(CMD_VEL)
    ]
    if not rows:
        raise ValueError(f"no {CMD_VEL} messages")
    a = np.array(rows, float)
    return a[:, 0], a[:, 1:]


def read_base_track(
    store: Any, base_frame: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """LIO odometry resolved to ``base_frame``: ``(t, pos, quat, stamp_t)``, unix seconds.

    The recorded odometry is stamped on the lidar (``child_frame_id`` is
    ``mid360_link``); the mount leg comes off the recorded tf, so the track is
    the body's, not the sensor's -- a 0.3 m lever arm that shows up the moment
    the robot turns. ``t`` is the recorder's receipt and ``stamp_t`` what the
    payload itself claims (nan where it claims nothing); which one times the
    track is :func:`load_pair`'s decision, not this one's.
    """
    from dimos.memory2.tf import StreamTF

    tf = StreamTF.from_store(store, TF)
    if tf is None:
        raise ValueError(f"no {TF} stream — cannot resolve the base pose")
    base = OdomBasePose(tf, base_frame)
    ts: list[float] = []
    stamp: list[float] = []
    pos: list[list[float]] = []
    quat: list[list[float]] = []
    for obs in store.stream(ODOMETRY):
        p = base.resolve(obs.data)
        if p is None:
            continue
        ts.append(obs.ts)
        stamp.append(payload_ts(obs.data))
        pos.append([p.position.x, p.position.y, p.position.z])
        q = p.orientation
        quat.append([q.w, q.x, q.y, q.z])
    if not ts:
        raise ValueError("no odometry resolved to the base frame")
    return np.array(ts), np.array(pos), np.array(quat), np.array(stamp)


def joint_limits() -> np.ndarray:
    """The 12 leg joints' (low, high) angle limits, MuJoCo actuator order."""
    import mujoco

    model, _ = go2_model.load()
    ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{n}_joint")
        for n in go2_model.MUJOCO_ACTUATOR_NAMES
    ]
    return np.asarray(model.jnt_range[ids])


def joint_fault(q: np.ndarray, dq: np.ndarray, tau: np.ndarray, limits: np.ndarray) -> str | None:
    """Why a recorded joint stream cannot be scored, or ``None`` when it can.

    This guard exists because it caught a real one. The dimos CDR decoder used
    to align nested structs to their widest member, which CDR does not do -- a
    struct starts where the previous field ended and its first primitive does
    the aligning. That put ``motor_state`` four bytes late, so ``q`` read as the
    recorded ``dq``: nonzero, so a presence check passed, and noise-shaped, so
    the resulting RMS read exactly like a wrong motor permutation. The mapping
    was never wrong.

    So the test is physical rather than statistical: an angle outside the
    joint's own travel is not a mis-mapped angle, it is not an angle. Silence
    from ``dq`` and ``tau_est`` beside it says the frame is off, not the robot.
    """
    out = float(((q < limits[:, 0]) | (q > limits[:, 1])).mean())
    if not np.any(dq) and not np.any(tau):
        return (
            f"motor_state carries no joint velocity or torque and {out:.0%} of its "
            "angles lie outside the joint travel — suspect the LowState CDR frame, "
            "not the robot"
        )
    if out > OUT_OF_RANGE_MAX:
        return (
            f"{out:.0%} of recorded joint angles lie outside the Go2 joint travel — "
            "suspect the LowState CDR frame or the motor permutation"
        )
    return None


def read_joints(sidecar: str | Path) -> tuple[np.ndarray, np.ndarray, str | None]:
    """Sidecar ``lowstate`` joint angles: ``(t, q, fault)``, unix seconds, MuJoCo order.

    Ordering follows :func:`replay.read_states` -- Unitree motor order off the
    wire, permuted by name through :func:`model.unitree_to_mujoco`. ``fault``
    is :func:`joint_fault`'s verdict on whether the stream is real at all.
    """
    from dimos.navigation.motion.simulation.replay import read_states

    states = read_states(sidecar)
    if not states:
        raise ValueError(f"{sidecar}: no lowstate messages")
    model, _ = go2_model.load()
    perm = go2_model.unitree_to_mujoco(model)
    q = np.stack([s.q for s in states])[:, perm]
    dq = np.stack([s.dq for s in states])[:, perm]
    tau = np.stack([s.tau for s in states])[:, perm]
    return np.array([s.ts for s in states]), q, joint_fault(q, dq, tau, joint_limits())


def read_body_motion(sidecar: str | Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The robot's own motion estimate: ``(t, speed, yaw_rate)``, unix seconds.

    ``sportmodestate`` is the on-board estimator: independent of LIO, published
    at 300 Hz with no solver behind it, and stamped on the robot's clock. That
    is what makes it the reference :func:`clock_offset` aligns the LIO track
    against, and what makes :func:`response_lag` an outside check on the answer.
    """
    from dimos.memory2.cli.dataset import open_dataset

    store = open_dataset(str(sidecar))
    with store:
        stream: Any = store.stream("sportmodestate")
        rows = [(o.ts, o.data.velocity[0], o.data.velocity[1], o.data.yaw_speed) for o in stream]
    if not rows:
        raise ValueError(f"{sidecar}: no sportmodestate messages")
    a = np.array(rows, float)
    return a[:, 0], np.hypot(a[:, 1], a[:, 2]), a[:, 3]


# ---------------------------------------------------------------- alignment --


def _smooth(x: np.ndarray, n: int) -> np.ndarray:
    return metrics._moving_average(x, max(2, n))


def _interp(grid: np.ndarray, t: np.ndarray, x: np.ndarray) -> np.ndarray:
    if x.ndim == 1:
        return np.asarray(np.interp(grid, t, x))
    return np.stack([np.interp(grid, t, x[:, i]) for i in range(x.shape[1])], axis=1)


def clock_offset(
    a_t: np.ndarray,
    a_x: np.ndarray,
    b_t: np.ndarray,
    b_x: np.ndarray,
    *,
    max_lag: float = MAX_CLOCK_LAG_S,
    rate: float = 100.0,
) -> float:
    """Seconds ``a`` lags ``b``, by cross-correlation on a shared grid.

    Positive means ``a``'s samples are stamped late and want shifting earlier.
    Both signals are resampled and mean-removed first, so it measures shape
    agreement rather than level agreement.
    """
    lo = max(float(a_t[0]), float(b_t[0]))
    hi = min(float(a_t[-1]), float(b_t[-1]))
    if hi - lo < 2 * max_lag:
        return 0.0
    grid = np.arange(lo, hi, 1.0 / rate)
    a = _interp(grid, a_t, a_x)
    b = _interp(grid, b_t, b_x)
    a = a - a.mean()
    b = b - b.mean()
    if not np.any(a) or not np.any(b):
        return 0.0
    span = int(max_lag * rate)
    lags = list(range(-span, span + 1))
    scores = [
        float(a[max(0, k) : len(a) + min(0, k)] @ b[max(0, -k) : len(b) + min(0, -k)]) for k in lags
    ]
    return lags[int(np.argmax(scores))] / rate


def lio_lag(
    real_t: np.ndarray, real_pos: np.ndarray, sm_t: np.ndarray, sm_speed: np.ndarray
) -> float:
    """How late the LIO stream is stamped, against the on-board speed estimate."""
    grid = np.arange(float(real_t[0]), float(real_t[-1]), 0.01)
    p = _smooth(_interp(grid, real_t, real_pos[:, :2]), 40)
    v = np.gradient(p, 0.01, axis=0)
    return clock_offset(grid, np.hypot(v[:, 0], v[:, 1]), sm_t, _smooth(sm_speed, 40))


def direct_lag(dialect: Dialect) -> float | None:
    """The LIO lag straight off the odometry's stamps, or None when they cannot say.

    A stamp series that does not advance is not a clock, so it is refused rather
    than interpolated over.
    """
    if not dialect.sensor_time:
        return None
    stamps = dialect.ts - dialect.age
    return dialect.delta if bool(np.all(np.diff(stamps) >= 0.0)) else None


def choose_lag(measured: float, direct: float | None, override: float | None) -> tuple[float, str]:
    """Which lag times the LIO track, and where it came from.

    An explicit ``--lag`` wins outright. Otherwise the stamps win when they
    agree with the correlation, because they carry per-message truth rather
    than one number; a disagreement past :data:`LAG_DISAGREE_S` hands it back to
    the correlation, which is anchored to motion the robot actually made.
    """
    if override is not None:
        return override, "override"
    if direct is None:
        return (measured if abs(measured) >= LAG_TOLERANCE_S else 0.0), "correlation"
    if abs(direct - measured) > LAG_DISAGREE_S:
        return measured, "correlation (the stamps disagree)"
    return direct, "stamps"


def response_lag(cmd_t: np.ndarray, cmd: np.ndarray, t: np.ndarray, yaw_rate: np.ndarray) -> float:
    """How long the body takes to answer a turn command, seconds.

    An outside check on :func:`lio_lag`, because it can be asked of either yaw
    stream: against the on-board estimate it is the robot's true command-to-body
    latency, and against the raw LIO track it is that plus the pipeline lag. The
    two answers differing by exactly the measured lag is what says the lag is a
    timestamp artifact and not something the robot did.
    """
    lo = max(float(cmd_t[0]), float(t[0]))
    hi = min(float(cmd_t[-1]), float(t[-1]))
    if hi - lo < 5.0:
        return 0.0
    grid = np.arange(lo, hi, 0.01)
    answered = _smooth(_interp(grid, t, yaw_rate), 40)
    return -clock_offset(grid, held(grid, cmd_t, cmd)[:, 2], grid, answered)


def anchor(
    pos: np.ndarray, quat: np.ndarray, at: int, height: float = ANCHOR_HEIGHT
) -> tuple[np.ndarray, np.ndarray]:
    """Re-express a track relative to one of its own samples, yaw-levelled.

    The same trick :func:`vive.base_track` uses on tracker data: the recording
    frame's unknown origin and heading cancel, leaving motion relative to the
    anchor. Only yaw is levelled -- pitch and roll are physical, and the sim
    stands level anyway.
    """
    yaw0 = float(metrics.yaw_of(quat[at : at + 1])[0])
    c, s = math.cos(-yaw0), math.sin(-yaw0)
    rel = pos - pos[at]
    out = np.column_stack(
        [
            c * rel[:, 0] - s * rel[:, 1],
            s * rel[:, 0] + c * rel[:, 1],
            rel[:, 2] + height,
        ]
    )
    half = -yaw0 / 2.0
    rot = np.array([math.cos(half), 0.0, 0.0, math.sin(half)])
    return out, _quat_mul(rot, quat)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """``a`` (4,) times each row of ``b`` (n, 4), wxyz."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    return np.column_stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def relative_window(start: Instant | None, end: Instant | None, t0: float) -> Window:
    """A ``--from``/``--to`` pair as bounds in seconds from ``t0``.

    :class:`Window` is built in unix seconds because that is the clock the
    recordings carry; everything downstream works in run-relative time, so the
    epoch comes back off both bounds here rather than at every use.
    """
    win = Window.between(start, end, t0)
    return Window(win.lo - t0, win.hi - t0)


def load_pair(
    zenoh: str | Path,
    sidecar: str | Path | None = None,
    *,
    base_frame: str = "base_link",
    start: Instant | None = None,
    end: Instant | None = None,
    lag: float | None = None,
) -> FieldPair:
    """Decode a field pair onto one clock, anchored at the first command.

    ``lag`` forces the LIO clock correction; without it :func:`choose_lag`
    decides between the odometry's own stamps and the cross-correlation.
    """
    zpath = _resolve(zenoh)
    spath = _resolve(sidecar) if sidecar is not None else _resolve(sidecar_for(zpath))

    store = open_lcm_mcap(str(zpath))
    with store:
        cmd_t, cmd = read_cmd_vel(store)
        real_t, real_pos, real_quat, stamp_t = read_base_track(store, base_frame)

    sm_t, sm_speed, sm_yaw_rate = read_body_motion(spath)
    measured = lio_lag(real_t, real_pos, sm_t, sm_speed)
    dialect = stamp_dialect(real_t, stamp_t)
    direct = direct_lag(dialect)
    applied, source = choose_lag(measured, direct, lag)
    if direct is not None and source.startswith("correlation"):
        print(
            f"lio lag: odometry stamps say {direct * 1000:+.0f} ms, the correlation against "
            f"sportmodestate says {measured * 1000:+.0f} ms — over the {LAG_DISAGREE_S * 1000:.0f} "
            "ms they may differ by, so the correlation stands"
        )

    onboard = response_lag(cmd_t, cmd, sm_t, sm_yaw_rate)
    lio_yaw = np.unwrap(metrics.yaw_of(real_quat))
    lio_grid = np.arange(float(real_t[0]), float(real_t[-1]), 0.01)
    from_lio = response_lag(
        cmd_t, cmd, lio_grid, np.gradient(_smooth(_interp(lio_grid, real_t, lio_yaw), 40), 0.01)
    )

    joint_t, joint_q, fault = read_joints(spath)

    t0 = float(cmd_t[0])
    # timing the track by its own stamps beats subtracting one median from every
    # sample, so where they are trusted they ARE the clock
    real_t = stamp_t if source == "stamps" else real_t - applied
    at = int(np.searchsorted(real_t, t0).clip(0, len(real_t) - 1))
    real_pos, real_quat = anchor(real_pos, real_quat, at)

    return FieldPair(
        zenoh=str(zpath),
        sidecar=str(spath),
        t0=t0,
        cmd_t=cmd_t - t0,
        cmd=cmd,
        real_t=real_t - t0,
        real_pos=real_pos,
        real_quat=real_quat,
        joint_t=joint_t - t0,
        joint_q=joint_q,
        lag=measured,
        applied_lag=applied,
        direct_lag=direct,
        lag_source=source,
        onboard_response=onboard,
        lio_response=from_lio,
        joint_fault=fault,
        window=relative_window(start, end, t0),
    )


# ------------------------------------------------------------------ scoring --


def held(t: np.ndarray, cmd_t: np.ndarray, cmd: np.ndarray) -> np.ndarray:
    """The command in force at each of ``t``, zero-order held.

    cmd_vel arrives at a ragged 11 Hz while the policy ticks at 50; the robot
    holds the last one it got, so interpolating between them would score the
    sim against a command nobody ever issued.
    """
    i = np.clip(np.searchsorted(cmd_t, t, side="right") - 1, 0, len(cmd_t) - 1)
    out: np.ndarray = cmd[i]
    return out


def regime(values: np.ndarray) -> np.ndarray:
    """Band name per sample, from |commanded axis| against :data:`REGIME_BINS`."""
    names = np.empty(len(values), dtype=object)
    lo = 0.0
    for name, hi in REGIME_BINS:
        names[(values >= lo) & (values < hi)] = name
        lo = hi
    names[values >= lo] = REGIME_BINS[-1][0]
    return names


def dead_reckon(t: np.ndarray, cmd: np.ndarray) -> np.ndarray:
    """Integrate a body-frame twist schedule into an (n, 3) x/y/yaw path.

    This is the path the command asked for. Comparing a body against it is
    *tracking* error, which -- unlike sim-vs-real divergence -- does not
    accumulate chaos, so it survives a window long enough to be interesting.
    """
    out = np.zeros((len(t), 3))
    for i in range(1, len(t)):
        dt = float(t[i] - t[i - 1])
        vx, vy, wz = cmd[i - 1]
        yaw0 = out[i - 1, 2]
        yaw1 = yaw0 + wz * dt
        mid = 0.5 * (yaw0 + yaw1)
        out[i, 0] = out[i - 1, 0] + (vx * math.cos(mid) - vy * math.sin(mid)) * dt
        out[i, 1] = out[i - 1, 1] + (vx * math.sin(mid) + vy * math.cos(mid)) * dt
        out[i, 2] = yaw1
    return out


def body_rates(
    grid: np.ndarray, t: np.ndarray, pos: np.ndarray, quat: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """``(xyz, body_velocity, yaw_rate)`` for one track on a shared grid.

    Velocity comes back in the *body* frame, which is where a twist command
    lives; smoothing is a window in seconds (:data:`metrics.VELOCITY_WINDOW_S`)
    so a 30 Hz LIO track and a 50 Hz rollout get the same treatment.
    """
    dt = float(grid[1] - grid[0])
    n = max(2, round(metrics.VELOCITY_WINDOW_S / dt))
    p = _interp(grid, t, pos)
    v = np.gradient(_smooth(p, n), dt, axis=0)
    yaw = _interp(grid, t, np.unwrap(metrics.yaw_of(quat)))
    yaw_rate = np.gradient(_smooth(yaw, n), dt)
    c, s = np.cos(yaw), np.sin(yaw)
    body = np.column_stack([c * v[:, 0] + s * v[:, 1], -s * v[:, 0] + c * v[:, 1]])
    return p, body, yaw_rate


@dataclass
class Bin:
    """One regime band's numbers, on one commanded axis."""

    axis: str
    name: str
    n: int
    cmd_abs: float  # mean |commanded| in the band
    real_err: float  # RMS miss of the commanded axis, real body
    sim_err: float  # same, simulated body
    growth: float  # sim-vs-real planar divergence growth, m/s

    @property
    def ratio(self) -> float:
        return self.sim_err / self.real_err if self.real_err > 1e-9 else float("inf")


@dataclass
class WindowStats:
    """Position-space tracking over one window, both bodies re-anchored to it.

    Only honest over a short window. Dead reckoning integrates its own error,
    so over tens of seconds these numbers describe the integration, not the
    robot -- pick a window around one maneuver and read the bins for the rest.
    """

    lo: float
    hi: float
    n: int
    real_final: float  # distance from the commanded path at the window end, m
    real_max: float
    sim_final: float
    sim_max: float
    divergence: float  # sim vs real at the window end, m
    real_yaw_err: float  # heading miss against the commanded path, rad
    sim_yaw_err: float


@dataclass
class FieldReport:
    """Everything one field pair says about the fitted sim."""

    pair: FieldPair
    lag: float
    seconds: float
    start: float
    grid: np.ndarray = dc_field(repr=False)
    divergence: np.ndarray = dc_field(repr=False)  # sim vs real planar, m
    dz: np.ndarray = dc_field(repr=False)  # z drift difference, m
    bins: list[Bin] = dc_field(default_factory=list)
    standing: Bin | None = None
    joint_rms: np.ndarray = dc_field(default_factory=lambda: np.zeros(0))
    joint_fault: str | None = None
    window: WindowStats | None = None
    fell_at: float | None = None  # rollout time the sim base first sank, s
    preset: str = "stock"  # the named tune the rollout ran under
    physics: dict[str, float] = dc_field(default_factory=dict)

    @property
    def joint_rms_overall(self) -> float:
        return float(np.sqrt((self.joint_rms**2).mean())) if len(self.joint_rms) else float("nan")

    def refit_note(self) -> str:
        """Whether the aggressive band transfers, and the command to fix it."""
        agg = [b for b in self.bins if b.name == "aggressive"]
        worst = max((b.ratio for b in agg), default=float("nan"))
        verdict = (
            f"{self.preset!r} covers the aggressive regime"
            if math.isfinite(worst) and worst < REFIT_RATIO
            else "REFIT: the sim misses the aggressive commands differently than the robot does"
        )
        name = f"field_{Path(self.pair.zenoh).name.split('.')[0].split('-')[0]}"
        return "\n".join(
            [
                f"verdict: {verdict} (worst aggressive sim/real error ratio {worst:.2f})",
                "",
                "to refit including this run -- as a NEW preset, never over 'fitted':",
                "  python -m dimos.navigation.motion.simulation.search \\",
                "      data/ml-trajectory-research/unitree_himloco01.mcap \\",
                "      data/ml-trajectory-research/freewalk_mcf.bin \\",
                "      --also data/ml-trajectory-research/unitree_v11_gait_height01.mcap \\",
                "             data/ml-trajectory-research/v11_final.bin \\",
                f"      --also {self.pair.zenoh} \\",
                "             data/ml-trajectory-research/freewalk_mcf.bin \\",
                "      --seed-preset fitted --trials 300 --start 0 \\",
                f"      --save-preset {name}",
                "",
                "then compare the two side by side, and keep both:",
                f"  python -m dimos.navigation.motion.simulation.field {self.pair.zenoh} \\",
                f"      --policy data/ml-trajectory-research/freewalk_mcf.bin --preset {name}.json",
                "",
                "blocked: search scores through evaluate(), which reads control_log",
                "and vive_pose. A field pair has neither -- wire load_pair() in as an",
                "alternative source before that third --also resolves.",
            ]
        )

    def table(self) -> str:
        p = self.pair
        head = [
            f"{Path(p.zenoh).name} + {Path(p.sidecar).name}",
            f"{self.seconds:.1f}s from t={self.start:.1f}s  "
            f"{len(p.cmd_t)} cmds  {len(p.real_t)} lio  {len(p.joint_t)} lowstate",
            f"lio clock lag {self.lag * 1000:+.0f} ms measured, "
            f"{p.applied_lag * 1000:+.0f} ms taken off the track",
            f"turn answers the command in {p.onboard_response * 1000:.0f} ms on-board and "
            f"{p.lio_response * 1000:.0f} ms through lio; the gap corroborates the lag",
        ]
        if p.direct_lag is not None:
            head.append(
                f"odometry speaks sensor-time: its own stamps put the lag at "
                f"{p.direct_lag * 1000:+.0f} ms, and the track is timed by the {p.lag_source}"
            )
        if self.physics:
            head.append(
                f"preset {self.preset!r}: "
                + " ".join(f"{k}={v:g}" for k, v in sorted(self.physics.items()))
            )

        if self.fell_at is not None:
            head.append(f"SIM FELL at t={self.fell_at:.1f}s — everything past it is meaningless")

        lines = [*head, ""]
        lines.append("sim vs real base divergence (accumulating, chaos-dominated)")
        for frac in (0.25, 0.5, 0.75, 1.0):
            i = min(int(frac * len(self.grid)), len(self.grid) - 1)
            lines.append(
                f"  t={self.grid[i]:6.1f}s   planar {self.divergence[i]:6.2f} m"
                f"   z-drift gap {self.dz[i]:+6.3f} m"
            )
        for reach in (0.5, 1.0):
            hit = np.flatnonzero(self.divergence >= reach)
            when = f"t={self.grid[hit[0]]:.1f}s" if len(hit) else "never"
            lines.append(f"  first past {reach:.1f} m: {when}")

        lines += ["", "twist tracking by maneuver regime (RMS miss of the commanded axis)"]
        lines.append(
            f"  {'axis':>4} {'band':>10} {'n':>6} {'|cmd|':>7} {'real':>8} {'sim':>8} {'sim/real':>9} {'drift m/s':>10}"
        )
        for b in self.bins:
            lines.append(
                f"  {b.axis:>4} {b.name:>10} {b.n:6d} {b.cmd_abs:7.3f} "
                f"{b.real_err:8.3f} {b.sim_err:8.3f} {b.ratio:9.2f} {b.growth:10.3f}"
            )
        if self.standing is not None:
            s = self.standing
            lines.append(
                f"  {'--':>4} {'standing':>10} {s.n:6d} {s.cmd_abs:7.3f} "
                f"{s.real_err:8.3f} {s.sim_err:8.3f} {s.ratio:9.2f} {s.growth:10.3f}"
            )

        if self.window is not None:
            w = self.window
            lines += [
                "",
                f"window {w.lo:.2f}..{w.hi:.2f}s, both bodies re-anchored at its start",
                f"  vs commanded path   real {w.real_final:.3f} m final / {w.real_max:.3f} max"
                f"   yaw {w.real_yaw_err:+.3f} rad",
                f"                       sim {w.sim_final:.3f} m final / {w.sim_max:.3f} max"
                f"   yaw {w.sim_yaw_err:+.3f} rad",
                f"  sim vs real          {w.divergence:.3f} m at the window end",
            ]

        if self.joint_fault is not None:
            lines += ["", f"joint RMS unavailable: {self.joint_fault}"]
        elif len(self.joint_rms):
            lines += ["", f"joint RMS sim vs lowstate: {self.joint_rms_overall:.3f} rad overall"]
            for name, v in zip(go2_model.MUJOCO_ACTUATOR_NAMES, self.joint_rms, strict=True):
                lines.append(f"  {name:10s} {v:.3f}")

        lines += ["", self.refit_note()]
        return "\n".join(lines)


def _window_stats(
    lo: float,
    hi: float,
    grid: np.ndarray,
    real_p: np.ndarray,
    real_yaw: np.ndarray,
    sim_p: np.ndarray,
    sim_yaw: np.ndarray,
    cmd_grid: np.ndarray,
) -> WindowStats | None:
    m = (grid >= lo) & (grid <= hi)
    if m.sum() < 2:
        return None
    t = grid[m]
    want = dead_reckon(t, cmd_grid[m])

    def miss(p: np.ndarray, yaw: np.ndarray) -> tuple[float, float, float, np.ndarray]:
        rel = p[m][:, :2] - p[m][0, :2]
        yaw0 = yaw[m][0]
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        local = np.column_stack([c * rel[:, 0] - s * rel[:, 1], s * rel[:, 0] + c * rel[:, 1]])
        d = np.linalg.norm(local - want[:, :2], axis=1)
        dyaw = float((yaw[m][-1] - yaw0) - want[-1, 2])
        return float(d[-1]), float(d.max()), dyaw, local

    r_final, r_max, r_yaw, r_local = miss(real_p, real_yaw)
    s_final, s_max, s_yaw, s_local = miss(sim_p, sim_yaw)
    return WindowStats(
        lo=float(t[0]),
        hi=float(t[-1]),
        n=int(m.sum()),
        real_final=r_final,
        real_max=r_max,
        sim_final=s_final,
        sim_max=s_max,
        divergence=float(np.linalg.norm(s_local[-1] - r_local[-1])),
        real_yaw_err=r_yaw,
        sim_yaw_err=s_yaw,
    )


def _bins(
    grid: np.ndarray,
    cmd_grid: np.ndarray,
    real_body: np.ndarray,
    real_yr: np.ndarray,
    sim_body: np.ndarray,
    sim_yr: np.ndarray,
    div: np.ndarray,
    keep: np.ndarray,
) -> tuple[list[Bin], Bin | None]:
    dt = float(grid[1] - grid[0])
    growth_all = np.gradient(div, dt)
    moving = keep & (np.abs(cmd_grid).max(axis=1) > STANDING)

    axes = (
        ("wz", np.abs(cmd_grid[:, 2]), real_yr - cmd_grid[:, 2], sim_yr - cmd_grid[:, 2]),
        (
            "vy",
            np.abs(cmd_grid[:, 1]),
            real_body[:, 1] - cmd_grid[:, 1],
            sim_body[:, 1] - cmd_grid[:, 1],
        ),
    )
    out: list[Bin] = []
    for axis, mag, r_err, s_err in axes:
        bands = regime(mag)
        for name, _hi in REGIME_BINS:
            m = moving & (bands == name)
            if not m.any():
                continue
            out.append(
                Bin(
                    axis=axis,
                    name=name,
                    n=int(m.sum()),
                    cmd_abs=float(mag[m].mean()),
                    real_err=float(np.sqrt((r_err[m] ** 2).mean())),
                    sim_err=float(np.sqrt((s_err[m] ** 2).mean())),
                    growth=float(growth_all[m].mean()),
                )
            )

    stand = keep & ~moving
    standing = None
    if stand.any():
        standing = Bin(
            axis="--",
            name="standing",
            n=int(stand.sum()),
            cmd_abs=0.0,
            real_err=float(np.sqrt((np.linalg.norm(real_body[stand], axis=1) ** 2).mean())),
            sim_err=float(np.sqrt((np.linalg.norm(sim_body[stand], axis=1) ** 2).mean())),
            growth=float(growth_all[stand].mean()),
        )
    return out, standing


def joint_rms(
    grid: np.ndarray, sim_t: np.ndarray, sim_q: np.ndarray, real_t: np.ndarray, real_q: np.ndarray
) -> np.ndarray:
    """Per-joint RMS angle difference, sim against the sidecar's ``lowstate``.

    Nearest recorded sample, not interpolated: lowstate runs at 500 Hz, ten
    times the policy rate, so the nearest one is never more than a millisecond
    off. Expect a large number -- the gait's phase decorrelates within seconds
    (``FINDINGS.md``) -- and read it as a permutation check plus a posture
    comparison, not as tracking.
    """
    s = _interp(grid, sim_t, sim_q)
    i = np.clip(np.searchsorted(real_t, grid), 0, len(real_t) - 1)
    err = s - real_q[i]
    return np.asarray(np.sqrt((err**2).mean(axis=0)))


def report(
    pair: FieldPair,
    policy_bin: str | Path,
    *,
    start: float = 0.0,
    seconds: float | None = None,
    physics: dict[str, float] | None = None,
    command_delay: float = 0.0,
    actuator_tau: float = 0.0,
    preset: str = "stock",
    slew: bool = True,
    view: bool = False,
    ghost: bool = False,
    speed: float = 1.0,
) -> FieldReport:
    """Roll the policy out under the pair's commands and score it against the pair."""
    from dimos.navigation.motion.simulation import evaluate as ev

    policy = FreePolicy.load(policy_bin)
    if seconds is None:
        seconds = pair.span - start
    ghost_track = (pair.real_t, pair.real_pos, pair.real_quat) if ghost else None
    with ev._physics(physics):
        track = walk_mod.walk(
            policy,
            schedule=pair.schedule,
            seconds=seconds,
            start=start,
            command_delay=command_delay,
            actuator_tau=actuator_tau,
            slew=slew,
            view=view,
            speed=speed,
            ghost=ghost_track,
        )

    # One grid for both bodies, in rollout time (t=0 is the rollout's start).
    hi = min(float(track.t[-1]), float(pair.real_t[-1]) - start)
    grid = np.arange(0.0, hi, 1.0 / GRID_HZ)
    real_p, real_body, real_yr = body_rates(
        grid + start, pair.real_t, pair.real_pos, pair.real_quat
    )
    sim_p, sim_body, sim_yr = body_rates(grid, track.t, track.pos, track.quat)
    # Both tracks are anchored, but at different samples; level the sim onto the
    # real body's pose at the rollout's own start so t=0 divergence is zero.
    real_p = real_p - real_p[0] + sim_p[0]

    div = np.linalg.norm(sim_p[:, :2] - real_p[:, :2], axis=1)
    dz = (sim_p[:, 2] - sim_p[0, 2]) - (real_p[:, 2] - real_p[0, 2])
    cmd_grid = held(grid + start, pair.cmd_t, pair.cmd)

    keep = pair.window.mask(grid + start)
    bins, standing = _bins(grid, cmd_grid, real_body, real_yr, sim_body, sim_yr, div, keep)

    win = None
    if pair.window.bounded and keep.any():
        sel = grid[keep]
        win = _window_stats(
            float(sel[0]),
            float(sel[-1]),
            grid,
            real_p,
            _interp(grid + start, pair.real_t, np.unwrap(metrics.yaw_of(pair.real_quat))),
            sim_p,
            _interp(grid, track.t, np.unwrap(metrics.yaw_of(track.quat))),
            cmd_grid,
        )

    jg = grid[keep] if keep.any() else grid
    rms = (
        np.zeros(0)
        if pair.joint_fault
        else joint_rms(jg, track.t, track.joint_q, pair.joint_t - start, pair.joint_q)
    )

    sank = np.flatnonzero(track.pos[:, 2] < FALLEN_Z)

    return FieldReport(
        pair=pair,
        lag=pair.lag,
        seconds=seconds,
        start=start,
        grid=grid,
        divergence=div,
        dz=dz,
        bins=bins,
        standing=standing,
        joint_rms=rms,
        joint_fault=pair.joint_fault,
        window=win,
        fell_at=float(track.t[sank[0]]) if len(sank) else None,
        preset=preset,
        physics={
            **(physics or {}),
            **({"command_delay": command_delay} if command_delay else {}),
            **({"actuator_tau": actuator_tau} if actuator_tau else {}),
        },
    )


# ---------------------------------------------------------------------- cli --


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(prog="motion.simulation.field")
    ap.add_argument("zenoh", help="the nav-side *.zenoh.mcap of a field pair")
    ap.add_argument("--sidecar", default=None, help="default: the matching *.mcap")
    ap.add_argument("--policy", required=True, help="FREE .bin the robot ran")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument(
        "--from",
        dest="start_at",
        default=None,
        help="report window start: seconds into the run, or a UTC HH:MM:SS[.fff]",
    )
    ap.add_argument("--to", dest="end_at", default=None, help="window end, same two forms")
    ap.add_argument("--start", type=float, default=0.0, help="skip this much before rolling out")
    ap.add_argument("--seconds", type=float, default=None, help="rollout length")
    ap.add_argument("--lag", type=float, default=None, help="force the LIO clock correction, s")
    ap.add_argument("--physics", default="", help="overrides, e.g. armature=0.03,damping=2.0")
    ap.add_argument("--fitted", action="store_true", help="the validated 'fitted' preset")
    ap.add_argument(
        "--preset",
        default=None,
        help="named physics: 'fitted' (default tune), 'stock', or a path to a "
        "preset JSON written by search.py --save-preset",
    )
    ap.add_argument("--command-delay", type=float, default=None)
    ap.add_argument("--actuator-tau", type=float, default=None)
    ap.add_argument("--no-slew", action="store_true", help="skip the hardware command ramp")
    ap.add_argument("--view", action="store_true", help="interactive MuJoCo window")
    ap.add_argument("--ghost", action="store_true", help="draw the LIO base track as a box")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed for --view")
    args = ap.parse_args()

    from dimos.navigation.motion.simulation import evaluate as ev

    overrides = dict(
        (k, float(v)) for k, v in (p.split("=", 1) for p in args.physics.split(",") if p)
    )
    delay, tau = args.command_delay, args.actuator_tau
    preset = None
    if args.fitted or args.preset:
        preset = ev.load_preset(args.preset)
        overrides = {**preset.physics, **overrides}
        delay = preset.command_delay if delay is None else delay
        tau = preset.actuator_tau if tau is None else tau

    pair = load_pair(
        args.zenoh,
        args.sidecar,
        base_frame=args.base_frame,
        start=parse_instant(args.start_at) if args.start_at else None,
        end=parse_instant(args.end_at) if args.end_at else None,
        lag=args.lag,
    )
    print(
        report(
            pair,
            args.policy,
            start=args.start,
            seconds=args.seconds,
            physics=overrides,
            command_delay=0.0 if delay is None else delay,
            actuator_tau=0.0 if tau is None else tau,
            preset=preset.name if preset is not None else "stock",
            slew=not args.no_slew,
            view=args.view,
            ghost=args.ghost,
            speed=args.speed,
        ).table()
    )


if __name__ == "__main__":
    main()
