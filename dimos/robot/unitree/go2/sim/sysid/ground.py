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

"""Loop 2 — Mode B: the REAL policy, closed loop, scored on statistics.

    python -m dimos.robot.unitree.go2.sim.sysid.ground REC.mcap NET.bin \
        --preset accel --view

Loop 1 turns the knobs; this says whether turning them helped. It runs the
net that PRODUCED the recording (verify with ``sysid.verify_net`` first —
a grounding against the wrong net is confident and meaningless) closed loop
in the candidate plant, driven by the recording's operator commands, and
compares the eleven chaos-tolerant statistics of the two runs
(:mod:`~dimos.robot.unitree.go2.sim.sysid.stats`). Never UNBOUNDED
trajectories: free position error measures how long ago two chaotic runs
diverged, not whether the plant is right — the preset fitted on closed-loop
trajectories was worse than no overrides at all under the open-loop
objective. What IS scored on the trajectory is the TRACKING AREA
(:func:`tracking_curves`): mean |error| of the free rollouts against the
recording per heading-aligned component — measured to be regulated,
bounded wander (the shared command schedule pulls the loop back), so the
number is well defined without any snapping. The snapped LOCAL RESPONSE
rates (:func:`window_curves`) stay as a printed diagnostic.

Each statistic is normalised by its own NOISE FLOOR: the sim disagrees with
ITSELF under chaos, so ``SNR = |sim - real| / floor``, and an SNR under ~1
means matched to within what chaos already does — no better is askable. The
floor's SOURCE is a parameter of the claim:

* sim-perturb (the default): the spread of the verdict's own replicate
  rollouts — perturbed initial poses, what chaos alone does
  (:func:`sim_noise` measures the same thing standalone).
* :func:`~dimos.robot.unitree.go2.sim.sysid.real.robot_noise` spreads the
  statistics across REPEAT RECORDINGS of the same walk — captures battery
  sag and motor temperature too, and is the yardstick the publishable claim
  needs: *"the simulator differs from the robot by less than the robot
  differs from itself, on N of 11 statistics."* Swapping it in is
  ``--noise-from REC2.mcap REC3.mcap``, not a rewrite.

The REAL side of the comparison (:mod:`~dimos.robot.unitree.go2.sim.sysid
.real`) is pure recording processing and lives outside this module —
position from the tracker, attitude from the IMU (README 6). The SIM side
is engine-free too: :func:`rollout_policy` is a generic closed-loop driver
over the :class:`~dimos.simulation.sysid.backend.LoopSession` seam,
and the engine enters only through the ``backend`` argument.

A well-stabilised policy can drive a floor to ~0, sending that SNR to
infinity and letting one term dominate; :func:`usable_floor` clamps against
the same statistic's floor on another recording plus 5% of the real value.

HARD RULE, learned expensively: identify with Mode A, validate here, never
fit the plant on this loop. ~11 statistics support selecting two or three
METHOD hyperparameters (:mod:`~dimos.robot.unitree.go2.sim.sysid.meta`);
they cannot fit thirteen plant parameters.
"""

from __future__ import annotations

import argparse
import collections
from collections.abc import Sequence
from dataclasses import dataclass, field, replace

import numpy as np

from dimos.robot.unitree.go2.sim.plant import (
    TORQUE_ENVELOPES,
    TORQUE_LIMITS,
    TorqueEnvelope,
    actuator_step,
)
from dimos.robot.unitree.go2.sim.policy import FreePolicy
from dimos.robot.unitree.go2.sim.ranges import DEFAULT_PRESET, Preset, load_preset
from dimos.robot.unitree.go2.sim.sysid.gait import strides
from dimos.robot.unitree.go2.sim.sysid.ingest import TRACKER_Z, mount_matrix, read_streams
from dimos.robot.unitree.go2.sim.sysid.real import cmd_at, real_summary, robot_noise
from dimos.robot.unitree.go2.sim.sysid.replay import ghost_track, measured_state
from dimos.robot.unitree.go2.sim.sysid.stats import (
    NOT_COMPARABLE,
    Summary,
    median_summary,
    pitch_roll_of,
    spread_of,
    summarize,
    yaw_of,
)
from dimos.simulation.sysid.backend import ClosedLoopBackend, GhostTrack, State
from dimos.simulation.sysid.recording import Streams
from dimos.simulation.sysid.rotations import mat_to_quat, quat_to_mat

CONTROL_DT = 0.02  # 50 Hz policy rate; not stored in the blob

# Per-axis slew the executor applies to operator commands before the policy
# sees them: max change in (vx, vy, vyaw) per 20 ms control tick (go2web
# policy.rs ramp_velocity). The recorded control_log carries the operator
# TARGET; the policy on hardware only ever saw the ramped command.
COMMAND_SLEW = np.array([0.05, 0.04, 0.10])

# Body height a gait-height net holds when nobody moves the slider (obs 45,
# raw metres). 45-channel nets (himloco freewalk) never see it.
NOMINAL_GAIT_HEIGHT = 0.31

PERTURB_RAD = 0.005
"""Replicate joint-pose spread — about 0.3 degrees.

Exists only to DECORRELATE replicate draws, and README 8 measured that any
perturbation from 1e-9 up redraws the chaos distribution identically —
chaos saturates regardless of kick size, so nothing is lost going small.
The old 3-degree value was calibrated on standing starts and could TOPPLE
the mid-walk measured seed (a 55-loss fall draw); 0.3 degrees decorrelates
just as completely and respects the seeded dynamics.
"""


@dataclass(frozen=True)
class ObsNoise:
    """Sensor noise on the policy's observation, in raw units, uniform ±level.

    The real IMU and encoders are noisy and the net was TRAINED with noise —
    legged_gym adds uniform noise to every observation during training, and
    these defaults are its standard levels de-scaled to raw units (ang_vel
    0.2/0.25 · 0.25, dof_vel 1.5, dof_pos 0.01, gravity 0.05). A noiseless
    closed loop is therefore out-of-distribution smooth: the policy's
    reaction to noise IS body motion. Default-off — every existing grounding
    number reproduces bit-for-bit when this is ``None``.
    """

    gyro: float = 0.2  # rad/s
    gravity: float = 0.05  # unit-vector components
    q: float = 0.01  # rad
    dq: float = 1.5  # rad/s

    def scaled(self, s: float) -> ObsNoise:
        return ObsNoise(self.gyro * s, self.gravity * s, self.q * s, self.dq * s)


def projected_gravity(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_wxyz
    return np.array([-2 * (x * z - w * y), -2 * (y * z + w * x), -(1 - 2 * (x * x + y * y))])


@dataclass
class PolicyRun:
    """One closed-loop rollout, sampled at the policy rate.

    ``t`` starts at 0 = recording time ``start``; ``cmd`` is the slewed
    command the policy actually saw at each sample. The ``reinit_*`` arrays
    are the snap bookkeeping of a windowed-divergence run (empty on the
    ordinary free rollout): when each re-initialisation fired and the sim
    pose it left the base at — needed because window drift is only defined
    relative to the pose the window started from.
    """

    t: np.ndarray
    pos: np.ndarray = field(repr=False)  # (n, 3) base position, sim world
    quat: np.ndarray = field(repr=False)  # (n, 4) wxyz
    cmd: np.ndarray = field(repr=False)  # (n, 3)
    target: np.ndarray = field(repr=False)  # (n, 12) commanded joint targets
    q: np.ndarray = field(repr=False, default_factory=lambda: np.zeros((0, 12)))  # (n, 12) joints
    reinit_t: np.ndarray = field(repr=False, default_factory=lambda: np.zeros(0))  # run clock
    reinit_pos: np.ndarray = field(repr=False, default_factory=lambda: np.zeros((0, 3)))
    reinit_quat: np.ndarray = field(repr=False, default_factory=lambda: np.zeros((0, 4)))


def rollout_policy(
    st: Streams,
    policy: FreePolicy,
    preset: Preset,
    backend: ClosedLoopBackend,
    *,
    start: float = 6.0,
    seconds: float | None = None,
    settle: float = 0.0,
    command_delay: float = 0.0,
    action_latency: float = 0.0,
    obs_noise: ObsNoise | None = None,
    noise_seed: int = 0,
    control_intervals: np.ndarray | None = None,
    envelope: TorqueEnvelope | None = None,
    perturb: np.ndarray | None = None,
    reinit: Sequence[State] | None = None,
    view: bool = False,
    speed: float = 1.0,
    ghost: GhostTrack | None = None,
) -> PolicyRun:
    """Step the real policy in the candidate plant, driven by the recording.

    Closed loop, so :class:`RolloutPlan` (a complete instruction sheet
    decided before any physics) deliberately cannot express it: the policy
    sits INSIDE the physics step loop. The seam is a stepping primitive
    instead — this driver owns the policy, the observation build, the
    command slew and every loop mechanism, and asks the ``backend`` for a
    :class:`~dimos.simulation.sysid.backend.LoopSession` that only
    steps physics and reports state. A second backend implements the
    session and inherits this whole loop.

    The rollout SEEDS from the measured state at ``start`` (joints,
    attitude, and — with a tracker — base velocity; tracker-less
    recordings seed pose only and start at rest), and the policy acts
    from the FIRST tick — ``settle`` defaults to 0 because holding the
    seeded pose rigidly while the body glides at walking speed is itself
    a stumble (measured: a 0.5 s hold read tilt_p99 0.21 against 0.15).
    The few-tick gait-phase spin-up that remains is real and unavoidable
    without the policy's true hidden history. ``perturb`` offsets the seeded joint
    pose — :func:`sim_noise`'s knob. ``command_delay`` is kept at 0 by
    default: it is not identified open loop, and holding it identical
    across candidate plants cancels it from every between-plant
    comparison.

    ``reinit`` is the local-response schedule: measured states the loop is
    snapped to mid-run via :meth:`LoopSession.snap`; ``None`` (the
    default, every free verdict rollout) never snaps mid-run. A snap
    clears the actuator filter and pending delayed actions (Mode A's
    convention) but not the policy's observation history (the real
    policy's memory spans the boundary too), and mid-run snapped states
    are never perturbed (an injection would ride the measured slope).

    Four default-off mechanisms the real loop has and the ideal sim lacks —
    each is a candidate explanation for the sim walking SMOOTHER than the
    robot, and each leaves every existing number bit-identical when off:

    * ``action_latency`` — seconds between the policy emitting a joint
      target and the PD seeing it (obs transport + inference + lowcmd
      transport, indistinguishable in loop terms). Delay in a feedback loop
      eats phase margin, so LESS delay means LESS oscillation. This is not
      ``command_delay``, which shifts the operator schedule.
    * ``obs_noise`` (with ``noise_seed``) — sensor noise the policy reacts
      to; see :class:`ObsNoise`.
    * ``control_intervals`` — the executor's MEASURED inter-command
      intervals (:func:`~dimos.robot.unitree.go2.sim.sysid.loop
      .control_timing`), replayed as a sequence (tiled if the rollout
      outlives it) in place of the ideal 20 ms grid. The measured executor
      runs at ~44 Hz with jitter and dropouts, not 50 Hz — this mechanism
      has zero free parameters. ``None`` keeps the exact ``CONTROL_DT``
      grid, bit-identical to every existing number.
    * ``envelope`` — the measured torque derate above 3 rad/s
      (:data:`~dimos.robot.unitree.go2.sim.plant.TORQUE_ENVELOPES`); without
      it the sim's actuators track crisply at swing speeds the real drive
      cannot. ``None`` falls back to the PRESET's own envelope — a plant
      fitted with the envelope on must run with it, or the knobs silently
      shed the share of the drive the fit assigned to the envelope.

    THE VIEWER AND THE HEADLESS RUN ARE THE SAME FUNCTION: ``view`` only
    attaches a viewer and paces to wall clock; ``ghost`` draws the recorded
    tracker pose, anchored once at the start pose (closed loop never snaps).
    """
    if envelope is None and preset.envelope is not None:
        envelope = TORQUE_ENVELOPES[preset.envelope]
    if len(st.wt) == 0:
        raise ValueError("recording has no control_log walk commands: Mode B needs the drive")
    span = float(st.wt[-1])
    if start >= span:
        raise ValueError(f"start={start:g}s is at or past the last command ({span:.1f}s)")
    duration = span - start if seconds is None else seconds

    ghost = ghost if view else None
    backend.apply(preset.physics)
    # Seed from the MEASURED state at `start`, not the standing default:
    # the recording is mid-walk there, and starting at rest spent the first
    # ~1 s on standstill catch-up that contaminated every trajectory area
    # (visible as the early hump in the free curves, and visible to the eye
    # under --view as the ghost walking away). Joints, yaw-stripped
    # attitude and — WITH a tracker — the base velocity all come from
    # measured_state; a tracker-less recording seeds pose but starts at
    # rest (no velocity to seed), stated here rather than silent. The ±3°
    # verdict perturbation applies on top of the measured joints.
    base_p = base_r = None
    if st.has_markers:
        base_p, base_r = st.base_pose_room(mount_matrix(), TRACKER_Z)
    seed_state = measured_state(st, start, base_p=base_p, base_r=base_r)
    if perturb is not None:
        seed_state = replace(seed_state, q=seed_state.q + perturb)
    session = backend.session(seed_state.q, ghost=ghost is not None, view=view, view_speed=speed)
    session.snap(seed_state)
    sim_dt = session.timestep
    decim = max(1, round(CONTROL_DT / sim_dt))
    s = session.state()

    # Ghost anchor: rigid room -> sim-world map, fixed at the start pose.
    g_anchor_r = np.eye(3)
    g_anchor_p = np.zeros(3)
    if ghost is not None:
        j = int(np.clip(np.searchsorted(ghost.t, start, "right") - 1, 0, len(ghost.t) - 1))
        g_anchor_r = quat_to_mat(s.quat) @ ghost.rot[j].T
        g_anchor_p = s.pos - g_anchor_r @ ghost.pos[j]

    hist: collections.deque[np.ndarray] = collections.deque(maxlen=policy.hist)
    last_action = np.zeros(policy.act_dim)
    target = seed_state.q.copy()
    applied = np.zeros(12)
    rng = np.random.default_rng(noise_seed) if obs_noise is not None else None
    delay_steps = max(0, round(action_latency / sim_dt))
    pending: collections.deque[tuple[int, np.ndarray]] = collections.deque()

    # Measured control timing: each tick fires on the first step at or past
    # its scheduled time. None keeps the exact `step % decim` grid untouched.
    ctrl_steps: set[int] | None = None
    if control_intervals is not None:
        iv = np.asarray(control_intervals, dtype=float)
        if len(iv) == 0 or np.any(iv <= 0):
            raise ValueError("control_intervals must be a non-empty positive sequence")
        n_steps = int(duration / sim_dt)
        ctrl_steps = set()
        t_next, i = 0.0, 0
        while t_next < duration:
            ctrl_steps.add(min(int(np.ceil(t_next / sim_dt - 1e-9)), n_steps))
            t_next += float(iv[i % len(iv)])
            i += 1

    def observe(cmd: np.ndarray, height: float) -> np.ndarray:
        q = s.q
        dq = s.dq
        gyro = s.gyro
        grav = projected_gravity(s.quat)
        if obs_noise is not None and rng is not None:
            gyro = gyro + obs_noise.gyro * rng.uniform(-1.0, 1.0, 3)
            grav = grav + obs_noise.gravity * rng.uniform(-1.0, 1.0, 3)
            q = q + obs_noise.q * rng.uniform(-1.0, 1.0, 12)
            dq = dq + obs_noise.dq * rng.uniform(-1.0, 1.0, 12)
        raw = np.concatenate([cmd, gyro, grav, q, dq, last_action])
        extra = policy.obs_per_frame - raw.size
        if extra > 0:
            raw = np.concatenate([raw, [height], np.zeros(extra - 1)])
        return policy.normalize(raw)

    def target_cmd(t: float) -> np.ndarray:
        k = int(np.searchsorted(st.wt, t + start - command_delay, "right")) - 1
        held: np.ndarray = st.wcmd[max(0, k)]
        return held

    def height_at(t: float) -> float:
        if len(st.ght) == 0:
            return NOMINAL_GAIT_HEIGHT
        i = int(np.searchsorted(st.ght, t + start - command_delay, "right")) - 1
        return float(st.gh[i]) if i >= 0 else NOMINAL_GAIT_HEIGHT

    # The live command; starts converged on the schedule, the steady state
    # the real slew is in mid-run.
    vel_cmd = target_cmd(0.0).astype(float).copy()

    # Teacher-force the observation history from the RECORDING: this net
    # has no hidden state — its whole memory is `hist` stacked frames plus
    # its own last action, and the robot recorded all of it. last_action
    # inverts exactly from the recorded joint targets (act() is
    # target = action*scale + mean), so the first sim tick computes what
    # the robot's own tick computed at `start` — the rollout CONTINUES the
    # stride rather than re-finding the gait from a cold stack.
    def recorded_action(t_abs: float) -> np.ndarray:
        ci = int(np.clip(np.searchsorted(cmds_ct, t_abs, "right") - 1, 0, len(cmds_ct) - 1))
        out: np.ndarray = (st.cq[ci] - policy.act_mean) / policy.act_scale
        return out

    cmds_ct = st.ct
    for k in range(policy.hist, 0, -1):
        t_abs = start - k * CONTROL_DT
        li = int(np.clip(np.searchsorted(st.lt, t_abs, "right") - 1, 0, len(st.lt) - 1))
        raw = np.concatenate(
            [
                target_cmd(t_abs - start),
                st.lgyro[li],
                projected_gravity(st.lquat[li]),
                st.lq[li],
                st.ldq[li],
                recorded_action(t_abs),
            ]
        )
        extra = policy.obs_per_frame - raw.size
        if extra > 0:
            raw = np.concatenate([raw, [height_at(t_abs - start)], np.zeros(extra - 1)])
        hist.append(policy.normalize(raw))
    last_action = recorded_action(start)

    ts: list[float] = []
    pos: list[np.ndarray] = []
    quat: list[np.ndarray] = []
    used: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    joints: list[np.ndarray] = []
    snaps = list(reinit) if reinit is not None else []
    ri = 0
    snap_t: list[float] = []
    snap_pos: list[np.ndarray] = []
    snap_quat: list[np.ndarray] = []

    try:
        for step in range(int(duration / sim_dt)):
            t = step * sim_dt
            if ri < len(snaps) and t + start >= snaps[ri].t:
                session.snap(snaps[ri])
                applied[:] = 0.0
                pending.clear()
                s = session.state()
                snap_t.append(t)
                snap_pos.append(s.pos)
                snap_quat.append(s.quat)
                if ghost is not None:
                    j = int(
                        np.clip(
                            np.searchsorted(ghost.t, t + start, "right") - 1, 0, len(ghost.t) - 1
                        )
                    )
                    g_anchor_r = quat_to_mat(s.quat) @ ghost.rot[j].T
                    g_anchor_p = s.pos - g_anchor_r @ ghost.pos[j]
                ri += 1
            s = session.state()
            if (step % decim == 0) if ctrl_steps is None else (step in ctrl_steps):
                vel_cmd += np.clip(target_cmd(t) - vel_cmd, -COMMAND_SLEW, COMMAND_SLEW)
                if t >= settle:
                    hist.append(observe(vel_cmd, height_at(t)))
                    p_obs = np.concatenate(list(hist)[::-1])  # newest first
                    if delay_steps > 0:
                        # The policy remembers its own action at once (it IS
                        # last_action in the next obs); only the PD sees it late.
                        last_action, fresh = policy.act(p_obs, vel_cmd)
                        pending.append((step + delay_steps, fresh))
                    else:
                        last_action, target = policy.act(p_obs, vel_cmd)
                if ghost is not None:
                    j = int(
                        np.clip(
                            np.searchsorted(ghost.t, t + start, "right") - 1, 0, len(ghost.t) - 1
                        )
                    )
                    session.show_ghost(
                        g_anchor_r @ ghost.pos[j] + g_anchor_p,
                        mat_to_quat(g_anchor_r @ ghost.rot[j]),
                    )
                ts.append(t)
                pos.append(s.pos)
                quat.append(s.quat)
                used.append(vel_cmd.copy())
                targets.append(target.copy())
                joints.append(s.q)

            while pending and pending[0][0] <= step:
                target = pending.popleft()[1]
            tau = policy.kp * (target - s.q) - policy.kd * s.dq
            tau = np.clip(tau, -TORQUE_LIMITS, TORQUE_LIMITS)
            applied = actuator_step(
                applied, tau, sim_dt, preset.actuator_tau, dq=s.dq, envelope=envelope
            )
            if not session.step(applied):
                break
    finally:
        session.close()

    return PolicyRun(
        t=np.array(ts),
        pos=np.array(pos),
        quat=np.array(quat),
        cmd=np.array(used),
        target=np.array(targets),
        q=np.array(joints),
        reinit_t=np.array(snap_t),
        reinit_pos=np.array(snap_pos).reshape(-1, 3),
        reinit_quat=np.array(snap_quat).reshape(-1, 4),
    )


def sim_summary(run: PolicyRun, cmd: np.ndarray | None = None) -> Summary:
    """The sim side, with height read by a VIRTUAL TRACKER on the base.

    Height statistics are compared in sensor space: the real side keeps the
    raw tracker height, so the sim mounts a virtual tracker with the same
    lever arm — the guess distorts both sides identically and mostly cancels.

    ``cmd`` is the command axis for the gain/lag statistics. Pass the RAW
    operator schedule (``cmd_at``) when comparing against a real side, which
    can only be regressed against the same raw schedule — asymmetric axes
    (slewed on one side, raw on the other) would bias the lags. ``None``
    falls back to the slewed command the policy actually saw.

    The stride pair comes from ``run.q`` through the same FK instrument the
    real side uses (``sysid.gait``); a run without joints (an old caller)
    leaves it NaN and the pair drops out of the SNR.
    """
    rot = quat_to_mat(run.quat)
    p = run.pos.copy()
    p[:, 2] = (run.pos + rot @ np.array([0.0, 0.0, TRACKER_Z]))[:, 2]
    c = run.cmd if cmd is None else cmd
    s = summarize(run.t, p, run.quat, c)
    if len(run.q) == len(run.t) and len(run.t):
        moving = np.linalg.norm(c[:, :2], axis=1) > 0.25
        g = strides(run.t, run.q, run.quat, run.pos[:, :2], moving)
        s = replace(s, stride_hz=g.stride_hz, stride_len=g.stride_len)
    return replace(s, source="sim")


# -------------------------------------- tracking error (the headline)

# The trajectory terms: mean |error| of the FREE verdict rollouts against
# the recording over the full horizon, per heading-aligned component —
# area under the error curve / horizon, plain units, nothing chosen.
# Free, not snapped: measured (194142, 8x8), the good plant's error is
# BOUNDED wander, not growth (|along| at 70 s < at 10 s) — the shared
# command schedule re-anchors both trajectories, so a divergence RATE
# does not exist as an object here and the mean error is the correct
# quantity outright, not a proxy. Recording-dependent by construction:
# this robot mills, so wander cancels; a long straight walk would
# accumulate the speed deficit instead — same-recording, same-horizon
# comparison (the house rule) is what keeps the number meaningful.
TRACKING_TERMS = ("along", "cross", "yaw", "pitch", "roll")

# What is SCORED: pitch and roll are floor-limited by the instrument and
# report-not-score (gait_hz's species) — the tracker's instantaneous
# attitude error (README 6: 1.85/2.95 deg = 0.032/0.051 rad) IS the
# measured mean error (shipped plant 0.041/0.053: ratios 1.3x/1.04x), so
# those two areas measure the instrument. Yaw clears its 0.053 rad
# instrument floor 4.4x and stays scored; scored floors are clamped by
# the instrument floor either way.
TRACKING_SCORED = ("along", "cross", "yaw")
TRACKING_INSTRUMENT_FLOOR = {
    "along": 0.0,
    "cross": 0.0,
    "yaw": 0.053,
    "pitch": 0.032,
    "roll": 0.051,
}

# Where each free rollout is anchored to the room: the first policy-active
# sample. One-time anchor, tracker truth throughout (README 6).
TRACKING_ANCHOR_S = 0.5


def tracking_curves(run: PolicyRun, st: Streams, *, start: float) -> dict[str, np.ndarray]:
    """Signed error vs time for ONE free rollout, tracker truth.

    Position split along/cross in the REAL trajectory's heading frame (raw
    euclidean error over a long rollout is mostly a yaw measurement in
    disguise); yaw as the anchored increment difference (unwrapped — it
    may legitimately exceed pi); pitch/roll absolute. Empty dict without a
    tracker.
    """
    if not st.has_markers or len(run.t) == 0:
        return {}
    base_p, base_r = st.base_pose_room(mount_matrix(), TRACKER_Z)
    rq = np.stack([mat_to_quat(r) for r in base_r])
    yaw_room = np.unwrap(yaw_of(rq))
    pitch_t, roll_t = pitch_roll_of(rq)
    yaw_s = np.unwrap(yaw_of(run.quat))
    pitch_s, roll_s = pitch_roll_of(run.quat)
    i0 = int(np.searchsorted(run.t, TRACKING_ANCHOR_S - 1e-9, "left"))
    if i0 >= len(run.t):
        return {}
    a0 = float(run.t[i0]) + start
    a_yaw = float(yaw_s[i0]) - _local_mean(st.vt, yaw_room, a0)
    c_, s_ = np.cos(a_yaw), np.sin(a_yaw)
    rot2 = np.array([[c_, -s_], [s_, c_]])
    p_room0 = np.array([np.interp(a0, st.vt, base_p[:, 0]), np.interp(a0, st.vt, base_p[:, 1])])
    a_p = run.pos[i0][:2] - rot2 @ p_room0
    a = run.t[i0:] + start
    p_room = np.stack(
        [np.interp(a, st.vt, base_p[:, 0]), np.interp(a, st.vt, base_p[:, 1])], axis=1
    )
    e = run.pos[i0:, :2] - (p_room @ rot2.T + a_p)
    yr = np.interp(a, st.vt, yaw_room)
    h = yr + a_yaw  # the real heading in the sim frame
    u = np.stack([np.cos(h), np.sin(h)], axis=1)
    n = np.stack([-np.sin(h), np.cos(h)], axis=1)
    return {
        "along": np.sum(e * u, axis=1),
        "cross": np.sum(e * n, axis=1),
        "yaw": (yaw_s[i0:] - float(yaw_s[i0])) - (yr - float(np.interp(a0, st.vt, yaw_room))),
        "pitch": pitch_s[i0:] - np.interp(a, st.vt, pitch_t),
        "roll": roll_s[i0:] - np.interp(a, st.vt, roll_t),
    }


@dataclass(frozen=True)
class Tracking:
    """The tracking-error verdict: median-of-n mean |error| per component.

    ``areas[term]`` holds every replicate's area (mean |error| over the
    horizon) so the verdict is the median with the draw range beside it,
    exactly as the eleven statistics. ``floors`` is the chaos floor, from
    the SAME rollouts: the median pairwise sim-vs-sim area / sqrt(2) — what
    a PERFECT plant would still score from chaos alone (7's sim-perturb
    floor, applied to this instrument; two chaotic deviations from truth
    add in quadrature, hence the sqrt(2)). No robot-repeat analogue exists:
    two real walks cannot be trajectory-aligned.
    """

    horizon_s: float
    areas: dict[str, np.ndarray]
    floors: dict[str, float]

    def median(self, term: str) -> float:
        return float(np.median(self.areas[term]))

    def draws(self, term: str) -> tuple[float, float]:
        v = self.areas[term]
        return float(np.min(v)), float(np.max(v))

    def floor_of(self, term: str) -> float:
        """Chaos floor clamped by the instrument's own error (README 6)."""
        return max(self.floors.get(term, 0.0), TRACKING_INSTRUMENT_FLOOR.get(term, 0.0))

    def snr(self) -> dict[str, float]:
        out = {}
        for term in TRACKING_SCORED:
            if term not in self.areas or not len(self.areas[term]):
                continue
            f = self.floor_of(term)
            if np.isfinite(f) and f > 1e-9:
                out[term] = self.median(term) / f
        return out


def tracking_of(curve_sets: list[dict[str, np.ndarray]], horizon_s: float) -> Tracking | None:
    """Fold the free replicates' curves into the area verdict + chaos floor."""
    curve_sets = [c for c in curve_sets if c]
    if len(curve_sets) < 2:
        return None
    areas = {
        t: np.array([float(np.mean(np.abs(c[t]))) for c in curve_sets]) for t in TRACKING_TERMS
    }
    floors = {}
    for t in TRACKING_TERMS:
        pair = [
            float(
                np.mean(
                    np.abs(a[t][: min(len(a[t]), len(b[t]))] - b[t][: min(len(a[t]), len(b[t]))])
                )
            )
            for i, a in enumerate(curve_sets)
            for b in curve_sets[i + 1 :]
        ]
        floors[t] = float(np.median(pair)) / np.sqrt(2.0)
    return Tracking(horizon_s=horizon_s, areas=areas, floors=floors)


# ------------------------------- local response (snapped, diagnostic)

# Re-init window length: the horizon axis of the curve, not a judged
# parameter — it must reach BIAS_FIT_S's end, and only trades how many
# windows pool against how far out the curve is visible. 2 s, measured:
# a 4 s window halves the pool (35 -> 17) and single violent windows then
# dominate the RMS terms (yaw SNR fell 5.1 -> 1.2, SE ~ rate).
DIVERGENCE_WINDOW_S = 2.0

# Fit intervals; printing all three is the validity check. Measured
# (194142): the curve has THREE regimes — gait-envelope rise (~half a
# stride), systematic linear, chaos — so 0.5 s reads oscillation amplitude
# as a rate, 2 s dilutes into saturation, and 1 s is the quoted rate.
DIVERGENCE_FIT_S = (0.5, 1.0, 2.0)
DIVERGENCE_SCORED_FIT_S = 1.0

# The SIGNED bias fits a later stretch (past the snap's ~1 s velocity
# re-seed heal), moving windows only. REPORTED, NEVER SCORED: across
# three plants it bears no reliable relation to the speed deficit
# (over-reads stock 2x, under-reads joint-partition 7x, inverted
# ordering) and its jackknife SE is the size of the plant-to-plant
# spread — speed_gain stays the deficit's measure (README 4).
BIAS_FIT_S = (1.0, 2.0)
BIAS_MOVING_CMD = 0.3  # windows with mean |cmd| below this dilute the bias

# The five headline components of the DIAGNOSTIC detail table -> curve.
# DIAGNOSTIC, NEVER SCORED (the tracking areas above are the headline):
# snapping resets accumulation, and the free-rollout curves showed the
# snapped rates read local wander the command schedule re-absorbs — the
# right tool for local response fidelity with clean initial conditions,
# the wrong one for the judge. Attitude reads the TRACKER curve (windowed
# pose is the tracker's quantity, README 6); IMU curves print beside it.
DIVERGENCE_SCORED: dict[str, str] = {
    "along": "along",
    "cross": "cross",
    "yaw": "yaw_trk",
    "pitch": "pitch_trk",
    "roll": "roll_trk",
}

# Everything measured; attitude against both instruments (any gap between
# them IS the measurement uncertainty, published not hidden).
DIVERGENCE_TERMS = (
    "along",
    "cross",
    "pos",
    "yaw_trk",
    "pitch_trk",
    "roll_trk",
    "yaw_imu",
    "pitch_imu",
    "roll_imu",
)


def _wrap(a: np.ndarray | float) -> np.ndarray | float:
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def reinit_schedule(
    st: Streams, *, start: float, seconds: float, T: float, settle: float = 0.5
) -> list[State]:
    """Measured states every ``T`` seconds — loop 2's multiple shooting.

    The first snap fires at ``start + settle`` (the free loop's own settle
    time, before which the policy is not acting), then every ``T``; windows
    run between consecutive snaps.
    """
    base_p = base_r = None
    if st.has_markers:
        base_p, base_r = st.base_pose_room(mount_matrix(), TRACKER_Z)
    times = np.arange(start + settle, start + seconds - 1e-9, T)
    return [measured_state(st, float(t), base_p=base_p, base_r=base_r) for t in times]


def _local_mean(t: np.ndarray, x: np.ndarray, at: float, half: float = 0.25) -> float:
    """Mean of ``x`` within ``at +- half`` seconds — smooths the tracker's
    instantaneous yaw wobble out of the window anchor."""
    sel = (t >= at - half) & (t <= at + half)
    return float(np.mean(x[sel])) if sel.any() else float(np.interp(at, t, x))


def window_curves(run: PolicyRun, st: Streams, *, start: float) -> dict[str, np.ndarray]:
    """Signed error vs TIME SINCE RE-INIT, per window, for ONE snapped rollout.

    Returns ``term -> (n_windows, m)`` on a ``CONTROL_DT`` tau grid. Pooling
    over windows started at random gait phases averages the ~2 Hz
    oscillation out of the trend a single endpoint would ride.

    The JUDGE is tracker-only: position, the along/cross heading frame and
    the window anchor all come from the tracker (yaw locally smoothed at the
    snap so its instantaneous wobble cannot rotate the anchor), and the
    scored attitude curves are ``*_trk`` — windowed absolute pose is the
    tracker's quantity (README 6). The ``*_imu`` curves are a printed
    cross-check only; the IMU otherwise enters nothing but the snap's
    initial condition. ``yaw_*`` are window increments (the absolute yaws
    live in different frames); ``pitch_*``/``roll_*`` absolute (the snap
    equalised them at tau 0); ``along`` is signed by the direction of real
    motion (negative = falls behind — without the flip, backward spans
    cancel the stride deficit out of the bias). Position terms are NaN
    without a tracker.
    """
    n_win = len(run.reinit_t) - 1
    if n_win < 1:
        return {k: np.zeros((0, 0)) for k in DIVERGENCE_TERMS}
    m = max(round(float(np.min(np.diff(run.reinit_t))) / CONTROL_DT) - 1, 1)
    out = {k: np.full((n_win, m), np.nan) for k in DIVERGENCE_TERMS}
    lyaw = np.unwrap(yaw_of(st.lquat))
    pitch_i, roll_i = pitch_roll_of(st.lquat)
    pitch_s, roll_s = pitch_roll_of(run.quat)
    yaw_s = np.unwrap(yaw_of(run.quat))
    snap_yaw = yaw_of(run.reinit_quat)
    has_pos = st.has_markers
    if has_pos:
        base_p, base_r = st.base_pose_room(mount_matrix(), TRACKER_Z)
        rq = np.stack([mat_to_quat(r) for r in base_r])
        yaw_room = np.unwrap(yaw_of(rq))
        pitch_t, roll_t = pitch_roll_of(rq)
    for k in range(n_win):
        t0 = float(run.reinit_t[k])
        i0 = int(np.searchsorted(run.t, t0 - 1e-9, "left"))
        if i0 + m > len(run.t):
            continue
        a0 = t0 + start
        # The exact snap yaw on the run's unwrapped branch (the snap
        # preserves yaw, so the series is continuous through it).
        psi0s = float(yaw_s[i0] + _wrap(float(snap_yaw[k]) - float(_wrap(yaw_s[i0]))))
        psi0i = float(np.interp(a0, st.lt, lyaw))
        if has_pos:
            a_yaw = psi0s - _local_mean(st.vt, yaw_room, a0)
            c_, s_ = np.cos(a_yaw), np.sin(a_yaw)
            rot2 = np.array([[c_, -s_], [s_, c_]])
            p_room0 = np.array(
                [np.interp(a0, st.vt, base_p[:, 0]), np.interp(a0, st.vt, base_p[:, 1])]
            )
            a_p = run.reinit_pos[k][:2] - rot2 @ p_room0
            psi0t = float(np.interp(a0, st.vt, yaw_room))
        for j in range(m):
            i = i0 + j
            a = float(run.t[i]) + start
            li = int(np.clip(np.searchsorted(st.lt, a, "right") - 1, 0, len(st.lt) - 1))
            dpsi_s = float(yaw_s[i]) - psi0s
            dpsi_i = float(np.interp(a, st.lt, lyaw)) - psi0i
            out["yaw_imu"][k, j] = float(_wrap(dpsi_s - dpsi_i))
            out["pitch_imu"][k, j] = float(pitch_s[i] - pitch_i[li])
            out["roll_imu"][k, j] = float(roll_s[i] - roll_i[li])
            if has_pos:
                vi = int(np.clip(np.searchsorted(st.vt, a, "right") - 1, 0, len(st.vt) - 1))
                dpsi_t = float(np.interp(a, st.vt, yaw_room)) - psi0t
                out["yaw_trk"][k, j] = float(_wrap(dpsi_s - dpsi_t))
                out["pitch_trk"][k, j] = float(pitch_s[i] - pitch_t[vi])
                out["roll_trk"][k, j] = float(roll_s[i] - roll_t[vi])
                p_room = np.array(
                    [np.interp(a, st.vt, base_p[:, 0]), np.interp(a, st.vt, base_p[:, 1])]
                )
                e = run.pos[i][:2] - (rot2 @ p_room + a_p)
                h = psi0s + dpsi_t  # the robot's heading in the sim frame
                u = np.array([np.cos(h), np.sin(h)])
                out["pos"][k, j] = float(np.linalg.norm(e))
                disp = (rot2 @ (p_room - p_room0)) @ u
                flip = 1.0 if disp >= 0 else -1.0
                out["along"][k, j] = float(flip * (e @ u))
                out["cross"][k, j] = float(e @ np.array([-u[1], u[0]]))
    return out


@dataclass(frozen=True)
class TermRate:
    """One term's divergence rate: slope of pooled error vs time-since-reinit.

    ``rate``: slope of ``E(tau) = b + a·tau`` (RMS over windows of the
    signed error) over the scored interval — unit/s, the intercept
    absorbing instrument noise. Includes what chaos expresses inside the
    interval (no chaos reference, by design); comparable across plants
    because every plant sees identical windows. ``rates``: the same fit
    per :data:`DIVERGENCE_FIT_S` interval — the stability check. ``se``:
    jackknife-over-windows SE. ``bias_rate``: slope of the SIGNED
    window-mean — the provably systematic part (for ``along``, the stride
    deficit; negative = the sim falls behind).
    """

    rate: float
    rates: tuple[float, ...]
    se: float
    bias_rate: float
    bias_se: float = float("nan")  # jackknife over moving windows


def _rate_of(x: np.ndarray, tau: np.ndarray, moving: np.ndarray | None = None) -> TermRate:
    """The rate fit for one term's ``(windows, m)`` signed-error stack."""

    def slope(rows: np.ndarray, lo: float, hi: float, signed: bool = False) -> float:
        sel = (tau >= lo) & (tau <= hi)
        curve = (
            np.nanmean(rows[:, sel], axis=0)
            if signed
            else np.sqrt(np.nanmean(rows[:, sel] ** 2, axis=0))
        )
        ok = np.isfinite(curve)
        if ok.sum() < 4:
            return float("nan")
        a, _b = np.polyfit(tau[sel][ok], curve[ok], 1)
        return float(a)

    valid = ~np.all(np.isnan(x), axis=1)
    mov = valid if moving is None else (valid & moving[: len(valid)])
    xm = x[mov]
    x = x[valid]
    n = x.shape[0]
    if n == 0:
        nan = float("nan")
        return TermRate(nan, (nan,) * len(DIVERGENCE_FIT_S), nan, nan)
    rate = slope(x, 0.0, DIVERGENCE_SCORED_FIT_S)
    rates = tuple(slope(x, 0.0, f) for f in DIVERGENCE_FIT_S)
    jack = np.array(
        [slope(x[[i for i in range(n) if i != k]], 0.0, DIVERGENCE_SCORED_FIT_S) for k in range(n)]
    )
    jack = jack[np.isfinite(jack)]
    se = (
        float(np.sqrt((len(jack) - 1) / len(jack) * np.sum((jack - jack.mean()) ** 2)))
        if len(jack) > 2
        else float("nan")
    )
    b_lo, b_hi = BIAS_FIT_S
    b_hi = min(b_hi, float(tau[-1]))
    bias = slope(xm, b_lo, b_hi, signed=True) if len(xm) else float("nan")
    bj = np.array(
        [
            slope(xm[[i for i in range(len(xm)) if i != k]], b_lo, b_hi, signed=True)
            for k in range(len(xm))
        ]
    )
    bj = bj[np.isfinite(bj)]
    bias_se = (
        float(np.sqrt((len(bj) - 1) / len(bj) * np.sum((bj - bj.mean()) ** 2)))
        if len(bj) > 2
        else float("nan")
    )
    return TermRate(rate, rates, se, bias, bias_se)


@dataclass(frozen=True)
class Divergence:
    """The local-response diagnostic: one :class:`TermRate` per term, from
    a single UNREPLICATED snapped rollout — licensed by measurement, not
    by determinism (three rollouts under the verdict replicates' own
    +-3 deg perturbations move every rate by LESS than its own jackknife
    SE: spreads 0.001-0.019 against SEs 0.012-0.064 on the shipped
    plant). NEVER SCORED — the headline is :class:`Tracking` — printed
    below every table as the short-horizon response check."""

    window_s: float
    n_windows: int
    terms: dict[str, TermRate]

    def floor_of(self, name: str) -> float:
        se = self.terms[name].se
        return max(se, 1e-6) if np.isfinite(se) else float("nan")


def aggregate_divergence(
    curves: dict[str, np.ndarray], window_s: float, moving: np.ndarray | None = None
) -> Divergence:
    """Fold one snapped rollout's window curves into the rate verdict.

    ``moving`` marks windows where the robot was commanded to move; only
    the signed bias uses it (standing windows dilute a signed mean).
    """
    n_win = curves[DIVERGENCE_TERMS[0]].shape[0]
    n_tau = curves[DIVERGENCE_TERMS[0]].shape[1] if n_win else 0
    tau = (np.arange(n_tau) + 1) * CONTROL_DT
    terms = {t: _rate_of(curves[t], tau, moving) for t in DIVERGENCE_TERMS}
    return Divergence(window_s=window_s, n_windows=n_win, terms=terms)


def moving_windows(
    st: Streams, states: Sequence[State], *, threshold: float = BIAS_MOVING_CMD
) -> np.ndarray:
    """Which re-init windows carry a real motion command, per the operator log."""
    t0s = np.array([s.t for s in states])
    out = np.zeros(max(len(t0s) - 1, 0), bool)
    for k in range(len(out)):
        c = cmd_at(st, np.arange(t0s[k], t0s[k + 1], 0.1))
        out[k] = float(np.mean(np.linalg.norm(c[:, :2], axis=1))) > threshold
    return out


def _stability_note(tc: TermRate) -> str:
    """Name the interval-disagreement pattern (see DIVERGENCE_FIT_S): a
    0.5 s rate far above the scored one is the gait-envelope rise, not
    divergence; rates growing with interval are chaos entering."""
    if not (np.isfinite(tc.se) and all(np.isfinite(r) for r in tc.rates)):
        return ""
    tol = 2 * tc.se
    if max(tc.rates) - min(tc.rates) <= tol:
        return ""  # interval-stable: the linear regime holds
    short, long_ = tc.rates[0], tc.rates[-1]
    if short > tc.rate + tol and long_ <= tc.rate + tol:
        return "; 0.5s rides the gait-oscillation envelope, scored interval sits past it"
    if list(tc.rates) == sorted(tc.rates):
        return "; rates GROW with interval — chaos entering, trust the shortest"
    return "; INTERVALS DISAGREE — quote the scored rate with its SE, not alone"


def divergence_detail(div: Divergence) -> str:
    """The whole divergence judgement, term by term — nothing unreported.
    ``along`` leads (the stride-deficit progress measure, README 9); the
    ``*_trk``/``*_imu`` pairs put both instruments side by side."""
    scored = {v: k for k, v in DIVERGENCE_SCORED.items()}
    unit = {t: ("m/s" if t in ("along", "cross", "pos") else "rad/s") for t in DIVERGENCE_TERMS}
    ivals = "/".join(f"{f:g}" for f in DIVERGENCE_FIT_S)
    lines = [
        f"LOCAL RESPONSE (diagnostic, not scored)  {div.n_windows} windows of "
        f"{div.window_s:g}s, one snapped rollout; rate over {DIVERGENCE_SCORED_FIT_S:g}s, "
        "jackknife SE over windows — short-horizon response check, the headline "
        "is the tracking area above",
        f"  {'term':<11} {'rate':>10} {'':<6} {'se':>8}   rate over {ivals}s  note",
    ]
    for t in DIVERGENCE_TERMS:
        tc = div.terms[t]
        if not np.isfinite(tc.rate):
            lines.append(f"  {t:<11} {'n/a':>10}  (no measured side: no tracker)")
            continue
        triple = " ".join(f"{r:+8.4f}" for r in tc.rates)
        deg = f" ({np.degrees(tc.rate):+.2f} deg/s)" if unit[t] == "rad/s" else ""
        note = "headline component" if t in scored else "secondary"
        note += _stability_note(tc)
        if t == "along":
            note += (
                f"; signed bias {tc.bias_rate:+.3f} +- {tc.bias_se:.3f} m/s over "
                f"{BIAS_FIT_S[0]:g}-{BIAS_FIT_S[1]:g}s moving windows (reported only: "
                "no reliable cross-plant relation to the deficit — README 4)"
            )
        lines.append(
            f"  {t:<11} {tc.rate:+10.4f} {unit[t]:<6} {tc.se:8.4f}   {triple} {deg} {note}"
        )
    for axis in ("yaw", "pitch", "roll"):
        a, b = div.terms.get(f"{axis}_trk"), div.terms.get(f"{axis}_imu")
        if a and b and np.isfinite(a.rate) and np.isfinite(b.rate):
            gap = abs(a.rate - b.rate)
            res = max(div.floor_of(f"{axis}_trk"), div.floor_of(f"{axis}_imu"))
            verdict = "immaterial" if gap <= 2 * res else "REAL — publish the gap"
            lines.append(
                f"  {axis}: tracker vs IMU rates differ by {np.degrees(gap):.2f} deg/s "
                f"against a {np.degrees(res):.2f} deg/s SE — instrument choice {verdict}"
            )
    return "\n".join(lines)


# Worker-process state for ground's replicate fan-out (probe.py's pattern:
# spawned, never forked; everything arrives PICKLED once per worker via the
# initializer — the backend by seam contract, the parsed streams and policy
# because re-reading them per worker would be pure waste).
_GROUND_WORKER: dict[str, object] = {}


def _init_ground_worker(st: Streams, policy: FreePolicy, backend: ClosedLoopBackend) -> None:
    _GROUND_WORKER["st"] = st
    _GROUND_WORKER["policy"] = policy
    _GROUND_WORKER["backend"] = backend


def _free_one(
    st: Streams,
    policy: FreePolicy,
    preset: Preset,
    backend: ClosedLoopBackend,
    seed: int,
    start: float,
    seconds: float,
    mechanisms: dict[str, object],
) -> tuple[Summary, dict[str, np.ndarray]]:
    """One free verdict replicate — pure function, serial == parallel.

    Returns the summary AND the tracking-error curves: the same rollout
    feeds the eleven statistics and the tracking areas, so the headline
    costs no extra physics.
    """
    rng = np.random.default_rng(seed)
    run = rollout_policy(
        st,
        policy,
        preset,
        backend,
        start=start,
        seconds=seconds,
        perturb=rng.normal(0.0, PERTURB_RAD, 12),
        **mechanisms,  # type: ignore[arg-type]
    )
    return sim_summary(run, cmd_at(st, run.t + start)), tracking_curves(run, st, start=start)


def _curves_one(
    st: Streams,
    policy: FreePolicy,
    preset: Preset,
    backend: ClosedLoopBackend,
    start: float,
    seconds: float,
    mechanisms: dict[str, object],
    states: list[State],
) -> dict[str, np.ndarray]:
    """THE snapped divergence rollout — deterministic, no perturbation.

    The rate pools over windows, not replicate draws, and an injected
    perturbation would ride the very slope being measured.
    """
    run = rollout_policy(
        st,
        policy,
        preset,
        backend,
        start=start,
        seconds=seconds,
        reinit=states,
        **mechanisms,  # type: ignore[arg-type]
    )
    return window_curves(run, st, start=start)


def _free_in_worker(
    preset: Preset, seed: int, start: float, seconds: float, mechanisms: dict[str, object]
) -> tuple[Summary, dict[str, np.ndarray]]:
    st, policy, backend = _GROUND_WORKER["st"], _GROUND_WORKER["policy"], _GROUND_WORKER["backend"]
    assert isinstance(st, Streams) and isinstance(policy, FreePolicy)
    return _free_one(st, policy, preset, backend, seed, start, seconds, mechanisms)  # type: ignore[arg-type]


def _curves_in_worker(
    preset: Preset,
    start: float,
    seconds: float,
    mechanisms: dict[str, object],
    states: list[State],
) -> dict[str, np.ndarray]:
    st, policy, backend = _GROUND_WORKER["st"], _GROUND_WORKER["policy"], _GROUND_WORKER["backend"]
    assert isinstance(st, Streams) and isinstance(policy, FreePolicy)
    return _curves_one(st, policy, preset, backend, start, seconds, mechanisms, states)  # type: ignore[arg-type]


# ------------------------------------------------------------- noise floors


def sim_noise(
    st: Streams,
    policy: FreePolicy,
    preset: Preset,
    backend: ClosedLoopBackend,
    *,
    seeds: int = 4,
    start: float = 6.0,
    seconds: float | None = None,
    **kw: object,
) -> dict[str, float]:
    """Noise floor measured by perturbing the SIM's initial pose.

    What chaos alone does to each statistic. Measure once per (recording,
    preset) and reuse: it is a property of the system, not of any one
    comparison, and holding it fixed keeps losses comparable between trials.
    """
    runs = []
    for seed in range(seeds):
        rng = np.random.default_rng(seed)
        run = rollout_policy(
            st,
            policy,
            preset,
            backend,
            start=start,
            seconds=seconds,
            perturb=rng.normal(0.0, PERTURB_RAD, 12),
            **kw,  # type: ignore[arg-type]
        )
        runs.append(sim_summary(run, cmd_at(st, run.t + start)))
    return spread_of(runs)


def usable_floor(
    raw: dict[str, float],
    real: dict[str, float],
    cross: dict[str, float] | None = None,
) -> dict[str, float]:
    """Clamp a collapsed floor so a well-resolved statistic cannot dominate.

    The floor is a resolution limit, and a well-stabilised policy can drive
    one to ~0, sending its SNR to infinity and making the loss meaningless.
    Clamping to the same statistic's floor on ANOTHER recording plus 5% of
    the real value keeps every term finite and comparable.
    """
    cross = cross or {}
    return {
        k: max(v, cross.get(k, 0.0), 0.05 * abs(real.get(k, 0.0)), 1e-4) for k, v in raw.items()
    }


# ------------------------------------------------------------------ report


@dataclass
class Report:
    """One plant's grounding: sim vs real statistics over their noise floor.

    ``sim`` is the VERDICT summary — the per-statistic median over the
    replicate rollouts in ``sims`` (README 4a applied to loop 2: a 40 s
    contact-rich rollout is one draw of a chaotic system, and a single draw
    resamples across ~±10% of the loss under a 1e-7 anchor wiggle, README
    8). ``sims`` carries the replicates so the spread can always be quoted
    beside the point; an empty ``sims`` means a single-rollout report (a
    probe cell, a quick look) and prints without a spread.
    """

    preset: str
    sim: Summary
    real: Summary
    noise: dict[str, float]
    floor_source: str  # "sim-perturb" | "robot-repeat" — part of any claim
    start: float
    seconds: float
    sims: list[Summary] = field(default_factory=list)
    # Tracking areas (README 4, the headline trajectory terms): mean
    # |error| of the FREE rollouts per heading-aligned component,
    # median-of-n with the chaos floor from the same rollouts.
    tracking: Tracking | None = None
    # The snapped local-response rates: DIAGNOSTIC ONLY, printed below the
    # table, never scored — snapping resets the accumulation the headline
    # measures.
    divergence: Divergence | None = None

    def snr(self) -> dict[str, float]:
        """Sim-real difference over the statistic's own floor, per statistic.

        A NaN on either side (a tracker-less real side has no position
        statistics) means NOT COMPARABLE on this recording — the statistic is
        left out entirely rather than scored, matched, or counted. The
        tracking areas ride along under a ``trk_`` prefix; the snapped
        rates never appear here (diagnostic, not scored).
        """
        s, r = self.sim.as_dict(), self.real.as_dict()
        out = {}
        for k in s:
            if k in NOT_COMPARABLE or not (np.isfinite(s[k]) and np.isfinite(r[k])):
                continue
            n = self.noise.get(k, 0.0)
            out[k] = abs(s[k] - r[k]) / n if n > 1e-9 else float("inf")
        if self.tracking is not None:
            for k, v in self.tracking.snr().items():
                out[f"trk_{k}"] = v
        return out

    def loss(self) -> float:
        """RMS over valid SNRs — the single number the meta-search minimises."""
        vals = [v for v in self.snr().values() if np.isfinite(v)]
        return float(np.sqrt(np.mean([v * v for v in vals]))) if vals else float("inf")

    def n_matched(self) -> tuple[int, int]:
        """How many statistics sit within their own floor: the (N, of) pair
        in "the sim differs from the robot by less than the robot differs
        from itself on N of M statistics" — meaningful as that claim only
        when ``floor_source`` is robot-repeat."""
        snr = self.snr()
        return sum(1 for v in snr.values() if v <= 1.0), len(snr)

    def _replicate(self, i: int) -> Report:
        """The report a SINGLE draw would have produced: replicate ``i``'s
        summary and its own tracking areas (floors stay shared)."""
        trk = self.tracking
        if trk is not None:
            trk = replace(trk, areas={t: v[i : i + 1] for t, v in trk.areas.items()})
        return replace(self, sim=self.sims[i], sims=[], tracking=trk)

    def replicate_losses(self) -> list[float]:
        """Each replicate's own loss — what a single draw COULD have read."""
        return [self._replicate(i).loss() for i in range(len(self.sims))]

    def loss_range(self) -> tuple[float, float]:
        """Min-max of the per-replicate losses (empty ``sims``: the point twice)."""
        ls = self.replicate_losses() or [self.loss()]
        return min(ls), max(ls)

    def matched_range(self) -> tuple[int, int]:
        """Min-max of per-replicate "k of M" counts."""
        ks = [self._replicate(i).n_matched()[0] for i in range(len(self.sims))] or [
            self.n_matched()[0]
        ]
        return min(ks), max(ks)

    def _draw_range(self, k: str) -> str:
        """min..max of what single replicate draws read for statistic ``k``."""
        if k.startswith("trk_"):
            if self.tracking is None:
                return ""
            lo, hi = self.tracking.draws(k[4:])
            return f"{lo:.3f}..{hi:.3f}"
        if not self.sims:
            return ""
        vals = [x.as_dict()[k] for x in self.sims]
        return f"{min(vals):.3f}..{max(vals):.3f}"

    def table(self) -> str:
        s, r = self.sim.as_dict(), self.real.as_dict()
        snr = self.snr()
        n, of = self.n_matched()
        trk = self.tracking
        lines = [
            f"preset {self.preset}  {self.seconds:.0f}s from t={self.start:.0f}s  "
            f"floor: {self.floor_source}  real[{self.real.source or '?'}]"
            + (
                f"  tracking: mean |error| over {trk.horizon_s:.0f}s, "
                f"chaos floor = pairwise sim-sim / sqrt2"
                if trk
                else ""
            ),
            f"{'statistic':>14} {'sim':>9} {'real':>9} {'floor':>9} {'SNR':>7}  {'draws':>14}",
        ]

        def row(k: str, sim_v: float, real_v: float, floor_v: float, snr_v: float) -> str:
            return (
                f"{k:>14} {sim_v:9.3f} {real_v:9.3f} {floor_v:9.3f} {snr_v:7.1f}"
                f"  {self._draw_range(k):>14}"
            )

        for k in sorted(snr, key=lambda k: -snr[k]):
            if k.startswith("trk_"):
                assert trk is not None
                term = k[4:]
                lines.append(row(k, trk.median(term), 0.0, trk.floor_of(term), snr[k]))
            else:
                lines.append(row(k, s[k], r[k], self.noise.get(k, 0.0), snr[k]))
        for k in NOT_COMPARABLE:
            lines.append(f"{k:>14} {s[k]:9.3f} {r[k]:9.3f} {'--':>9} {'n/a':>7}")
        if trk is not None:
            for term in TRACKING_TERMS:
                if term in TRACKING_SCORED or term not in trk.areas:
                    continue
                lines.append(
                    f"{'trk_' + term:>14} {trk.median(term):9.3f} {0.0:9.3f} {'--':>9} "
                    f"{'n/a':>7}  {self._draw_range('trk_' + term):>14}   "
                    f"(instrument-floored at {TRACKING_INSTRUMENT_FLOOR[term]:.3f} rad — "
                    "reported, not scored)"
                )
        if self.divergence is not None:
            lines += ["", divergence_detail(self.divergence)]
        if self.sims:
            lo, hi = self.loss_range()
            klo, khi = self.matched_range()
            lines.append(
                f"{'loss':>14} {self.loss():9.2f}   ({n} of {of} within the floor)  "
                f"[median of {len(self.sims)} replicates; "
                f"single draws read {lo:.2f}-{hi:.2f}, {klo}-{khi} of {of}]"
            )
        else:
            lines.append(
                f"{'loss':>14} {self.loss():9.2f}   ({n} of {of} within the floor)  "
                "[SINGLE ROLLOUT — a draw, not a verdict]"
            )
        return "\n".join(lines)


def ground(
    st: Streams,
    policy: FreePolicy,
    preset: Preset,
    backend: ClosedLoopBackend,
    *,
    start: float = 6.0,
    seconds: float | None = None,
    noise: dict[str, float] | None = None,
    floor_source: str = "sim-perturb",
    replicates: int = 8,
    action_latency: float = 0.0,
    obs_noise: ObsNoise | None = None,
    control_intervals: np.ndarray | None = None,
    envelope: TorqueEnvelope | None = None,
    divergence_window: float | None = None,
    workers: int = 1,
    view: bool = False,
    speed: float = 1.0,
    with_ghost: bool = True,
) -> Report:
    """THE loop-2 call: a REPLICATED closed-loop verdict, with its spread.

    The verdict is ``replicates`` rollouts from perturbed initial poses
    (README 4a applied to loop 2): the report's ``sim`` is the per-statistic
    MEDIAN and the replicates ride along so the loss is always quotable as a
    spread. One rollout of a contact-rich 40 s gait is a single draw of a
    chaotic system — a 1e-7 anchor wiggle resamples it across ~0.74-0.90 in
    loss (README 8) — so a single draw is a LOOK, never a verdict.
    ``replicates=1`` is that look, and the report says so.

    ``noise=None`` reuses the SAME replicate rollouts as the sim-perturb
    floor (their spread is exactly what :func:`sim_noise` measured
    separately before); pass a floor from :func:`robot_noise` to ground
    against the robot's own variability instead — the ``floor_source``
    string travels with the report so no claim silently upgrades itself.
    The default-off loop mechanisms (``action_latency``, ``obs_noise``,
    ``envelope`` — see :func:`rollout_policy`) apply to every replicate, so
    a mechanism is judged against its own chaos, not the ideal loop's.

    The tracking areas (the scored trajectory terms) come from the SAME
    free rollouts as the eleven statistics — no extra physics.
    ``divergence_window`` adds the snapped local-response DIAGNOSTIC (one
    rollout of its own, printed, never scored). ``workers`` fans rollouts
    across processes, bit-identical to serial; a viewing run stays serial.
    """
    if replicates < 1:
        raise ValueError("replicates must be >= 1")
    span = float(st.wt[-1]) - start
    seconds = span if seconds is None else seconds
    mechanisms: dict[str, object] = {
        "action_latency": action_latency,
        "obs_noise": obs_noise,
        "control_intervals": control_intervals,
        "envelope": envelope,
    }
    states = (
        reinit_schedule(st, start=start, seconds=seconds, T=divergence_window)
        if divergence_window
        else None
    )
    snap_curves: dict[str, np.ndarray] | None = None
    if view or workers <= 1:
        sims = []
        curve_sets = []
        for seed in range(replicates):
            rng = np.random.default_rng(seed)
            run = rollout_policy(
                st,
                policy,
                preset,
                backend,
                start=start,
                seconds=seconds,
                perturb=rng.normal(0.0, PERTURB_RAD, 12),
                view=view and seed == 0,
                speed=speed,
                ghost=ghost_track(st) if with_ghost and view and seed == 0 else None,
                **mechanisms,  # type: ignore[arg-type]
            )
            sims.append(sim_summary(run, cmd_at(st, run.t + start)))
            curve_sets.append(tracking_curves(run, st, start=start))
        if states is not None:
            snap_curves = _curves_one(
                st, policy, preset, backend, start, seconds, mechanisms, states
            )
    else:
        import concurrent.futures
        import multiprocessing

        with concurrent.futures.ProcessPoolExecutor(
            max_workers=workers,
            mp_context=multiprocessing.get_context("spawn"),
            initializer=_init_ground_worker,
            initargs=(st, policy, backend),
        ) as pool:
            free = [
                pool.submit(_free_in_worker, preset, seed, start, seconds, mechanisms)
                for seed in range(replicates)
            ]
            cf = (
                pool.submit(_curves_in_worker, preset, start, seconds, mechanisms, states)
                if states is not None
                else None
            )
            results = [f.result() for f in free]
            sims = [s for s, _c in results]
            curve_sets = [c for _s, c in results]
            snap_curves = cf.result() if cf is not None else None
    real = real_summary(st, start=start, seconds=seconds)
    if noise is None:
        noise = usable_floor(spread_of(sims), real.as_dict())
        floor_source = "sim-perturb"
    div = (
        aggregate_divergence(snap_curves, divergence_window, moving_windows(st, states))
        if snap_curves is not None and divergence_window and states is not None
        else None
    )
    return Report(
        preset=preset.name,
        sim=median_summary(sims),
        real=real,
        noise=noise,
        floor_source=floor_source,
        start=start,
        seconds=seconds,
        sims=sims if replicates > 1 else [],
        tracking=tracking_of(curve_sets, seconds),
        divergence=div,
    )


def main() -> None:
    ap = argparse.ArgumentParser(prog="go2.sim.sysid.ground")
    ap.add_argument("recording")
    ap.add_argument("policy_bin", help="the net that PRODUCED the recording (verify_net first)")
    ap.add_argument("--preset", default=DEFAULT_PRESET, help="plant preset name or JSON path")
    ap.add_argument("--start", type=float, default=6.0)
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument(
        "--replicates",
        type=int,
        default=8,
        help="perturbed verdict rollouts: the report is their per-statistic median "
        "with the loss spread beside it; 1 = a single-draw LOOK, labelled as such "
        "(when no --noise-from floor is given, the same rollouts are the sim-perturb floor)",
    )
    ap.add_argument(
        "--noise-from",
        nargs="+",
        default=None,
        metavar="REC",
        help="two or more repeat recordings of the same walk: floor = the robot "
        "against itself. The grounded recording itself stays OUT of the floor, "
        "so the floor and the verdict are measured on different data.",
    )
    ap.add_argument(
        "--latency",
        type=float,
        default=0.0,
        metavar="S",
        help="loop latency: policy target reaches the PD this many seconds late",
    )
    ap.add_argument(
        "--obs-noise",
        default="0",
        metavar="SCALE",
        help="observation noise: a multiple of the training levels (see ObsNoise), "
        "or 'measured' for the recording's own >20 Hz sensor floor",
    )
    ap.add_argument(
        "--timing",
        default="ideal",
        choices=("ideal", "recorded"),
        help="policy tick schedule: the ideal 20 ms grid, or the recording's own "
        "measured policy/lowcmd intervals (rate, jitter and dropouts included)",
    )
    ap.add_argument(
        "--envelope",
        default=None,
        choices=sorted(TORQUE_ENVELOPES),
        help="apply a measured torque envelope to the actuators",
    )
    ap.add_argument(
        "--divergence",
        type=float,
        default=DIVERGENCE_WINDOW_S,
        metavar="WINDOW_S",
        help="re-init window for the snapped local-response DIAGNOSTIC "
        "(0 disables it; the scored tracking areas come from the free rollouts)",
    )
    ap.add_argument(
        "--workers",
        type=int,
        default=1,
        help="fan the replicate rollouts across processes (bit-identical to serial)",
    )
    ap.add_argument(
        "--engine",
        choices=("mujoco", "mjx"),
        default="mujoco",
        help="which backend hosts the loop. At a converged solver they are the "
        "same plant; at a truncated one (the fast preset's 1/5) they are NOT, "
        "and grading the mjx flavour is grading the plant that trains",
    )
    ap.add_argument(
        "--f32",
        action="store_true",
        help="mjx only: run the TRAINING dtype instead of the plant's float64",
    )
    ap.add_argument("--view", action="store_true", help="watch it against the recorded ghost")
    ap.add_argument("--speed", type=float, default=1.0, help="viewer playback rate")
    args = ap.parse_args()

    backend: ClosedLoopBackend
    if args.engine == "mjx":
        if args.view:
            ap.error("--view is a MuJoCo viewer; mjx has no scene to attach it to")
        from dimos.robot.unitree.go2.sim.engines.mjx import MjxBackend

        backend = MjxBackend(x64=not args.f32)
    else:
        if args.f32:
            ap.error("--f32 is an mjx dtype choice; the CPU engine is float64 only")
        from dimos.robot.unitree.go2.sim.engines.mujoco import MujocoBackend

        backend = MujocoBackend()

    st = read_streams(args.recording)
    policy = FreePolicy.load(args.policy_bin)
    preset = load_preset(args.preset)
    obs_noise: ObsNoise | None
    if args.obs_noise == "measured":
        from dimos.robot.unitree.go2.sim.sysid.loop import sensor_noise

        span = float(st.wt[-1])
        obs_noise = sensor_noise(st, t0=args.start, t1=span).obs_noise()
    else:
        scale = float(args.obs_noise)
        obs_noise = ObsNoise().scaled(scale) if scale > 0 else None
    intervals = None
    if args.timing == "recorded":
        from dimos.robot.unitree.go2.sim.sysid.loop import timing_of

        intervals = timing_of(st).intervals
    noise = None
    source = "sim-perturb"
    if args.noise_from:
        repeats = [read_streams(r) for r in args.noise_from]
        raw = robot_noise(repeats, start=args.start, seconds=args.seconds)
        span = float(st.wt[-1]) - args.start
        real = real_summary(
            st, start=args.start, seconds=span if args.seconds is None else args.seconds
        )
        noise = usable_floor(raw, real.as_dict())
        source = "robot-repeat"
    report = ground(
        st,
        policy,
        preset,
        backend,
        start=args.start,
        seconds=args.seconds,
        noise=noise,
        floor_source=source,
        replicates=args.replicates,
        action_latency=args.latency,
        obs_noise=obs_noise,
        control_intervals=intervals,
        envelope=TORQUE_ENVELOPES[args.envelope] if args.envelope else None,
        divergence_window=args.divergence if args.divergence > 0 else None,
        workers=args.workers,
        view=args.view,
        speed=args.speed,
    )
    print(report.table())


if __name__ == "__main__":
    main()
