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
(:mod:`~dimos.robot.unitree.go2.sim.sysid.stats`). Never trajectories:
position error measures how long ago two chaotic runs diverged, not whether
the plant is right — the preset fitted on closed-loop trajectories was worse
than no overrides at all under the open-loop objective.

Each statistic is normalised by its own NOISE FLOOR: the sim disagrees with
ITSELF under chaos, so ``SNR = |sim - real| / floor``, and an SNR under ~1
means matched to within what chaos already does — no better is askable. The
floor's SOURCE is a parameter of the claim:

* :func:`sim_noise` repeats the rollout from perturbed initial poses —
  measures what chaos alone does. Available today, and the default.
* :func:`~dimos.robot.unitree.go2.sim.sysid.real.robot_noise` spreads the
  statistics across REPEAT RECORDINGS of the same walk — captures battery
  sag and motor temperature too, and is the yardstick the publishable claim
  needs: *"the simulator differs from the robot by less than the robot
  differs from itself, on N of 11 statistics."* Swapping it in is
  ``--noise-from REC2.mcap REC3.mcap``, not a rewrite.

The REAL side of the comparison (:mod:`~dimos.robot.unitree.go2.sim.sysid
.real`) is pure recording processing and lives outside this engine-coupled
module — position from the tracker, attitude from the IMU (README 5e).

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
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from dimos.robot.unitree.go2.sim.backend import GhostTrack
from dimos.robot.unitree.go2.sim.plant import (
    TORQUE_ENVELOPES,
    TORQUE_LIMITS,
    TorqueEnvelope,
    actuator_step,
)
from dimos.robot.unitree.go2.sim.policy import FreePolicy
from dimos.robot.unitree.go2.sim.ranges import Preset, load_preset
from dimos.robot.unitree.go2.sim.rotations import mat_to_quat, quat_to_mat
from dimos.robot.unitree.go2.sim.sysid.ingest import TRACKER_Z, Streams, read_streams
from dimos.robot.unitree.go2.sim.sysid.real import cmd_at, real_summary, robot_noise
from dimos.robot.unitree.go2.sim.sysid.replay import ghost_track
from dimos.robot.unitree.go2.sim.sysid.stats import NOT_COMPARABLE, Summary, spread_of, summarize

__all__ = [  # the real side (cmd_at, real_summary, robot_noise) lives in .real
    "COMMAND_SLEW",
    "CONTROL_DT",
    "NOMINAL_GAIT_HEIGHT",
    "PERTURB_RAD",
    "ObsNoise",
    "PolicyRun",
    "Report",
    "cmd_at",
    "ground",
    "projected_gravity",
    "real_summary",
    "robot_noise",
    "rollout_policy",
    "sim_noise",
    "sim_summary",
    "usable_floor",
]

CONTROL_DT = 0.02  # 50 Hz policy rate; not stored in the blob

# Per-axis slew the executor applies to operator commands before the policy
# sees them: max change in (vx, vy, vyaw) per 20 ms control tick (go2web
# policy.rs ramp_velocity). The recorded control_log carries the operator
# TARGET; the policy on hardware only ever saw the ramped command.
COMMAND_SLEW = np.array([0.05, 0.04, 0.10])

# Body height a gait-height net holds when nobody moves the slider (obs 45,
# raw metres). 45-channel nets (himloco freewalk) never see it.
NOMINAL_GAIT_HEIGHT = 0.31

PERTURB_RAD = 0.05
"""Initial-pose spread for :func:`sim_noise` — about 3 degrees.

Small on purpose: the gait is chaotic enough that this already decorrelates
position within seconds, so it measures the noise a comparison has to beat
rather than a plausible modelling error.
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
    command the policy actually saw at each sample.
    """

    t: np.ndarray
    pos: np.ndarray = field(repr=False)  # (n, 3) base position, sim world
    quat: np.ndarray = field(repr=False)  # (n, 4) wxyz
    cmd: np.ndarray = field(repr=False)  # (n, 3)
    target: np.ndarray = field(repr=False)  # (n, 12) commanded joint targets


def rollout_policy(
    st: Streams,
    policy: FreePolicy,
    preset: Preset,
    *,
    start: float = 6.0,
    seconds: float | None = None,
    settle: float = 0.5,
    command_delay: float = 0.0,
    action_latency: float = 0.0,
    obs_noise: ObsNoise | None = None,
    noise_seed: int = 0,
    control_intervals: np.ndarray | None = None,
    envelope: TorqueEnvelope | None = None,
    perturb: np.ndarray | None = None,
    menagerie: Path | None = None,
    view: bool = False,
    speed: float = 1.0,
    ghost: GhostTrack | None = None,
) -> PolicyRun:
    """Step the real policy in the candidate plant, driven by the recording.

    Closed loop, so it lives below the Mode-A seam: the policy must sit
    INSIDE the physics step loop, which :class:`RolloutPlan` (a complete
    instruction sheet decided before any physics) deliberately cannot
    express. A second backend brings its own closed-loop driver.

    ``perturb`` offsets the initial joint pose — :func:`sim_noise`'s knob.
    ``command_delay`` is kept at 0 by default: it is not identified open
    loop, and holding it identical across candidate plants cancels it from
    every between-plant comparison (the lag statistics carry the same
    constant offset on all of them).

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
    import mujoco

    from dimos.robot.unitree.go2.sim.model import (
        GHOST_BODY,
        apply_physics,
        load,
        mocap_index,
    )

    if envelope is None and preset.envelope is not None:
        envelope = TORQUE_ENVELOPES[preset.envelope]
    if len(st.wt) == 0:
        raise ValueError("recording has no control_log walk commands: Mode B needs the drive")
    span = float(st.wt[-1])
    if start >= span:
        raise ValueError(f"start={start:g}s is at or past the last command ({span:.1f}s)")
    duration = span - start if seconds is None else seconds

    ghost = ghost if view else None
    model, data = load(menagerie, ghost=ghost is not None)
    if preset.physics:
        apply_physics(model, preset.physics)
    sim_dt = float(model.opt.timestep)
    decim = max(1, round(CONTROL_DT / sim_dt))
    gi = mocap_index(model, GHOST_BODY) if ghost is not None else -1

    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")  # type: ignore[attr-defined]
    if kid >= 0:
        mujoco.mj_resetDataKeyframe(model, data, kid)
    pose0 = policy.default_pose + (perturb if perturb is not None else 0.0)
    data.qpos[7:19] = pose0
    mujoco.mj_forward(model, data)

    # Ghost anchor: rigid room -> sim-world map, fixed at the start pose.
    g_anchor_r = np.eye(3)
    g_anchor_p = np.zeros(3)
    if ghost is not None:
        j = int(np.clip(np.searchsorted(ghost.t, start, "right") - 1, 0, len(ghost.t) - 1))
        g_anchor_r = quat_to_mat(data.qpos[3:7].copy()) @ ghost.rot[j].T
        g_anchor_p = data.qpos[0:3].copy() - g_anchor_r @ ghost.pos[j]

    hist: collections.deque[np.ndarray] = collections.deque(maxlen=policy.hist)
    last_action = np.zeros(policy.act_dim)
    target = pose0.copy()
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
        q = data.qpos[7:19]
        dq = data.qvel[6:18]
        gyro = data.qvel[3:6]
        grav = projected_gravity(data.qpos[3:7])
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
    for _ in range(policy.hist):
        hist.append(observe(vel_cmd, height_at(0.0)))

    ts: list[float] = []
    pos: list[np.ndarray] = []
    quat: list[np.ndarray] = []
    used: list[np.ndarray] = []
    targets: list[np.ndarray] = []

    viewer_cm = None
    if view:
        from mujoco import viewer as mj_viewer

        viewer_cm = mj_viewer.launch_passive(model, data)
    viewer = viewer_cm.__enter__() if viewer_cm is not None else None

    try:
        import time

        wall = time.perf_counter()
        for step in range(int(duration / sim_dt)):
            t = step * sim_dt
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
                    data.mocap_pos[gi] = g_anchor_r @ ghost.pos[j] + g_anchor_p
                    data.mocap_quat[gi] = mat_to_quat(g_anchor_r @ ghost.rot[j])
                ts.append(t)
                pos.append(data.qpos[0:3].copy())
                quat.append(data.qpos[3:7].copy())
                used.append(vel_cmd.copy())
                targets.append(target.copy())

            while pending and pending[0][0] <= step:
                target = pending.popleft()[1]
            tau = policy.kp * (target - data.qpos[7:19]) - policy.kd * data.qvel[6:18]
            tau = np.clip(tau, -TORQUE_LIMITS, TORQUE_LIMITS)
            applied = actuator_step(
                applied, tau, sim_dt, preset.actuator_tau, dq=data.qvel[6:18], envelope=envelope
            )
            data.ctrl[:] = applied
            mujoco.mj_step(model, data)

            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.sync()
                wall += sim_dt / max(speed, 1e-6)
                lag = wall - time.perf_counter()
                if lag > 0:
                    time.sleep(lag)
                else:
                    wall = time.perf_counter()
    finally:
        if viewer_cm is not None:
            viewer_cm.__exit__(None, None, None)

    return PolicyRun(
        t=np.array(ts),
        pos=np.array(pos),
        quat=np.array(quat),
        cmd=np.array(used),
        target=np.array(targets),
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
    """
    rot = quat_to_mat(run.quat)
    p = run.pos.copy()
    p[:, 2] = (run.pos + rot @ np.array([0.0, 0.0, TRACKER_Z]))[:, 2]
    return replace(summarize(run.t, p, run.quat, run.cmd if cmd is None else cmd), source="sim")


# ------------------------------------------------------------- noise floors


def sim_noise(
    st: Streams,
    policy: FreePolicy,
    preset: Preset,
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
    """One plant's grounding: sim vs real statistics over their noise floor."""

    preset: str
    sim: Summary
    real: Summary
    noise: dict[str, float]
    floor_source: str  # "sim-perturb" | "robot-repeat" — part of any claim
    start: float
    seconds: float

    def snr(self) -> dict[str, float]:
        """Sim-real difference over the statistic's own floor, per statistic.

        A NaN on either side (a tracker-less real side has no position
        statistics) means NOT COMPARABLE on this recording — the statistic is
        left out entirely rather than scored, matched, or counted.
        """
        s, r = self.sim.as_dict(), self.real.as_dict()
        out = {}
        for k in s:
            if k in NOT_COMPARABLE or not (np.isfinite(s[k]) and np.isfinite(r[k])):
                continue
            n = self.noise.get(k, 0.0)
            out[k] = abs(s[k] - r[k]) / n if n > 1e-9 else float("inf")
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

    def table(self) -> str:
        s, r = self.sim.as_dict(), self.real.as_dict()
        snr = self.snr()
        n, of = self.n_matched()
        lines = [
            f"preset {self.preset}  {self.seconds:.0f}s from t={self.start:.0f}s  "
            f"floor: {self.floor_source}  real[{self.real.source or '?'}]",
            f"{'statistic':>14} {'sim':>9} {'real':>9} {'floor':>9} {'SNR':>7}",
        ]
        for k in sorted(snr, key=lambda k: -snr[k]):
            lines.append(
                f"{k:>14} {s[k]:9.3f} {r[k]:9.3f} {self.noise.get(k, 0):9.3f} {snr[k]:7.1f}"
            )
        for k in NOT_COMPARABLE:
            lines.append(f"{k:>14} {s[k]:9.3f} {r[k]:9.3f} {'--':>9} {'n/a':>7}")
        lines.append(f"{'loss':>14} {self.loss():9.2f}   ({n} of {of} within the floor)")
        return "\n".join(lines)


def ground(
    st: Streams,
    policy: FreePolicy,
    preset: Preset,
    *,
    start: float = 6.0,
    seconds: float | None = None,
    noise: dict[str, float] | None = None,
    floor_source: str = "sim-perturb",
    seeds: int = 4,
    action_latency: float = 0.0,
    obs_noise: ObsNoise | None = None,
    control_intervals: np.ndarray | None = None,
    envelope: TorqueEnvelope | None = None,
    view: bool = False,
    speed: float = 1.0,
    with_ghost: bool = True,
    menagerie: Path | None = None,
) -> Report:
    """THE loop-2 call: one closed-loop run, both summaries, the report.

    ``noise=None`` measures the sim-perturb floor here (``seeds`` extra
    rollouts); pass a floor from :func:`robot_noise` to ground against the
    robot's own variability instead — the ``floor_source`` string travels
    with the report so no claim silently upgrades itself. The default-off
    loop mechanisms (``action_latency``, ``obs_noise``, ``envelope`` — see
    :func:`rollout_policy`) apply to the floor rollouts too, so a mechanism
    is judged against its own chaos, not the ideal loop's.
    """
    span = float(st.wt[-1]) - start
    seconds = span if seconds is None else seconds
    mechanisms: dict[str, object] = {
        "action_latency": action_latency,
        "obs_noise": obs_noise,
        "control_intervals": control_intervals,
        "envelope": envelope,
    }
    run = rollout_policy(
        st,
        policy,
        preset,
        start=start,
        seconds=seconds,
        menagerie=menagerie,
        view=view,
        speed=speed,
        ghost=ghost_track(st) if with_ghost else None,
        **mechanisms,  # type: ignore[arg-type]
    )
    sim = sim_summary(run, cmd_at(st, run.t + start))
    real = real_summary(st, start=start, seconds=seconds)
    if noise is None:
        raw = sim_noise(
            st,
            policy,
            preset,
            seeds=seeds,
            start=start,
            seconds=seconds,
            menagerie=menagerie,
            **mechanisms,
        )
        noise = usable_floor(raw, real.as_dict())
        floor_source = "sim-perturb"
    return Report(
        preset=preset.name,
        sim=sim,
        real=real,
        noise=noise,
        floor_source=floor_source,
        start=start,
        seconds=seconds,
    )


def main() -> None:
    ap = argparse.ArgumentParser(prog="go2.sim.sysid.ground")
    ap.add_argument("recording")
    ap.add_argument("policy_bin", help="the net that PRODUCED the recording (verify_net first)")
    ap.add_argument("--preset", default="measured", help="plant preset name or JSON path")
    ap.add_argument("--start", type=float, default=6.0)
    ap.add_argument("--seconds", type=float, default=None)
    ap.add_argument("--seeds", type=int, default=4, help="perturbed reruns for the sim floor")
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
    ap.add_argument("--view", action="store_true", help="watch it against the recorded ghost")
    ap.add_argument("--speed", type=float, default=1.0, help="viewer playback rate")
    args = ap.parse_args()

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
        start=args.start,
        seconds=args.seconds,
        noise=noise,
        floor_source=source,
        seeds=args.seeds,
        action_latency=args.latency,
        obs_noise=obs_noise,
        control_intervals=intervals,
        envelope=TORQUE_ENVELOPES[args.envelope] if args.envelope else None,
        view=args.view,
        speed=args.speed,
    )
    print(report.table())


if __name__ == "__main__":
    main()
