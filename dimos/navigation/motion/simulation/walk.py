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

"""Run a FREE policy on the flat-ground MuJoCo Go2.

Commands come either from a constant (vx, vy, vyaw) or from a recording's
``control_log``, which is what makes a run comparable to its ``vive_pose``.
"""

from __future__ import annotations

import collections
from collections.abc import Callable
from dataclasses import dataclass, field
import json
from pathlib import Path

import mujoco
import numpy as np

from dimos.navigation.motion.simulation import model as go2_model
from dimos.navigation.motion.simulation.policy import FreePolicy

CONTROL_DT = 0.02  # 50 Hz policy rate; not stored in the blob (cfg "dt")

# Per-joint torque limits, also absent from the blob (cfg "torque_limits").
# Slightly tighter than the MJCF ctrlrange, so they bind first.
TORQUE_LIMITS = np.array([23.0, 23.0, 35.0] * 4)

# Per-axis slew the robot applies to operator commands before the policy sees
# them: max change in (vx, vy, vyaw) per 20 ms control tick (go2web policy.rs
# ramp_velocity, VEL_DV_*). The recorded control_log carries the operator
# *target*; the policy on hardware only ever saw the ramped command. This is
# what makes the real yaw answer ~0.1 s later than the real speed does -- a
# yaw reversal has further to ramp than a speed nudge -- which no uniform
# command delay can reproduce.
COMMAND_SLEW = np.array([0.05, 0.04, 0.10])

# Body height a gait-height net is commanded to hold when nobody moves the
# slider (go2web policy.rs). Fed raw in metres as obs channel 45; the blob's
# own ob_mean/ob_scale normalize it. 45-channel policies never see it.
NOMINAL_GAIT_HEIGHT = 0.31


@dataclass
class Track:
    """Simulated base trajectory, sampled at the policy rate."""

    t: np.ndarray
    pos: np.ndarray = field(repr=False)  # (n, 3)
    quat: np.ndarray = field(repr=False)  # (n, 4) wxyz
    cmd: np.ndarray = field(repr=False)  # (n, 3) vx, vy, vyaw applied
    joint_q: np.ndarray = field(repr=False)  # (n, 12) leg angles, FL FR RL RR
    foot_z: np.ndarray = field(repr=False)  # (n, 4) foot centre heights, FL FR RL RR
    target: np.ndarray = field(repr=False)  # (n, 12) commanded joint targets

    def clearance(self) -> np.ndarray:
        """Foot-ground clearance (n, 4); the foot sphere radius is 0.022."""
        out: np.ndarray = self.foot_z - 0.022
        return out


def actuator_step(applied: np.ndarray, requested: np.ndarray, dt: float, tau: float) -> np.ndarray:
    """One first-order step of the motor toward the requested torque.

    ``tau`` is the current-loop time constant in seconds; zero means the ideal
    MuJoCo motor, which delivers the request on the same step. The response to
    a step reaches 1 - 1/e of it after ``tau``.
    """
    if tau <= 0.0:
        return requested
    alpha = dt / (tau + dt)
    stepped: np.ndarray = applied + alpha * (requested - applied)
    return stepped


def projected_gravity(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_wxyz
    return np.array([-2 * (x * z - w * y), -2 * (y * z + w * x), -(1 - 2 * (x * x + y * y))])


def read_control_log(dataset: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(t, cmd)`` — seconds from run start, and (n, 3) vx/vy/vyaw.

    Only ``action == "walk"`` entries carry a velocity; everything else
    (pitch, gait_height, engage events) is ignored here.
    """
    from mcap.reader import make_reader

    ts: list[float] = []
    cmds: list[list[float]] = []
    with Path(dataset).open("rb") as f:
        for _schema, channel, msg in make_reader(f).iter_messages(topics=["control_log"]):
            if channel.topic != "control_log":
                continue
            d = json.loads(msg.data)
            if d.get("action") != "walk":
                continue
            ts.append(msg.log_time / 1e9)
            cmds.append([d.get("vx", 0.0), d.get("vy", 0.0), d.get("vyaw", 0.0)])
    if not ts:
        raise ValueError(f"{dataset}: no walk commands in control_log")
    t = np.array(ts)
    return t - t[0], np.array(cmds)


def read_gait_height(dataset: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Commanded body heights: ``(t, h)`` in metres, first-walk-command epoch.

    ``gait_height`` entries only exist when the operator moved the height
    slider against a net that listens (obs 46, e.g. v11); empty arrays
    otherwise, which :func:`walk` reads as the nominal height throughout.
    """
    from mcap.reader import make_reader

    ts: list[float] = []
    hs: list[float] = []
    first_walk: float | None = None
    with Path(dataset).open("rb") as f:
        for _schema, _channel, msg in make_reader(f).iter_messages(topics=["control_log"]):
            d = json.loads(msg.data)
            t_cmd = msg.log_time / 1e9
            if d.get("action") == "walk":
                first_walk = t_cmd if first_walk is None else min(first_walk, t_cmd)
            elif d.get("action") == "gait_height":
                ts.append(t_cmd)
                hs.append(d["gh"])
    if not ts:
        return np.array([]), np.array([])
    t = np.array(ts)
    zero = t[0] if first_walk is None else first_walk
    return t - zero, np.array(hs)


def _parse_lowcmd_q(b: bytes) -> list[float]:
    """The 12 leg-motor target angles out of one CDR-encoded LowCmd payload.

    Layout: 4-byte encapsulation, head u8[2] + levelFlag + frameReserve,
    sn u32[2], version u32[2], bandWidth u16, then MotorCmd[20] of
    {mode u8 (aligned), q dq tau kp kd f32, reserve u32[3]}.
    """
    import struct

    pos, out = 26, []
    for _ in range(12):
        pos += 1  # mode
        pos = (pos + 3) & ~3  # align f32
        out.append(struct.unpack_from("<f", b, pos)[0])
        pos += 32  # 5 f32 + reserve
    return out


def read_policy_lowcmd(dataset: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """The executor's commanded joint targets: ``(t, q)``, training order.

    ``policy/lowcmd`` is the executor's own log of what it sent the motors --
    the real policy's output at ~44 Hz, the only joint-space ground truth these
    recordings carry (``rt/lowcmd`` is zeroed on the Air). Times share the
    first-walk-command epoch like every other reader here; q is remapped from
    SDK order to the FL,FR,RL,RR training order the rest of the code uses.
    """
    from mcap.reader import make_reader

    from dimos.navigation.motion.simulation.model import (
        MUJOCO_ACTUATOR_NAMES,
        UNITREE_MOTOR_NAMES,
    )

    perm = [UNITREE_MOTOR_NAMES.index(n) for n in MUJOCO_ACTUATOR_NAMES]
    ts: list[float] = []
    qs: list[list[float]] = []
    first_walk: float | None = None
    with Path(dataset).open("rb") as f:
        for _schema, channel, msg in make_reader(f).iter_messages(
            topics=["policy/lowcmd", "control_log"]
        ):
            if channel.topic == "control_log":
                d = json.loads(msg.data)
                if d.get("action") == "walk":
                    t_cmd = msg.log_time / 1e9
                    first_walk = t_cmd if first_walk is None else min(first_walk, t_cmd)
                continue
            ts.append(msg.log_time / 1e9)
            qs.append(_parse_lowcmd_q(msg.data))
    if not ts:
        raise ValueError(f"{dataset}: no policy/lowcmd messages")
    t = np.array(ts)
    zero = t[0] if first_walk is None else first_walk
    return t - zero, np.array(qs)[:, perm]


def walk(
    policy: FreePolicy,
    *,
    command: np.ndarray | None = None,
    schedule: tuple[np.ndarray, np.ndarray] | None = None,
    heights: tuple[np.ndarray, np.ndarray] | None = None,
    seconds: float | None = None,
    start: float = 0.0,
    command_delay: float = 0.0,
    actuator_tau: float = 0.0,
    slew: bool = True,
    settle: float = 0.5,
    menagerie: Path | None = None,
    view: bool = False,
    speed: float = 1.0,
    ghost: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None,
    probe: Callable[[mujoco.MjModel, mujoco.MjData], None] | None = None,
) -> Track:
    """Step the policy in MuJoCo.

    ``command`` holds vx/vy/vyaw fixed; ``schedule`` is a ``(t, cmd)`` pair as
    returned by :func:`read_control_log`, held zero-order between samples.
    Exactly one must be given.

    ``ghost`` is a ``(t, pos, quat)`` recorded base_link track (see
    :func:`vive.base_track`) drawn as a translucent box alongside the robot.

    ``command_delay`` holds each command back before the policy sees it, in
    seconds. On hardware the operator's command crosses a network and the
    robot's own filtering before it reaches the policy; with both streams on
    a shared epoch (see ``vive.read_vive_pose``) the real turn answers its
    command in ~0.17 s, where the sim answers on the same tick, 0.04-0.06 s
    -- a gap no leg-joint parameter can close. (An earlier 0.46-0.50 s figure
    was mostly a stream-misalignment artifact, since fixed at the source.)

    ``actuator_tau`` is the motor's first-order time constant in seconds. A
    MuJoCo ``motor`` actuator produces exactly the requested torque on the same
    step; a real BLDC through a gearbox has finite current-loop bandwidth and
    reaches it over a few milliseconds. Zero reproduces the ideal actuator.

    ``heights`` is a ``(t, h)`` gait-height command schedule as returned by
    :func:`read_gait_height`, held zero-order like the velocity schedule and
    subject to the same ``command_delay`` -- but not slewed, because the
    hardware clamps this command without ramping it. ``None`` or empty holds
    :data:`NOMINAL_GAIT_HEIGHT`. Only a policy with more than 45 observation
    channels ever sees the value.

    ``slew`` applies the robot's own per-axis command rate limit
    (:data:`COMMAND_SLEW`) between the schedule and the policy, exactly as the
    hardware does. Not a fitted parameter -- the constants come from the
    executor that produced the recordings. Off only for A/B comparison.

    ``probe`` is called after every physics step, for anything the Track does
    not carry (``envelope.py`` reads the geometry there at the full sim rate).
    """
    if (command is None) == (schedule is None):
        raise ValueError("pass exactly one of command= or schedule=")
    if schedule is not None:
        sched_t, sched_cmd = schedule
        span = float(sched_t[-1])
        if start >= span:
            raise ValueError(
                f"start={start:g}s is at or past the end of the commands "
                f"({span:.1f}s of them). Pick a smaller --start."
            )
        duration = span - start if seconds is None else seconds
    else:
        duration = 8.0 if seconds is None else seconds
    if duration <= 0:
        raise ValueError(f"nothing to simulate: duration is {duration:g}s")

    if ghost is None:
        model, data = go2_model.load(menagerie)
    else:
        model, data = go2_model.load_with_ghost(menagerie)
    sim_dt = model.opt.timestep
    decim = max(1, round(CONTROL_DT / sim_dt))

    kid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")  # type: ignore[attr-defined]
    if kid >= 0:
        mujoco.mj_resetDataKeyframe(model, data, kid)
    data.qpos[7:19] = policy.default_pose
    mujoco.mj_forward(model, data)

    hist: collections.deque[np.ndarray] = collections.deque(maxlen=policy.hist)
    last_action = np.zeros(policy.act_dim)
    target = policy.default_pose.copy()
    applied = np.zeros(12)  # torque actually delivered, lagging the request

    def observe(cmd: np.ndarray, height: float) -> np.ndarray:
        q = data.qpos[7:19]
        dq = data.qvel[6:18]
        raw = np.concatenate(
            [cmd, data.qvel[3:6], projected_gravity(data.qpos[3:7]), q, dq, last_action]
        )
        extra = policy.obs_per_frame - raw.size
        if extra > 0:
            # index 45 is the commanded height in raw metres; anything past it
            # is zero (go2web himloco.rs raw_frame).
            raw = np.concatenate([raw, [height], np.zeros(extra - 1)])
        return policy.normalize(raw)

    def cmd_at(t: float) -> np.ndarray:
        if schedule is None:
            assert command is not None
            return command
        held: np.ndarray = sched_cmd[
            max(0, int(np.searchsorted(sched_t, t + start - command_delay, side="right")) - 1)
        ]
        return held

    def height_at(t: float) -> float:
        if heights is None or len(heights[0]) == 0:
            return NOMINAL_GAIT_HEIGHT
        i = int(np.searchsorted(heights[0], t + start - command_delay, side="right")) - 1
        # before the first slider touch the robot holds the nominal height
        return float(heights[1][i]) if i >= 0 else NOMINAL_GAIT_HEIGHT

    # The live command the policy sees; starts converged on the schedule, the
    # same steady state the real slew is in mid-run.
    vel_cmd = cmd_at(0.0).astype(float).copy()

    for _ in range(policy.hist):
        hist.append(observe(vel_cmd, height_at(0.0)))

    ts: list[float] = []
    pos: list[np.ndarray] = []
    quat: list[np.ndarray] = []
    used: list[np.ndarray] = []
    joint_q: list[np.ndarray] = []
    foot_z: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    feet = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in ("FL", "FR", "RL", "RR")]

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
            if step % decim == 0:
                cmd_target = cmd_at(t)
                if slew:
                    vel_cmd += np.clip(cmd_target - vel_cmd, -COMMAND_SLEW, COMMAND_SLEW)
                else:
                    vel_cmd = cmd_target.astype(float).copy()
                cmd = vel_cmd
                if t >= settle:
                    hist.append(observe(cmd, height_at(t)))
                    # deque is oldest..newest; the nets want newest first.
                    p_obs = np.concatenate(list(hist)[::-1])
                    last_action, target = policy.act(p_obs, cmd)
                if ghost is not None:
                    g_t, g_p, g_q = ghost
                    i = max(0, int(np.searchsorted(g_t, t + start, side="right")) - 1)
                    data.mocap_pos[0] = g_p[i]
                    data.mocap_quat[0] = g_q[i]

                ts.append(t)
                pos.append(data.qpos[0:3].copy())
                quat.append(data.qpos[3:7].copy())
                used.append(cmd.copy())  # vel_cmd mutates in place; snapshot it
                joint_q.append(data.qpos[7:19].copy())
                foot_z.append(data.geom_xpos[feet, 2].copy())
                targets.append(target.copy())

            tau = policy.kp * (target - data.qpos[7:19]) - policy.kd * data.qvel[6:18]
            tau = np.clip(tau, -TORQUE_LIMITS, TORQUE_LIMITS)
            applied = actuator_step(applied, tau, sim_dt, actuator_tau)
            data.ctrl[:] = applied
            mujoco.mj_step(model, data)
            if probe is not None:
                probe(model, data)

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

    return Track(
        t=np.array(ts),
        pos=np.array(pos),
        quat=np.array(quat),
        cmd=np.array(used),
        joint_q=np.array(joint_q),
        foot_z=np.array(foot_z),
        target=np.array(targets),
    )
