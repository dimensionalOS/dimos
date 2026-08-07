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

"""Score a simulated rollout against the recording that produced its commands.

One call runs the policy under the recorded command schedule, summarizes both
sides, and measures each statistic's own noise by repeating the rollout from
perturbed initial poses. A statistic is only worth fitting when the sim-real
difference clearly exceeds that noise, so :class:`Report` carries the ratio.

    from dimos.navigation.motion.simulation.evaluate import evaluate
    print(evaluate(DATASET, POLICY).table())

``physics`` overrides leg-joint parameters on the compiled model, which is the
hook a parameter search drives:

    evaluate(DATASET, POLICY, physics={"armature": 0.03, "damping": 2.0})
"""

from __future__ import annotations

from collections.abc import Iterator
import contextlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from dimos.navigation.motion.simulation import (
    metrics,
    model as go2_model,
    walk as walk_mod,
)
from dimos.navigation.motion.simulation.policy import FreePolicy
from dimos.navigation.motion.simulation.vive import base_track, mount_rotation, quat_to_mat

# Leg dofs in qvel/dof indexing: 6 free-joint dofs, then the twelve joints.
LEG_DOFS = slice(6, 18)

PERTURB_RAD = 0.05
"""Initial-pose spread used to measure a statistic's noise floor.

Small on purpose -- about 3 degrees. The gait is chaotic enough that this
already decorrelates position within seconds, so it measures the noise a
parameter search has to beat rather than a plausible modelling error.
"""

# The recorded height lives in the Vive room frame, whose floor is unknown,
# so the mean cannot be compared. See vive.py.
NOT_COMPARABLE = ("height_mean",)

# Foot geom names in the menagerie MJCF. Their contacts are already condim=6
# (tangential + torsional + rolling friction, priority 1 -- the "condim=1"
# in the go2 default class only governs the calf capsules), so what is open
# to fitting is the friction values, not the contact dimensionality.
FOOT_GEOMS = ("FL", "FR", "RL", "RR")

# Leg-space statistics, scored command-to-command against policy/lowcmd. The
# judge's base statistics cannot see the legs at all -- the sim matched every
# one of them while visibly high-stepping its front feet. Spans and the
# command-space gait frequency use the joint-angle streams directly rather
# than collapsing them to a single clearance number.
LEG_STATS = (
    "front_lift",
    "rear_lift",
    "thigh_span_front",
    "thigh_span_rear",
    "calf_span_front",
    "calf_span_rear",
    "cmd_gait_hz",
)

_FK_CACHE: tuple[mujoco.MjModel, mujoco.MjData, list[int]] | None = None


def commanded_clearance(targets: np.ndarray, base_z: float = 0.318) -> np.ndarray:
    """Foot clearance (n, 4) implied by joint targets, FK with the base level.

    Kinematics only, so it works identically on the simulator's commanded
    targets and on the recording's ``policy/lowcmd`` -- comparing command to
    command sidesteps both the unrecorded real leg state and the instability
    of open-loop replay.
    """
    global _FK_CACHE
    if _FK_CACHE is None:
        model, data = go2_model.load()
        feet = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n) for n in FOOT_GEOMS]
        _FK_CACHE = (model, data, feet)
    model, data, feet = _FK_CACHE
    out = np.empty((len(targets), 4))
    for i, q in enumerate(targets):
        data.qpos[:] = 0.0
        data.qpos[2] = base_z
        data.qpos[3] = 1.0
        data.qpos[7:19] = q
        # mj_kinematics exists at runtime; the bundled mujoco stubs omit it.
        mujoco.mj_kinematics(model, data)  # type: ignore[attr-defined]
        out[i] = data.geom_xpos[feet, 2]
    return out - 0.022  # foot sphere radius


def leg_stats(t: np.ndarray, targets: np.ndarray) -> dict[str, float]:
    """Distributional statistics of the commanded joint targets.

    Lift is the p95 FK foot clearance (swing apex); spans are the p5..p95
    excursion of the thigh and calf commands, pooled per front/rear pair;
    ``cmd_gait_hz`` is the stepping frequency read off the FL thigh command
    by the same autocorrelation the body-bob estimate uses. Joint layout is
    (hip, thigh, calf) x FL, FR, RL, RR.
    """

    def span(cols: np.ndarray) -> float:
        return float(np.percentile(cols, 95) - np.percentile(cols, 5))

    c = commanded_clearance(targets)
    _grid, thigh_fl = metrics.resample(t, targets[:, 1])
    return {
        "front_lift": float(np.percentile(c[:, :2], 95)),
        "rear_lift": float(np.percentile(c[:, 2:], 95)),
        "thigh_span_front": span(targets[:, [1, 4]]),
        "thigh_span_rear": span(targets[:, [7, 10]]),
        "calf_span_front": span(targets[:, [2, 5]]),
        "calf_span_rear": span(targets[:, [8, 11]]),
        "cmd_gait_hz": metrics.gait_frequency(thigh_fl),
    }


# Best-known configuration: the joint two-recording fit (see FINDINGS) --
# 300 CMA-ES trials scored on himloco01 AND v11 at once, seeded with the
# himloco-only preset it had to beat (joint loss 3.44 -> 1.52), confirmed in
# the viewer on both recordings. The single-recording fit's payload story
# (+4.4 cm com, heavier trunk) was partly policy-style absorption; jointly
# the trunk sits near stock and the legs carry the extra mass instead.
FITTED_PHYSICS = {
    "armature": 0.00712,
    "damping": 0.2850,
    "frictionloss": 0.3650,
    "trunk_mass_scale": 0.9412,
    "trunk_inertia_scale": 0.8601,
    "foot_friction": 0.7860,
    "foot_friction_torsional": 0.006134,
    "trunk_com_x": -0.006845,
    "leg_mass_scale": 1.616,
}
FITTED_COMMAND_DELAY = 0.00898
FITTED_ACTUATOR_TAU = 0.01510

# The searched parameter space; apply_physics/_physics accept nothing else.
PHYSICS_KEYS = frozenset(FITTED_PHYSICS)


def virtual_tracker(
    pos: np.ndarray, quat: np.ndarray, *, mount_yaw: float, tracker_z: float
) -> np.ndarray:
    """Positions with z replaced by a virtual tracker's height.

    Height statistics are compared in *sensor space*: the real side keeps the
    raw tracker height (``base_track(sensor_z=True)``), and the sim side mounts
    a virtual tracker on its base with the same guessed offset. Inverting the
    guess on the real data instead put 11.4 mm of its z std against 5.6 mm from
    the tracker itself; done this way the guess distorts both sides identically
    and mostly cancels.
    """
    off_base = mount_rotation(mount_yaw).T @ np.array([0.0, 0.0, tracker_z])
    out = pos.copy()
    out[:, 2] = pos[:, 2] - np.einsum("nij,j->ni", quat_to_mat(quat), off_base)[:, 2]
    return out


def apply_physics(model: mujoco.MjModel, overrides: dict[str, float]) -> None:
    """Patch fitted physics onto a compiled model, in place.

    The keys are the 9 searched parameters (see :data:`FITTED_PHYSICS`);
    anything else raises. Callers building their own scenes (obstacle worlds)
    use this directly; recording-driven paths go through :func:`_physics`.
    """
    unknown = set(overrides) - PHYSICS_KEYS
    if unknown:
        raise ValueError(f"unknown physics override(s): {sorted(unknown)}")
    if "armature" in overrides:
        model.dof_armature[LEG_DOFS] = overrides["armature"]
    if "damping" in overrides:
        model.dof_damping[LEG_DOFS] = overrides["damping"]
    if "frictionloss" in overrides:
        model.dof_frictionloss[LEG_DOFS] = overrides["frictionloss"]
    # A heavier or more rotationally sluggish trunk is the competing
    # explanation for the turn lag: inertia delays *and* smooths the
    # response, where transport delay only shifts it.
    trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    if "trunk_mass_scale" in overrides:
        model.body_mass[trunk] *= overrides["trunk_mass_scale"]
    if "trunk_inertia_scale" in overrides:
        model.body_inertia[trunk] *= overrides["trunk_inertia_scale"]
    # Payload placement, not just payload mass: the lidar and tracker sit
    # forward and top of the trunk, which body_mass scaling cannot express.
    if "trunk_com_x" in overrides:
        model.body_ipos[trunk][0] += overrides["trunk_com_x"]
    # Real legs carry covers and cabling the MJCF omits; heavier swing
    # inertia damps how high a foot flies for the same action.
    if "leg_mass_scale" in overrides:
        for prefix in FOOT_GEOMS:
            for part in ("thigh", "calf"):
                bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{prefix}_{part}")
                model.body_mass[bid] *= overrides["leg_mass_scale"]
                model.body_inertia[bid] *= overrides["leg_mass_scale"]
    # geom_friction columns are (tangential, torsional, rolling); the foot
    # has priority 1, so its values dictate the foot-floor contact pair.
    # Torsional is what resists pivoting the stance feet in a turn.
    for name in FOOT_GEOMS:
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if "foot_friction" in overrides:
            model.geom_friction[gid, 0] = overrides["foot_friction"]
        if "foot_friction_torsional" in overrides:
            model.geom_friction[gid, 1] = overrides["foot_friction_torsional"]


@contextlib.contextmanager
def _physics(overrides: dict[str, float] | None) -> Iterator[None]:
    """Temporarily patch leg-joint parameters onto every model that gets built."""
    if not overrides:
        yield
        return
    unknown = set(overrides) - PHYSICS_KEYS
    if unknown:
        raise ValueError(f"unknown physics override(s): {sorted(unknown)}")

    original = go2_model.load
    original_ghost = go2_model.load_with_ghost

    def patched(menagerie: Path | None = None) -> tuple[mujoco.MjModel, mujoco.MjData]:
        model, data = original(menagerie)
        apply_physics(model, overrides)
        return model, data

    # The ghost loader builds its own model, so it must be patched too --
    # --view --ghost once silently ran stock physics under --fitted.
    def patched_ghost(
        menagerie: Path | None = None,
        rgba: tuple[float, float, float, float] = (0.2, 1.0, 0.2, 0.35),
    ) -> tuple[mujoco.MjModel, mujoco.MjData]:
        model, data = original_ghost(menagerie, rgba)
        apply_physics(model, overrides)
        return model, data

    go2_model.load = patched  # type: ignore[assignment]
    go2_model.load_with_ghost = patched_ghost  # type: ignore[assignment]
    try:
        yield
    finally:
        go2_model.load = original  # type: ignore[assignment]
        go2_model.load_with_ghost = original_ghost  # type: ignore[assignment]


def _noise_floor(
    policy: FreePolicy,
    sched: Any,
    heights: Any,
    start: float,
    seconds: float,
    seeds: int,
    mount_yaw: float = 94.0,
    tracker_z: float = 0.207,
) -> dict[str, float]:
    runs = []
    leg_runs: list[dict[str, float]] = []
    base = policy.default_pose.copy()
    for seed in range(seeds):
        rng = np.random.default_rng(seed)
        object.__setattr__(policy, "default_pose", base + rng.normal(0, PERTURB_RAD, 12))
        try:
            t2 = walk_mod.walk(
                policy, schedule=sched, heights=heights, seconds=seconds, start=start
            )
        finally:
            object.__setattr__(policy, "default_pose", base)
        p2 = virtual_tracker(t2.pos, t2.quat, mount_yaw=mount_yaw, tracker_z=tracker_z)
        runs.append(_summarize_run(t2.t, p2, t2.quat, sched, start))
        leg_runs.append(leg_stats(t2.t, t2.target))
    spread = metrics.chaos_spread(runs)
    for k in LEG_STATS:
        vals = [lr[k] for lr in leg_runs]
        spread[k] = float(max(vals) - min(vals))
    return spread


def measure_noise(
    dataset: str | Path,
    policy_bin: str | Path,
    *,
    start: float = 6.0,
    seconds: float | None = None,
    seeds: int = 4,
) -> dict[str, float]:
    """Each statistic's noise floor, to be measured once and reused by a search.

    ``seconds=None`` covers the whole recording, matching :func:`evaluate`.
    """
    policy = FreePolicy.load(policy_bin)
    sched = walk_mod.read_control_log(dataset)
    heights = walk_mod.read_gait_height(dataset)
    if seconds is None:
        seconds = float(sched[0][-1]) - start
    return _noise_floor(policy, sched, heights, start, seconds, seeds)


@dataclass
class Report:
    sim: metrics.Summary
    real: metrics.Summary
    noise: dict[str, float]
    seconds: float
    start: float
    physics: dict[str, float] = field(default_factory=dict)
    sim_legs: dict[str, float] = field(default_factory=dict)
    real_legs: dict[str, float] = field(default_factory=dict)

    def snr(self) -> dict[str, float]:
        """Sim-real difference over the statistic's own noise, per statistic."""
        s = {**self.sim.as_dict(), **self.sim_legs}
        r = {**self.real.as_dict(), **self.real_legs}
        out = {}
        for k in s:
            if k in NOT_COMPARABLE or k not in r:
                continue
            n = self.noise[k]
            out[k] = abs(s[k] - r[k]) / n if n > 1e-9 else float("inf")
        return out

    def loss(self) -> float:
        """Noise-weighted distance, for a parameter search to minimize.

        Each statistic is scaled by its own noise floor, so a term cannot be
        won by driving a quantity the simulator cannot resolve anyway.
        """
        return float(np.sqrt(np.mean([v**2 for v in self.snr().values()])))

    def table(self) -> str:
        s = {**self.sim.as_dict(), **self.sim_legs}
        r = {**self.real.as_dict(), **self.real_legs}
        snr = self.snr()
        head = f"{self.seconds:.0f}s from t={self.start:.0f}s"
        if self.physics:
            head += "  " + " ".join(f"{k}={v:g}" for k, v in sorted(self.physics.items()))
        lines = [head, f"{'statistic':>14} {'sim':>9} {'real':>9} {'noise':>9} {'SNR':>7}"]
        for k in sorted(snr, key=lambda k: -snr[k]):
            lines.append(f"{k:>14} {s[k]:9.3f} {r[k]:9.3f} {self.noise[k]:9.3f} {snr[k]:7.1f}")
        for k in NOT_COMPARABLE:
            lines.append(f"{k:>14} {s[k]:9.3f} {r[k]:9.3f} {'--':>9} {'n/a':>7}")
        lines.append(f"{'loss':>14} {self.loss():9.2f}")
        return "\n".join(lines)


def _summarize_run(
    t: np.ndarray, pos: np.ndarray, quat: np.ndarray, sched: Any, offset: float
) -> metrics.Summary:
    ct, cc = sched
    idx = np.clip(np.searchsorted(ct, t + offset, side="right") - 1, 0, len(ct) - 1)
    return metrics.summarize(t, pos, quat, cc[idx])


def evaluate(
    dataset: str | Path,
    policy_bin: str | Path,
    *,
    start: float = 6.0,
    seconds: float | None = None,
    mount_yaw: float = 94.0,
    tracker_z: float = 0.207,
    anchor_height: float = 0.28,
    seeds: int = 4,
    physics: dict[str, float] | None = None,
    command_delay: float = 0.0,
    actuator_tau: float = 0.0,
    noise: dict[str, float] | None = None,
) -> Report:
    """Run the policy under the recording's commands and score it against them.

    ``noise`` skips the perturbed rollouts. A search should call
    :func:`measure_noise` once and reuse the result: it costs ``seeds``
    rollouts, it is a property of the system rather than of any one
    parameter set, and holding it fixed keeps the loss comparable between
    trials. That is a 5x saving per trial at the default four seeds.
    """
    policy = FreePolicy.load(policy_bin)
    sched = walk_mod.read_control_log(dataset)
    heights = walk_mod.read_gait_height(dataset)
    if seconds is None:
        # Score the entire recording. A judge windowed to the first 20 s once
        # certified a config that visibly degraded after t=26 -- unscored time
        # is unconstrained time.
        seconds = float(sched[0][-1]) - start

    with _physics(physics):
        track = walk_mod.walk(
            policy,
            schedule=sched,
            heights=heights,
            seconds=seconds,
            start=start,
            command_delay=command_delay,
            actuator_tau=actuator_tau,
        )
        sim_p = virtual_tracker(track.pos, track.quat, mount_yaw=mount_yaw, tracker_z=tracker_z)
        sim = _summarize_run(track.t, sim_p, track.quat, sched, start)
        if noise is None:
            noise = _noise_floor(
                policy, sched, heights, start, seconds, seeds, mount_yaw, tracker_z
            )

    sim_legs = leg_stats(track.t, track.target)
    try:
        lt, lq = walk_mod.read_policy_lowcmd(dataset)
        lsel = (lt >= start) & (lt < start + seconds)
        real_legs = leg_stats(lt[lsel], lq[lsel])
    except ValueError:  # recording without policy/lowcmd: legs stay unscored
        real_legs = {}

    gt, gp, gq = base_track(
        dataset,
        tracker_offset=np.array([0.0, 0.0, tracker_z]),
        mount=mount_rotation(mount_yaw),
        anchor_at=start,
        anchor_pos=np.array([0.0, 0.0, anchor_height]),
        sensor_z=True,
    )
    sel = (gt >= start) & (gt < start + seconds)
    real = _summarize_run(gt[sel] - start, gp[sel], gq[sel], sched, start)

    return Report(
        sim=sim,
        real=real,
        noise=noise,
        seconds=seconds,
        start=start,
        sim_legs=sim_legs,
        real_legs=real_legs,
        physics={
            **(physics or {}),
            **({"command_delay": command_delay} if command_delay else {}),
            **({"actuator_tau": actuator_tau} if actuator_tau else {}),
        },
    )
