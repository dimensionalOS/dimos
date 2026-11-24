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

"""The calibration steps, in order, each run on a ``module.Calibration`` and returning its outcome."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import replace
from datetime import datetime, timezone
import itertools
import math
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.control.calibration._inertial import LinkInertial, apply_inertial_overrides
from dimos.control.calibration.float_analysis import analyze_float
from dimos.control.calibration.gravity_fit import (
    Compensation,
    DitherHold,
    Measurement,
    dither_friction_nm,
    fit_gravity,
    fit_stats,
    measure_dither,
    refusals,
    regressions,
    stuck_pairs,
    totals,
)
from dimos.control.calibration.gravity_poses import ArmModel, Pose, generate_poses
from dimos.control.calibration.schema import TRACKING_CHECK, gravity_links_from_inertial
from dimos.control.calibration.step_analysis import (
    StepEvent,
    StepMetrics,
    analyze_step,
    propose_gains,
    rest_limit_deg,
    rest_noise,
    sweep_friction_nm,
)
from dimos.robot.manipulators.damiao.arm import MIT_KD_MAX, MIT_KP_MAX

if TYPE_CHECKING:
    from dimos.control.calibration.module import Calibration
    from dimos.control.calibration.motion import Motion


class AbortedError(Exception):
    """The operator aborted, or the module is stopping."""


class SkippedError(Exception):
    """The operator skipped what was asked."""


class MotionError(RuntimeError):
    """A motion ended off its target or unsettled; ``detail`` names the joints and how far."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(reason)
        self.detail = detail or reason


class StalledError(MotionError):
    """A joint fell behind its target by more than its motor can push."""


class UnsettledError(MotionError):
    """A move ended unsettled: a joint oscillates in place, or it held ``max_hold_s``."""


TITLES = {
    "check": "measure the noise at rest and the sample period, then go home",
    "gravity_poses": "visit the gravity poses and dither each measured joint",
    "gravity_fit": "fit the link masses, save and apply them on confirm",
    "float_check": "rest, then float each measured joint and measure the fingertip drift",
    "step_sweep": "step each joint at its gains, then at each kp by kd scale",
    "pick_gains": "propose the gains that rest and settle best for confirm_tracking to try",
    "confirm_tracking": "revisit gravity poses with the proposed gains, save them if they track",
    "summary": "show the calibration file the arm runs now",
}
NEEDS = {"gravity_fit": "gravity_poses", "pick_gains": "step_sweep"}


def plan(selected: list[str], can_float: bool) -> list[tuple[str, str | None]]:
    """Every step with why it will not run, or None when it will; check always runs."""
    rows: list[tuple[str, str | None]] = []
    for step in TITLES:
        reason = None
        if step != "check" and step not in selected:
            reason = "not selected"
        elif step in NEEDS and NEEDS[step] not in selected:
            reason = f"needs {NEEDS[step]} in the same run"
        elif step == "float_check" and not can_float:
            reason = "the robot does not compensate gravity, so a joint cannot float"
        rows.append((step, reason))
    return rows


def _deg(q: np.ndarray) -> str:
    return ", ".join(f"{math.degrees(v):.1f}" for v in q)


def _rest_error(run: Calibration, rest: Motion, target: np.ndarray) -> np.ndarray:
    t = np.asarray(rest.t)
    error: np.ndarray = np.asarray(rest.q)[t > t[-1] - run.config.rest_window_s].mean(0) - target
    return error


def _links(run: Calibration) -> dict[str, LinkInertial]:
    return {
        name: LinkInertial(mass_kg=link["mass_kg"], com_m=tuple(link["com_m"]))
        for name, link in run.state["links"].items()
    }


def _applied(run: Calibration) -> ArmModel:
    model = run.arm.model.copy()
    apply_inertial_overrides(model, _links(run))
    return ArmModel(run.spec, model)


def _dithered(run: Calibration, poses: list[Pose]) -> list[str]:
    qs, applied = [np.asarray(pose.joints_rad) for pose in poses], _applied(run)
    torque = np.asarray([applied.gravity(q) for q in qs])
    every = range(run.arm.dof)
    sensitivity = np.asarray([run.arm.scaled_regressor(q, every) for q in qs])
    span = np.maximum(np.ptp(torque, axis=0), np.ptp(sensitivity, axis=0).max(axis=1))
    return [
        name
        for name, varies in zip(run.arm.joint_names, span, strict=True)
        if name in run.arm.measured_joints or varies > run.config.dither_span_nm
    ]


def _dither_deg(run: Calibration, joint: str) -> float:
    spring = run.config.dither_torque_fraction * run.spec.max_torque_nm[joint] / run.gains(joint)[0]
    return min(max(math.degrees(spring), run.config.dither_deg), run.config.max_dither_deg)


def _dither(
    run: Calibration, pose: str, target: np.ndarray, joint: str, dither: float
) -> Measurement:
    i = run.arm.joint_names.index(joint)
    kp, kd = run.gains(joint)
    holds = []
    for sign in (1.0, -1.0):
        dithered = target.copy()
        dithered[i] += math.radians(sign * dither)
        hold = run.stream(run.motion([dithered], "fixed", run.config.dither_hold_s))
        event = StepEvent(
            joint,
            sign * dither,
            kp,
            kd,
            hold.t[0],
            hold.t[-1] - hold.t[0],
            math.degrees(target[i]),
            math.degrees(dithered[i]),
        )
        holds.append(DitherHold(event, np.asarray(hold.t), np.asarray(hold.q)))
    return measure_dither(
        pose,
        run.arm.joint_names,
        holds[0],
        holds[1],
        run.noise[joint],
        stuck_fraction=run.config.stuck_fraction,
    )


def _halt(run: Calibration) -> None:
    """Stream nothing past the connection's deadman, so it holds the arm where it is."""
    deadman = float(run.state["deadman_timeout_s"])
    run.stream(run.motion([run.measured()], "fixed", 2.0 * deadman, publish=False))


def _set_aside(
    run: Calibration, aside: dict[str, str], what: str, how: str, error: MotionError
) -> None:
    run.show(f"{what} {how}: {error.detail}")
    aside[what] = error.detail
    _halt(run)


def _listed(how: str, aside: dict[str, str]) -> str:
    if not aside:
        return ""
    return f", {len(aside)} {how}: " + "; ".join(f"{what} ({why})" for what, why in aside.items())


def _gravity_poses(run: Calibration) -> list[Pose]:
    if run.poses is None:
        pose_set = generate_poses(
            run.arm,
            count=run.config.poses,
            margin_rad=run.excursion_rad,
            grid_points=run.config.grid_points,
        )
        for note in pose_set.notes:
            run.show(note)
        run.poses = pose_set.identification
    return run.poses


def check(run: Calibration) -> str:
    run.wait(run.first_sample)
    run.state = run.connection.get_config()[run.spec.arm_id]
    rest = run.stream(run.motion([run.measured()], "fixed", publish=False))
    t, q, dq = np.asarray(rest.t), np.asarray(rest.q), np.asarray(rest.dq)
    period, deadman = float(np.diff(t).max()), float(run.state["deadman_timeout_s"])
    if period >= deadman:
        raise RuntimeError(
            f"joint_state came {period * 1e3:.1f} ms apart, not within the connection's "
            f"{deadman * 1e3:.0f} ms deadman, so streamed targets would not keep the arm"
        )
    names = run.spec.joint_names
    run.noise = {
        name: rest_noise(t, np.degrees(q[:, i]), np.degrees(dq[:, i]))
        for i, name in enumerate(names)
    }
    outside = [
        f"{name} {math.degrees(value):.1f} deg"
        for name, value in zip(names, q.mean(0), strict=True)
        if not (
            run.spec.joint_ranges[name][0] - math.radians(run.noise[name].band_deg)
            <= value
            <= run.spec.joint_ranges[name][1] + math.radians(run.noise[name].band_deg)
        )
    ]
    if outside:
        raise RuntimeError(f"joints outside their limits: {', '.join(outside)}")
    run.confirm(
        f"The arm moves home [{_deg(run.arm.home)}] deg at {run.config.speed_deg_s:g} deg/s. "
        "Clear the workspace.",
        skippable=False,
    )
    run.move(run.arm.home, "home", ask=False)
    band = max(noise.band_deg for noise in run.noise.values())
    return f"samples at most {period * 1e3:.1f} ms apart, noise within {band:.3f} deg, at home"


def gravity_poses(run: Calibration) -> str:
    poses = _gravity_poses(run)
    run.dithered = _dithered(run, poses)
    dithers = {joint: _dither_deg(run, joint) for joint in run.dithered}
    run.confirm(
        f"The arm visits {len(poses)} poses and dithers "
        f"{', '.join(f'{j} by +-{d:.1f}' for j, d in dithers.items())} deg at each, redoing "
        "once at twice the dither a pair the joint followed less than "
        f"{run.config.stuck_fraction:.0%} of the way, then returns home."
    )
    visited: list[np.ndarray] = []
    skipped: dict[str, str] = {}
    dropped: dict[str, str] = {}
    redone = 0
    for k, pose in enumerate(poses, 1):
        where = f"pose {k}/{len(poses)}"
        run.progress(where)
        target = np.asarray(pose.joints_rad)
        try:
            rest = run.move(target, pose.name)
        except SkippedError:
            continue
        except MotionError as error:
            _set_aside(run, skipped, pose.name, "skipped", error)
            continue
        visited.append(target)
        run.baseline[pose.name] = _rest_error(run, rest, target)
        for j, (joint, dither) in enumerate(dithers.items(), 1):
            run.progress(f"{where} · dither {j}/{len(dithers)} {joint}")
            try:
                measurement = _dither(run, pose.name, target, joint, dither)
                if measurement.stuck:
                    redone += 1
                    wider = min(2.0 * dither, run.config.max_dither_deg)
                    measurement = _dither(run, pose.name, target, joint, wider)
            except MotionError as error:
                _set_aside(run, dropped, f"{joint} at {pose.name}", "dropped", error)
                continue
            run.measurements.append(measurement)
    run.home(visited)
    stuck, unfollowed = stuck_pairs(run.measurements)
    if stuck:
        run.show(
            f"Stuck dither pairs, kept in the fit: {', '.join(stuck)}"
            + "".join(
                f"\n  {joint}: every pair stuck, friction not cancelled" for joint in unfollowed
            )
        )
    measured, least = len({m.pose for m in run.measurements}), run.config.min_poses
    if measured < least:
        raise RuntimeError(
            f"only {measured} of {len(poses)} poses measured; need at least {least} (min_poses)"
            + _listed("skipped", skipped)
            + _listed("dropped", dropped)
        )
    return (
        f"{len(visited)} of {len(poses)} poses{_listed('skipped', skipped)}, "
        f"{len(run.measurements)} dither pairs{_listed('dropped', dropped)}, "
        f"{redone} redone at twice the dither, {len(stuck)} stuck"
    )


def gravity_fit(run: Calibration) -> str:
    compensation = Compensation(run.state["can_float"], _links(run))
    fit = fit_gravity(run.arm, run.measurements, compensation, run.dithered)
    urdf, running, held = (
        fit.rms(v) for v in (fit.urdf_error, fit.running_error, fit.held_out_error)
    )
    total = totals(fit)
    run.show(
        "Gravity torque error, RMS N m: URDF -> running -> fitted (each pose held out)\n"
        + "\n".join(
            f"  {j}: {urdf[j]:.3f} -> {running[j]:.3f} -> {held[j]:.3f}" for j in sorted(held)
        )
        + f"\n  all: {total['urdf']:.3f} -> {total['running']:.3f} -> {total['held_out']:.3f}"
    )
    reasons = refusals(fit)
    if reasons:
        return "not offered: " + "; ".join(reasons)
    worse = regressions(fit)
    listed = f" Worse than the running model: {', '.join(worse)}." if worse else ""
    if total["held_out"] > total["running"]:
        answer = run.ask(
            f"The fit is worse over all the dithered joints than the running model "
            f"({total['running']:.3f} -> {total['held_out']:.3f} N m RMS).{listed} "
            "Keep what the arm runs, or save and apply the fit anyway?",
            ["keep", "save"],
        )
        if answer != "save":
            return (
                f"not saved: worse over all than the running model, {total['running']:.3f} -> "
                f"{total['held_out']:.3f} N m RMS"
            )
    else:
        run.confirm(
            f"Save the fitted masses to the arm's calibration file and apply them now?{listed}"
        )
    fitted = gravity_links_from_inertial(run.spec, fit.links)
    run.configure(
        {
            "links": {name: link.model_dump(mode="json") for name, link in fitted.items()},
            "fit": fit_stats(fit),
            "friction_nm": dither_friction_nm(run.measurements),
        },
        save=True,
    )
    return f"saved {sorted(fitted)} to {run.state['file']['path']}"


def float_check(run: Calibration) -> str:
    config, poses = run.config, _gravity_poses(run)
    limit, applied = config.float_drift_limit_mm, _applied(run)
    torques = [applied.gravity(np.asarray(p.joints_rad)) for p in poses]
    where = {
        joint: poses[int(np.argmax([abs(torque[i]) for torque in torques]))]
        for i, joint in enumerate(run.arm.joint_names)
        if joint in run.arm.measured_joints
    }
    run.confirm(
        f"Each of {', '.join(where)} rests {config.hold_s:g} s, then floats {config.float_window_s:g} s "
        "(kp 0, gravity compensated) at the pose where it carries the most gravity; its "
        f"fingertip must end within {limit:g} mm of where it went limp, and a joint that turns "
        f"{config.float_trip_deg:g} deg is held again."
    )
    rows, joints, passed, visited = [], {}, 0, []
    for k, (joint, pose) in enumerate(where.items(), 1):
        run.progress(f"joint {k}/{len(where)} {joint}")
        i, target = run.arm.joint_names.index(joint), np.asarray(pose.joints_rad)
        try:
            if config.confirm_each:
                run.confirm(f"Float {joint} at {pose.name}?")
            run.move(target, pose.name, ask=False)
        except SkippedError:
            continue
        visited.append(target)
        run.stream(run.motion([target], "fixed", config.hold_s))
        kp, kd = run.gains(joint)
        try:
            hold = run.stream(
                run.motion(
                    [target],
                    "fixed",
                    config.float_window_s,
                    patch={"gains": {joint: {"kp": 0.0, "kd": kd}}},
                    floating=i,
                    drift_limit_rad=math.radians(config.float_trip_deg),
                )
            )
        finally:
            run.configure({"gains": {joint: {"kp": kp, "kd": kd}}})
        drift = analyze_float(
            run.arm,
            hold.t,
            np.asarray(hold.q),
            duration_s=hold.t[-1] - hold.t[0],
            drift_limit_mm=limit,
        )
        ok = hold.ended == "rested" and drift.passed
        passed += ok
        joints[joint] = {
            "pose": pose.name,
            "gravity_nm": float(torques[poses.index(pose)][i]),
            "ended": hold.ended,
            "duration_s": drift.duration_s,
            "fingertip_drift_mm": drift.fingertip_drift_mm,
            "fingertip_peak_mm": drift.fingertip_peak_mm,
            "joint_drift_deg": drift.joint_drift_deg[joint],
            "passed": ok,
        }
        ended = "tripped" if hold.ended == "drifted" else f"{drift.duration_s:.1f} s"
        rows.append(
            f"  {joint} at {pose.name}: {'pass' if ok else 'FAIL'} ({ended}, {joint} turned "
            f"{drift.joint_drift_deg[joint]:+.2f} deg, fingertip {drift.fingertip_drift_mm:.1f} "
            f"mm, at most {drift.fingertip_peak_mm:.1f} mm)"
        )
    run.show("Float check:\n" + "\n".join(rows))
    run.home(visited)
    if joints:
        check = {
            "created": datetime.now(timezone.utc).isoformat(),
            "limit_mm": limit,
            "window_s": config.float_window_s,
            "hold_s": config.hold_s,
            "trip_deg": config.float_trip_deg,
            "joints": joints,
        }
        run.configure({"checks": {"float": check}}, save=True)
    return f"{passed} of {len(joints)} floated joints ended within {limit:g} mm"


def _candidates(run: Calibration, joint: str) -> list[tuple[float, float]]:
    kp0, kd0 = run.gains(joint)

    def capped(kp: float, kd: float) -> tuple[float, float]:
        kp = min(kp, MIT_KP_MAX)
        return kp, min(kd, MIT_KD_MAX, run.config.kd_per_kp * kp)

    grid = [capped(kp0 * p, kd0 * d) for p in run.config.kp_scales for d in run.config.kd_scales]
    return list(dict.fromkeys([capped(kp0, kd0), *grid]))


def _step_metrics(run: Calibration, event: StepEvent, hold: Motion, limit: float) -> StepMetrics:
    i = run.arm.joint_names.index(event.joint)
    q, dq = np.degrees(np.asarray(hold.q)[:, i]), np.degrees(np.asarray(hold.dq)[:, i])
    metrics = analyze_step(
        event,
        hold.t,
        q,
        dq,
        run.noise[event.joint],
        max_overshoot_pct=run.config.max_overshoot_pct,
        rest_limit_deg=limit,
        stuck_fraction=run.config.stuck_fraction,
    )
    if hold.ended != "settled":
        step = f"{event.delta_deg:+g} deg step"
        metrics = replace(metrics, failures=(*metrics.failures, f"{step} {hold.ended}"))
    return metrics


def _rest_limit(run: Calibration, joint: str, kp: float) -> float:
    return rest_limit_deg(
        run.friction.get(joint, 0.0),
        kp,
        smallest_deg=run.config.rest_limit_deg,
        friction_factor=run.config.friction_rest_factor,
    )


def step_sweep(run: Calibration) -> str:
    config, deltas, low_high = run.config, run.config.step_deltas_deg, run.spec.joint_ranges
    low, high = (np.asarray([low_high[n][k] for n in run.arm.joint_names]) for k in (0, 1))
    excursion = math.radians(max(abs(x) for x in itertools.accumulate(deltas)))
    candidates = [("home", run.arm.home)] + [
        (p.name, np.asarray(p.joints_rad)) for p in _gravity_poses(run)
    ]
    name, pose = max(candidates, key=lambda c: float(np.minimum(c[1] - low, high - c[1]).min()))
    if float(np.minimum(pose - low, high - pose).min()) < excursion:
        raise ValueError(f"no pose keeps every joint {math.degrees(excursion):g} deg from a limit")
    tried = {joint: _candidates(run, joint) for joint in run.arm.joint_names}
    run.confirm(
        f"At {name}, each joint steps {', '.join(f'{d:+g}' for d in deltas)} deg at its gains, "
        f"then at kp x {', '.join(f'{s:g}' for s in config.kp_scales)} by kd x "
        f"{', '.join(f'{s:g}' for s in config.kd_scales)} (kd at most {MIT_KD_MAX:g} and kp x "
        f"{config.kd_per_kp:g}; {', '.join(f'{j} {len(c)}' for j, c in tried.items())} "
        "candidates); a step that buzzes ends its candidate. Then the arm returns home."
    )
    run.move(pose, name, ask=False)
    records: list[tuple[bool, StepEvent, Motion]] = []
    joints = run.arm.joint_names
    for i, joint in enumerate(joints):
        run.progress(f"joint {i + 1}/{len(joints)} {joint}")
        try:
            if config.confirm_each:
                run.confirm(f"Sweep {joint}?")
        except SkippedError:
            continue
        kp0, kd0 = run.gains(joint)
        baseline = {"gains": {joint: {"kp": kp0, "kd": kd0}}}
        try:
            for index, pair in enumerate(tried[joint]):
                run.progress(
                    f"joint {i + 1}/{len(joints)} {joint} · candidate {index + 1}/{len(tried[joint])}"
                )
                live = {"gains": {joint: {"kp": pair[0], "kd": pair[1]}}}
                rest = run.stream(run.motion([pose], "converge", patch=live))
                kp, kd = run.gains(joint)
                why = None if rest.ended == "settled" else f"{rest.ended} at rest"
                target = pose.copy()
                for delta in deltas if why is None else []:
                    before, target = target[i], target.copy()
                    target[i] += math.radians(delta)
                    hold = run.stream(run.motion([target], "converge"))
                    t0, t1 = hold.t[0], hold.t[-1]
                    event = StepEvent(
                        joint,
                        delta,
                        kp,
                        kd,
                        t0,
                        t1 - t0,
                        math.degrees(before),
                        math.degrees(target[i]),
                    )
                    records.append((index == 0, event, hold))
                    if hold.ended != "settled":
                        why = f"stopped at its {delta:+g} deg step, which was {hold.ended}"
                    elif _step_metrics(run, event, hold, config.rest_limit_deg).buzzing:
                        why = f"stopped at its {delta:+g} deg step, which buzzed"
                    if why is not None:
                        break
                if why is not None:
                    run.stopped.setdefault(joint, {})[(kp, kd)] = why
                    run.show(f"{joint} at kp {kp:g} kd {kd:g}: {why}; back to its own gains")
                    run.stream(run.motion([pose], "converge", patch=baseline))
        finally:
            run.configure(baseline)
    run.move(run.arm.home, "home")
    first = [
        _step_metrics(run, event, hold, config.rest_limit_deg)
        for base, event, hold in records
        if base
    ]
    run.friction = sweep_friction_nm(first)
    for joint, friction in dither_friction_nm(run.measurements).items():
        run.friction[joint] = max(friction, run.friction.get(joint, 0.0))
    run.step_metrics = [
        _step_metrics(run, event, hold, _rest_limit(run, event.joint, event.kp))
        for _, event, hold in records
    ]
    failed = [m for m in run.step_metrics if not m.passed]
    run.show(
        "Friction, N m: "
        + ", ".join(f"{j} {f:.3f}" for j, f in run.friction.items())
        + "\nSteps (kp kd delta: rest error / limit deg, settle s, overshoot %, failures):\n"
        + "\n".join(
            f"  {m.joint} {m.kp:g} {m.kd:g} {m.delta_deg:+g}: "
            f"{m.steady_state_error_deg:+.2f} / {m.rest_limit_deg:.2f}, "
            + ("stuck" if m.stuck else f"{m.settle_time_s:.2f}, {m.overshoot_pct:.0f}")
            + (", " + "; ".join(m.failures) if m.failures else "")
            for m in run.step_metrics
        )
    )
    stopped = sum(len(pairs) for pairs in run.stopped.values())
    return (
        f"{len(run.step_metrics)} steps at {name}, {len(failed)} failed or stuck, "
        f"{stopped} candidates stopped"
    )


def pick_gains(run: Calibration) -> str:
    proposals = propose_gains(
        run.step_metrics,
        stopped=run.stopped,
        settle_s=run.config.settle_s,
        kd_limit=MIT_KD_MAX,
        max_overshoot_pct=run.config.max_overshoot_pct,
    )
    run.show(
        "Proposed gains:\n"
        + "\n".join(
            f"  {j}: {p.reason}" + "".join(f"\n    {k}: {why}" for k, why in p.rejected.items())
            for j, p in proposals.items()
        )
    )
    chosen = {
        j: {"kp": p.kp, "kd": p.kd}
        for j, p in proposals.items()
        if p.kp is not None and p.kd is not None
    }
    if not chosen:
        return "no gains passed every step; nothing to try"
    listed = _gains_text(chosen)
    if "confirm_tracking" not in run.config.steps:
        return f"proposed {listed}; not saved: confirm_tracking, which tries and saves them, is not selected"
    run.confirm(
        f"Proposed gains: {listed}. confirm_tracking tries them and saves them only if no joint "
        "stalls and each rests about as well as before. Accept them?"
    )
    run.proposed = chosen
    return f"accepted gains for {', '.join(chosen)}; confirm_tracking tries them before saving"


def _gains_text(gains: dict[str, dict[str, float]]) -> str:
    return ", ".join(f"{j} kp {g['kp']:g} kd {g['kd']:g}" for j, g in gains.items())


def _revisit(
    run: Calibration, poses: list[Pose], patch: dict[str, Any] | None
) -> tuple[dict[str, np.ndarray], dict[str, str]]:
    """Each pose's rest error, and why each pose that did not settle was skipped; a stall with
    a ``patch`` still raises."""
    rests, visited = {}, []
    skipped: dict[str, str] = {}
    for k, pose in enumerate(poses, 1):
        run.progress(f"pose {k}/{len(poses)}")
        target = np.asarray(pose.joints_rad)
        try:
            rest = run.move(target, pose.name, patch=patch)
        except SkippedError:
            continue
        except MotionError as error:
            if patch is not None and isinstance(error, StalledError):
                raise
            _set_aside(run, skipped, pose.name, "skipped", error)
            continue
        visited.append(target)
        rests[pose.name] = _rest_error(run, rest, target)
    run.home(visited, patch)
    return rests, skipped


def _rms_deg(errors: list[np.ndarray]) -> np.ndarray:
    rms: np.ndarray = np.degrees(np.sqrt(np.mean(np.square(errors), axis=0)))
    return rms


def confirm_tracking(run: Calibration) -> str:
    poses, proposed, ratio = (
        _gravity_poses(run)[: run.config.confirm_poses],
        run.proposed,
        run.config.tracking_rest_ratio,
    )
    trying = (
        f" and the proposed gains ({_gains_text(proposed)}), re-applied before each move; "
        f"they are saved only if no joint stalls and each rests within {ratio:g} times its first "
        "visit or what its friction allows at its new kp"
        if proposed
        else ""
    )
    run.confirm(
        f"The arm revisits {', '.join(p.name for p in poses)} with the calibration now "
        f"applied{trying}, comparing how far each joint rests from its target with the first visit."
    )
    if proposed and any(pose.name not in run.baseline for pose in poses):
        run.show("No first visit of these poses in this run: a first pass with the running gains")
        run.baseline.update(_revisit(run, poses, None)[0])
    held = {joint: dict(zip(("kp", "kd"), run.gains(joint), strict=True)) for joint in proposed}
    try:
        rests, skipped = _revisit(run, poses, {"gains": proposed} if proposed else None)
    except StalledError as error:
        if not proposed:
            raise
        run.configure({"gains": held})
        run.proposed = {}
        if run.ask(f"{error}; the proposed gains are undone. Go home?", ["home", "stay"]) == "home":
            run.home()
        return f"gains not saved: {error}"
    unsettled = {name: why for name, why in skipped.items() if name in run.baseline}
    if proposed and unsettled:
        run.configure({"gains": held})
        run.proposed = {}
        return "gains not saved: " + "; ".join(
            f"{name} does not settle with them ({why})" for name, why in unsettled.items()
        )
    names = [name for name in rests if name in run.baseline]
    if not names:
        if proposed:
            run.configure({"gains": held})
        return f"visited {len(rests)} poses; none has a first visit in this run to compare"
    before = _rms_deg([run.baseline[name] for name in names])
    after = _rms_deg([rests[name] for name in names])
    run.show(
        "Rest error, RMS deg: first visit -> now\n"
        + "\n".join(
            f"  {name}: {b:.3f} -> {a:.3f}"
            for name, b, a in zip(run.arm.joint_names, before, after, strict=True)
        )
    )
    worse = []
    for joint, gains in proposed.items():
        i = run.arm.joint_names.index(joint)
        limit = max(ratio * before[i], _rest_limit(run, joint, gains["kp"]))
        if after[i] > limit:
            worse.append(f"{joint} rests {after[i]:.2f} deg RMS (limit {limit:.2f})")
    check = {
        "created": datetime.now(timezone.utc).isoformat(),
        "pose_names": names,
        "rest_rms_deg": dict(zip(run.arm.joint_names, map(float, after), strict=True)),
        "baseline": {
            "rest_rms_deg": dict(zip(run.arm.joint_names, map(float, before), strict=True))
        },
    }
    compared = (
        f"{len(names)} poses compared{_listed('skipped', skipped)}; "
        f"worst joint now {after.max():.3f} deg RMS"
    )
    run.proposed = {}
    if worse:
        run.configure({"gains": held})
        return f"{compared}; gains not saved: {'; '.join(worse)}"
    patch: dict[str, Any] = {"checks": {TRACKING_CHECK: check}}
    if proposed:
        patch["gains"] = proposed
    run.configure(patch, save=True)
    saved = f"; saved gains for {', '.join(proposed)}" if proposed else ""
    return f"{compared}{saved} to {run.state['file']['path']}"


def summary(run: Calibration) -> str:
    state = run.connection.get_config()[run.spec.arm_id]
    file = state["file"]
    run.show(
        f"{run.spec.arm_id} runs {file['path'] if file else 'no calibration file'}\n"
        + "\n".join(f"  {j}: kp {g['kp']:g} kd {g['kd']:g}" for j, g in state["gains"].items())
        + f"\n  fitted links: {', '.join(state['links']) or 'none (URDF masses)'}"
    )
    return file["path"] if file else "no calibration file"


STEPS: dict[str, Callable[[Calibration], str]] = {
    "check": check,
    "gravity_poses": gravity_poses,
    "gravity_fit": gravity_fit,
    "float_check": float_check,
    "step_sweep": step_sweep,
    "pick_gains": pick_gains,
    "confirm_tracking": confirm_tracking,
    "summary": summary,
}
