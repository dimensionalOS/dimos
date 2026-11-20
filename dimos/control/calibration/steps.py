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
import itertools
import math
from typing import TYPE_CHECKING

import numpy as np

from dimos.control.calibration._inertial import LinkInertial
from dimos.control.calibration.float_analysis import analyze_float
from dimos.control.calibration.gravity_fit import (
    Compensation,
    DitherHold,
    fit_gravity,
    fit_stats,
    measure_dither,
    refusals,
)
from dimos.control.calibration.gravity_poses import Pose, generate_poses
from dimos.control.calibration.schema import gravity_links_from_inertial
from dimos.control.calibration.step_analysis import (
    StepEvent,
    StepMetrics,
    analyze_step,
    propose_gains,
    rest_noise,
)
from dimos.robot.manipulators.damiao.arm import MIT_KD_MAX, MIT_KP_MAX

if TYPE_CHECKING:
    from dimos.control.calibration.module import Calibration
    from dimos.control.calibration.motion import Motion


class AbortedError(Exception):
    """The operator aborted, or the module is stopping."""


class SkippedError(Exception):
    """The operator skipped what was asked."""


TITLES = {
    "check": "measure the noise at rest and the sample period, then go home",
    "gravity_poses": "visit the gravity poses and dither each measured joint",
    "gravity_fit": "fit the link masses, save and apply them on confirm",
    "float_check": "float each measured joint and measure the fingertip drift",
    "step_sweep": "step each joint at each kp scale",
    "pick_gains": "propose the stiffest gains that pass, save and apply them on confirm",
    "confirm_tracking": "revisit gravity poses and compare the rest error with the first visit",
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
    poses, measured, dither = _gravity_poses(run), run.arm.measured_joints, run.config.dither_deg
    run.confirm(
        f"The arm visits {len(poses)} poses and dithers {', '.join(measured)} by "
        f"+-{dither:g} deg at each, then returns home."
    )
    visited: list[np.ndarray] = []
    for pose in poses:
        target = np.asarray(pose.joints_rad)
        try:
            rest = run.move(target, pose.name)
        except SkippedError:
            continue
        visited.append(target)
        run.baseline[pose.name] = _rest_error(run, rest, target)
        for joint in measured:
            i = run.arm.joint_names.index(joint)
            kp, kd = run.gains(joint)
            holds = []
            for sign in (1.0, -1.0):
                dithered = target.copy()
                dithered[i] += math.radians(sign * dither)
                hold = run.stream(run.motion([dithered], "converge"))
                if hold.ended != "settled":
                    raise RuntimeError(f"{joint} is {hold.ended} in its dither at {pose.name}")
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
            run.measurements.append(
                measure_dither(pose.name, run.arm.joint_names, holds[0], holds[1], run.noise[joint])
            )
    run.home(visited)
    return f"{len(visited)} of {len(poses)} poses, {len(run.measurements)} dither pairs"


def gravity_fit(run: Calibration) -> str:
    links = {
        name: LinkInertial(mass_kg=link["mass_kg"], com_m=tuple(link["com_m"]))
        for name, link in run.state["links"].items()
    }
    fit = fit_gravity(run.arm, run.measurements, Compensation(run.state["can_float"], links))
    urdf, running, held = (
        fit.rms(v) for v in (fit.urdf_error, fit.running_error, fit.held_out_error)
    )
    run.show(
        "Gravity torque error, RMS N m: URDF -> running -> fitted (each pose held out)\n"
        + "\n".join(
            f"  {j}: {urdf[j]:.3f} -> {running[j]:.3f} -> {held[j]:.3f}" for j in sorted(held)
        )
    )
    reasons = refusals(fit)
    if reasons:
        return "not offered: " + "; ".join(reasons)
    run.confirm("Save the fitted masses to the arm's calibration file and apply them now?")
    fitted = gravity_links_from_inertial(run.spec, fit.links)
    run.configure(
        {
            "links": {name: link.model_dump(mode="json") for name, link in fitted.items()},
            "fit": fit_stats(fit),
        },
        save=True,
    )
    return f"saved {sorted(fitted)} to {run.state['file']['path']}"


def float_check(run: Calibration) -> str:
    poses, limit = _gravity_poses(run), run.config.float_drift_limit_mm
    torques = [np.abs(run.arm.gravity(np.asarray(p.joints_rad))) for p in poses]
    where = {
        joint: poses[int(np.argmax([torque[i] for torque in torques]))]
        for i, joint in enumerate(run.arm.joint_names)
        if joint in run.arm.measured_joints
    }
    run.confirm(
        f"Each of {', '.join(where)} floats in turn (kp 0, gravity compensated) at the pose "
        f"where it carries the most gravity; its fingertip must stay within {limit:g} mm."
    )
    rows, passed, visited = [], 0, []
    for joint, pose in where.items():
        i, target = run.arm.joint_names.index(joint), np.asarray(pose.joints_rad)
        try:
            if run.config.confirm_each:
                run.confirm(f"Float {joint} at {pose.name}?")
            run.move(target, pose.name, ask=False)
        except SkippedError:
            continue
        visited.append(target)
        kp, kd = run.gains(joint)
        motion = run.motion([target], "converge", floating=i, drift_limit_m=limit / 1e3)
        try:
            hold = run.stream(motion, patch={"gains": {joint: {"kp": 0.0, "kd": kd}}})
        finally:
            run.configure({"gains": {joint: {"kp": kp, "kd": kd}}})
        drift = analyze_float(
            run.arm,
            hold.t,
            np.asarray(hold.q),
            duration_s=hold.t[-1] - hold.t[0],
            drift_limit_mm=limit,
        )
        ok = hold.ended == "settled" and drift.passed
        passed += ok
        rows.append(
            f"  {joint} at {pose.name}: {'pass' if ok else 'FAIL'} ({hold.ended}, fingertip "
            f"{drift.fingertip_drift_mm:.1f} mm, at most {drift.fingertip_peak_mm:.1f} mm)"
        )
    run.show("Float check:\n" + "\n".join(rows))
    run.home(visited)
    return f"{passed} of {len(rows)} floated joints stayed within {limit:g} mm"


def step_sweep(run: Calibration) -> str:
    deltas, low_high = run.config.step_deltas_deg, run.spec.joint_ranges
    low, high = (np.asarray([low_high[n][k] for n in run.arm.joint_names]) for k in (0, 1))
    excursion = math.radians(max(abs(x) for x in itertools.accumulate(deltas)))
    candidates = [("home", run.arm.home)] + [
        (p.name, np.asarray(p.joints_rad)) for p in _gravity_poses(run)
    ]
    name, pose = max(candidates, key=lambda c: float(np.minimum(c[1] - low, high - c[1]).min()))
    if float(np.minimum(pose - low, high - pose).min()) < excursion:
        raise ValueError(f"no pose keeps every joint {math.degrees(excursion):g} deg from a limit")
    scales = sorted(set(run.config.kp_scales), key=lambda s: (s != 1.0, s))
    run.confirm(
        f"At {name}, each joint steps {', '.join(f'{d:+g}' for d in deltas)} deg at kp x "
        f"{', '.join(f'{s:g}' for s in scales)} (kd x its square root), then the arm returns home."
    )
    run.move(pose, name, ask=False)
    records: list[tuple[float, StepEvent, Motion]] = []
    for joint in run.arm.joint_names:
        i = run.arm.joint_names.index(joint)
        try:
            if run.config.confirm_each:
                run.confirm(f"Sweep {joint}?")
        except SkippedError:
            continue
        kp0, kd0 = run.gains(joint)
        converging = True
        try:
            for scale in scales:
                if not converging:
                    break
                live = {
                    joint: {
                        "kp": min(kp0 * scale, MIT_KP_MAX),
                        "kd": min(kd0 * math.sqrt(scale), MIT_KD_MAX),
                    }
                }
                rest = run.stream(run.motion([pose], "converge"), patch={"gains": live})
                kp, kd = run.gains(joint)
                if rest.ended != "settled":
                    run.show(
                        f"{joint} at kp {kp:g} kd {kd:g} is {rest.ended} at rest; its sweep stops"
                    )
                    break
                target = pose.copy()
                for delta in deltas:
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
                    records.append((scale, event, hold))
                    converging = hold.ended == "settled"
                    if not converging:
                        break
        finally:
            run.configure({"gains": {joint: {"kp": kp0, "kd": kd0}}})
    run.move(run.arm.home, "home")
    baseline: dict[tuple[str, float], StepMetrics] = {}
    for scale, event, hold in records:
        i = run.arm.joint_names.index(event.joint)
        q, dq = np.degrees(np.asarray(hold.q)[:, i]), np.degrees(np.asarray(hold.dq)[:, i])
        metrics = analyze_step(
            event,
            hold.t,
            q,
            dq,
            run.noise[event.joint],
            max_overshoot_pct=run.config.max_overshoot_pct,
            baseline=None if scale == 1.0 else baseline[(event.joint, event.delta_deg)],
        )
        if hold.ended != "settled":
            step = f"{event.delta_deg:+g} deg step"
            metrics = replace(metrics, failures=(*metrics.failures, f"{step} {hold.ended}"))
        if scale == 1.0:
            baseline[(event.joint, event.delta_deg)] = metrics
        run.step_metrics.append(metrics)
    failed = [m for m in run.step_metrics if not m.passed]
    run.show(
        "Steps (kp kd delta: settle s, overshoot %, failures):\n"
        + "\n".join(
            f"  {m.joint} {m.kp:g} {m.kd:g} {m.delta_deg:+g}: {m.settle_time_s:.2f}, "
            f"{m.overshoot_pct:.0f}{', ' + '; '.join(m.failures) if m.failures else ''}"
            for m in run.step_metrics
        )
    )
    return f"{len(run.step_metrics)} steps at {name}, {len(failed)} failed"


def pick_gains(run: Calibration) -> str:
    proposals = propose_gains(run.step_metrics)
    run.show("Proposed gains:\n" + "\n".join(f"  {j}: {p.reason}" for j, p in proposals.items()))
    chosen = {j: {"kp": p.kp, "kd": p.kd} for j, p in proposals.items() if p.kp is not None}
    if not chosen:
        return "no gains passed every step; nothing saved"
    run.confirm(f"Save the gains of {', '.join(chosen)} and apply them now?")
    run.configure({"gains": chosen}, save=True)
    return f"saved gains for {', '.join(chosen)} to {run.state['file']['path']}"


def confirm_tracking(run: Calibration) -> str:
    poses = _gravity_poses(run)[: run.config.confirm_poses]
    run.confirm(
        f"The arm revisits {', '.join(p.name for p in poses)} with the calibration now applied, "
        "comparing how far each joint rests from its target with the first visit."
    )
    before, after, visited = [], [], []
    for pose in poses:
        target = np.asarray(pose.joints_rad)
        try:
            rest = run.move(target, pose.name)
        except SkippedError:
            continue
        visited.append(target)
        if pose.name in run.baseline:
            before.append(run.baseline[pose.name])
            after.append(_rest_error(run, rest, target))
    run.home(visited)
    if not after:
        return f"visited {len(visited)} poses; none has a first visit in this run to compare"
    rms = [np.degrees(np.sqrt(np.mean(np.square(e), axis=0))) for e in (before, after)]
    run.show(
        "Rest error, RMS deg: first visit -> now\n"
        + "\n".join(
            f"  {name}: {b:.3f} -> {a:.3f}"
            for name, b, a in zip(run.arm.joint_names, *rms, strict=True)
        )
    )
    return f"{len(after)} poses compared; worst joint now {rms[1].max():.3f} deg RMS"


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
