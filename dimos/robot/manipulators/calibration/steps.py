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

"""The steps of an arm calibration, in the order :func:`calibration_steps` gives them.

1. ``check``: the connection answers, has no fault and drives this arm, its
   readings are finite and inside the joint limits, the arm has an explicit
   workspace guard; then, after a confirmation, the arm goes home.
2. ``gravity_poses``: the bench-proven sweep. At each identification pose
   (``gravity_poses.generate_poses``) the pose is marked, the arm ramps there
   (through home when the straight ramp would dip), holds, and dithers each
   measured joint +d then -d, holding after each; an end marker closes it.
3. ``gravity_fit``: fits the sweep's recording (``gravity_fit``), shows the
   held-out error before and after, and saves and applies the fit when it
   passes the save gate and the operator agrees.
4. ``float_check``: floats one measured joint at a time at a float pose (the first
   gravity pose, or ``float_pose_deg``) and measures
   how far the fingertip drifts (``float_analysis``); robots that do not
   compensate gravity cannot float.
5. ``step_sweep``: steps every joint +1, -1, +3, -3 deg at each kp scale of
   its robot kp, with its robot kd, holding after each step.
6. ``pick_gains``: analyses those steps (``step_analysis.propose_gains``) and
   saves and applies the gains the operator accepts.
7. ``confirm_tracking``: visits random guard-safe poses with what is applied
   and compares how well the arm tracks with the run's own gravity sweep.
8. ``summary``: the calibration file, its history and what the arm runs.

A step that needs a recording reads it from the run's recording
(``recording.RecordingSource``), cut to the time it ran. Calibrations are saved
with ``schema.save_calibration`` (the replaced file goes to ``history/``) and
applied with ``tuning_apply_calibration`` at home; a gravity change too large
to apply there is forced only after a second confirmation.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import replace
from datetime import datetime, timezone
import itertools
import math
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray
import pinocchio as pin

from dimos.robot.manipulators.calibration._inertial import apply_inertial_overrides
from dimos.robot.manipulators.calibration.arm_session import (
    ArmSession,
    JointDriftedError,
    StepFailedError,
    TrackingLog,
    plain_error,
)
from dimos.robot.manipulators.calibration.float_analysis import (
    DRIFT_LIMIT_MM,
    FingertipModel,
    analyze_window,
)
from dimos.robot.manipulators.calibration.gravity_fit import (
    REPORT_DIR_NAME as GRAVITY_REPORT_DIR,
    FitResult,
    Measurement,
    calibration_from_fit,
    extract_measurements,
    fit_gravity,
    format_report,
    merge_with_stored,
    plot_resting_torque,
    save_refusals,
    write_measurements_csv,
)
from dimos.robot.manipulators.calibration.gravity_poses import ArmModel, generate_poses
from dimos.robot.manipulators.calibration.motion import (
    MIN_JERK_PEAK_RATIO,
    plan_path,
    speed_rad_s,
)
from dimos.robot.manipulators.calibration.procedure import (
    Capabilities,
    SkipRequestedError,
    Step,
    StepContext,
    StepResult,
    StepUnavailableError,
)
from dimos.robot.manipulators.calibration.schema import (
    HISTORY_DIR_NAME,
    ArmCalibration,
    ArmCalibrationError,
    JointGains,
    calibration_path,
    load_calibration,
    load_for_spec,
    save_calibration,
    to_inertial_overrides,
)
from dimos.robot.manipulators.calibration.spec import (
    ArmCalibrationSpec,
    NoWorkspaceGuardError,
    urdf_limits,
)
from dimos.robot.manipulators.calibration.step_analysis import propose_gains
from dimos.robot.manipulators.calibration.step_report import (
    REPORT_DIR_NAME as STEP_REPORT_DIR,
    extract_steps,
    write_tables,
)

END_LABEL = "end"
# A reading this far past a joint limit fails the check.
LIMIT_TOLERANCE_DEG = 2.0
# A calibration is applied with every joint this close to home.
HOME_TOLERANCE_DEG = 3.0
TRACKING_POSE_TRIES = 4000

Vector = NDArray[np.float64]


def _deg(names: Sequence[str], q: Vector) -> dict[str, float]:
    return {name: round(math.degrees(float(v)), 3) for name, v in zip(names, q, strict=True)}


def _angles(q: Sequence[float] | Vector) -> str:
    return " ".join(f"{math.degrees(float(v)):.1f}" for v in q)


def _ramp_s(a: Vector, b: Vector, speed_deg_s: float) -> float:
    travel = float(np.max(np.abs(np.asarray(b) - np.asarray(a)), initial=0.0))
    return MIN_JERK_PEAK_RATIO * travel / speed_rad_s(speed_deg_s)


def _minutes(seconds: float) -> str:
    return f"{seconds / 60.0:.1f} min"


def _now() -> datetime:
    return datetime.now(timezone.utc)


def _recording_unavailable(caps: Capabilities) -> bool | str:
    return True if caps.recording_unavailable is None else caps.recording_unavailable


def _measured(ctx: StepContext) -> Vector:
    """The arm's reading outside a tuning session."""
    try:
        samples = ctx.connection.tuning_read()
    except Exception as error:
        raise StepFailedError(f"the connection could not read the arm: {plain_error(error)}")
    return np.asarray([samples[name]["position"] for name in ctx.spec.joint_names], dtype=float)


def _ensure_home(ctx: StepContext, why: str) -> None:
    """Bring the arm home after a confirmation, unless it is there already."""
    away = np.degrees(np.abs(_measured(ctx) - ctx.arm.home))
    if float(away.max()) <= HOME_TOLERANCE_DEG:
        return
    ctx.confirm(
        f"Arm will move to home at {ctx.settings.speed_deg_s:g} deg/s to {why}. Clear the "
        "workspace. Enter to continue"
    )
    with ctx.session() as session:
        session.move_to(ctx.arm.home)


def _apply(ctx: StepContext, path: Path) -> dict[str, Any]:
    """Apply ``path`` at home; a gravity change too large is forced only on a second yes."""
    _ensure_home(ctx, "apply the calibration")
    forced = False
    try:
        applied = ctx.connection.tuning_apply_calibration(str(path))
    except ValueError as refused:
        # The connection refuses a gravity change above its limit at this pose.
        try:
            ctx.confirm(
                f"The connection refused: {plain_error(refused)}. Apply anyway? Joints may move "
                "a little when their gravity torque changes; support the arm. Enter to force it"
            )
        except SkipRequestedError:
            return {"applied": False, "forced": False, "refused": plain_error(refused)}
        applied = ctx.connection.tuning_apply_calibration(str(path), True)
        forced = True
    except Exception as error:
        raise StepFailedError(f"the connection did not apply {path}: {plain_error(error)}")
    steps = {joint: round(float(v), 3) for joint, v in applied["gravity_step_nm"].items()}
    return {"applied": True, "forced": forced, "gravity_step_nm": steps}


def _stored(ctx: StepContext) -> tuple[Path, ArmCalibration | None]:
    """The arm's calibration file and what it holds, ``None`` when there is none."""
    path = calibration_path(ctx.spec.arm_id, ctx.settings.calibration_root)
    if not path.exists():
        return path, None
    try:
        return path, load_for_spec(ctx.spec, path)
    except ArmCalibrationError as error:
        raise StepFailedError(f"the stored calibration cannot be updated: {error}")


def _record_check(ctx: StepContext, name: str, value: dict[str, Any]) -> str | None:
    """Store ``value`` as ``checks[name]`` of the arm's calibration file, if it has one."""
    path, stored = _stored(ctx)
    if stored is None:
        return None
    checks = {**(stored.checks or {}), name: value}
    save_calibration(path, stored.model_copy(update={"created": _now(), "checks": checks}))
    return str(path)


def _compensation_offset(
    ctx: StepContext, applied: str | None
) -> Callable[[Vector], NDArray[np.float64]]:
    """Gravity torque the connection compensated during the sweep minus the URDF's, by joint.

    The fit corrects the URDF, but the arm was held with whatever the
    connection compensated: nothing on a robot without gravity compensation,
    the applied calibration's masses on one that ran a file.
    """
    arm = ctx.arm
    if not ctx.caps.gravity_compensation:
        return lambda q: -arm.gravity(q)
    if applied is None:
        return lambda q: np.zeros(arm.dof)
    try:
        links = to_inertial_overrides(load_calibration(Path(applied)))
        model = arm.model.copy()
        apply_inertial_overrides(model, links)
    except (ArmCalibrationError, ValueError) as error:
        raise StepFailedError(
            f"cannot read back the calibration the sweep ran ({applied}): {error}"
        )
    data = model.createData()

    def offset(q: Vector) -> NDArray[np.float64]:
        compensated = np.asarray(pin.computeGeneralizedGravity(model, data, q), dtype=np.float64)
        difference: NDArray[np.float64] = compensated - np.asarray(arm.gravity(q))
        return difference

    return offset


def _corrected(
    arm: ArmModel, measurements: list[Measurement], offset: Callable[[Vector], Vector]
) -> list[Measurement]:
    rows = {name: index for index, name in enumerate(arm.joint_names)}
    return [
        replace(m, tau_pd_nm=m.tau_pd_nm + float(offset(arm.q(m.q_rad))[rows[m.joint]]))
        for m in measurements
    ]


def _rms_table(before: dict[str, float], after: dict[str, float]) -> dict[str, dict[str, float]]:
    return {
        joint: {"before": round(before[joint], 4), "after": round(after[joint], 4)}
        for joint in before
    }


def random_poses(arm: ArmModel, count: int, seed: int) -> list[Vector]:
    """Up to ``count`` random poses that pass ``pose_ok``, each reachable from the one before.

    Measured joints are drawn inside their ``joint_ranges``; the others stay at
    home. The first is reachable from home.
    """
    rng = np.random.default_rng(seed)
    columns = list(arm.measured_rows)
    low = np.asarray([arm.spec.joint_ranges[arm.joint_names[i]][0] for i in columns])
    high = np.asarray([arm.spec.joint_ranges[arm.joint_names[i]][1] for i in columns])
    poses: list[Vector] = []
    previous = np.asarray(arm.home, dtype=np.float64)
    for _ in range(TRACKING_POSE_TRIES):
        if len(poses) >= count:
            break
        q = np.asarray(arm.home, dtype=np.float64).copy()
        q[columns] = rng.uniform(low, high)
        if arm.pose_ok(q) and plan_path(arm, previous, q) is not None:
            poses.append(q)
            previous = q
    return poses


class CheckStep(Step):
    id = "check"
    title = "Check the arm"
    instructions = (
        "Checks that the connection is running without a fault and drives this arm, that every "
        "joint reads a finite position inside its limits, and that the arm has a workspace guard "
        "(the table height and the line its fingertip stays in front of). It reports the "
        "calibration file the arm runs now. Then the arm moves home: clear the workspace first."
    )

    def run(self, ctx: StepContext) -> StepResult:
        spec = ctx.spec
        try:
            status = ctx.connection.status()
            arm_ids = ctx.connection.tuning_arm_ids()
            joints = ctx.connection.tuning_joint_names()
        except Exception as error:
            raise StepFailedError(
                f"the connection does not serve calibration: {plain_error(error)}"
            )
        if not status.connected:
            raise StepFailedError("the connection is not connected to the robot")
        if spec.arm_id not in arm_ids:
            raise StepFailedError(f"the connection drives the arms {arm_ids}, not {spec.arm_id}")
        missing = [name for name in spec.joint_names if name not in joints]
        if missing:
            raise StepFailedError(f"the connection cannot tune {missing}")
        q = _measured(ctx)
        if not np.isfinite(q).all():
            raise StepFailedError(f"joints read non-finite positions: {_deg(spec.joint_names, q)}")
        limits = {joint.name: urdf_limits(joint) for joint in spec.model.load().joints}
        tolerance = math.radians(LIMIT_TOLERANCE_DEG)
        outside = [
            f"{name} at {math.degrees(value):.1f} deg"
            for name, value in zip(spec.joint_names, q, strict=True)
            if not limits[name][0] - tolerance <= value <= limits[name][1] + tolerance
        ]
        if outside:
            raise StepFailedError(f"joints read outside their limits: {', '.join(outside)}")
        try:
            spec.require_workspace_guard()
        except NoWorkspaceGuardError as error:
            raise StepFailedError(f"{error} (the calibration module's floor_z_m and min_x_m)")
        applied = status.calibration.get(spec.arm_id)
        data: dict[str, Any] = {
            "arm_id": spec.arm_id,
            "calibration": None if applied is None else dict(applied),
            "measured_deg": _deg(spec.joint_names, q),
            "last_error": status.last_error,
            "home": "reached",
        }
        running = (
            "runs no calibration file (URDF masses, robot gains)"
            if applied is None
            else f"runs {applied['path']} from {applied['created']}"
        )
        try:
            ctx.confirm(
                f"Arm will move to home at {ctx.settings.speed_deg_s:g} deg/s. Clear the "
                "workspace. Enter to continue"
            )
        except SkipRequestedError:
            data["home"] = "skipped"
            return StepResult(f"checks passed; {spec.arm_id} {running}; not moved home", data)
        with ctx.session() as session:
            session.move_to(ctx.arm.home)
        return StepResult(f"checks passed; {spec.arm_id} {running}; at home", data)


class GravityPosesStep(Step):
    id = "gravity_poses"
    title = "Gravity poses"
    instructions = (
        "Visits the gravity identification poses, ramping slowly and through home when the "
        "straight way would come close to the table. At each pose the arm rests, then every "
        "joint that carries gravity moves a little up and back down, resting after each move, "
        "so the fit can tell the model's error from friction. Keep clear of the arm."
    )

    def supported(self, spec: ArmCalibrationSpec, caps: Capabilities) -> bool | str:
        if not caps.torque_readback:
            return "needs the motors' torque readback"
        return _recording_unavailable(caps)

    def run(self, ctx: StepContext) -> StepResult:
        ctx.require_recording()
        settings, arm = ctx.settings, ctx.arm
        ctx.say("generating the identification poses")
        pose_set = generate_poses(arm, count=settings.poses)
        ctx.run.pose_set = pose_set
        poses = pose_set.identification
        applied = ctx.connection.status().calibration.get(ctx.spec.arm_id)
        dithers = len(arm.measured_joints) * 2 * settings.dither_hold_s
        stops = [arm.home, *(np.asarray(p.joints_rad) for p in poses), arm.home]
        travel = sum(_ramp_s(a, b, settings.speed_deg_s) for a, b in itertools.pairwise(stops))
        expected = travel + len(poses) * (settings.hold_s + dithers)
        ctx.confirm(
            f"The arm will visit {len(poses)} poses, about {_minutes(expected)} of motion, "
            f"dithering {', '.join(arm.measured_joints)} by {settings.dither_deg:g} deg at each. "
            "Clear the workspace. Enter to start"
        )
        log = TrackingLog(ctx.spec.joint_names)
        visited: list[Vector] = []
        skipped: list[str] = []
        unreachable: list[str] = []
        first: float | None = None
        with ctx.session() as session:
            session.move_to(arm.home)
            try:
                for index, pose in enumerate(poses, 1):
                    ctx.progress(index - 1, len(poses), f"pose {index}/{len(poses)} {pose.name}")
                    if settings.confirm_each:
                        try:
                            ctx.confirm(
                                f"Pose {index}/{len(poses)} {pose.name} "
                                f"[{_angles(pose.joints_rad)}] deg: Enter to move there"
                            )
                        except SkipRequestedError:
                            skipped.append(pose.name)
                            continue
                    target = np.asarray(pose.joints_rad, dtype=np.float64)
                    path = plan_path(arm, session.target, target)
                    if path is None:
                        unreachable.append(pose.name)
                        continue
                    t0 = session.mark_pose(
                        pose.name,
                        {"index": index, "targets_deg": [math.degrees(v) for v in target]},
                    )
                    first = t0 if first is None else first
                    for waypoint in path:
                        session.ramp_to(waypoint, log)
                    visited.append(target)
                    session.hold(settings.hold_s, log)
                    for joint in arm.measured_joints:
                        for sign in (1.0, -1.0):
                            session.step_joint(
                                joint, sign * settings.dither_deg, settings.dither_hold_s
                            )
            finally:
                if first is not None:
                    end = session.mark_pose(END_LABEL, {"poses": len(visited)})
                    ctx.run.gravity_span = (first, end)
                    ctx.run.gravity_applied = None if applied is None else applied["path"]
            ctx.progress(len(poses), len(poses), "back home")
            session.retrace_home(visited)
        ctx.run.baseline = log
        data = {
            "poses": len(visited),
            "planned": len(poses),
            "skipped": skipped,
            "unreachable": unreachable,
            "rank": pose_set.rank,
            "condition_number": round(pose_set.condition_number, 2),
            "notes": list(pose_set.notes),
            "span": None if ctx.run.gravity_span is None else list(ctx.run.gravity_span),
            "calibration": ctx.run.gravity_applied,
        }
        return StepResult(f"visited {len(visited)} of {len(poses)} poses", data)


class GravityFitStep(Step):
    id = "gravity_fit"
    title = "Fit gravity"
    instructions = (
        "Fits the link masses and centres of mass to what the gravity poses recorded, keeping "
        "some poses out of the fit to score it. It is saved only when it does not make the "
        "held-out error worse on any joint, and only when you agree; it is then applied with "
        "the arm at home, and the replaced file is kept in the history."
    )

    def supported(self, spec: ArmCalibrationSpec, caps: Capabilities) -> bool | str:
        return _recording_unavailable(caps)

    def run(self, ctx: StepContext) -> StepResult:
        recording = ctx.require_recording()
        span = ctx.run.gravity_span
        if span is None:
            raise StepUnavailableError(
                "the gravity poses were not recorded in this procedure, so there is nothing to fit"
            )
        ctx.say("reading the recording")
        recording.wait_for(span[1])
        grec = recording.gravity(span[0], span[1])
        measurements = extract_measurements(grec, ctx.spec)
        if not measurements:
            raise StepFailedError("the recording holds no paired dithers to fit")
        offset = _compensation_offset(ctx, ctx.run.gravity_applied)
        ctx.say(f"fitting {len(measurements)} measurements")
        result = fit_gravity(ctx.arm, _corrected(ctx.arm, measurements, offset))
        ctx.check_abort()
        refusals = (
            save_refusals(result)
            if result.bounds_ok
            else [f"the fit failed its physical bounds: {'; '.join(result.bound_violations)}"]
        )
        holdout = _rms_table(
            result.rms_by_joint(holdout=True, after=False),
            result.rms_by_joint(holdout=True, after=True),
        )
        data: dict[str, Any] = {
            "holdout_rms_nm": holdout,
            "rank": result.rank,
            "condition_number": round(result.condition_number, 2),
            "at_bounds": list(result.at_bounds),
            "bounds_ok": result.bounds_ok,
            "refusals": refusals,
            "holdout_poses": list(result.holdout_poses),
            "measurements": len(measurements),
            "report": None,
            "saved": None,
        }
        if recording.directory is not None:
            data["report"] = str(self._write_report(result, recording.directory, refusals))
        table = ", ".join(
            f"{joint} {row['before']:.3f} -> {row['after']:.3f}" for joint, row in holdout.items()
        )
        if refusals:
            return StepResult(f"not saved: {'; '.join(refusals)}", data)
        bounds = f"; at bounds: {'; '.join(result.at_bounds)}" if result.at_bounds else ""
        try:
            ctx.confirm(
                f"Held-out RMS (Nm, before -> after): {table}; rank {result.rank}{bounds}. "
                "Save and apply? Enter to save"
            )
        except SkipRequestedError:
            return StepResult("not saved, as asked", data, skipped=True)
        path = calibration_path(ctx.spec.arm_id, ctx.settings.calibration_root)
        calibration = calibration_from_fit(
            result, ctx.spec, recording_dir=recording.directory, created=_now()
        )
        save_calibration(path, merge_with_stored(path, calibration, ctx.spec))
        data["saved"] = str(path)
        data.update(_apply(ctx, path))
        applied = "applied" if data["applied"] else "not applied"
        return StepResult(f"saved {path} and {applied}; held-out RMS {table}", data)

    @staticmethod
    def _write_report(result: FitResult, directory: Path, refusals: list[str]) -> Path:
        out = directory / GRAVITY_REPORT_DIR
        out.mkdir(parents=True, exist_ok=True)
        (out / "report.md").write_text(format_report(result, None, None, not_saved=refusals))
        write_measurements_csv(result, out / "measurements.csv")
        plot_resting_torque(result, out / "resting_torque.png")
        return out


class FloatCheckStep(Step):
    id = "float_check"
    title = "Float check"
    instructions = (
        "The arm moves to the float pose (the first gravity pose, where gravity acts on every "
        "joint, unless one is set), each joint that carries gravity goes limp in turn, held up "
        "by gravity compensation alone, and the fingertip's drift is measured; a good gravity "
        f"model keeps it under {DRIFT_LIMIT_MM:g} mm. Be ready to support the arm: a joint that "
        "drifts too far is held again at once."
    )

    def supported(self, spec: ArmCalibrationSpec, caps: Capabilities) -> bool | str:
        if not caps.gravity_compensation:
            return "the robot does not compensate gravity, so a floating joint would fall"
        return True

    @staticmethod
    def float_pose(ctx: StepContext) -> tuple[Vector, str]:
        """Where the joints float and why there: the configured pose, else the first gravity pose."""
        arm, configured = ctx.arm, ctx.settings.float_pose_deg
        if configured is not None:
            if len(configured) != len(arm.joint_names):
                raise StepFailedError(
                    f"float_pose_deg needs {len(arm.joint_names)} angles for "
                    f"{', '.join(arm.joint_names)}, got {len(configured)}"
                )
            pose = np.radians(np.asarray(configured, dtype=np.float64))
            if not arm.pose_ok(pose):
                raise StepFailedError(
                    f"the float pose [{_angles(pose)}] deg is outside the workspace guard"
                )
            return pose, "the configured float pose"
        pose_set = ctx.run.pose_set
        if pose_set is None:
            try:
                pose_set = generate_poses(arm, count=ctx.settings.poses)
            except NoWorkspaceGuardError:
                return np.asarray(arm.home, dtype=np.float64), "home (there is no workspace guard)"
        if not pose_set.identification:
            return np.asarray(arm.home, dtype=np.float64), "home (no gravity pose was found)"
        first = pose_set.identification[0]
        return np.asarray(first.joints_rad, dtype=np.float64), f"gravity pose {first.name}"

    def run(self, ctx: StepContext) -> StepResult:
        settings, arm = ctx.settings, ctx.arm
        fingertip = FingertipModel(arm)
        pose, where = self.float_pose(ctx)
        ctx.confirm(
            f"The arm will move to {where} [{_angles(pose)}] deg at {settings.speed_deg_s:g} "
            f"deg/s, then each of {', '.join(arm.measured_joints)} will float for "
            f"{settings.float_settle_s + settings.float_window_s:g} s. Clear the workspace. "
            "Enter to start"
        )
        joints: dict[str, dict[str, Any]] = {}
        with ctx.session() as session:
            session.move_to(pose)
            for index, joint in enumerate(arm.measured_joints, 1):
                ctx.progress(index - 1, len(arm.measured_joints), f"float {joint}")
                try:
                    ctx.confirm(
                        f"{joint} will go limp; be ready to support the arm. Enter to float it"
                    )
                except SkipRequestedError:
                    joints[joint] = {"skipped": True}
                    continue
                joints[joint] = self._float(ctx, session, fingertip, joint, index)
                # Holds the joint again at the float pose before the next one goes limp.
                session.move_to(pose)
        tried = {joint: row for joint, row in joints.items() if "passed" in row}
        passed = [joint for joint, row in tried.items() if row["passed"]]
        check = {
            "created": _now().isoformat(),
            "limit_mm": DRIFT_LIMIT_MM,
            "kd": settings.float_kd,
            "window_s": settings.float_window_s,
            "pose_deg": [round(math.degrees(float(v)), 3) for v in pose],
            "joints": joints,
        }
        data = {
            "joints": joints,
            "pose": where,
            "pose_deg": check["pose_deg"],
            "saved": _record_check(ctx, "float", check),
        }
        return StepResult(
            f"{len(passed)} of {len(tried)} joints held within the limit; arm holds at {where}",
            data,
        )

    @staticmethod
    def _float(
        ctx: StepContext, session: ArmSession, fingertip: FingertipModel, joint: str, index: int
    ) -> dict[str, Any]:
        settings = ctx.settings
        try:
            with session.floating(joint, settings.float_kd) as kd:
                if settings.float_settle_s > 0.0:
                    session.float_window(joint, settings.float_settle_s)
                event = session.mark_float(ctx.side, index, settings.float_window_s, kd)
                t, q = session.float_window(joint, settings.float_window_s)
        except JointDriftedError as drifted:
            return {"passed": False, "reason": str(drifted)}
        metrics = analyze_window(fingertip, event, t, q)
        return {**metrics.as_row(), "kd": kd}


class StepSweepStep(Step):
    id = "step_sweep"
    title = "Gain sweep"
    instructions = (
        "At home, every joint in turn steps a little, +1, -1, +3 and -3 degrees, at each tried "
        "stiffness (multiples of its robot kp, with its robot kd), resting after each step. The "
        "gains go back when each joint is done."
    )

    def supported(self, spec: ArmCalibrationSpec, caps: Capabilities) -> bool | str:
        if not caps.set_gains:
            return "the connection cannot change gains live"
        return _recording_unavailable(caps)

    def run(self, ctx: StepContext) -> StepResult:
        ctx.require_recording()
        settings, spec = ctx.settings, ctx.spec
        scales = settings.kp_scales
        deltas = settings.step_deltas_deg
        per_joint = len(scales) * len(deltas) * settings.step_hold_s
        ctx.confirm(
            f"Each of the {len(spec.joint_names)} joints will step "
            f"{', '.join(f'{d:+g}' for d in deltas)} deg at kp x "
            f"{', '.join(f'{s:g}' for s in scales)}, about "
            f"{_minutes(per_joint * len(spec.joint_names))}. Enter to start"
        )
        swept: dict[str, list[list[float]]] = {}
        start: float | None = None
        with ctx.session() as session:
            session.move_to(ctx.arm.home)
            start = ctx.clock.now()
            try:
                for number, joint in enumerate(spec.joint_names, 1):
                    ctx.progress(number - 1, len(spec.joint_names), f"sweep {joint}")
                    if settings.confirm_each:
                        try:
                            ctx.confirm(
                                f"{joint}: {len(scales) * len(deltas)} steps. Enter to step it"
                            )
                        except SkipRequestedError:
                            continue
                    kp0, kd0 = spec.gains.get(joint, session.gains(joint))
                    swept[joint] = []
                    try:
                        for scale in scales:
                            kp, kd = session.set_gains(joint, kp0 * scale, kd0)
                            swept[joint].append([kp, kd])
                            for delta in deltas:
                                session.step_joint(joint, delta, settings.step_hold_s)
                    finally:
                        try:
                            session.reset_gains()
                        except StepFailedError:
                            pass  # the session ended; ending it restored the gains
            finally:
                ctx.run.step_span = (start, ctx.clock.now())
        data = {
            "joints": swept,
            "kp_scales": list(scales),
            "steps_deg": list(deltas),
            "span": list(ctx.run.step_span),
        }
        return StepResult(f"swept {len(swept)} of {len(spec.joint_names)} joints", data)


class PickGainsStep(Step):
    id = "pick_gains"
    title = "Pick gains"
    instructions = (
        "Scores every step of the gain sweep (settling time, overshoot, resting error, buzz) "
        "and proposes, for each joint, the stiffest gains whose steps all passed, with every "
        "softer one passing too. Accepted gains are saved with the gravity fit and applied."
    )

    def supported(self, spec: ArmCalibrationSpec, caps: Capabilities) -> bool | str:
        return _recording_unavailable(caps)

    def run(self, ctx: StepContext) -> StepResult:
        recording = ctx.require_recording()
        span = ctx.run.step_span
        if span is None:
            raise StepUnavailableError("the gain sweep did not run in this procedure")
        ctx.say("reading the recording")
        recording.wait_for(span[1])
        steps = recording.steps(span[0], span[1])
        records = [r for r in extract_steps(steps) if r.event.joint in ctx.spec.joint_names]
        proposals = propose_gains(record.metrics for record in records)
        if recording.directory is not None:
            out = recording.directory / STEP_REPORT_DIR
            out.mkdir(parents=True, exist_ok=True)
            write_tables(records, out)
        rows = {
            joint: {
                "kp": proposal.kp,
                "kd": proposal.kd,
                "reason": proposal.reason,
                "rejected": dict(proposal.rejected),
            }
            for joint, proposal in sorted(proposals.items())
        }
        chosen = {joint: p.gains for joint, p in proposals.items() if p.gains is not None}
        data: dict[str, Any] = {"proposals": rows, "steps": len(records), "saved": None}
        if not chosen:
            return StepResult("no gains passed every step; nothing to save", data)
        listed = ", ".join(f"{j} kp={g[0]:g} kd={g[1]:g}" for j, g in sorted(chosen.items()))
        try:
            ctx.confirm(f"Proposed gains: {listed}. Save and apply? Enter to save")
        except SkipRequestedError:
            return StepResult("not saved, as asked", data, skipped=True)
        path, stored = _stored(ctx)
        base = stored or ArmCalibration.for_spec(ctx.spec)
        gains = {
            **(base.gains or {}),
            **{joint: JointGains(kp=kp, kd=kd) for joint, (kp, kd) in chosen.items()},
        }
        save_calibration(path, base.model_copy(update={"created": _now(), "gains": gains}))
        data["saved"] = str(path)
        data.update(_apply(ctx, path))
        return StepResult(f"saved gains for {sorted(chosen)} to {path}", data)


class ConfirmTrackingStep(Step):
    id = "confirm_tracking"
    title = "Confirm tracking"
    instructions = (
        "Visits a few random poses inside the workspace guard with the values now applied and "
        "measures how closely each joint follows its target, while moving and at rest, against "
        "the same measure from this run's gravity poses, taken before any new values were "
        "applied."
    )

    def run(self, ctx: StepContext) -> StepResult:
        settings, arm = ctx.settings, ctx.arm
        poses = random_poses(arm, settings.tracking_poses, settings.tracking_seed)
        if not poses:
            raise StepUnavailableError("no random pose inside the workspace guard was found")
        ctx.confirm(f"The arm will visit {len(poses)} random poses. Enter to start")
        log = TrackingLog(ctx.spec.joint_names)
        visited: list[Vector] = []
        with ctx.session() as session:
            session.move_to(arm.home)
            for index, pose in enumerate(poses, 1):
                ctx.progress(index - 1, len(poses), f"tracking pose {index}/{len(poses)}")
                if settings.confirm_each:
                    try:
                        ctx.confirm(
                            f"Tracking pose {index}/{len(poses)} [{_angles(pose)}] deg: Enter to "
                            "move there"
                        )
                    except SkipRequestedError:
                        continue
                path = plan_path(arm, session.target, pose)
                if path is None:
                    continue
                for waypoint in path:
                    session.ramp_to(waypoint, log)
                visited.append(pose)
                session.hold(settings.hold_s, log)
            session.retrace_home(visited)
        tracking = log.summary()
        baseline = None if ctx.run.baseline is None else ctx.run.baseline.summary()
        check = {
            "created": _now().isoformat(),
            "poses": len(visited),
            **tracking,
            "baseline": baseline,
        }
        data = {**check, "saved": _record_check(ctx, "tracking", check)}
        rest = tracking["rest_rms_deg"]
        worst = max(rest.values(), default=0.0)
        return StepResult(f"{len(visited)} poses; worst resting RMS {worst:.3f} deg", data)


class SummaryStep(Step):
    id = "summary"
    title = "Summary"
    instructions = (
        "Lists the arm's calibration file, the files it replaced, what the arm runs now and "
        "what to do next."
    )

    def run(self, ctx: StepContext) -> StepResult:
        path, stored = _stored(ctx)
        history = sorted(str(p) for p in (path.parent / HISTORY_DIR_NAME).glob("*.json"))
        try:
            applied = ctx.connection.status().calibration.get(ctx.spec.arm_id)
        except Exception as error:
            raise StepFailedError(f"the connection did not answer: {plain_error(error)}")
        steps = {state["id"]: state["status"] for state in ctx.step_states()}
        next_steps = [
            "Blueprints apply each arm's calibration file when the connection starts; restart "
            "the robot's normal blueprint to run it.",
        ]
        if stored is None or stored.gravity is None:
            next_steps.append("The arm has no fitted gravity yet: run gravity_poses and the fit.")
        if stored is None or not stored.gains:
            next_steps.append("No gains are saved: run step_sweep and pick_gains.")
        data: dict[str, Any] = {
            "file": None if stored is None else str(path),
            "created": None if stored is None else stored.created.isoformat(),
            "gravity_links": []
            if stored is None or stored.gravity is None
            else sorted(stored.gravity.links),
            "gains": {}
            if stored is None or not stored.gains
            else {j: [g.kp, g.kd] for j, g in sorted(stored.gains.items())},
            "checks": [] if stored is None or not stored.checks else sorted(stored.checks),
            "history": history,
            "applied": None if applied is None else dict(applied),
            "steps": steps,
            "next_steps": next_steps,
        }
        where = "no calibration file" if stored is None else str(path)
        return StepResult(f"{ctx.spec.arm_id}: {where}; {len(history)} earlier files", data)


def calibration_steps() -> list[Step]:
    """The calibration steps, in the order they run."""
    return [
        CheckStep(),
        GravityPosesStep(),
        GravityFitStep(),
        FloatCheckStep(),
        StepSweepStep(),
        PickGainsStep(),
        ConfirmTrackingStep(),
        SummaryStep(),
    ]
