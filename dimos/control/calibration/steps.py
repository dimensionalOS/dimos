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

1. ``check``: the connection answers, serves config sessions and drives this
   arm, its readings are finite and inside the joint limits, the arm has an explicit
   workspace guard, and the recorder writes (a probe marker lands in the
   recording within 2 s); then, after a confirmation, the arm goes home.
2. ``gravity_poses``: the bench-proven sweep. At each identification pose
   (``gravity_poses.generate_poses``) the pose is marked, the arm ramps there
   (through home when the straight ramp would dip), holds, and dithers each
   measured joint +d then -d, holding after each; an end marker closes it.
   What the arm runs meanwhile is taken from the connection (``config_get``),
   not from the file on disk.
3. ``gravity_fit``: fits the sweep's recording (``gravity_fit``) against the
   masses the arm ran, shows the held-out error of the URDF, of the running
   file and of the fit, and saves and applies the fit when it is no worse
   than either on any joint and the operator agrees. It refuses to fit when
   the arm's calibration file on disk holds other masses than the arm runs.
4. ``float_check``: floats each measured joint at up to two gravity poses
   where the gravity torque of the masses the arm runs is largest and it is
   clear of its limits (or at ``float_pose_deg``), after resting there
   ``hold_s``, and measures how far the fingertip drifts from the first limp
   sample (``float_analysis``); a joint with no gravity load anywhere is
   skipped, and robots that do not compensate gravity cannot float.
5. ``step_sweep``: at the sweep pose (the gravity pose farthest from every
   joint limit, or ``sweep_pose_deg``), steps every joint +1, -1, +3, -3 deg
   at each kp scale of its robot kp, with its robot kd times the square root
   of the scale, holding after each step.
6. ``pick_gains``: analyses those steps (``step_analysis.propose_gains``) and
   saves and applies the gains the operator accepts.
7. ``confirm_tracking``: revisits the held-out gravity poses with what is
   applied and compares how well the arm rests there with the gravity sweep's
   own rests at the same poses.
8. ``summary``: the calibration file, its history and what the arm runs.

Every step but ``check`` and ``summary`` needs a connection that serves
config sessions, and each what it changes (``config_keys``): the gravity
poses read the gains, the fit saves links, the float check floats joints
live, the gain sweep changes gains live and picking saves them. A step the
connection cannot do is listed unsupported with the reason and skipped; a
check is recorded only where the connection saves checks, and only in the
file the arm runs as it is on disk (else the step says it was not recorded).

A step that needs a recording reads it from the run's recording
(``recording.RecordingSource``), cut to the time it ran. Calibrations are saved
through the connection (``config_set`` with scope "persist"), with the arm at
home: it merges them into the arm's file (``config_keys().persist_paths``; the
replaced file goes to ``history/``) and applies it. When applying it would
change a joint's gravity torque too much there, the connection writes nothing;
the step then forces it only after a second confirmation, and otherwise saves
it without applying it.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
import itertools
import math
from pathlib import Path
import time
from typing import Any
import uuid

import numpy as np
from numpy.typing import NDArray

from dimos.control.calibration._inertial import LinkInertial, apply_inertial_overrides
from dimos.control.calibration.arm_session import (
    HOME_TOLERANCE_DEG,
    ArmSession,
    JointDriftedError,
    StepFailedError,
    TrackingLog,
    joint_reading,
    plain_error,
)
from dimos.control.calibration.float_analysis import (
    DRIFT_LIMIT_MM,
    FingertipModel,
    analyze_window,
)
from dimos.control.calibration.gravity_fit import (
    COMPENSATION_KEY,
    REPORT_DIR_NAME as GRAVITY_REPORT_DIR,
    Compensation,
    FitResult,
    calibration_from_fit,
    default_holdout,
    extract_measurements,
    fit_gravity,
    format_report,
    plot_resting_torque,
    recorded_compensation,
    save_refusals,
    to_urdf_error,
    write_measurements_csv,
)
from dimos.control.calibration.gravity_poses import ArmModel, Pose, generate_poses
from dimos.control.calibration.motion import (
    MIN_JERK_PEAK_RATIO,
    joint_limits,
    limit_error,
    limit_margin_rad,
    plan_path,
    speed_rad_s,
)
from dimos.control.calibration.procedure import (
    Capabilities,
    RunState,
    SkipRequestedError,
    Step,
    StepContext,
    StepResult,
    StepUnavailableError,
)
from dimos.control.calibration.recording import RecordingUnavailableError
from dimos.control.calibration.schema import (
    HISTORY_DIR_NAME,
    ArmCalibration,
    ArmCalibrationError,
    ArmConfigPatch,
    ArmConfigState,
    JointGains,
    load_for_spec,
    to_inertial_overrides,
)
from dimos.control.calibration.spec import (
    ArmCalibrationSpec,
    NoWorkspaceGuardError,
    urdf_limits,
)
from dimos.control.calibration.step_analysis import propose_gains
from dimos.control.calibration.step_report import (
    REPORT_DIR_NAME as STEP_REPORT_DIR,
    extract_steps,
    write_tables,
)
from dimos.control.connection.module_config import ConfigApplied, ConfigKeys

END_LABEL = "end"
# A reading this far past a joint limit fails the check.
LIMIT_TOLERANCE_DEG = 2.0
# The sweep pose keeps every joint this much farther from its limits than the
# sweep's largest excursion.
SWEEP_LIMIT_MARGIN_DEG = 2.0
# Float poses: per joint, this many of the poses where it carries the most
# gravity torque, with it this far from its limits; a joint whose torque stays
# below the floor everywhere has nothing to hold up.
FLOAT_POSES_PER_JOINT = 2
FLOAT_LIMIT_MARGIN_DEG = 5.0
MIN_FLOAT_GRAVITY_NM = 0.2
NO_GRAVITY_LOAD = "no gravity load to test"
# Two link masses differ when any value differs by more than this.
SAME_MASS_TOLERANCE = 1e-9

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


def _needs(
    caps: Capabilities,
    *,
    readable: str | None = None,
    session: str | None = None,
    persist: str | None = None,
) -> bool | str:
    """True when the connection serves config sessions and what is named, else why not.

    ``readable``, ``session`` and ``persist`` name an entry of the
    connection's ``config_keys`` lists, e.g. ``session="float"``.
    """
    keys = caps.keys
    if keys is None:
        return "the connection has not said what it can change (config_keys)"
    if not keys.sessions:
        return (
            "the connection does not serve config sessions: it was not started from the "
            "robot's calibration blueprint"
        )
    if readable is not None and readable not in keys.readable:
        return f"the connection does not report the arm's {readable}"
    if session is not None and session not in keys.session:
        return f"the connection cannot change {session} live"
    if persist is not None and persist not in keys.persist:
        return f"the connection cannot save {persist}"
    return True


def _compensates_gravity(keys: ConfigKeys) -> bool:
    """Whether the connection holds the arm's weight up (``features["gravity_compensation"]``)."""
    return keys.features.get("gravity_compensation") is True


def _measured(ctx: StepContext) -> Vector:
    """The arm's latest reading, outside a config session too; a stale one fails the step."""
    position, _, _ = joint_reading(ctx.state, ctx.keys().source, ctx.spec.joint_names, ctx.clock)
    return position


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


def _persist(
    ctx: StepContext, patch: ArmConfigPatch, *, force: bool = False, apply_now: bool = True
) -> ConfigApplied:
    """Save ``patch`` through the connection; raises ``StepFailedError`` when it cannot.

    A key the connection cannot save fails the step, rather than being left
    out quietly.
    """
    try:
        saved = ctx.persist(patch, force=force, apply_now=apply_now)
    except Exception as error:
        raise StepFailedError(f"the connection did not save the calibration: {plain_error(error)}")
    if saved.unsupported:
        reasons = "; ".join(
            f"{key}: {saved.why.get(key, 'not supported')}" for key in saved.unsupported
        )
        where = f"it saved {saved.path} without them" if saved.path else "nothing was saved"
        raise StepFailedError(f"the connection cannot save {reasons}; {where}")
    return saved


def _save(ctx: StepContext, patch: ArmConfigPatch) -> dict[str, Any]:
    """Save ``patch`` at home and apply it; a gravity step too large is forced on a second yes.

    The connection writes nothing when applying the saved file would step a
    joint's gravity torque too far at this pose. Then, on a yes, the save is
    forced; skipped, the file is saved without being applied.
    """
    ctx.check_abort()
    _ensure_home(ctx, "apply the calibration")
    saved = _persist(ctx, patch)
    refused, forced = saved.refused, False
    step = _gravity_step(saved)
    if refused is not None:
        try:
            ctx.confirm(
                f"The connection refused: {refused}. Apply anyway? Joints may move a little when "
                "their gravity torque changes; support the arm. Enter to force it"
            )
        except SkipRequestedError:
            saved = _persist(ctx, patch, apply_now=False)
        else:
            saved = _persist(ctx, patch, force=True)
            forced = True
            step = _gravity_step(saved)
    data: dict[str, Any] = {
        "saved": saved.path,
        "applied": saved.applied_now,
        "forced": forced,
        "gravity_step_nm": {joint: round(float(v), 3) for joint, v in step.items()},
    }
    if refused is not None and not forced:
        data["refused"] = refused
    return data


def _gravity_step(saved: ConfigApplied) -> dict[str, float]:
    """By joint, how much a save changed its gravity torque at this pose, or would have."""
    step: Mapping[str, Any] = saved.details.get("gravity_step_nm") or {}
    return {joint: float(value) for joint, value in step.items()}


def _persist_path(ctx: StepContext) -> Path:
    """Where the connection saves the arm's calibration (``config_keys().persist_paths``)."""
    path = ctx.keys().persist_paths.get(ctx.spec.arm_id)
    if path is None:
        raise StepFailedError(
            f"the connection does not say where it saves {ctx.spec.arm_id}'s calibration"
        )
    return Path(path)


def _stored(ctx: StepContext) -> tuple[Path, ArmCalibration | None]:
    """The arm's calibration file and what it holds, ``None`` when there is none.

    The file is read where the connection saves it, on this host.
    """
    path = _persist_path(ctx)
    if not path.exists():
        return path, None
    try:
        return path, load_for_spec(ctx.spec, path)
    except ArmCalibrationError as error:
        raise StepFailedError(f"the stored calibration cannot be updated: {error}")


def _record_check(ctx: StepContext, name: str, value: dict[str, Any]) -> dict[str, Any]:
    """Save ``value`` as ``checks[name]`` in the arm's calibration file, if it has one.

    Returns the step's data on it: ``saved``, the file written or ``None``,
    and ``not_saved``, why, when the connection would not record it. Nothing
    is saved when the connection cannot save checks or the arm has no file.
    The connection refuses a check while the arm does not run the file as
    it is on disk (a fit saved without being applied): the check describes
    what the arm ran, not that file. That skips the record, not the step. A
    check changes nothing the arm runs, so the connection does not apply it.
    """
    if "checks" not in ctx.keys().persist:
        return {"saved": None}
    _, stored = _stored(ctx)
    if stored is None:
        return {"saved": None}
    try:
        saved = ctx.persist(ArmConfigPatch(arm_id=ctx.spec.arm_id, checks={name: value}))
    except Exception as error:
        raise StepFailedError(f"the connection did not save the calibration: {plain_error(error)}")
    if saved.unsupported:
        why = "; ".join(saved.why.get(key, "not supported") for key in saved.unsupported)
        ctx.say(f"the {name} check is not recorded: {why}")
        return {"saved": None, "not_saved": why}
    return {"saved": saved.path}


def _not_recorded(data: Mapping[str, Any]) -> str:
    """ "; not recorded: why" for a step message when the connection refused its check."""
    why = data.get("not_saved")
    return "" if why is None else f"; not recorded: {why}"


def _applied_now(ctx: StepContext) -> ArmConfigState | None:
    """How the arm is driven now, as its connection says: its file and link masses."""
    try:
        state = ctx.connection.config_get().get(ctx.spec.arm_id)
        return None if state is None else ArmConfigState.from_state(state)
    except Exception as error:
        raise StepFailedError(f"the connection did not say what the arm runs: {plain_error(error)}")


def _links_of(applied: ArmConfigState | None) -> dict[str, LinkInertial]:
    if applied is None:
        return {}
    return {
        name: LinkInertial(mass_kg=float(link.mass_kg), com_m=link.com_m)
        for name, link in applied.links.items()
    }


def _same_links(a: Mapping[str, LinkInertial], b: Mapping[str, LinkInertial]) -> bool:
    if set(a) != set(b):
        return False
    return all(
        np.allclose(
            [a[name].mass_kg, *a[name].com_m],
            [b[name].mass_kg, *b[name].com_m],
            rtol=SAME_MASS_TOLERANCE,
            atol=SAME_MASS_TOLERANCE,
        )
        for name in a
    )


def _disk_differs(ctx: StepContext, applied: ArmConfigState | None) -> str | None:
    """Why the arm's calibration file on disk holds other masses than the arm runs, or ``None``.

    No file on disk is no conflict: a fit then creates it.
    """
    path, stored = _stored(ctx)
    if stored is None or _same_links(to_inertial_overrides(stored), _links_of(applied)):
        return None
    file = None if applied is None else applied.file
    runs = (
        "its URDF masses (no calibration file)"
        if file is None
        else f"the calibration created {file.created} ({file.path})"
    )
    return (
        f"the arm runs {runs}, the file on disk is the one created "
        f"{stored.created.isoformat()} ({path}), with other masses: restart the robot or apply "
        "the file first"
    )


def _rms_table(
    before: dict[str, float], running: dict[str, float], after: dict[str, float]
) -> dict[str, dict[str, float]]:
    return {
        joint: {
            "before": round(before[joint], 4),
            "running": round(running.get(joint, math.nan), 4),
            "after": round(after[joint], 4),
        }
        for joint in before
    }


def gravity_poses(ctx: StepContext) -> list[Pose]:
    """This run's gravity identification poses, generated once when the sweep did not run."""
    run: RunState = ctx.run
    pose_set = run.pose_set or run.generated_poses
    if pose_set is None:
        try:
            pose_set = generate_poses(ctx.arm, count=ctx.settings.poses)
        except NoWorkspaceGuardError as error:
            raise StepFailedError(f"{error} (the calibration module's floor_z_m and min_x_m)")
        run.generated_poses = pose_set
    return list(pose_set.identification)


def reachable_from_home(arm: ArmModel, poses: Sequence[Pose]) -> list[Pose]:
    """The poses a straight ramp from home reaches with the clearance.

    ``plan_path`` goes straight or through home, so such a pose is reachable
    from any pose that can ramp home.
    """
    return [pose for pose in poses if arm.transition_ok(arm.home, pose.joints_rad)]


def _configured_pose(ctx: StepContext, degrees: Sequence[float], setting: str) -> Vector:
    arm = ctx.arm
    if len(degrees) != len(arm.joint_names):
        raise StepFailedError(
            f"{setting} needs {len(arm.joint_names)} angles for {', '.join(arm.joint_names)}, "
            f"got {len(degrees)}"
        )
    pose = np.radians(np.asarray(degrees, dtype=np.float64))
    if not arm.pose_ok(pose):
        raise StepFailedError(
            f"the {setting} pose [{_angles(pose)}] deg is outside the workspace guard"
        )
    return pose


def sweep_excursion_deg(deltas: Sequence[float]) -> float:
    """How far the sweep's steps take a joint from where it started, deg."""
    return max(abs(total) for total in itertools.accumulate(deltas))


def sweep_pose(ctx: StepContext) -> tuple[Vector, str]:
    """Where the gain sweep steps the joints, and its name.

    ``sweep_pose_deg`` when set, else the gravity pose whose joint nearest a
    limit is farthest from it; either way every joint must stay at least the
    sweep's excursion plus ``SWEEP_LIMIT_MARGIN_DEG`` from its limits, or the
    step fails before the arm moves.
    """
    spec, settings = ctx.spec, ctx.settings
    lower, upper = joint_limits(spec)
    needed = math.radians(sweep_excursion_deg(settings.step_deltas_deg) + SWEEP_LIMIT_MARGIN_DEG)
    if settings.sweep_pose_deg is not None:
        pose = _configured_pose(ctx, settings.sweep_pose_deg, "sweep_pose_deg")
        close = limit_error(spec.joint_names, pose, lower, upper, needed)
        if close is not None:
            raise StepFailedError(
                f"the sweep pose has joints within {math.degrees(needed):g} deg of a limit: {close}"
            )
        return pose, "the configured sweep pose"
    poses = reachable_from_home(ctx.arm, gravity_poses(ctx))
    scored = [(limit_margin_rad(np.asarray(p.joints_rad), lower, upper), p) for p in poses]
    if not scored or max(margin for margin, _ in scored) < needed:
        raise StepFailedError(
            f"no gravity pose keeps every joint {math.degrees(needed):g} deg from its limits for "
            "the sweep; set sweep_pose_deg"
        )
    home = np.asarray(ctx.arm.home, dtype=np.float64)

    def preference(item: tuple[float, Pose]) -> tuple[float, float]:
        # The widest margin, to a tenth of a degree; then the least travel from home.
        margin, pose = item
        travel = float(np.max(np.abs(np.asarray(pose.joints_rad) - home)))
        return (round(math.degrees(margin), 1), -travel)

    _, best = max(scored, key=preference)
    return np.asarray(best.joints_rad, dtype=np.float64), f"gravity pose {best.name}"


def float_plan(
    ctx: StepContext,
) -> tuple[dict[str, list[tuple[str, Vector, float]]], dict[str, str]]:
    """Per measured joint, where to float it: up to ``FLOAT_POSES_PER_JOINT`` of (name, pose, torque).

    The candidates are ``float_pose_deg`` when set, else this run's gravity
    poses. Each joint floats where the gravity torque of the masses the arm
    runs now (``config_get``) is largest, with it at least
    ``FLOAT_LIMIT_MARGIN_DEG`` from its limits. A joint whose
    torque stays under ``MIN_FLOAT_GRAVITY_NM`` at every candidate, or that
    has no candidate clear of its limits, is returned in the second dict
    with why.
    """
    arm, settings = ctx.arm, ctx.settings
    if settings.float_pose_deg is not None:
        pose = _configured_pose(ctx, settings.float_pose_deg, "float_pose_deg")
        candidates = [("the configured float pose", pose)]
    else:
        candidates = [
            (p.name, np.asarray(p.joints_rad, dtype=np.float64))
            for p in reachable_from_home(arm, gravity_poses(ctx))
        ]
    lower, upper = joint_limits(ctx.spec)
    margin = math.radians(FLOAT_LIMIT_MARGIN_DEG)
    model = arm.model.copy()
    apply_inertial_overrides(model, _links_of(_applied_now(ctx)))
    applied = ArmModel(ctx.spec, model)
    torques = [(name, q, applied.gravity(applied.q(q))) for name, q in candidates]
    plan: dict[str, list[tuple[str, Vector, float]]] = {}
    skipped: dict[str, str] = {}
    for joint in arm.measured_joints:
        i = arm.joint_names.index(joint)
        largest = max((abs(float(g[i])) for _, _, g in torques), default=0.0)
        if largest < MIN_FLOAT_GRAVITY_NM:
            skipped[joint] = (
                f"{NO_GRAVITY_LOAD}: at most {largest:.2f} Nm of model gravity torque over the "
                f"poses (floor {MIN_FLOAT_GRAVITY_NM:g} Nm)"
            )
            continue
        usable = sorted(
            (
                (name, q, float(g[i]))
                for name, q, g in torques
                if abs(float(g[i])) >= MIN_FLOAT_GRAVITY_NM
                and min(q[i] - lower[i], upper[i] - q[i]) >= margin
            ),
            key=lambda row: -abs(row[2]),
        )
        if not usable:
            skipped[joint] = (
                f"every pose where it carries gravity has it within {FLOAT_LIMIT_MARGIN_DEG:g} deg "
                "of a limit"
            )
            continue
        plan[joint] = usable[:FLOAT_POSES_PER_JOINT]
    return plan, skipped


class CheckStep(Step):
    id = "check"
    title = "Check the arm"
    instructions = (
        "Checks that the connection is running, serves calibration and drives this arm, that "
        "every joint reads a finite position inside its limits, and that the arm has a workspace guard "
        "(the table height and the line its fingertip stays in front of), and that the recorder "
        "is writing. It reports the calibration file the arm runs now. Then the arm moves home: "
        "clear the workspace first."
    )

    def run(self, ctx: StepContext) -> StepResult:
        spec = ctx.spec
        try:
            status = ctx.connection.status()
            keys = ctx.connection.config_keys()
            states = ctx.connection.config_get()
        except Exception as error:
            raise StepFailedError(
                f"the connection does not serve calibration: {plain_error(error)}"
            )
        if not status.connected:
            raise StepFailedError("the connection is not connected to the robot")
        if not keys.sessions:
            raise StepFailedError(
                "the connection does not serve config sessions: start the robot's calibration "
                "blueprint (<robot>-calibrate), which deploys it with config_sessions=True"
            )
        joints = keys.targets.get(spec.arm_id)
        if joints is None:
            raise StepFailedError(
                f"the connection drives the arms {list(keys.targets)}, not {spec.arm_id}"
            )
        missing = [name for name in spec.joint_names if name not in joints]
        if missing:
            raise StepFailedError(f"the connection does not drive {missing}")
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
        state = states.get(spec.arm_id)
        applied = None if state is None else state.file
        data: dict[str, Any] = {
            "arm_id": spec.arm_id,
            "calibration": None if applied is None else applied.model_dump(),
            "measured_deg": _deg(spec.joint_names, q),
            "last_error": status.last_error,
            "recording": self._check_recorder(ctx),
            "home": "reached",
        }
        running = (
            "runs no calibration file (URDF masses, robot gains)"
            if applied is None
            else f"runs {applied.path} from {applied.created}"
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

    @staticmethod
    def _check_recorder(ctx: StepContext) -> str:
        """ "writing" once a probe marker lands in the recording; why not when nothing records."""
        recording = ctx.recording
        if recording is None or ctx.caps.recording_unavailable is not None:
            return ctx.caps.recording_unavailable or "nothing is being recorded"
        token = f"check-{ctx.run.run_id}-{uuid.uuid4().hex[:8]}"
        since = time.time()
        ctx.probe_recording(token)
        try:
            recording.wait_for_probe(token, since)
        except RecordingUnavailableError as error:
            raise StepFailedError(f"{error}; nothing has moved")
        return "writing"


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
        needs = _needs(caps, readable="gains")
        return _recording_unavailable(caps) if needs is True else needs

    def run(self, ctx: StepContext) -> StepResult:
        ctx.require_recording()
        settings, arm = ctx.settings, ctx.arm
        ctx.say("generating the identification poses")
        pose_set = generate_poses(arm, count=settings.poses)
        ctx.run.pose_set = pose_set
        poses = pose_set.identification
        applied = _applied_now(ctx)
        compensation = Compensation(_compensates_gravity(ctx.keys()), _links_of(applied))
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
                        {
                            "index": index,
                            "targets_deg": [math.degrees(v) for v in target],
                            COMPENSATION_KEY: compensation.as_data(),
                        },
                    )
                    first = t0 if first is None else first
                    log.pose = pose.name
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
                    ctx.run.gravity_applied = applied
            ctx.progress(len(poses), len(poses), "back home")
            session.retrace_home(visited)
        ctx.run.baseline = log
        file = None if applied is None else applied.file
        data = {
            "poses": len(visited),
            "planned": len(poses),
            "skipped": skipped,
            "unreachable": unreachable,
            "rank": pose_set.rank,
            "condition_number": round(pose_set.condition_number, 2),
            "notes": list(pose_set.notes),
            "span": None if ctx.run.gravity_span is None else list(ctx.run.gravity_span),
            "calibration": None if file is None else file.path,
            "calibration_sha256": None if file is None else file.sha256,
        }
        return StepResult(f"visited {len(visited)} of {len(poses)} poses", data)


class GravityFitStep(Step):
    id = "gravity_fit"
    title = "Fit gravity"
    instructions = (
        "Fits the link masses and centres of mass to what the gravity poses recorded, keeping "
        "some poses out of the fit to score it against the URDF and against the file the arm "
        "runs. It is saved only when it is no worse than either on any joint, and only when you "
        "agree; it is then applied with the arm at home, and the replaced file is kept in the "
        "history. It does not fit when the file on disk holds other masses than the arm runs."
    )

    def supported(self, spec: ArmCalibrationSpec, caps: Capabilities) -> bool | str:
        needs = _needs(caps, persist="links")
        return _recording_unavailable(caps) if needs is True else needs

    def run(self, ctx: StepContext) -> StepResult:
        recording = ctx.require_recording()
        span = ctx.run.gravity_span
        if span is None:
            raise StepUnavailableError(
                "the gravity poses were not recorded in this procedure, so there is nothing to fit"
            )
        applied = ctx.run.gravity_applied
        differs = _disk_differs(ctx, applied)
        if differs is not None:
            raise StepUnavailableError(f"not fitted: {differs}")
        ctx.say("reading the recording")
        recording.wait_for(span[1])
        grec = recording.gravity(span[0], span[1])
        measurements, incomplete = extract_measurements(grec, ctx.spec)
        if not measurements:
            raise StepFailedError("the recording holds no paired dithers to fit")
        compensation = recorded_compensation(grec)
        if compensation is None:
            raise StepFailedError("the recording does not say what gravity the poses ran with")
        try:
            errors = to_urdf_error(ctx.arm, measurements, compensation)
        except ValueError as error:
            raise StepFailedError(f"cannot model the masses the sweep ran: {error}")
        ctx.say(f"fitting {len(measurements)} measurements")
        result = fit_gravity(ctx.arm, errors, incomplete_pairs=incomplete)
        ctx.check_abort()
        # The raw resting torque is the error of the masses the arm ran.
        running = result.rms_of(np.asarray([m.tau_pd_nm for m in measurements]), holdout=True)
        refusals = (
            save_refusals(result, running)
            if result.bounds_ok
            else [f"the fit failed its physical bounds: {'; '.join(result.bound_violations)}"]
        )
        holdout = _rms_table(
            result.rms_by_joint(holdout=True, after=False),
            running,
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
            "incomplete_pairs": len(incomplete),
            "report": None,
            "saved": None,
        }
        if recording.directory is not None:
            data["report"] = str(self._write_report(result, recording.directory, refusals, running))
        table = ", ".join(
            f"{joint} {row['before']:.3f} -> {row['running']:.3f} -> {row['after']:.3f}"
            for joint, row in holdout.items()
        )
        if refusals:
            return StepResult(f"not saved: {'; '.join(refusals)}", data)
        bounds = f"; at bounds: {'; '.join(result.at_bounds)}" if result.at_bounds else ""
        try:
            ctx.confirm(
                f"Held-out RMS (Nm, URDF -> running -> new): {table}; rank {result.rank}{bounds}. "
                "Save and apply? Enter to save"
            )
        except SkipRequestedError:
            return StepResult("not saved, as asked", data, skipped=True)
        calibration = calibration_from_fit(
            result, ctx.spec, recording_dir=recording.directory, created=_now()
        )
        gravity = calibration.gravity
        assert gravity is not None
        # The connection merges it into the stored file: the gains and notes stay,
        # the checks only when the masses did not change.
        patch = ArmConfigPatch(
            arm_id=ctx.spec.arm_id,
            links=gravity.links,
            fit=gravity.fit,
            friction_nm=calibration.friction_nm,
        )
        data.update(_save(ctx, patch))
        outcome = "applied" if data["applied"] else "not applied"
        return StepResult(f"saved {data['saved']} and {outcome}; held-out RMS {table}", data)

    @staticmethod
    def _write_report(
        result: FitResult, directory: Path, refusals: list[str], running: dict[str, float]
    ) -> Path:
        out = directory / GRAVITY_REPORT_DIR
        out.mkdir(parents=True, exist_ok=True)
        (out / "report.md").write_text(
            format_report(result, None, None, not_saved=refusals, running_rms=running)
        )
        write_measurements_csv(result, out / "measurements.csv")
        plot_resting_torque(result, out / "resting_torque.png")
        return out


class FloatCheckStep(Step):
    id = "float_check"
    title = "Float check"
    instructions = (
        "Each joint that carries gravity goes limp in turn, held up by gravity compensation "
        "alone, at the gravity poses where it carries the most (or at the float pose, if one "
        "is set). The arm rests at the pose first; the fingertip's drift is measured from the "
        "moment the joint goes limp, and a good gravity model keeps it under "
        f"{DRIFT_LIMIT_MM:g} mm. A joint with no gravity load anywhere is skipped. You confirm "
        "each move. Be ready to support the arm: a joint that drifts or speeds up is held "
        "again at once."
    )

    def supported(self, spec: ArmCalibrationSpec, caps: Capabilities) -> bool | str:
        if caps.keys is not None and not _compensates_gravity(caps.keys):
            return "the robot does not compensate gravity, so a floating joint would fall"
        return _needs(caps, session="float")

    def run(self, ctx: StepContext) -> StepResult:
        settings, arm = ctx.settings, ctx.arm
        fingertip = FingertipModel(arm)
        plan, no_load = float_plan(ctx)
        joints: dict[str, dict[str, Any]] = {
            joint: {"skipped": True, "reason": why} for joint, why in no_load.items()
        }
        where = (
            "the configured float pose"
            if settings.float_pose_deg is not None
            else "the gravity poses where each joint carries the most gravity torque"
        )
        floats = sum(len(poses) for poses in plan.values())
        if floats:
            skipped = f"; {', '.join(no_load)} skipped ({NO_GRAVITY_LOAD})" if no_load else ""
            ctx.confirm(
                f"{floats} floats of {', '.join(plan)} at {where}, "
                f"{settings.hold_s + settings.float_window_s:g} s each{skipped}. You "
                "confirm each move. Clear the workspace. Enter to start"
            )
            last: str | None = None
            with ctx.session() as session:
                number = 0
                for joint, poses in plan.items():
                    rows: list[dict[str, Any]] = []
                    for name, pose, torque in poses:
                        number += 1
                        ctx.progress(number - 1, floats, f"float {joint} at {name}")
                        row = {
                            "pose": name,
                            "pose_deg": [round(math.degrees(float(v)), 3) for v in pose],
                            "gravity_nm": round(torque, 3),
                        }
                        try:
                            ctx.confirm(
                                f"Float {joint} at {name} [{_angles(pose)}] deg, where it carries "
                                f"{torque:+.2f} Nm: the arm moves there at "
                                f"{settings.speed_deg_s:g} deg/s, then {joint} goes limp; be "
                                "ready to support the arm. Enter to move and float it"
                            )
                        except SkipRequestedError:
                            rows.append({**row, "skipped": True})
                            continue
                        session.move_to(pose)
                        session.hold(settings.hold_s)
                        last = name
                        # Ending the float holds the joint again where it is; the next
                        # move takes every joint to the next pose.
                        rows.append({**row, **self._float(ctx, session, fingertip, joint, number)})
                    joints[joint] = self._summary(rows)
            ctx.progress(floats, floats, "floats done")
        tried = {joint: row for joint, row in joints.items() if "passed" in row}
        passed = [joint for joint, row in tried.items() if row["passed"]]
        check = {
            "created": _now().isoformat(),
            "limit_mm": DRIFT_LIMIT_MM,
            "kd": settings.float_kd,
            "window_s": settings.float_window_s,
            "poses": where,
            "joints": joints,
        }
        data = {
            "joints": joints,
            "pose": where,
            **(_record_check(ctx, "float", check) if tried else {"saved": None}),
        }
        held = "" if not floats or last is None else f"; arm holds at {last}"
        return StepResult(
            f"{len(passed)} of {len(tried)} joints held within the limit at every pose"
            f"{f'; {len(no_load)} with no gravity load' if no_load else ''}{held}"
            f"{_not_recorded(data)}",
            data,
        )

    @staticmethod
    def _summary(rows: list[dict[str, Any]]) -> dict[str, Any]:
        """One joint's float verdict: its worst pose, every pose's row, and whether all passed."""
        tried = [row for row in rows if "passed" in row]
        if not tried:
            return {"skipped": True, "reason": "skipped at the prompt", "poses": rows}
        failed = [row for row in tried if not row["passed"]]
        worst = failed[0] if failed else max(tried, key=lambda row: row["fingertip_drift_mm"])
        summary = {key: value for key, value in worst.items() if key != "pose_deg"}
        return {**summary, "passed": not failed, "poses": rows}

    @staticmethod
    def _float(
        ctx: StepContext, session: ArmSession, fingertip: FingertipModel, joint: str, index: int
    ) -> dict[str, Any]:
        settings = ctx.settings
        try:
            with session.floating(joint, settings.float_kd) as kd:
                event = session.mark_float(ctx.side, index, settings.float_window_s, kd)
                t, q = session.float_window(joint, settings.float_window_s)
        except JointDriftedError as drifted:
            return {"passed": False, "reason": str(drifted)}
        metrics = analyze_window(fingertip, event, t, q)
        return {**metrics.as_row(), "kd": kd}


class StepSweepStep(Step):
    """The gain sweep: each joint steps at each kp scale, at a pose clear of every limit.

    Home sits on a joint limit on several arms (the Metal's and B601's
    shoulder and elbow, the OpenArm's elbow), where half the steps would pass
    the limit, so the sweep runs at :func:`sweep_pose`. At kp scale ``s`` a
    joint is held with ``kp0 * s`` and ``kd0 * sqrt(s)``, as the bench sweep
    did, keeping its damping ratio; the connection clamps kd to the MIT
    frame's 5 N m s/rad, so a joint already at 5 stays there. ``pick_gains``
    takes a scale only when every lower scale on that joint passed too.
    """

    id = "step_sweep"
    title = "Gain sweep"
    instructions = (
        "The arm moves to the sweep pose (clear of every joint limit), and every joint in turn "
        "steps a little, +1, -1, +3 and -3 degrees, at each tried stiffness (multiples of its "
        "robot kp, its robot kd times the square root of the multiple), resting after each "
        "step. The gains go back when each joint is done, and the arm returns home."
    )

    def supported(self, spec: ArmCalibrationSpec, caps: Capabilities) -> bool | str:
        needs = _needs(caps, session="gains")
        return _recording_unavailable(caps) if needs is True else needs

    def run(self, ctx: StepContext) -> StepResult:
        ctx.require_recording()
        settings, spec = ctx.settings, ctx.spec
        scales = settings.kp_scales
        deltas = settings.step_deltas_deg
        pose, where = sweep_pose(ctx)
        per_joint = len(scales) * len(deltas) * settings.step_hold_s
        ctx.confirm(
            f"The arm will move to {where} [{_angles(pose)}] deg at {settings.speed_deg_s:g} "
            f"deg/s; there each of the {len(spec.joint_names)} joints will step "
            f"{', '.join(f'{d:+g}' for d in deltas)} deg at kp x "
            f"{', '.join(f'{s:g}' for s in scales)} (kd x the square root), about "
            f"{_minutes(per_joint * len(spec.joint_names))}, then it returns home. Clear the "
            "workspace. Enter to start"
        )
        swept: dict[str, list[list[float]]] = {}
        start: float | None = None
        with ctx.session() as session:
            session.move_to(pose)
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
                            kp, kd = session.set_gains(joint, kp0 * scale, kd0 * math.sqrt(scale))
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
            ctx.progress(len(spec.joint_names), len(spec.joint_names), "back home")
            session.move_to(ctx.arm.home)
        data = {
            "joints": swept,
            "kp_scales": list(scales),
            "steps_deg": list(deltas),
            "pose": where,
            "pose_deg": [round(math.degrees(float(v)), 3) for v in pose],
            "span": list(ctx.run.step_span),
        }
        return StepResult(f"swept {len(swept)} of {len(spec.joint_names)} joints at {where}", data)


class PickGainsStep(Step):
    id = "pick_gains"
    title = "Pick gains"
    instructions = (
        "Scores every step of the gain sweep (settling time, overshoot, resting error, buzz) "
        "and proposes, for each joint, the stiffest gains whose steps all passed, with every "
        "softer one passing too. Accepted gains are saved with the gravity fit and applied."
    )

    def supported(self, spec: ArmCalibrationSpec, caps: Capabilities) -> bool | str:
        needs = _needs(caps, persist="gains")
        return _recording_unavailable(caps) if needs is True else needs

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
        gains = {joint: JointGains(kp=kp, kd=kd) for joint, (kp, kd) in chosen.items()}
        # Merged by joint into the stored file and applied from it. Applying a file
        # (DamiaoArm.apply_calibration) drops every live gain override, so from here
        # on the arm holds the saved gains, in any session too: confirm_tracking
        # runs on the file's gains, as it should.
        data.update(_save(ctx, ArmConfigPatch(arm_id=ctx.spec.arm_id, gains=gains)))
        return StepResult(f"saved gains for {sorted(chosen)} to {data['saved']}", data)


class ConfirmTrackingStep(Step):
    id = "confirm_tracking"
    title = "Confirm tracking"
    instructions = (
        "Revisits the gravity poses the fit held out with the values now applied and measures "
        "how closely each joint follows its target, while moving and at rest, against the same "
        "measure at the same poses from this run's gravity sweep, taken before any new values "
        "were applied."
    )

    def supported(self, spec: ArmCalibrationSpec, caps: Capabilities) -> bool | str:
        return _needs(caps)

    @staticmethod
    def poses(ctx: StepContext) -> tuple[list[Pose], bool]:
        """The poses to visit, and whether the gravity sweep's rests there are the baseline.

        The held-out poses of this run's fit (else every fifth sweep pose, else
        every sweep pose), so before and after are paired; without a sweep in
        this run, the generated poses' held-out ones, with no baseline.
        """
        baseline = ctx.run.baseline
        candidates = {pose.name: pose for pose in gravity_poses(ctx)}
        held = [name for name in baseline.poses if name in candidates] if baseline else []
        if held:
            fitted = (ctx.results.get("gravity_fit") or {}).get("holdout_poses") or []
            names = [name for name in fitted if name in held] or default_holdout(held) or held
        else:
            names = default_holdout(list(candidates)) or list(candidates)
        names = names[: ctx.settings.tracking_poses]
        return [candidates[name] for name in names], bool(held)

    def run(self, ctx: StepContext) -> StepResult:
        settings, arm = ctx.settings, ctx.arm
        poses, paired = self.poses(ctx)
        if not poses:
            raise StepUnavailableError("there is no gravity pose to revisit")
        ctx.confirm(
            f"The arm will revisit {len(poses)} held-out gravity poses "
            f"({', '.join(p.name for p in poses)}). Enter to start"
        )
        log = TrackingLog(ctx.spec.joint_names)
        visited: list[Vector] = []
        names: list[str] = []
        with ctx.session() as session:
            session.move_to(arm.home)
            for index, pose in enumerate(poses, 1):
                ctx.progress(index - 1, len(poses), f"tracking pose {index}/{len(poses)}")
                target = np.asarray(pose.joints_rad, dtype=np.float64)
                if settings.confirm_each:
                    try:
                        ctx.confirm(
                            f"Tracking pose {index}/{len(poses)} {pose.name} [{_angles(target)}] "
                            "deg: Enter to move there"
                        )
                    except SkipRequestedError:
                        continue
                path = plan_path(arm, session.target, target)
                if path is None:
                    continue
                log.pose = pose.name
                for waypoint in path:
                    session.ramp_to(waypoint, log)
                visited.append(target)
                names.append(pose.name)
                session.hold(settings.hold_s, log)
            session.retrace_home(visited)
        tracking = log.summary()
        baseline = (
            ctx.run.baseline.summary(poses=names)
            if paired and ctx.run.baseline is not None
            else None
        )
        check = {
            "created": _now().isoformat(),
            "poses": len(visited),
            "pose_names": names,
            **tracking,
            "baseline": baseline,
        }
        data = {**check, **_record_check(ctx, "tracking", check)}
        rest = tracking["rest_rms_deg"]
        worst = max(rest.values(), default=0.0)
        return StepResult(
            f"{len(visited)} poses; worst resting RMS {worst:.3f} deg{_not_recorded(data)}", data
        )


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
            state = ctx.connection.config_get().get(ctx.spec.arm_id)
        except Exception as error:
            raise StepFailedError(f"the connection did not answer: {plain_error(error)}")
        applied = None if state is None else state.file
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
            "applied": None if applied is None else applied.model_dump(),
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
