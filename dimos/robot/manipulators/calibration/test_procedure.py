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

"""The calibration procedure end to end on a simulated arm, in simulated time."""

from __future__ import annotations

from collections import defaultdict
from collections.abc import Callable, Iterator
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
from typing import Any

import numpy as np
import pinocchio as pin
from pydantic import ValidationError
import pytest

from dimos.control.connection.arm_config import ConfigPatch, JointGains, LinkGravity
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.calibration import (
    arm_session as arm_session_module,
    procedure as procedure_module,
)
from dimos.robot.manipulators.calibration._calibration_test_helpers import (
    PERTURBATION,
    perturbed_model,
    robot_specs_or_skip,
)
from dimos.robot.manipulators.calibration._fake_tuning_arm import (
    FakeTuningArm,
    MemoryRecording,
    SimClock,
    drive,
)
from dimos.robot.manipulators.calibration._inertial import LinkInertial, apply_inertial_overrides
from dimos.robot.manipulators.calibration.arm_session import StepFailedError
from dimos.robot.manipulators.calibration.gravity_poses import (
    ArmModel,
    body_joint_id,
    generate_poses,
)
from dimos.robot.manipulators.calibration.motion import joint_limits, limit_error, limit_margin_rad
from dimos.robot.manipulators.calibration.procedure import (
    Capabilities,
    Procedure,
    ProcedureSettings,
)
from dimos.robot.manipulators.calibration.schema import (
    ArmCalibration,
    ArmCalibrationError,
    GravityCalibration,
    calibration_path,
    gravity_links_from_inertial,
    load_for_spec,
    save_calibration,
    to_inertial_overrides,
)
from dimos.robot.manipulators.calibration.spec import (
    DEFAULT_TORQUE_SAFETY_FACTOR,
    ArmCalibrationSpec,
    WorkspaceGuard,
)
from dimos.robot.manipulators.calibration.steps import calibration_steps
from dimos.robot.manipulators.makermods_metal.config import METAL

STEP_IDS = [step.id for step in calibration_steps()]
LEFT_MEASURED = [f"left_joint{i}" for i in (2, 3, 4, 5)]
# Short holds and fast ramps: a full run is about 4 simulated minutes.
FAST = {
    "poses": 8,
    "hold_s": 0.6,
    "dither_hold_s": 1.0,
    "step_hold_s": 1.0,
    "speed_deg_s": 30.0,
    "float_settle_s": 0.5,
    "float_window_s": 2.0,
    "tracking_poses": 3,
}


@dataclass
class Rig:
    procedure: Procedure
    fake: FakeTuningArm
    recording: MemoryRecording
    root: Path

    def run(
        self, steps: list[str] | None = None, answer: Callable[[str], str] | None = None
    ) -> dict[str, Any]:
        self.procedure.start(steps)
        status = drive(self.procedure, self.recording, answer)
        assert self.procedure.join(timeout=10.0)
        return dict(status)

    def statuses(self) -> dict[str, str]:
        return {step["id"]: step["status"] for step in self.procedure.status()["steps"]}

    def messages(self) -> dict[str, str]:
        return {step["id"]: step["message"] for step in self.procedure.status()["steps"]}

    def labels(self) -> list[str]:
        return [m["label"] for m in self.recording.markers.get("gravity_events", [])]


@pytest.fixture(scope="module")
def arm(left_spec: ArmCalibrationSpec) -> ArmModel:
    return ArmModel(left_spec)


@pytest.fixture(scope="module")
def truth(arm: ArmModel) -> pin.Model:
    return perturbed_model(arm, PERTURBATION)


@pytest.fixture
def make_rig(
    left_spec: ArmCalibrationSpec, arm: ArmModel, truth: pin.Model, tmp_path: Path
) -> Iterator[Callable[..., Rig]]:
    made: list[Procedure] = []

    def make(
        *,
        spec: ArmCalibrationSpec | None = None,
        model: ArmModel | None = None,
        caps: Capabilities | None = None,
        fake: dict[str, Any] | None = None,
        reports: bool = False,
        **settings: Any,
    ) -> Rig:
        spec = spec or left_spec
        model = model or arm
        root = tmp_path / "arms"
        recording = MemoryRecording(directory=tmp_path / "recording" if reports else None)
        options: dict[str, Any] = {"truth": truth} if spec is left_spec else {}
        options.update(fake or {})
        arm_fake = FakeTuningArm(
            spec, arm=model, recording=recording, calibration_root=root, **options
        )
        procedure = Procedure(
            spec,
            arm_fake,
            calibration_steps(),
            state=arm_fake,
            settings=ProcedureSettings(**{**FAST, **settings}),
            caps=caps,
            recording=recording,
            events=recording,
            clock=SimClock(arm_fake),
            arm=model,
        )
        made.append(procedure)
        return Rig(procedure, arm_fake, recording, root)

    yield make
    for procedure in made:
        procedure.close(timeout=30.0)


def fitted_error_nm(arm: ArmModel, truth: pin.Model, links: dict[str, LinkInertial]) -> float:
    """Largest gap between the corrected and the true gravity torque over random poses, N m.

    With no links, the gap the URDF alone leaves.
    """
    model = arm.model.copy()
    apply_inertial_overrides(model, links)
    rng = np.random.default_rng(1)
    worst = 0.0
    for _ in range(50):
        q = arm.home.copy()
        for row in arm.measured_rows:
            low, high = arm.spec.joint_ranges[arm.joint_names[row]]
            q[row] = rng.uniform(low, high)
        fitted = pin.computeGeneralizedGravity(model, model.createData(), q)
        true = pin.computeGeneralizedGravity(truth, truth.createData(), q)
        worst = max(worst, float(np.abs(np.asarray(fitted) - np.asarray(true)).max()))
    return worst


def test_a_full_run_on_the_bench_arm_saves_gravity_gains_and_checks(
    make_rig: Callable[..., Rig], arm: ArmModel, truth: pin.Model, left_spec: ArmCalibrationSpec
) -> None:
    rig = make_rig(reports=True)
    sent: dict[str | None, list[float]] = defaultdict(list)
    rig.fake.on_command = lambda fake: sent[fake.session_id].append(fake.now)
    plan = rig.procedure.plan()
    assert [step["id"] for step in plan] == STEP_IDS
    assert all(step["supported"] and step["instructions"] for step in plan)

    status = rig.run()

    assert status["state"] == "done", status["message"]
    assert set(rig.statuses().values()) == {"done"}
    path = calibration_path(left_spec.arm_id, rig.root)
    calibration = load_for_spec(left_spec, path)
    assert calibration.gravity is not None
    assert set(calibration.gravity.links) == set(left_spec.gravity_links)
    # Eight poses do not pin every direction down, so the gap shrinks but stays.
    urdf_error = fitted_error_nm(arm, truth, {})
    assert urdf_error > 1.0
    assert fitted_error_nm(arm, truth, to_inertial_overrides(calibration)) < 0.2 * urdf_error
    assert calibration.friction_nm is not None and set(calibration.friction_nm) == set(
        LEFT_MEASURED
    )
    # The big joints settle at twice their kp; the wrists rest past 0.1 deg
    # on their friction at every kp tried, so they keep their robot gains.
    assert calibration.gains is not None
    assert {j: (g.kp, g.kd) for j, g in calibration.gains.items()} == {
        f"left_joint{i}": (160.0, pytest.approx(math.sqrt(2))) for i in (1, 2, 3)
    }
    results = rig.procedure.results()
    wrist = results["pick_gains"]["proposals"]["left_joint4"]
    # The sweep's kd follows the square root of the kp scale: 1.5, 1.84, 2.12.
    assert wrist["kp"] is None and "rests" in wrist["rejected"]["kp=10 kd=1.5"]
    assert set(wrist["rejected"]) == {"kp=10 kd=1.5", "kp=15 kd=1.83712", "kp=20 kd=2.12132"}
    assert calibration.checks is not None and set(calibration.checks) == {"float", "tracking"}
    floats = calibration.checks["float"]["joints"]
    assert set(floats) == set(LEFT_MEASURED) and all(row["passed"] for row in floats.values())
    # Each joint floated at the two poses where it carries the most gravity.
    assert all(len(row["poses"]) == 2 for row in floats.values())
    assert floats["left_joint2"]["poses"][0]["gravity_nm"] < -5.0
    # The tracking check revisits the held-out pose, paired with the sweep's rest there.
    tracking = calibration.checks["tracking"]
    assert tracking["pose_names"] == results["gravity_fit"]["holdout_poses"] == ["pose_05"]
    assert tracking["poses"] == 1 and tracking["baseline"]["rest_samples"] > 0
    assert set(tracking["baseline"]["rest_rms_deg"]) == set(left_spec.joint_names)
    # The sweep ran clear of every limit, away from home.
    assert results["step_sweep"]["pose"] == "gravity pose pose_03"
    # Saved after the fit, then by the float check, the gains and the tracking check.
    assert len(list((path.parent / "history").glob("*.json"))) == 3
    applied = rig.fake.config_get()[left_spec.arm_id].file
    assert applied is not None and applied.path == str(path)

    fit = results["gravity_fit"]
    assert fit["saved"] == str(path) and fit["applied"] and not fit["forced"]
    reports = rig.recording.directory
    assert reports is not None and fit["report"] == str(reports / "gravity_report")
    assert (reports / "gravity_report" / "resting_torque.png").exists()
    assert "Pass:" in (reports / "step_report" / "report.md").read_text()
    assert calibration.gravity.fit["recording"] == "recording"
    for joint in LEFT_MEASURED:
        assert fit["holdout_rms_nm"][joint]["after"] < fit["holdout_rms_nm"][joint]["before"]
    assert rig.labels() == [f"pose_{i:02d}" for i in range(1, 9)] + ["end"]
    steps = rig.recording.markers["step_events"]
    assert len(steps) == 8 * 4 * 2 + 6 * 3 * 4
    assert len(rig.recording.markers["float_events"]) == 8
    # The fit was scored against the URDF, the masses the arm ran (the URDF's
    # here) and itself.
    for joint, row in fit["holdout_rms_nm"].items():
        assert row["running"] == pytest.approx(row["before"]), joint
    assert all(
        m["data"]["arm"] == left_spec.arm_id for m in rig.recording.markers["gravity_events"]
    )

    # Every session streamed a target at least 50 times a second and ended on
    # config_reset: no lease lapsed and no deadman fired while one lasted.
    # One session each for the check, the poses, the floats, the sweep and the
    # tracking; the fit and the gains are applied at home without one.
    assert len(sent) == 5 and None not in sent
    for times in sent.values():
        assert np.diff(times).max() <= 1.0 / 50.0 + 1e-6
    assert rig.fake.ended == ["config_reset"] * 5
    assert not rig.fake.session_active()
    for value in (rig.procedure.plan(), rig.procedure.status(), results):
        assert json.loads(json.dumps(value)) == value


def test_skip_and_abort_at_a_pose_stop_where_the_arm_is(make_rig: Callable[..., Rig]) -> None:
    rig = make_rig()
    prompts: list[str] = []

    def answer(prompt: str) -> str:
        prompts.append(prompt)
        # An answer meant for the prompt before is not taken for this one.
        assert not rig.procedure.confirm(rig.procedure.status()["prompt_id"] - 1)
        with pytest.raises(TypeError, match="needs the prompt_id"):
            rig.procedure.confirm(None)  # type: ignore[arg-type]
        if prompt.startswith("Pose 2/"):
            return "skip"
        if prompt.startswith("Pose 3/"):
            return "abort"
        return "confirm"

    status = rig.run(["check", "gravity_poses", "gravity_fit"], answer)

    assert status["state"] == "aborted"
    assert status["message"].endswith("the arm holds where it is; it was not moved home")
    assert rig.statuses() == {
        "check": "done",
        "gravity_poses": "aborted",
        "gravity_fit": "pending",
        **{step: "not_selected" for step in STEP_IDS[3:]},
    }
    assert [p.split(" ")[0:2] for p in prompts[2:5]] == [
        ["Pose", "1/8"],
        ["Pose", "2/8"],
        ["Pose", "3/8"],
    ]
    # Pose 2 was skipped and pose 3 never reached; the sweep still closes.
    assert rig.labels() == ["pose_01", "end"]
    # A pose marker carries the targets it was marked at; its data, the pose's.
    marker = rig.recording.markers["gravity_events"][0]
    assert np.allclose(list(marker["targets_deg"].values()), np.degrees(rig.fake.arm.home))
    pose = np.radians(marker["data"]["targets_deg"])
    # Held at pose 1, give or take the uncalibrated sag, not moved home.
    assert np.degrees(np.abs(rig.fake.q - pose)).max() < 3.0
    assert np.degrees(np.abs(rig.fake.q - rig.fake.arm.home)).max() > 30.0
    assert not rig.fake.session_active()
    assert "gravity_poses" not in rig.procedure.results()
    assert not rig.procedure.confirm(0) and not rig.procedure.skip(0)
    assert not rig.procedure.abort()


def test_a_stalled_joint_fails_the_step_and_the_arm_holds(make_rig: Callable[..., Rig]) -> None:
    rig = make_rig(confirm_each=False)
    rig.fake.blocked = {"left_joint3"}

    status = rig.run(["check", "gravity_poses"])

    assert status["state"] == "failed"
    message = rig.messages()["gravity_poses"]
    assert rig.statuses()["gravity_poses"] == "failed"
    # Caught while ramping: the elbow fell 5 deg behind more than a PD follower lags.
    assert "from its target (left_joint3 -5." in message
    assert message.endswith("the arm holds where it is")
    assert not rig.fake.session_active()
    # Nothing is sent any more: ending the session held the arm where it
    # stopped, and the deadman holds it there again, give or take its sag
    # (each hold gives up the PD's steady error once more), far from home.
    commands, held = rig.fake.commands, rig.fake.q.copy()
    rig.fake.advance(1.0)
    assert rig.fake.commands == commands and rig.fake.deadman_fired
    assert np.degrees(np.abs(rig.fake.q - held)).max() < 3.0
    assert np.degrees(np.abs(rig.fake.q - rig.fake.arm.home)).max() > 4.0


def test_a_lost_session_fails_the_step_without_raising(make_rig: Callable[..., Rig]) -> None:
    rig = make_rig(confirm_each=False)
    rig.fake.fail_after_commands = 400

    status = rig.run(["check", "gravity_poses", "gravity_fit"])

    assert status["state"] == "failed"
    assert rig.statuses()["gravity_poses"] == "failed"
    assert rig.statuses()["gravity_fit"] == "pending"
    assert rig.messages()["gravity_poses"] == (
        "config_command failed: feedback failed: the bus went silent"
    )
    assert status["message"].startswith("Gravity poses failed: config_command failed")
    assert (
        rig.fake.ended[-1]
        == "config_command raised RuntimeError: feedback failed: the bus went silent"
    )
    assert not rig.fake.session_active()
    assert rig.labels()[-1] == "end"


def test_steps_the_robot_cannot_run_are_listed_unsupported_and_skipped(
    make_rig: Callable[..., Rig],
) -> None:
    # The Metal does not compensate gravity, so none of its joints may float.
    [spec] = robot_specs_or_skip(
        METAL, "MakerMods Metal", workspace=WorkspaceGuard.for_bench(floor_z_m=0.0, min_x_m=0.1)
    )
    # What the connection cannot do it says in config_keys; the fake's lists no float.
    caps = Capabilities(recording_unavailable="nothing is recorded")
    rig = make_rig(spec=spec, model=ArmModel(spec), caps=caps, fake={"gravity_compensation": False})

    reasons = {step["id"]: step["reason"] for step in rig.procedure.plan() if not step["supported"]}

    float_reason = "the robot does not compensate gravity, so a floating joint would fall"
    assert reasons == {
        "gravity_poses": "nothing is recorded",
        "gravity_fit": "nothing is recorded",
        "float_check": float_reason,
        "step_sweep": "nothing is recorded",
        "pick_gains": "nothing is recorded",
    }
    status = rig.run(["check", "float_check", "summary"])
    assert status["state"] == "done", status["message"]
    assert rig.statuses()["float_check"] == "unsupported"
    assert rig.messages()["float_check"] == float_reason
    assert rig.statuses()["check"] == rig.statuses()["summary"] == "done"
    assert "float_events" not in rig.recording.markers


def test_steps_the_connection_cannot_save_or_serve_are_skipped_with_why(
    make_rig: Callable[..., Rig], left_spec: ArmCalibrationSpec
) -> None:
    # A connection that saves nothing: no fit, no picked gains, no checks recorded.
    rig = make_rig(fake={"persist": []}, float_window_s=1.0, float_settle_s=0.0)
    path = save_calibration(rig.root / left_spec.arm_id, ArmCalibration.for_spec(left_spec))

    reasons = {step["id"]: step["reason"] for step in rig.procedure.plan() if not step["supported"]}

    assert reasons == {
        "gravity_fit": "the connection cannot save links",
        "pick_gains": "the connection cannot save gains",
    }
    status = rig.run(["float_check"])
    assert status["state"] == "done", status["message"]
    assert rig.procedure.results()["float_check"]["saved"] is None
    assert load_for_spec(left_spec, path).checks is None

    # One deployed without config sessions: only the check and the summary may run,
    # and the check refuses it before anything moves.
    closed = make_rig(fake={"sessions": False})
    reasons = {
        step["id"]: step["reason"] for step in closed.procedure.plan() if not step["supported"]
    }
    assert set(reasons) == set(STEP_IDS) - {"check", "summary"}
    assert all("does not serve config sessions" in reason for reason in reasons.values())
    status = closed.run(["check"])
    assert status["state"] == "failed"
    assert closed.messages()["check"].startswith(
        "the connection does not serve config sessions: start the robot's calibration blueprint"
    )
    assert closed.fake.commands == 0


@pytest.mark.parametrize("steps", [None, ["gravity_poses", "summary"]])
def test_a_connection_that_does_not_say_what_it_serves_fails_the_run(
    make_rig: Callable[..., Rig], steps: list[str] | None
) -> None:
    rig = make_rig()

    def unanswered() -> Any:
        raise TimeoutError("RPC call to 'config_keys' timed out after 120.0 seconds")

    rig.fake.config_keys = unanswered  # type: ignore[method-assign]

    status = rig.run(steps)

    assert status["state"] == "failed"
    assert status["message"] == (
        "the connection did not say what it serves: RPC call to 'config_keys' timed out "
        "after 120.0 seconds; no step ran"
    )
    assert set(rig.statuses().values()) <= {"pending", "not_selected"}
    assert rig.fake.commands == 0


def test_a_workspace_without_a_guard_fails_the_check_before_any_motion(
    make_rig: Callable[..., Rig],
) -> None:
    unguarded = robot_specs_or_skip(METAL, "MakerMods Metal")[0]
    rig = make_rig(spec=unguarded, model=ArmModel(unguarded), fake={"gravity_compensation": False})

    status = rig.run(["check"])

    assert status["state"] == "failed"
    assert rig.messages()["check"] == (
        "no workspace guard defined for metal: pass floor height and front limit "
        "(the calibration module's floor_z_m and min_x_m)"
    )
    assert rig.fake.commands == 0


def test_the_fit_undoes_the_calibration_the_arm_ran_and_forces_only_on_a_second_yes(
    make_rig: Callable[..., Rig],
    arm: ArmModel,
    truth: pin.Model,
    left_spec: ArmCalibrationSpec,
    tmp_path: Path,
) -> None:
    # The arm starts on an old file whose forearm is 3 kg too heavy.
    forearm = arm.model.inertias[body_joint_id(arm.model, "left_link3")]
    heavy = {
        "left_link3": LinkInertial(
            mass_kg=float(forearm.mass) + 3.0, com_m=tuple(float(v) for v in forearm.lever)
        )
    }
    old = tmp_path / "old_calibration.json"
    save_calibration(
        old,
        ArmCalibration.for_spec(
            left_spec,
            gravity=GravityCalibration(links=gravity_links_from_inertial(left_spec, heavy)),
        ),
    )
    rig = make_rig(confirm_each=False, fake={"applied": old})
    prompts: list[str] = []

    status = rig.run(
        ["check", "gravity_poses", "gravity_fit"], lambda p: prompts.append(p) or "confirm"
    )

    assert status["state"] == "done", status["message"]
    calibration = load_for_spec(left_spec, calibration_path(left_spec.arm_id, rig.root))
    urdf_error = fitted_error_nm(arm, truth, {})
    assert fitted_error_nm(arm, truth, to_inertial_overrides(calibration)) < 0.2 * urdf_error
    fit = rig.procedure.results()["gravity_fit"]
    assert fit["applied"] and fit["forced"]
    assert any(p.startswith("The connection refused: applying") for p in prompts)
    assert max(abs(v) for v in fit["gravity_step_nm"].values()) > 2.0
    assert rig.procedure.results()["gravity_poses"]["calibration"] == str(old)


def test_declining_the_forced_apply_leaves_the_old_calibration_running(
    make_rig: Callable[..., Rig], arm: ArmModel, left_spec: ArmCalibrationSpec, tmp_path: Path
) -> None:
    forearm = arm.model.inertias[body_joint_id(arm.model, "left_link3")]
    heavy = {
        "left_link3": LinkInertial(
            mass_kg=float(forearm.mass) + 3.0, com_m=tuple(float(v) for v in forearm.lever)
        )
    }
    old = tmp_path / "old_calibration.json"
    save_calibration(
        old,
        ArmCalibration.for_spec(
            left_spec,
            gravity=GravityCalibration(links=gravity_links_from_inertial(left_spec, heavy)),
        ),
    )
    rig = make_rig(confirm_each=False, fake={"applied": old})

    status = rig.run(
        ["check", "gravity_poses", "gravity_fit"],
        lambda p: "skip" if p.startswith("The connection refused") else "confirm",
    )

    assert status["state"] == "done", status["message"]
    fit = rig.procedure.results()["gravity_fit"]
    assert fit["saved"] is not None and not fit["applied"]
    assert "pass force=True" in fit["refused"]
    applied = rig.fake.config_get()[left_spec.arm_id].file
    assert applied is not None and applied.path == str(old)


def test_a_check_is_not_recorded_against_a_fit_the_arm_never_ran(
    make_rig: Callable[..., Rig], arm: ArmModel, left_spec: ArmCalibrationSpec, tmp_path: Path
) -> None:
    # The fit is refused at home and saved without applying it: the arm still
    # runs the old file when it floats, so the float check is not the new file's.
    forearm = arm.model.inertias[body_joint_id(arm.model, "left_link3")]
    heavy = {
        "left_link3": LinkInertial(
            mass_kg=float(forearm.mass) + 3.0, com_m=tuple(float(v) for v in forearm.lever)
        )
    }
    old = tmp_path / "old_calibration.json"
    save_calibration(
        old,
        ArmCalibration.for_spec(
            left_spec,
            gravity=GravityCalibration(links=gravity_links_from_inertial(left_spec, heavy)),
        ),
    )
    rig = make_rig(
        confirm_each=False, fake={"applied": old}, float_window_s=1.0, float_settle_s=0.0
    )

    status = rig.run(
        ["check", "gravity_poses", "gravity_fit", "float_check"],
        lambda p: "skip" if p.startswith("The connection refused") else "confirm",
    )

    assert status["state"] == "done", status["message"]
    assert rig.statuses()["float_check"] == "done"
    path = calibration_path(left_spec.arm_id, rig.root)
    floats = rig.procedure.results()["float_check"]
    assert floats["saved"] is None
    assert floats["not_saved"].startswith(f"the arm does not run {path} as it is on disk")
    assert "; not recorded: the arm does not run" in rig.messages()["float_check"]
    assert load_for_spec(left_spec, path).checks is None


def test_a_floating_joint_that_drifts_is_held_again_and_fails(
    make_rig: Callable[..., Rig],
) -> None:
    # Without the fit the gripper is 0.25 kg heavier than compensated: joints sag.
    rig = make_rig(float_window_s=5.0)

    status = rig.run(["check", "float_check"])

    assert status["state"] == "done", status["message"]
    joints = rig.procedure.results()["float_check"]["joints"]
    shoulder = joints["left_joint2"]
    assert not shoulder["passed"]
    assert "drifted" in shoulder["reason"] or shoulder["fingertip_drift_mm"] > 2.0
    assert "left_joint2" not in rig.fake.config_get()[rig.fake.spec.arm_id].floating
    assert rig.procedure.results()["float_check"]["saved"] is None


@pytest.mark.parametrize(
    "float_kd,caught",
    [
        (0.5, r"drifted at [+-]\d+ deg/s while floating \(limit 20 deg/s\): it was falling"),
        (5.0, r"drifted [+-]5\.\d deg while floating \(limit 5 deg\)"),
    ],
)
def test_a_falling_joint_is_held_again_on_speed_or_on_drift(
    make_rig: Callable[..., Rig], float_kd: float, caught: str
) -> None:
    # The shoulder carries about 1 N m more than compensated at its float pose:
    # lightly damped it falls fast, heavily damped it creeps past 5 deg.
    rig = make_rig(float_window_s=5.0, float_settle_s=0.0, float_kd=float_kd)

    status = rig.run(["float_check"])

    assert status["state"] == "done", status["message"]
    shoulder = rig.procedure.results()["float_check"]["joints"]["left_joint2"]
    first = shoulder["poses"][0]
    assert not first["passed"] and re.search(caught, first["reason"]), first["reason"]
    assert "left_joint2" not in rig.fake.config_get()[rig.fake.spec.arm_id].floating


def test_the_fake_keeps_the_config_session_rules(
    left_spec: ArmCalibrationSpec, arm: ArmModel, tmp_path: Path
) -> None:
    fake = FakeTuningArm(left_spec, arm=arm, calibration_root=tmp_path)
    arm_id = left_spec.arm_id
    home = dict(zip(left_spec.joint_names, (float(v) for v in arm.home), strict=True))

    def begin() -> str:
        session = fake.config_set(ConfigPatch(arm_id=arm_id), lease_s=1.0).session_id
        assert session is not None
        return session

    with pytest.raises(RuntimeError, match="needs the session_id"):
        fake.config_command(home, "")
    with pytest.raises(RuntimeError, match="no config session"):
        fake.config_command(home, "fake-0")
    begin()
    with pytest.raises(RuntimeError, match="a config session is live"):
        begin()
    assert not fake.session_active()

    session = begin()
    assert not fake.position_command({"left_joint1": 0.1})
    assert fake.dropped_position_commands == 1
    # Reading carries no session id, so it renews nothing.
    fake.advance(0.75)
    fake.config_get()
    fake.config_keys()
    fake.advance(0.75)
    assert not fake.session_active() and fake.ended[-1].startswith("no config call within")
    with pytest.raises(RuntimeError, match=r"the config session ended \(no config call"):
        fake.config_command(home, session)

    session = begin()
    fake.config_command(home, session)
    fake.advance(0.2)
    assert not fake.session_active() and fake.ended[-1] == "the command deadman fired"

    session = begin()
    floated = fake.config_set(
        ConfigPatch(
            arm_id=arm_id,
            gains={
                "left_joint2": JointGains(kp=0.0, kd=0.5),
                "left_joint3": JointGains(kp=40.0, kd=9.0),
            },
        ),
        session_id=session,
    )
    assert floated.clamped == ["gains.left_joint3.kd"]
    assert fake.config_get()[arm_id].floating == {"left_joint2": 0.5}
    with pytest.raises(ValueError):
        fake.config_command({"left_joint1": 10.0}, session)
    assert not fake.session_active()
    state = fake.config_get()[arm_id]
    assert state.floating == {} and state.gains["left_joint3"] == JointGains(kp=80.0, kd=1.0)
    assert fake.position_command({"left_joint1": 0.0})
    assert math.isclose(fake.target[0], 0.0)


def test_the_fake_checks_and_saves_as_the_connection_does(
    left_spec: ArmCalibrationSpec, arm: ArmModel, tmp_path: Path
) -> None:
    fake = FakeTuningArm(left_spec, arm=arm, calibration_root=tmp_path)
    arm_id = left_spec.arm_id
    path = calibration_path(arm_id, tmp_path)
    session = fake.config_set(ConfigPatch(arm_id=arm_id)).session_id
    # A torque offset past a fifth of the motor's rated torque is clamped, and says so.
    offset = fake.config_set(
        ConfigPatch(arm_id=arm_id, torque_offset_nm={"left_joint2": 100.0}), session_id=session
    )
    rated = left_spec.max_torque_nm["left_joint2"] / DEFAULT_TORQUE_SAFETY_FACTOR
    assert offset.applied.torque_offset_nm == {"left_joint2": pytest.approx(0.2 * rated)}
    assert offset.clamped == ["torque_offset_nm.left_joint2"]
    # A patch is validated as the connection validates it, and a bad one ends the session.
    with pytest.raises(ValidationError):
        fake.config_set(
            {"arm_id": arm_id, "gains": {"left_joint2": {"kp": "80", "kd": 1.0}}},  # type: ignore[arg-type]
            session_id=session,
        )
    assert not fake.session_active()

    # A save clamps its gains and keeps the connection's file rules.
    saved = fake.config_set(
        ConfigPatch(arm_id=arm_id, gains={"left_joint2": JointGains(kp=900.0, kd=9.0)}),
        scope="persist",
    )
    assert saved.clamped == ["gains.left_joint2.kp", "gains.left_joint2.kd"]
    assert load_for_spec(left_spec, path).gains == {"left_joint2": JointGains(kp=500.0, kd=5.0)}
    with pytest.raises(ArmCalibrationError, match="gives gains to joints of no arm"):
        fake.config_set(
            ConfigPatch(arm_id=arm_id, gains={"right_joint2": JointGains(kp=10.0, kd=1.0)}),
            scope="persist",
        )
    stranger = {"right_link3": LinkGravity(mass_kg=1.0, com_m=(0.0, 0.0, 0.0))}
    with pytest.raises(ArmCalibrationError, match=r"does not move: \['right_link3'\]"):
        fake.config_set(ConfigPatch(arm_id=arm_id, links=stranger), scope="persist")
    # A stored file made for another model is saved afresh, not merged.
    stored = load_for_spec(left_spec, path)
    other = stored.robot_model.model_copy(update={"sha256": "0" * 64})
    save_calibration(path, stored.model_copy(update={"robot_model": other}))
    fake.config_set(ConfigPatch(arm_id=arm_id, notes="afresh"), scope="persist")
    fresh = load_for_spec(left_spec, path)
    assert fresh.gains is None and fresh.notes == "afresh"

    # Without config sessions it refuses every call that changes the arm.
    closed = FakeTuningArm(left_spec, arm=arm, calibration_root=tmp_path, sessions=False)
    for call in (
        lambda: closed.config_set(ConfigPatch(arm_id=arm_id)),
        closed.config_reset,
        lambda: closed.config_command({}, "fake-1"),
    ):
        with pytest.raises(RuntimeError, match="config sessions are off"):
            call()


def _far_from_home(rig: Rig) -> float:
    return float(np.degrees(np.abs(rig.fake.q - rig.fake.arm.home)).max())


def test_the_float_check_floats_each_joint_where_it_carries_the_most_gravity(
    make_rig: Callable[..., Rig],
) -> None:
    rig = make_rig(float_window_s=1.0, float_settle_s=0.0)
    prompts: list[str] = []

    def answer(prompt: str) -> str:
        prompts.append(prompt)
        return "confirm"

    status = rig.run(["float_check"], answer)

    assert status["state"] == "done", status["message"]
    result = rig.procedure.results()["float_check"]
    arm = rig.procedure.arm
    poses = generate_poses(arm, count=8).identification
    assert set(result["joints"]) == set(LEFT_MEASURED)
    for joint, row in result["joints"].items():
        i = arm.joint_names.index(joint)
        largest = sorted(abs(float(arm.gravity(np.asarray(p.joints_rad))[i])) for p in poses)
        chosen = [abs(pose["gravity_nm"]) for pose in row["poses"]]
        # The two poses where the joint carries the most, each a real load.
        assert chosen == pytest.approx(largest[::-1][:2], abs=1e-3), joint
        assert min(chosen) >= 0.2
    # One prompt for the plan, then one before every move.
    assert len(prompts) == 1 + 2 * len(LEFT_MEASURED)
    assert prompts[1].startswith("Float left_joint2 at pose_01 [")
    assert "where it carries -9.87 Nm" in prompts[1]
    # Without the fit the arm is heavier than compensated: the drift is caught.
    assert not result["joints"]["left_joint2"]["passed"]
    assert len(rig.recording.markers["float_events"]) == 2 * len(LEFT_MEASURED)
    assert not rig.fake.session_active()


def test_a_joint_with_no_gravity_load_is_not_floated(make_rig: Callable[..., Rig]) -> None:
    # At the 60/60 home pose the last wrist joint carries nothing.
    home_deg = tuple(float(v) for v in np.degrees(make_rig().procedure.arm.home))
    rig = make_rig(float_window_s=1.0, float_settle_s=0.0, float_pose_deg=home_deg)

    status = rig.run(["float_check"])

    assert status["state"] == "done", status["message"]
    result = rig.procedure.results()["float_check"]
    assert result["pose"] == "the configured float pose"
    wrist = result["joints"]["left_joint5"]
    assert wrist["skipped"] and wrist["reason"].startswith("no gravity load to test")
    assert all(
        row["poses"][0]["pose"] == "the configured float pose"
        for joint, row in result["joints"].items()
        if joint != "left_joint5"
    )
    assert len(rig.recording.markers["float_events"]) == len(LEFT_MEASURED) - 1

    short = make_rig(float_pose_deg=(0.0, 1.0))
    status = short.run(["float_check"])
    assert status["state"] == "failed" and "needs 6 angles" in status["message"]


def test_go_home_confirms_first_and_works_after_an_abort(make_rig: Callable[..., Rig]) -> None:
    rig = make_rig()
    status = rig.run(
        ["check", "gravity_poses"], lambda p: "abort" if p.startswith("Pose 2/") else "confirm"
    )
    assert status["state"] == "aborted" and _far_from_home(rig) > 30.0
    prompts: list[str] = []

    def answer(prompt: str) -> str:
        prompts.append(prompt)
        return "confirm"

    rig.procedure.go_home()
    status = drive(rig.procedure, rig.recording, answer)

    assert rig.procedure.join(timeout=10.0)
    assert status["state"] == "done" and status["message"] == "the arm is at home"
    assert len(prompts) == 1 and prompts[0].startswith("The arm will ramp to home at 30 deg/s")
    assert _far_from_home(rig) < 3.0
    assert not rig.fake.session_active()


def test_go_home_can_be_skipped_aborted_and_not_started_while_running(
    make_rig: Callable[..., Rig],
) -> None:
    rig = make_rig()
    rig.procedure.go_home()
    status = drive(rig.procedure, rig.recording, lambda p: "skip")
    assert status["state"] == "done" and "not moved home" in status["message"]
    rig.procedure.join(timeout=10.0)

    rig.procedure.go_home()
    status = drive(rig.procedure, rig.recording, lambda p: "abort")
    assert status["state"] == "aborted" and "going home" in status["message"]
    rig.procedure.join(timeout=10.0)

    rig.procedure.start(["check"])
    with pytest.raises(RuntimeError, match="already"):
        rig.procedure.go_home()
    drive(rig.procedure, rig.recording)


def _abort_at(prompt_start: str) -> Callable[[str], str]:
    return lambda prompt: "abort" if prompt.startswith(prompt_start) else "confirm"


def test_shutdown_parks_an_arm_left_at_a_pose_and_ends_the_session(
    make_rig: Callable[..., Rig],
) -> None:
    rig = make_rig()
    # Aborted while waiting for pose 4: the arm holds at pose 3, 75 deg from home.
    status = rig.run(["check", "gravity_poses"], _abort_at("Pose 4/"))
    assert status["state"] == "aborted" and _far_from_home(rig) > 30.0
    sessions = len(rig.fake.ended)

    outcome = rig.procedure.shutdown()

    assert outcome.startswith("parked at home in ")
    assert outcome.endswith("(the calibration module is stopping)")
    assert _far_from_home(rig) < 3.0
    status = rig.procedure.status()
    assert status["park"] == outcome and status["state"] == "aborted"
    # One more session, the park's, ended by the park itself.
    assert rig.fake.ended[sessions:] == ["config_reset"] and not rig.fake.session_active()
    with pytest.raises(RuntimeError, match="shutting down"):
        rig.procedure.start(["check"])


def test_shutdown_during_a_run_parks_once_the_run_lets_go(make_rig: Callable[..., Rig]) -> None:
    rig = make_rig()
    outcomes: list[str] = []

    def answer(prompt: str) -> str:
        if prompt.startswith("Pose 4/"):
            # The module stops while the arm waits at pose 3 inside the sweep's session.
            outcomes.append(rig.procedure.shutdown())
            return "ignore"
        return "confirm"

    status = rig.run(["check", "gravity_poses"], answer)

    assert outcomes and outcomes[0].startswith("parked at home")
    assert status["state"] == "aborted"
    assert status["message"] == f"aborted during gravity_poses: {outcomes[0]}"
    assert _far_from_home(rig) < 3.0 and not rig.fake.session_active()


def test_a_park_over_its_budget_or_without_a_safe_path_holds_and_says_why(
    make_rig: Callable[..., Rig], mocker: Any
) -> None:
    # Pose 1 is 100 deg from home: 9.4 s at 20 deg/s, over the 8 s budget.
    rig = make_rig()
    rig.run(["check", "gravity_poses"], _abort_at("Pose 2/"))
    held, commands = rig.fake.q.copy(), rig.fake.commands
    error = mocker.spy(procedure_module.logger, "error")

    outcome = rig.procedure.shutdown()

    assert outcome.startswith("held where it is, not parked (the calibration module is stopping)")
    assert re.search(r"the way home takes 9\.\d s at 20 deg/s, more than the 8\.0 s", outcome)
    assert any("Support the arm" in str(call.args[0]) for call in error.call_args_list)
    # Held, not moved: only the targets that hold it where it is were sent.
    assert np.degrees(np.abs(rig.fake.q - held)).max() < 1.0
    assert rig.fake.commands - commands <= 2

    unsafe = make_rig()
    unsafe.run(["check", "gravity_poses"], _abort_at("Pose 4/"))
    mocker.patch.object(arm_session_module, "plan_path", return_value=None)
    outcome = unsafe.procedure.shutdown()
    assert "no ramp home keeps the workspace clearance" in outcome
    assert _far_from_home(unsafe) > 30.0


def test_an_unanswered_prompt_aborts_and_parks_the_arm(make_rig: Callable[..., Rig]) -> None:
    rig = make_rig(prompt_timeout_s=5.0)
    deadlines: list[float | None] = []

    def answer(prompt: str) -> str:
        deadlines.append(rig.procedure.status()["prompt_deadline"])
        return "ignore" if prompt.startswith("Pose 4/") else "confirm"

    status = rig.run(["check", "gravity_poses"], answer)

    assert status["state"] == "aborted"
    assert status["message"].startswith(
        "aborted during gravity_poses: parked at home in "
    ) and status["message"].endswith("(the prompt went unanswered for 5 s)")
    assert status["park"] is not None and status["park"].startswith("parked at home")
    assert all(deadline is not None for deadline in deadlines)
    assert _far_from_home(rig) < 3.0 and not rig.fake.session_active()
    # Not a shutdown: the operator can run again.
    assert rig.run(["summary"])["state"] == "done"


def test_a_stale_reading_fails_the_tick_and_holds(make_rig: Callable[..., Rig]) -> None:
    rig = make_rig(confirm_each=False)
    read = rig.fake.latest

    def stale() -> JointState:
        msg = read()
        if rig.fake.commands >= 100:
            msg.ts -= 0.3
        return msg

    rig.fake.latest = stale  # type: ignore[method-assign]

    status = rig.run(["check", "gravity_poses"])

    assert status["state"] == "failed"
    message = rig.messages()["gravity_poses"]
    assert message.startswith("stopped: the latest reading is 0.30 s old (limit 0.25 s)")
    # The stale reading stopped the tick before it sent anything.
    assert rig.fake.commands == 100 and not rig.fake.session_active()


def test_a_stale_reading_fails_the_check_before_any_motion(make_rig: Callable[..., Rig]) -> None:
    # The connection stopped publishing: its last joint state is a second old.
    rig = make_rig()
    read = rig.fake.latest

    def stale() -> JointState:
        msg = read()
        msg.ts -= 1.0
        return msg

    rig.fake.latest = stale  # type: ignore[method-assign]

    status = rig.run(["check"])

    assert status["state"] == "failed"
    assert rig.messages()["check"] == (
        "the latest reading is 1.00 s old (limit 0.25 s), so the connection is not "
        "publishing the arm's joint states"
    )
    assert rig.fake.commands == 0 and not rig.fake.session_active()


def test_the_gain_sweep_runs_clear_of_the_limits_home_sits_on(
    make_rig: Callable[..., Rig], b601_spec: ArmCalibrationSpec
) -> None:
    # B601 home has joint2 1 deg and joint3 0 deg from a limit: +1 deg steps
    # there would be refused.
    model = ArmModel(b601_spec)
    lower, upper = joint_limits(b601_spec)
    assert limit_error(b601_spec.joint_names, model.home, lower, upper, math.radians(5.0))
    rig = make_rig(spec=b601_spec, model=model, confirm_each=False)
    with rig.procedure.open_session() as session:
        with pytest.raises(StepFailedError, match="would pass a joint limit"):
            session.step_joint("joint3", 1.0, 0.5)

    status = rig.run(["check", "step_sweep"])

    assert status["state"] == "done", status["message"]
    sweep = rig.procedure.results()["step_sweep"]
    assert set(sweep["joints"]) == set(b601_spec.joint_names)
    assert sweep["pose"].startswith("gravity pose ")
    pose = np.radians(sweep["pose_deg"])
    assert math.degrees(limit_margin_rad(pose, lower, upper)) >= 3.0 + 2.0
    # Back home after the sweep, every step at kp x 1, 1.5, 2 with kd x sqrt.
    assert _far_from_home(rig) < 3.0
    kp0, kd0 = b601_spec.gains["joint4"]
    assert sweep["joints"]["joint4"] == [
        [pytest.approx(kp0 * s), pytest.approx(min(kd0 * math.sqrt(s), 5.0))]
        for s in (1.0, 1.5, 2.0)
    ]


def _heavy_forearm(arm: ArmModel, spec: ArmCalibrationSpec, extra_kg: float) -> ArmCalibration:
    forearm = arm.model.inertias[body_joint_id(arm.model, "left_link3")]
    heavy = {
        "left_link3": LinkInertial(
            mass_kg=float(forearm.mass) + extra_kg, com_m=tuple(float(v) for v in forearm.lever)
        )
    }
    return ArmCalibration.for_spec(
        spec, gravity=GravityCalibration(links=gravity_links_from_inertial(spec, heavy))
    )


def test_the_fit_refuses_when_the_file_on_disk_is_not_what_the_arm_runs(
    make_rig: Callable[..., Rig], arm: ArmModel, left_spec: ArmCalibrationSpec, tmp_path: Path
) -> None:
    # The arm starts on its saved file C0, with a forearm 3 kg too heavy.
    path = calibration_path(left_spec.arm_id, tmp_path / "arms")
    save_calibration(path, _heavy_forearm(arm, left_spec, 3.0))
    rig = make_rig(confirm_each=False, fake={"applied": path})
    c0 = rig.fake.config_get()[left_spec.arm_id].file
    assert c0 is not None and c0.sha256

    # Run 1 fits and saves C1, but the forced apply is declined: the arm keeps C0.
    status = rig.run(
        ["check", "gravity_poses", "gravity_fit"],
        lambda p: "skip" if p.startswith("The connection refused") else "confirm",
    )
    assert status["state"] == "done", status["message"]
    assert rig.procedure.results()["gravity_fit"]["applied"] is False
    c1 = load_for_spec(left_spec, path)
    assert rig.fake.config_get()[left_spec.arm_id].file == c0

    # Run 2 records with C0 again; fitting against C1 on disk would be wrong.
    status = rig.run(["gravity_poses", "gravity_fit"])

    assert status["state"] == "done", status["message"]
    assert rig.statuses()["gravity_fit"] == "skipped"
    message = rig.messages()["gravity_fit"]
    assert message.startswith(f"not fitted: the arm runs the calibration created {c0.created}")
    assert f"the file on disk is the one created {c1.created.isoformat()}" in message
    assert message.endswith("restart the robot or apply the file first")
    assert "gravity_fit" not in rig.procedure.results()
    assert load_for_spec(left_spec, path) == c1


def test_a_fit_worse_than_the_running_file_is_not_saved(
    make_rig: Callable[..., Rig], arm: ArmModel, truth: pin.Model, left_spec: ArmCalibrationSpec
) -> None:
    # The arm already runs its true masses; a fit from eight poses cannot beat them.
    exact = {
        link: LinkInertial(
            mass_kg=float(truth.inertias[body_joint_id(truth, link)].mass),
            com_m=tuple(float(v) for v in truth.inertias[body_joint_id(truth, link)].lever),
        )
        for link in left_spec.gravity_links
    }
    root_path = calibration_path(left_spec.arm_id, make_rig().root)
    save_calibration(
        root_path,
        ArmCalibration.for_spec(
            left_spec,
            gravity=GravityCalibration(links=gravity_links_from_inertial(left_spec, exact)),
        ),
    )
    saved = load_for_spec(left_spec, root_path)
    rig = make_rig(confirm_each=False, fake={"applied": root_path})

    status = rig.run(["check", "gravity_poses", "gravity_fit"])

    assert status["state"] == "done", status["message"]
    fit = rig.procedure.results()["gravity_fit"]
    assert fit["saved"] is None and fit["refusals"]
    assert all("is worse than the running calibration's" in r for r in fit["refusals"])
    for joint, row in fit["holdout_rms_nm"].items():
        # URDF -> running -> new: the fit beats the URDF, not what the arm runs.
        assert row["running"] < row["after"] < row["before"], joint
    assert load_for_spec(left_spec, root_path) == saved


def test_a_recorder_that_is_not_writing_fails_the_check_before_any_motion(
    make_rig: Callable[..., Rig],
) -> None:
    rig = make_rig()
    rig.recording.drop_probes = True

    status = rig.run(["check"])

    assert status["state"] == "failed"
    message = rig.messages()["check"]
    assert "was not recorded" in message and message.endswith("nothing has moved")
    assert rig.fake.commands == 0

    writing = make_rig()
    assert writing.run(["check"])["state"] == "done"
    assert writing.procedure.results()["check"]["recording"] == "writing"
    assert len(writing.recording.probes) == 1


def test_a_save_inside_a_session_carries_its_id(
    make_rig: Callable[..., Rig], left_spec: ArmCalibrationSpec
) -> None:
    rig = make_rig()
    patch = ConfigPatch(arm_id=left_spec.arm_id, notes="bench")
    with rig.procedure.open_session():
        with pytest.raises(RuntimeError, match="a config session is live"):
            rig.fake.config_set(patch, scope="persist")
    # That raised, which ended the session; a save from inside one names it.
    with rig.procedure.open_session() as session:
        saved = rig.procedure.persist(patch)
        assert saved.session_id == session.session_id and saved.path is not None
        session.keepalive()
    assert load_for_spec(left_spec, Path(saved.path)).notes == "bench"
    assert rig.fake.ended[-2].startswith("config_set raised RuntimeError: a config session is live")
