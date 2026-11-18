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
from typing import Any

import numpy as np
import pinocchio as pin
import pytest

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
from dimos.robot.manipulators.calibration.gravity_poses import (
    ArmModel,
    body_joint_id,
    generate_poses,
)
from dimos.robot.manipulators.calibration.procedure import (
    Capabilities,
    Procedure,
    ProcedureSettings,
)
from dimos.robot.manipulators.calibration.schema import (
    ArmCalibration,
    GravityCalibration,
    calibration_path,
    gravity_links_from_inertial,
    load_for_spec,
    save_calibration,
    to_inertial_overrides,
)
from dimos.robot.manipulators.calibration.spec import ArmCalibrationSpec, WorkspaceGuard
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
            settings=ProcedureSettings(**{**FAST, **settings}, calibration_root=root),
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
        f"left_joint{i}": (160.0, 5.0) for i in (1, 2, 3)
    }
    results = rig.procedure.results()
    wrist = results["pick_gains"]["proposals"]["left_joint4"]
    assert wrist["kp"] is None and "rests" in wrist["rejected"]["kp=20 kd=1.5"]
    assert calibration.checks is not None and set(calibration.checks) == {"float", "tracking"}
    floats = calibration.checks["float"]["joints"]
    assert set(floats) == set(LEFT_MEASURED) and all(row["passed"] for row in floats.values())
    tracking = calibration.checks["tracking"]
    assert tracking["poses"] == 3 and tracking["baseline"]["rest_samples"] > 0
    # Saved after the fit, then by the float check, the gains and the tracking check.
    assert len(list((path.parent / "history").glob("*.json"))) == 3
    applied = rig.fake.status().calibration[left_spec.arm_id]
    assert applied is not None and applied["path"] == str(path)

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
    assert len(rig.recording.markers["float_events"]) == 4
    assert all(
        m["data"]["arm"] == left_spec.arm_id for m in rig.recording.markers["gravity_events"]
    )

    # Every session streamed a target at least 50 times a second and ended on
    # tuning_end: no lease lapsed and no deadman fired while one lasted.
    # One session each for the check, the poses, the floats, the sweep and the
    # tracking; the fit and the gains are applied at home without one.
    assert len(sent) == 5 and None not in sent
    for times in sent.values():
        assert np.diff(times).max() <= 1.0 / 50.0 + 1e-6
    assert rig.fake.ended == ["tuning_end"] * 5
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
    assert not rig.procedure.confirm() and not rig.procedure.skip() and not rig.procedure.abort()


def test_a_stalled_joint_fails_the_step_and_the_arm_holds(make_rig: Callable[..., Rig]) -> None:
    rig = make_rig(confirm_each=False)
    rig.fake.blocked = {"left_joint3"}

    status = rig.run(["check", "gravity_poses"])

    assert status["state"] == "failed"
    message = rig.messages()["gravity_poses"]
    assert rig.statuses()["gravity_poses"] == "failed"
    assert "left_joint3" in message and "more than 10 deg from its target" in message
    assert message.endswith("the arm holds where it is")
    assert not rig.fake.session_active()
    # Nothing is sent any more: the deadman holds the arm where it stopped,
    # give or take its sag, far from home.
    commands, held = rig.fake.commands, rig.fake.q.copy()
    rig.fake.advance(1.0)
    assert rig.fake.commands == commands and rig.fake.deadman_fired
    assert np.degrees(np.abs(rig.fake.q - held)).max() < 2.0
    assert np.degrees(np.abs(rig.fake.q - rig.fake.arm.home)).max() > 5.0


def test_a_lost_session_fails_the_step_without_raising(make_rig: Callable[..., Rig]) -> None:
    rig = make_rig(confirm_each=False)
    rig.fake.fail_after_commands = 400

    status = rig.run(["check", "gravity_poses", "gravity_fit"])

    assert status["state"] == "failed"
    assert rig.statuses()["gravity_poses"] == "failed"
    assert rig.statuses()["gravity_fit"] == "pending"
    assert rig.messages()["gravity_poses"] == (
        "tuning_command failed: feedback failed: the bus went silent"
    )
    assert status["message"].startswith("Gravity poses failed: tuning_command failed")
    assert (
        rig.fake.ended[-1]
        == "tuning_command raised RuntimeError: feedback failed: the bus went silent"
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
    caps = Capabilities(gravity_compensation=False, recording_unavailable="nothing is recorded")
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
    applied = rig.fake.status().calibration[left_spec.arm_id]
    assert applied is not None and applied["path"] == str(old)


def test_a_floating_joint_that_drifts_is_held_again_and_fails(
    make_rig: Callable[..., Rig],
) -> None:
    # Without the fit the gripper is 0.25 kg heavier than compensated: joints sag.
    rig = make_rig(float_window_s=5.0)

    status = rig.run(["check", "float_check"])

    assert status["state"] == "done", status["message"]
    joints = rig.procedure.results()["float_check"]["joints"]
    assert not joints["left_joint2"]["passed"]
    assert (
        "drifted" in joints["left_joint2"]["reason"]
        or joints["left_joint2"]["fingertip_drift_mm"] > 2.0
    )
    assert rig.fake.tuning_get_gains()["left_joint2"]["floating"] is False
    assert rig.procedure.results()["float_check"]["saved"] is None


def test_the_fake_keeps_the_tuning_session_rules(
    left_spec: ArmCalibrationSpec, arm: ArmModel, tmp_path: Path
) -> None:
    fake = FakeTuningArm(left_spec, arm=arm, calibration_root=tmp_path)
    home = dict(zip(left_spec.joint_names, (float(v) for v in arm.home), strict=True))
    with pytest.raises(RuntimeError, match="no tuning session"):
        fake.tuning_command(home)
    fake.tuning_begin(1.0)
    with pytest.raises(RuntimeError, match="already active"):
        fake.tuning_begin(1.0)
    assert not fake.session_active()

    fake.tuning_begin(1.0)
    assert not fake.position_command({"left_joint1": 0.1})
    assert fake.dropped_position_commands == 1
    fake.advance(1.5)
    assert not fake.session_active() and fake.ended[-1].startswith("no tuning call within")

    fake.tuning_begin(1.0)
    fake.tuning_command(home)
    fake.advance(0.2)
    assert not fake.session_active() and fake.ended[-1] == "the command deadman fired"

    fake.tuning_begin(1.0)
    fake.tuning_set_float("left_joint2", True, 0.5)
    fake.tuning_set_gains("left_joint3", 40.0, 2.0)
    with pytest.raises(ValueError):
        fake.tuning_command({"left_joint1": 10.0})
    assert not fake.session_active()
    gains = fake.tuning_get_gains()
    assert not gains["left_joint2"]["floating"] and (
        gains["left_joint3"]["kp"],
        gains["left_joint3"]["kd"],
    ) == (80.0, 5.0)
    assert fake.position_command({"left_joint1": 0.0})
    assert math.isclose(fake.target[0], 0.0)


def _far_from_home(rig: Rig) -> float:
    return float(np.degrees(np.abs(rig.fake.q - rig.fake.arm.home)).max())


def test_the_float_check_floats_at_the_first_gravity_pose_not_at_home(
    make_rig: Callable[..., Rig],
) -> None:
    rig = make_rig(float_window_s=1.0)
    prompts: list[str] = []

    def answer(prompt: str) -> str:
        prompts.append(prompt)
        return "confirm"

    status = rig.run(["float_check"], answer)

    assert status["state"] == "done", status["message"]
    assert prompts[0].startswith("The arm will move to gravity pose pose_01 [")
    result = rig.procedure.results()["float_check"]
    assert result["pose"] == "gravity pose pose_01"
    first = generate_poses(rig.procedure.arm, count=8).identification[0]
    assert np.allclose(result["pose_deg"], first.joints_deg, atol=1e-2)
    assert len(prompts) == 1 + len(LEFT_MEASURED)
    # It ends holding at the float pose, far from the folded home.
    assert _far_from_home(rig) > 20.0
    assert np.degrees(np.abs(rig.fake.q - np.asarray(first.joints_rad))).max() < 5.0
    assert not rig.fake.session_active()


def test_the_float_pose_can_be_set_and_must_be_valid(make_rig: Callable[..., Rig]) -> None:
    arm = make_rig().procedure.arm
    chosen = generate_poses(arm, count=8).identification[2]
    rig = make_rig(float_window_s=1.0, float_pose_deg=tuple(chosen.joints_deg))

    status = rig.run(["float_check"])

    assert status["state"] == "done", status["message"]
    assert rig.procedure.results()["float_check"]["pose"] == "the configured float pose"
    assert np.degrees(np.abs(rig.fake.q - np.asarray(chosen.joints_rad))).max() < 5.0

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
