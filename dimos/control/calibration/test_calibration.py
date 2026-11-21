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

"""The calibration run on a fake toy arm, stepped sample by sample by the run's own waits."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import threading
import time
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import pytest

from dimos.control.calibration._fake_arm import FakeArm, FakeOperator
from dimos.control.calibration._inertial import LinkInertial, apply_inertial_overrides
from dimos.control.calibration._toy_arm import toy_arm
from dimos.control.calibration.gravity_poses import ArmModel
from dimos.control.calibration.module import (
    Calibration,
    CalibrationModuleConfig,
    CalibrationStatus,
    Operator,
    bake_calibration,
)
from dimos.control.calibration.motion import Motion, Sample
from dimos.control.calibration.schema import (
    ArmCalibration,
    ArmCalibrationError,
    GravityCalibration,
    calibration_path,
    gravity_links_from_inertial,
    save_calibration,
)
from dimos.control.calibration.steps import plan

TRUE_LINKS = {"fore": LinkInertial(mass_kg=0.7, com_m=(0.13, 0.0, 0.02))}
WAIT_S = 10.0


@pytest.fixture(scope="module")
def arm() -> ArmModel:
    return toy_arm()


def calibration(arm: ArmModel, operator: Operator, fake: FakeArm, **settings: Any) -> Calibration:
    config = CalibrationModuleConfig(
        poses=4,
        speed_deg_s=50.0,
        grid_points=400,
        confirm_poses=2,
        **settings,
    )
    operator.begin(plan(config.steps, can_float=True))
    run = Calibration(
        arm.spec, config, fake, operator, fake.command, arm.max_torque, fake.run_until
    )
    fake.listener = run.feed
    return run


def test_a_full_run_saves_the_fit_and_the_gains_and_ends_home(arm: ArmModel) -> None:
    fake = FakeArm(arm, true_links=TRUE_LINKS, friction_nm=dict.fromkeys(arm.joint_names, 0.02))
    operator = FakeOperator()
    run = calibration(arm, operator, fake)
    run.run()

    assert operator.done is not None
    ok, text, home = operator.done
    assert ok and home, text
    assert set(fake.file["links"]) == set(arm.fitted_links)
    fitted = arm.model.copy()
    apply_inertial_overrides(
        fitted,
        {k: LinkInertial(v["mass_kg"], tuple(v["com_m"])) for k, v in fake.file["links"].items()},
    )
    for pose in run.poses or []:
        q = np.asarray(pose.joints_rad)
        expected = pin.computeGeneralizedGravity(fake.truth, fake.truth.createData(), q)
        assert pin.computeGeneralizedGravity(fitted, fitted.createData(), q) == pytest.approx(
            expected, abs=0.01
        )
    assert set(fake.file["gains"]) == set(arm.joint_names)
    # The fit made the floats pass, and the tracking compared both visits.
    assert "2 of 2 floated joints ended within 2 mm" in text
    assert all(row["passed"] for row in fake.file["checks"]["float"]["joints"].values())
    assert "2 poses compared" in text
    assert fake.live == {} and fake.floating == {}
    np.testing.assert_allclose(fake.q, arm.home, atol=1e-3)


def test_every_move_is_confirmed_and_a_skipped_step_moves_nothing(arm: ArmModel) -> None:
    fake = FakeArm(arm, true_links=TRUE_LINKS)
    operator = FakeOperator(lambda prompt, c: "skip" if "visits" in prompt else c[0])
    calibration(arm, operator, fake, steps=["gravity_poses"]).run()

    assert operator.done is not None and operator.done[0], operator.done
    assert "gravity_poses: skipped by the operator" in operator.done[1]
    assert sum("Move to" in prompt for prompt in operator.asked) == 0
    moving = FakeOperator()
    calibration(arm, moving, FakeArm(arm), steps=["confirm_tracking"]).run()
    assert sum("Move to" in prompt for prompt in moving.asked) == 2 + 1


def test_abort_stops_streaming_so_the_deadman_holds_then_home_is_offered(arm: ArmModel) -> None:
    fake = FakeArm(arm)
    aborted_at: list[float] = []

    def answer(prompt: str, choices: list[str]) -> str:
        if "Go home?" in prompt:
            # The operator thinks it over: the arm gets no command, and the deadman holds it.
            for _ in range(30):
                fake.tick()
            return "home"
        return choices[0]

    operator = FakeOperator(answer)
    run = calibration(arm, operator, fake, steps=["gravity_poses"])

    def abort_mid_move(arm_: FakeArm) -> None:
        if not aborted_at and arm_.t > 1.0 and run._motion is not None:
            aborted_at.append(arm_.t)
            run.abort()

    fake.on_tick = abort_mid_move
    run.run()

    assert operator.done is not None
    ok, text, home = operator.done
    assert not ok and home and "aborted by the operator" in text
    [t_abort] = aborted_at
    held = [t for t in fake.halts if t > t_abort]
    assert held and held[0] - t_abort <= fake.deadman_s + 2 * fake.period_s
    resumed = [t for t in fake.commands if t > t_abort]
    assert resumed[0] > held[0]
    np.testing.assert_allclose(fake.q, arm.home, atol=1e-3)


def started(run: Calibration) -> threading.Thread:
    thread = threading.Thread(target=run.run, daemon=True, name="calibration")
    thread.start()
    return thread


def asked(operator: Operator, words: str) -> CalibrationStatus:
    """The status once the run asks a question with ``words`` in it, the earlier ones answered
    with their first answer."""
    deadline = time.monotonic() + WAIT_S
    while time.monotonic() < deadline:
        status = operator.status()
        if status.question is not None and words in status.question:
            return status
        if status.question is not None:
            operator.answer(status.answers[0])
        time.sleep(0.001)
    raise AssertionError(f"never asked {words!r}: {operator.status()}")


def test_the_run_waits_on_its_question_until_it_is_answered(arm: ArmModel) -> None:
    operator = Operator(arm.arm_id, [])
    thread = started(calibration(arm, operator, FakeArm(arm), steps=["summary"]))

    waiting = asked(operator, "Clear the workspace")
    assert (waiting.state, waiting.step, waiting.answers) == ("waiting", "check", ["go"])
    assert waiting.message == f"Calibrating {arm.arm_id}: check, summary"
    view = str(waiting).splitlines()
    assert view[0] == f"{arm.arm_id}: waiting at check"
    assert view[1].startswith(f"  {'check':<17} {'running':<8} measure the noise")
    assert view[2] == f"  {'gravity_poses':<17} {'skipped':<8} not selected"
    assert view[-2].startswith("? The arm moves home") and view[-1] == "  answer: go"
    assert operator.answer("skip") == "unknown answer 'skip'; answer go"
    assert operator.answer(" Go ") == "ok"
    assert operator.answer("go") == "no question pending"
    thread.join(WAIT_S)

    done = operator.status()
    rows = {row.name: (row.state, row.result) for row in done.steps}
    assert (done.state, done.question, done.answers) == ("done", None, [])
    assert rows["check"][0] == "done" and rows["check"][1].endswith("at home")
    assert rows["summary"] == ("done", "no calibration file")
    assert rows["gravity_fit"] == ("skipped", "not selected")
    assert done.summary.endswith("  summary: no calibration file")
    assert done.message == f"Calibration done:\n{done.summary}"
    assert done.log[1].startswith("? The arm moves home") and done.log[2] == "> go"


def test_an_abort_while_the_run_waits_holds_the_arm_and_offers_home(arm: ArmModel) -> None:
    fake, operator = FakeArm(arm), Operator(arm.arm_id, [])
    run = calibration(arm, operator, fake, steps=["gravity_poses"])
    thread = started(run)

    asked(operator, "Move to pose_01")
    commands = len(fake.commands)
    run.abort()
    offered = asked(operator, "Go home?")
    assert offered.question == (
        "gravity_poses: aborted by the operator\nThe arm holds where it is. Go home?"
    )
    assert offered.answers == ["home", "stay"]
    [row] = [row for row in offered.steps if row.name == "gravity_poses"]
    assert (row.state, row.result) == ("aborted", "aborted by the operator")
    assert operator.answer("stay") == "ok"
    thread.join(WAIT_S)

    ended = operator.status()
    assert ended.state == "aborted" and ended.summary == "gravity_poses: aborted by the operator"
    assert len(fake.commands) == commands and run.measurements == []


def test_bake_writes_the_fitted_links_of_the_file_the_arm_runs_beside_it(
    arm: ArmModel, tmp_path: Path
) -> None:
    fake = FakeArm(arm)
    with pytest.raises(ArmCalibrationError, match="runs no calibration file"):
        bake_calibration(arm.spec, fake.get_config()[arm.arm_id], "toy.urdf")
    links = gravity_links_from_inertial(arm.spec, TRUE_LINKS)
    fitted = ArmCalibration.for_spec(arm.spec, gravity=GravityCalibration(links=links))
    fake.path = str(save_calibration(calibration_path(arm.arm_id, tmp_path), fitted))
    fake.file = {"links": {name: link.model_dump(mode="json") for name, link in links.items()}}
    urdf = tmp_path / "toy.urdf"
    urdf.write_text(arm.spec.model.load().xml)

    written = Path(bake_calibration(arm.spec, fake.get_config()[arm.arm_id], str(urdf)))

    assert written == tmp_path / arm.arm_id / "calibrated.urdf"
    [mass] = ET.fromstring(written.read_text()).findall("./link[@name='fore']/inertial/mass")
    assert float(mass.attrib["value"]) == pytest.approx(0.7)
    other = tmp_path / "other.urdf"
    assert bake_calibration(arm.spec, fake.get_config()[arm.arm_id], str(urdf), str(other)) == str(
        other
    )
    assert other.read_text() == written.read_text()


def test_a_refused_set_config_fails_the_run_and_the_arm_holds(arm: ArmModel) -> None:
    fake = FakeArm(arm)
    fake.refuse = "refused by the test"
    operator = FakeOperator(lambda prompt, c: "stay" if "Go home?" in prompt else c[0])
    calibration(arm, operator, fake, steps=["step_sweep"]).run()

    assert operator.done is not None
    ok, text, home = operator.done
    assert not ok and not home
    assert "step_sweep failed: ValueError: set_config changed nothing" in text
    assert "refused by the test" in text
    assert fake.live == {}


def test_a_buzzing_candidate_stops_at_its_first_buzzing_step_and_the_sweep_goes_on(
    arm: ArmModel,
) -> None:
    fake = FakeArm(arm, buzz_above_kp=50.0)
    operator = FakeOperator()
    run = calibration(arm, operator, fake, steps=["step_sweep", "pick_gains", "confirm_tracking"])
    run.run()

    assert operator.done is not None and operator.done[0], operator.done
    assert {(j, pair) for j, pairs in run.stopped.items() for pair in pairs} == {
        (j, (60.0, kd)) for j in arm.joint_names for kd in (1.0, 2.5, 5.0)
    }
    stiff = [m for m in run.step_metrics if m.kp == 60.0]
    assert sorted({m.delta_deg for m in stiff}) == [-1.0, 1.0, 3.0]
    assert {m.kp for m in run.step_metrics} == {24.0, 40.0, 60.0}
    saved = {g["kp"] for g in fake.file["gains"].values()}
    assert 60.0 not in saved
    assert fake.live == {}


def test_friction_sets_each_joints_rest_limit_and_a_stuck_largest_step_rejects_its_gains(
    arm: ArmModel,
) -> None:
    fake, operator = FakeArm(arm, friction_nm={"elbow": 0.4}), FakeOperator()
    run = calibration(arm, operator, fake, steps=["step_sweep", "pick_gains", "confirm_tracking"])
    run.run()

    assert operator.done is not None and operator.done[0], operator.done
    assert run.friction["elbow"] == pytest.approx(0.4, rel=0.02) and run.friction["yaw"] < 0.01
    for m in run.step_metrics:
        friction = 0.4 if m.joint == "elbow" else 0.0
        assert m.rest_limit_deg == pytest.approx(
            max(0.1, math.degrees(1.5 * friction / m.kp)), rel=0.02
        )
    elbow = [m for m in run.step_metrics if m.joint == "elbow"]
    assert all(m.stuck for m in elbow if abs(m.delta_deg) == 1.0)
    assert all(m.stuck for m in elbow if m.kp == 24.0 and abs(m.delta_deg) == 3.0)
    assert all(not m.stuck for m in elbow if m.kp > 24.0 and abs(m.delta_deg) == 3.0)
    assert fake.file["gains"]["elbow"]["kp"] > 24.0


@pytest.mark.parametrize(
    "kp, dither_deg", [(40.0, math.degrees(0.3 * 10.0 / 40.0)), (200.0, 1.0), (10.0, 5.0)]
)
def test_each_joint_dithers_by_a_share_of_its_torque_over_kp_from_the_smallest_to_5_deg(
    arm: ArmModel, kp: float, dither_deg: float
) -> None:
    run = calibration(arm, FakeOperator(), FakeArm(arm, kp=kp), steps=["gravity_poses"])
    run.run()

    assert {m.joint: m.dither_deg for m in run.measurements} == {
        joint: pytest.approx(dither_deg) for joint in arm.measured_joints
    }
    assert not any(m.stuck for m in run.measurements)


def test_a_pair_the_joint_did_not_follow_is_redone_at_twice_the_dither_and_kept_if_still_stuck(
    arm: ArmModel,
) -> None:
    operator = FakeOperator()
    run = calibration(
        arm, operator, FakeArm(arm, friction_nm={"elbow": 1.6}), steps=["gravity_poses"]
    )
    run.run()
    elbow = [m for m in run.measurements if m.joint == "elbow"]
    assert len(elbow) == 4 and {m.dither_deg for m in elbow} == {5.0}
    assert not any(m.stuck for m in run.measurements)
    assert operator.done is not None and "4 redone at twice the dither, 0 stuck" in operator.done[1]

    operator = FakeOperator()
    run = calibration(
        arm, operator, FakeArm(arm, friction_nm={"elbow": 3.0}), steps=["gravity_poses"]
    )
    run.run()
    assert operator.done is not None and operator.done[0], operator.done
    assert [m.pose for m in run.measurements if m.stuck] == [p.name for p in run.poses or []]
    assert any("elbow: every pair stuck, friction not cancelled" in text for text in operator.shown)


def test_every_joint_whose_gravity_torque_can_vary_is_dithered(arm: ArmModel) -> None:
    elbow_only = ArmModel(replace(arm.spec, measured_joints=["elbow"]))
    fake, operator = FakeArm(elbow_only, true_links=TRUE_LINKS), FakeOperator()
    run = calibration(elbow_only, operator, fake, steps=["gravity_poses", "gravity_fit"])
    run.run()

    assert operator.done is not None and operator.done[0], operator.done
    assert run.dithered == ["shoulder", "elbow"]
    assert {m.joint for m in run.measurements} == {"shoulder", "elbow"}
    assert fake.file["fit"]["held_joints"] == {}


@pytest.mark.parametrize("answer", ["keep", "save"])
def test_a_fit_worse_over_all_than_the_running_model_is_saved_only_when_asked(
    arm: ArmModel, answer: str
) -> None:
    fake = FakeArm(arm, true_links=TRUE_LINKS, friction_nm={"shoulder": 0.5, "elbow": 0.5})
    fake.file = {
        "links": {k: {"mass_kg": v.mass_kg, "com_m": list(v.com_m)} for k, v in TRUE_LINKS.items()}
    }
    for _ in range(100):
        fake.tick()
    operator = FakeOperator(lambda prompt, c: answer if "anyway" in prompt else c[0])
    calibration(arm, operator, fake, steps=["gravity_poses", "gravity_fit"]).run()

    assert operator.done is not None and operator.done[0], operator.done
    assert any("worse over all the dithered joints" in prompt for prompt in operator.asked)
    assert ("fit" in fake.file) == (answer == "save")


def test_a_joint_rests_then_floats_its_whole_window_and_a_falling_one_is_held_again(
    arm: ArmModel,
) -> None:
    fake, operator = FakeArm(arm), FakeOperator()
    limp: dict[str, list[float]] = {}
    held: list[tuple[float, np.ndarray]] = []

    def watch(arm_: FakeArm) -> None:
        for joint in arm_.floating:
            limp.setdefault(joint, []).append(arm_.t)
        if not arm_.floating:
            held.append((arm_.t, arm_.target.copy()))

    fake.on_tick = watch
    calibration(arm, operator, fake, steps=["float_check"]).run()

    assert operator.done is not None and operator.done[0], operator.done
    assert set(limp) == {"shoulder", "elbow"}
    for times in limp.values():
        assert times[-1] - times[0] == pytest.approx(10.0, abs=0.05)
        before = [(t, target) for t, target in held if t < times[0]]
        moving = [t for t, target in before if not np.array_equal(target, before[-1][1])]
        assert before[-1][0] - moving[-1] > 2.0
    assert "2 of 2 floated joints ended within 2 mm" in operator.done[1]
    float_check = fake.file["checks"]["float"]
    assert float_check["window_s"] == 10.0 and float_check["limit_mm"] == 2.0

    falling, operator = FakeArm(arm, true_links=TRUE_LINKS), FakeOperator()
    calibration(arm, operator, falling, steps=["float_check"]).run()
    assert operator.done is not None and operator.done[0], operator.done
    rows = falling.file["checks"]["float"]["joints"]
    assert {row["ended"] for row in rows.values()} == {"drifted"}
    assert all(not row["passed"] and row["duration_s"] < 2.0 for row in rows.values())
    assert all(5.0 < abs(row["joint_drift_deg"]) < 5.5 for row in rows.values())
    assert falling.floating == {} and "0 of 2 floated joints" in operator.done[1]


@pytest.mark.parametrize("hz", [15.0, 35.0])
def test_a_hold_that_oscillates_in_place_ends_whatever_its_phase(arm: ArmModel, hz: float) -> None:
    motion = Motion(arm, [arm.home], 1.0, "converge", 0.5, np.full(3, 1e-4), np.full(3, np.inf))
    for k in range(500):
        q = arm.home + np.array([0.0, 0.0, 0.01 * math.cos(2.0 * math.pi * hz * k / 100.0)])
        motion.feed(Sample(k / 100.0, q, np.zeros(3)))
    assert motion.done.is_set() and motion.ended == "not converging"


TRY_GAINS = {
    "steps": ["gravity_poses", "step_sweep", "pick_gains", "confirm_tracking"],
    "kp_scales": [1.0, 1.5],
    "kd_scales": [1.0],
}


def test_the_proposed_gains_stream_with_every_move_and_are_saved_once_they_track(
    arm: ArmModel,
) -> None:
    fake = FakeArm(arm)
    trying: list[dict[str, tuple[float, float]]] = []

    def answer(prompt: str, choices: list[str]) -> str:
        if "Accept them?" in prompt:
            fake.on_tick = lambda arm_: (
                trying.append(arm_.gains()) if arm_.commands[-1] == arm_.t else None
            )
        if trying and "Move to" in prompt:
            for _ in range(30):
                fake.tick()
        return choices[0]

    operator = FakeOperator(answer)
    calibration(arm, operator, fake, **TRY_GAINS).run()

    assert operator.done is not None and operator.done[0], operator.done
    proposed = {j: (g["kp"], g["kd"]) for j, g in fake.file["gains"].items()}
    assert proposed == dict.fromkeys(arm.joint_names, (60.0, 1.0))
    assert trying and all(gains == proposed for gains in trying)
    assert any(t > 0.0 for t in fake.halts)
    assert set(fake.file["checks"]) == {"tracking"}
    assert "saved gains for yaw, shoulder, elbow" in operator.done[1]


def test_a_stall_with_the_proposed_gains_saves_nothing_and_gives_the_gains_back(
    arm: ArmModel,
) -> None:
    fake = FakeArm(arm)

    def answer(prompt: str, choices: list[str]) -> str:
        if "The arm revisits" in prompt:
            pinned = fake.q[2]

            def pin_elbow(arm_: FakeArm) -> None:
                arm_.q[2] = pinned

            fake.on_tick = pin_elbow
        return "stay" if "Go home?" in prompt else choices[0]

    operator = FakeOperator(answer)
    calibration(arm, operator, fake, **TRY_GAINS).run()

    assert operator.done is not None and operator.done[0], operator.done
    assert "confirm_tracking: gains not saved: stalled or blocked" in operator.done[1]
    assert any("the proposed gains are undone. Go home?" in prompt for prompt in operator.asked)
    assert "gains" not in fake.file and fake.gains() == fake.defaults


def test_proposed_gains_that_rest_worse_than_the_first_visit_are_not_saved(
    arm: ArmModel,
) -> None:
    fake = FakeArm(arm)

    def answer(prompt: str, choices: list[str]) -> str:
        if "The arm revisits" in prompt:
            fake.friction[2] = 1.5
        return choices[0]

    operator = FakeOperator(answer)
    calibration(arm, operator, fake, **TRY_GAINS).run()

    assert operator.done is not None and operator.done[0], operator.done
    assert "gains not saved: elbow rests" in operator.done[1]
    assert "gains" not in fake.file and fake.gains() == fake.defaults


def test_moves_default_to_10_deg_s_up_to_60_and_each_dither_holds_a_second(
    arm: ArmModel,
) -> None:
    config = CalibrationModuleConfig()
    assert (config.speed_deg_s, config.dither_hold_s) == (10.0, 1.0)
    CalibrationModuleConfig(speed_deg_s=60.0)
    with pytest.raises(ValueError, match="less than or equal to 60"):
        CalibrationModuleConfig(speed_deg_s=61.0)
    with pytest.raises(ValueError, match="outlast rest_window_s"):
        CalibrationModuleConfig(dither_hold_s=0.5)
    run = calibration(arm, FakeOperator(), FakeArm(arm), steps=["gravity_poses"])
    holds: list[float] = []
    stream = run.stream

    def timed(motion: Motion, patch: dict[str, Any] | None = None) -> Motion:
        ended = stream(motion, patch)
        if motion.mode == "fixed":
            holds.append(motion.t[-1] - motion.t[0])
        return ended

    run.stream = timed
    run.run()
    assert len(holds) == 4 * 2 * 2 + 1
    assert all(hold == pytest.approx(1.0, abs=0.011) for hold in holds[1:])


def test_a_hold_on_a_creeping_joint_ends_not_converging_at_its_bound(arm: ArmModel) -> None:
    def creep(max_hold_s: float) -> Motion:
        motion = Motion(
            arm,
            [arm.home],
            1.0,
            "converge",
            0.5,
            np.full(3, 1e-4),
            np.full(3, np.inf),
            max_hold_s=max_hold_s,
        )
        for k in range(1000):
            motion.feed(
                Sample(k / 100.0, arm.home + np.array([0.0, 0.0, 0.002 * k / 100.0]), np.zeros(3))
            )
        return motion

    bounded = creep(CalibrationModuleConfig().max_hold_s)
    assert bounded.ended == "not converging"
    assert bounded.t[-1] - bounded.t[0] == pytest.approx(5.0, abs=0.011)
    assert not creep(math.inf).done.is_set()
    with pytest.raises(ValueError, match="two rest windows"):
        CalibrationModuleConfig(max_hold_s=1.0)


def test_the_plan_skips_what_cannot_run() -> None:
    rows = dict(plan(["gravity_fit", "float_check"], can_float=False))
    assert rows["check"] is None
    assert rows["gravity_fit"] == "needs gravity_poses in the same run"
    assert rows["float_check"] is not None and "compensate gravity" in rows["float_check"]
    assert rows["summary"] == "not selected"
