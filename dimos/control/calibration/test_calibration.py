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

from typing import Any

import numpy as np
import pinocchio as pin
import pytest

from dimos.control.calibration._fake_arm import FakeArm, FakeOperator
from dimos.control.calibration._inertial import LinkInertial, apply_inertial_overrides
from dimos.control.calibration._toy_arm import toy_arm
from dimos.control.calibration.gravity_poses import ArmModel
from dimos.control.calibration.module import Calibration, CalibrationModuleConfig
from dimos.control.calibration.steps import plan

TRUE_LINKS = {"fore": LinkInertial(mass_kg=0.7, com_m=(0.13, 0.0, 0.02))}


@pytest.fixture(scope="module")
def arm() -> ArmModel:
    return toy_arm()


def calibration(
    arm: ArmModel, operator: FakeOperator, fake: FakeArm, **settings: Any
) -> Calibration:
    config = CalibrationModuleConfig(
        robot="openyam",
        poses=4,
        speed_deg_s=50.0,
        grid_points=400,
        confirm_poses=2,
        **settings,
    )
    run = Calibration(
        arm.spec, config, fake, operator.call, fake.command, arm.max_torque, fake.run_until
    )
    fake.listener = run.feed
    return run


def test_a_full_run_saves_the_fit_and_the_gains_and_ends_home(arm: ArmModel) -> None:
    fake, operator = FakeArm(arm, true_links=TRUE_LINKS), FakeOperator()
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
    assert "2 of 2 floated joints stayed within" in text
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


def test_a_buzzing_gain_fails_its_steps_and_is_not_picked(arm: ArmModel) -> None:
    fake = FakeArm(arm, buzz_above_kp=50.0)
    operator = FakeOperator()
    run = calibration(arm, operator, fake, steps=["step_sweep", "pick_gains"])
    run.run()

    assert operator.done is not None and operator.done[0], operator.done
    buzzed = [m for m in run.step_metrics if any("not converging" in f for f in m.failures)]
    assert {(m.joint, m.kp, m.delta_deg) for m in buzzed} == {
        (joint, 60.0, 3.0) for joint in arm.joint_names
    }
    assert {m.kp for m in run.step_metrics} == {40.0, 60.0}
    saved = {g["kp"] for g in fake.file["gains"].values()}
    assert saved == {40.0}


def test_the_plan_skips_what_cannot_run() -> None:
    rows = dict(plan(["gravity_fit", "float_check"], can_float=False))
    assert rows["check"] is None
    assert rows["gravity_fit"] == "needs gravity_poses in the same run"
    assert rows["float_check"] is not None and "compensate gravity" in rows["float_check"]
    assert rows["summary"] == "not selected"
