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

"""Live tuning in the motor layer: gains, floating joints, feed-forward and their limits."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from dimos.robot.manipulators.calibration._inertial import LinkInertial
from dimos.robot.manipulators.damiao._damiao_native_test_helpers import (
    TEST_ROBOT,
    make_calibrated_test_robot,
    mass_test_model,
)
from dimos.robot.manipulators.damiao.arm import DamiaoArm, PhysicalModel
from dimos.robot.manipulators.damiao.robot import DamiaoRobot


def _arm(boundary, *, teach: bool = False, robot: DamiaoRobot = TEST_ROBOT) -> DamiaoArm:
    arm = DamiaoArm(robot, ["can1", "can0"], teach=teach)
    arm.connect()
    arm.enable()
    return arm


def _rows(boundary, bus: str) -> np.ndarray:
    return boundary[0].groups[bus].commands[-1]


def test_a_gain_override_holds_its_joint_and_reset_restores_the_bus_gains(boundary):
    arm = _arm(boundary)
    assert arm.set_gains("joint2", 40.0, 2.0) == (40.0, 2.0)
    # Sent at once, without waiting for a command.
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, :2], [[80, 5], [40, 2]])
    arm.command({"joint1": 0.5})
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, :3], [[80, 5, 0.5], [40, 2, 0.2]])
    assert arm.get_gains() == {"joint1": (80, 5), "joint2": (40, 2), "joint3": (20, 0.6)}
    arm.reset_gains()
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, :2], [[80, 5], [10, 1.5]])
    assert arm.get_gains()["joint2"] == (10.0, 1.5)


def test_gains_set_before_enable_apply_from_enable(boundary):
    arm = DamiaoArm(TEST_ROBOT, ["can1", "can0"])
    arm.connect()
    arm.set_gains("joint3", 30.0, 1.0)
    assert boundary[0].groups["right_arm"].commands == []
    arm.enable()
    arm.command({"joint1": 0.1})
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [30, 1])


@pytest.mark.parametrize(
    "kp,kd,applied",
    [(900.0, 7.0, (500.0, 5.0)), (-3.0, -1.0, (0.0, 0.0)), (500.0, 5.0, (500.0, 5.0))],
)
def test_gains_are_clamped_into_the_mit_frame_range(boundary, kp, kd, applied):
    arm = _arm(boundary)
    assert arm.set_gains("joint1", kp, kd) == applied
    np.testing.assert_equal(_rows(boundary, "left_arm")[0, :2], applied)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_non_finite_gains_are_refused_and_change_nothing(boundary, bad):
    arm = _arm(boundary)
    with pytest.raises(ValueError, match="finite"):
        arm.set_gains("joint1", bad, 1.0)
    with pytest.raises(ValueError, match="finite"):
        arm.set_gains("joint1", 1.0, bad)
    with pytest.raises(ValueError, match="finite"):
        arm.set_float("joint1", True, kd=bad)
    with pytest.raises(ValueError, match="finite"):
        arm.set_feedforward("joint1", bad)
    assert arm.get_gains()["joint1"] == (80.0, 5.0)
    assert arm.floating == {} and arm.feedforward == {}


@pytest.mark.parametrize("joint", ["missing", "left_finger"])
def test_tuning_names_an_arm_joint(boundary, joint):
    arm = _arm(boundary)
    for call in (
        lambda: arm.set_gains(joint, 1.0, 1.0),
        lambda: arm.set_float(joint, True),
        lambda: arm.set_feedforward(joint, 0.1),
        lambda: arm.apply_calibration({joint: (1.0, 1.0)}, {}),
    ):
        with pytest.raises(ValueError, match="is not an arm joint"):
            call()


def test_a_floating_joint_has_no_stiffness_keeps_gravity_and_is_held_where_released(
    boundary, mocker
):
    robot = replace(TEST_ROBOT, teach_kd=[2.0, 1.0, 0.5])
    arm = _arm(boundary, robot=robot)
    mocker.patch.object(arm._physical, "gravity", return_value=np.array([1.0, 2.0, 3.0]))
    native = boundary[0]
    arm.command({"joint2": -0.5})
    arm.set_float("joint2", True)
    np.testing.assert_equal(_rows(boundary, "left_arm"), [[80, 5, 0.2, 0, 1], [0, 1, 0.2, 0, 2]])
    # Moved by hand: the reader re-sends the gravity torque and the target follows.
    native.groups["left_arm"].q[:] = [0.2, 0.6]
    sent = len(native.groups["left_arm"].commands)
    arm.read()
    assert len(native.groups["left_arm"].commands) == sent + 1
    np.testing.assert_equal(_rows(boundary, "left_arm")[1], [0, 1, 0.6, 0, 2])
    # A coordinator's stale target for the floating joint is ignored too.
    arm.command({"joint2": -0.5})
    assert _rows(boundary, "left_arm")[1, 2] == 0.6
    arm.set_float("joint2", False)
    np.testing.assert_equal(_rows(boundary, "left_arm")[1], [10, 1.5, 0.6, 0, 2])
    # Not floating any more: the reader stops re-commanding the arm.
    sent = len(native.groups["left_arm"].commands)
    arm.read()
    assert len(native.groups["left_arm"].commands) == sent


def test_float_damping_is_given_else_teach_kd_else_the_holding_kd(boundary):
    arm = _arm(boundary)
    arm.set_float("joint3", True)
    assert arm.floating == {"joint3": 0.6}
    arm.set_float("joint3", True, kd=9.0)
    assert arm.floating == {"joint3": 5.0}
    np.testing.assert_equal(_rows(boundary, "right_arm")[0, :2], [0, 5])
    taught = _arm(boundary, robot=replace(TEST_ROBOT, teach_kd=[2.0, 1.0, 0.5]))
    taught.set_float("joint3", True)
    assert taught.floating == {"joint3": 0.5}


def test_feed_forward_adds_to_gravity_and_the_total_stays_within_the_motor_peak(boundary, mocker):
    arm = _arm(boundary)
    gravity = mocker.patch.object(arm._physical, "gravity", return_value=np.array([1.0, 2.0, 3.0]))
    assert arm.set_feedforward("joint3", 0.5) == 0.5
    assert _rows(boundary, "right_arm")[0, 4] == 3.5
    # joint3 is a DM4310: 10 N m peak, so a larger request is clamped.
    assert arm.set_feedforward("joint3", 50.0) == 10.0
    assert _rows(boundary, "right_arm")[0, 4] == 10.0
    # The clamp holds for the total too, gravity included: 28 N m on a DM4340.
    gravity.return_value = np.array([100.0, -100.0, -3.0])
    arm.command({"joint1": 0.1})
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, 4], [28.0, -10.0])
    assert _rows(boundary, "right_arm")[0, 4] == 7.0
    arm.set_feedforward("joint3", 0.0)
    assert arm.feedforward == {}


def test_teaching_wins_over_overrides_and_floating(boundary, mocker):
    robot = replace(TEST_ROBOT, teach_kd=[2.0, 1.0, 0.5])
    arm = _arm(boundary, teach=True, robot=robot)
    mocker.patch.object(arm._physical, "gravity", return_value=np.array([1.0, 2.0, 3.0]))
    arm.set_gains("joint1", 200.0, 4.0)
    arm.set_float("joint2", True, kd=3.0)
    arm.set_feedforward("joint1", 0.25)
    np.testing.assert_equal(_rows(boundary, "left_arm")[:, [0, 1, 4]], [[0, 2, 1.25], [0, 1, 2]])
    assert arm.get_gains()["joint1"] == (200.0, 4.0)


def test_effort_is_the_torque_the_motors_report_not_the_one_commanded(boundary, mocker):
    arm = _arm(boundary)
    mocker.patch.object(arm._physical, "gravity", return_value=np.array([1.0, 2.0, 3.0]))
    native = boundary[0]
    native.groups["left_arm"].tau[:] = [0.7, -0.3]
    native.groups["right_arm"].tau[:] = [0.05]
    arm.command({"joint1": 0.1})
    assert arm.read().effort == [0.7, -0.3, 0.05, 0.0, 0.0]


def test_a_calibration_replaces_gains_and_masses_and_reset_returns_to_it(boundary):
    robot = make_calibrated_test_robot()
    arm = _arm(boundary, robot=robot)
    arm.command({"joint1": 0.2})
    before = _rows(boundary, "right_arm")[0, 4]
    heavy = {"link3": LinkInertial(mass_kg=2.0, com_m=(0.1, 0.0, 0.0))}
    arm.apply_calibration({"joint3": (60.0, 3.0)}, heavy)
    after = _rows(boundary, "right_arm")[0]
    np.testing.assert_equal(after[:2], [60, 3])
    # link3 alone loads joint3, so twice its mass doubles that torque.
    assert after[4] == pytest.approx(2 * before)
    assert arm.inertial_overrides == heavy
    arm.set_gains("joint3", 20.0, 1.0)
    arm.reset_gains()
    assert arm.get_gains()["joint3"] == (60.0, 3.0)
    arm.apply_calibration({}, {})
    assert arm.get_gains()["joint3"] == (20.0, 0.6)
    assert _rows(boundary, "right_arm")[0, 4] == pytest.approx(before)


def test_a_calibration_that_does_not_fit_changes_nothing(boundary):
    arm = _arm(boundary, robot=make_calibrated_test_robot())
    arm.apply_calibration({"joint1": (60.0, 3.0)}, {})
    sent = len(boundary[0].groups["left_arm"].commands)
    with pytest.raises(ValueError, match=r"lacks: \['ghost', 'phantom'\]"):
        arm.apply_calibration(
            {"joint1": (1.0, 1.0)},
            {
                "phantom": LinkInertial(mass_kg=1.0, com_m=(0, 0, 0)),
                "ghost": LinkInertial(mass_kg=1.0, com_m=(0, 0, 0)),
            },
        )
    assert arm.get_gains()["joint1"] == (60.0, 3.0)
    assert len(boundary[0].groups["left_arm"].commands) == sent


def test_a_calibration_applied_before_connect_is_built_into_the_model(boundary):
    arm = DamiaoArm(make_calibrated_test_robot(), ["can1", "can0"])
    heavy = {"link3": LinkInertial(mass_kg=2.0, com_m=(0.1, 0.0, 0.0))}
    arm.apply_calibration({}, heavy)
    arm.connect()
    assert arm._physical is not None and arm._physical.inertial_overrides == heavy


def test_inertial_overrides_change_the_gravity_model():
    joints = ["joint1", "joint2", "joint3"]
    pose = np.array([0.2, -0.1, 0.3])
    plain = PhysicalModel(mass_test_model(), joints, gravity_comp=True).gravity(pose)
    heavy = PhysicalModel(
        mass_test_model(),
        joints,
        gravity_comp=True,
        inertial_overrides={"link3": LinkInertial(mass_kg=3.0, com_m=(0.1, 0.0, 0.0))},
    ).gravity(pose)
    assert heavy[2] == pytest.approx(3 * plain[2])
    assert abs(heavy[0]) > abs(plain[0])
    # No compensation: the overrides are checked but the torques stay zero.
    off = PhysicalModel(
        mass_test_model(),
        joints,
        gravity_comp=False,
        inertial_overrides={"link3": LinkInertial(mass_kg=3.0, com_m=(0.1, 0.0, 0.0))},
    )
    np.testing.assert_equal(off.gravity(pose), [0, 0, 0])


def test_inertial_overrides_must_name_bodies_of_the_reduced_model():
    with pytest.raises(ValueError, match=r"lacks: \['link9', 'tool'\]"):
        PhysicalModel(
            mass_test_model(),
            ["joint1", "joint2", "joint3"],
            gravity_comp=True,
            inertial_overrides={
                "tool": LinkInertial(mass_kg=1.0, com_m=(0, 0, 0)),
                "link9": LinkInertial(mass_kg=1.0, com_m=(0, 0, 0)),
                "link2": LinkInertial(mass_kg=1.0, com_m=(0, 0, 0)),
            },
        )
    with pytest.raises(ValueError, match="not finite"):
        PhysicalModel(
            mass_test_model(),
            ["joint1", "joint2", "joint3"],
            gravity_comp=True,
            inertial_overrides={"link2": LinkInertial(mass_kg=1.0, com_m=(float("nan"), 0, 0))},
        )
