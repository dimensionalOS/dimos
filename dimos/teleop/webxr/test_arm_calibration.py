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

"""Calibration endpoint accuracy, capture rejection and persistent profile isolation."""

import json
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dimos.teleop.webxr.arm_calibration import (
    ArmAnchors,
    ArmCalibration,
    CalibrationPose,
    RobotArmGeometry,
    arm_targets,
    head_frame,
    map_arm,
)


def pose(x, y=0, z=0, rotation=0):
    return CalibrationPose(
        position=(x, y, z), orientation=tuple(Rotation.from_euler("z", rotation).as_quat())
    )


@pytest.fixture
def calibration(tmp_path):
    geometry = {
        side: RobotArmGeometry(
            shoulder=(0, y, 0),
            reach_m=0.6,
            neutral_orientation=(0, 0, 0, 1),
            tool_to_wrist=(-0.1, 0, 0),
        )
        for side, y in (("left", 0.25), ("right", -0.25))
    }
    return ArmCalibration(geometry, "robot-geometry-v2", tmp_path)


def observe(c, side, p, start):
    for i in range(30):
        c.observe(side, p, np.eye(4), start + i * 0.02, start + i * 0.02)
    return start + 0.58


def complete(c, kind="hands"):
    c.begin("Alice", kind, f"device-{kind}", "session")
    for side, y in (("left", 0.2), ("right", -0.2)):
        observe(c, side, pose(0, y, -0.2), 1)
    c.capture("shoulders", 1.58, "session")
    for side, y in (("left", 0.2), ("right", -0.2)):
        observe(c, side, pose(0.3, y, -0.4), 2)
    c.capture("neutral", 2.58, "session")
    for index, (side, y) in enumerate((("left", 0.2), ("right", -0.2))):
        now = observe(c, side, pose(0.75, y, -0.2), 3 + index)
        c.capture(f"{side}_reach", now, "session")


@pytest.mark.parametrize("kind", ["hands", "controllers"])
def test_neutral_preserves_height_and_full_extension_uses_robot_wrist_reach(calibration, kind):
    c = calibration
    complete(c, kind)
    p = c.save(10, "session")
    assert not c.active
    c.activate(c.key(p), kind, f"device-{kind}", "next-session")
    observe(c, "left", pose(0.3, 0.2, -0.4), 20)
    for side, y in (("left", 0.2), ("right", -0.2)):
        robot = c.geometry[side]
        observe(c, side, pose(0.3, y, -0.4), 21)
        neutral = c.map(side)
        assert neutral.position == pytest.approx([0.34, robot.shoulder[1], -0.16])
        observe(c, side, pose(0.75, y, -0.2), 22)
        assert c.map(side).position == pytest.approx([0.7, robot.shoulder[1], 0])


def test_diagonal_reach_does_not_rotate_forward_into_up(calibration):
    # Reproduces the old bug: a forward-and-up capture used to rotate the whole mapping.
    human = ArmAnchors(
        shoulder=(0, 0.2, -0.2), neutral=pose(0.3, 0.2, -0.4), reach=pose(0.65, 0.2, -0.05)
    )
    first = map_arm(human.neutral, human, calibration.geometry["left"])
    second = map_arm(pose(0.4, 0.2, -0.4), human, calibration.geometry["left"])
    assert second.position[0] > first.position[0]
    assert second.position[1:] == pytest.approx(first.position[1:])
    assert first.position[2] < calibration.geometry["left"].shoulder[2]


def test_grip_rotation_moves_the_tool_around_the_wrist(calibration):
    human = ArmAnchors(
        shoulder=(0, 0, 0), neutral=pose(0.3, rotation=math.pi / 2), reach=pose(0.75)
    )
    robot = calibration.geometry["left"]
    result = map_arm(pose(0.3, rotation=math.pi), human, robot)
    assert Rotation.from_quat(result.orientation).as_rotvec() == pytest.approx([0, 0, math.pi / 2])
    assert result.position == pytest.approx([0.24, 0.35, 0])


def test_outside_reach_is_radially_bounded_without_axis_or_height_cross_coupling(calibration):
    complete(calibration)
    p = calibration.save(10, "session")
    requested, bounded, limited = arm_targets(
        pose(1.5, 0.2, -0.2), p.arms["left"], calibration.geometry["left"]
    )
    assert limited
    assert requested.position == pytest.approx([1.3, 0.25, 0])
    assert bounded.position == pytest.approx([0.7, 0.25, 0])


@pytest.mark.parametrize("yaw", [0, math.pi / 2, -math.pi / 2, math.pi, 2.9])
def test_turning_or_translating_operator_keeps_body_relative_hand_position_and_orientation(
    calibration, yaw
):
    c = calibration
    complete(c)
    p = c.save(10, "session")
    c.activate(c.key(p), "hands", "device-hands", "session-two")
    for side, y in (("left", 0.2), ("right", -0.2)):
        c.observe(side, pose(0.4, y, -0.3, rotation=0.3), np.eye(4), 20, 20)
    expected = {s: c.map(s) for s in ("left", "right")}
    # Rotate the whole operator, including wrist axes, and change their location.
    frame = head_frame((2, 3, 1), yaw)
    rotation = Rotation.from_matrix(frame[:3, :3])
    for side, y in (("left", 0.2), ("right", -0.2)):
        local = pose(0.4, y, -0.3, rotation=0.3)
        moved = CalibrationPose(
            position=tuple(frame[:3, :3] @ local.position + frame[:3, 3]),
            orientation=tuple((rotation * Rotation.from_quat(local.orientation)).as_quat()),
        )
        c.observe(side, moved, frame, 21, 21)
        actual = c.map(side)
        assert actual.position == pytest.approx(expected[side].position)
        assert (
            Rotation.from_quat(actual.orientation)
            * Rotation.from_quat(expected[side].orientation).inv()
        ).magnitude() < 1e-10
    # A sample for one hand must never reinterpret the other hand's paired frame.
    left = c.map("left")
    c.observe("right", pose(0.4, -0.2, -0.3), head_frame((0, 0, 0), -0.8), 22, 22)
    assert c.map("left") == left
    assert c.status(21)["operator_reference"] == "live_head_yaw"


def test_unstable_stale_and_replayed_tracking_cannot_be_captured(calibration):
    c = calibration
    c.begin("Alice", "hands", "hand-profile", "session")
    for side, y in (("left", 0.2), ("right", -0.2)):
        for i in range(30):
            c.observe(side, pose((i % 2) * 0.1, y, -0.2), np.eye(4), 1 + i * 0.02, 1 + i * 0.02)
    with pytest.raises(ValueError, match="Hold"):
        c.capture("shoulders", 1.58, "session")
    for side in ("left", "right"):
        observe(c, side, pose(0.3), 3)
        c.observe(side, pose(0.3), np.eye(4), 5, 3.58)
    with pytest.raises(ValueError, match="Hold|stale"):
        c.capture("shoulders", 5, "session")


def test_recalibration_is_transactional_and_neutral_invalidates_reaches(calibration):
    c = calibration
    complete(c)
    saved = c.save(10, "session")
    complete(c)
    for side, y in (("left", 0.2), ("right", -0.2)):
        observe(c, side, pose(0.35, y, -0.4), 5)
    c.capture("neutral", 5.58, "session")
    with pytest.raises(ValueError, match="both reaches"):
        c.save(11, "session")
    c.cancel("session")
    assert c.profiles() == [saved]
    assert not c.active


def test_profiles_cannot_cross_devices_input_methods_robot_models_or_sessions(
    calibration, tmp_path
):
    c = calibration
    complete(c)
    p = c.save(10, "session")
    for kind, device in (("controllers", "device-hands"), ("hands", "other-device")):
        with pytest.raises(ValueError, match="does not match"):
            c.activate(c.key(p), kind, device, "session")
    assert ArmCalibration(c.geometry, "different-robot", tmp_path).profiles() == []
    with pytest.raises(ValueError, match="another headset"):
        c.capture("neutral", 10, "wrong-session")
    reloaded = ArmCalibration(c.geometry, c.robot, tmp_path)
    assert reloaded.profiles() == [p]
    review = reloaded.status(20, c.key(p))
    assert review["review"]["left"]["scale"] == pytest.approx(0.8)
    assert review["profile_id"] is None
    assert not reloaded.active
    reloaded.delete(c.key(p))
    assert reloaded.profiles() == []


def test_bad_reach_cannot_replace_a_saved_profile(calibration):
    c = calibration
    complete(c)
    saved = c.save(10, "session")
    complete(c)
    now = observe(c, "left", pose(0.1, 0.2, -0.2), 5)
    with pytest.raises(ValueError, match="25 cm"):
        c.capture("left_reach", now, "session")
    assert c.profiles() == [saved]


def test_old_profile_requires_new_calibration_and_is_preserved(calibration):
    c = calibration
    complete(c)
    p = c.save(10, "session")
    old = p.model_dump()
    old["schema_version"] = 1
    for arm in old["arms"].values():
        arm.pop("shoulder")
    path = c.directory / f"{c.key(p)}.json"
    original = json.dumps(old)
    path.write_text(original)
    with pytest.raises(ValueError, match="shoulder calibration"):
        c.activate(c.key(p), "hands", "device-hands", "session-two")
    assert path.read_text() == original
