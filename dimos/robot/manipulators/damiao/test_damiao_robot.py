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

"""Every arm's robot data: unchanged motor addresses, profiles, gains and jaws."""

from __future__ import annotations

from dataclasses import replace

import pytest

from dimos.robot.manipulators.damiao.robot import DamiaoMotor, DamiaoProfile, DamiaoRobot
from dimos.robot.manipulators.dual_makermods_metal.config import DUAL_METAL
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM
from dimos.robot.manipulators.makermods_metal.config import METAL
from dimos.robot.manipulators.openarm.config import OPENARM
from dimos.robot.manipulators.openyam.config import OPENYAM
from dimos.robot.manipulators.seeed_b601_dm.config import B601_DM

LOW = ("low", 0x8000, 6.28, 10.0, 30.0)
SHOULDER = ("shoulder", 0x8001, 6.28, 10.0, 120.0)
HIGH = ("high", 0x8002, 6.28, 30.0, 20.0)
METAL_ARMS = [LOW, SHOULDER, LOW, LOW, HIGH, HIGH]


def _profile(profile: DamiaoProfile | None) -> tuple | None:
    if profile is None:
        return None
    for name, code, p_max, v_max, t_max in (LOW, SHOULDER, HIGH):
        if (code, p_max, v_max, t_max) == (
            profile.code,
            profile.p_max,
            profile.v_max,
            profile.t_max,
        ):
            return (name, code, p_max, v_max, t_max)
    return (None, profile.code, profile.p_max, profile.v_max, profile.t_max)


def _wiring(robot: DamiaoRobot) -> list[tuple]:
    return [
        (
            bus.name,
            [(motor.name, motor.type, motor.send_id, motor.reply_id) for motor in bus.arm],
            bus.kp,
            bus.kd,
            [_profile(motor.profile) for motor in bus.arm],
            None
            if bus.gripper is None
            else (
                bus.gripper.name,
                bus.gripper.motor.name,
                bus.gripper.motor.type,
                bus.gripper.motor.send_id,
                bus.gripper.motor.reply_id,
                _profile(bus.gripper.motor.profile),
                bus.gripper.opening,
                bus.gripper.current,
                bus.gripper.calibration_travel,
            ),
        )
        for bus in robot.buses
    ]


def _motors(prefix: str, types: list[str]) -> list[tuple]:
    return [
        (f"{prefix}{index}", type_, index, index + 0x10) for index, type_ in enumerate(types, 1)
    ]


def _gripper(
    name: str, motor: str, motor_type: str, send: int, reply: int, profile, opening, current, travel
):
    return (name, motor, motor_type, send, reply, profile, opening, current, travel)


OPENYAM_WIRING = [
    (
        "arm",
        _motors("yam_joint", ["DM4340"] * 3 + ["DM4310"] * 3),
        [80.0, 80.0, 80.0, 10.0, 10.0, 10.0],
        [5.0, 5.0, 5.0, 1.5, 1.5, 1.5],
        [None] * 6,
        _gripper(
            "gripper", "yam_gripper", "DM4310", 0x08, 0x18, None, "decreasing_position", 0.15, None
        ),
    )
]
DUAL_OPENYAM_WIRING = [
    (
        f"{side}_arm",
        _motors(f"{side}_joint", ["DM4340"] * 3 + ["DM4310"] * 3),
        [80.0, 80.0, 80.0, 10.0, 10.0, 10.0],
        [1.0, 1.0, 1.0, 1.5, 1.5, 1.5],
        [None] * 6,
        _gripper(
            f"{side}_gripper",
            f"{side}_gripper",
            "DM4310",
            0x08,
            0x18,
            None,
            "decreasing_position",
            0.15,
            None,
        ),
    )
    for side in ("left", "right")
]
OPENARM_WIRING = [
    (
        f"{side}_arm",
        _motors(f"openarm_{side}_joint", ["DM8009"] * 2 + ["DM4340"] * 2 + ["DM4310"] * 3),
        [100.0, 100.0, 80.0, 80.0, 60.0, 60.0, 60.0],
        [1.5, 1.5, 1.0, 1.0, 0.8, 0.8, 0.8],
        [None] * 7,
        _gripper(
            f"{side}_gripper",
            f"openarm_{side}_gripper",
            "DM4310",
            0x08,
            0x18,
            None,
            "decreasing_position",
            0.15,
            None,
        ),
    )
    for side in ("left", "right")
]
METAL_WIRING = [
    (
        "arm",
        _motors("joint", ["profile"] * 6),
        [100.0, 100.0, 100.0, 50.0, 20.0, 20.0],
        [5.0, 5.0, 5.0, 5.0, 0.6, 0.6],
        METAL_ARMS,
        _gripper(
            "gripper", "gripper", "profile", 0x07, 0x17, HIGH, "increasing_position", 0.1, None
        ),
    )
]
DUAL_METAL_WIRING = [
    (
        f"{side}_arm",
        _motors(f"{side}_joint", ["profile"] * 6),
        [100.0, 100.0, 100.0, 50.0, 20.0, 20.0],
        [5.0, 5.0, 5.0, 5.0, 0.6, 0.6],
        METAL_ARMS,
        _gripper(
            f"{side}_gripper",
            f"{side}_gripper",
            "profile",
            0x07,
            0x17,
            HIGH,
            "increasing_position",
            0.1,
            None,
        ),
    )
    for side in ("left", "right")
]
B601_WIRING = [
    (
        "arm",
        _motors("joint", ["DM4340"] * 3 + ["DM4310"] * 3),
        [120.0, 120.0, 120.0, 18.0, 18.0, 18.0],
        [5.0, 5.0, 5.0, 2.0, 2.0, 2.0],
        [None] * 6,
        _gripper(
            "gripper", "gripper_motor", "DM4310", 0x07, 0x17, None, "decreasing_position", 0.15, 7.0
        ),
    )
]

ARMS = [
    pytest.param(OPENYAM, OPENYAM_WIRING, "base", True, [2.0, 2.0, 2.0, 0.5, 0.5, 0.5], "openyam"),
    pytest.param(
        DUAL_OPENYAM,
        DUAL_OPENYAM_WIRING,
        "dual_openyam_base",
        True,
        None,
        "dual_openyam",
    ),
    pytest.param(OPENARM, OPENARM_WIRING, "openarm_body_link0", True, None, "openarm"),
    pytest.param(METAL, METAL_WIRING, "base_link", False, None, "metal"),
    pytest.param(DUAL_METAL, DUAL_METAL_WIRING, "dual_metal_base", False, None, "dual_metal"),
    pytest.param(B601_DM, B601_WIRING, "base_link", True, None, "b601"),
]


@pytest.mark.parametrize(
    "robot,wiring,base_link,gravity,teach_kd,name",
    ARMS,
    ids=[case.id for case in ARMS],
)
def test_robot_data_is_unchanged(robot, wiring, base_link, gravity, teach_kd, name):
    assert _wiring(robot) == wiring
    assert robot.base_link == base_link
    assert robot.gravity is gravity
    assert robot.teach_kd == teach_kd
    assert robot.joint_names == [*robot.arm_joint_names, *robot.gripper_joint_names]
    assert robot.model is not None
    assert robot.gravity_model is not None


@pytest.mark.parametrize("robot", [case.values[0] for case in ARMS])
def test_robot_data_is_internally_consistent(robot):
    assert len(robot.buses) >= 1
    for bus in robot.buses:
        assert bus.name
        assert len(bus.kp) == len(bus.kd) == len(bus.arm)
        if bus.gripper is not None:
            assert bus.gripper.current > 0
    if robot.home_joints is not None:
        assert len(robot.home_joints) == len(robot.arm_joint_names)
    assert len(set(robot.joint_names)) == len(robot.joint_names)


def test_robot_data_rejects_a_mismatched_joint_order():
    with pytest.raises(ValueError, match="joint_names"):
        replace(OPENYAM, joint_names=["gripper", *OPENYAM.arm_joint_names])


def test_robot_data_rejects_a_gain_count_mismatch():
    bus = replace(OPENYAM.buses[0], kp=[1.0])
    with pytest.raises(ValueError, match="one gain per arm motor"):
        replace(OPENYAM, buses=[bus])


def test_robot_data_rejects_duplicate_bus_names():
    with pytest.raises(ValueError, match="bus names"):
        replace(OPENYAM, buses=[OPENYAM.buses[0], OPENYAM.buses[0]])


def test_robot_data_rejects_duplicate_motor_names():
    bus = OPENYAM.buses[0]
    with pytest.raises(ValueError, match="motor names"):
        replace(OPENYAM, buses=[replace(bus, arm=[bus.arm[0], bus.arm[0], *bus.arm[2:]])])


def test_robot_data_rejects_an_empty_bus_list():
    with pytest.raises(ValueError, match="at least one bus"):
        replace(OPENYAM, buses=[])


def test_robot_data_rejects_a_bus_without_motors():
    bus = replace(OPENYAM.buses[0], arm=[], kp=[], kd=[])
    with pytest.raises(ValueError, match="no arm motors"):
        replace(OPENYAM, buses=[bus], joint_names=["gripper"])


def test_robot_data_rejects_duplicate_gripper_names():
    bus = OPENYAM.buses[0]
    second = replace(
        bus,
        arm=[DamiaoMotor("extra", "DM4310", 9, 25)],
        kp=[1.0],
        kd=[1.0],
        name="second",
    )
    with pytest.raises(ValueError, match="gripper names"):
        replace(
            OPENYAM,
            buses=[bus, second],
            joint_names=[*OPENYAM.arm_joint_names, "extra", "gripper"],
        )


def test_robot_data_rejects_wrong_teach_damping_or_home_length():
    with pytest.raises(ValueError, match="teach_kd"):
        replace(OPENYAM, teach_kd=[1.0])
    with pytest.raises(ValueError, match="home_joints"):
        replace(OPENYAM, home_joints=[0.0])
