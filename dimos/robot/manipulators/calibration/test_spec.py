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

from dataclasses import replace
import math

import pytest

from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.calibration._calibration_test_helpers import (
    require_model,
    robot_specs_or_skip,
)
from dimos.robot.manipulators.calibration.robots import KNOWN_ROBOTS, arm_spec, robot_id
from dimos.robot.manipulators.calibration.spec import (
    DAMIAO_STOCK_TORQUE_LIMIT_NM,
    DEFAULT_TORQUE_SAFETY_FACTOR,
    ArmCalibrationSpec,
    build_arm_spec,
    merged_links,
    model_sha256,
    specs_from_damiao_robot,
    urdf_limits,
)
from dimos.robot.manipulators.dual_openyam.calibration_workspace import (
    DUAL_OPENYAM_BENCH_BASE_SPACING_M,
    DUAL_OPENYAM_MIDLINE_MARGIN_M,
    DUAL_OPENYAM_TABLE_Z_M,
)
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM
from dimos.robot.manipulators.makermods_metal.config import METAL
from dimos.robot.manipulators.openarm.config import OPENARM
from dimos.robot.manipulators.openyam.config import OPENYAM

DM4340 = DAMIAO_STOCK_TORQUE_LIMIT_NM["DM4340"] * DEFAULT_TORQUE_SAFETY_FACTOR
DM4310 = DAMIAO_STOCK_TORQUE_LIMIT_NM["DM4310"] * DEFAULT_TORQUE_SAFETY_FACTOR


def assert_ranges_inside_urdf_limits(spec: ArmCalibrationSpec) -> None:
    joints = {joint.name: joint for joint in spec.model.load().joints}
    for name, (low, high) in spec.joint_ranges.items():
        lower, upper = urdf_limits(joints[name])
        assert lower <= low <= high <= upper, name


def test_dual_openyam_bench_specs(dual_openyam_specs: list[ArmCalibrationSpec]) -> None:
    left, right = dual_openyam_specs
    assert [left.arm_id, right.arm_id] == ["dual_openyam/left", "dual_openyam/right"]
    for spec, side in ((left, "left"), (right, "right")):
        assert spec.joint_names == [f"{side}_joint{i}" for i in range(1, 7)]
        assert spec.base_frame == f"{side}_base" and spec.tip_frame == f"{side}_grasp_frame"
        # The yaw and the wrist roll carry no gravity signal.
        assert spec.measured_joints == [f"{side}_joint{i}" for i in (2, 3, 4, 5)]
        assert spec.gravity_links == [
            *(f"{side}_link{i}" for i in (2, 3, 4, 5)),
            f"{side}_gripper",
        ]
        assert spec.max_torque_nm == {
            **{f"{side}_joint{i}": pytest.approx(DM4340) for i in (1, 2, 3)},
            **{f"{side}_joint{i}": pytest.approx(DM4310) for i in (4, 5, 6)},
        }
        assert spec.max_torque_nm[f"{side}_joint2"] == pytest.approx(10.0)
        assert spec.joint_ranges[f"{side}_joint2"] == pytest.approx(
            (math.radians(25.0), math.radians(150.0))
        )
        assert spec.joint_ranges[f"{side}_joint1"] == (0.0, 0.0)
        assert spec.home_joints[f"{side}_joint2"] == pytest.approx(1.047)
        assert spec.gains[f"{side}_joint2"] == (80.0, 5.0)
        assert_ranges_inside_urdf_limits(spec)
        guard = spec.workspace
        assert guard.floor_z_m == DUAL_OPENYAM_TABLE_Z_M
        assert guard.min_pose_z_m == pytest.approx(0.12)
        assert guard.min_transition_z_m == pytest.approx(0.08)
        assert guard.min_x_m == 0.15 and guard.point_offset_m == (0.0, 0.0, -0.046)
        (midline,) = guard.keep_out
        assert midline.offset_m == pytest.approx(
            DUAL_OPENYAM_BENCH_BASE_SPACING_M / 2 + DUAL_OPENYAM_MIDLINE_MARGIN_M
        )
        assert midline.normal == (0.0, -1.0 if side == "left" else 1.0, 0.0)
        # Every frame the old filter checked is still checked, plus the elbow and
        # the grasp frame; the shoulder, which cannot move, is not.
        assert guard.clearance_frames is not None
        for suffix in ("tip_left", "tip_right", "gripper", "link5", "link4", "link3"):
            assert f"{side}_{suffix}" in guard.clearance_frames
        assert f"{side}_link2" not in guard.clearance_frames
    assert left.model_sha256() == right.model_sha256()


def test_dual_openyam_default_spec_derives_everything_from_the_model() -> None:
    require_model(DUAL_OPENYAM.gravity_model, "Dual OpenYAM")
    left, right = specs_from_damiao_robot(DUAL_OPENYAM, arm_id="dual_openyam")
    for spec in (left, right):
        assert len(spec.joint_names) == 6
        assert_ranges_inside_urdf_limits(spec)
        joints = {joint.name: joint for joint in spec.model.load().joints}
        for name, (low, high) in spec.joint_ranges.items():
            lower, upper = urdf_limits(joints[name])
            margin = max(0.1 * (upper - lower), math.radians(5.0))
            assert low == pytest.approx(lower + margin) and high == pytest.approx(upper - margin)
        # Sampled over the whole yaw range, joint 1 still carries no gravity.
        assert spec.joint_names[0] not in spec.measured_joints
        assert spec.joint_names[5] not in spec.measured_joints
        assert len(spec.gravity_links) == 5
        # The default floor is the model root's z = 0 plane: the bases' level here.
        assert spec.workspace.floor_z_m == pytest.approx(0.0)
        assert spec.workspace.keep_out == () and spec.workspace.min_x_m is None


def test_openyam_spec() -> None:
    (spec,) = robot_specs_or_skip(OPENYAM, "openyam", "OpenYAM (yam_description LFS archive)")
    assert spec.arm_id == "openyam"
    assert spec.joint_names == [f"yam_joint{i}" for i in range(1, 7)]
    assert "yam_joint1" not in spec.measured_joints
    assert spec.max_torque_nm["yam_joint1"] == pytest.approx(DM4340)
    assert_ranges_inside_urdf_limits(spec)


def test_b601_spec_from_its_motor_table(b601_spec: ArmCalibrationSpec) -> None:
    assert b601_spec.arm_id == "b601_dm"
    assert b601_spec.joint_names == [f"joint{i}" for i in range(1, 7)]
    assert b601_spec.base_frame == "base_link" and b601_spec.tip_frame == "end_link"
    assert b601_spec.measured_joints == ["joint2", "joint3", "joint4", "joint5"]
    assert b601_spec.gravity_links == ["link2", "link3", "link4", "link5", "link6"]
    assert b601_spec.max_torque_nm == {
        **{f"joint{i}": pytest.approx(DM4340) for i in (1, 2, 3)},
        **{f"joint{i}": pytest.approx(DM4310) for i in (4, 5, 6)},
    }
    assert_ranges_inside_urdf_limits(b601_spec)
    joints = list(b601_spec.model.load().joints)
    # The tool flange and both fingers ride on link6's body.
    assert merged_links(joints, b601_spec.joint_names, "link6") == [
        "end_link",
        "finger_left_link",
        "finger_right_link",
    ]


def test_metal_spec_takes_torque_limits_from_its_custom_profiles() -> None:
    (spec,) = robot_specs_or_skip(METAL, "metal", "MakerMods Metal")
    profiles = [motor.profile for motor in METAL.buses[0].arm]
    assert all(profile is not None for profile in profiles)
    assert list(spec.max_torque_nm.values()) == pytest.approx(
        [profile.t_max * DEFAULT_TORQUE_SAFETY_FACTOR for profile in profiles if profile]
    )
    assert spec.max_torque_nm["joint2"] == pytest.approx(120.0 * DEFAULT_TORQUE_SAFETY_FACTOR)
    assert spec.tip_frame == "gripper_base"
    assert "joint1" not in spec.measured_joints
    assert_ranges_inside_urdf_limits(spec)


def test_openarm_keeps_its_horizontal_shoulder_joint() -> None:
    left, right = robot_specs_or_skip(OPENARM, "openarm", "OpenArm")
    assert [left.arm_id, right.arm_id] == ["openarm/left", "openarm/right"]
    for spec in (left, right):
        assert len(spec.joint_names) == 7
        # The first joint pitches the shoulder about a horizontal axis: it is measured.
        assert spec.joint_names[0] in spec.measured_joints
        assert spec.max_torque_nm[spec.joint_names[0]] == pytest.approx(
            DAMIAO_STOCK_TORQUE_LIMIT_NM["DM8009"] * DEFAULT_TORQUE_SAFETY_FACTOR
        )
        assert spec.tip_frame.endswith("grasp_frame")
        # The arms hang below their bases: the floor is the stand's, far below.
        assert spec.workspace.floor_z_m < -0.5
        assert_ranges_inside_urdf_limits(spec)


def test_overrides_are_checked_against_the_urdf(left_spec: ArmCalibrationSpec) -> None:
    names = left_spec.joint_names
    with pytest.raises(ValueError, match="leaves the URDF limits"):
        build_arm_spec(
            "x", left_spec.model, names, joint_ranges={names[1]: (-1.0, 1.0)}, max_torque_nm={}
        )
    with pytest.raises(ValueError, match="unknown joint"):
        build_arm_spec("x", left_spec.model, names, joint_ranges={"nope": (0.0, 0.0)})
    with pytest.raises(ValueError, match="no URDF effort"):
        build_arm_spec("x", _without_efforts(left_spec.model), names)
    with pytest.raises(ValueError, match="arm_id"):
        build_arm_spec("../escape", left_spec.model, names, max_torque_nm=left_spec.max_torque_nm)
    spec = build_arm_spec(
        "x",
        left_spec.model,
        names,
        measured_joints=[names[1]],
        gravity_links=[f"left_link{i}" for i in (2, 3)],
        max_torque_nm=left_spec.max_torque_nm,
    )
    assert spec.measured_joints == [names[1]] and spec.gravity_links == ["left_link2", "left_link3"]


def _without_efforts(model: RobotModel) -> RobotModel:
    return RobotModel.from_urdf_string(model.load().xml.replace('effort="1"', 'effort="0"'))


def test_model_fingerprint_ignores_mesh_paths_but_not_edits(left_spec: ArmCalibrationSpec) -> None:
    xml = left_spec.model.load().xml
    here = RobotModel.from_urdf_string(xml)
    elsewhere = RobotModel.from_urdf_string(
        xml.replace("/home/anvil/dimos-wt/arm-calibration/", "/srv/other/checkout/")
    )
    assert model_sha256(here) == model_sha256(elsewhere) == left_spec.model_sha256()
    heavier = RobotModel.from_urdf_string(xml.replace('value="1.46908"', 'value="1.6"', 1))
    assert model_sha256(heavier) != model_sha256(here)
    limited = here.with_joint_position_limits("left_joint2", lower=0.1, upper=3.0)
    assert model_sha256(limited) != model_sha256(here)
    assert left_spec.model_name() == "dual_openyam"


def test_known_robots_resolve_arms_by_name(dual_openyam_specs: list[ArmCalibrationSpec]) -> None:
    assert set(KNOWN_ROBOTS) == {
        "openyam",
        "dual-openyam",
        "metal",
        "dual-metal",
        "b601-dm",
        "openarm",
    }
    assert robot_id("dual-openyam") == "dual_openyam"
    for arm in ("left", "left_arm", "dual_openyam/left"):
        assert arm_spec("dual-openyam", arm).arm_id == "dual_openyam/left"
    assert arm_spec("dual-openyam", "right").workspace.keep_out[0].normal == (0.0, 1.0, 0.0)
    with pytest.raises(ValueError, match="several arms"):
        arm_spec("dual-openyam")
    with pytest.raises(ValueError, match="no arm 'middle'"):
        arm_spec("dual-openyam", "middle")
    with pytest.raises(ValueError, match="unknown robot"):
        arm_spec("hexapod")


def test_spec_rejects_inconsistent_fields(left_spec: ArmCalibrationSpec) -> None:
    with pytest.raises(ValueError, match="measured joints"):
        replace(left_spec, measured_joints=["right_joint2"])
    with pytest.raises(ValueError, match="home_joints"):
        replace(left_spec, home_joints={})
    with pytest.raises(ValueError, match="empty"):
        replace(left_spec, joint_ranges={**left_spec.joint_ranges, "left_joint2": (1.0, 0.0)})
