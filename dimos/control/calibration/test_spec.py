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
from pathlib import Path
import re

import numpy as np
import pytest
from pytest_mock import MockerFixture

import dimos
from dimos.control.calibration import spec as spec_module
from dimos.control.calibration._calibration_test_helpers import (
    require_model,
    robot_specs_or_skip,
)
from dimos.control.calibration.gravity_poses import (
    ArmModel,
    candidate_poses,
    generate_poses,
    grid_axes,
)
from dimos.control.calibration.robots import (
    KNOWN_ROBOTS,
    arm_spec,
    robot_id,
    robot_specs,
)
from dimos.control.calibration.spec import (
    DAMIAO_STOCK_TORQUE_LIMIT_NM,
    DEFAULT_TORQUE_SAFETY_FACTOR,
    ArmCalibrationSpec,
    NoWorkspaceGuardError,
    WorkspaceGuard,
    build_arm_spec,
    merged_links,
    model_sha256,
    specs_from_damiao_robot,
    urdf_limits,
)
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.damiao.robot import DamiaoRobot
from dimos.robot.manipulators.dual_makermods_metal.config import DUAL_METAL
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM
from dimos.robot.manipulators.makermods_metal.config import METAL
from dimos.robot.manipulators.openarm.config import OPENARM
from dimos.robot.manipulators.openyam.config import OPENYAM
from dimos.robot.manipulators.seeed_b601_dm.config import B601_DM

DM4340 = DAMIAO_STOCK_TORQUE_LIMIT_NM["DM4340"] * DEFAULT_TORQUE_SAFETY_FACTOR
DM4310 = DAMIAO_STOCK_TORQUE_LIMIT_NM["DM4310"] * DEFAULT_TORQUE_SAFETY_FACTOR


def assert_ranges_inside_urdf_limits(spec: ArmCalibrationSpec) -> None:
    joints = {joint.name: joint for joint in spec.model.load().joints}
    for name, (low, high) in spec.joint_ranges.items():
        lower, upper = urdf_limits(joints[name])
        assert lower <= low <= high <= upper, name


def half_base_spacing_m(left: ArmCalibrationSpec, right: ArmCalibrationSpec) -> float:
    """Half the distance between two arms' base frames in their model, at neutral."""
    return float(np.linalg.norm(ArmModel(left).origin - ArmModel(right).origin)) / 2.0


def test_dual_openyam_specs_on_a_bench(dual_openyam_specs: list[ArmCalibrationSpec]) -> None:
    left, right = dual_openyam_specs
    assert [left.arm_id, right.arm_id] == ["dual_openyam/left", "dual_openyam/right"]
    half_spacing = half_base_spacing_m(left, right)
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
        assert spec.home_joints[f"{side}_joint2"] == pytest.approx(1.047)
        assert spec.gains[f"{side}_joint2"] == (80.0, 1.0)
        assert_ranges_inside_urdf_limits(spec)
        guard = spec.workspace
        assert guard.explicit
        assert guard.floor_z_m == -0.03 and guard.min_x_m == 0.15
        assert guard.min_pose_z_m == pytest.approx(0.12)
        assert guard.min_transition_z_m == pytest.approx(0.08)
        assert guard.point_offset_m == (0.0, 0.0, 0.0)
        # The only keep-out: the midline halfway between the URDF's bases.
        (midline,) = guard.keep_out
        other = right if spec is left else left
        assert midline.name == f"midline to {other.arm_id}"
        assert midline.normal == pytest.approx((0.0, -1.0 if side == "left" else 1.0, 0.0))
        assert midline.offset_m == pytest.approx(half_spacing)
        default = arm_spec("dual-openyam", side)
        assert default.on_bench(-0.03, 0.15) == spec
        assert replace(spec, workspace=default.workspace) == default
        assert spec.grid_step_rad is None
        # Every frame the old filter checked is still checked, plus the elbow and
        # the grasp frame; the shoulder, which cannot move, is not.
        assert guard.clearance_frames is not None
        for suffix in ("tip_left", "tip_right", "gripper", "link5", "link4", "link3"):
            assert f"{side}_{suffix}" in guard.clearance_frames
        assert f"{side}_link2" not in guard.clearance_frames
    assert left.model_sha256() == right.model_sha256()


def test_dual_openyam_default_spec_derives_everything_from_the_model() -> None:
    require_model(DUAL_OPENYAM.gravity_model, "Dual OpenYAM")
    left, right = specs_from_damiao_robot(DUAL_OPENYAM)
    assert [left.arm_id, right.arm_id] == DUAL_OPENYAM.arm_ids()
    for spec, toward_other in ((left, (0.0, -1.0, 0.0)), (right, (0.0, 1.0, 0.0))):
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
        # The implicit guard: the model root's z = 0 plane (the bases' level here)
        # and the midline between the URDF's bases, nothing in front or behind.
        assert not spec.workspace.explicit
        assert spec.workspace.floor_z_m == pytest.approx(0.0)
        assert spec.workspace.min_x_m is None and spec.grid_step_rad is None
        (midline,) = spec.workspace.keep_out
        assert midline.normal == pytest.approx(toward_other)
        assert midline.offset_m == pytest.approx(half_base_spacing_m(left, right))


def test_openyam_spec() -> None:
    (spec,) = robot_specs_or_skip(OPENYAM, "OpenYAM (yam_description LFS archive)")
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


def test_metal_spec_does_not_take_torque_caps_from_its_profiles_mit_range(
    mocker: MockerFixture,
) -> None:
    # The custom profiles' t_max (30, 120 and 20 Nm) only scale the MIT frames;
    # the motors' ratings are unknown, so the caps fall back to the smallest
    # known rating, with a warning, unless given.
    warning = mocker.spy(spec_module.logger, "warning")
    (spec,) = robot_specs_or_skip(METAL, "MakerMods Metal")
    assert all(motor.profile is not None for motor in METAL.buses[0].arm)
    smallest = min(DAMIAO_STOCK_TORQUE_LIMIT_NM.values()) * DEFAULT_TORQUE_SAFETY_FACTOR
    assert spec.max_torque_nm == pytest.approx(dict.fromkeys(spec.joint_names, smallest))
    (message,) = [call.args[0] for call in warning.call_args_list]
    assert message.startswith("metal: no torque rating is known for ['joint1'")
    assert "Pass max_torque_nm" in message
    warning.reset_mock()
    (given,) = robot_specs_or_skip(
        METAL, "MakerMods Metal", max_torque_nm={"joint2": 12.0, "joint3": 8.0}
    )
    assert given.max_torque_nm["joint2"] == 12.0 and given.max_torque_nm["joint3"] == 8.0
    assert given.max_torque_nm["joint1"] == pytest.approx(smallest)
    assert "['joint1', 'joint4', 'joint5', 'joint6']" in warning.call_args.args[0]
    assert spec.tip_frame == "gripper_base"
    assert "joint1" not in spec.measured_joints
    assert_ranges_inside_urdf_limits(spec)


def test_openarm_keeps_its_horizontal_shoulder_joint() -> None:
    left, right = robot_specs_or_skip(OPENARM, "OpenArm")
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
    checkout = f"{Path(dimos.__file__).resolve().parents[1]}/"
    assert checkout in xml  # the meshes are under this checkout
    here = RobotModel.from_urdf_string(xml)
    elsewhere = RobotModel.from_urdf_string(xml.replace(checkout, "/srv/other/checkout/"))
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


# Arm ids: one source of truth, the robot's name


@pytest.mark.parametrize("key", sorted(KNOWN_ROBOTS))
def test_spec_arm_ids_are_the_robots_arm_ids(key: str) -> None:
    robot = KNOWN_ROBOTS[key]
    require_model(robot.gravity_model, key)
    ids = robot.arm_ids()
    assert ids and ids[0].split("/")[0] == robot.name == robot_id(key)
    assert [spec.arm_id for spec in specs_from_damiao_robot(robot)] == ids
    assert [spec.arm_id for spec in robot_specs(key)] == ids


def test_an_unnamed_robot_needs_an_arm_id(b601_spec: ArmCalibrationSpec) -> None:
    unnamed: DamiaoRobot = replace(B601_DM, name="")
    with pytest.raises(ValueError, match="no name"):
        specs_from_damiao_robot(unnamed)
    (spec,) = specs_from_damiao_robot(unnamed, arm_id="bench_b601")
    assert spec.arm_id == "bench_b601"


# Workspace guards: nothing moves without an explicit one


@pytest.mark.parametrize(
    ("robot", "what"), [(OPENYAM, "OpenYAM"), (METAL, "MakerMods Metal"), (OPENARM, "OpenArm")]
)
def test_poses_are_not_generated_without_an_explicit_guard(robot: DamiaoRobot, what: str) -> None:
    for spec in robot_specs_or_skip(robot, what):
        assert not spec.workspace.explicit
        expected = (
            f"no workspace guard defined for {spec.arm_id}: pass floor height and front limit"
        )
        with pytest.raises(NoWorkspaceGuardError, match=re.escape(expected)):
            spec.require_workspace_guard()
        with pytest.raises(NoWorkspaceGuardError, match=re.escape(expected)):
            generate_poses(spec)


def test_a_bench_guard_is_explicit_and_a_spec_without_one_is_not(
    left_spec: ArmCalibrationSpec,
) -> None:
    bench = WorkspaceGuard.for_bench(-0.03, 0.15, pose_clearance_m=0.2)
    assert bench.explicit and bench.floor_z_m == -0.03 and bench.min_x_m == 0.15
    assert bench.min_pose_z_m == pytest.approx(0.17) and bench.keep_out == ()
    assert WorkspaceGuard().explicit
    fields = {name: getattr(left_spec, name) for name in left_spec.__dataclass_fields__}
    del fields["workspace"]
    assert not ArmCalibrationSpec(**fields).workspace.explicit


@pytest.mark.parametrize(
    ("robot", "what", "floor_z_m", "half_spacing_m"),
    [
        (DUAL_METAL, "Dual MakerMods Metal", 0.0, 0.30),
        (OPENARM, "OpenArm", -0.6, 0.031),
    ],
)
def test_two_arm_robots_keep_each_arm_out_of_the_others_half(
    robot: DamiaoRobot, what: str, floor_z_m: float, half_spacing_m: float
) -> None:
    implicit = robot_specs_or_skip(robot, what)
    bench = WorkspaceGuard.for_bench(floor_z_m, 0.10)
    specs = robot_specs_or_skip(robot, what, workspace=bench)
    for spec, default, other, toward_y in zip(
        specs, implicit, reversed(specs), (-1.0, 1.0), strict=True
    ):
        # Added to the implicit guard and to an explicit one alike.
        (midline,) = spec.workspace.keep_out
        assert default.workspace.keep_out == (midline,)
        assert midline.name == f"midline to {other.arm_id}"
        assert midline.normal == pytest.approx((0.0, toward_y, 0.0))
        assert midline.offset_m == pytest.approx(half_spacing_m, abs=1e-3)
        arm = ArmModel(spec)
        pose_set = generate_poses(arm, count=6)
        assert pose_set.identification
        for pose in [*pose_set.identification, *(g.pose for g in pose_set.gravity_free.values())]:
            assert not midline.violated_by(np.vstack(arm.points(arm.q(pose.joints_rad))))
    if robot is OPENARM:
        # Without the midline some grid poses reach into the other arm's half.
        left = specs[0]
        free = ArmModel(replace(left, workspace=replace(left.workspace, keep_out=())))
        crossing = [
            pose
            for pose in candidate_poses(free, grid_axes(free, 6)[0])
            if left.workspace.keep_out[0].violated_by(np.vstack(free.points(free.q(pose))))
        ]
        assert crossing
