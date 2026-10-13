# Copyright 2025-2026 Dimensional Inc.
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

"""Tests that the presets are a shortcut and nothing more.

The first three tests are the important ones. ``conftest.py`` writes out three
descriptions by hand -- an arm, a humanoid body, a base -- and each test
rebuilds one through its preset and checks they match exactly. The arm and
body are rebuilt from small robot models that say the same things.

The rest cover what each preset decides: which joints an arm or body takes
from its robot model and with what limits, that a gripper and an orientation
sensor are added as asked, and that a base is one part whose speed limits
clamp.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
import pickle

import pytest

from dimos.control.contract.conftest import ARM_JOINTS, G1_JOINTS
from dimos.control.contract.description import ControlDescription, Limits, ResourceKind
from dimos.control.contract.keys import Interface, Key, Unit
from dimos.control.contract.presets import (
    GripperSpec,
    imu_resource,
    manipulator_description,
    pd_joint_description,
    twist_base_description,
)
from dimos.control.contract.validate import (
    CommandBatch,
    DescriptionError,
    Rejected,
    validate_command,
    validate_description,
)
from dimos.msgs.control_msgs.ControlValues import ControlValues
from dimos.robot.assets.model import RobotModel

CHASSIS_LIMITS = {
    Key.of("chassis", "base", Interface.VX): Limits(-1.5, 1.5),
    Key.of("chassis", "base", Interface.VY): Limits(-1.0, 1.0),
    Key.of("chassis", "base", Interface.WZ): Limits(-2.0, 2.0),
}


def model_from(tmp_path: Path, urdf: str, name: str = "robot") -> RobotModel:
    """Write ``urdf`` to a file called ``name`` and load it as a robot model."""
    path = tmp_path / f"{name}.urdf"
    path.write_text(urdf)
    return RobotModel.from_file(path)


def chain_model(
    tmp_path: Path, name: str, joints: Sequence[str], limit: str, joint_type: str = "revolute"
) -> RobotModel:
    """A robot model whose joints follow one after another, all alike.

    Args:
        name: The robot's name, also used for its file.
        joints: The joint names, in order.
        limit: The attributes of every joint's ``<limit>`` tag, e.g.
            'lower="-1.0" upper="1.0" velocity="2.0" effort="3.0"'.
        joint_type: The URDF kind of every joint, e.g. "revolute" or "fixed".
    """
    links = "".join(f'<link name="link{i}"/>' for i in range(len(joints) + 1))
    chain = "".join(
        f'<joint name="{joint}" type="{joint_type}"><parent link="link{i}"/>'
        f'<child link="link{i + 1}"/><limit {limit}/></joint>'
        for i, joint in enumerate(joints)
    )
    return model_from(tmp_path, f'<robot name="{name}">{links}{chain}</robot>', name)


@pytest.fixture
def arm_model(tmp_path: Path) -> RobotModel:
    """The conftest arm's seven joints, with the limits written out there."""
    limit = 'lower="-3.14" upper="3.14" velocity="1.0" effort="50.0"'
    return chain_model(tmp_path, "arm", ARM_JOINTS, limit)


@pytest.fixture
def g1_model(tmp_path: Path) -> RobotModel:
    """The conftest humanoid's 29 joints, with the limits written out there."""
    limit = 'lower="-2.0" upper="2.0" velocity="32.0" effort="88.0"'
    return chain_model(tmp_path, "g1", G1_JOINTS, limit)


def preset_arm(model: RobotModel) -> ControlDescription:
    """The conftest arm, built through the preset."""
    return manipulator_description(
        model,
        source="arm",
        state=(Interface.POSITION, Interface.EFFORT),
        gripper=GripperSpec(unit=Unit.M, lo=0.0, hi=0.085),
    )


def preset_g1(model: RobotModel) -> ControlDescription:
    """The conftest humanoid, built through the preset."""
    return pd_joint_description(model, source="g1", sensors=[imu_resource()])


def preset_chassis() -> ControlDescription:
    """The conftest base, built through the preset."""
    return twist_base_description("chassis", limits=CHASSIS_LIMITS)


def command(values: dict[str, float]) -> ControlValues:
    return ControlValues(
        source="coordinator",
        epoch=0,
        sequence=1,
        interface_names=list(values),
        values=list(values.values()),
    )


def test_manipulator_preset_rebuilds_the_hand_written_arm(
    xarm: ControlDescription, arm_model: RobotModel
) -> None:
    assert preset_arm(arm_model) == xarm


def test_pd_joint_preset_rebuilds_the_hand_written_g1(
    g1: ControlDescription, g1_model: RobotModel
) -> None:
    assert preset_g1(g1_model) == g1


def test_twist_base_preset_rebuilds_the_hand_written_chassis(chassis: ControlDescription) -> None:
    assert preset_chassis() == chassis


def test_every_preset_output_passes_the_description_check(
    arm_model: RobotModel, g1_model: RobotModel
) -> None:
    for described in (preset_arm(arm_model), preset_g1(g1_model), preset_chassis()):
        validate_description(described)


def test_every_preset_output_survives_a_pickle(arm_model: RobotModel, g1_model: RobotModel) -> None:
    # Descriptions are sent between processes.
    for described in (preset_arm(arm_model), preset_g1(g1_model), preset_chassis()):
        assert pickle.loads(pickle.dumps(described)) == described


def test_an_arm_is_told_position_and_velocity_unless_it_says_otherwise(
    arm_model: RobotModel,
) -> None:
    both = manipulator_description(arm_model, source="arm")
    assert both.resource("joint1").command_interfaces == (
        Interface.POSITION,
        Interface.VELOCITY,
    )
    only = manipulator_description(arm_model, source="arm", command=(Interface.POSITION,))
    assert only.resource("joint1").command_interfaces == (Interface.POSITION,)
    assert "arm/joint1/velocity" not in only.limits


def test_an_override_for_a_name_the_arm_is_not_told_is_an_error(arm_model: RobotModel) -> None:
    overrides = {Key.of("arm", "joint99", Interface.POSITION): Limits(-1.0, 1.0)}
    with pytest.raises(DescriptionError, match="joint99"):
        manipulator_description(arm_model, source="arm", limit_overrides=overrides)


def test_an_arm_refuses_a_command_past_a_limit(arm_model: RobotModel) -> None:
    refused = validate_command(
        preset_arm(arm_model), command({"arm/joint1/position": 9.0}), last_sequence=None
    )
    assert isinstance(refused, Rejected) and refused.reason == "limit"


def test_a_gripper_keeps_its_own_unit_and_range(arm_model: RobotModel) -> None:
    described = manipulator_description(
        arm_model,
        source="arm",
        gripper=GripperSpec(name="hand", unit=Unit.NORMALIZED, lo=0.0, hi=1.0),
    )
    assert described.unit_of("arm/hand/position") is Unit.NORMALIZED
    assert described.limits["arm/hand/position"] == Limits(0.0, 1.0)
    assert "arm/hand" in described.joint_names()


def test_an_arm_with_no_joints_or_nothing_to_command_raises(arm_model: RobotModel) -> None:
    with pytest.raises(ValueError, match="no joints"):
        manipulator_description(arm_model, source="arm", joints=())
    with pytest.raises(ValueError, match="nothing to command"):
        manipulator_description(arm_model, source="arm", command=())


def test_an_interface_the_preset_has_no_unit_for_raises(arm_model: RobotModel) -> None:
    with pytest.raises(ValueError, match="no preset unit"):
        manipulator_description(arm_model, source="arm", state=(Interface.POSITION, Interface.VX))


def test_a_body_is_told_stiffness_and_damping_with_every_target(g1_model: RobotModel) -> None:
    described = pd_joint_description(g1_model, source="g1")
    assert described.resource("joint1").command_interfaces == (
        Interface.POSITION,
        Interface.VELOCITY,
        Interface.EFFORT,
        Interface.KP,
        Interface.KD,
    )
    # No orientation sensor unless asked for.
    assert [r.kind for r in described.resources] == [ResourceKind.JOINT] * len(G1_JOINTS)


def test_an_orientation_sensor_only_reports(g1_model: RobotModel) -> None:
    described = pd_joint_description(g1_model, source="g1", sensors=[imu_resource("chest_imu")])
    imu = described.resource("chest_imu")
    assert imu.kind is ResourceKind.SENSOR and imu.command_interfaces == ()
    assert "g1/chest_imu/qw" in described.state_keys()
    assert "g1/chest_imu" not in described.joint_names()


def test_a_body_whose_model_has_nothing_that_moves_raises(tmp_path: Path) -> None:
    model = chain_model(tmp_path, "statue", ["neck"], "", joint_type="fixed")
    with pytest.raises(ValueError, match="no joints"):
        pd_joint_description(model, source="statue")


def test_a_base_is_one_part_told_three_speeds() -> None:
    described = preset_chassis()
    [base] = described.resources
    assert base.name == "base" and base.kind is ResourceKind.BASE
    assert base.command_interfaces == (Interface.VX, Interface.VY, Interface.WZ)
    assert described.joint_names() == ()


def test_a_base_clamps_every_limit_it_is_given() -> None:
    described = preset_chassis()
    assert set(described.limits) == set(CHASSIS_LIMITS)
    assert all(limit.clamp for limit in described.limits.values())


def test_a_base_told_to_go_too_fast_goes_at_its_top_speed() -> None:
    accepted = validate_command(
        preset_chassis(),
        command({"chassis/base/vx": 9.0, "chassis/base/vy": 0.0, "chassis/base/wz": -9.0}),
        last_sequence=None,
    )
    assert accepted == CommandBatch(
        values={"chassis/base/vx": 1.5, "chassis/base/vy": 0.0, "chassis/base/wz": -2.0},
        clamped=("chassis/base/vx", "chassis/base/wz"),
    )


def test_a_base_that_cannot_move_sideways_limits_that_speed_to_zero() -> None:
    limits = CHASSIS_LIMITS | {Key.of("chassis", "base", Interface.VY): Limits(0.0, 0.0)}
    described = twist_base_description("chassis", limits=limits)
    accepted = validate_command(described, command({"chassis/base/vy": 0.5}), last_sequence=None)
    assert accepted == CommandBatch(values={"chassis/base/vy": 0.0}, clamped=("chassis/base/vy",))


def test_a_base_limit_needs_both_bounds() -> None:
    with pytest.raises(DescriptionError, match="not bounded on both sides"):
        twist_base_description(
            "chassis", limits={Key.of("chassis", "base", Interface.VX): Limits(hi=1.5)}
        )


def test_a_base_reports_only_what_it_can() -> None:
    no_pose = twist_base_description("chassis", limits={}, odometry=False)
    assert no_pose.resources[0].state_interfaces == (Interface.VX, Interface.VY, Interface.WZ)
    # One that can only repeat back what it was told does not claim to measure.
    no_speed = twist_base_description("chassis", limits={}, measured_velocity=False)
    assert no_speed.resources[0].state_interfaces == ("x", "y", "yaw")


def test_any_preset_can_carry_an_orientation_sensor(arm_model: RobotModel) -> None:
    arm = manipulator_description(arm_model, source="arm", sensors=[imu_resource()])
    base = twist_base_description("chassis", limits=CHASSIS_LIMITS, sensors=[imu_resource()])
    assert "arm/imu/qw" in arm.state_keys()
    assert "chassis/imu/qw" in base.state_keys()


# A small robot model covering each kind of joint a preset has to handle,
# because no real robot has all of them at once.
TOY_URDF = """<?xml version="1.0"?>
<robot name="toy">
  <link name="base"/>
  <link name="l1"/>
  <link name="l2"/>
  <link name="l3"/>
  <link name="l4"/>
  <joint name="shoulder" type="revolute">
    <parent link="base"/>
    <child link="l1"/>
    <limit lower="-1.5" upper="2.5" velocity="3.0" effort="40.0"/>
  </joint>
  <joint name="spinner" type="continuous">
    <parent link="l1"/>
    <child link="l2"/>
    <limit velocity="10.0" effort="5.0"/>
  </joint>
  <joint name="mount" type="fixed">
    <parent link="l2"/>
    <child link="l3"/>
  </joint>
  <joint name="slider" type="prismatic">
    <parent link="l3"/>
    <child link="l4"/>
    <limit lower="0.0" upper="0.4" velocity="0.2" effort="100.0"/>
  </joint>
</robot>
"""


def test_by_default_every_joint_that_moves_is_driven_in_model_order(tmp_path: Path) -> None:
    arm = manipulator_description(model_from(tmp_path, TOY_URDF), source="arm")
    # The fixed mount is skipped.
    assert arm.joint_names() == ("arm/shoulder", "arm/spinner", "arm/slider")


def test_joints_picks_and_reorders_the_models_joints(tmp_path: Path) -> None:
    arm = manipulator_description(
        model_from(tmp_path, TOY_URDF), source="arm", joints=["slider", "shoulder"]
    )
    assert arm.joint_names() == ("arm/slider", "arm/shoulder")


def test_an_arm_takes_its_limits_from_its_model(tmp_path: Path) -> None:
    arm = manipulator_description(model_from(tmp_path, TOY_URDF), source="arm", joints=["shoulder"])
    assert arm.limits == {
        "arm/shoulder/position": Limits(-1.5, 2.5),
        "arm/shoulder/velocity": Limits(-3.0, 3.0),
    }
    # The arm is not told an effort, so the model's effort limit has no place.
    assert "arm/shoulder/effort" not in arm.limits


def test_a_body_takes_its_effort_limits_from_its_model(tmp_path: Path) -> None:
    body = pd_joint_description(model_from(tmp_path, TOY_URDF), source="toy", joints=["shoulder"])
    assert body.limits["toy/shoulder/effort"] == Limits(-40.0, 40.0)
    assert "toy/shoulder/kp" not in body.limits


def test_a_prismatic_joint_is_measured_in_metres(tmp_path: Path) -> None:
    arm = manipulator_description(model_from(tmp_path, TOY_URDF), source="arm", joints=["slider"])
    assert arm.unit_of("arm/slider/position") is Unit.M
    assert arm.unit_of("arm/slider/velocity") is Unit.M_PER_S
    assert arm.unit_of("arm/slider/effort") is Unit.N


def test_a_joint_that_spins_freely_gets_no_position_limit(tmp_path: Path) -> None:
    arm = manipulator_description(model_from(tmp_path, TOY_URDF), source="arm", joints=["spinner"])
    assert "arm/spinner/position" not in arm.limits
    assert arm.limits["arm/spinner/velocity"] == Limits(-10.0, 10.0)


def test_limit_overrides_replace_only_their_own_names(tmp_path: Path) -> None:
    slower = {"arm/shoulder/velocity": Limits(-0.5, 0.5)}
    arm = manipulator_description(
        model_from(tmp_path, TOY_URDF), source="arm", limit_overrides=slower
    )
    assert arm.limits["arm/shoulder/velocity"] == Limits(-0.5, 0.5)
    assert arm.limits["arm/shoulder/position"] == Limits(-1.5, 2.5)
    assert arm.limits["arm/spinner/velocity"] == Limits(-10.0, 10.0)


def test_a_joint_missing_from_the_model_raises(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="not in the robot model"):
        manipulator_description(model_from(tmp_path, TOY_URDF), source="arm", joints=["elbow"])


def test_a_fixed_joint_cannot_be_driven(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="cannot be driven"):
        manipulator_description(model_from(tmp_path, TOY_URDF), source="arm", joints=["mount"])


@pytest.mark.parametrize(
    ("broken", "match"),
    [
        ('lower="-1.5" upper="2.5"', "no position range"),
        ('upper="2.5"', "half a position range"),
        ('velocity="3.0"', "no velocity limit"),
    ],
)
def test_a_model_unclear_about_a_limit_raises(tmp_path: Path, broken: str, match: str) -> None:
    # Reading a missing bound as "no limit" would let the joint go anywhere.
    model = model_from(tmp_path, TOY_URDF.replace(broken, "", 1))
    with pytest.raises(ValueError, match=match):
        manipulator_description(model, source="arm", joints=["shoulder"])


# An arm whose model includes its gripper's two fingers, as many arm URDFs do.
# The right finger copies the left one (a "mimic" joint).
ARM_WITH_FINGERS_URDF = """<?xml version="1.0"?>
<robot name="arm_with_fingers">
  <link name="base"/>
  <link name="upper"/>
  <link name="hand"/>
  <link name="left"/>
  <link name="right"/>
  <joint name="shoulder" type="revolute">
    <parent link="base"/>
    <child link="upper"/>
    <limit lower="-1.5" upper="1.5" velocity="2.0" effort="40.0"/>
  </joint>
  <joint name="wrist" type="revolute">
    <parent link="upper"/>
    <child link="hand"/>
    <limit lower="-2.0" upper="2.0" velocity="2.0" effort="20.0"/>
  </joint>
  <joint name="left_finger" type="prismatic">
    <parent link="hand"/>
    <child link="left"/>
    <limit lower="0.0" upper="0.04" velocity="0.1" effort="10.0"/>
  </joint>
  <joint name="right_finger" type="prismatic">
    <parent link="hand"/>
    <child link="right"/>
    <limit lower="0.0" upper="0.04" velocity="0.1" effort="10.0"/>
    <mimic joint="left_finger"/>
  </joint>
</robot>
"""


def test_an_arm_whose_model_has_fingers_names_its_joints(tmp_path: Path) -> None:
    model = model_from(tmp_path, ARM_WITH_FINGERS_URDF)
    # By default each finger would be driven as one more arm joint.
    every = manipulator_description(model, source="arm")
    assert "arm/left_finger" in every.joint_names()
    assert "arm/right_finger" in every.joint_names()
    # Naming the arm's joints leaves the fingers to a gripper with one opening.
    arm = manipulator_description(
        model,
        source="arm",
        joints=["shoulder", "wrist"],
        gripper=GripperSpec(unit=Unit.M, lo=0.0, hi=0.08),
    )
    assert arm.joint_names() == ("arm/shoulder", "arm/wrist", "arm/gripper")


#: The G1 shipped in this repo, to check the presets against a model nobody
#: here wrote. Its expected numbers are typed out rather than read back from
#: the same file, so a change to the model fails here instead of agreeing.
G1_URDF = Path(__file__).resolve().parents[2] / "robot" / "unitree" / "g1" / "g1.urdf"
G1_LEG = ("left_hip_pitch", "left_hip_roll", "left_knee")


def g1_leg_model() -> RobotModel:
    return RobotModel.from_file(G1_URDF).with_renamed_joints({f"{j}_joint": j for j in G1_LEG})


def test_the_in_repo_g1_drives_every_joint_that_moves() -> None:
    body = pd_joint_description(RobotModel.from_file(G1_URDF), source="g1")
    # 29 motors; the floating base and the fixed sensor mounts are skipped.
    assert len(body.joint_names()) == 29
    assert body.joint_names()[0] == "g1/left_hip_pitch_joint"
    assert "g1/floating_base_joint" not in body.joint_names()


def test_the_in_repo_g1_gets_its_limits_from_its_model() -> None:
    body = pd_joint_description(g1_leg_model(), source="g1", joints=G1_LEG)
    assert body.limits["g1/left_hip_pitch/position"] == Limits(-2.5307, 2.8798)
    assert body.limits["g1/left_hip_pitch/velocity"] == Limits(-32.0, 32.0)
    assert body.limits["g1/left_hip_pitch/effort"] == Limits(-88.0, 88.0)
    # An asymmetric range, so nothing here is quietly symmetrizing position.
    assert body.limits["g1/left_knee/position"] == Limits(-0.087267, 2.8798)
    assert body.limits["g1/left_knee/effort"] == Limits(-139.0, 139.0)


def test_the_in_repo_g1_refuses_a_command_past_its_model_limit() -> None:
    body = pd_joint_description(g1_leg_model(), source="g1", joints=G1_LEG)
    assert not isinstance(
        validate_command(body, command({"g1/left_knee/position": 1.0}), last_sequence=None),
        Rejected,
    )
    refused = validate_command(body, command({"g1/left_knee/position": 3.0}), last_sequence=None)
    assert isinstance(refused, Rejected) and refused.reason == "limit"
