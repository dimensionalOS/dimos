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

"""Tests for ``control_description``: a config and some ports in, one
description out."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
import pickle

import pytest

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.description import ControlDescription, Limits, ResourceKind
from dimos.control.contract.keys import Interface, Unit
from dimos.control.contract.ports import (
    JointPort,
    MotorPort,
    TwistPort,
    control_description,
    port_specs,
    reading_ports,
)
from dimos.control.contract.presets import GripperSpec
from dimos.control.contract.validate import DescriptionError, validate_description
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.assets.model import RobotModel

JOINTS = ("j1", "j2", "j3")
LIMIT = 'lower="-1.0" upper="1.0" velocity="2.0" effort="3.0"'
ALL_STATE = (Interface.POSITION, Interface.VELOCITY, Interface.EFFORT)
RANGE = Twist(linear=Vector3(1.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 2.0))


def chain_model(tmp_path: Path, joints: Sequence[str] = JOINTS, limit: str = LIMIT) -> RobotModel:
    """A robot model whose joints turn one after another, all alike."""
    links = "".join(f'<link name="link{i}"/>' for i in range(len(joints) + 1))
    chain = "".join(
        f'<joint name="{joint}" type="revolute"><parent link="link{i}"/>'
        f'<child link="link{i + 1}"/><limit {limit}/></joint>'
        for i, joint in enumerate(joints)
    )
    path = tmp_path / "robot.urdf"
    path.write_text(f'<robot name="robot">{links}{chain}</robot>')
    return RobotModel.from_file(path)


@pytest.fixture
def model(tmp_path: Path) -> RobotModel:
    return chain_model(tmp_path)


class ArmConfig(ConnectionConfig):
    source: str = "arm"
    position_command: JointPort = JointPort()


class ArmWithVelocityConfig(ArmConfig):
    velocity_command: JointPort = JointPort(interface=Interface.VELOCITY, joints=["j1"])


class BodyConfig(ConnectionConfig):
    source: str = "g1"
    motor_command: MotorPort = MotorPort()


class BaseConfig(ConnectionConfig):
    source: str = "go2"
    cmd_vel: TwistPort = TwistPort(range=RANGE)


class ArmOnBaseConfig(ConnectionConfig):
    source: str = "r1"
    position_command: JointPort = JointPort(joints=["j3", "j1"])
    motor_command: MotorPort = MotorPort(joints=["j1", "j2"])
    cmd_vel: TwistPort = TwistPort(range=RANGE)


def test_a_joint_port_drives_every_moving_joint_of_the_model_in_its_order(
    model: RobotModel,
) -> None:
    described = control_description(
        ArmConfig(model=model), {"position_command": JointState}, {"joint_state": JointState}
    )
    assert described.source == "arm"
    assert described.joint_names() == ("arm/j1", "arm/j2", "arm/j3")
    for joint in described.resources:
        assert joint.kind is ResourceKind.JOINT
        assert joint.command_interfaces == (Interface.POSITION,)
        assert joint.state_interfaces == ALL_STATE
        assert joint.units[Interface.POSITION] is Unit.RAD
        assert joint.units[Interface.EFFORT] is Unit.NM
    assert described.limits == {f"arm/{j}/position": Limits(-1.0, 1.0) for j in JOINTS}


def test_readings_come_from_the_output_ports_alone(model: RobotModel) -> None:
    silent = control_description(ArmConfig(model=model), {"position_command": JointState}, {})
    assert all(joint.state_interfaces == () for joint in silent.resources)
    assert silent.state_keys() == ()
    # An output of another message changes nothing.
    other = control_description(
        ArmConfig(model=model), {"position_command": JointState}, {"image": str}
    )
    assert other == silent


def test_a_joint_on_two_ports_becomes_one_joint_that_takes_both(model: RobotModel) -> None:
    described = control_description(
        ArmWithVelocityConfig(model=model),
        {"position_command": JointState, "velocity_command": JointState},
        {},
    )
    assert described.joint_names() == ("arm/j1", "arm/j2", "arm/j3")
    j1 = described.resource("j1")
    j2 = described.resource("j2")
    assert j1 is not None and j2 is not None
    assert j1.command_interfaces == (Interface.POSITION, Interface.VELOCITY)
    assert j2.command_interfaces == (Interface.POSITION,)
    assert described.limits["arm/j1/velocity"] == Limits(-2.0, 2.0)
    assert "arm/j2/velocity" not in described.limits


def test_a_gripper_is_one_more_joint_with_its_own_unit_and_range(model: RobotModel) -> None:
    config = ArmConfig(
        model=model,
        position_command=JointPort(gripper=GripperSpec(unit=Unit.M, lo=0.0, hi=0.085)),
    )
    described = control_description(config, {"position_command": JointState}, {"js": JointState})
    gripper = described.resource("gripper")
    assert gripper is not None
    assert gripper.kind is ResourceKind.JOINT
    assert gripper.units == {Interface.POSITION: Unit.M}
    # It reports its opening only.
    assert gripper.state_interfaces == (Interface.POSITION,)
    assert described.limits["arm/gripper/position"] == Limits(0.0, 0.085)


def test_a_gripper_needs_a_position_port(model: RobotModel) -> None:
    config = ArmConfig(
        model=model,
        position_command=JointPort(
            interface=Interface.VELOCITY, gripper=GripperSpec(unit=Unit.M, lo=0.0, hi=0.1)
        ),
    )
    with pytest.raises(ValueError, match="gripper is told a position"):
        control_description(config, {"position_command": JointState}, {})


def test_a_joint_port_override_replaces_the_model_limit_for_that_joint(
    model: RobotModel,
) -> None:
    config = ArmConfig(
        model=model, position_command=JointPort(limit_overrides={"j2": Limits(-0.5, 0.5)})
    )
    described = control_description(config, {"position_command": JointState}, {})
    assert described.limits["arm/j2/position"] == Limits(-0.5, 0.5)
    assert described.limits["arm/j1/position"] == Limits(-1.0, 1.0)

    config = ArmConfig(model=model, position_command=JointPort(limit_overrides={"j9": Limits()}))
    with pytest.raises(ValueError, match="'j9', which is not on this port"):
        control_description(config, {"position_command": JointState}, {})


def test_a_motor_port_tells_each_joint_five_things(model: RobotModel) -> None:
    described = control_description(
        BodyConfig(model=model), {"motor_command": MotorCommandArray}, {}
    )
    for joint in described.resources:
        assert joint.command_interfaces == (
            Interface.POSITION,
            Interface.VELOCITY,
            Interface.EFFORT,
            Interface.KP,
            Interface.KD,
        )
    assert described.limits["g1/j1/position"] == Limits(-1.0, 1.0)
    assert described.limits["g1/j1/velocity"] == Limits(-2.0, 2.0)
    assert described.limits["g1/j1/effort"] == Limits(-3.0, 3.0)
    assert "g1/j1/kp" not in described.limits


def test_a_motor_port_override_names_the_joint_and_what_is_limited(model: RobotModel) -> None:
    config = BodyConfig(
        model=model, motor_command=MotorPort(limit_overrides={"j1/velocity": Limits(-0.1, 0.1)})
    )
    described = control_description(config, {"motor_command": MotorCommandArray}, {})
    assert described.limits["g1/j1/velocity"] == Limits(-0.1, 0.1)

    for bad in ("j1", "j9/velocity", "j1/speed"):
        config = BodyConfig(model=model, motor_command=MotorPort(limit_overrides={bad: Limits()}))
        with pytest.raises(ValueError, match="<joint>/<interface>"):
            control_description(config, {"motor_command": MotorCommandArray}, {})


def test_a_motor_ports_joints_come_first_in_its_order(model: RobotModel) -> None:
    described = control_description(
        ArmOnBaseConfig(model=model),
        {"position_command": JointState, "motor_command": MotorCommandArray, "cmd_vel": Twist},
        {},
    )
    assert described.joint_names() == ("r1/j1", "r1/j2", "r1/j3")
    j3 = described.resource("j3")
    assert j3 is not None and j3.command_interfaces == (Interface.POSITION,)


def test_a_twist_port_drives_one_base_with_clamped_symmetric_limits() -> None:
    described = control_description(BaseConfig(), {"cmd_vel": Twist}, {})
    [base] = described.resources
    assert base.name == "base" and base.kind is ResourceKind.BASE
    assert base.command_interfaces == (Interface.VX, Interface.VY, Interface.WZ)
    assert base.state_interfaces == ()
    assert base.units[Interface.VX] is Unit.M_PER_S
    assert described.limits == {
        "go2/base/vx": Limits(-1.5, 1.5, clamp=True),
        "go2/base/vy": Limits(0.0, 0.0, clamp=True),
        "go2/base/wz": Limits(-2.0, 2.0, clamp=True),
    }
    assert described.joint_names() == ()


def test_a_twist_range_must_be_zero_or_more() -> None:
    config = BaseConfig(cmd_vel=TwistPort(range=Twist(linear=Vector3(-1.0, 0.0, 0.0))))
    with pytest.raises(ValueError, match="range for vx must be 0 or more"):
        control_description(config, {"cmd_vel": Twist}, {})


def test_pose_twist_and_imu_outputs_add_what_the_base_and_sensor_report() -> None:
    described = control_description(
        BaseConfig(),
        {"cmd_vel": Twist},
        {"odom": PoseStamped, "speed": TwistStamped, "imu": Imu},
    )
    base = described.resource("base")
    imu = described.resource("imu")
    assert base is not None and imu is not None
    assert base.state_interfaces == (
        Interface.VX,
        Interface.VY,
        Interface.WZ,
        Interface.X,
        Interface.Y,
        Interface.YAW,
    )
    assert base.units[Interface.YAW] is Unit.RAD
    assert imu.kind is ResourceKind.SENSOR and imu.command_interfaces == ()
    assert "go2/imu/qw" in described.state_keys()


def test_a_base_may_only_report() -> None:
    described = control_description(
        ConnectionConfig(source="cart"), {}, {"odom": PoseStamped, "speed": TwistStamped}
    )
    [base] = described.resources
    assert base.command_interfaces == ()
    assert len(base.state_interfaces) == 6
    assert described.limits == {}


def test_rates_come_from_the_config() -> None:
    described = control_description(
        BaseConfig(state_rate_hz=50.0, deadman_timeout_s=0.2), {"cmd_vel": Twist}, {}
    )
    assert (described.state_rate_hz, described.deadman_timeout_s) == (50.0, 0.2)
    with pytest.raises(DescriptionError, match="deadman_timeout_s"):
        control_description(BaseConfig(deadman_timeout_s=0.0), {"cmd_vel": Twist}, {})


@pytest.mark.parametrize(
    ("config", "inputs", "outputs", "match"),
    [
        (ArmConfig(), {"position_command": JointState}, {}, "needs the robot model"),
        (BaseConfig(), {}, {}, "no input port called 'cmd_vel'"),
        (BaseConfig(), {"cmd_vel": JointState}, {}, "'cmd_vel' must carry Twist, not JointState"),
        (BaseConfig(), {"cmd_vel": Twist}, {"a": Imu, "b": Imu}, "'a' and 'b' both report Imu"),
        (BaseConfig(), {"cmd_vel": Twist}, {"js": JointState}, "'js' reports joints, but no"),
        (ConnectionConfig(source="x"), {}, {}, "no command ports and reports nothing"),
    ],
)
def test_a_config_that_does_not_match_its_ports_is_refused(
    config: ConnectionConfig, inputs: dict[str, type], outputs: dict[str, type], match: str
) -> None:
    with pytest.raises(ValueError, match=match):
        control_description(config, inputs, outputs)


def test_two_bases_or_two_motor_ports_are_refused(model: RobotModel) -> None:
    class TwoBases(BaseConfig):
        other: TwistPort = TwistPort(range=RANGE)

    with pytest.raises(ValueError, match="'cmd_vel' and 'other' both drive the base"):
        control_description(TwoBases(), {"cmd_vel": Twist, "other": Twist}, {})

    class TwoMotors(BodyConfig):
        other: MotorPort = MotorPort()

    with pytest.raises(ValueError, match="'motor_command' and 'other' are both motor ports"):
        control_description(
            TwoMotors(model=model),
            {"motor_command": MotorCommandArray, "other": MotorCommandArray},
            {},
        )


def test_the_result_is_checked_and_crosses_processes(model: RobotModel) -> None:
    described = control_description(
        ArmOnBaseConfig(model=model),
        {"position_command": JointState, "motor_command": MotorCommandArray, "cmd_vel": Twist},
        {"joint_state": JointState, "odom": PoseStamped, "imu": Imu},
    )
    validate_description(described)
    assert isinstance(described, ControlDescription)
    assert pickle.loads(pickle.dumps(described)) == described


def test_port_specs_are_the_spec_fields_in_declaration_order(model: RobotModel) -> None:
    assert list(port_specs(ArmOnBaseConfig(model=model))) == [
        "position_command",
        "motor_command",
        "cmd_vel",
    ]
    assert port_specs(ConnectionConfig(source="x")) == {}


def test_reading_ports_find_each_reading_by_message_under_any_name() -> None:
    found = reading_ports({"anything": JointState, "where": PoseStamped, "image": str})
    assert found == {JointState: "anything", PoseStamped: "where"}
    with pytest.raises(ValueError, match="both report TwistStamped"):
        reading_ports({"a": TwistStamped, "b": TwistStamped})
