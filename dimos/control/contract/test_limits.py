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

"""The limits table a connection module's config turns into.

Small robot models are written as URDF text and loaded, so each test says
exactly what the model holds; the in-repo G1 at the end checks the table
against a model nobody here wrote.
"""

from __future__ import annotations

from collections.abc import Sequence
import math
from pathlib import Path
import pickle
from typing import Any

import pytest

from dimos.control.connection.connection_module import ConnectionConfig
from dimos.control.contract.keys import Interface
from dimos.control.contract.limits import (
    PD_INTERFACES,
    Limits,
    limits,
    model_limit,
    movable_joints,
)
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import JointDescription, RobotModel

JOINTS = ("j1", "j2", "j3")
LIMIT = 'lower="-1.0" upper="1.0" velocity="2.0" effort="3.0"'
RANGE = Twist(linear=Vector3(1.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 2.0))


def model_from(tmp_path: Path, urdf: str, name: str = "robot") -> RobotModel:
    """Write ``urdf`` to a file called ``name`` and load it as a robot model."""
    path = tmp_path / f"{name}.urdf"
    path.write_text(urdf)
    return RobotModel.from_file(path)


def chain_urdf(name: str, joints: Sequence[tuple[str, str, str]]) -> str:
    """A robot whose joints follow one after another, as URDF text.

    Args:
        name: The robot's name.
        joints: One ``(name, type, limit)`` per joint, in order: its URDF
            kind, e.g. "revolute", and the attributes of its ``<limit>`` tag,
            e.g. 'lower="-1.0" upper="1.0" velocity="2.0" effort="3.0"'.
    """
    links = "".join(f'<link name="link{i}"/>' for i in range(len(joints) + 1))
    chain = "".join(
        f'<joint name="{joint}" type="{joint_type}"><parent link="link{i}"/>'
        f'<child link="link{i + 1}"/><limit {limit}/></joint>'
        for i, (joint, joint_type, limit) in enumerate(joints)
    )
    return f'<robot name="{name}">{links}{chain}</robot>'


def chain_model(
    tmp_path: Path, name: str, joints: Sequence[str], limit: str, joint_type: str = "revolute"
) -> RobotModel:
    """A robot model whose joints follow one after another, all alike."""
    urdf = chain_urdf(name, [(joint, joint_type, limit) for joint in joints])
    return model_from(tmp_path, urdf, name)


def config(**fields: Any) -> ConnectionConfig:
    """A config for a robot called "arm"."""
    return ConnectionConfig(source="arm", **fields)


@pytest.fixture
def model(tmp_path: Path) -> RobotModel:
    """Three turning joints, all alike."""
    return chain_model(tmp_path, "robot", JOINTS, LIMIT)


# A small robot model covering each kind of joint, because no real robot has
# all of them at once.
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


def test_a_joint_command_gives_every_moving_joint_a_key_in_model_order(
    model: RobotModel,
) -> None:
    table = limits(config(model=model, joint_commands=(Interface.POSITION,)))
    assert table == {f"arm/{j}/position": Limits(-1.0, 1.0) for j in JOINTS}
    assert list(table) == ["arm/j1/position", "arm/j2/position", "arm/j3/position"]


def test_each_joint_command_takes_its_own_bound_from_the_model(model: RobotModel) -> None:
    table = limits(config(model=model, joint_commands=(Interface.POSITION, Interface.VELOCITY)))
    assert table["arm/j1/position"] == Limits(-1.0, 1.0)
    assert table["arm/j1/velocity"] == Limits(-2.0, 2.0)
    assert len(table) == 6
    assert "arm/j1/effort" not in table


def test_pd_tells_each_joint_five_things_with_gains_any_size_from_zero(
    model: RobotModel,
) -> None:
    table = limits(config(model=model, pd=True))
    assert [key for key in table if key.startswith("arm/j1/")] == [
        f"arm/j1/{interface}" for interface in PD_INTERFACES
    ]
    assert table["arm/j1/effort"] == Limits(-3.0, 3.0)
    assert table["arm/j1/kp"] == table["arm/j1/kd"] == Limits(0.0, None)
    assert len(table) == 15


def test_pd_and_a_joint_command_share_their_keys(model: RobotModel) -> None:
    both = limits(config(model=model, pd=True, joint_commands=(Interface.POSITION,)))
    assert both == limits(config(model=model, pd=True))


def test_moving_joints_are_the_revolute_continuous_and_prismatic_ones(tmp_path: Path) -> None:
    joints = movable_joints(model_from(tmp_path, TOY_URDF))
    # The fixed mount is skipped; the rest keep the model's order.
    assert [joint.name for joint in joints] == ["shoulder", "spinner", "slider"]


def test_a_joint_that_spins_freely_has_no_position_bound(tmp_path: Path) -> None:
    table = limits(
        config(
            model=model_from(tmp_path, TOY_URDF),
            joint_commands=(Interface.POSITION, Interface.VELOCITY),
        )
    )
    assert table["arm/spinner/position"] == Limits()
    assert table["arm/spinner/velocity"] == Limits(-10.0, 10.0)
    assert table["arm/slider/position"] == Limits(0.0, 0.4)


def test_model_limit_reads_one_joint() -> None:
    joint = JointDescription(
        name="j", type="revolute", lower=-1.0, upper=2.0, velocity=3.0, effort=4
    )
    assert model_limit(joint, Interface.POSITION) == Limits(-1.0, 2.0)
    assert model_limit(joint, Interface.VELOCITY) == Limits(-3.0, 3.0)
    assert model_limit(joint, Interface.EFFORT) == Limits(-4.0, 4.0)
    assert model_limit(joint, Interface.KP) == Limits(0.0, None)


@pytest.mark.parametrize(
    ("broken", "interface", "match"),
    [
        ('lower="-1.5" upper="2.5"', Interface.POSITION, "no position range"),
        ('upper="2.5"', Interface.POSITION, "half a position range"),
        ('velocity="3.0"', Interface.VELOCITY, "no velocity limit"),
        ('effort="40.0"', Interface.EFFORT, "no effort limit"),
    ],
)
def test_a_model_unclear_about_a_bound_a_command_needs_is_refused(
    tmp_path: Path, broken: str, interface: Interface, match: str
) -> None:
    # Reading a missing bound as "no limit" would let the joint go anywhere.
    model = model_from(tmp_path, TOY_URDF.replace(broken, "", 1))
    with pytest.raises(ValueError, match=match):
        limits(config(model=model, joint_commands=(interface,)))


def test_a_bound_no_command_needs_may_be_missing(tmp_path: Path) -> None:
    model = model_from(tmp_path, TOY_URDF.replace('effort="40.0"', "", 1))
    assert "arm/shoulder/position" in limits(
        config(model=model, joint_commands=(Interface.POSITION,))
    )


def test_a_joint_is_told_joint_things_only(model: RobotModel) -> None:
    with pytest.raises(ValueError, match="not told vx"):
        limits(config(model=model, joint_commands=(Interface.VX,)))


def test_a_joint_command_without_a_model_is_refused() -> None:
    with pytest.raises(ValueError, match="no model"):
        limits(config(joint_commands=(Interface.POSITION,)))
    with pytest.raises(ValueError, match="no model"):
        limits(config(pd=True))


def test_a_model_with_no_moving_joint_is_refused(tmp_path: Path) -> None:
    statue = chain_model(tmp_path, "statue", ["neck"], "", joint_type="fixed")
    with pytest.raises(ValueError, match="no joints to drive"):
        limits(config(model=statue, joint_commands=(Interface.POSITION,)))


def test_a_base_range_gives_three_clamped_keys_symmetric_about_zero() -> None:
    table = limits(ConnectionConfig(source="go2", base_range=RANGE))
    assert table == {
        "go2/base/vx": Limits(-1.5, 1.5, clamp=True),
        "go2/base/vy": Limits(0.0, 0.0, clamp=True),
        "go2/base/wz": Limits(-2.0, 2.0, clamp=True),
    }


def test_joints_come_before_the_base(model: RobotModel) -> None:
    table = limits(config(model=model, joint_commands=(Interface.POSITION,), base_range=RANGE))
    assert list(table) == [
        "arm/j1/position",
        "arm/j2/position",
        "arm/j3/position",
        "arm/base/vx",
        "arm/base/vy",
        "arm/base/wz",
    ]


def test_a_base_range_below_zero_or_without_end_is_refused() -> None:
    with pytest.raises(ValueError, match="lo above hi"):
        limits(config(base_range=Twist(linear=Vector3(-1.0, 0.0, 0.0))))
    with pytest.raises(ValueError, match="non-finite"):
        limits(config(base_range=Twist(angular=Vector3(0.0, 0.0, math.inf))))


def test_a_config_that_commands_nothing_is_refused(model: RobotModel) -> None:
    with pytest.raises(ValueError, match="told nothing"):
        limits(config())
    with pytest.raises(ValueError, match="told nothing"):
        limits(config(model=model))


def test_the_table_crosses_processes(model: RobotModel) -> None:
    table = limits(config(model=model, pd=True, base_range=RANGE))
    assert pickle.loads(pickle.dumps(table)) == table


def test_limits_refuse_unless_told_to_clamp() -> None:
    assert Limits(-1.0, 1.0).clamp is False
    assert Limits() == Limits(None, None, clamp=False)


@pytest.mark.parametrize(
    ("bad", "match"),
    [
        ({"lo": 1.0, "hi": -1.0}, "lo above hi"),
        ({"lo": math.nan}, "non-finite"),
        ({"hi": math.inf}, "non-finite"),
        ({"lo": -1.0, "clamp": True}, "both sides"),
    ],
)
def test_a_limit_must_be_a_real_range(bad: dict[str, Any], match: str) -> None:
    with pytest.raises(ValueError, match=match):
        Limits(**bad)


#: Its expected numbers are typed out rather than read back from the same
#: file, so a change to the model fails here instead of agreeing.
G1_URDF = Path(__file__).resolve().parents[2] / "robot" / "unitree" / "g1" / "g1.urdf"
G1_LEG = ("left_hip_pitch", "left_hip_roll", "left_knee")


def g1_leg_model() -> RobotModel:
    return RobotModel.from_file(G1_URDF).with_renamed_joints({f"{j}_joint": j for j in G1_LEG})


def test_the_in_repo_g1_drives_every_joint_that_moves() -> None:
    table = limits(ConnectionConfig(source="g1", model=RobotModel.from_file(G1_URDF), pd=True))
    joints = list(dict.fromkeys(key.rsplit("/", 1)[0] for key in table))
    # 29 motors; the floating base and the fixed sensor mounts are skipped.
    assert len(joints) == 29
    assert joints[0] == "g1/left_hip_pitch_joint"
    assert "g1/floating_base_joint" not in joints
    assert len(table) == 29 * 5


def test_the_in_repo_g1_gets_its_bounds_from_its_model() -> None:
    table = limits(ConnectionConfig(source="g1", model=g1_leg_model(), pd=True))
    assert table["g1/left_hip_pitch/position"] == Limits(-2.5307, 2.8798)
    assert table["g1/left_hip_pitch/velocity"] == Limits(-32.0, 32.0)
    assert table["g1/left_hip_pitch/effort"] == Limits(-88.0, 88.0)
    # An asymmetric range, so nothing here is quietly symmetrizing position.
    assert table["g1/left_knee/position"] == Limits(-0.087267, 2.8798)
    assert table["g1/left_knee/effort"] == Limits(-139.0, 139.0)
