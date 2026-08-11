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

import math

import pytest

pytest.importorskip("pinocchio")

from dimos.hardware.whole_body.gravity import GravityFeedforward

L = 0.5
M = 2.0
G = 9.81

PENDULUM_URDF = f"""<?xml version="1.0"?>
<robot name="pendulum">
  <link name="base"/>
  <link name="arm">
    <inertial>
      <origin xyz="{L} 0 0"/>
      <mass value="{M}"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <link name="tip"/>
  <joint name="shoulder" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="10"/>
  </joint>
  <joint name="tip_fix" type="fixed">
    <origin xyz="{2 * L} 0 0"/>
    <parent link="arm"/>
    <child link="tip"/>
  </joint>
</robot>
"""


def pendulum(tmp_path, **kwargs):
    urdf = tmp_path / "pendulum.urdf"
    urdf.write_text(PENDULUM_URDF)
    return GravityFeedforward(
        model_path=str(urdf),
        joint_map={"r/shoulder": "shoulder"},
        ff_joints=("r/shoulder",),
        **kwargs,
    )


def test_holding_torque_matches_analytic(tmp_path):
    ff = pendulum(tmp_path)
    # Horizontal arm (q=0, +x): supporting torque about +y is -m g L.
    assert ff.tau({"r/shoulder": 0.0})["r/shoulder"] == pytest.approx(-M * G * L, rel=1e-6)
    # Hanging arm: no moment.
    assert ff.tau({"r/shoulder": math.pi / 2})["r/shoulder"] == pytest.approx(0.0, abs=1e-9)


def test_payload_adds_point_mass_moment(tmp_path):
    ff = pendulum(tmp_path, payloads=(("tip", 1.5),))
    expected = -(M * G * L) - (1.5 * G * 2 * L)
    assert ff.tau({"r/shoulder": 0.0})["r/shoulder"] == pytest.approx(expected, rel=1e-6)


def test_scale_multiplies_torque(tmp_path):
    ff = pendulum(tmp_path, scale=0.7)
    assert ff.tau({"r/shoulder": 0.0})["r/shoulder"] == pytest.approx(-0.7 * M * G * L, rel=1e-6)


def test_unknown_joint_and_frame_raise(tmp_path):
    urdf = tmp_path / "pendulum.urdf"
    urdf.write_text(PENDULUM_URDF)
    with pytest.raises(ValueError, match="nope"):
        GravityFeedforward(model_path=str(urdf), joint_map={"r/x": "nope"}, ff_joints=("r/x",)).tau(
            {}
        )
    with pytest.raises(ValueError, match="ghost"):
        GravityFeedforward(
            model_path=str(urdf),
            joint_map={"r/shoulder": "shoulder"},
            ff_joints=("r/shoulder",),
            payloads=(("ghost", 1.0),),
        ).tau({})


def test_ff_joint_missing_from_map_raises(tmp_path):
    urdf = tmp_path / "pendulum.urdf"
    urdf.write_text(PENDULUM_URDF)
    with pytest.raises(ValueError, match="missing from joint map"):
        GravityFeedforward(
            model_path=str(urdf), joint_map={"r/shoulder": "shoulder"}, ff_joints=("r/other",)
        ).tau({})
