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

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.world.mujoco_world import compile_mujoco_model_from_config
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("xacro")


def test_compile_mujoco_model_from_config_processes_xacro_args(tmp_path: Path) -> None:
    model_path = tmp_path / "robot.urdf.xacro"
    model_path.write_text(
        """<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">
  <xacro:arg name="joint_name" default="joint1"/>
  <mujoco>
    <compiler strippath="false" discardvisual="true"/>
  </mujoco>
  <link name="base">
    <inertial>
      <origin xyz="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="tip">
    <inertial>
      <origin xyz="0 0 0.1"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="$(arg joint_name)" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""
    )
    config = RobotModelConfig(
        name="arm",
        model_path=model_path,
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["custom_joint"],
        end_effector_link="tip",
        base_link="base",
        xacro_args={"joint_name": "custom_joint"},
    )

    model = compile_mujoco_model_from_config(config)

    assert model.njnt == 1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "custom_joint") == 0


def test_compile_mujoco_model_from_config_infers_bad_inertials(tmp_path: Path) -> None:
    model_path = tmp_path / "robot.urdf"
    model_path.write_text(
        """<robot name="bad_inertia_robot">
  <link name="base">
    <inertial>
      <origin xyz="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="tip">
    <inertial>
      <origin xyz="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="-0.01" iyz="0" izz="0.01"/>
    </inertial>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""
    )
    config = RobotModelConfig(
        name="arm",
        model_path=model_path,
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1"],
        end_effector_link="tip",
        base_link="base",
    )

    model = compile_mujoco_model_from_config(config)

    assert model.njnt == 1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tip") > 0
