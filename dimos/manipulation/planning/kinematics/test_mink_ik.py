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

import numpy as np
import pytest

from dimos.manipulation.planning.kinematics.mink_ik import _resolve_end_effector_frame
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.world.mujoco_world import compile_mujoco_model_from_config
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3

mujoco = pytest.importorskip("mujoco")


def test_resolve_end_effector_frame_handles_fixed_tcp_link(tmp_path: Path) -> None:
    model_path = tmp_path / "robot.urdf"
    model_path.write_text(
        """<robot name="fixed_tcp_robot">
  <link name="base">
    <inertial>
      <origin xyz="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="tool_body">
    <inertial>
      <origin xyz="0 0 0.1"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="tcp"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="tool_body"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="tcp_joint" type="fixed">
    <parent link="tool_body"/>
    <child link="tcp"/>
    <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>
  </joint>
</robot>
"""
    )
    config = RobotModelConfig(
        name="arm",
        model_path=model_path,
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1"],
        end_effector_link="tcp",
        base_link="base",
    )
    model = compile_mujoco_model_from_config(config)

    body_name, body_id, body_to_ee = _resolve_end_effector_frame(mujoco, model, config)

    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tcp") < 0
    assert body_name == "tool_body"
    assert body_id == mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tool_body")
    np.testing.assert_allclose(body_to_ee[:3, 3], [0.1, 0.2, 0.3])
