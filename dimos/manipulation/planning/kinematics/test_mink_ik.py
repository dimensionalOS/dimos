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


_TWO_DOF_ARM_URDF = """<robot name="two_dof_arm">
  <link name="base">
    <inertial>
      <origin xyz="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="upper">
    <inertial>
      <origin xyz="0 0 0.1"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <link name="lower">
    <inertial>
      <origin xyz="0 0 0.1"/>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  <joint name="shoulder" type="revolute">
    <parent link="base"/>
    <child link="upper"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="10" velocity="10"/>
  </joint>
  <joint name="elbow" type="revolute">
    <parent link="upper"/>
    <child link="lower"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5" effort="10" velocity="10"/>
  </joint>
</robot>
"""


def _make_arm_world(tmp_path: Path):
    from dimos.manipulation.planning.kinematics.mink_ik import MinkIK
    from dimos.manipulation.planning.world.mujoco_world import MujocoWorld

    model_path = tmp_path / "arm.urdf"
    model_path.write_text(_TWO_DOF_ARM_URDF)
    config = RobotModelConfig(
        name="arm",
        model_path=model_path,
        base_pose=PoseStamped(
            frame_id="world",
            position=Vector3(0.0, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        joint_names=["shoulder", "elbow"],
        end_effector_link="lower",
        base_link="base",
    )
    world = MujocoWorld()
    robot_id = world.add_robot(config)
    world.finalize()
    return world, robot_id, MinkIK()


def _reachable_target(world, robot_id, positions: list[float]) -> PoseStamped:
    from dimos.msgs.sensor_msgs.JointState import JointState

    with world.scratch_context() as ctx:
        world.set_joint_state(
            ctx, robot_id, JointState(name=["shoulder", "elbow"], position=positions)
        )
        return world.get_ee_pose(ctx, robot_id)


def test_mink_warm_start_reuses_previous_solution(tmp_path: Path) -> None:
    pytest.importorskip("mink")
    world, robot_id, kin = _make_arm_world(tmp_path)
    target = _reachable_target(world, robot_id, [0.5, -0.7])

    cold = kin.solve(world=world, robot_id=robot_id, target_pose=target, orientation_tolerance=10.0)
    assert cold.status.name == "SUCCESS"
    context = kin._robot_contexts[str(robot_id)]
    assert context.q_warm is not None

    # A nearby target must solve from the warm seed: attempt 0 (the warm
    # start) converges, and the solution stays in the same branch.
    near = _reachable_target(world, robot_id, [0.55, -0.75])
    attempts: list[int] = []
    warm = kin.solve(
        world=world,
        robot_id=robot_id,
        target_pose=near,
        orientation_tolerance=10.0,
        on_step=lambda _js, _err, attempt: attempts.append(attempt) and None,
    )
    assert warm.status.name == "SUCCESS"
    assert set(attempts) <= {0}
    assert np.allclose(warm.joint_state.position, [0.55, -0.75], atol=0.2)


def test_mink_on_step_abort_returns_best_so_far(tmp_path: Path) -> None:
    pytest.importorskip("mink")
    world, robot_id, kin = _make_arm_world(tmp_path)
    # Unreachable target (arm length is 0.4 m from a 0.1 m base riser).
    target = PoseStamped(
        frame_id="world",
        position=Vector3(1.0, 0.0, 0.2),
        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
    )
    calls: list[int] = []
    result = kin.solve(
        world=world,
        robot_id=robot_id,
        target_pose=target,
        orientation_tolerance=10.0,
        on_step=lambda _js, _err, _attempt: calls.append(1) or True,
    )
    assert len(calls) == 1
    assert result.status.name != "SUCCESS"
    assert result.joint_state is not None  # best partial candidate is preserved
