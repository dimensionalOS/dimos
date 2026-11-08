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

"""Offline regression using the pinned R1 asset; never connects to hardware."""

from io import BytesIO

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from yourdfpy import URDF

from dimos.control.tasks.teleop_ik_task.teleop_ik_task import create_task
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_spatial import spatial_tasks
from dimos.robot.galaxea.r1pro.config import R1PRO_HOME_POSE, R1PRO_MODEL
from dimos.utils.transform_utils import pose_to_matrix

pytestmark = pytest.mark.self_hosted


@pytest.mark.parametrize("fold", [0.0, 0.4, 0.7])
def test_spatial_ik_matches_full_robot_at_measured_height(fold):
    robot = URDF.load(BytesIO(R1PRO_MODEL.load().xml.encode()), load_meshes=False)
    positions = {f"r1pro/{name}": q for name, q in R1PRO_HOME_POSE.items()}
    positions.update(
        {"r1pro/torso_joint1": fold, "r1pro/torso_joint2": -2.0 * fold, "r1pro/torso_joint3": -fold}
    )
    robot.update_cfg(positions)
    config = next(task for task in spatial_tasks() if task.type == "teleop_ik")
    task = create_task(config, {})
    try:
        measured = JointState(name=list(positions), position=list(positions.values()))
        frames = [f"{side}_gripper_link" for side in ("left", "right")]
        actual = task._solver.frame_poses(measured, frames)
        upper_from_root = np.linalg.inv(robot.get_transform("torso_link2"))
        for frame in frames:
            np.testing.assert_allclose(
                pose_to_matrix(actual[frame]),
                upper_from_root @ robot.get_transform(frame),
                atol=1e-9,
            )
    finally:
        task.stop()


def test_both_arms_reach_near_extension_at_lowered_height_with_bounded_steps():
    robot = URDF.load(BytesIO(R1PRO_MODEL.load().xml.encode()), load_meshes=False)
    neutral_anchor = robot.get_transform("torso_link3")[:3, 3].copy()
    offsets = {
        side: robot.get_transform(f"{side}_arm_link2")[:3, 3] - neutral_anchor
        for side in ("left", "right")
    }
    reaches = {
        side: np.linalg.norm(
            robot.get_transform(f"{side}_gripper_link")[:3, 3]
            - robot.get_transform(f"{side}_arm_link2")[:3, 3]
        )
        for side in offsets
    }
    positions = {f"r1pro/{name}": q for name, q in R1PRO_HOME_POSE.items()}
    positions.update(
        {"r1pro/torso_joint1": 0.4, "r1pro/torso_joint2": -0.8, "r1pro/torso_joint3": -0.4}
    )
    robot.update_cfg(positions)
    anchor = robot.get_transform("torso_link3")[:3, 3].copy()
    root_from_upper = robot.get_transform("torso_link2").copy()
    upper_from_root = np.linalg.inv(root_from_upper)
    targets = {}
    expected = {}
    for side in offsets:
        frame = f"{side}_gripper_link"
        expected[frame] = anchor + offsets[side] + [reaches[side] * 0.98, 0, 0]
        targets[frame] = PoseStamped(
            position=upper_from_root[:3, :3] @ expected[frame] + upper_from_root[:3, 3],
            orientation=(
                Rotation.from_matrix(upper_from_root[:3, :3]) * Rotation.from_euler("y", -np.pi / 2)
            ).as_quat(),
            frame_id="torso_link2",
        )
    config = next(task for task in spatial_tasks() if task.type == "teleop_ik")
    task = create_task(config, {})
    try:
        for _ in range(1200):
            measured = JointState(name=list(positions), position=list(positions.values()))
            command = task._solver.step(targets, measured, 0.02)
            assert command is not None
            assert (
                max(
                    abs(q - positions[n])
                    for n, q in zip(command.name, command.position, strict=True)
                )
                <= 0.0050001
            )
            positions.update(zip(command.name, command.position, strict=True))
        robot.update_cfg(positions)
        for frame, target in expected.items():
            assert np.linalg.norm(robot.get_transform(frame)[:3, 3] - target) < 0.005
        assert positions["r1pro/torso_joint1"] == 0.4
        assert positions["r1pro/torso_joint2"] == -0.8
    finally:
        task.stop()
