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

"""Integration coverage for the pinned RoboPlan OInK Python bindings."""

import importlib
from pathlib import Path

import numpy as np
import pytest

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.transform_utils import pose_to_matrix

pytest.importorskip("roboplan.optimal_ik")


def test_pinned_roboplan_solves_reachable_target_without_mutating_scene(
    tmp_path: Path,
) -> None:
    from dimos.manipulation.planning.world import roboplan_oink, roboplan_world

    importlib.reload(roboplan_oink)
    RoboPlanWorld = importlib.reload(roboplan_world).RoboPlanWorld
    model_path = tmp_path / "one_joint.urdf"
    model_path.write_text(
        """
        <robot name="one_joint">
          <link name="base"/>
          <link name="tip"/>
          <joint name="joint1" type="revolute">
            <parent link="base"/>
            <child link="tip"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>
        """
    )
    config = RobotModelConfig(
        name="arm",
        model_path=model_path,
        base_pose=PoseStamped(
            frame_id="world",
            position=Vector3(),
            orientation=Quaternion(),
        ),
        joint_names=["joint1"],
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint1",),
                base_link="base",
                tip_link="tip",
            )
        ],
        joint_limits_lower=[-1.0],
        joint_limits_upper=[1.0],
    )
    world = RoboPlanWorld()
    robot_id = world.add_robot(config)
    world.finalize()
    world.sync_from_joint_state(
        robot_id,
        JointState(name=["joint1"], position=[0.2]),
    )
    context = world.get_live_context()
    target = world.get_group_ee_pose(context, "arm/manipulator")
    before = pose_to_matrix(target).copy()
    target.orientation = Quaternion.from_euler(Vector3(z=0.3))

    result = world.solve(
        world,
        robot_id,
        target,
        check_collision=False,
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["arm/joint1"]
    assert result.joint_state.position == pytest.approx([0.3], abs=0.01)
    np.testing.assert_allclose(
        pose_to_matrix(world.get_group_ee_pose(context, "arm/manipulator")),
        before,
    )
