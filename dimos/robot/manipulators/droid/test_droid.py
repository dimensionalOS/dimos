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

import numpy as np
import pytest

from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.manipulation.planning.world import roboplan_world as roboplan_world_module
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.droid.config import (
    DROID_ARM_JOINTS,
    DROID_HOME_JOINTS,
    make_droid_model_config,
)

pytestmark = pytest.mark.self_hosted

# Joint poses with the pad midpoint Isaac Sim reports for them, RoboLab v0.3.3: position
# in the world, orientation (x, y, z, w) of z along the approach and y along the closing.
ISAAC_POSES = [
    (
        [0.733758, -0.70649, -2.445523, -2.883965, 2.101814, 3.232456, 0.320425],
        [0.053424, -0.085009, -0.058657],
        [0.843582, -0.476962, 0.113535, -0.219059],
    ),
    (
        [0.239766, 1.457534, 1.739217, -2.910806, 1.97941, 0.117433, 1.276785],
        [0.133141, 0.257942, 0.502041],
        [0.310652, 0.557445, 0.399804, 0.657956],
    ),
]


@pytest.fixture(scope="module")
def world() -> roboplan_world_module.RoboPlanWorld:
    world = roboplan_world_module.RoboPlanWorld()
    world.load_model(prepare_robot_model(make_droid_model_config()))
    world.finalize()
    return world


@pytest.mark.parametrize(("joints", "position", "orientation"), ISAAC_POSES)
def test_grasp_frame_matches_isaac(
    world: roboplan_world_module.RoboPlanWorld,
    joints: list[float],
    position: list[float],
    orientation: list[float],
) -> None:
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, JointState(name=list(DROID_ARM_JOINTS), position=joints))
        pose = world.get_ee_pose(ctx)

    assert [pose.position.x, pose.position.y, pose.position.z] == pytest.approx(position, abs=1e-5)
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    expected = np.asarray(orientation) / np.linalg.norm(orientation)
    angle = 2 * np.arccos(min(1.0, abs(float(np.dot(q / np.linalg.norm(q), expected)))))
    assert angle < 1e-5


def test_home_pose_is_collision_free(world: roboplan_world_module.RoboPlanWorld) -> None:
    home = JointState(name=list(DROID_ARM_JOINTS), position=DROID_HOME_JOINTS)

    assert world.check_config_collision_free(home)
