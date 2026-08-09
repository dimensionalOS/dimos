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

"""Re-placing a robot in a built RoboPlan scene."""

import importlib
import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest

pytest.importorskip("roboplan.core")
roboplan_world_module = importlib.import_module("dimos.manipulation.planning.world.roboplan_world")

from dimos.manipulation.planning.spec.enums import ObstacleType
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.xarm.config import make_xarm6_model_config

pytestmark = pytest.mark.self_hosted


@pytest.fixture
def world_type() -> type[Any]:
    """Reload real bindings after any fake-binding tests in the same process."""
    return importlib.reload(roboplan_world_module).RoboPlanWorld


@pytest.fixture
def built_world(world_type: type[Any]) -> tuple[Any, str, Any]:
    config = make_xarm6_model_config(name="arm")
    if not Path(config.model_path).exists():
        pytest.skip(f"xArm model is unavailable: {config.model_path}")
    world = world_type()
    robot_id = world.add_robot(config)
    world.finalize()
    world.sync_from_joint_state(
        robot_id,
        JointState(name=config.joint_names, position=[0.0] * len(config.joint_names)),
    )
    return world, robot_id, config


def _ee_position(world: Any, group_id: str) -> np.ndarray:
    pose = world.get_group_ee_pose(world.get_live_context(), group_id)
    return np.array([pose.position.x, pose.position.y, pose.position.z])


def _pose(x: float, y: float, z: float, yaw: float = 0.0) -> PoseStamped:
    return PoseStamped(
        frame_id="world",
        position=Vector3(x, y, z),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def test_rebuild_moves_forward_kinematics_by_the_new_placement(
    built_world: tuple[Any, str, Any],
) -> None:
    world, robot_id, _config = built_world
    group_id = "arm/manipulator"
    before = _ee_position(world, group_id)

    world.rebuild_with_base_poses({robot_id: _pose(2.0, 1.0, 0.5, yaw=math.pi / 2)})

    rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    expected = rotation @ before + np.array([2.0, 1.0, 0.5])
    assert _ee_position(world, group_id) == pytest.approx(expected, abs=1e-9)


def test_rebuild_publishes_the_new_placement_on_the_robot_config(
    built_world: tuple[Any, str, Any],
) -> None:
    world, robot_id, _config = built_world

    world.rebuild_with_base_poses({robot_id: _pose(2.0, 1.0, 0.5)})

    base = world.get_robot_config(robot_id).base_pose
    assert (base.position.x, base.position.y, base.position.z) == (2.0, 1.0, 0.5)


def test_rebuild_keeps_obstacles_and_joint_state(
    built_world: tuple[Any, str, Any],
) -> None:
    world, robot_id, config = built_world
    world.add_obstacle(
        Obstacle(
            name="box",
            pose=PoseStamped(position=Vector3(1.0, 0.0, 0.5)),
            obstacle_type=ObstacleType.BOX,
            dimensions=(0.2, 0.2, 0.2),
        )
    )
    world.sync_from_joint_state(
        robot_id,
        JointState(name=config.joint_names, position=[0.1] * len(config.joint_names)),
    )

    world.rebuild_with_base_poses({robot_id: _pose(2.0, 1.0, 0.5)})

    assert [obstacle.name for obstacle in world.get_obstacles()] == ["box"]
    state = world.get_joint_state(world.get_live_context(), robot_id)
    assert list(state.position) == pytest.approx([0.1] * len(config.joint_names))


def test_rebuild_advances_the_model_epoch(built_world: tuple[Any, str, Any]) -> None:
    world, robot_id, _config = built_world
    before = world.model_epoch

    world.rebuild_with_base_poses({robot_id: _pose(1.0, 0.0, 0.0)})

    assert world.model_epoch == before + 1


def test_a_rejected_placement_leaves_the_previous_scene_serving(
    built_world: tuple[Any, str, Any],
) -> None:
    world, robot_id, _config = built_world
    group_id = "arm/manipulator"
    before = _ee_position(world, group_id)
    epoch = world.model_epoch

    with pytest.raises(ValueError, match="finite"):
        world.rebuild_with_base_poses({robot_id: _pose(float("nan"), 0.0, 0.0)})

    assert world.model_epoch == epoch
    assert _ee_position(world, group_id) == pytest.approx(before, abs=1e-9)


def test_rebuild_with_no_placements_is_a_no_op(built_world: tuple[Any, str, Any]) -> None:
    world, _robot_id, _config = built_world
    epoch = world.model_epoch

    world.rebuild_with_base_poses({})

    assert world.model_epoch == epoch


def test_rebuild_rejects_an_unknown_robot(built_world: tuple[Any, str, Any]) -> None:
    world, _robot_id, _config = built_world

    with pytest.raises(KeyError):
        world.rebuild_with_base_poses({"robot_99": _pose(1.0, 0.0, 0.0)})
