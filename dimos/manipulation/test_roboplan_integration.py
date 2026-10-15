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

"""Self-hosted integration tests for official RoboPlan Cartesian planning."""

import importlib
from pathlib import Path
from typing import Any

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.sensor_msgs.msg import JointState
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

pytest.importorskip("roboplan.cartesian_planning")
roboplan_world_module = importlib.import_module("dimos.manipulation.planning.world.roboplan_world")
roboplan_planner_module = importlib.import_module(
    "dimos.manipulation.planning.planners.roboplan_planner"
)

from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.planners.roboplan_config import (
    RoboPlanCartesianPathConfig,
    RoboPlanPlannerConfig,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.manipulation.planning.spec.validation import prepare_robot_model
from dimos.manipulation.planning.trajectory_generator.config import (
    RoboPlanTOPPRAParametrizationConfig,
)
from dimos.manipulation.planning.utils.kinematics_utils import compute_pose_error
from dimos.msgs.geometry import pose_matrix
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.xarm.config import (
    make_dual_xarm6_model_config,
    make_xarm6_model_config,
)

pytestmark = pytest.mark.self_hosted


@pytest.fixture
def roboplan_types() -> tuple[type[Any], type[Any]]:
    """Reload real bindings after any fake-binding tests in the same pytest process."""
    world_type = importlib.reload(roboplan_world_module).RoboPlanWorld
    planner_type = importlib.reload(roboplan_planner_module).RoboPlanPlanner
    return world_type, planner_type


def _sync_zero_state(
    world: Any,
    joint_names: list[str],
) -> None:
    world.sync_from_joint_state(
        JointState(
            name=joint_names,
            position=np.asarray([0.0] * len(joint_names), dtype=np.float64),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            velocity=np.array([], dtype=np.float64),
            effort=np.array([], dtype=np.float64),
        ),
    )


def test_real_roboplan_plans_fixed_orientation_cartesian_path(
    roboplan_types: tuple[type[Any], type[Any]],
) -> None:
    config = make_xarm6_model_config()
    model_path = Path(config.model.source_path)
    if not model_path.exists():
        pytest.skip(f"xArm model is unavailable: {model_path}")

    world_type, planner_type = roboplan_types
    world = world_type()
    world.load_model(prepare_robot_model(config))
    world.finalize()
    planner = planner_type(world, RoboPlanPlannerConfig())
    _sync_zero_state(world, config.joint_names)
    group_id = "manipulator"
    selection = world._planning_groups.select((group_id,))
    start = JointState(
        name=list(selection.joint_names),
        position=np.asarray([0.0] * len(selection.joint_names), dtype=np.float64),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        velocity=np.array([], dtype=np.float64),
        effort=np.array([], dtype=np.float64),
    )
    with world.scratch_context() as ctx:
        planner._apply_selected_state(ctx, start)
        start_pose = world.get_group_ee_pose(ctx, group_id)

    result = planner.plan_cartesian_path(
        world,
        selection,
        start,
        {
            group_id: (
                TransformStamped(
                    header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
                    child_frame_id="",
                    transform=Transform(
                        translation=Vector3(x=0.0, y=0.0, z=0.0),
                        rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                ),
                TransformStamped(
                    header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
                    transform=Transform(
                        translation=Vector3(x=0.005, y=0.0, z=0.0),
                        rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                    child_frame_id="",
                ),
            )
        },
        RoboPlanCartesianPathConfig(
            speed_mode="time_optimal",
            toppra_blend_deviation=0.0,
        ),
    )

    assert result.status == PlanningStatus.SUCCESS, result.message
    assert len(result.path) >= 2
    assert result.timestamps is not None
    assert len(result.timestamps) == len(result.path)
    with world.scratch_context() as ctx:
        planner._apply_selected_state(ctx, result.path[-1])
        final_pose = world.get_group_ee_pose(ctx, group_id)
    expected = pose_matrix(start_pose.pose)
    expected[0, 3] += 0.005
    position_error, orientation_error = compute_pose_error(pose_matrix(final_pose.pose), expected)
    assert position_error <= 0.005
    assert orientation_error <= 0.01


def test_real_roboplan_synchronizes_different_length_dual_arm_targets(
    roboplan_types: tuple[type[Any], type[Any]],
) -> None:
    config = make_dual_xarm6_model_config()
    model_path = Path(config.model.source_path)
    if not model_path.exists():
        pytest.skip(f"xArm model is unavailable: {model_path}")

    world_type, planner_type = roboplan_types
    world = world_type()
    world.load_model(prepare_robot_model(config))
    world.finalize()
    planner = planner_type(world, RoboPlanPlannerConfig())
    _sync_zero_state(world, config.joint_names)
    left_group_id = "left_arm"
    right_group_id = "right_arm"
    selection = world._planning_groups.select((left_group_id, right_group_id))
    start = JointState(
        name=list(selection.joint_names),
        position=np.asarray([0.0] * len(selection.joint_names), dtype=np.float64),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        velocity=np.array([], dtype=np.float64),
        effort=np.array([], dtype=np.float64),
    )

    result = planner.plan_cartesian_path(
        world,
        selection,
        start,
        {
            left_group_id: (
                TransformStamped(
                    header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
                    child_frame_id="",
                    transform=Transform(
                        translation=Vector3(x=0.0, y=0.0, z=0.0),
                        rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                ),
                TransformStamped(
                    header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
                    transform=Transform(
                        translation=Vector3(x=0.0015, y=0.001, z=0.0),
                        rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                    child_frame_id="",
                ),
                TransformStamped(
                    header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
                    transform=Transform(
                        translation=Vector3(x=0.003, y=0.0, z=0.0),
                        rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                    child_frame_id="",
                ),
            ),
            right_group_id: (
                TransformStamped(
                    header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
                    child_frame_id="",
                    transform=Transform(
                        translation=Vector3(x=0.0, y=0.0, z=0.0),
                        rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                ),
                TransformStamped(
                    header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
                    transform=Transform(
                        translation=Vector3(x=0.005, y=0.0, z=0.0),
                        rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                    child_frame_id="",
                ),
            ),
        },
        RoboPlanCartesianPathConfig(),
    )

    assert result.status == PlanningStatus.SUCCESS, result.message
    assert result.timestamps is not None
    assert len(result.timestamps) == len(result.path)
    assert all(state.name == list(selection.joint_names) for state in result.path)


@pytest.fixture
def scalar_world(tmp_path, roboplan_types):
    model_path = tmp_path / "scalar.urdf"
    model_path.write_text("""
        <robot name="scalar">
          <link name="base"/>
          <link name="arm"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
          <joint name="slide" type="prismatic">
            <parent link="base"/><child link="arm"/><axis xyz="1 0 0"/>
            <limit lower="-2" upper="2" effort="1" velocity="0.2" acceleration="0.4"/>
          </joint>
        </robot>
    """)
    config = RobotModelConfig(
        model=RobotModel.from_file(model_path),
        joint_names=["slide"],
        base_link="base",
        base_pose=PoseStamped(
            header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
            pose=Pose(
                position=Point(x=1, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        planning_groups=[
            PlanningGroupDefinition(
                name="arm", joint_names=("slide",), base_link="base", tip_link="arm"
            )
        ],
    )
    world = roboplan_types[0]()
    world.load_model(prepare_robot_model(config))
    world.finalize()
    world.sync_from_joint_state(
        JointState(
            name=["slide"],
            position=np.asarray([0.0], dtype=np.float64),
            header=Header(frame_id="", stamp=Time(sec=0, nanosec=0)),
            velocity=np.array([], dtype=np.float64),
            effort=np.array([], dtype=np.float64),
        )
    )
    return world


def test_real_contexts_keep_independent_state_and_refresh_geometry(scalar_world):
    world = scalar_world
    with world.scratch_context() as first, world.scratch_context() as second:
        world.set_joint_state(
            first,
            JointState(
                name=["slide"],
                position=np.asarray([0.0], dtype=np.float64),
                header=Header(frame_id="", stamp=Time(sec=0, nanosec=0)),
                velocity=np.array([], dtype=np.float64),
                effort=np.array([], dtype=np.float64),
            ),
        )
        world.set_joint_state(
            second,
            JointState(
                name=["slide"],
                position=np.asarray([0.4], dtype=np.float64),
                header=Header(frame_id="", stamp=Time(sec=0, nanosec=0)),
                velocity=np.array([], dtype=np.float64),
                effort=np.array([], dtype=np.float64),
            ),
        )
        np.testing.assert_allclose(world.get_link_pose(first, "arm")[:3, 3], [1, 0, 0])
        np.testing.assert_allclose(world.get_link_pose(second, "arm")[:3, 3], [1.4, 0, 0])
        with world.parametrization_model() as model:
            np.testing.assert_allclose(model.scene.getCurrentJointPositions(), [0.0])
        world.add_obstacle(
            Obstacle(
                name="block",
                obstacle_type=ObstacleType.BOX,
                dimensions=(0.1, 0.1, 0.1),
                pose=PoseStamped(
                    header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
                    pose=Pose(
                        position=Point(x=1.4, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                ),
            )
        )
        assert not world.is_collision_free(second)
        assert world.is_collision_free(first)
        assert world.update_obstacle_pose(
            "block",
            PoseStamped(
                header=Header(frame_id="world", stamp=Time(sec=0, nanosec=0)),
                pose=Pose(
                    position=Point(x=1.8, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
            ),
        )
        assert world.is_collision_free(second)
        assert world.remove_obstacle("block")
        assert world.is_collision_free(second)


def test_real_rrt_and_toppra_use_finite_prepared_limits(scalar_world, roboplan_types, monkeypatch):
    world = scalar_world
    group = world.all_planning_group()
    with world.parametrization_model() as model:
        np.testing.assert_allclose(model.scene.getVelocityLimitVectors(group.name), [[-0.2], [0.2]])
        np.testing.assert_allclose(
            model.scene.getAccelerationLimitVectors(group.name), [[-0.4], [0.4]]
        )
    assert world.get_prepared_model().joint_space.acceleration_limits == (0.4,)

    planner = roboplan_types[1](world, RoboPlanPlannerConfig())
    start = JointState(
        name=["slide"],
        position=np.asarray([0.0], dtype=np.float64),
        header=Header(frame_id="", stamp=Time(sec=0, nanosec=0)),
        velocity=np.array([], dtype=np.float64),
        effort=np.array([], dtype=np.float64),
    )
    goal = JointState(
        name=["slide"],
        position=np.asarray([0.5], dtype=np.float64),
        header=Header(frame_id="", stamp=Time(sec=0, nanosec=0)),
        velocity=np.array([], dtype=np.float64),
        effort=np.array([], dtype=np.float64),
    )
    result = planner.plan_joint_path(world, start, goal, timeout=1.0)
    assert result.status == PlanningStatus.SUCCESS, result.message
    selection = world._planning_groups.select(("arm",))
    # The integration fixture reloads World after fake bindings. Restore the
    # parametrizer's type binding after this test so earlier imports stay valid.
    parameterizer_module = importlib.import_module(
        "dimos.manipulation.planning.trajectory_generator.roboplan_toppra_parametrizer"
    )
    monkeypatch.setattr(parameterizer_module, "RoboPlanWorld", type(world))
    parameterizer_type = parameterizer_module.RoboPlanTOPPRAParametrizer
    plan = parameterizer_type(RoboPlanTOPPRAParametrizationConfig()).materialize_plan(
        world, selection, result
    )
    np.testing.assert_allclose(plan.trajectory.points[0].positions, [0.0])
    np.testing.assert_allclose(plan.trajectory.points[-1].positions, [0.5])
    velocities = np.asarray([point.velocities for point in plan.trajectory.points])
    times = np.asarray([point.time_from_start for point in plan.trajectory.points])
    accelerations = np.diff(velocities, axis=0) / np.diff(times)[:, None]
    assert np.max(np.abs(velocities)) <= 0.2 * 1.05
    assert np.max(np.abs(accelerations)) <= 0.4 * 1.05
    assert world.check_edge_collision_free(start, goal)
