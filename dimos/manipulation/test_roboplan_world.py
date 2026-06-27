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

"""Pure-Python tests for the optional RoboPlan world adapter."""

from __future__ import annotations

import importlib
from pathlib import Path
import sys
from types import ModuleType
from typing import Any, ClassVar
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from dimos.manipulation.planning.groups.models import (
    PlanningGroupDefinition,
    PlanningGroupSelection,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import (
    CartesianDelta,
    CartesianPlanningRequest,
    Obstacle,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.transform_utils import pose_to_matrix


class FakeJointConfiguration:
    def __init__(
        self, joint_names: list[str] | None = None, positions: np.ndarray | None = None
    ) -> None:
        self.joint_names = joint_names or []
        self.positions = np.asarray(positions if positions is not None else [], dtype=np.float64)


class FakeJointPath:
    def __init__(self, joint_names: list[str], positions: list[np.ndarray]) -> None:
        self.joint_names = joint_names
        self.positions = positions


class FakeCartesianConfiguration:
    def __init__(self, frame_name: str = "", matrix: np.ndarray | None = None) -> None:
        self.frame_name = frame_name
        self.matrix = np.asarray(matrix if matrix is not None else np.eye(4), dtype=np.float64)


class FakeJointGroupInfo:
    def __init__(self, joint_names: list[str]) -> None:
        self.joint_names = joint_names


class FakeScene:
    joint_group_joint_names: ClassVar[list[str] | None] = None
    position_limits_lower: ClassVar[list[float]] = [-1.0, -2.0]
    position_limits_upper: ClassVar[list[float]] = [1.0, 2.0]
    fk_rotation: ClassVar[np.ndarray | None] = None

    def __init__(self, *args: Any) -> None:
        self.constructor_args = args
        self.models: list[tuple[str, str, dict[str, str]]] = []
        self.geometry: dict[str, np.ndarray] = {}
        self.current_positions: np.ndarray | None = None
        self.joint_groups = self._parse_joint_groups(args[2] if len(args) > 2 else None)

    def _parse_joint_groups(self, srdf_path: Any) -> dict[str, list[str]]:
        if srdf_path is None:
            return {}
        try:
            root = ET.parse(str(srdf_path)).getroot()
        except (ET.ParseError, FileNotFoundError, TypeError):
            return {}
        groups: dict[str, list[str]] = {}
        for group in root.findall("group"):
            group_name = group.get("name")
            if group_name:
                groups[group_name] = [
                    name for joint in group.findall("joint") if (name := joint.get("name"))
                ]
        return groups

    def addRobotModel(self, path: str, name: str, package_paths: dict[str, str]) -> str:
        self.models.append((path, name, package_paths))
        return name

    def hasCollisions(self, q: np.ndarray) -> bool:
        return bool(np.any(np.asarray(q) > 0.9))

    def getPositionLimitVectors(
        self, group_name: str = "", collapsed: bool = False
    ) -> tuple[np.ndarray, np.ndarray]:
        _ = (group_name, collapsed)
        return np.asarray(self.position_limits_lower), np.asarray(self.position_limits_upper)

    def getJointGroupInfo(self, name: str) -> FakeJointGroupInfo:
        if self.joint_group_joint_names is not None:
            return FakeJointGroupInfo(self.joint_group_joint_names)
        return FakeJointGroupInfo(self.joint_groups[name])

    def toFullJointPositions(self, group_name: str, q: np.ndarray) -> np.ndarray:
        _ = group_name
        return q

    def setJointPositions(self, q: np.ndarray) -> None:
        self.current_positions = np.asarray(q, dtype=np.float64)

    def addBoxGeometry(
        self, obstacle_id: str, width: float, height: float, depth: float, matrix: np.ndarray
    ) -> str:
        _ = (width, height, depth)
        self.geometry[obstacle_id] = matrix
        return obstacle_id

    def updateGeometryPlacement(self, handle: str, matrix: np.ndarray) -> None:
        self.geometry[handle] = matrix

    def removeGeometry(self, handle: str) -> None:
        del self.geometry[handle]

    def forwardKinematics(self, q: np.ndarray, frame_name: str, base_frame: str = "") -> np.ndarray:
        _ = (frame_name, base_frame)
        mat = np.eye(4)
        if self.fk_rotation is not None:
            mat[:3, :3] = self.fk_rotation
        mat[0, 3] = float(np.sum(q))
        return mat

    def computeFrameJacobian(
        self, q: np.ndarray, frame_name: str, local: bool = True
    ) -> np.ndarray:
        _ = (q, frame_name, local)
        group_size = len(self.joint_group_joint_names or next(iter(self.joint_groups.values())))
        columns = np.arange(1, group_size + 1, dtype=np.float64)
        return np.tile(columns, (6, 1))


class FakeRRTOptions:
    def __init__(self) -> None:
        self.group_name = ""
        self.timeout = 0.0
        self.max_time = 0.0
        self.max_planning_time = 0.0
        self.collision_check_use_bisection = True
        self.collision_check_step_size = 0.05


class FakeRRT:
    last_options: ClassVar[FakeRRTOptions | None] = None
    last_start: ClassVar[FakeJointConfiguration | None] = None
    last_goal: ClassVar[FakeJointConfiguration | None] = None

    def __init__(self, scene: FakeScene, options: FakeRRTOptions) -> None:
        self.scene = scene
        self.options = options
        FakeRRT.last_options = options

    def plan(
        self, q_start: FakeJointConfiguration, q_goal: FakeJointConfiguration
    ) -> FakeJointPath:
        assert isinstance(q_start, FakeJointConfiguration)
        assert isinstance(q_goal, FakeJointConfiguration)
        FakeRRT.last_start = q_start
        FakeRRT.last_goal = q_goal
        midpoint = (np.asarray(q_start.positions) + np.asarray(q_goal.positions)) / 2.0
        return FakeJointPath(
            q_start.joint_names,
            [np.asarray(q_start.positions), midpoint, np.asarray(q_goal.positions)],
        )


class FakeSimpleIkOptions:
    def __init__(self) -> None:
        self.max_time = 0.0
        self.timeout = 0.0
        self.max_solve_time = 0.0
        self.max_iterations = 0


class FakeSimpleIk:
    last_options: ClassVar[FakeSimpleIkOptions | None] = None
    last_goal: ClassVar[FakeCartesianConfiguration | None] = None
    last_start: ClassVar[FakeJointConfiguration | None] = None

    def __init__(self, scene: FakeScene, options: FakeSimpleIkOptions) -> None:
        self.scene = scene
        self.options = options
        FakeSimpleIk.last_options = options

    def solveIk(
        self,
        goal: FakeCartesianConfiguration,
        start: FakeJointConfiguration,
        solution: FakeJointConfiguration,
    ) -> bool:
        _ = self.scene
        FakeSimpleIk.last_goal = goal
        FakeSimpleIk.last_start = start
        joint_count = len(start.joint_names)
        if joint_count == 0:
            return False
        solution.joint_names = list(start.joint_names)
        solution.positions = np.full(joint_count, goal.matrix[0, 3] / joint_count)
        return True


def _install_fake_roboplan(
    monkeypatch: pytest.MonkeyPatch, *, include_simple_ik: bool = False
) -> None:
    roboplan_pkg = ModuleType("roboplan")
    roboplan_pkg.__path__ = []  # type: ignore[attr-defined]
    core = ModuleType("roboplan.core")
    core.Scene = FakeScene  # type: ignore[attr-defined]
    core.JointConfiguration = FakeJointConfiguration  # type: ignore[attr-defined]
    core.CartesianConfiguration = FakeCartesianConfiguration  # type: ignore[attr-defined]

    def has_collisions_along_path(
        scene: FakeScene,
        q_start: np.ndarray,
        q_end: np.ndarray,
        max_step_size: float,
        bisection: bool = False,
        check_endpoints: bool = True,
    ) -> bool:
        _ = (scene, max_step_size, bisection, check_endpoints)
        for t in np.linspace(0.0, 1.0, 5):
            if scene.hasCollisions(q_start + t * (q_end - q_start)):
                return True
        return False

    core.hasCollisionsAlongPath = has_collisions_along_path  # type: ignore[attr-defined]

    rrt = ModuleType("roboplan.rrt")
    rrt.RRTOptions = FakeRRTOptions  # type: ignore[attr-defined]
    rrt.RRT = FakeRRT  # type: ignore[attr-defined]

    monkeypatch.setitem(sys.modules, "roboplan", roboplan_pkg)
    monkeypatch.setitem(sys.modules, "roboplan.core", core)
    monkeypatch.setitem(sys.modules, "roboplan.rrt", rrt)
    if include_simple_ik:
        simple_ik = ModuleType("roboplan.simple_ik")
        simple_ik.SimpleIkOptions = FakeSimpleIkOptions  # type: ignore[attr-defined]
        simple_ik.SimpleIk = FakeSimpleIk  # type: ignore[attr-defined]
        monkeypatch.setitem(sys.modules, "roboplan.simple_ik", simple_ik)


@pytest.fixture
def fake_roboplan(monkeypatch: pytest.MonkeyPatch) -> None:
    _install_fake_roboplan(monkeypatch)


@pytest.fixture
def robot_config(tmp_path: Path) -> RobotModelConfig:
    model_path = tmp_path / "robot.urdf"
    model_path.write_text(
        """
        <robot name="fake">
          <material name="Black">
            <color rgba="0 0 0 1"/>
          </material>
          <link name="base"/>
          <link name="link1">
            <visual>
              <geometry><box size="0.1 0.1 0.1"/></geometry>
              <material name="Black"/>
            </visual>
          </link>
          <joint name="joint1" type="revolute">
            <parent link="base"/>
            <child link="link1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>
        """
    )
    return RobotModelConfig(
        name="arm",
        model_path=model_path,
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        joint_names=["joint1", "joint2"],
        end_effector_link="tcp",
        joint_limits_lower=[-1.0, -2.0],
        joint_limits_upper=[1.0, 2.0],
    )


def _make_world(fake_roboplan: None, robot_config: RobotModelConfig) -> tuple[Any, str]:
    module = _import_roboplan_world(fake_roboplan)

    world = module.RoboPlanWorld()
    robot_id = world.add_robot(robot_config)
    return world, robot_id


def _default_selection(world: Any, robot_config: RobotModelConfig) -> PlanningGroupSelection:
    return world._planning_groups.select([f"{robot_config.name}/manipulator"])


def _import_roboplan_world(fake_roboplan: None) -> ModuleType:
    _ = fake_roboplan
    module_name = "dimos.manipulation.planning.world.roboplan_world"
    if module_name in sys.modules:
        return importlib.reload(sys.modules[module_name])
    return importlib.import_module(module_name)


def test_roboplan_bindings_are_imported_at_module_load(fake_roboplan: None) -> None:
    module = _import_roboplan_world(fake_roboplan)

    assert module.roboplan_core.Scene is FakeScene
    assert module.roboplan_rrt.RRT is FakeRRT


def test_robot_registration_finalization_and_joint_limits(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)

    assert world.get_robot_ids() == [robot_id]
    assert world.get_robot_config(robot_id) is robot_config
    assert world._scene is None
    assert not world.is_finalized

    lower, upper = world.get_joint_limits(robot_id)
    np.testing.assert_allclose(lower, [-1.0, -2.0])
    np.testing.assert_allclose(upper, [1.0, 2.0])

    world.finalize()
    assert world._scene.constructor_args[0] == "arm"
    assert Path(world._scene.constructor_args[1]).suffix == ".urdf"
    assert Path(world._scene.constructor_args[2]).suffix == ".srdf"
    srdf_text = Path(world._scene.constructor_args[2]).read_text()
    assert '<group name="manipulator">' in srdf_text
    assert 'disable_collisions link1="base" link2="link1"' in srdf_text
    assert world.is_finalized


def test_scene_joint_limits_are_reordered_to_configured_joint_order(
    fake_roboplan: None, robot_config: RobotModelConfig, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = robot_config.model_copy(
        update={"joint_limits_lower": None, "joint_limits_upper": None}
    )
    monkeypatch.setattr(FakeScene, "joint_group_joint_names", ["joint2", "joint1"])
    monkeypatch.setattr(FakeScene, "position_limits_lower", [-2.0, -1.0])
    monkeypatch.setattr(FakeScene, "position_limits_upper", [2.0, 1.0])

    world, robot_id = _make_world(fake_roboplan, config)
    with pytest.raises(RuntimeError, match="joint limits are unavailable until world is finalized"):
        world.get_joint_limits(robot_id)

    world.finalize()

    lower, upper = world.get_joint_limits(robot_id)
    np.testing.assert_allclose(lower, [-1.0, -2.0])
    np.testing.assert_allclose(upper, [1.0, 2.0])


def test_scene_joint_limits_validate_joint_names(
    fake_roboplan: None, robot_config: RobotModelConfig, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = robot_config.model_copy(
        update={"joint_limits_lower": None, "joint_limits_upper": None}
    )
    monkeypatch.setattr(FakeScene, "joint_group_joint_names", ["joint2", "extra_joint"])

    world, _robot_id = _make_world(fake_roboplan, config)
    with pytest.raises(ValueError, match="planning group joint names do not match"):
        world.finalize()


def test_multiple_robots_can_register_before_composite_finalization(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld

    second_model = Path(robot_config.model_path).with_name("robot2.urdf")
    second_model.write_text(Path(robot_config.model_path).read_text())
    second_config = robot_config.model_copy(
        update={"name": "right_arm", "model_path": second_model}
    )

    world = RoboPlanWorld()
    first_id = world.add_robot(robot_config)
    second_id = world.add_robot(second_config)

    assert world.get_robot_ids() == [first_id, second_id]
    assert world._scene is None
    assert not world.is_finalized
    world.finalize()
    assert world.is_finalized
    assert world._scene.constructor_args[0] == "dimos_composite"
    urdf_text = Path(world._scene.constructor_args[1]).read_text()
    srdf_text = Path(world._scene.constructor_args[2]).read_text()
    assert "arm__joint1" in urdf_text
    assert "right_arm__joint1" in urdf_text
    assert 'material name="arm__Black"' in urdf_text
    assert 'material name="right_arm__Black"' in urdf_text
    assert 'material name="Black"' not in urdf_text
    assert "_dimos_composite__arm_manipulator__right_arm_manipulator" in srdf_text


def test_composite_urdf_strips_model_world_joint_before_base_pose_attachment(
    fake_roboplan: None, tmp_path: Path
) -> None:
    from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld

    model_path = tmp_path / "world_joint_robot.urdf"
    model_path.write_text(
        """
        <robot name="fake">
          <link name="world"/>
          <link name="base"/>
          <link name="link1"/>
          <joint name="world_to_base" type="fixed">
            <parent link="world"/>
            <child link="base"/>
          </joint>
          <joint name="joint1" type="revolute">
            <parent link="base"/>
            <child link="link1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>
        """
    )
    left_config = RobotModelConfig(
        name="arm",
        model_path=model_path,
        base_pose=PoseStamped(position=Vector3(x=0.1), orientation=Quaternion()),  # type: ignore[call-arg]
        strip_model_world_joint=True,
        joint_names=["joint1"],
        base_link="base",
        end_effector_link="link1",
        joint_limits_lower=[-1.0],
        joint_limits_upper=[1.0],
    )
    right_model_path = tmp_path / "right_world_joint_robot.urdf"
    right_model_path.write_text(model_path.read_text())
    right_config = left_config.model_copy(
        update={
            "name": "right_arm",
            "model_path": right_model_path,
            "base_pose": PoseStamped(position=Vector3(x=-0.1), orientation=Quaternion()),  # type: ignore[call-arg]
        }
    )

    world = RoboPlanWorld()
    world.add_robot(left_config)
    world.add_robot(right_config)
    world.finalize()

    urdf_text = Path(world._scene.constructor_args[1]).read_text()
    assert "arm__world_to_base" not in urdf_text
    assert "right_arm__world_to_base" not in urdf_text
    assert 'parent link="dimos_world"' in urdf_text
    assert 'child link="arm__base"' in urdf_text
    assert 'child link="right_arm__base"' in urdf_text


def test_full_scene_q_includes_non_config_movable_joints(
    fake_roboplan: None, tmp_path: Path
) -> None:
    from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld

    model_path = tmp_path / "robot_with_gripper.urdf"
    model_path.write_text(
        """
        <robot name="fake">
          <link name="base"/>
          <link name="link1"/>
          <link name="finger"/>
          <joint name="joint1" type="revolute">
            <parent link="base"/>
            <child link="link1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
          <joint name="gripper_joint" type="prismatic">
            <parent link="link1"/>
            <child link="finger"/>
            <limit lower="0" upper="1" effort="1" velocity="1"/>
          </joint>
          <link name="mimic_finger"/>
          <joint name="mimic_gripper_joint" type="prismatic">
            <parent link="link1"/>
            <child link="mimic_finger"/>
            <mimic joint="gripper_joint" multiplier="1" offset="0"/>
            <limit lower="0" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>
        """
    )
    left_config = RobotModelConfig(
        name="arm",
        model_path=model_path,
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        joint_names=["joint1"],
        base_link="base",
        end_effector_link="link1",
        joint_limits_lower=[-1.0],
        joint_limits_upper=[1.0],
    )
    right_model_path = tmp_path / "right_robot_with_gripper.urdf"
    right_model_path.write_text(model_path.read_text())
    right_config = left_config.model_copy(
        update={"name": "right_arm", "model_path": right_model_path}
    )

    world = RoboPlanWorld()
    left_id = world.add_robot(left_config)
    right_id = world.add_robot(right_config)
    world.finalize()
    ctx = world.get_live_context()
    world.set_joint_state(ctx, left_id, JointState(name=[], position=[0.1]))
    world.set_joint_state(ctx, right_id, JointState(name=[], position=[0.2]))

    assert world._full_native_joint_names == (
        "arm__joint1",
        "arm__gripper_joint",
        "right_arm__joint1",
        "right_arm__gripper_joint",
    )
    np.testing.assert_allclose(world._full_scene_q(ctx), [0.1, 0.0, 0.2, 0.0])


def test_composite_native_planner_sets_scene_state_and_returns_caller_order(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld

    second_model = Path(robot_config.model_path).with_name("robot2.urdf")
    second_model.write_text(Path(robot_config.model_path).read_text())
    right_config = robot_config.model_copy(update={"name": "right_arm", "model_path": second_model})
    world = RoboPlanWorld()
    left_id = world.add_robot(robot_config)
    right_id = world.add_robot(right_config)
    world.finalize()
    ctx = world.get_live_context()
    world.set_joint_state(ctx, left_id, JointState(name=[], position=[0.1, 0.2]))
    world.set_joint_state(ctx, right_id, JointState(name=[], position=[0.3, 0.4]))
    selection = world._planning_groups.select(["right_arm/manipulator", "arm/manipulator"])

    result = world.plan_selected_joint_path(
        world,
        selection,
        JointState(
            name=["right_arm/joint1", "right_arm/joint2", "arm/joint1", "arm/joint2"],
            position=[0.3, 0.4, 0.1, 0.2],
        ),
        JointState(
            name=["right_arm/joint1", "right_arm/joint2", "arm/joint1", "arm/joint2"],
            position=[0.5, 0.6, 0.7, 0.8],
        ),
        timeout=1.0,
    )

    assert result.status == PlanningStatus.SUCCESS
    assert FakeRRT.last_options is not None
    assert (
        FakeRRT.last_options.group_name
        == "_dimos_composite__arm_manipulator__right_arm_manipulator"
    )
    assert FakeRRT.last_start is not None
    assert FakeRRT.last_start.joint_names == [
        "arm__joint1",
        "arm__joint2",
        "right_arm__joint1",
        "right_arm__joint2",
    ]
    np.testing.assert_allclose(FakeRRT.last_start.positions, [0.1, 0.2, 0.3, 0.4])
    np.testing.assert_allclose(
        world._scene.current_positions,
        [0.1, 0.2, 0.3, 0.4],
    )
    assert result.path[0].name == [
        "right_arm/joint1",
        "right_arm/joint2",
        "arm/joint1",
        "arm/joint2",
    ]
    assert result.path[-1].position == [0.5, 0.6, 0.7, 0.8]


def test_composite_collision_checks_use_scratch_context_for_all_robots(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld

    second_model = Path(robot_config.model_path).with_name("robot2.urdf")
    second_model.write_text(Path(robot_config.model_path).read_text())
    right_config = robot_config.model_copy(update={"name": "right_arm", "model_path": second_model})
    world = RoboPlanWorld()
    left_id = world.add_robot(robot_config)
    right_id = world.add_robot(right_config)
    world.finalize()

    def collides_when_both_robots_move(q: np.ndarray) -> bool:
        return bool(q[0] > 0.7 and q[2] > 0.7)

    world._scene.hasCollisions = collides_when_both_robots_move
    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, left_id, JointState(name=[], position=[0.8, 0.0]))
        world.set_joint_state(ctx, right_id, JointState(name=[], position=[0.8, 0.0]))

        assert not world.is_collision_free(ctx, left_id)
        assert not world.is_collision_free(ctx, right_id)


def test_native_planner_rejects_returned_path_that_collides(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class CollidingPathRRT(FakeRRT):
        def plan(
            self, q_start: FakeJointConfiguration, q_goal: FakeJointConfiguration
        ) -> FakeJointPath:
            FakeRRT.last_start = q_start
            FakeRRT.last_goal = q_goal
            return FakeJointPath(
                q_start.joint_names,
                [
                    np.asarray(q_start.positions),
                    np.asarray([0.95, 0.0]),
                    np.asarray(q_goal.positions),
                ],
            )

    monkeypatch.setattr(sys.modules["roboplan.rrt"], "RRT", CollidingPathRRT)
    world, _robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()
    selection = _default_selection(world, robot_config)

    result = world.plan_selected_joint_path(
        world,
        selection,
        JointState(name=["arm/joint1", "arm/joint2"], position=[0.0, 0.0]),
        JointState(name=["arm/joint1", "arm/joint2"], position=[0.2, 0.2]),
        timeout=1.0,
    )

    assert result.status == PlanningStatus.NO_SOLUTION
    assert "path is in collision" in result.message


def test_selected_start_mismatch_returns_invalid_start(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()
    selection = _default_selection(world, robot_config)

    result = world.plan_selected_joint_path(
        world,
        selection,
        JointState(name=["arm/joint1", "arm/joint2"], position=[0.1, 0.0]),
        JointState(name=["arm/joint1", "arm/joint2"], position=[0.2, 0.0]),
        timeout=1.0,
    )

    assert result.status == PlanningStatus.INVALID_START
    assert "does not match" in result.message


def test_composite_group_generation_cap_is_enforced(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld

    second_model = Path(robot_config.model_path).with_name("robot2.urdf")
    second_model.write_text(Path(robot_config.model_path).read_text())
    right_config = robot_config.model_copy(update={"name": "right_arm", "model_path": second_model})
    world = RoboPlanWorld(max_generated_composite_groups=0)
    world.add_robot(robot_config)
    world.add_robot(right_config)

    with pytest.raises(ValueError, match="max_generated_composite_groups"):
        world.finalize()


def test_context_cloning_and_joint_state_round_trip(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()

    live_state = JointState(name=["joint1", "joint2"], position=[0.1, 0.2])
    world.sync_from_joint_state(robot_id, live_state)

    with world.scratch_context() as scratch:
        scratch_state = world.get_joint_state(scratch, robot_id)
        assert scratch_state.name == ["joint1", "joint2"]
        assert scratch_state.position == [0.1, 0.2]
        world.set_joint_state(
            scratch, robot_id, JointState(name=["joint1", "joint2"], position=[0.3, 0.4])
        )

    live_round_trip = world.get_joint_state(world.get_live_context(), robot_id)
    assert live_round_trip.position == [0.1, 0.2]


def test_global_joint_names_are_applied_to_input_states(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()

    world.sync_from_joint_state(
        robot_id, JointState(name=["arm/joint1", "arm/joint2"], position=[0.2, 0.3])
    )

    live_round_trip = world.get_joint_state(world.get_live_context(), robot_id)
    assert live_round_trip.position == [0.2, 0.3]


def test_obstacle_mutation_updates_scene_and_stored_pose(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)
    world.finalize()

    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )
    assert world.add_obstacle(obstacle) == "box"
    assert "box" in world._scene.geometry
    updated_pose = PoseStamped(position=Vector3(1, 0, 0), orientation=Quaternion())  # type: ignore[call-arg]
    assert world.update_obstacle_pose(
        "box",
        updated_pose,
    )
    assert world.get_obstacles()[0].pose is updated_pose
    np.testing.assert_allclose(world._scene.geometry["box"], pose_to_matrix(updated_pose))
    assert world.remove_obstacle("box")
    assert world.get_obstacles() == []


def test_collision_config_and_edge_checks(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()

    safe = JointState(name=["joint1", "joint2"], position=[0.1, 0.2])
    colliding = JointState(name=["joint1", "joint2"], position=[0.95, 0.2])

    assert world.check_config_collision_free(robot_id, safe)
    assert not world.check_config_collision_free(robot_id, colliding)
    assert not world.check_edge_collision_free(robot_id, safe, colliding, step_size=0.05)


def test_collision_check_uses_scene_queries(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()

    safe = JointState(name=["joint1", "joint2"], position=[0.1, 0.2])
    colliding = JointState(name=["joint1", "joint2"], position=[0.95, 0.2])

    assert world.check_config_collision_free(robot_id, safe)
    assert not world.check_config_collision_free(robot_id, colliding)


def test_generic_rrt_planner_uses_roboplan_world_collision_checks(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner

    world, robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()
    planner = RRTConnectPlanner(step_size=0.5, connect_step_size=0.5, goal_tolerance=10.0)

    start = JointState(name=["joint1", "joint2"], position=[0.0, 0.0])
    goal = JointState(name=["joint1", "joint2"], position=[0.2, 0.1])
    result = planner.plan_joint_path(world, robot_id, start, goal, timeout=1.0, max_iterations=3)

    assert result.status == PlanningStatus.SUCCESS
    assert len(result.path) >= 2


def test_group_fk_jacobian_and_explicit_min_distance_unsupported(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()
    ctx = world.get_live_context()
    world.set_joint_state(
        ctx, robot_id, JointState(name=["joint1", "joint2"], position=[0.25, 0.5])
    )

    group_id = f"{robot_config.name}/manipulator"
    pose = world.get_group_ee_pose(ctx, group_id)
    assert pose.position.x == pytest.approx(0.75)
    assert world.get_group_jacobian(ctx, group_id).shape == (6, 2)
    assert not hasattr(world, "get_ee_pose")
    assert not hasattr(world, "get_jacobian")
    with pytest.raises(NotImplementedError, match="get_min_distance"):
        world.get_min_distance(ctx, robot_id)


def test_selected_native_planner_converts_path(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()

    selection = _default_selection(world, robot_config)
    start = JointState(name=["arm/joint1", "arm/joint2"], position=[0.0, 0.0])
    goal = JointState(name=["arm/joint1", "arm/joint2"], position=[0.4, 0.2])
    result = world.plan_selected_joint_path(world, selection, start, goal, timeout=1.0)

    assert result.status == PlanningStatus.SUCCESS
    assert [state.position for state in result.path] == [[0.0, 0.0], [0.2, 0.1], [0.4, 0.2]]
    assert [state.name for state in result.path] == [["arm/joint1", "arm/joint2"]] * 3
    assert FakeRRT.last_options is not None
    assert FakeRRT.last_options.group_name == "manipulator"
    assert FakeRRT.last_options.collision_check_use_bisection is True
    assert FakeRRT.last_options.collision_check_step_size == pytest.approx(0.02)


def test_robot_scoped_native_planner_satisfies_planner_spec(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()

    start = JointState(name=["joint1", "joint2"], position=[0.0, 0.0])
    goal = JointState(name=["joint1", "joint2"], position=[0.4, 0.2])
    result = world.plan_joint_path(world, robot_id, start, goal, timeout=1.0)

    assert result.status == PlanningStatus.SUCCESS
    assert [state.position for state in result.path] == [[0.0, 0.0], [0.2, 0.1], [0.4, 0.2]]
    assert [state.name for state in result.path] == [["joint1", "joint2"]] * 3
    assert FakeRRT.last_options is not None
    assert FakeRRT.last_options.group_name == "manipulator"


def test_cartesian_free_requires_optional_simple_ik(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()
    selection = _default_selection(world, robot_config)

    result = world.plan_cartesian_path(
        world,
        CartesianPlanningRequest(
            selection=selection,
            group_id="arm/manipulator",
            start=JointState(name=["arm/joint1", "arm/joint2"], position=[0.0, 0.0]),
            target=PoseStamped(frame_id="world", position=Vector3(0.2, 0.0, 0.0)),  # type: ignore[call-arg]
            target_mode="absolute",
        ),
    )

    assert result.status == PlanningStatus.UNSUPPORTED
    assert "simple_ik" in result.message


def test_cartesian_free_absolute_uses_simple_ik_then_rrt(
    monkeypatch: pytest.MonkeyPatch, robot_config: RobotModelConfig
) -> None:
    _install_fake_roboplan(monkeypatch, include_simple_ik=True)
    module = _import_roboplan_world(None)
    FakeRRT.last_start = None
    FakeRRT.last_goal = None
    FakeSimpleIk.last_goal = None
    world = module.RoboPlanWorld()
    robot_id = world.add_robot(robot_config)
    world.finalize()
    world.set_joint_state(
        world.get_live_context(), robot_id, JointState(name=[], position=[0.8, 0.1])
    )
    selection = _default_selection(world, robot_config)

    result = world.plan_cartesian_path(
        world,
        CartesianPlanningRequest(
            selection=selection,
            group_id="arm/manipulator",
            start=JointState(name=["arm/joint1", "arm/joint2"], position=[0.1, 0.1]),
            target=PoseStamped(frame_id="world", position=Vector3(0.4, 0.0, 0.0)),  # type: ignore[call-arg]
            target_mode="absolute",
            timeout=2.0,
        ),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert [state.name for state in result.path] == [["arm/joint1", "arm/joint2"]] * 3
    assert result.path[-1].position == [0.2, 0.2]
    assert FakeSimpleIk.last_options is not None
    assert FakeSimpleIk.last_options.max_time == pytest.approx(2.0)
    assert FakeSimpleIk.last_goal is not None
    assert FakeSimpleIk.last_goal.frame_name == "tcp"
    assert FakeRRT.last_start is not None
    np.testing.assert_allclose(FakeRRT.last_start.positions, [0.1, 0.1])
    np.testing.assert_allclose(world._scene.current_positions, [0.1, 0.1])


def test_cartesian_free_relative_delta_uses_world_axes(
    monkeypatch: pytest.MonkeyPatch, robot_config: RobotModelConfig
) -> None:
    _install_fake_roboplan(monkeypatch, include_simple_ik=True)
    module = _import_roboplan_world(None)
    rotation_z_90 = np.asarray(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    monkeypatch.setattr(FakeScene, "fk_rotation", rotation_z_90)
    FakeSimpleIk.last_goal = None
    world = module.RoboPlanWorld()
    world.add_robot(robot_config)
    world.finalize()
    selection = _default_selection(world, robot_config)

    result = world.plan_cartesian_path(
        world,
        CartesianPlanningRequest(
            selection=selection,
            group_id="arm/manipulator",
            start=JointState(name=["arm/joint1", "arm/joint2"], position=[0.1, 0.2]),
            target=CartesianDelta(translation=(0.2, 0.0, 0.0), frame_id="world"),
            target_mode="relative",
        ),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert FakeSimpleIk.last_goal is not None
    np.testing.assert_allclose(FakeSimpleIk.last_goal.matrix[:3, 3], [0.5, 0.0, 0.0])
    np.testing.assert_allclose(FakeSimpleIk.last_goal.matrix[:3, :3], rotation_z_90, atol=1e-12)


def test_cartesian_free_relative_delta_composes_world_rotation(
    monkeypatch: pytest.MonkeyPatch, robot_config: RobotModelConfig
) -> None:
    _install_fake_roboplan(monkeypatch, include_simple_ik=True)
    module = _import_roboplan_world(None)
    rotation_z_90 = np.asarray(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    rotation_z_180 = np.asarray(
        [
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    monkeypatch.setattr(FakeScene, "fk_rotation", rotation_z_90)
    FakeSimpleIk.last_goal = None
    world = module.RoboPlanWorld()
    world.add_robot(robot_config)
    world.finalize()
    selection = _default_selection(world, robot_config)

    result = world.plan_cartesian_path(
        world,
        CartesianPlanningRequest(
            selection=selection,
            group_id="arm/manipulator",
            start=JointState(name=["arm/joint1", "arm/joint2"], position=[0.1, 0.2]),
            target=CartesianDelta(rotation_rpy=(0.0, 0.0, np.pi / 2.0), frame_id="world"),
            target_mode="relative",
        ),
    )

    assert result.status == PlanningStatus.NO_SOLUTION
    assert "final TCP pose missed target" in result.message
    assert FakeSimpleIk.last_goal is not None
    np.testing.assert_allclose(FakeSimpleIk.last_goal.matrix[:3, :3], rotation_z_180, atol=1e-12)


def test_cartesian_free_malformed_start_returns_invalid_start(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()
    selection = _default_selection(world, robot_config)

    result = world.plan_cartesian_path(
        world,
        CartesianPlanningRequest(
            selection=selection,
            group_id="arm/manipulator",
            start=JointState(name=["arm/joint1"], position=[0.0]),
            target=PoseStamped(frame_id="world", position=Vector3(0.4, 0.0, 0.0)),  # type: ignore[call-arg]
            target_mode="absolute",
        ),
    )

    assert result.status == PlanningStatus.INVALID_START


def test_cartesian_free_rejects_final_tcp_mismatch(
    monkeypatch: pytest.MonkeyPatch, robot_config: RobotModelConfig
) -> None:
    _install_fake_roboplan(monkeypatch, include_simple_ik=True)

    class MismatchedPoseSimpleIk(FakeSimpleIk):
        def solveIk(
            self,
            goal: FakeCartesianConfiguration,
            start: FakeJointConfiguration,
            solution: FakeJointConfiguration,
        ) -> bool:
            FakeSimpleIk.last_goal = goal
            FakeSimpleIk.last_start = start
            solution.joint_names = list(start.joint_names)
            solution.positions = np.zeros(len(start.joint_names), dtype=np.float64)
            return True

    monkeypatch.setattr(sys.modules["roboplan.simple_ik"], "SimpleIk", MismatchedPoseSimpleIk)
    module = _import_roboplan_world(None)
    world = module.RoboPlanWorld()
    world.add_robot(robot_config)
    world.finalize()
    selection = _default_selection(world, robot_config)

    result = world.plan_cartesian_path(
        world,
        CartesianPlanningRequest(
            selection=selection,
            group_id="arm/manipulator",
            start=JointState(name=["arm/joint1", "arm/joint2"], position=[0.1, 0.1]),
            target=PoseStamped(frame_id="world", position=Vector3(0.4, 0.0, 0.0)),  # type: ignore[call-arg]
            target_mode="absolute",
        ),
    )

    assert result.status == PlanningStatus.NO_SOLUTION
    assert "final TCP pose missed target" in result.message


def test_cartesian_free_rejects_coupled_selection_for_v1(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()
    group = world._planning_groups.get("arm/manipulator")
    selection = PlanningGroupSelection(
        groups=(group,),
        group_ids=(group.id, "other/manipulator"),
        joint_names=group.joint_names,
        robot_names=(group.robot_name,),
    )

    result = world.plan_cartesian_path(
        world,
        CartesianPlanningRequest(
            selection=selection,
            group_id="arm/manipulator",
            start=JointState(name=["arm/joint1", "arm/joint2"], position=[0.0, 0.0]),
            target=PoseStamped(frame_id="world", position=Vector3(0.4, 0.0, 0.0)),  # type: ignore[call-arg]
            target_mode="absolute",
        ),
    )

    assert result.status == PlanningStatus.UNSUPPORTED
    assert "exactly one selected TCP group" in result.message


def test_cartesian_linear_mode_is_explicitly_unsupported_without_free_fallback(
    monkeypatch: pytest.MonkeyPatch, robot_config: RobotModelConfig
) -> None:
    _install_fake_roboplan(monkeypatch, include_simple_ik=True)
    module = _import_roboplan_world(None)
    FakeRRT.last_start = None
    FakeSimpleIk.last_goal = None
    world = module.RoboPlanWorld()
    world.add_robot(robot_config)
    world.finalize()
    selection = _default_selection(world, robot_config)

    result = world.plan_cartesian_path(
        world,
        CartesianPlanningRequest(
            selection=selection,
            group_id="arm/manipulator",
            start=JointState(name=["arm/joint1", "arm/joint2"], position=[0.0, 0.0]),
            target=PoseStamped(frame_id="world", position=Vector3(0.4, 0.0, 0.0)),  # type: ignore[call-arg]
            target_mode="absolute",
            path_mode="linear",
        ),
    )

    assert result.status == PlanningStatus.UNSUPPORTED
    assert "Linear Cartesian planning" in result.message
    assert FakeSimpleIk.last_goal is None
    assert FakeRRT.last_start is None


@pytest.mark.parametrize(
    ("request_update", "expected_status", "expected_message"),
    [
        ({"target_mode": "nonsense"}, PlanningStatus.INVALID_GOAL, "target_mode"),
        ({"path_mode": "nonsense"}, PlanningStatus.INVALID_GOAL, "path_mode"),
        ({"reference_frame": "tool"}, PlanningStatus.UNSUPPORTED, "reference_frame"),
        ({"max_translation_step": 0.0}, PlanningStatus.INVALID_GOAL, "max_translation_step"),
        ({"max_rotation_step": 0.0}, PlanningStatus.INVALID_GOAL, "max_rotation_step"),
        ({"timeout": 0.0}, PlanningStatus.INVALID_GOAL, "timeout"),
        (
            {"target": CartesianDelta(), "target_mode": "absolute"},
            PlanningStatus.INVALID_GOAL,
            "PoseStamped",
        ),
        (
            {"target": PoseStamped(frame_id="map"), "target_mode": "absolute"},
            PlanningStatus.UNSUPPORTED,
            "world-frame poses",
        ),
        (
            {"target": PoseStamped(frame_id="world"), "target_mode": "relative"},
            PlanningStatus.INVALID_GOAL,
            "CartesianDelta",
        ),
        (
            {"target": CartesianDelta(frame_id="tool"), "target_mode": "relative"},
            PlanningStatus.UNSUPPORTED,
            "world-frame deltas",
        ),
    ],
)
def test_cartesian_request_validation_branches(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    request_update: dict[str, object],
    expected_status: PlanningStatus,
    expected_message: str,
) -> None:
    world, _robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()
    selection = _default_selection(world, robot_config)
    request_kwargs = {
        "selection": selection,
        "group_id": "arm/manipulator",
        "start": JointState(name=["arm/joint1", "arm/joint2"], position=[0.0, 0.0]),
        "target": PoseStamped(frame_id="world", position=Vector3(0.2, 0.0, 0.0)),
        "target_mode": "absolute",
    }
    request_kwargs.update(request_update)

    result = world.plan_cartesian_path(world, CartesianPlanningRequest(**request_kwargs))

    assert result.status == expected_status
    assert expected_message in result.message


def test_selected_native_planner_handles_unnamed_group_states(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()

    selection = _default_selection(world, robot_config)
    start = JointState(name=[], position=[0.0, 0.0])
    goal = JointState(name=["arm/joint1", "arm/joint2"], position=[0.4, 0.2])
    result = world.plan_selected_joint_path(world, selection, start, goal, timeout=1.0)

    assert result.status == PlanningStatus.SUCCESS
    assert [state.name for state in result.path] == [["arm/joint1", "arm/joint2"]] * 3


def test_native_planner_rejects_empty_path(
    fake_roboplan: None, robot_config: RobotModelConfig, monkeypatch: pytest.MonkeyPatch
) -> None:
    class EmptyPathRRT(FakeRRT):
        def plan(
            self, q_start: FakeJointConfiguration, q_goal: FakeJointConfiguration
        ) -> FakeJointPath:
            _ = (q_start, q_goal)
            return FakeJointPath(["joint1", "joint2"], [])

    monkeypatch.setattr(sys.modules["roboplan.rrt"], "RRT", EmptyPathRRT)
    world, _robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()

    selection = _default_selection(world, robot_config)
    start = JointState(name=["arm/joint1", "arm/joint2"], position=[0.0, 0.0])
    goal = JointState(name=["arm/joint1", "arm/joint2"], position=[0.4, 0.2])
    result = world.plan_selected_joint_path(world, selection, start, goal, timeout=1.0)

    assert result.status == PlanningStatus.NO_SOLUTION
    assert result.path == []
    assert "empty path" in result.message


def test_selected_native_planner_reorders_native_group_order(
    fake_roboplan: None, robot_config: RobotModelConfig, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(FakeScene, "joint_group_joint_names", ["joint2", "joint1"])
    world, robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()
    ctx = world.get_live_context()
    world.set_joint_state(
        ctx, robot_id, JointState(name=["joint1", "joint2"], position=[0.25, 0.5])
    )

    group_id = f"{robot_config.name}/manipulator"
    jacobian = world.get_group_jacobian(ctx, group_id)
    np.testing.assert_allclose(jacobian[0], [2.0, 1.0])

    selection = _default_selection(world, robot_config)
    start = JointState(name=["arm/joint1", "arm/joint2"], position=[0.1, 0.2])
    goal = JointState(name=["arm/joint1", "arm/joint2"], position=[0.3, 0.4])
    world.set_joint_state(
        ctx, robot_id, JointState(name=["joint1", "joint2"], position=start.position)
    )
    result = world.plan_selected_joint_path(world, selection, start, goal, timeout=1.0)

    assert result.status == PlanningStatus.SUCCESS
    assert FakeRRT.last_start is not None
    np.testing.assert_allclose(FakeRRT.last_start.positions, [0.2, 0.1])
    assert FakeRRT.last_start.joint_names == ["joint2", "joint1"]
    np.testing.assert_allclose(
        [state.position for state in result.path], [[0.1, 0.2], [0.2, 0.3], [0.3, 0.4]]
    )
    assert [state.name for state in result.path] == [["arm/joint1", "arm/joint2"]] * 3


def test_selected_native_planner_rejects_non_matching_selection(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _robot_id = _make_world(fake_roboplan, robot_config)
    world.finalize()
    group = world._planning_groups.get("arm/manipulator")
    selection = PlanningGroupSelection(
        groups=(group,),
        group_ids=(group.id,),
        joint_names=(group.joint_names[0],),
        robot_names=(group.robot_name,),
    )

    result = world.plan_selected_joint_path(
        world,
        selection,
        JointState(name=["arm/joint1"], position=[0.0]),
        JointState(name=["arm/joint1"], position=[0.1]),
        timeout=1.0,
    )

    assert result.status == PlanningStatus.UNSUPPORTED
    assert "exactly match" in result.message


def test_provided_srdf_path_is_passed_directly(
    fake_roboplan: None, robot_config: RobotModelConfig, tmp_path: Path
) -> None:
    srdf_path = tmp_path / "provided.srdf"
    srdf_path.write_text(
        '<robot name="arm"><group name="manipulator">'
        '<joint name="joint1"/><joint name="joint2"/>'
        "</group></robot>"
    )
    config = robot_config.model_copy(update={"srdf_path": srdf_path})

    world, _robot_id = _make_world(fake_roboplan, config)
    world.finalize()

    assert world._scene.constructor_args[2] == str(srdf_path)


def test_no_srdf_multi_group_configuration_generates_groups(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld

    config = robot_config.model_copy(
        update={
            "planning_groups": [
                PlanningGroupDefinition(
                    name="arm",
                    joint_names=("joint1",),
                    base_link="base",
                    tip_link="tcp",
                ),
                PlanningGroupDefinition(
                    name="wrist",
                    joint_names=("joint2",),
                    base_link="link1",
                    tip_link="tcp",
                ),
            ]
        }
    )

    world = RoboPlanWorld()
    world.add_robot(config)
    world.finalize()

    srdf_text = Path(world._scene.constructor_args[2]).read_text()
    assert '<group name="arm">' in srdf_text
    assert '<group name="wrist">' in srdf_text


def test_collision_exclusion_pairs_are_written_to_generated_srdf(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    robot_config.collision_exclusion_pairs = [("a", "b")]
    world, _ = _make_world(fake_roboplan, robot_config)
    world.finalize()

    srdf_path = Path(world._scene.constructor_args[2])
    assert 'disable_collisions link1="a" link2="b"' in srdf_path.read_text()


def test_generated_srdf_uses_scoped_temp_directory(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)
    world.finalize()

    srdf_path = Path(world._scene.constructor_args[2])
    assert srdf_path.parent.name.startswith("dimos_roboplan_srdf_")
    assert srdf_path.exists()
    assert world._srdf_tempdirs


@pytest.mark.parametrize(
    ("field_name", "field_value"),
    [
        (
            "base_pose",
            PoseStamped(position=Vector3(1, 0, 0), orientation=Quaternion()),
        ),  # type: ignore[call-arg]
    ],
)
def test_supported_base_pose_registers_before_planning(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    field_name: str,
    field_value: Any,
) -> None:
    from dimos.manipulation.planning.world.roboplan_world import RoboPlanWorld

    setattr(robot_config, field_name, field_value)
    world = RoboPlanWorld()
    world.add_robot(robot_config)
