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
import threading
from types import ModuleType
from typing import Any, ClassVar
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.planning.groups.models import (
    PlanningGroupDefinition,
    PlanningGroupSelection,
)
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import Obstacle
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


class FakeBox:
    def __init__(self, x: float, y: float, z: float) -> None:
        self.dimensions = (x, y, z)


class FakeSphere:
    def __init__(self, radius: float) -> None:
        self.radius = radius


class FakeCylinder:
    def __init__(self, radius: float, length: float) -> None:
        self.radius = radius
        self.length = length


class FakeMesh:
    def __init__(self, filename: str) -> None:
        self.filename = filename


class FakeJointGroupInfo:
    def __init__(self, joint_names: list[str]) -> None:
        self.joint_names = joint_names


class FakeScene:
    joint_group_joint_names: ClassVar[list[str] | None] = None
    position_limits_lower: ClassVar[list[float]] = [-1.0, -2.0]
    position_limits_upper: ClassVar[list[float]] = [1.0, 2.0]

    def __init__(self, *args: Any) -> None:
        self.constructor_args = args
        self.models: list[tuple[str, str, dict[str, str]]] = []
        self.geometry: dict[str, np.ndarray] = {}
        self.collision_settings: dict[tuple[str, str], bool] = {}
        self.groups = self._read_groups(Path(args[2]))
        self.native_joint_names = self._read_joint_names(Path(args[1]))
        self.current_positions = np.zeros(len(self.native_joint_names), dtype=np.float64)

    @staticmethod
    def _read_groups(path: Path) -> dict[str, list[str]]:
        root = ET.parse(path).getroot()
        return {
            group.get("name", ""): [joint.get("name", "") for joint in group.findall("joint")]
            for group in root.findall("group")
        }

    @staticmethod
    def _read_joint_names(path: Path) -> list[str]:
        root = ET.parse(path).getroot()
        return [
            joint.get("name", "")
            for joint in root.findall("joint")
            if joint.get("type") != "fixed" and joint.find("mimic") is None
        ]

    def addRobotModel(self, path: str, name: str, package_paths: dict[str, str]) -> str:
        self.models.append((path, name, package_paths))
        return name

    def hasCollisions(self, q: np.ndarray) -> bool:
        return bool(np.any(np.asarray(q) > 0.9))

    def getPositionLimitVectors(
        self, group_name: str = "", collapsed: bool = False
    ) -> tuple[np.ndarray, np.ndarray]:
        return np.asarray(self.position_limits_lower), np.asarray(self.position_limits_upper)

    def getJointGroupInfo(self, name: str) -> FakeJointGroupInfo:
        names = (
            self.joint_group_joint_names
            if self.joint_group_joint_names is not None and name == self.constructor_args[0]
            else self.groups[name]
        )
        return FakeJointGroupInfo(list(names))

    def toFullJointPositions(self, group_name: str, q: np.ndarray) -> np.ndarray:
        full = self.current_positions.copy()
        for name, value in zip(self.groups[group_name], q, strict=True):
            full[self.native_joint_names.index(name)] = value
        return full

    def setJointPositions(self, q: np.ndarray) -> None:
        self.current_positions = np.asarray(q, dtype=np.float64)

    def getJointNames(self) -> list[str]:
        return list(self.native_joint_names)

    def addBoxGeometry(
        self,
        obstacle_id: str,
        parent_frame: str,
        box: FakeBox,
        matrix: np.ndarray,
        color: np.ndarray,
    ) -> None:
        self.geometry[obstacle_id] = matrix

    def addSphereGeometry(
        self,
        obstacle_id: str,
        parent_frame: str,
        sphere: FakeSphere,
        matrix: np.ndarray,
        color: np.ndarray,
    ) -> None:
        self.geometry[obstacle_id] = matrix

    def addCylinderGeometry(
        self,
        obstacle_id: str,
        parent_frame: str,
        cylinder: FakeCylinder,
        matrix: np.ndarray,
        color: np.ndarray,
    ) -> None:
        self.geometry[obstacle_id] = matrix

    def addMeshGeometry(
        self,
        obstacle_id: str,
        parent_frame: str,
        mesh: FakeMesh,
        matrix: np.ndarray,
        color: np.ndarray,
    ) -> None:
        self.geometry[obstacle_id] = matrix

    def updateGeometryPlacement(
        self, obstacle_id: str, parent_frame: str, matrix: np.ndarray
    ) -> None:
        self.geometry[obstacle_id] = matrix

    def removeGeometry(self, obstacle_id: str) -> None:
        del self.geometry[obstacle_id]

    def setCollisions(self, body1: str, body2: str, enable: bool) -> None:
        self.collision_settings[(body1, body2)] = enable

    def forwardKinematics(self, q: np.ndarray, frame_name: str, base_frame: str = "") -> np.ndarray:
        mat = np.eye(4)
        mat[0, 3] = float(np.sum(q))
        return mat

    def computeFrameJacobian(
        self, q: np.ndarray, frame_name: str, local: bool = True
    ) -> np.ndarray:
        return np.ones((6, len(self.native_joint_names)))


class FakeRRTOptions:
    def __init__(self) -> None:
        self.group_name = ""
        self.max_planning_time = 0.0
        self.max_nodes = 0
        self.collision_check_use_bisection = True


class FakeRRT:
    def __init__(self, scene: FakeScene, options: FakeRRTOptions) -> None:
        self.scene = scene
        self.options = options

    def plan(
        self, q_start: FakeJointConfiguration, q_goal: FakeJointConfiguration
    ) -> FakeJointPath:
        assert isinstance(q_start, FakeJointConfiguration)
        assert isinstance(q_goal, FakeJointConfiguration)
        midpoint = (np.asarray(q_start.positions) + np.asarray(q_goal.positions)) / 2.0
        return FakeJointPath(
            q_start.joint_names,
            [np.asarray(q_start.positions), midpoint, np.asarray(q_goal.positions)],
        )


def _install_fake_roboplan(monkeypatch: pytest.MonkeyPatch) -> None:
    roboplan_pkg = ModuleType("roboplan")
    roboplan_pkg.__path__ = []  # type: ignore[attr-defined]
    core = ModuleType("roboplan.core")
    core.Scene = FakeScene  # type: ignore[attr-defined]
    core.JointConfiguration = FakeJointConfiguration  # type: ignore[attr-defined]
    core.JointPath = FakeJointPath  # type: ignore[attr-defined]
    core.Box = FakeBox  # type: ignore[attr-defined]
    core.Sphere = FakeSphere  # type: ignore[attr-defined]
    core.Cylinder = FakeCylinder  # type: ignore[attr-defined]
    core.Mesh = FakeMesh  # type: ignore[attr-defined]

    def has_collisions_along_path(
        scene: FakeScene,
        q_start: np.ndarray,
        q_end: np.ndarray,
        max_step_size: float,
        bisection: bool = False,
        check_endpoints: bool = True,
    ) -> bool:
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


@pytest.fixture
def fake_roboplan(monkeypatch: pytest.MonkeyPatch) -> None:
    _install_fake_roboplan(monkeypatch)


@pytest.fixture
def robot_config(tmp_path: Path) -> RobotModelConfig:
    model_path = tmp_path / "robot.urdf"
    model_path.write_text(
        """
        <robot name="fake">
          <link name="base"/>
          <link name="link1"/>
          <link name="link2"/>
          <link name="link3"/>
          <link name="tcp"/>
          <joint name="joint1" type="revolute">
            <parent link="base"/>
            <child link="link1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
          <joint name="joint2" type="revolute">
            <parent link="link1"/>
            <child link="link2"/>
            <limit lower="-2" upper="2" effort="1" velocity="1"/>
          </joint>
          <joint name="joint3" type="revolute">
            <parent link="link2"/>
            <child link="link3"/>
            <limit lower="-3" upper="3" effort="1" velocity="1"/>
          </joint>
          <joint name="tcp_fixed" type="fixed">
            <parent link="link3"/>
            <child link="tcp"/>
          </joint>
        </robot>
        """
    )
    return RobotModelConfig(
        name="arm",
        model_path=model_path,
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        joint_names=["joint1", "joint2"],
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint1", "joint2"),
                base_link="base",
                tip_link="tcp",
            )
        ],
        joint_limits_lower=[-1.0, -2.0],
        joint_limits_upper=[1.0, 2.0],
    )


def _make_world(fake_roboplan: None, robot_config: RobotModelConfig) -> tuple[Any, str]:
    module = _import_roboplan_world(fake_roboplan)

    world = module.RoboPlanWorld()
    robot_id = world.add_robot(robot_config)
    world.finalize()
    world.sync_from_joint_state(
        robot_id,
        JointState(
            name=list(robot_config.joint_names),
            position=[0.0] * len(robot_config.joint_names),
        ),
    )
    return world, robot_id


def _make_two_robot_world(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> tuple[Any, str, str, RobotModelConfig]:
    module = _import_roboplan_world(fake_roboplan)
    second_config = robot_config.model_copy(
        update={
            "name": "right",
            "base_pose": PoseStamped(
                position=Vector3(1, 0, 0),
                orientation=Quaternion(),
            ),
        }
    )
    world = module.RoboPlanWorld()
    first_id = world.add_robot(robot_config)
    second_id = world.add_robot(second_config)
    world.finalize()
    world.sync_from_joint_state(
        first_id,
        JointState(name=list(robot_config.joint_names), position=[0.0, 0.0]),
    )
    world.sync_from_joint_state(
        second_id,
        JointState(name=list(second_config.joint_names), position=[0.0, 0.0]),
    )
    return world, first_id, second_id, second_config


def _selection(
    configs: tuple[RobotModelConfig, ...],
    *group_ids: str,
) -> PlanningGroupSelection:
    registry = PlanningGroupRegistry(configs)
    return PlanningGroupSelection.from_groups(
        tuple(registry.get(group_id) for group_id in group_ids)
    )


def _import_roboplan_world(fake_roboplan: None) -> ModuleType:
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
    assert world._scene.constructor_args[0] == "arm"
    assert Path(world._scene.constructor_args[1]).suffix == ".urdf"
    assert Path(world._scene.constructor_args[2]).suffix == ".srdf"
    assert (
        'disable_collisions link1="base" link2="link1"'
        in Path(world._scene.constructor_args[2]).read_text()
    )
    lower, upper = world.get_joint_limits(robot_id)
    np.testing.assert_allclose(lower, [-1.0, -2.0])
    np.testing.assert_allclose(upper, [1.0, 2.0])

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

    with pytest.raises(ValueError, match="does not match the composed model"):
        _make_world(fake_roboplan, config)


def test_context_cloning_and_joint_state_round_trip(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)

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


def test_joint_name_mapping_is_applied_to_input_states(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    robot_config.joint_name_mapping = {"arm/j1": "joint1", "arm/j2": "joint2"}
    world, robot_id = _make_world(fake_roboplan, robot_config)

    world.sync_from_joint_state(
        robot_id, JointState(name=["arm/j1", "arm/j2"], position=[0.2, 0.3])
    )

    live_round_trip = world.get_joint_state(world.get_live_context(), robot_id)
    assert live_round_trip.position == [0.2, 0.3]


def test_global_joint_names_are_mapped_without_regressing_coordinator_names(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    robot_config.joint_name_mapping = {"arm/j1": "joint1", "arm/j2": "joint2"}
    world, robot_id = _make_world(fake_roboplan, robot_config)

    world.sync_from_joint_state(
        robot_id, JointState(name=["arm/j1", "arm/j2"], position=[0.4, 0.5])
    )
    assert world.get_joint_state(world.get_live_context(), robot_id).position == [0.4, 0.5]

    world.sync_from_joint_state(
        robot_id, JointState(name=["arm/joint1", "arm/joint2"], position=[0.2, 0.3])
    )
    assert world.get_joint_state(world.get_live_context(), robot_id).position == [0.2, 0.3]


def test_duplicate_resolved_joint_names_fail_clearly(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    robot_config.joint_name_mapping = {"alias": "joint1"}
    world, robot_id = _make_world(fake_roboplan, robot_config)

    with pytest.raises(ValueError, match="duplicate joint 'joint1'"):
        world.sync_from_joint_state(
            robot_id, JointState(name=["joint1", "alias"], position=[0.1, 0.2])
        )


def test_obstacle_mutation_updates_scene_and_stored_pose(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)
    add_box = mocker.patch.object(
        FakeScene,
        "addBoxGeometry",
        autospec=True,
        side_effect=FakeScene.addBoxGeometry,
    )

    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )
    assert world.add_obstacle(obstacle) == "box"
    assert add_box.call_args.args[2] == "dimos_world"
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


def test_obstacle_added_before_finalize_is_materialized(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    module = _import_roboplan_world(fake_roboplan)
    world = module.RoboPlanWorld()
    world.add_robot(robot_config)
    add_box = mocker.patch.object(FakeScene, "addBoxGeometry", autospec=True)
    world.add_obstacle(
        Obstacle(
            name="pending",
            obstacle_type=ObstacleType.BOX,
            pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
            dimensions=(0.1, 0.2, 0.3),
        )
    )

    world.finalize()

    assert add_box.call_count == 1
    assert add_box.call_args.args[1] == "pending"


def test_failed_obstacle_add_rolls_back_and_can_be_retried(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)
    obstacle = Obstacle(
        name="retryable",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        dimensions=(0.1, 0.2, 0.3),
    )
    add_box = mocker.patch.object(
        FakeScene,
        "addBoxGeometry",
        autospec=True,
        side_effect=[RuntimeError("backend rejected geometry"), None],
    )

    with pytest.raises(RuntimeError, match="backend rejected geometry"):
        world.add_obstacle(obstacle)

    assert world.get_obstacles() == []
    assert world.add_obstacle(obstacle) == "retryable"
    assert world.get_obstacles() == [obstacle]
    assert add_box.call_count == 2


def test_concurrent_remove_waits_for_obstacle_add(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)
    obstacle = Obstacle(
        name="concurrent",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        dimensions=(0.1, 0.2, 0.3),
    )
    backend_add_started = threading.Event()
    allow_backend_add = threading.Event()
    native_add = FakeScene.addBoxGeometry

    def blocking_add(*args: Any, **kwargs: Any) -> None:
        backend_add_started.set()
        assert allow_backend_add.wait(timeout=1.0)
        native_add(*args, **kwargs)

    mocker.patch.object(
        FakeScene,
        "addBoxGeometry",
        autospec=True,
        side_effect=blocking_add,
    )
    added: list[str] = []
    removed: list[bool] = []
    add_thread = threading.Thread(target=lambda: added.append(world.add_obstacle(obstacle)))
    remove_thread = threading.Thread(
        target=lambda: removed.append(world.remove_obstacle(obstacle.name))
    )

    add_thread.start()
    assert backend_add_started.wait(timeout=1.0)
    remove_thread.start()
    allow_backend_add.set()
    add_thread.join(timeout=1.0)
    remove_thread.join(timeout=1.0)

    assert not add_thread.is_alive()
    assert not remove_thread.is_alive()
    assert added == ["concurrent"]
    assert removed == [True]
    assert world.get_obstacles() == []
    assert "concurrent" not in world._scene.geometry


def test_collision_config_and_edge_checks(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)

    safe = JointState(name=["joint1", "joint2"], position=[0.1, 0.2])
    colliding = JointState(name=["joint1", "joint2"], position=[0.95, 0.2])

    assert world.check_config_collision_free(robot_id, safe)
    assert not world.check_config_collision_free(robot_id, colliding)
    assert not world.check_edge_collision_free(robot_id, safe, colliding, step_size=0.05)


def test_collision_check_uses_scene_queries(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)

    safe = JointState(name=["joint1", "joint2"], position=[0.1, 0.2])
    colliding = JointState(name=["joint1", "joint2"], position=[0.95, 0.2])

    assert world.check_config_collision_free(robot_id, safe)
    assert not world.check_config_collision_free(robot_id, colliding)


def test_generic_rrt_planner_uses_roboplan_world_collision_checks(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    planner = RRTConnectPlanner(step_size=0.5, connect_step_size=0.5, goal_tolerance=10.0)

    start = JointState(name=["joint1", "joint2"], position=[0.0, 0.0])
    goal = JointState(name=["joint1", "joint2"], position=[0.2, 0.1])
    result = planner.plan_joint_path(world, robot_id, start, goal, timeout=1.0, max_iterations=3)

    assert result.status == PlanningStatus.SUCCESS
    assert len(result.path) >= 2


def test_fk_jacobian_and_explicit_min_distance_unsupported(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    ctx = world.get_live_context()
    world.set_joint_state(
        ctx, robot_id, JointState(name=["joint1", "joint2"], position=[0.25, 0.5])
    )

    pose = world.get_ee_pose(ctx, robot_id)
    assert pose.position.x == pytest.approx(0.75)
    assert world.get_jacobian(ctx, robot_id).shape == (6, 2)
    with pytest.raises(NotImplementedError, match="get_min_distance"):
        world.get_min_distance(ctx, robot_id)


def test_group_fk_and_jacobian_use_group_tip_and_local_joint_order(
    fake_roboplan: None, robot_config: RobotModelConfig, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = robot_config.model_copy(
        update={
            "joint_names": ["joint1", "joint2", "joint3"],
            "planning_groups": [
                PlanningGroupDefinition(
                    name="wrist",
                    joint_names=("joint3", "joint1"),
                    base_link="base",
                    tip_link="tcp",
                )
            ],
            "joint_limits_lower": [-1.0, -2.0, -3.0],
            "joint_limits_upper": [1.0, 2.0, 3.0],
        }
    )
    monkeypatch.setattr(FakeScene, "joint_group_joint_names", ["joint2", "joint1", "joint3"])
    monkeypatch.setattr(FakeScene, "position_limits_lower", [-2.0, -1.0, -3.0])
    monkeypatch.setattr(FakeScene, "position_limits_upper", [2.0, 1.0, 3.0])
    fk_frames: list[str] = []

    def fake_fk(
        self: FakeScene, q: np.ndarray, frame_name: str, base_frame: str = ""
    ) -> np.ndarray:
        fk_frames.append(frame_name)
        mat = np.eye(4)
        mat[0, 3] = float(np.sum(q))
        return mat

    def fake_jacobian(
        self: FakeScene, q: np.ndarray, frame_name: str, local: bool = True
    ) -> np.ndarray:
        assert frame_name == "tcp"
        assert local is True
        return np.arange(18, dtype=np.float64).reshape(6, 3)

    monkeypatch.setattr(FakeScene, "forwardKinematics", fake_fk)
    monkeypatch.setattr(FakeScene, "computeFrameJacobian", fake_jacobian)
    world, robot_id = _make_world(fake_roboplan, config)
    ctx = world.get_live_context()
    world.set_joint_state(
        ctx,
        robot_id,
        JointState({"name": ["joint1", "joint2", "joint3"], "position": [1.0, 2.0, 3.0]}),
    )

    pose = world.get_group_ee_pose(ctx, "arm/wrist")
    jacobian = world.get_group_jacobian(ctx, "arm/wrist")

    assert fk_frames == ["tcp"]
    assert pose.position.x == pytest.approx(6.0)
    np.testing.assert_allclose(jacobian, np.arange(18, dtype=np.float64).reshape(6, 3)[:, [2, 0]])


def test_group_kinematics_reject_missing_tip_or_missing_context(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    no_tip_config = robot_config.model_copy(
        update={
            "planning_groups": [
                PlanningGroupDefinition(
                    name="joint_only", joint_names=("joint1", "joint2"), base_link="base"
                )
            ]
        }
    )
    world, robot_id = _make_world(fake_roboplan, no_tip_config)

    with pytest.raises(ValueError, match="no tip link"):
        world.get_group_ee_pose(world.get_live_context(), "arm/joint_only")
    with pytest.raises(ValueError, match="no tip link"):
        world.get_group_jacobian(world.get_live_context(), "arm/joint_only")

    ctx = world.get_live_context()
    del ctx.q_by_robot[robot_id]
    with pytest.raises(KeyError, match=robot_id):
        world.get_link_pose(ctx, robot_id, "tcp")

    jacobian_world, jacobian_robot_id = _make_world(fake_roboplan, robot_config)
    jacobian_ctx = jacobian_world.get_live_context()
    del jacobian_ctx.q_by_robot[jacobian_robot_id]
    with pytest.raises(RuntimeError, match="Missing authoritative state"):
        jacobian_world.get_group_jacobian(jacobian_ctx, "arm/manipulator")


def test_group_jacobian_validates_projection_shape(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    ctx = world.get_live_context()
    world.set_joint_state(ctx, robot_id, JointState(name=["joint1", "joint2"], position=[0.0, 0.0]))

    monkeypatch.setattr(
        FakeScene,
        "computeFrameJacobian",
        lambda self, q, frame_name, local=True: np.ones((5, 2)),
    )
    with pytest.raises(ValueError, match="Unexpected RoboPlan Jacobian shape"):
        world.get_group_jacobian(ctx, "arm/manipulator")

    monkeypatch.setattr(
        FakeScene,
        "computeFrameJacobian",
        lambda self, q, frame_name, local=True: np.ones((6, 4)),
    )
    with pytest.raises(ValueError, match="cannot project"):
        world.get_group_jacobian(ctx, "arm/manipulator")


def test_legacy_kinematics_wrappers_require_unique_pose_group(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    no_pose_config = robot_config.model_copy(
        update={
            "planning_groups": [
                PlanningGroupDefinition(name="base", joint_names=("joint1",), base_link="base")
            ]
        }
    )
    no_pose_world, no_pose_id = _make_world(fake_roboplan, no_pose_config)
    with pytest.raises(ValueError, match="no pose-targetable"):
        no_pose_world.get_ee_pose(no_pose_world.get_live_context(), no_pose_id)
    with pytest.raises(ValueError, match="no pose-targetable"):
        no_pose_world.get_jacobian(no_pose_world.get_live_context(), no_pose_id)

    ambiguous_config = robot_config.model_copy(
        update={
            "planning_groups": [
                PlanningGroupDefinition(
                    name="a", joint_names=("joint1",), base_link="base", tip_link="a_tip"
                ),
                PlanningGroupDefinition(
                    name="b", joint_names=("joint2",), base_link="base", tip_link="b_tip"
                ),
            ]
        }
    )
    ambiguous_world, ambiguous_id = _make_world(fake_roboplan, ambiguous_config)
    with pytest.raises(ValueError, match="pose-targetable planning groups"):
        ambiguous_world.get_jacobian(ambiguous_world.get_live_context(), ambiguous_id)


def test_group_lookup_rejects_unknown_group_id(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)

    with pytest.raises(KeyError, match="Unknown planning group ID"):
        world.get_group_ee_pose(world.get_live_context(), "other/missing")


def test_native_planner_converts_path(fake_roboplan: None, robot_config: RobotModelConfig) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)

    start = JointState(name=["joint1", "joint2"], position=[0.0, 0.0])
    goal = JointState(name=["joint1", "joint2"], position=[0.4, 0.2])
    result = world.plan_joint_path(world, robot_id, start, goal, timeout=1.0)

    assert result.status == PlanningStatus.SUCCESS
    assert [state.position for state in result.path] == [[0.0, 0.0], [0.2, 0.1], [0.4, 0.2]]
    assert [state.name for state in result.path] == [["joint1", "joint2"]] * 3


def test_native_planner_names_path_from_robot_config_when_start_is_unnamed(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)

    start = JointState(name=[], position=[0.0, 0.0])
    goal = JointState(name=["joint1", "joint2"], position=[0.4, 0.2])
    result = world.plan_joint_path(world, robot_id, start, goal, timeout=1.0)

    assert result.status == PlanningStatus.SUCCESS
    assert [state.name for state in result.path] == [["joint1", "joint2"]] * 3


def test_native_selected_planner_returns_global_selected_joint_names(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)
    selection = _selection((robot_config,), "arm/manipulator")

    result = world.plan_selected_joint_path(
        world,
        selection,
        JointState(name=["arm/joint1", "arm/joint2"], position=[0.0, 0.0]),
        JointState(name=["arm/joint1", "arm/joint2"], position=[0.4, 0.2]),
        timeout=1.0,
    )

    assert result.status == PlanningStatus.SUCCESS
    assert [state.name for state in result.path] == [["arm/joint1", "arm/joint2"]] * 3
    assert [state.position for state in result.path] == [[0.0, 0.0], [0.2, 0.1], [0.4, 0.2]]


def test_native_selected_planner_accepts_local_joint_names(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)
    selection = _selection((robot_config,), "arm/manipulator")

    result = world.plan_selected_joint_path(
        world,
        selection,
        JointState(name=["joint2", "joint1"], position=[0.0, 0.0]),
        JointState(name=["joint2", "joint1"], position=[0.4, 0.2]),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert result.path[0].name == ["arm/joint1", "arm/joint2"]
    assert result.path[-1].position == [0.2, 0.4]


def test_native_selected_planner_rejects_multi_group_selection(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    config = robot_config.model_copy(
        update={
            "planning_groups": [
                PlanningGroupDefinition("left", ("joint1",), "base", "left_tip"),
                PlanningGroupDefinition("right", ("joint2",), "base", "right_tip"),
            ]
        }
    )
    world, _ = _make_world(fake_roboplan, config)
    selection = _selection((config,), "arm/left", "arm/right")

    result = world.plan_selected_joint_path(
        world,
        selection,
        JointState(name=list(selection.joint_names), position=[0.0, 0.0]),
        JointState(name=list(selection.joint_names), position=[0.1, 0.1]),
    )

    assert result.status == PlanningStatus.UNSUPPORTED
    assert "no generated group" in result.message


def test_native_planner_coordinates_groups_across_two_robots(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _, _, second_config = _make_two_robot_world(fake_roboplan, robot_config)
    selection = _selection(
        (robot_config, second_config),
        "right/manipulator",
        "arm/manipulator",
    )

    result = world.plan_selected_joint_path(
        world,
        selection,
        JointState(name=list(selection.joint_names), position=[0.0, 0.0, 0.0, 0.0]),
        JointState(name=list(selection.joint_names), position=[0.4, 0.2, 0.1, 0.3]),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert result.path[-1].name == [
        "arm/joint1",
        "arm/joint2",
        "right/joint1",
        "right/joint2",
    ]
    assert result.path[-1].position == [0.1, 0.3, 0.4, 0.2]


def test_native_planner_preserves_other_robot_and_auxiliary_joint_state(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world, _, second_id, second_config = _make_two_robot_world(fake_roboplan, robot_config)
    world.sync_from_joint_state(
        second_id,
        JointState(name=["joint1", "joint2"], position=[0.3, 0.1]),
    )
    selection = _selection((robot_config, second_config), "arm/manipulator")
    observed_positions: dict[str, float] = {}
    native_plan = FakeRRT.plan

    def capture_scene_state(
        planner: FakeRRT,
        q_start: FakeJointConfiguration,
        q_goal: FakeJointConfiguration,
    ) -> FakeJointPath:
        observed_positions.update(
            zip(
                planner.scene.native_joint_names,
                planner.scene.current_positions,
                strict=True,
            )
        )
        return native_plan(planner, q_start, q_goal)

    mocker.patch.object(FakeRRT, "plan", autospec=True, side_effect=capture_scene_state)

    result = world.plan_selected_joint_path(
        world,
        selection,
        JointState(name=list(selection.joint_names), position=[0.0, 0.0]),
        JointState(name=list(selection.joint_names), position=[0.2, 0.1]),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert observed_positions["right__joint1"] == pytest.approx(0.3)
    assert observed_positions["right__joint2"] == pytest.approx(0.1)
    assert observed_positions["arm__joint3"] == pytest.approx(0.0)
    assert observed_positions["right__joint3"] == pytest.approx(0.0)


def test_native_planner_waits_for_every_robot_state(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    module = _import_roboplan_world(fake_roboplan)
    second_config = robot_config.model_copy(update={"name": "right"})
    world = module.RoboPlanWorld()
    first_id = world.add_robot(robot_config)
    world.add_robot(second_config)
    world.finalize()
    world.sync_from_joint_state(
        first_id,
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
    )
    selection = _selection((robot_config, second_config), "arm/manipulator")

    result = world.plan_selected_joint_path(
        world,
        selection,
        JointState(name=list(selection.joint_names), position=[0.0, 0.0]),
        JointState(name=list(selection.joint_names), position=[0.2, 0.1]),
    )

    assert result.status == PlanningStatus.INVALID_START
    assert "authoritative state is incomplete" in result.message


def test_native_planner_rejects_empty_path(
    fake_roboplan: None, robot_config: RobotModelConfig, monkeypatch: pytest.MonkeyPatch
) -> None:
    class EmptyPathRRT(FakeRRT):
        def plan(
            self, q_start: FakeJointConfiguration, q_goal: FakeJointConfiguration
        ) -> FakeJointPath:
            return FakeJointPath(["joint1", "joint2"], [])

    monkeypatch.setattr(sys.modules["roboplan.rrt"], "RRT", EmptyPathRRT)
    world, robot_id = _make_world(fake_roboplan, robot_config)

    start = JointState(name=["joint1", "joint2"], position=[0.0, 0.0])
    goal = JointState(name=["joint1", "joint2"], position=[0.4, 0.2])
    result = world.plan_joint_path(world, robot_id, start, goal, timeout=1.0)

    assert result.status == PlanningStatus.NO_SOLUTION
    assert result.path == []
    assert "empty path" in result.message


def test_collision_exclusion_pairs_are_written_to_generated_srdf(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    robot_config.collision_exclusion_pairs = [
        ("base", "link2"),
        ("other_base", "other_tip"),
    ]
    world, _ = _make_world(fake_roboplan, robot_config)

    srdf_path = Path(world._scene.constructor_args[2])
    srdf = srdf_path.read_text()
    assert 'disable_collisions link1="base" link2="link2"' in srdf
    assert "other_base" not in srdf


def test_collision_exclusion_with_one_unknown_link_is_rejected(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    robot_config.collision_exclusion_pairs = [("base", "missing")]
    module = _import_roboplan_world(fake_roboplan)
    world = module.RoboPlanWorld()
    world.add_robot(robot_config)

    with pytest.raises(
        ValueError,
        match=r"collision exclusion references unknown links: base <-> missing",
    ):
        world.finalize()


def test_generated_srdf_uses_scoped_temp_directory(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)

    srdf_path = Path(world._scene.constructor_args[2])
    assert srdf_path.parent.name.startswith("dimos_roboplan_")
    assert srdf_path.exists()


def test_base_pose_is_written_to_composed_model(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    robot_config.base_pose = PoseStamped(  # type: ignore[call-arg]
        position=Vector3(1, 0, 0), orientation=Quaternion()
    )
    world, _ = _make_world(fake_roboplan, robot_config)

    urdf_root = ET.parse(Path(world._scene.constructor_args[1])).getroot()
    origin = urdf_root.find("./joint[@name='dimos_world_joint']/origin")
    assert origin is not None
    assert origin.get("xyz") == "1 0 0"
