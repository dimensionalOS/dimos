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

"""Pure-Python tests for the optional RoboPlan world and planner adapters."""

from __future__ import annotations

from dataclasses import replace
from enum import Enum
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
from dimos.manipulation.planning.planners.roboplan_config import (
    RoboPlanCartesianPathConfig,
    RoboPlanPathShortcuttingConfig,
    RoboPlanPlannerConfig,
)
from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.manipulation.planning.spec.validation import MAX_OCTREE_POINTS
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
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


class FakeJointTrajectory:
    def __init__(
        self,
        joint_names: list[str],
        positions: list[np.ndarray],
        velocities: list[np.ndarray],
        times: list[float],
    ) -> None:
        self.joint_names = joint_names
        self.positions = positions
        self.velocities = velocities
        self.accelerations = [np.zeros_like(row) for row in positions]
        self.times = times


class FakeCartesianPath:
    def __init__(
        self,
        base_frames: list[str],
        tip_frames: list[str],
        tforms: list[list[np.ndarray]],
    ) -> None:
        self.base_frames = base_frames
        self.tip_frames = tip_frames
        self.tforms = tforms


class FakeCartesianSpeedMode(Enum):
    Bounded = "bounded"
    TimeOptimal = "time_optimal"


class FakeCartesianPlannerOptions:
    def __init__(self) -> None:
        self.group_name = ""
        self.speed_mode = FakeCartesianSpeedMode.TimeOptimal
        self.dt = 0.01
        self.max_linear_speed = 0.1
        self.max_angular_speed = 0.5
        self.max_linear_acceleration = 0.5
        self.max_angular_acceleration = 2.5
        self.max_position_error = 0.005
        self.max_orientation_error = 0.01
        self.position_cost = 1.0
        self.orientation_cost = 1.0
        self.task_gain = 1.0
        self.lm_damping = 0.01
        self.regularization = 1e-6
        self.config_task_weight = 0.05
        self.velocity_scale = 1.0
        self.acceleration_scale = 1.0
        self.limit_ratio_tolerance = 1.05
        self.toppra_blend_deviation = 0.05
        self.position_limit_gain = 1.0
        self.max_attempts_per_step = 16


class FakeCartesianPathPlanner:
    instances: ClassVar[list[FakeCartesianPathPlanner]] = []

    def __init__(self, scene: FakeScene, options: FakeCartesianPlannerOptions) -> None:
        self.scene = scene
        self.options = options
        self.paths: list[FakeCartesianPath] = []
        type(self).instances.append(self)

    def plan(
        self,
        path: FakeCartesianPath,
        q_start: FakeJointConfiguration,
    ) -> FakeJointTrajectory:
        self.paths.append(path)
        start = np.asarray(q_start.positions, dtype=np.float64)
        midpoint = start + 0.05
        goal = start + 0.1
        positions = [start, midpoint, goal]
        velocities = [np.zeros_like(start), np.ones_like(start) * 0.5, np.zeros_like(start)]
        return FakeJointTrajectory(
            q_start.joint_names,
            positions,
            velocities,
            [0.0, self.options.dt, 2.0 * self.options.dt],
        )


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


class FakeOcTree:
    def __init__(self, boxes: list[np.ndarray], resolution: float) -> None:
        self.boxes = [np.asarray(box, dtype=np.float64) for box in boxes]
        self.resolution = resolution


class FakeJointGroupInfo:
    def __init__(self, joint_names: list[str]) -> None:
        self.joint_names = joint_names


class FakeScene:
    joint_group_joint_names: ClassVar[list[str] | None] = None
    position_limits_lower: ClassVar[list[float]] = [-1.0, -2.0]
    position_limits_upper: ClassVar[list[float]] = [1.0, 2.0]
    valid_frames: ClassVar[set[str]] = {"dimos_world"}

    def __init__(
        self,
        *,
        name: str,
        urdf: str,
        srdf: str,
        package_paths: list[str],
    ) -> None:
        self.constructor_kwargs = {
            "name": name,
            "urdf": urdf,
            "srdf": srdf,
            "package_paths": package_paths,
        }
        self.models: list[tuple[str, str, dict[str, str]]] = []
        self.geometry: dict[str, np.ndarray] = {}
        self.geometry_shapes: dict[str, object] = {}
        self.collision_settings: dict[tuple[str, str], bool] = {}
        self.groups = self._read_groups(srdf)
        self.native_joint_names = self._read_joint_names(urdf)
        self.current_positions = np.zeros(len(self.native_joint_names), dtype=np.float64)

    def _require_frame(self, parent_frame: str) -> None:
        if parent_frame not in self.valid_frames:
            raise RuntimeError(f"Frame name '{parent_frame}' not found in frame_map_.")

    @staticmethod
    def _read_groups(srdf: str) -> dict[str, list[str]]:
        root = ET.fromstring(srdf)
        return {
            group.get("name", ""): [joint.get("name", "") for joint in group.findall("joint")]
            for group in root.findall("group")
        }

    @staticmethod
    def _read_joint_names(urdf: str) -> list[str]:
        root = ET.fromstring(urdf)
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
            if self.joint_group_joint_names is not None and name == "__dimos_all_configured__"
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
        self._require_frame(parent_frame)
        self.geometry[obstacle_id] = matrix
        self.geometry_shapes[obstacle_id] = box

    def addSphereGeometry(
        self,
        obstacle_id: str,
        parent_frame: str,
        sphere: FakeSphere,
        matrix: np.ndarray,
        color: np.ndarray,
    ) -> None:
        self._require_frame(parent_frame)
        self.geometry[obstacle_id] = matrix
        self.geometry_shapes[obstacle_id] = sphere

    def addCylinderGeometry(
        self,
        obstacle_id: str,
        parent_frame: str,
        cylinder: FakeCylinder,
        matrix: np.ndarray,
        color: np.ndarray,
    ) -> None:
        self._require_frame(parent_frame)
        self.geometry[obstacle_id] = matrix
        self.geometry_shapes[obstacle_id] = cylinder

    def addOcTreeGeometry(
        self,
        obstacle_id: str,
        parent_frame: str,
        octree: FakeOcTree,
        matrix: np.ndarray,
        color: np.ndarray,
    ) -> None:
        self._require_frame(parent_frame)
        self.geometry[obstacle_id] = matrix
        self.geometry_shapes[obstacle_id] = octree

    def addMeshGeometry(
        self,
        obstacle_id: str,
        parent_frame: str,
        mesh: FakeMesh,
        matrix: np.ndarray,
        color: np.ndarray,
    ) -> None:
        self._require_frame(parent_frame)
        self.geometry[obstacle_id] = matrix
        self.geometry_shapes[obstacle_id] = mesh

    def updateGeometryPlacement(
        self, obstacle_id: str, parent_frame: str, matrix: np.ndarray
    ) -> None:
        self._require_frame(parent_frame)
        self.geometry[obstacle_id] = matrix

    def removeGeometry(self, obstacle_id: str) -> None:
        del self.geometry[obstacle_id]
        self.geometry_shapes.pop(obstacle_id, None)

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


class FakePathShortcuttingOptions:
    def __init__(self) -> None:
        self.group_name = ""
        self.max_step_size = 0.05
        self.max_iters = 100
        self.seed = 0
        self.max_convergence_iters = 20
        self.redundant_removal_iters = 20


class FakePathShortcutter:
    instances: ClassVar[list[FakePathShortcutter]] = []

    def __init__(self, scene: FakeScene, options: FakePathShortcuttingOptions) -> None:
        self.scene = scene
        self.options = options
        type(self).instances.append(self)

    def shortcut(self, path: FakeJointPath) -> FakeJointPath:
        return path


def _install_fake_roboplan(monkeypatch: pytest.MonkeyPatch) -> None:
    FakeCartesianPathPlanner.instances.clear()
    FakePathShortcutter.instances.clear()
    roboplan_pkg = ModuleType("roboplan")
    roboplan_pkg.__path__ = []  # type: ignore[attr-defined]
    core = ModuleType("roboplan.core")
    core.Scene = FakeScene  # type: ignore[attr-defined]
    core.JointConfiguration = FakeJointConfiguration  # type: ignore[attr-defined]
    core.JointPath = FakeJointPath  # type: ignore[attr-defined]
    core.JointTrajectory = FakeJointTrajectory  # type: ignore[attr-defined]
    core.CartesianPath = FakeCartesianPath  # type: ignore[attr-defined]
    core.PathShortcuttingOptions = FakePathShortcuttingOptions  # type: ignore[attr-defined]
    core.PathShortcutter = FakePathShortcutter  # type: ignore[attr-defined]
    core.Box = FakeBox  # type: ignore[attr-defined]
    core.Sphere = FakeSphere  # type: ignore[attr-defined]
    core.Cylinder = FakeCylinder  # type: ignore[attr-defined]
    core.Mesh = FakeMesh  # type: ignore[attr-defined]
    core.OcTree = FakeOcTree  # type: ignore[attr-defined]

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

    cartesian = ModuleType("roboplan.cartesian_planning")
    cartesian.CartesianPathPlanner = FakeCartesianPathPlanner  # type: ignore[attr-defined]
    cartesian.CartesianPlannerOptions = FakeCartesianPlannerOptions  # type: ignore[attr-defined]
    cartesian.CartesianSpeedMode = FakeCartesianSpeedMode  # type: ignore[attr-defined]

    monkeypatch.setitem(sys.modules, "roboplan", roboplan_pkg)
    monkeypatch.setitem(sys.modules, "roboplan.core", core)
    monkeypatch.setitem(sys.modules, "roboplan.rrt", rrt)
    monkeypatch.setitem(sys.modules, "roboplan.cartesian_planning", cartesian)


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
        model=RobotModel.from_file(model_path),
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


def _make_world(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    planner_config: RoboPlanPlannerConfig | None = None,
) -> Any:
    module = _import_roboplan_world(fake_roboplan)

    world = module.RoboPlanWorld()
    world.load_model(robot_config)
    world.finalize()
    world.sync_from_joint_state(
        JointState(
            name=list(robot_config.joint_names),
            position=[0.0] * len(robot_config.joint_names),
        ),
    )
    planner_module = _import_roboplan_planner(fake_roboplan)
    _PLANNERS_BY_WORLD[world] = planner_module.RoboPlanPlanner(
        world,
        planner_config or RoboPlanPlannerConfig(),
    )
    return world


def test_roboplan_loads_canonical_slash_names_natively(
    fake_roboplan: None,
    tmp_path: Path,
) -> None:
    model_path = tmp_path / "canonical.urdf"
    model_path.write_text(
        """
<robot name="canonical">
  <link name="world"/><link name="left/base"/><link name="left/tool"/>
  <joint name="left/mount" type="fixed">
    <parent link="world"/><child link="left/base"/>
  </joint>
  <joint name="left/j1" type="revolute">
    <parent link="left/base"/><child link="left/tool"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""
    )
    config = RobotModelConfig(
        model=RobotModel.from_file(model_path),
        joint_names=["left/j1"],
        base_link="world",
        planning_groups=[
            PlanningGroupDefinition(
                name="left_arm",
                joint_names=("left/j1",),
                base_link="left/base",
                tip_link="left/tool",
            )
        ],
        joint_limits_lower=[-1.0],
        joint_limits_upper=[1.0],
    )

    world = _make_world(fake_roboplan, config)

    assert world.get_model_config().joint_names == ["left/j1"]
    assert "left/j1" in world._scene.constructor_kwargs["urdf"]


def _selection(
    config: RobotModelConfig,
    *group_ids: str,
) -> PlanningGroupSelection:
    registry = PlanningGroupRegistry(config.planning_groups)
    return PlanningGroupSelection.from_groups(
        tuple(registry.get(group_id) for group_id in group_ids)
    )


def _relative_target(*waypoints: Transform) -> tuple[Transform, ...]:
    return (Transform.identity(), *waypoints)


def _absolute_target(*waypoints: PoseStamped) -> tuple[PoseStamped, ...]:
    return (
        PoseStamped(
            frame_id="world",
            position=Vector3(),
            orientation=Quaternion(),
        ),
        *waypoints,
    )


def _import_roboplan_world(fake_roboplan: None) -> ModuleType:
    module_name = "dimos.manipulation.planning.world.roboplan_world"
    if module_name in sys.modules:
        return importlib.reload(sys.modules[module_name])
    return importlib.import_module(module_name)


def _import_roboplan_planner(fake_roboplan: None) -> ModuleType:
    module_name = "dimos.manipulation.planning.planners.roboplan_planner"
    if module_name in sys.modules:
        return importlib.reload(sys.modules[module_name])
    return importlib.import_module(module_name)


_PLANNERS_BY_WORLD: dict[Any, Any] = {}


def _planner_for(world: Any) -> Any:
    if world not in _PLANNERS_BY_WORLD:
        planner_module = _import_roboplan_planner(None)
        _PLANNERS_BY_WORLD[world] = planner_module.RoboPlanPlanner(
            world,
            RoboPlanPlannerConfig(),
        )
    return _PLANNERS_BY_WORLD[world]


def test_roboplan_bindings_are_imported_at_module_load(fake_roboplan: None) -> None:
    world_module = _import_roboplan_world(fake_roboplan)
    planner_module = _import_roboplan_planner(fake_roboplan)

    assert world_module.roboplan_core.Scene is FakeScene
    assert planner_module.roboplan_rrt.RRT is FakeRRT
    assert planner_module.roboplan_cartesian.CartesianPathPlanner is FakeCartesianPathPlanner


def test_roboplan_planner_rejects_non_roboplan_world(fake_roboplan: None) -> None:
    planner_module = _import_roboplan_planner(fake_roboplan)

    with pytest.raises(TypeError, match="requires a RoboPlanWorld"):
        planner_module.RoboPlanPlanner(object(), RoboPlanPlannerConfig())


def test_robot_registration_finalization_and_joint_limits(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)

    assert [world.get_model_config()] == [robot_config]
    assert world.get_model_config() is robot_config
    assert world._scene.constructor_kwargs["name"] == "dimos_model"
    assert ET.fromstring(world._scene.constructor_kwargs["urdf"]).get("name") == "dimos_model"
    assert world._scene.constructor_kwargs["srdf"].startswith('<robot name="dimos_model">')
    assert (
        'disable_collisions link1="base" link2="link1"' in world._scene.constructor_kwargs["srdf"]
    )
    lower, upper = world.get_joint_limits()
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

    world = _make_world(fake_roboplan, config)

    lower, upper = world.get_joint_limits()
    np.testing.assert_allclose(lower, [-1.0, -2.0])
    np.testing.assert_allclose(upper, [1.0, 2.0])


def test_scene_joint_limits_validate_joint_names(
    fake_roboplan: None, robot_config: RobotModelConfig, monkeypatch: pytest.MonkeyPatch
) -> None:
    config = robot_config.model_copy(
        update={"joint_limits_lower": None, "joint_limits_upper": None}
    )
    monkeypatch.setattr(FakeScene, "joint_group_joint_names", ["joint2", "extra_joint"])

    with pytest.raises(ValueError, match="does not match the prepared model"):
        _make_world(fake_roboplan, config)


def test_context_cloning_and_joint_state_round_trip(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)

    live_state = JointState(name=["joint1", "joint2"], position=[0.1, 0.2])
    world.sync_from_joint_state(live_state)

    with world.scratch_context() as scratch:
        scratch_state = world.get_joint_state(scratch)
        assert scratch_state.name == ["joint1", "joint2"]
        assert scratch_state.position == [0.1, 0.2]
        world.set_joint_state(scratch, JointState(name=["joint1", "joint2"], position=[0.3, 0.4]))

    live_round_trip = world.get_joint_state(world.get_live_context())
    assert live_round_trip.position == [0.1, 0.2]


def test_obstacle_mutation_updates_scene_and_stored_pose(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
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
    np.testing.assert_allclose(
        pose_to_matrix(world.get_obstacles()[0].pose),
        pose_to_matrix(updated_pose),
    )
    np.testing.assert_allclose(world._scene.geometry["box"], pose_to_matrix(updated_pose))
    assert world.add_obstacle(obstacle) is None
    assert world.remove_obstacle("box")
    assert world.get_obstacles() == []


def test_octree_obstacle_reaches_the_scene_as_occupied_cells(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    world = _make_world(fake_roboplan, robot_config)

    obstacle = Obstacle(
        name="voxel-map",
        obstacle_type=ObstacleType.OCTREE,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        points=((0.0, 0.0, 0.0), (0.05, 0.0, 0.0)),
        octree_resolution=0.05,
    )
    assert world.add_obstacle(obstacle) == "voxel-map"

    octree = world._scene.geometry_shapes["voxel-map"]
    assert octree.resolution == 0.05
    # Coal reads a cell as center, edge, cost, occupancy threshold. Every cell
    # here is occupied, so cost must clear the 0.5 default threshold.
    np.testing.assert_allclose(octree.boxes[0], (0.0, 0.0, 0.0, 0.05, 1.0, 0.5))
    np.testing.assert_allclose(octree.boxes[1], (0.05, 0.0, 0.0, 0.05, 1.0, 0.5))

    assert world.remove_obstacle("voxel-map")
    assert world.get_obstacles() == []


def test_octree_obstacle_survives_the_deepcopy_and_equality_the_world_does(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    # The world snapshots obstacles with deepcopy and hands copies back out. A
    # raw ndarray field would make that snapshot uncomparable, so the points
    # stay plain tuples.
    world = _make_world(fake_roboplan, robot_config)
    obstacle = Obstacle(
        name="voxel-map",
        obstacle_type=ObstacleType.OCTREE,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        points=((0.0, 0.0, 0.0),),
        octree_resolution=0.05,
    )
    assert world.add_obstacle(obstacle) == "voxel-map"

    stored = world.get_obstacles()[0]
    assert stored == world.get_obstacles()[0]
    assert stored.points == ((0.0, 0.0, 0.0),)
    assert stored.octree_resolution == 0.05


def test_octree_obstacle_rejects_geometry_it_cannot_build(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    world = _make_world(fake_roboplan, robot_config)

    def octree(**kwargs: object) -> Obstacle:
        base: dict[str, object] = {
            "name": "voxel-map",
            "obstacle_type": ObstacleType.OCTREE,
            "pose": PoseStamped(position=Vector3(), orientation=Quaternion()),
            "points": ((0.0, 0.0, 0.0),),
            "octree_resolution": 0.05,
        }
        base.update(kwargs)
        return Obstacle(**base)  # type: ignore[arg-type]

    for bad in (
        octree(points=()),
        octree(octree_resolution=None),
        octree(octree_resolution=0.0),
        octree(points=((0.0, 0.0, float("nan")),)),
        octree(points=tuple((float(i), 0.0, 0.0) for i in range(MAX_OCTREE_POINTS + 1))),
    ):
        with pytest.raises(ValueError):
            world.add_obstacle(bad)
    assert world.get_obstacles() == []


def test_obstacle_operations_require_finalization(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    module = _import_roboplan_world(fake_roboplan)
    world = module.RoboPlanWorld()
    world.load_model(robot_config)
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        dimensions=(0.1, 0.2, 0.3),
    )

    operations = [
        lambda: world.add_obstacle(obstacle),
        lambda: world.remove_obstacle("box"),
        lambda: world.update_obstacle(obstacle),
        lambda: world.update_obstacle_pose("box", obstacle.pose),
        world.clear_obstacles,
        world.get_obstacles,
    ]
    for operation in operations:
        with pytest.raises(RuntimeError, match="finalized"):
            operation()


def test_failed_obstacle_add_rolls_back_and_can_be_retried(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
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
    world = _make_world(fake_roboplan, robot_config)
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
    added: list[str | None] = []
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


def test_obstacle_ids_are_world_owned_and_invalid_insertions_are_rejected(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)

    unnamed = Obstacle(
        name="",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.1, 0.1),
    )
    named = replace(unnamed, name="world-owned")

    assert world.add_obstacle(unnamed) is None
    assert world.add_obstacle(named) == "world-owned"
    assert world.add_obstacle(named) is None
    assert world.remove_obstacle("missing") is False
    assert world.update_obstacle(replace(named, name="missing")) is False
    assert world.update_obstacle_pose("missing", named.pose) is False


def test_complete_update_rejects_invalid_obstacle_values(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    valid = Obstacle(
        name="shape",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )
    invalid_obstacles = [
        (replace(valid, name=""), "name must be non-empty"),
        (replace(valid, dimensions=(0.1, -0.2, 0.3)), "finite and positive"),
        (
            replace(valid, obstacle_type=ObstacleType.MESH, dimensions=(), mesh_path=None),
            "requires mesh_path",
        ),
        (replace(valid, color=(1.0, 0.0, 0.0)), "four finite values"),
        (
            replace(
                valid,
                pose=PoseStamped(
                    position=[np.nan, 0.0, 0.0],
                    orientation=[0.0, 0.0, 0.0, 1.0],
                ),
            ),
            "finite values",
        ),
    ]

    for invalid, message in invalid_obstacles:
        with pytest.raises(ValueError, match=message):
            world.update_obstacle(invalid)

    assert world.get_obstacles() == []


def test_complete_replacement_and_defensive_obstacle_snapshots(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    original = Obstacle(
        name="shape",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
        color=(1.0, 0.0, 0.0, 1.0),
    )
    assert world.add_obstacle(original) == "shape"
    original.dimensions = (9.0, 9.0, 9.0)
    assert world.get_obstacles()[0].dimensions == (0.1, 0.2, 0.3)

    replacement = Obstacle(
        name="shape",
        obstacle_type=ObstacleType.SPHERE,
        pose=PoseStamped(position=Vector3(1, 2, 3), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.4,),
        color=(0.0, 1.0, 0.0, 0.5),
    )
    assert world.update_obstacle(replacement)
    assert isinstance(world._scene.geometry_shapes["shape"], FakeSphere)
    stored = world.get_obstacles()[0]
    assert stored.obstacle_type == ObstacleType.SPHERE
    assert stored.dimensions == (0.4,)
    assert stored.color == (0.0, 1.0, 0.0, 0.5)

    replacement.dimensions = (2.0,)
    stored.dimensions = (3.0,)
    stored.pose.position.x = 99.0
    assert world.get_obstacles()[0].dimensions == (0.4,)
    assert world.get_obstacles()[0].pose.position.x == pytest.approx(1.0)
    assert world.update_obstacle(replace(replacement, name="missing")) is False


def test_collision_query_blocks_during_obstacle_replacement(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )
    world.add_obstacle(obstacle)
    replacement_started = threading.Event()
    allow_replacement = threading.Event()
    query_finished = threading.Event()
    original_remove = world._scene.removeGeometry

    def blocking_remove(obstacle_id: str) -> None:
        original_remove(obstacle_id)
        replacement_started.set()
        assert allow_replacement.wait(1.0)

    monkeypatch.setattr(world._scene, "removeGeometry", blocking_remove)
    update_thread = threading.Thread(
        target=lambda: world.update_obstacle(replace(obstacle, dimensions=(1, 1, 1)))
    )
    update_thread.start()
    assert replacement_started.wait(1.0)
    query_thread = threading.Thread(
        target=lambda: (
            world.check_config_collision_free(
                JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
            ),
            query_finished.set(),
        )
    )
    query_thread.start()
    assert not query_finished.wait(0.05)
    allow_replacement.set()
    update_thread.join(1.0)
    query_thread.join(1.0)
    assert query_finished.is_set()


def test_obstacle_replacement_blocks_during_collision_query(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )
    world.add_obstacle(obstacle)
    query_started = threading.Event()
    allow_query = threading.Event()
    update_finished = threading.Event()
    original_query = world._scene.hasCollisions

    def blocking_query(q: np.ndarray) -> bool:
        query_started.set()
        assert allow_query.wait(1.0)
        return original_query(q)

    monkeypatch.setattr(world._scene, "hasCollisions", blocking_query)
    query_thread = threading.Thread(
        target=lambda: world.check_config_collision_free(
            JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
        )
    )
    query_thread.start()
    assert query_started.wait(1.0)
    update_thread = threading.Thread(
        target=lambda: (
            world.update_obstacle(replace(obstacle, dimensions=(1, 1, 1))),
            update_finished.set(),
        )
    )
    update_thread.start()
    assert not update_finished.wait(0.05)
    allow_query.set()
    query_thread.join(1.0)
    update_thread.join(1.0)
    assert update_finished.is_set()


def test_native_update_failure_invalidates_world(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )
    world.add_obstacle(obstacle)
    monkeypatch.setattr(
        world._scene,
        "addBoxGeometry",
        lambda *_args: (_ for _ in ()).throw(ValueError("native replacement failed")),
    )

    with pytest.raises(ValueError, match="native replacement failed"):
        world.update_obstacle(replace(obstacle, dimensions=(1.0, 1.0, 1.0)))
    with pytest.raises(RuntimeError, match="invalid"):
        world.get_obstacles()
    with pytest.raises(RuntimeError, match="invalid"):
        world.check_config_collision_free(
            JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
        )


def test_native_pose_update_failure_invalidates_world(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )
    world.add_obstacle(obstacle)
    monkeypatch.setattr(
        world._scene,
        "updateGeometryPlacement",
        lambda *_args: (_ for _ in ()).throw(ValueError("native pose update failed")),
    )

    with pytest.raises(ValueError, match="native pose update failed"):
        world.update_obstacle_pose("box", obstacle.pose)
    with pytest.raises(RuntimeError, match="invalid"):
        world.get_obstacles()


def test_collision_config_and_edge_checks(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)

    safe = JointState(name=["joint1", "joint2"], position=[0.1, 0.2])
    colliding = JointState(name=["joint1", "joint2"], position=[0.95, 0.2])

    assert world.check_config_collision_free(safe)
    assert not world.check_config_collision_free(colliding)
    assert not world.check_edge_collision_free(safe, colliding, step_size=0.05)


def test_collision_check_uses_scene_queries(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)

    safe = JointState(name=["joint1", "joint2"], position=[0.1, 0.2])
    colliding = JointState(name=["joint1", "joint2"], position=[0.95, 0.2])

    assert world.check_config_collision_free(safe)
    assert not world.check_config_collision_free(colliding)


def test_generic_rrt_planner_uses_roboplan_world_collision_checks(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    planner = RRTConnectPlanner(step_size=0.5, connect_step_size=0.5, goal_tolerance=10.0)

    start = JointState(name=["joint1", "joint2"], position=[0.0, 0.0])
    goal = JointState(name=["joint1", "joint2"], position=[0.2, 0.1])
    result = planner.plan_joint_path(world, start, goal, timeout=1.0, max_iterations=3)

    assert result.status == PlanningStatus.SUCCESS
    assert len(result.path) >= 2


def test_generic_planner_allows_update_between_collision_checks(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )
    world.add_obstacle(obstacle)
    original_check = world.check_config_collision_free
    checks = 0
    updated = False

    def checking_with_interleaved_update(state: JointState) -> bool:
        nonlocal checks, updated
        result = original_check(state)
        checks += 1
        if checks == 1:
            updated = world.update_obstacle(replace(obstacle, dimensions=(1.0, 1.0, 1.0)))
        return result

    monkeypatch.setattr(world, "check_config_collision_free", checking_with_interleaved_update)
    planner = RRTConnectPlanner(step_size=0.5, connect_step_size=0.5, goal_tolerance=10.0)
    result = planner.plan_joint_path(
        world,
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
        JointState(name=["joint1", "joint2"], position=[0.2, 0.1]),
        timeout=1.0,
        max_iterations=3,
    )

    assert result.status == PlanningStatus.SUCCESS
    assert updated
    assert checks >= 2


def test_fk_jacobian_and_explicit_min_distance_unsupported(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    ctx = world.get_live_context()
    world.set_joint_state(ctx, JointState(name=["joint1", "joint2"], position=[0.25, 0.5]))

    pose = world.get_ee_pose(ctx)
    assert pose.position.x == pytest.approx(0.75)
    assert world.get_jacobian(ctx).shape == (6, 2)
    with pytest.raises(NotImplementedError, match="get_min_distance"):
        world.get_min_distance(ctx)


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
    world = _make_world(fake_roboplan, config)
    ctx = world.get_live_context()
    world.set_joint_state(
        ctx,
        JointState({"name": ["joint1", "joint2", "joint3"], "position": [1.0, 2.0, 3.0]}),
    )

    pose = world.get_group_ee_pose(ctx, "wrist")
    jacobian = world.get_group_jacobian(ctx, "wrist")

    assert fk_frames == ["tcp"]
    assert pose.position.x == pytest.approx(6.0)
    np.testing.assert_allclose(jacobian, np.arange(18, dtype=np.float64).reshape(6, 3)[:, [2, 0]])


def test_group_kinematics_rejects_missing_tip(
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
    world = _make_world(fake_roboplan, no_tip_config)

    with pytest.raises(ValueError, match="no tip link"):
        world.get_group_ee_pose(world.get_live_context(), "joint_only")
    with pytest.raises(ValueError, match="no tip link"):
        world.get_group_jacobian(world.get_live_context(), "joint_only")


def test_group_jacobian_validates_projection_shape(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    ctx = world.get_live_context()
    world.set_joint_state(ctx, JointState(name=["joint1", "joint2"], position=[0.0, 0.0]))

    monkeypatch.setattr(
        FakeScene,
        "computeFrameJacobian",
        lambda self, q, frame_name, local=True: np.ones((5, 2)),
    )
    with pytest.raises(ValueError, match="Unexpected RoboPlan Jacobian shape"):
        world.get_group_jacobian(ctx, "manipulator")

    monkeypatch.setattr(
        FakeScene,
        "computeFrameJacobian",
        lambda self, q, frame_name, local=True: np.ones((6, 4)),
    )
    with pytest.raises(ValueError, match="cannot project"):
        world.get_group_jacobian(ctx, "manipulator")


def test_kinematics_convenience_methods_require_unique_pose_group(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    no_pose_config = robot_config.model_copy(
        update={
            "planning_groups": [
                PlanningGroupDefinition(name="base", joint_names=("joint1",), base_link="base")
            ]
        }
    )
    no_pose_world = _make_world(fake_roboplan, no_pose_config)
    with pytest.raises(ValueError, match="no pose-targetable"):
        no_pose_world.get_ee_pose(no_pose_world.get_live_context())
    with pytest.raises(ValueError, match="no pose-targetable"):
        no_pose_world.get_jacobian(no_pose_world.get_live_context())

    ambiguous_config = robot_config.model_copy(
        update={
            "planning_groups": [
                PlanningGroupDefinition(
                    name="a", joint_names=("joint1",), base_link="base", tip_link="tcp"
                ),
                PlanningGroupDefinition(
                    name="b", joint_names=("joint2",), base_link="base", tip_link="tcp"
                ),
            ]
        }
    )
    ambiguous_world = _make_world(fake_roboplan, ambiguous_config)
    with pytest.raises(ValueError, match="pose-targetable planning groups"):
        ambiguous_world.get_jacobian(ambiguous_world.get_live_context())


def test_group_lookup_rejects_unknown_group_id(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)

    with pytest.raises(KeyError, match="Unknown planning group ID"):
        world.get_group_ee_pose(world.get_live_context(), "missing")


def test_native_planner_converts_path(fake_roboplan: None, robot_config: RobotModelConfig) -> None:
    world = _make_world(fake_roboplan, robot_config)

    start = JointState(name=["joint1", "joint2"], position=[0.0, 0.0])
    goal = JointState(name=["joint1", "joint2"], position=[0.4, 0.2])
    result = _planner_for(world).plan_joint_path(world, start, goal, timeout=1.0)

    assert result.status == PlanningStatus.SUCCESS
    assert [state.position for state in result.path] == [[0.0, 0.0], [0.2, 0.1], [0.4, 0.2]]
    assert [state.name for state in result.path] == [["joint1", "joint2"]] * 3


def test_native_planner_shortcuts_path_with_configured_options(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    shortcutting = RoboPlanPathShortcuttingConfig(
        max_step_size=0.02,
        max_iters=40,
        seed=7,
        max_convergence_iters=8,
        redundant_removal_iters=5,
    )
    world = _make_world(
        fake_roboplan,
        robot_config,
        RoboPlanPlannerConfig(path_shortcutting=shortcutting),
    )

    def endpoints_only(shortcutter: FakePathShortcutter, path: FakeJointPath) -> FakeJointPath:
        return FakeJointPath(path.joint_names, [path.positions[0], path.positions[-1]])

    mocker.patch.object(
        FakePathShortcutter,
        "shortcut",
        autospec=True,
        side_effect=endpoints_only,
    )

    result = _planner_for(world).plan_joint_path(
        world,
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
        JointState(name=["joint1", "joint2"], position=[0.4, 0.2]),
        timeout=1.0,
    )

    assert [state.position for state in result.path] == [[0.0, 0.0], [0.4, 0.2]]
    options = FakePathShortcutter.instances[-1].options
    assert options.group_name == "__dimos_all_configured__"
    assert options.max_step_size == 0.02
    assert options.max_iters == 40
    assert options.seed == 7
    assert options.max_convergence_iters == 8
    assert options.redundant_removal_iters == 5


def test_native_planner_can_disable_path_shortcutting(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world = _make_world(
        fake_roboplan,
        robot_config,
        RoboPlanPlannerConfig(path_shortcutting=RoboPlanPathShortcuttingConfig(enabled=False)),
    )

    shortcut = mocker.patch.object(FakePathShortcutter, "shortcut", autospec=True)

    result = _planner_for(world).plan_joint_path(
        world,
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
        JointState(name=["joint1", "joint2"], position=[0.4, 0.2]),
        timeout=1.0,
    )

    assert [state.position for state in result.path] == [[0.0, 0.0], [0.2, 0.1], [0.4, 0.2]]
    shortcut.assert_not_called()


def test_native_planner_uses_raw_path_when_shortcutting_fails(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    mocker.patch.object(
        FakePathShortcutter,
        "shortcut",
        autospec=True,
        side_effect=RuntimeError("shortcut failed"),
    )

    result = _planner_for(world).plan_joint_path(
        world,
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
        JointState(name=["joint1", "joint2"], position=[0.4, 0.2]),
        timeout=1.0,
    )

    assert result.status == PlanningStatus.SUCCESS
    assert [state.position for state in result.path] == [[0.0, 0.0], [0.2, 0.1], [0.4, 0.2]]


def test_native_planner_surfaces_unexpected_shortcutting_error(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    mocker.patch.object(
        FakePathShortcutter,
        "shortcut",
        autospec=True,
        side_effect=TypeError("unexpected shortcut integration error"),
    )

    with pytest.raises(TypeError, match="unexpected shortcut integration error"):
        _planner_for(world).plan_joint_path(
            world,
            JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
            JointState(name=["joint1", "joint2"], position=[0.4, 0.2]),
            timeout=1.0,
        )


@pytest.mark.parametrize("endpoint_index", [0, -1], ids=["start", "goal"])
def test_native_planner_uses_raw_path_when_shortcutting_changes_endpoint(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
    endpoint_index: int,
) -> None:
    world = _make_world(fake_roboplan, robot_config)

    def changed_endpoint(shortcutter: FakePathShortcutter, path: FakeJointPath) -> FakeJointPath:
        positions = [*path.positions]
        positions[endpoint_index] = np.asarray(positions[endpoint_index]) + 0.1
        return FakeJointPath(path.joint_names, positions)

    mocker.patch.object(
        FakePathShortcutter,
        "shortcut",
        autospec=True,
        side_effect=changed_endpoint,
    )

    result = _planner_for(world).plan_joint_path(
        world,
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
        JointState(name=["joint1", "joint2"], position=[0.4, 0.2]),
        timeout=1.0,
    )

    assert result.status == PlanningStatus.SUCCESS
    assert [state.position for state in result.path] == [[0.0, 0.0], [0.2, 0.1], [0.4, 0.2]]


def test_native_planner_uses_raw_path_when_shortcutting_returns_empty_path(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    mocker.patch.object(
        FakePathShortcutter,
        "shortcut",
        autospec=True,
        return_value=FakeJointPath(["joint1", "joint2"], []),
    )

    result = _planner_for(world).plan_joint_path(
        world,
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
        JointState(name=["joint1", "joint2"], position=[0.4, 0.2]),
        timeout=1.0,
    )

    assert result.status == PlanningStatus.SUCCESS
    assert [state.position for state in result.path] == [[0.0, 0.0], [0.2, 0.1], [0.4, 0.2]]


def test_roboplan_planner_is_distinct_and_bound_to_world(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    planner = _planner_for(world)

    assert planner is not world
    assert planner._world is world
    assert not hasattr(world, "plan_joint_path")


def test_roboplan_planner_copies_configuration(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    config = RoboPlanPlannerConfig(path_shortcutting=RoboPlanPathShortcuttingConfig(enabled=False))
    world = _make_world(fake_roboplan, robot_config, config)

    config.path_shortcutting.enabled = True

    assert not _planner_for(world)._config.path_shortcutting.enabled


def test_native_planning_blocks_obstacle_replacement(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    obstacle = Obstacle(
        name="box",
        obstacle_type=ObstacleType.BOX,
        pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        dimensions=(0.1, 0.2, 0.3),
    )
    world.add_obstacle(obstacle)
    planning_started = threading.Event()
    allow_planning = threading.Event()
    update_finished = threading.Event()
    original_plan = FakeRRT.plan

    def blocking_plan(
        self: FakeRRT,
        q_start: FakeJointConfiguration,
        q_goal: FakeJointConfiguration,
    ) -> FakeJointPath:
        planning_started.set()
        assert allow_planning.wait(1.0)
        return original_plan(self, q_start, q_goal)

    monkeypatch.setattr(FakeRRT, "plan", blocking_plan)
    start = JointState(name=["joint1", "joint2"], position=[0.0, 0.0])
    goal = JointState(name=["joint1", "joint2"], position=[0.2, 0.1])
    planning_thread = threading.Thread(
        target=lambda: _planner_for(world).plan_joint_path(world, start, goal, timeout=1.0)
    )
    planning_thread.start()
    assert planning_started.wait(1.0)
    update_thread = threading.Thread(
        target=lambda: (
            world.update_obstacle(replace(obstacle, dimensions=(1.0, 1.0, 1.0))),
            update_finished.set(),
        )
    )
    update_thread.start()
    assert not update_finished.wait(0.05)
    allow_planning.set()
    planning_thread.join(1.0)
    update_thread.join(1.0)
    assert update_finished.is_set()


def test_native_planner_names_path_from_robot_config_when_start_is_unnamed(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)

    start = JointState(name=[], position=[0.0, 0.0])
    goal = JointState(name=["joint1", "joint2"], position=[0.4, 0.2])
    result = _planner_for(world).plan_joint_path(world, start, goal, timeout=1.0)

    assert result.status == PlanningStatus.SUCCESS
    assert [state.name for state in result.path] == [["joint1", "joint2"]] * 3


def test_native_selected_planner_returns_canonical_selected_joint_names(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    selection = _selection(robot_config, "manipulator")

    result = _planner_for(world).plan_selected_joint_path(
        world,
        selection,
        JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
        JointState(name=["joint1", "joint2"], position=[0.4, 0.2]),
        timeout=1.0,
    )

    assert result.status == PlanningStatus.SUCCESS
    assert [state.name for state in result.path] == [["joint1", "joint2"]] * 3
    assert [state.position for state in result.path] == [[0.0, 0.0], [0.2, 0.1], [0.4, 0.2]]


def test_native_selected_planner_uses_explicit_start_after_live_state_advances(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    selection = _selection(robot_config, "manipulator")
    observed_scene_start: list[float] = []
    native_plan = FakeRRT.plan

    def capture_scene_start(
        planner: FakeRRT,
        q_start: FakeJointConfiguration,
        q_goal: FakeJointConfiguration,
    ) -> FakeJointPath:
        observed_scene_start.extend(planner.scene.current_positions.tolist())
        return native_plan(planner, q_start, q_goal)

    mocker.patch.object(FakeRRT, "plan", autospec=True, side_effect=capture_scene_start)
    start = JointState(name=list(selection.joint_names), position=[0.1, -0.1])
    world.sync_from_joint_state(
        JointState(name=["joint1", "joint2"], position=[0.3, 0.2]),
    )

    result = _planner_for(world).plan_selected_joint_path(
        world,
        selection,
        start,
        JointState(name=list(selection.joint_names), position=[0.4, 0.2]),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert result.path[0].position == pytest.approx(start.position)
    assert observed_scene_start[:2] == pytest.approx(start.position)


def test_native_selected_planner_supports_non_overlapping_multi_group_selection(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    config = robot_config.model_copy(
        update={
            "planning_groups": [
                PlanningGroupDefinition("left", ("joint1",), "base", "tcp"),
                PlanningGroupDefinition("right", ("joint2",), "base", "tcp"),
            ]
        }
    )
    world = _make_world(fake_roboplan, config)
    selection = _selection(config, "left", "right")

    result = _planner_for(world).plan_selected_joint_path(
        world,
        selection,
        JointState(name=list(selection.joint_names), position=[0.0, 0.0]),
        JointState(name=list(selection.joint_names), position=[0.1, 0.1]),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert result.path[-1].name == ["joint1", "joint2"]
    assert result.path[-1].position == pytest.approx([0.1, 0.1])


def test_cartesian_planner_returns_timed_canonical_joint_states_and_options(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    selection = _selection(robot_config, "manipulator")
    option_overrides = {
        "dt": 0.02,
        "max_linear_speed": 0.2,
        "max_angular_speed": 0.6,
        "max_linear_acceleration": 0.7,
        "max_angular_acceleration": 2.0,
        "max_position_error": 0.006,
        "max_orientation_error": 0.02,
        "position_cost": 2.0,
        "orientation_cost": 3.0,
        "task_gain": 0.8,
        "lm_damping": 0.02,
        "regularization": 2e-6,
        "config_task_weight": 0.1,
        "velocity_scale": 0.9,
        "acceleration_scale": 0.8,
        "toppra_blend_deviation": 0.0,
        "position_limit_gain": 0.7,
    }

    result = _planner_for(world).plan_cartesian_path(
        world,
        selection,
        JointState(name=list(selection.joint_names), position=[0.0, 0.0]),
        {
            "manipulator": _relative_target(
                Transform(
                    translation=Vector3(0.1, 0.0, 0.0),
                    rotation=Quaternion.from_euler(Vector3(0.0, 0.0, np.pi / 2.0)),
                )
            )
        },
        RoboPlanCartesianPathConfig(
            speed_mode="time_optimal",
            **option_overrides,
        ),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert result.timestamps == [0.0, 0.02, 0.04]
    assert [state.name for state in result.path] == [list(selection.joint_names)] * 3
    assert result.path[-1].position == pytest.approx([0.1, 0.1])
    assert result.path[1].velocity == pytest.approx([0.5, 0.5])
    planner = FakeCartesianPathPlanner.instances[-1]
    assert planner.options.speed_mode == FakeCartesianSpeedMode.TimeOptimal
    for field_name, expected in option_overrides.items():
        assert getattr(planner.options, field_name) == pytest.approx(expected)
    assert planner.paths[0].base_frames == ["dimos_world"]
    np.testing.assert_allclose(planner.paths[0].tforms[0][0], np.eye(4), atol=1e-12)
    expected_rotation = np.array(
        [
            [0.0, -1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    np.testing.assert_allclose(
        planner.paths[0].tforms[0][1][:3, :3],
        expected_rotation,
        atol=1e-12,
    )


def test_cartesian_zero_rotation_preserves_start_orientation(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    selection = _selection(robot_config, "manipulator")

    result = _planner_for(world).plan_cartesian_path(
        world,
        selection,
        JointState(name=list(selection.joint_names), position=[0.0, 0.0]),
        {
            "manipulator": _relative_target(
                Transform(translation=Vector3(0.05, 0.02, 0.0)),
                Transform(translation=Vector3(0.1, 0.0, 0.0)),
            )
        },
        RoboPlanCartesianPathConfig(),
    )

    assert result.status == PlanningStatus.SUCCESS
    track = FakeCartesianPathPlanner.instances[-1].paths[0].tforms[0]
    assert len(track) == 3
    for waypoint in track[1:]:
        np.testing.assert_allclose(waypoint[:3, :3], track[0][:3, :3], atol=1e-12)


def test_cartesian_uses_explicit_start_after_live_state_advances(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    selection = _selection(robot_config, "manipulator")
    start = JointState(name=list(selection.joint_names), position=[0.1, 0.0])
    world.sync_from_joint_state(
        JointState(name=["joint1", "joint2"], position=[0.3, 0.2]),
    )

    result = _planner_for(world).plan_cartesian_path(
        world,
        selection,
        start,
        {"manipulator": _relative_target(Transform(translation=Vector3(0.1, 0.0, 0.0)))},
        RoboPlanCartesianPathConfig(),
    )

    assert result.status == PlanningStatus.SUCCESS
    assert result.path[0].position == pytest.approx(start.position)
    start_pose = FakeCartesianPathPlanner.instances[-1].paths[0].tforms[0][0]
    assert start_pose[0, 3] == pytest.approx(sum(start.position))


def test_cartesian_rejects_official_planner_failure(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    selection = _selection(robot_config, "manipulator")
    mocker.patch.object(
        FakeCartesianPathPlanner,
        "plan",
        autospec=True,
        side_effect=ValueError("tracking failed"),
    )

    result = _planner_for(world).plan_cartesian_path(
        world,
        selection,
        JointState(name=list(selection.joint_names), position=[0.0, 0.0]),
        {"manipulator": _relative_target(Transform(translation=Vector3(0.1, 0.0, 0.0)))},
        RoboPlanCartesianPathConfig(),
    )

    assert result.status == PlanningStatus.NO_SOLUTION
    assert "tracking failed" in result.message


def test_cartesian_postvalidation_checks_between_waypoints(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world = _make_world(fake_roboplan, robot_config)
    selection = _selection(robot_config, "manipulator")

    def two_point_trajectory(
        planner: FakeCartesianPathPlanner,
        path: FakeCartesianPath,
        q_start: FakeJointConfiguration,
    ) -> FakeJointTrajectory:
        start = np.asarray(q_start.positions, dtype=np.float64)
        goal = start.copy()
        goal[:2] = 0.1
        zeros = np.zeros_like(start)
        return FakeJointTrajectory(
            q_start.joint_names,
            [start, goal],
            [zeros, zeros],
            [0.0, 0.1],
        )

    def collides_only_mid_edge(scene: FakeScene, q: np.ndarray) -> bool:
        return 0.04 < q[0] < 0.06

    mocker.patch.object(
        FakeCartesianPathPlanner,
        "plan",
        autospec=True,
        side_effect=two_point_trajectory,
    )
    mocker.patch.object(
        FakeScene,
        "hasCollisions",
        autospec=True,
        side_effect=collides_only_mid_edge,
    )

    result = _planner_for(world).plan_cartesian_path(
        world,
        selection,
        JointState(name=list(selection.joint_names), position=[0.0, 0.0]),
        {"manipulator": _relative_target(Transform(translation=Vector3(0.05, 0.0, 0.0)))},
        RoboPlanCartesianPathConfig(),
    )

    assert result.status == PlanningStatus.NO_SOLUTION
    assert result.path == []
    assert "collision post-validation" in result.message


def test_native_planner_rejects_empty_path(
    fake_roboplan: None, robot_config: RobotModelConfig, monkeypatch: pytest.MonkeyPatch
) -> None:
    class EmptyPathRRT(FakeRRT):
        def plan(
            self, q_start: FakeJointConfiguration, q_goal: FakeJointConfiguration
        ) -> FakeJointPath:
            return FakeJointPath(["joint1", "joint2"], [])

    monkeypatch.setattr(sys.modules["roboplan.rrt"], "RRT", EmptyPathRRT)
    world = _make_world(fake_roboplan, robot_config)

    start = JointState(name=["joint1", "joint2"], position=[0.0, 0.0])
    goal = JointState(name=["joint1", "joint2"], position=[0.4, 0.2])
    result = _planner_for(world).plan_joint_path(world, start, goal, timeout=1.0)

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
    world = _make_world(fake_roboplan, robot_config)

    srdf = world._scene.constructor_kwargs["srdf"]
    assert 'disable_collisions link1="base" link2="link2"' in srdf
    assert "other_base" not in srdf


def test_generated_srdf_uses_python_planning_group_definitions(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    robot_config.planning_groups = [
        PlanningGroupDefinition("left_arm", ("joint1",), "base", "link1"),
        PlanningGroupDefinition("right_arm", ("joint2",), "link1", "link2"),
        PlanningGroupDefinition("both_arms", ("joint1", "joint2"), "base"),
    ]
    assert robot_config.srdf_path is None

    world = _make_world(fake_roboplan, robot_config)
    root = ET.fromstring(world._scene.constructor_kwargs["srdf"])
    configured = {
        group.get("name"): [joint.get("name") for joint in group.findall("joint")]
        for group in root.findall("group")
        if group.get("name") in {"left_arm", "right_arm", "both_arms"}
    }

    assert configured == {
        "left_arm": ["joint1"],
        "right_arm": ["joint2"],
        "both_arms": ["joint1", "joint2"],
    }


def test_collision_exclusion_with_one_unknown_link_is_rejected(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    robot_config.collision_exclusion_pairs = [("base", "missing")]
    module = _import_roboplan_world(fake_roboplan)
    world = module.RoboPlanWorld()
    world.load_model(robot_config)

    with pytest.raises(
        ValueError,
        match=r"collision exclusion references unknown links: base <-> missing",
    ):
        world.finalize()


def test_scene_receives_generated_model_contents_inline(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world = _make_world(fake_roboplan, robot_config)

    urdf = ET.fromstring(world._scene.constructor_kwargs["urdf"])
    srdf = ET.fromstring(world._scene.constructor_kwargs["srdf"])
    assert urdf.tag == "robot"
    assert srdf.tag == "robot"
    assert world._scene.constructor_kwargs["package_paths"] == []


def test_composed_model_fills_only_missing_acceleration_limits(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    model_path = Path(robot_config.model.source_path)
    tree = ET.parse(model_path)
    authored = tree.find("./joint[@name='joint1']/limit")
    assert authored is not None
    authored.set("acceleration", "3.5")
    tree.write(model_path)

    world = _make_world(fake_roboplan, robot_config)

    urdf = ET.fromstring(world._scene.constructor_kwargs["urdf"])
    acceleration_by_joint = {
        joint.get("name"): limit.get("acceleration")
        for joint in urdf.findall("./joint")
        if (limit := joint.find("./limit")) is not None
    }
    assert acceleration_by_joint == {
        "joint1": "3.5",
        "joint2": "2.0",
        "joint3": "2.0",
    }


def test_base_pose_is_written_to_composed_model(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    robot_config.base_pose = PoseStamped(  # type: ignore[call-arg]
        position=Vector3(1, 0, 0), orientation=Quaternion()
    )
    world = _make_world(fake_roboplan, robot_config)

    urdf_root = ET.fromstring(world._scene.constructor_kwargs["urdf"])
    origin = urdf_root.find("./joint[@name='dimos_world_joint']/origin")
    assert origin is not None
    assert origin.get("xyz") == "1 0 0"
