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

from collections.abc import Iterator
from dataclasses import replace
import importlib
from pathlib import Path
import subprocess
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
from dimos.manipulation.planning.spec.enums import IKStatus, ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import Obstacle
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.transform_utils import pose_to_matrix

_ARGUMENT_OMITTED = object()


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
    def __init__(self) -> None:
        self.base_frame = ""
        self.tip_frame = ""
        self.tform = np.eye(4)


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
            if self.joint_group_joint_names is not None and name == self.constructor_kwargs["name"]
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

    def integrate(self, q: np.ndarray, delta_q: np.ndarray) -> np.ndarray:
        return np.asarray(q) + np.asarray(delta_q)

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


class FakeFrameTaskOptions:
    pass


class FakeFrameTask:
    instances: ClassVar[list[FakeFrameTask]] = []

    def __init__(
        self,
        oink: FakeOink,
        scene: FakeScene,
        target: FakeCartesianConfiguration,
        options: FakeFrameTaskOptions | None = None,
    ) -> None:
        self.oink = oink
        self.scene = scene
        self.target = target
        self.options = options or FakeFrameTaskOptions()
        self.instances.append(self)


class FakePositionLimit:
    def __init__(self, oink: FakeOink, gain: float = 1.0) -> None:
        self.oink = oink
        self.gain = gain


class FakeOink:
    instances: ClassVar[list[FakeOink]] = []
    step: ClassVar[float] = 0.25
    solve_error: ClassVar[Exception | None] = None

    def __init__(self, scene: FakeScene, group_name: str) -> None:
        self.scene = scene
        self.group_name = group_name
        self.q_indices = tuple(
            scene.native_joint_names.index(name) for name in scene.groups[group_name]
        )
        self.v_indices = self.q_indices
        self.num_variables = len(self.v_indices)
        self.solve_calls: list[tuple[list[FakeFrameTask], list[Any], object]] = []
        self.scene_positions: list[np.ndarray] = []
        self.instances.append(self)

    def solveIk(
        self,
        scene: FakeScene,
        tasks: list[FakeFrameTask],
        constraints: list[Any],
        delta_q: np.ndarray,
        regularization: float | object = _ARGUMENT_OMITTED,
    ) -> None:
        self.solve_calls.append((tasks, constraints, regularization))
        self.scene_positions.append(scene.current_positions.copy())
        if self.solve_error is not None:
            raise self.solve_error
        delta_q[:] = self.step


def _install_fake_roboplan(monkeypatch: pytest.MonkeyPatch) -> None:
    roboplan_pkg = ModuleType("roboplan")
    roboplan_pkg.__path__ = []  # type: ignore[attr-defined]
    core = ModuleType("roboplan.core")
    core.Scene = FakeScene  # type: ignore[attr-defined]
    core.JointConfiguration = FakeJointConfiguration  # type: ignore[attr-defined]
    core.JointPath = FakeJointPath  # type: ignore[attr-defined]
    core.CartesianConfiguration = FakeCartesianConfiguration  # type: ignore[attr-defined]
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

    optimal_ik = ModuleType("roboplan.optimal_ik")
    optimal_ik.Oink = FakeOink  # type: ignore[attr-defined]
    optimal_ik.FrameTaskOptions = FakeFrameTaskOptions  # type: ignore[attr-defined]
    optimal_ik.FrameTask = FakeFrameTask  # type: ignore[attr-defined]
    optimal_ik.PositionLimit = FakePositionLimit  # type: ignore[attr-defined]

    FakeFrameTask.instances = []
    FakeOink.instances = []
    FakeOink.step = 0.25
    FakeOink.solve_error = None

    monkeypatch.setitem(sys.modules, "roboplan", roboplan_pkg)
    monkeypatch.setitem(sys.modules, "roboplan.core", core)
    monkeypatch.setitem(sys.modules, "roboplan.rrt", rrt)
    monkeypatch.setitem(sys.modules, "roboplan.optimal_ik", optimal_ik)


@pytest.fixture
def fake_roboplan(monkeypatch: pytest.MonkeyPatch) -> Iterator[None]:
    with monkeypatch.context() as fake_modules:
        _install_fake_roboplan(fake_modules)
        yield

    package = sys.modules.get("dimos.manipulation.planning.world")
    for module_name in (
        "dimos.manipulation.planning.world.roboplan_oink",
        "dimos.manipulation.planning.world.roboplan_world",
    ):
        sys.modules.pop(module_name, None)
        if package is not None:
            module_attribute = module_name.rsplit(".", maxsplit=1)[-1]
            if hasattr(package, module_attribute):
                delattr(package, module_attribute)


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


def _target(x: float, frame_id: str = "world") -> PoseStamped:
    return PoseStamped(
        frame_id=frame_id,
        position=Vector3(x=x),
        orientation=Quaternion(),
    )


def _import_roboplan_world(fake_roboplan: None) -> ModuleType:
    oink_module_name = "dimos.manipulation.planning.world.roboplan_oink"
    if oink_module_name in sys.modules:
        importlib.reload(sys.modules[oink_module_name])
    else:
        importlib.import_module(oink_module_name)
    module_name = "dimos.manipulation.planning.world.roboplan_world"
    if module_name in sys.modules:
        return importlib.reload(sys.modules[module_name])
    return importlib.import_module(module_name)


def test_roboplan_bindings_are_imported_at_module_load(fake_roboplan: None) -> None:
    module = _import_roboplan_world(fake_roboplan)
    oink_module = sys.modules["dimos.manipulation.planning.world.roboplan_oink"]

    assert module.roboplan_core.Scene is FakeScene
    assert oink_module.roboplan_optimal_ik.Oink is FakeOink
    assert module.roboplan_rrt.RRT is FakeRRT


def test_missing_bundled_oink_fails_fast_without_pink_fallback() -> None:
    script = """
import builtins
import sys
from types import ModuleType

roboplan = ModuleType("roboplan")
roboplan.__path__ = []
sys.modules["roboplan"] = roboplan
sys.modules["roboplan.core"] = ModuleType("roboplan.core")
sys.modules["roboplan.rrt"] = ModuleType("roboplan.rrt")

real_import = builtins.__import__

def blocked_import(name, *args, **kwargs):
    if name == "roboplan.optimal_ik":
        raise ImportError("simulated missing bundled OInK")
    return real_import(name, *args, **kwargs)

builtins.__import__ = blocked_import
try:
    import dimos.manipulation.planning.world.roboplan_world
except ImportError as exc:
    assert "Install the manipulation extra" in str(exc)
else:
    raise AssertionError("Missing roboplan.optimal_ik did not fail fast")

assert "dimos.manipulation.planning.kinematics.pink_ik" not in sys.modules
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


def test_robot_registration_finalization_and_joint_limits(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)

    assert world.get_robot_ids() == [robot_id]
    assert world.get_robot_config(robot_id) is robot_config
    assert world._scene.constructor_kwargs["name"] == "arm"
    assert ET.fromstring(world._scene.constructor_kwargs["urdf"]).get("name") == "arm"
    assert world._scene.constructor_kwargs["srdf"].startswith('<robot name="arm">')
    assert (
        'disable_collisions link1="base" link2="link1"' in world._scene.constructor_kwargs["srdf"]
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
    np.testing.assert_allclose(
        pose_to_matrix(world.get_obstacles()[0].pose),
        pose_to_matrix(updated_pose),
    )
    np.testing.assert_allclose(world._scene.geometry["box"], pose_to_matrix(updated_pose))
    assert world.add_obstacle(obstacle) is None
    assert world.remove_obstacle("box")
    assert world.get_obstacles() == []


def test_obstacle_operations_require_finalization(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    module = _import_roboplan_world(fake_roboplan)
    world = module.RoboPlanWorld()
    world.add_robot(robot_config)
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
    world, _ = _make_world(fake_roboplan, robot_config)

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
    world, _ = _make_world(fake_roboplan, robot_config)
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
    world, _ = _make_world(fake_roboplan, robot_config)
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
    world, robot_id = _make_world(fake_roboplan, robot_config)
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
                robot_id,
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
    world, robot_id = _make_world(fake_roboplan, robot_config)
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
            robot_id,
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
    world, robot_id = _make_world(fake_roboplan, robot_config)
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
            robot_id,
            JointState(name=["joint1", "joint2"], position=[0.0, 0.0]),
        )


def test_native_pose_update_failure_invalidates_world(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)
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


def test_generic_planner_allows_update_between_collision_checks(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
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

    def checking_with_interleaved_update(robot: str, state: JointState) -> bool:
        nonlocal checks, updated
        result = original_check(robot, state)
        checks += 1
        if checks == 1:
            updated = world.update_obstacle(replace(obstacle, dimensions=(1.0, 1.0, 1.0)))
        return result

    monkeypatch.setattr(world, "check_config_collision_free", checking_with_interleaved_update)
    planner = RRTConnectPlanner(step_size=0.5, connect_step_size=0.5, goal_tolerance=10.0)
    result = planner.plan_joint_path(
        world,
        robot_id,
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


def test_oink_single_target_uses_defaults_and_restores_scene(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    world.sync_from_joint_state(
        robot_id,
        JointState(name=["joint1", "joint2"], position=[0.1, 0.2]),
    )
    expected_scene_q = world._full_scene_q(world.get_live_context())

    result = world.solve(
        world,
        robot_id,
        _target(0.8),
        seed=JointState(
            name=["arm/joint2", "arm/joint1"],
            position=[0.2, 0.1],
        ),
        check_collision=False,
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["arm/joint1", "arm/joint2"]
    assert result.joint_state.position == pytest.approx([0.35, 0.45])
    assert result.iterations == 1
    assert len(FakeOink.instances) == 1
    oink = FakeOink.instances[0]
    assert oink.group_name == "manipulator"
    assert len(oink.solve_calls) == 1
    tasks, constraints, regularization = oink.solve_calls[0]
    assert tasks == FakeFrameTask.instances
    assert len(constraints) == 1
    assert isinstance(constraints[0], FakePositionLimit)
    assert regularization is _ARGUMENT_OMITTED
    assert FakeFrameTask.instances[0].target.base_frame == ""
    assert FakeFrameTask.instances[0].target.tip_frame == "tcp"
    np.testing.assert_allclose(world._scene.current_positions, expected_scene_q)


def test_oink_reports_worst_multi_target_error_and_caps_each_attempt(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = robot_config.model_copy(
        update={
            "planning_groups": [
                PlanningGroupDefinition("shoulder", ("joint1",), "base", "tcp"),
                PlanningGroupDefinition("wrist", ("joint2",), "link1", "tcp"),
            ]
        }
    )
    world, _ = _make_world(fake_roboplan, config)
    FakeOink.step = 0.0
    _import_roboplan_world(fake_roboplan)
    oink_module = sys.modules["dimos.manipulation.planning.world.roboplan_oink"]
    monkeypatch.setattr(oink_module, "_MAX_ITERATIONS_PER_ATTEMPT", 3)
    shoulder = world._planning_groups.get("arm/shoulder")
    wrist = world._planning_groups.get("arm/wrist")

    result = world.solve_pose_targets(
        world,
        {shoulder: _target(0.2), wrist: _target(0.8)},
        check_collision=False,
        max_attempts=1,
    )

    assert result.status == IKStatus.NO_SOLUTION
    assert result.position_error == pytest.approx(0.8)
    assert result.orientation_error == pytest.approx(0.0)
    assert result.iterations == 3
    assert len(FakeOink.instances[0].solve_calls) == 3


def test_oink_partial_seed_auxiliary_only_preserves_selection_order(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    world.sync_from_joint_state(
        robot_id,
        JointState(name=["joint1", "joint2"], position=[0.1, 0.2]),
    )
    group = world._planning_groups.get("arm/manipulator")

    result = world.solve_pose_targets(
        world,
        {},
        auxiliary_groups=(group,),
        seed=JointState(name=["arm/joint2"], position=[0.6]),
        check_collision=False,
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == ["arm/joint1", "arm/joint2"]
    assert result.joint_state.position == pytest.approx([0.1, 0.6])
    assert result.iterations == 0
    assert FakeOink.instances == []


def test_oink_restarts_only_pose_joints_and_reuses_one_instance(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
) -> None:
    config = robot_config.model_copy(
        update={
            "planning_groups": [
                PlanningGroupDefinition("pose", ("joint1",), "base", "tcp"),
                PlanningGroupDefinition("aux", ("joint2",), "link1"),
            ]
        }
    )
    world, robot_id = _make_world(fake_roboplan, config)
    world.sync_from_joint_state(
        robot_id,
        JointState(name=["joint1", "joint2"], position=[0.1, 0.4]),
    )
    _import_roboplan_world(fake_roboplan)
    oink_module = sys.modules["dimos.manipulation.planning.world.roboplan_oink"]
    monkeypatch.setattr(oink_module, "_MAX_ITERATIONS_PER_ATTEMPT", 1)
    FakeOink.step = 0.0
    random_uniform = mocker.patch(
        "dimos.manipulation.planning.world.roboplan_oink.np.random.uniform",
        return_value=np.asarray([0.7]),
    )

    result = world.solve_pose_targets(
        world,
        {world._planning_groups.get("arm/pose"): _target(10.0)},
        auxiliary_groups=(world._planning_groups.get("arm/aux"),),
        check_collision=False,
        max_attempts=2,
    )

    assert result.status == IKStatus.NO_SOLUTION
    assert len(FakeOink.instances) == 1
    oink = FakeOink.instances[0]
    assert len(oink.solve_calls) == 2
    np.testing.assert_allclose(oink.scene_positions[0][:2], [0.1, 0.4])
    np.testing.assert_allclose(oink.scene_positions[1][:2], [0.7, 0.4])
    random_uniform.assert_called_once()


def test_oink_collision_precedence_and_collision_override(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    seed = JointState(name=["joint1", "joint2"], position=[0.85, 0.0])

    collision = world.solve(
        world,
        robot_id,
        _target(1.25),
        seed=seed,
        check_collision=True,
        max_attempts=1,
    )
    accepted = world.solve(
        world,
        robot_id,
        _target(1.25),
        seed=seed,
        check_collision=False,
        max_attempts=1,
    )

    assert collision.status == IKStatus.COLLISION
    assert collision.joint_state is None
    assert accepted.status == IKStatus.SUCCESS
    assert accepted.joint_state is not None
    assert max(accepted.joint_state.position) > 0.9


def test_oink_collision_free_endpoint_is_checked_and_accepted(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    has_collisions = mocker.spy(FakeScene, "hasCollisions")

    result = world.solve(
        world,
        robot_id,
        _target(0.5),
        check_collision=True,
        max_attempts=1,
    )

    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    has_collisions.assert_called_once()


def test_oink_no_solution_reports_closest_attempt(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)
    expected_scene_q = world._full_scene_q(world.get_live_context())
    _import_roboplan_world(fake_roboplan)
    oink_module = sys.modules["dimos.manipulation.planning.world.roboplan_oink"]
    monkeypatch.setattr(oink_module, "_MAX_ITERATIONS_PER_ATTEMPT", 1)
    FakeOink.step = 0.0
    mocker.patch(
        "dimos.manipulation.planning.world.roboplan_oink.np.random.uniform",
        return_value=np.asarray([0.4, 0.4]),
    )
    group = world._planning_groups.get("arm/manipulator")

    result = world.solve_pose_targets(
        world,
        {group: _target(0.9)},
        position_tolerance=0.01,
        orientation_tolerance=0.01,
        check_collision=False,
        max_attempts=2,
    )

    assert result.status == IKStatus.NO_SOLUTION
    assert result.joint_state is None
    assert result.position_error == pytest.approx(0.1)
    assert result.orientation_error == pytest.approx(0.0)
    np.testing.assert_allclose(world._scene.current_positions, expected_scene_q)


def test_oink_supports_multi_robot_composite_selection(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    world, _, _, second_config = _make_two_robot_world(fake_roboplan, robot_config)
    right = world._planning_groups.get("right/manipulator")
    left = world._planning_groups.get("arm/manipulator")

    result = world.solve_pose_targets(
        world,
        {right: _target(1.0), left: _target(1.0)},
        check_collision=False,
        max_attempts=1,
    )

    assert second_config.name == "right"
    assert result.status == IKStatus.SUCCESS
    assert result.joint_state is not None
    assert result.joint_state.name == [
        "right/joint1",
        "right/joint2",
        "arm/joint1",
        "arm/joint2",
    ]
    assert result.joint_state.position == pytest.approx([0.25] * 4)
    assert len(FakeFrameTask.instances) == 2
    assert {task.target.tip_frame for task in FakeFrameTask.instances} == {
        "arm__tcp",
        "right__tcp",
    }


def test_oink_multi_robot_collision_applies_composite_endpoint(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    mocker: MockerFixture,
) -> None:
    world, _, _, _ = _make_two_robot_world(fake_roboplan, robot_config)
    left = world._planning_groups.get("arm/manipulator")
    right = world._planning_groups.get("right/manipulator")
    has_collisions = mocker.patch.object(
        FakeScene,
        "hasCollisions",
        autospec=True,
        side_effect=FakeScene.hasCollisions,
    )

    result = world.solve_pose_targets(
        world,
        {},
        auxiliary_groups=(right, left),
        seed=JointState(
            name=[
                "right/joint1",
                "right/joint2",
                "arm/joint1",
                "arm/joint2",
            ],
            position=[0.95, 0.2, 0.1, 0.3],
        ),
        check_collision=True,
    )

    assert result.status == IKStatus.COLLISION
    checked_q = has_collisions.call_args.args[1]
    checked = dict(zip(world._scene.native_joint_names, checked_q, strict=True))
    assert checked["arm__joint1"] == pytest.approx(0.1)
    assert checked["arm__joint2"] == pytest.approx(0.3)
    assert checked["right__joint1"] == pytest.approx(0.95)
    assert checked["right__joint2"] == pytest.approx(0.2)


def test_oink_rejects_unsupported_frame_and_overlapping_groups(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)
    group = world._planning_groups.get("arm/manipulator")

    frame_result = world.solve_pose_targets(
        world,
        {group: _target(0.0, frame_id="base")},
    )
    overlap_result = world.solve_pose_targets(
        world,
        {group: _target(0.0)},
        auxiliary_groups=(group,),
    )

    assert frame_result.status == IKStatus.UNSUPPORTED
    assert frame_result.joint_state is None
    assert overlap_result.status == IKStatus.UNSUPPORTED
    assert overlap_result.joint_state is None


def test_oink_restores_scene_after_solver_exception(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
    world.sync_from_joint_state(
        robot_id,
        JointState(name=["joint1", "joint2"], position=[0.1, 0.2]),
    )
    expected_scene_q = world._full_scene_q(world.get_live_context())
    FakeOink.solve_error = RuntimeError("qp failed")

    result = world.solve(
        world,
        robot_id,
        _target(10.0),
        check_collision=False,
        max_attempts=1,
    )

    assert result.status == IKStatus.NO_SOLUTION
    assert "qp failed" in result.message
    np.testing.assert_allclose(world._scene.current_positions, expected_scene_q)


def test_oink_single_target_wrapper_requires_unique_pose_group(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
) -> None:
    config = robot_config.model_copy(
        update={
            "planning_groups": [
                PlanningGroupDefinition("first", ("joint1",), "base", "tcp"),
                PlanningGroupDefinition("second", ("joint2",), "link1", "tcp"),
            ]
        }
    )
    world, robot_id = _make_world(fake_roboplan, config)

    result = world.solve(world, robot_id, _target(0.0))

    assert result.status == IKStatus.UNSUPPORTED
    assert "pose-targetable planning groups" in result.message


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


def test_native_planning_blocks_obstacle_replacement(
    fake_roboplan: None,
    robot_config: RobotModelConfig,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world, robot_id = _make_world(fake_roboplan, robot_config)
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
        target=lambda: world.plan_joint_path(world, robot_id, start, goal, timeout=1.0)
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


def test_native_selected_planner_supports_disjoint_same_robot_groups(
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

    assert result.status == PlanningStatus.SUCCESS
    assert result.path[-1].name == ["arm/joint1", "arm/joint2"]
    assert result.path[-1].position == [0.1, 0.1]


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

    srdf = world._scene.constructor_kwargs["srdf"]
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


def test_scene_receives_generated_model_contents_inline(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    world, _ = _make_world(fake_roboplan, robot_config)

    urdf = ET.fromstring(world._scene.constructor_kwargs["urdf"])
    srdf = ET.fromstring(world._scene.constructor_kwargs["srdf"])
    assert urdf.tag == "robot"
    assert srdf.tag == "robot"
    assert world._scene.constructor_kwargs["package_paths"] == []


def test_base_pose_is_written_to_composed_model(
    fake_roboplan: None, robot_config: RobotModelConfig
) -> None:
    robot_config.base_pose = PoseStamped(  # type: ignore[call-arg]
        position=Vector3(1, 0, 0), orientation=Quaternion()
    )
    world, _ = _make_world(fake_roboplan, robot_config)

    urdf_root = ET.fromstring(world._scene.constructor_kwargs["urdf"])
    origin = urdf_root.find("./joint[@name='dimos_world_joint']/origin")
    assert origin is not None
    assert origin.get("xyz") == "1 0 0"
