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

"""Tests for the optional VAMP planning backend adapters."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
import sys
from types import ModuleType

import numpy as np
from numpy.typing import NDArray
from pydantic import ValidationError
import pytest

from dimos.manipulation.planning.factory import create_planner, create_world
from dimos.manipulation.planning.planners.config import RRTConnectPlannerConfig, VampPlannerConfig
from dimos.manipulation.planning.planners.vamp_planner import VampPlanner
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.vamp.errors import (
    UnsupportedWorldCapabilityError,
    VampDependencyError,
)
from dimos.manipulation.planning.vamp.loader import load_vamp_robot_module
from dimos.manipulation.planning.world.config import (
    CustomVampArtifactConfig,
    OfficialVampArtifactConfig,
    VampWorldConfig,
)
from dimos.manipulation.planning.world.vamp_world import VampWorld
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState


@dataclass(frozen=True)
class FakeSphere:
    center: list[float]
    radius: float


@dataclass(frozen=True)
class FakeCuboid:
    center: list[float]
    euler_xyz: list[float]
    half_extents: list[float]


@dataclass(frozen=True)
class FakeCylinder:
    center: list[float]
    euler_xyz: list[float]
    radius: float
    length: float


@dataclass(frozen=True)
class FakePlannerSettings:
    max_iterations: int


@dataclass(frozen=True)
class FakeSimplifySettings:
    enabled: bool = True


@dataclass(frozen=True)
class FakeSampler:
    name: str


class FakeEnvironment:
    """Small fake VAMP environment that records converted primitives."""

    def __init__(self) -> None:
        self.spheres: list[FakeSphere] = []
        self.cuboids: list[FakeCuboid] = []
        self.capsules: list[FakeCylinder] = []

    def add_sphere(self, sphere: FakeSphere) -> None:
        self.spheres.append(sphere)

    def add_cuboid(self, cuboid: FakeCuboid) -> None:
        self.cuboids.append(cuboid)

    def add_capsule(self, capsule: FakeCylinder) -> None:
        self.capsules.append(capsule)


class FakePath:
    """Fake VAMP path value exposing the numpy() method used by bindings."""

    def __init__(self, waypoints: list[list[float]]) -> None:
        self._waypoints = np.array(waypoints, dtype=np.float64)

    def numpy(self) -> NDArray[np.float64]:
        return self._waypoints


class FakePlanningResult:
    """Fake VAMP planning result."""

    def __init__(self, solved: bool, path: FakePath, iterations: int = 7) -> None:
        self.solved = solved
        self.path = path
        self.iterations = iterations


class FakeRobotModule(ModuleType):
    """Fake official VAMP robot module."""

    def __init__(self) -> None:
        super().__init__("vamp.panda")
        self.validate_calls: list[tuple[list[float], bool]] = []
        self.motion_calls: list[tuple[list[float], list[float], bool]] = []
        self.simplify_calls: list[FakePath] = []
        self.valid = True
        self.motion_valid = True

    def halton(self) -> FakeSampler:
        return FakeSampler("fake_sampler")

    def validate(
        self, configuration: list[float], environment: FakeEnvironment, check_bounds: bool
    ) -> bool:
        del environment
        self.validate_calls.append((configuration, check_bounds))
        return self.valid

    def validate_motion(
        self,
        configuration_in: list[float],
        configuration_out: list[float],
        environment: FakeEnvironment,
        check_bounds: bool,
    ) -> bool:
        del environment
        self.motion_calls.append((configuration_in, configuration_out, check_bounds))
        return self.motion_valid

    def eefk(self, configuration: list[float]) -> np.ndarray:
        transform = np.eye(4, dtype=np.float64)
        transform[0, 3] = configuration[0]
        transform[1, 3] = configuration[1]
        transform[2, 3] = configuration[2]
        return transform

    def simplify(
        self,
        path: FakePath,
        environment: FakeEnvironment,
        settings: FakeSimplifySettings,
        sampler: FakeSampler,
    ) -> FakePlanningResult:
        del environment, settings, sampler
        self.simplify_calls.append(path)
        return FakePlanningResult(True, FakePath([[0.0, 0.0, 0.0], [1.0, 0.5, 0.25]]), 2)


class FakeVampModule(ModuleType):
    """Fake top-level VAMP package module."""

    def __init__(self, robot_module: FakeRobotModule) -> None:
        super().__init__("vamp")
        self.panda = robot_module
        self.Environment = FakeEnvironment
        self.Sphere = FakeSphere
        self.Cuboid = FakeCuboid
        self.Cylinder = FakeCylinder
        self.configure_calls: list[tuple[str, str, int]] = []
        self.planner_calls: list[tuple[list[float], list[float], FakeSampler]] = []
        self.planner_solved = True

    def configure_robot_and_planner_with_kwargs(
        self, robot_name: str, planner_name: str, max_iterations: int
    ) -> tuple[
        FakeRobotModule,
        Callable[
            [
                list[float],
                list[float],
                FakeEnvironment,
                FakePlannerSettings,
                FakeSampler,
            ],
            FakePlanningResult,
        ],
        FakePlannerSettings,
        FakeSimplifySettings,
    ]:
        self.configure_calls.append((robot_name, planner_name, max_iterations))

        def planner_func(
            start: list[float],
            goal: list[float],
            environment: FakeEnvironment,
            settings: FakePlannerSettings,
            sampler: FakeSampler,
        ) -> FakePlanningResult:
            del environment, settings
            self.planner_calls.append((start, goal, sampler))
            return FakePlanningResult(
                self.planner_solved,
                FakePath([start, [0.5, 0.25, 0.125], goal]),
                11,
            )

        return (
            self.panda,
            planner_func,
            FakePlannerSettings(max_iterations=max_iterations),
            FakeSimplifySettings(),
        )


@pytest.fixture
def fake_vamp_modules(mocker) -> tuple[FakeVampModule, FakeRobotModule]:
    """Install fake VAMP modules into sys.modules."""
    robot_module = FakeRobotModule()
    vamp_module = FakeVampModule(robot_module)
    mocker.patch.dict(sys.modules, {"vamp": vamp_module, "vamp.panda": robot_module})
    mocker.patch("dimos.manipulation.planning.vamp.loader._vamp_module", vamp_module)
    mocker.patch(
        "dimos.manipulation.planning.vamp.loader._VAMP_OFFICIAL_ROBOT_MODULES",
        {"panda": robot_module},
    )
    return vamp_module, robot_module


def robot_config() -> RobotModelConfig:
    """Create a minimal robot model config for VAMP adapter tests."""
    return RobotModelConfig(
        name="panda",
        model_path=Path("/tmp/panda.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["joint1", "joint2", "joint3"],
        end_effector_link="panda_hand",
        base_link="panda_link0",
        joint_limits_lower=[-1.0, -1.0, -1.0],
        joint_limits_upper=[1.0, 1.0, 1.0],
        home_joints=[0.0, 0.0, 0.0],
    )


def finalized_vamp_world() -> VampWorld:
    """Create a finalized fake-backed VAMP world."""
    world = VampWorld(VampWorldConfig())
    world.add_robot(robot_config())
    world.finalize()
    return world


def test_vamp_dependency_error_has_install_guidance(mocker) -> None:
    """Selecting VAMP without the optional package raises an actionable error."""
    mocker.patch("dimos.manipulation.planning.vamp.loader._vamp_module", None)

    with pytest.raises(VampDependencyError, match="vamp-planner"):
        load_vamp_robot_module(OfficialVampArtifactConfig(robot="panda"))


def test_rrt_planner_creation_works_when_vamp_unavailable(mocker) -> None:
    """Default planner creation still works when the optional VAMP package is unavailable."""
    mocker.patch("dimos.manipulation.planning.vamp.loader._vamp_module", None)

    planner = create_planner(config=RRTConnectPlannerConfig())

    assert planner.get_name() == "RRTConnect"


def test_vamp_config_rejects_invalid_algorithm() -> None:
    """VAMP planner config validates the finite algorithm set."""
    with pytest.raises(ValidationError, match="algorithm"):
        VampPlannerConfig.model_validate({"backend": "vamp", "algorithm": "invalid"})


def test_official_vamp_artifact_loading_uses_installed_robot_module(fake_vamp_modules) -> None:
    """Official artifact mode loads robot modules exposed by the VAMP package."""
    vamp_module, robot_module = fake_vamp_modules

    loaded_vamp, loaded_robot = load_vamp_robot_module(OfficialVampArtifactConfig(robot="panda"))

    assert loaded_vamp is vamp_module
    assert loaded_robot is robot_module


def test_custom_vamp_artifact_loading_uses_explicit_module_path(
    fake_vamp_modules, tmp_path
) -> None:
    """Custom artifact mode imports a user-prepared local Python robot module."""
    vamp_module, _ = fake_vamp_modules
    artifact_path = tmp_path / "custom_panda.py"
    artifact_path.write_text("ROBOT_NAME = 'custom_panda'\n", encoding="utf-8")

    loaded_vamp, loaded_robot = load_vamp_robot_module(CustomVampArtifactConfig(path=artifact_path))

    assert loaded_vamp is vamp_module
    assert isinstance(loaded_robot, ModuleType)
    assert loaded_robot.ROBOT_NAME == "custom_panda"


def test_create_world_and_planner_from_vamp_configs(fake_vamp_modules) -> None:
    """Factory functions create VAMP world and planner adapters from typed configs."""
    world = create_world(config=VampWorldConfig())
    planner = create_planner(config=VampPlannerConfig(algorithm="fcit"))

    assert isinstance(world, VampWorld)
    assert isinstance(planner, VampPlanner)
    assert planner.get_name() == "VAMP/fcit"


def test_vamp_world_validity_fk_and_unsupported_jacobian(fake_vamp_modules) -> None:
    """VAMP world delegates native validity/FK and rejects unsupported Jacobian."""
    _, robot_module = fake_vamp_modules
    world = finalized_vamp_world()
    robot_id = world.get_robot_ids()[0]
    state = JointState(name=["joint1", "joint2", "joint3"], position=[0.1, 0.2, 0.3])

    assert world.check_config_collision_free(robot_id, state)
    assert robot_module.validate_calls[-1] == ([0.1, 0.2, 0.3], True)

    with world.scratch_context() as ctx:
        world.set_joint_state(ctx, robot_id, state)
        ee_pose = world.get_ee_pose(ctx, robot_id)
        assert ee_pose.x == pytest.approx(0.1)
        assert ee_pose.y == pytest.approx(0.2)
        assert ee_pose.z == pytest.approx(0.3)
        with pytest.raises(UnsupportedWorldCapabilityError, match="Jacobian"):
            world.get_jacobian(ctx, robot_id)


def test_vamp_world_accepts_one_robot(fake_vamp_modules) -> None:
    """VAMP world exposes WorldSpec robot APIs through a single robot slot."""
    world = VampWorld(VampWorldConfig())
    config = robot_config()

    robot_id = world.add_robot(config)

    assert robot_id == "robot_1"
    assert world.get_robot_ids() == ["robot_1"]
    assert world.get_robot_config(robot_id) is config
    with pytest.raises(ValueError, match="one robot"):
        world.add_robot(robot_config())


def test_vamp_planner_dispatches_algorithm_simplifies_and_validates(fake_vamp_modules) -> None:
    """VAMP planner uses configured algorithm and VAMP-native path utilities."""
    vamp_module, robot_module = fake_vamp_modules
    world = finalized_vamp_world()
    robot_id = world.get_robot_ids()[0]
    planner = VampPlanner(VampPlannerConfig(algorithm="prm", simplify=True, validate_path=True))
    start = JointState(name=["joint1", "joint2", "joint3"], position=[0.0, 0.0, 0.0])
    goal = JointState(name=["joint1", "joint2", "joint3"], position=[1.0, 0.5, 0.25])

    result = planner.plan_joint_path(world, robot_id, start, goal, timeout=0.25)

    assert result.status == PlanningStatus.SUCCESS
    assert [point.position for point in result.path] == [[0.0, 0.0, 0.0], [1.0, 0.5, 0.25]]
    assert vamp_module.configure_calls == [("panda", "prm", 250)]
    assert vamp_module.planner_calls == [
        ([0.0, 0.0, 0.0], [1.0, 0.5, 0.25], FakeSampler("fake_sampler"))
    ]
    assert len(robot_module.simplify_calls) == 1
    assert robot_module.motion_calls == [([0.0, 0.0, 0.0], [1.0, 0.5, 0.25], True)]


def test_vamp_planner_reports_native_planning_failure(fake_vamp_modules) -> None:
    """Unsolved VAMP results map to a failed DimOS planning result."""
    vamp_module, _ = fake_vamp_modules
    vamp_module.planner_solved = False
    world = finalized_vamp_world()
    robot_id = world.get_robot_ids()[0]
    planner = VampPlanner(VampPlannerConfig(simplify=False, validate_path=False))

    result = planner.plan_joint_path(
        world,
        robot_id,
        JointState(name=["joint1", "joint2", "joint3"], position=[0.0, 0.0, 0.0]),
        JointState(name=["joint1", "joint2", "joint3"], position=[1.0, 0.5, 0.25]),
    )

    assert result.status == PlanningStatus.NO_SOLUTION
    assert "did not find a path" in result.message
