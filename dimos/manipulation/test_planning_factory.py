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

"""Focused tests for manipulation planning wiring."""

from __future__ import annotations

from collections.abc import Callable, Generator
from pathlib import Path
import sys
from types import ModuleType
from typing import Any
from unittest.mock import ANY

from pydantic import ValidationError
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.manipulation_module import ManipulationModule, ManipulationModuleConfig
from dimos.manipulation.planning.factory import (
    create_kinematics,
    create_planner,
    create_planning_specs,
    create_planning_stack,
    create_world,
    validate_backend_combination,
)
from dimos.manipulation.planning.groups.models import (
    PlanningGroup,
    PlanningGroupDefinition,
    PlanningGroupSelection,
)
from dimos.manipulation.planning.kinematics.config import (
    JacobianKinematicsConfig,
    PinkKinematicsConfig,
)
from dimos.manipulation.planning.kinematics.jacobian_ik import JacobianIK
from dimos.manipulation.planning.kinematics.pink_ik import PinkIK
from dimos.manipulation.planning.planners.config import CuRoboPlannerConfig, RRTConnectPlannerConfig
from dimos.manipulation.planning.planners.roboplan_config import RoboPlanPlannerConfig
from dimos.manipulation.planning.planners.rrt_planner import RRTConnectPlanner
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import PlanningResult
from dimos.manipulation.planning.trajectory_generator.config import (
    SimpleTrapezoidParametrizationConfig,
    TrajectoryParametrizationConfig,
)
from dimos.manipulation.planning.trajectory_generator.simple_parametrizer import (
    SimpleTrapezoidParametrizer,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel


@pytest.fixture
def make_module() -> Generator[Callable[..., ManipulationModule], None, None]:
    """Build ManipulationModules and stop them on teardown, even on failure."""
    modules: list[ManipulationModule] = []

    def _make(**kwargs: Any) -> ManipulationModule:
        module = ManipulationModule(**kwargs)
        modules.append(module)
        return module

    yield _make
    for module in modules:
        module.stop()


@pytest.fixture
def robot_config() -> RobotModelConfig:
    return RobotModelConfig(
        model=RobotModel.from_file(Path("/path/to/robot.urdf")),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),  # type: ignore[call-arg]
        joint_names=["joint1", "joint2"],
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator",
                joint_names=("joint1", "joint2"),
                base_link="base_link",
                tip_link="tcp",
            )
        ],
    )


def test_create_world_unknown_backend() -> None:
    with pytest.raises(
        ValueError, match=r"Unknown backend: fake\. Available: \['drake', 'roboplan', 'curobo'\]"
    ):
        create_world(backend="fake")


def test_factory_selects_expected_implementations() -> None:
    assert isinstance(create_planner(config=RRTConnectPlannerConfig()), RRTConnectPlanner)
    assert isinstance(create_kinematics(name="jacobian"), JacobianIK)


def test_rrt_planner_backend_does_not_import_roboplan(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    for module_name in list(sys.modules):
        if module_name == "roboplan" or module_name.startswith("roboplan."):
            monkeypatch.delitem(sys.modules, module_name, raising=False)

    create_planner(config=RRTConnectPlannerConfig())
    validate_backend_combination(
        world_backend="drake",
        planner_backend="rrt_connect",
        kinematics_name="pink",
    )

    assert "roboplan.core" not in sys.modules
    assert "roboplan.rrt" not in sys.modules


def test_validate_backend_combination_rejects_invalid_combinations() -> None:
    with pytest.raises(
        ValueError, match='planner.backend="roboplan" requires world_backend="roboplan"'
    ):
        validate_backend_combination(world_backend="drake", planner_backend="roboplan")

    with pytest.raises(
        ValueError, match='kinematics_name="drake_optimization" requires world_backend="drake"'
    ):
        validate_backend_combination(world_backend="roboplan", kinematics_name="drake_optimization")

    with pytest.raises(
        ValueError,
        match='trajectory_parametrization.backend="roboplan_toppra" requires',
    ):
        validate_backend_combination(
            world_backend="drake",
            planner_backend="rrt_connect",
            trajectory_parametrization_backend="roboplan_toppra",
        )


@pytest.mark.parametrize(
    ("world_backend", "planner_backend", "kinematics", "parametrization", "message"),
    [
        ("drake", "curobo", "pink", None, 'planner.backend="curobo" requires'),
        ("roboplan", "curobo", "pink", None, 'planner.backend="curobo" requires'),
        ("curobo", "roboplan", "pink", None, 'world_backend="curobo" requires planner'),
        ("curobo", "rrt_connect", "pink", None, 'world_backend="curobo" requires planner'),
        ("curobo", "curobo", "jacobian", None, 'requires kinematics_name="pink"'),
        ("curobo", "curobo", "drake_optimization", None, 'requires kinematics_name="pink"'),
        ("curobo", "curobo", "pink", "roboplan_toppra", 'requires world_backend="roboplan"'),
    ],
)
def test_validate_curobo_backend_combinations(
    world_backend: str,
    planner_backend: str,
    kinematics: str,
    parametrization: str | None,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        validate_backend_combination(
            world_backend=world_backend,
            planner_backend=planner_backend,
            kinematics_name=kinematics,
            trajectory_parametrization_backend=parametrization,
        )


def test_curobo_factory_wires_typed_config_and_preserves_native_timing(
    mocker: MockerFixture,
    monkeypatch: pytest.MonkeyPatch,
    robot_config: RobotModelConfig,
) -> None:
    world = mocker.MagicMock(name="world")
    planner = mocker.MagicMock(name="planner")
    world_type = mocker.Mock(return_value=world)
    planner_type = mocker.Mock(return_value=planner)
    world_module = ModuleType("dimos.manipulation.planning.world.curobo_world")
    planner_module = ModuleType("dimos.manipulation.planning.planners.curobo_planner")
    world_module.CuRoboWorld = world_type  # type: ignore[attr-defined]
    planner_module.CuRoboPlanner = planner_type  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, world_module.__name__, world_module)
    monkeypatch.setitem(sys.modules, planner_module.__name__, planner_module)
    monkeypatch.setitem(sys.modules, "curobo", None)

    config = ManipulationModuleConfig.model_validate(
        {"model": robot_config, "world_backend": "curobo", "planner": {"backend": "curobo"}}
    )
    assert isinstance(config.planner, CuRoboPlannerConfig)
    assert create_world(backend=config.world_backend, visualization=config.visualization) is world
    world_type.assert_called_once_with(enable_viz=False)
    specs = create_planning_specs(
        world=world,
        world_backend=config.world_backend,
        planner=config.planner,
        kinematics=config.kinematics,
        trajectory_parametrization=config.trajectory_parametrization,
    )
    assert specs.planner is planner
    planner_type.assert_called_once_with(world, config.planner)
    assert isinstance(specs.kinematics, PinkIK)
    assert isinstance(specs.trajectory_parametrizer, SimpleTrapezoidParametrizer)
    retime = mocker.spy(specs.trajectory_parametrizer, "_parametrize_path")
    selection = PlanningGroupSelection.from_groups(
        (PlanningGroup(id="arm", joint_names=("joint1",), base_link="base_link"),)
    )
    result = PlanningResult(
        status=PlanningStatus.SUCCESS,
        path=[
            JointState(name=["joint1"], position=[0.0], velocity=[0.0]),
            JointState(name=["joint1"], position=[0.2], velocity=[0.3]),
            JointState(name=["joint1"], position=[0.4], velocity=[0.0]),
        ],
        timestamps=[0.0, 0.4, 1.2],
    )
    plan = specs.trajectory_parametrizer.materialize_plan(world, selection, result, speed_scale=0.5)
    assert [point.time_from_start for point in plan.trajectory.points] == result.timestamps
    assert [point.velocities for point in plan.trajectory.points] == [[0.0], [0.3], [0.0]]
    retime.assert_not_called()


def test_invalid_curobo_selection_does_not_import_native_backends(
    monkeypatch: pytest.MonkeyPatch, robot_config: RobotModelConfig
) -> None:
    monkeypatch.setitem(sys.modules, "dimos.manipulation.planning.world.curobo_world", None)
    monkeypatch.setitem(sys.modules, "dimos.manipulation.planning.planners.curobo_planner", None)
    with pytest.raises(ValueError, match='world_backend="curobo" requires planner'):
        create_planning_stack(robot_config, world_backend="curobo")
    with pytest.raises(ValueError, match='planner.backend="curobo" requires'):
        create_planner(config=CuRoboPlannerConfig(), world_backend="curobo")


@pytest.mark.parametrize(
    ("world_backend", "planner", "configured", "expected_backend"),
    [
        ("roboplan", RoboPlanPlannerConfig(), None, "roboplan_toppra"),
        ("drake", RRTConnectPlannerConfig(), None, "simple_trapezoid"),
        (
            "roboplan",
            RoboPlanPlannerConfig(),
            SimpleTrapezoidParametrizationConfig(),
            "simple_trapezoid",
        ),
    ],
)
def test_create_planning_specs_selects_world_default_unless_overridden(
    mocker: MockerFixture,
    world_backend: str,
    planner: RoboPlanPlannerConfig | RRTConnectPlannerConfig,
    configured: TrajectoryParametrizationConfig | None,
    expected_backend: str,
) -> None:
    world = mocker.MagicMock()
    trajectory_parametrizer = mocker.MagicMock()
    mocker.patch(
        "dimos.manipulation.planning.factory.create_kinematics",
        return_value=mocker.MagicMock(),
    )
    mocker.patch(
        "dimos.manipulation.planning.factory.create_planner",
        return_value=mocker.MagicMock(),
    )
    create_parametrizer = mocker.patch(
        "dimos.manipulation.planning.factory.create_trajectory_parametrizer",
        return_value=trajectory_parametrizer,
    )

    result = create_planning_specs(
        world=world,
        world_backend=world_backend,
        planner=planner,
        trajectory_parametrization=configured,
    )

    selected = create_parametrizer.call_args.args[0]
    assert selected.backend == expected_backend
    create_parametrizer.assert_called_once_with(selected, world_backend=world_backend)
    assert result.trajectory_parametrizer is trajectory_parametrizer


def test_create_planner_binds_distinct_roboplan_planner_to_world(
    mocker: MockerFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    world = mocker.MagicMock()
    planner = mocker.MagicMock()
    planner_type = mocker.MagicMock(return_value=planner)
    roboplan_planner_module = ModuleType("dimos.manipulation.planning.planners.roboplan_planner")
    roboplan_planner_module.RoboPlanPlanner = planner_type  # type: ignore[attr-defined]
    monkeypatch.setitem(
        sys.modules,
        "dimos.manipulation.planning.planners.roboplan_planner",
        roboplan_planner_module,
    )
    config = RoboPlanPlannerConfig()

    result = create_planner(
        config=config,
        world=world,
        world_backend="roboplan",
    )

    assert result is planner
    assert result is not world
    planner_type.assert_called_once_with(world, config)


def test_create_planner_rejects_non_roboplan_world(
    mocker: MockerFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class RejectingRoboPlanPlanner:
        def __init__(self, world: Any, config: RoboPlanPlannerConfig) -> None:
            raise TypeError("RoboPlanPlanner requires a RoboPlanWorld")

    roboplan_planner_module = ModuleType("dimos.manipulation.planning.planners.roboplan_planner")
    roboplan_planner_module.RoboPlanPlanner = RejectingRoboPlanPlanner  # type: ignore[attr-defined]
    monkeypatch.setitem(
        sys.modules,
        "dimos.manipulation.planning.planners.roboplan_planner",
        roboplan_planner_module,
    )

    with pytest.raises(TypeError, match="requires a RoboPlanWorld"):
        create_planner(
            config=RoboPlanPlannerConfig(),
            world=mocker.MagicMock(),
            world_backend="roboplan",
        )


def test_create_planner_rejects_roboplan_without_importing_optional_backend(
    mocker: MockerFixture,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delitem(
        sys.modules,
        "dimos.manipulation.planning.world.roboplan_world",
        raising=False,
    )
    monkeypatch.setitem(sys.modules, "roboplan", None)

    with pytest.raises(
        ValueError, match='planner.backend="roboplan" requires world_backend="roboplan"'
    ):
        create_planner(
            config=RoboPlanPlannerConfig(),
            world=mocker.MagicMock(),
            world_backend="drake",
        )


def test_create_planning_stack_defaults_to_roboplan(
    mocker: MockerFixture, monkeypatch: pytest.MonkeyPatch, robot_config: RobotModelConfig
) -> None:
    monkeypatch.setitem(sys.modules, "dimos.manipulation.planning.world.curobo_world", None)
    monkeypatch.setitem(sys.modules, "dimos.manipulation.planning.planners.curobo_planner", None)
    config = ManipulationModuleConfig(model=robot_config)
    assert config.world_backend == "roboplan"
    assert isinstance(config.planner, RoboPlanPlannerConfig)
    world = mocker.MagicMock()

    kinematics = mocker.MagicMock(name="kinematics")
    planner = mocker.MagicMock(name="planner")

    mock_world = mocker.patch(
        "dimos.manipulation.planning.factory.create_world", return_value=world
    )
    mock_kinematics = mocker.patch(
        "dimos.manipulation.planning.factory.create_kinematics",
        return_value=kinematics,
    )
    mock_planner = mocker.patch(
        "dimos.manipulation.planning.factory.create_planner",
        return_value=planner,
    )
    mocker.patch(
        "dimos.manipulation.planning.factory.create_trajectory_parametrizer",
        return_value=mocker.MagicMock(name="trajectory_parametrizer"),
    )
    prepared = object()
    mock_prepare = mocker.patch(
        "dimos.manipulation.planning.factory.prepare_robot_model",
        return_value=prepared,
    )

    result = create_planning_stack(robot_config)

    assert result == (world, kinematics, planner)
    mock_world.assert_called_once_with(backend="roboplan", visualization=None)
    mock_kinematics.assert_called_once_with(config=PinkKinematicsConfig())
    mock_planner.assert_called_once_with(
        config=RoboPlanPlannerConfig(),
        world=world,
        world_backend="roboplan",
    )
    mock_prepare.assert_called_once_with(robot_config)
    world.load_model.assert_called_once_with(prepared)
    world.finalize.assert_called_once()


def test_configuration_requires_one_model() -> None:
    with pytest.raises(ValidationError, match="model"):
        ManipulationModuleConfig()


def test_start_uses_configured_planner_and_kinematics(
    mocker: MockerFixture,
    robot_config: RobotModelConfig,
    make_module: Callable[..., ManipulationModule],
) -> None:
    planner_config = RRTConnectPlannerConfig()
    module = make_module(
        model=robot_config,
        planner=planner_config,
        kinematics=JacobianKinematicsConfig(),
    )
    world = mocker.MagicMock(name="world")
    world_monitor = mocker.MagicMock()
    planner = mocker.MagicMock(name="planner")
    kinematics = mocker.MagicMock(name="kinematics")
    planning_specs = mocker.MagicMock(
        world_monitor=world_monitor,
        planner=planner,
        kinematics=kinematics,
        trajectory_parametrizer=mocker.MagicMock(name="trajectory_parametrizer"),
    )
    create_world_mock = mocker.patch(
        "dimos.manipulation.manipulation_module.create_world", return_value=world
    )
    create_planning_specs_mock = mocker.patch(
        "dimos.manipulation.manipulation_module.create_planning_specs",
        return_value=planning_specs,
    )
    module._initialize_planning()

    create_world_mock.assert_called_once_with(
        backend="roboplan", visualization=module.config.visualization
    )
    create_planning_specs_mock.assert_called_once_with(
        world=world,
        world_backend="roboplan",
        planner=planner_config,
        kinematics=module.config.kinematics,
        trajectory_parametrization=ANY,
    )
    assert module._planner is planner
    assert module._kinematics is kinematics
    world_monitor.load_model.assert_called_once_with(robot_config)
