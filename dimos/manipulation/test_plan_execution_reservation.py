# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");

"""Module-side tests for runtime-owned plan submission and lifecycle handling."""

from collections.abc import Iterator
from inspect import signature
from pathlib import Path
from unittest.mock import MagicMock

from pydantic import ValidationError
import pytest

from dimos.manipulation._test_manipulation_helpers import (
    FakeCoordinatorGateway,
    close_test_runtimes,
    install_ready_plan,
    install_runtime,
    make_module,
)
from dimos.manipulation.execution_runtime import (
    ExecutionRuntime,
    LifecycleState,
    Outcome,
    prepare_generated_plan,
)
from dimos.manipulation.manipulation_module import (
    ManipulationExecutionSnapshot,
    ManipulationModule,
    ManipulationModuleConfig,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


def test_physical_operation_timeout_config_defaults_to_sixty_and_rejects_invalid_values() -> None:
    assert ManipulationModuleConfig().physical_operation_timeout == 60.0
    for value in (0.0, -1.0, float("inf"), float("nan")):
        with pytest.raises(ValidationError):
            ManipulationModuleConfig(physical_operation_timeout=value)


def test_module_runtime_factory_receives_distinct_action_and_physical_timeouts(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = ManipulationModule(
        robots=[_config()],
        planning_timeout=3.0,
        physical_operation_timeout=17.0,
        visualization={"backend": "none"},
    )
    module.coordinator_joint_state = None
    captured: dict[str, float] = {}
    runtimes: list[ExecutionRuntime] = []

    class WorldMonitor:
        visualization = None

        def add_robot(self, config: RobotModelConfig) -> str:
            del config
            return "robot-id"

        def finalize(self) -> None: ...

        def start_state_monitor(self, robot_id: str) -> None:
            del robot_id

        def set_visualization(self, visualization: object) -> None:
            del visualization

        def initialize_visualization(self, operator: object) -> None:
            del operator

        def stop_all_monitors(self) -> None: ...

    world_monitor = WorldMonitor()
    specs = type(
        "PlanningSpecs",
        (),
        {"world_monitor": world_monitor, "planner": object(), "kinematics": object()},
    )()

    def runtime_factory(_gateway_factory, **kwargs):
        captured.update(kwargs)
        runtime = ExecutionRuntime(lambda: FakeCoordinatorGateway(), **kwargs)
        runtimes.append(runtime)
        return runtime

    monkeypatch.setattr("dimos.manipulation.manipulation_module.create_world", lambda **_: object())
    monkeypatch.setattr(
        "dimos.manipulation.manipulation_module.create_planning_specs",
        lambda **_: specs,
    )
    monkeypatch.setattr(
        "dimos.manipulation.manipulation_module.create_manipulation_visualization",
        lambda *_, **__: None,
    )
    monkeypatch.setattr(ManipulationModule, "_runtime_factory", staticmethod(runtime_factory))

    try:
        module.start()
        assert captured["action_timeout"] == 3.0
        assert captured["physical_operation_timeout"] == 17.0
    finally:
        module.stop()


def test_public_move_to_joints_reports_timeout_fault_without_extra_cancel(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    gateway = FakeCoordinatorGateway(
        status_outcome=Outcome.RUNNING,
        status_outcomes=[Outcome.RUNNING],
        cancel_outcome=Outcome.UNKNOWN,
    )
    module, _ = _module(physical_operation_timeout=0.02, gateway=gateway)
    plan = _plan()

    def plan_to_joints(_goal, _robot_name=None) -> bool:
        install_ready_plan(module, plan)
        return True

    monkeypatch.setattr(module, "plan_to_joints", plan_to_joints)
    monkeypatch.setattr(module, "preview_path", lambda *args, **kwargs: True)

    result = module.move_to_joints("1.0")

    assert not result.is_success()
    assert result.error_code == "EXECUTION_FAILED"
    assert "uncertain" in result.message.lower()
    assert module.get_state() == LifecycleState.FAULT.name
    assert module.get_error() == result.message
    snapshot = module.get_execution_snapshot()
    assert snapshot.state == LifecycleState.FAULT.name
    assert snapshot.diagnostic == result.message
    assert gateway.cancel_calls == ["traj_arm"]

    module.get_state()
    assert gateway.cancel_calls == ["traj_arm"]


def _config(task_name: str | None = "traj_arm") -> RobotModelConfig:
    return RobotModelConfig(
        name="arm",
        model_path=Path("/path/to/robot.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion()),
        joint_names=["j0"],
        base_link="base",
        planning_groups=[
            PlanningGroupDefinition(
                name="manipulator", joint_names=("j0",), base_link="base", tip_link="tool"
            )
        ],
        max_velocity=1.0,
        max_acceleration=1.0,
        coordinator_task_name=task_name,
    )


def _plan(group_id: str = "arm/manipulator") -> GeneratedPlan:
    return GeneratedPlan(
        group_ids=(group_id,),
        path=[
            JointState(name=["arm/j0"], position=[0.0]),
            JointState(name=["arm/j0"], position=[1.0]),
        ],
        trajectory=JointTrajectory(
            joint_names=["arm/j0"],
            points=[
                TrajectoryPoint(time_from_start=0.0, positions=[0.0], velocities=[0.0]),
                TrajectoryPoint(time_from_start=1.0, positions=[1.0], velocities=[0.0]),
            ],
        ),
    )


def _module(
    task_name: str | None = "traj_arm",
    physical_operation_timeout: float = 60.0,
    gateway: FakeCoordinatorGateway | None = None,
) -> tuple[ManipulationModule, FakeCoordinatorGateway]:
    module = make_module()
    module.config.physical_operation_timeout = physical_operation_timeout
    config = _config(task_name)
    module._robots = {"arm": ("robot_id", config, object())}
    module._world_monitor = MagicMock()
    module._world_monitor.planning_groups = PlanningGroupRegistry([config])
    return module, install_runtime(module, [config], gateway)


@pytest.fixture(autouse=True)
def _close_runtime() -> Iterator[None]:
    yield
    close_test_runtimes()


def test_execution_snapshot_is_atomic_and_reports_ready_identity() -> None:
    module, _gateway = _module()
    runtime = module._execution_runtime
    original_snapshot = runtime.snapshot
    runtime.snapshot = MagicMock(wraps=original_snapshot)

    view = module.get_execution_snapshot()
    assert isinstance(view, ManipulationExecutionSnapshot)
    assert view.state == LifecycleState.IDLE.name
    assert view.diagnostic == ""
    assert view.ready_plan_id is None
    assert not view.has_ready_plan
    runtime.snapshot.assert_called_once_with()

    runtime.snapshot.reset_mock()
    token = runtime.start_planning()
    prepared = prepare_generated_plan(_plan(), module._execution_topology)
    assert runtime.complete_planning(token, prepared).accepted
    runtime.snapshot.reset_mock()
    view = module.get_execution_snapshot()
    assert view.state == LifecycleState.READY.name
    assert view.ready_plan_id is not None
    assert view.has_ready_plan
    runtime.snapshot.assert_called_once_with()


def test_execution_snapshot_is_safe_before_runtime_initialization() -> None:
    view = make_module().get_execution_snapshot()
    assert view == ManipulationExecutionSnapshot("IDLE", "", None, False)


def test_execution_api_has_no_robot_name_and_uses_exact_ready_identity() -> None:
    module, gateway = _module()
    plan = _plan()
    token = module._execution_runtime.start_planning()
    prepared = prepare_generated_plan(plan, module._execution_topology)
    assert module._execution_runtime.complete_planning(token, prepared).accepted

    assert "robot_name" not in signature(module.execute).parameters
    assert "robot_name" not in signature(module.execute_plan).parameters
    assert module.execute_plan(plan=plan) is True
    assert len(gateway.execute_calls) == 1
    assert module.get_state() in (LifecycleState.RUNNING.name, LifecycleState.IDLE.name)
    assert module.cancel() is True


def test_trajectory_status_uses_operation_handle_identity() -> None:
    module, _gateway = _module()
    token = module._execution_runtime.start_planning()
    prepared = prepare_generated_plan(_plan(), module._execution_topology)
    assert module._execution_runtime.complete_planning(token, prepared).accepted

    result = module._execution_runtime.execute_ready()
    assert result.accepted and result.value is not None
    operation = module._execution_runtime.snapshot().operation
    assert operation is not None
    status = module.get_trajectory_status()
    assert status is not None
    assert status["operation_id"] == operation.handle.operation_id
    assert module.cancel() is True


def test_execute_plan_explicit_plan_prepares_without_using_other_ready_plan() -> None:
    module, gateway = _module()
    ready = _plan()
    explicit = _plan()
    token = module._execution_runtime.start_planning()
    prepared = prepare_generated_plan(ready, module._execution_topology)
    assert module._execution_runtime.complete_planning(token, prepared).accepted

    assert module.execute_plan(plan=explicit) is True
    assert len(gateway.execute_calls) == 1
    assert module.cancel() is True


def test_plan_without_coordinator_task_fails_during_preparation_without_dispatch() -> None:
    module, gateway = _module(task_name=None)
    assert module.execute_plan(plan=_plan()) is False
    assert gateway.execute_calls == []
    assert module.get_state() == LifecycleState.IDLE.name


def test_cancel_ready_clears_plan_and_status_is_snapshot_only() -> None:
    module, _gateway = _module()
    plan = _plan()
    token = module._execution_runtime.start_planning()
    prepared = prepare_generated_plan(plan, module._execution_topology)
    assert module._execution_runtime.complete_planning(token, prepared).accepted
    assert module.get_trajectory_status() is not None
    assert module.cancel() is True
    assert module._execution_runtime.snapshot().ready_plan is None


def test_reset_fault_and_runtime_shutdown_are_owned_by_runtime() -> None:
    module, gateway = _module()
    gateway.execute_outcome = Outcome.REJECTED
    assert module.execute_plan(plan=_plan()) is False
    assert module.get_state() == LifecycleState.IDLE.name
    assert module.reset().is_success()
    assert module.get_state() == LifecycleState.IDLE.name
    module.stop()
