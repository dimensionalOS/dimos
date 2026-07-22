"""Topology and generated-plan materialization tests."""

from pathlib import Path

import pytest

from dimos.manipulation.execution_models import Outcome
from dimos.manipulation.execution_runtime import ExecutionRuntime, ExecutionTopology
from dimos.manipulation.execution_topology import (
    ExecutionPlan,
    PreparedPlan,
    prepare_generated_plan,
)
from dimos.manipulation.planning.groups.models import PlanningGroupDefinition
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


class Gateway:
    def __init__(self, outcomes: dict[str, Outcome]) -> None:
        self.outcomes = outcomes

    def execute(self, task_name: str, request: object) -> Outcome:
        return self.outcomes.get(task_name, Outcome.ACCEPTED)

    def cancel(self, task_name: str) -> Outcome:
        return Outcome.CANCELLED

    def status(self, task_name: str) -> Outcome:
        return Outcome.INACTIVE

    def reset(self, task_name: str) -> Outcome:
        return Outcome.INACTIVE

    def set_gripper_position(self, hardware_id: str, position: float) -> Outcome:
        return Outcome.ACCEPTED

    def get_gripper_position(self, hardware_id: str) -> float:
        return 0.0

    def stop(self) -> None:
        return None


def _prep_robot(
    name: str,
    joints: tuple[str, ...],
    group: str,
    task: str | None,
    mapping: dict[str, str] | None = None,
) -> RobotModelConfig:
    return RobotModelConfig(
        name=name,
        model_path=Path(f"/{name}.urdf"),
        base_pose=PoseStamped(position=Vector3(), orientation=Quaternion([0.0, 0.0, 0.0, 1.0])),
        joint_names=list(joints),
        planning_groups=[PlanningGroupDefinition(name=group, joint_names=joints, base_link="base")],
        coordinator_task_name=task,
        joint_name_mapping=mapping or {},
    )


def _generated(names: list[str], groups: tuple[str, ...], width: int) -> GeneratedPlan:
    return GeneratedPlan(
        group_ids=groups,
        trajectory=JointTrajectory(
            joint_names=names,
            points=[
                TrajectoryPoint(
                    time_from_start=0.0, positions=[0.0] * width, velocities=[0.1] * width
                ),
                TrajectoryPoint(
                    time_from_start=1.5, positions=[1.0] * width, velocities=[0.2] * width
                ),
            ],
        ),
    )


def test_prepare_generated_plan_uses_ordered_groups_and_ignores_extraneous_robot_columns() -> None:
    left = _prep_robot("left", ("l0", "l1"), "arm", "left_task", {"left_l0": "l0"})
    right = _prep_robot("right", ("r0",), "arm", "right_task")
    extra = _prep_robot("extra", ("x0",), "arm", "extra_task")
    topology = ExecutionTopology.from_robot_configs((left, right, extra))
    generated = _generated(
        ["extra/x0", "right/r0", "left/l1", "left/l0"], ("left/arm", "right/arm"), 4
    )

    prepared = prepare_generated_plan(generated, topology)
    assert prepared.generated_plan is generated
    assert [entry.robot_name for entry in prepared.entries] == ["left", "right"]
    assert [entry.task_name for entry in prepared.entries] == ["left_task", "right_task"]
    assert prepared.entries[0].request["trajectory"].joint_names == ["l1", "left_l0"]
    assert prepared.entries[0].request["trajectory"].points[1].time_from_start == 1.5
    assert prepared.entries[1].request["trajectory"].joint_names == ["r0"]


def test_prepare_rejects_missing_routes_tasks_malformed_and_missing_or_duplicate_joints() -> None:
    left = _prep_robot("left", ("l0", "l1"), "arm", "left_task")
    right = _prep_robot("right", ("r0",), "arm", "right_task")
    topology = ExecutionTopology.from_robot_configs((left, right))
    with pytest.raises(ValueError, match="route set"):
        ExecutionTopology.from_robot_configs((left, right), {"left/arm": ("left", "left_task")})
    with pytest.raises(ValueError, match="malformed"):
        prepare_generated_plan(_generated(["left/l0", "bad"], ("left/arm",), 2), topology)
    with pytest.raises(ValueError, match="missing robot joints"):
        prepare_generated_plan(_generated(["left/l0"], ("left/arm",), 1), topology)
    with pytest.raises(ValueError, match="duplicate global"):
        prepare_generated_plan(_generated(["left/l0", "left/l0"], ("left/arm",), 2), topology)


def test_prepared_validation_selects_only_generated_groups_from_larger_topology() -> None:
    left = _prep_robot("left", ("l0",), "arm", "left_task")
    right = _prep_robot("right", ("r0",), "arm", "right_task")
    extra = _prep_robot("extra", ("x0",), "arm", "extra_task")
    topology = ExecutionTopology.from_robot_configs((left, right, extra))
    prepared = prepare_generated_plan(
        _generated(["left/l0", "right/r0"], ("right/arm", "left/arm"), 2), topology
    )
    runtime = ExecutionRuntime(
        lambda: Gateway({"left": Outcome.ACCEPTED, "right": Outcome.ACCEPTED}),
        poll_interval=10,
    )
    try:
        result = runtime.execute_explicit(prepared)
        assert result.accepted
        assert [entry.planning_group for entry in prepared.entries] == ["right/arm", "left/arm"]
    finally:
        runtime.shutdown()


def test_prepare_rejects_additional_selected_robot_joint_column() -> None:
    robot = _prep_robot("arm", ("j0", "j1"), "manipulator", "arm_task")
    topology = ExecutionTopology.from_robot_configs((robot,))
    with pytest.raises(ValueError, match="additional joint column"):
        prepare_generated_plan(
            _generated(["arm/j0", "arm/j1", "arm/extra"], ("arm/manipulator",), 3), topology
        )


def test_topology_rejects_duplicate_routes_bad_mapping_targets_and_unknown_group_joints() -> None:
    robot = _prep_robot("arm", ("j0",), "manipulator", "arm_task")
    with pytest.raises(ValueError, match="duplicate group route"):
        ExecutionTopology.from_robot_configs(
            (robot,),
            [("arm/manipulator", "arm", "arm_task"), ("arm/manipulator", "arm", "arm_task")],
        )
    with pytest.raises(ValueError, match="mapped local joint"):
        ExecutionTopology.from_robot_configs(
            (_prep_robot("arm", ("j0",), "manipulator", "arm_task", {"coord": "missing"}),)
        )
    bad_group = _prep_robot("arm", ("j0",), "manipulator", "arm_task")
    bad_group.planning_groups[0] = PlanningGroupDefinition(
        name="manipulator", joint_names=("missing",), base_link="base"
    )
    with pytest.raises(ValueError, match="planning group joints"):
        ExecutionTopology.from_robot_configs((bad_group,))


def test_planning_only_config_has_no_route_and_generated_execution_rejects_it() -> None:
    planning_only = _prep_robot("planner", ("j0",), "manipulator", None)
    topology = ExecutionTopology.from_robot_configs((planning_only,))
    assert topology.routes == ()
    with pytest.raises(ValueError, match="no coordinator task"):
        prepare_generated_plan(_generated(["planner/j0"], ("planner/manipulator",), 1), topology)


def test_prepared_plan_is_real_type_and_partial_generated_entries_are_not_executable() -> None:
    robot = _prep_robot("arm", ("j0",), "manipulator", "arm_task", {"coord_j0": "j0"})
    topology = ExecutionTopology.from_robot_configs((robot,))
    generated = _generated(["arm/j0"], ("arm/manipulator",), 1)
    prepared = prepare_generated_plan(generated, topology)
    assert isinstance(prepared, PreparedPlan)
    assert prepared is not generated
    partial = PreparedPlan(generated, (), topology)
    runtime = ExecutionRuntime(lambda: Gateway({"arm": Outcome.ACCEPTED}), poll_interval=10)
    try:
        assert not runtime.execute_explicit(partial).accepted
    finally:
        runtime.shutdown()


def test_real_generated_plan_cannot_use_legacy_synthetic_execution_plan() -> None:
    robot = _prep_robot("arm", ("j0",), "manipulator", "arm_task")
    topology = ExecutionTopology.from_robot_configs((robot,))
    generated = _generated(["arm/j0"], ("arm/manipulator",), 1)
    prepared = prepare_generated_plan(generated, topology)
    runtime = ExecutionRuntime(lambda: Gateway({"arm": Outcome.ACCEPTED}), poll_interval=10)
    try:
        legacy = ExecutionPlan(generated, prepared.entries)
        assert not runtime.execute_explicit(legacy).accepted
        assert runtime.execute_explicit(prepared).accepted
    finally:
        runtime.shutdown()
