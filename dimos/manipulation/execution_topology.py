# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0 (the "License").
"""Pure topology binding and generated-plan materialization helpers."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.models import GeneratedPlan
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint


@dataclass(frozen=True)
class ExecutionTopology:
    """Canonical group -> robot -> coordinator-task routing."""

    routes: tuple[tuple[str, str, str], ...]
    robot_configs: tuple[RobotModelConfig, ...] = ()
    inverse_joint_mappings: tuple[tuple[str, tuple[tuple[str, str], ...]], ...] = ()

    @classmethod
    def from_robot_configs(
        cls,
        robot_configs: Iterable[RobotModelConfig],
        group_routes: Mapping[str, tuple[str, str]] | Iterable[tuple[str, str, str]] | None = None,
        group_resolver: Callable[[str], tuple[str, str]] | None = None,
    ) -> ExecutionTopology:
        """Build canonical routes while retaining the supplied config references."""
        configs = tuple(robot_configs)
        by_robot: dict[str, RobotModelConfig] = {}
        for config in configs:
            if config.name in by_robot:
                raise ValueError(f"duplicate robot config: {config.name}")
            by_robot[config.name] = config
        explicit: dict[str, tuple[str, str]] = {}
        if isinstance(group_routes, Mapping):
            for group, route in group_routes.items():
                if not isinstance(group, str) or not isinstance(route, tuple) or len(route) != 2:
                    raise ValueError("invalid group route mapping")
                explicit[group] = (str(route[0]), str(route[1]))
        elif group_routes is not None:
            for group, robot, task in group_routes:
                if group in explicit:
                    raise ValueError(f"duplicate group route: {group}")
                explicit[group] = (robot, task)
        routes: list[tuple[str, str, str]] = []
        inverse: list[tuple[str, tuple[tuple[str, str], ...]]] = []
        for config in configs:
            local_names = set(config.joint_names)
            local_to_coordinator: dict[str, str] = {}
            for coordinator_name, local_name in config.joint_name_mapping.items():
                if local_name not in local_names:
                    raise ValueError(
                        f"mapped local joint is not configured: {config.name}/{local_name}"
                    )
                previous = local_to_coordinator.get(local_name)
                if previous is not None and previous != coordinator_name:
                    raise ValueError(
                        f"duplicate local joint mapping for {config.name}/{local_name}"
                    )
                local_to_coordinator[local_name] = coordinator_name
            coordinator_names: set[str] = set()
            for local_name in config.joint_names:
                coordinator_name = local_to_coordinator.get(local_name, local_name)
                if coordinator_name in coordinator_names:
                    raise ValueError(
                        f"duplicate coordinator joint mapping for {config.name}/{coordinator_name}"
                    )
                coordinator_names.add(coordinator_name)
            inverse.append(
                (
                    config.name,
                    tuple(
                        (local, local_to_coordinator.get(local, local))
                        for local in config.joint_names
                    ),
                )
            )
            for definition in config.planning_groups:
                if not set(definition.joint_names).issubset(local_names):
                    raise ValueError(
                        f"planning group joints are not configured: {config.name}/{definition.name}"
                    )
                group_id = f"{config.name}/{definition.name}"
                if group_resolver is not None:
                    route_robot, route_task = group_resolver(group_id)
                else:
                    if not config.coordinator_task_name and group_id not in explicit:
                        continue
                    route_robot, route_task = explicit.get(
                        group_id, (config.name, config.coordinator_task_name or "")
                    )
                if route_robot != config.name or route_task != config.coordinator_task_name:
                    raise ValueError(f"route/config mismatch for {group_id}")
                routes.append((group_id, route_robot, route_task))
        if explicit and set(explicit) != {group for group, _, _ in routes}:
            raise ValueError("route set does not match configured planning groups")
        return cls(tuple(routes), configs, tuple(inverse))

    from_configs = from_robot_configs
    from_robot_model_configs = from_robot_configs

    def config_for_robot(self, robot_name: str) -> RobotModelConfig:
        for config in self.robot_configs:
            if config.name == robot_name:
                return config
        raise KeyError(f"unknown robot: {robot_name}")

    def coordinator_joint_name(self, robot_name: str, local_name: str) -> str:
        for name, mappings in self.inverse_joint_mappings:
            if name == robot_name:
                for local, coordinator in mappings:
                    if local == local_name:
                        return coordinator
                break
        return local_name

    def route_for_group(self, group_id: str) -> tuple[str, str]:
        routes = {group: (robot, task) for group, robot, task in self.routes}
        try:
            return routes[group_id]
        except KeyError as exc:
            raise ValueError(f"missing route for planning group {group_id}") from exc

    def affected_robots(self, group_ids: Iterable[str]) -> tuple[str, ...]:
        robots: list[str] = []
        for group_id in group_ids:
            robot_name, _task_name = self.route_for_group(group_id)
            if robot_name in robots:
                raise ValueError(f"multiple selected groups resolve to robot {robot_name}")
            robots.append(robot_name)
        return tuple(robots)

    @property
    def groups(self) -> tuple[str, ...]:
        return tuple(group for group, _, _ in self.routes)


@dataclass(frozen=True)
class TaskEntry:
    planning_group: str
    task_name: str
    request: Any
    robot_name: str | None = None


@dataclass(frozen=True)
class ExecutionPlan:
    generated_plan: Any
    entries: tuple[TaskEntry, ...]


@dataclass(frozen=True)
class PreparedPlan:
    generated_plan: GeneratedPlan
    entries: tuple[TaskEntry, ...]
    topology: ExecutionTopology | None = None
    _canonical: bool = field(default=False, repr=False, compare=False)


def prepare_generated_plan(
    generated_plan: GeneratedPlan, topology: ExecutionTopology
) -> PreparedPlan:
    """Purely bind a generated multi-robot plan to canonical coordinator tasks."""
    if not isinstance(generated_plan, GeneratedPlan):
        raise TypeError("expected GeneratedPlan")
    group_ids = tuple(generated_plan.group_ids)
    if not group_ids or len(set(group_ids)) != len(group_ids):
        raise ValueError("generated plan must contain unique planning groups")
    route_by_group = {group: (robot, task) for group, robot, task in topology.routes}
    if len(route_by_group) != len(topology.routes):
        raise ValueError("topology contains duplicate group routes")
    selected_routes: list[tuple[str, str, str, RobotModelConfig, tuple[str, ...]]] = []
    affected: set[str] = set()
    for group_id in group_ids:
        route = route_by_group.get(group_id)
        if route is None:
            robot_name = group_id.split("/", 1)[0]
            try:
                if not topology.config_for_robot(robot_name).coordinator_task_name:
                    raise ValueError(f"no coordinator task for selected robot {robot_name}")
            except KeyError:
                pass
            raise ValueError(f"missing route for planning group {group_id}")
        robot_name, task_name = route
        if not task_name:
            raise ValueError(f"missing coordinator task for {group_id}")
        if robot_name in affected:
            raise ValueError(f"multiple selected groups resolve to robot {robot_name}")
        config = topology.config_for_robot(robot_name)
        prefix, _, definition_name = group_id.partition("/")
        if prefix != robot_name or not definition_name:
            raise ValueError(f"invalid canonical planning group {group_id}")
        definitions = [
            definition
            for definition in config.planning_groups
            if definition.name == definition_name
        ]
        if len(definitions) != 1:
            raise ValueError(f"missing config group for {group_id}")
        expected_local = tuple(definitions[0].joint_names)
        if len(set(expected_local)) != len(expected_local):
            raise ValueError(f"duplicate intended joints for {group_id}")
        affected.add(robot_name)
        selected_routes.append((group_id, robot_name, task_name, config, expected_local))
    expected_by_robot = {robot: set(expected) for _, robot, _, _, expected in selected_routes}

    trajectory = generated_plan.trajectory
    names = tuple(trajectory.joint_names)
    if not names:
        raise ValueError("generated trajectory has no joint names")
    if len(set(names)) != len(names):
        raise ValueError("duplicate global trajectory joint assignment")
    columns: dict[str, dict[str, int]] = {robot: {} for robot in affected}
    known_robots = {config.name for config in topology.robot_configs}
    for index, global_name in enumerate(names):
        if global_name.count("/") != 1:
            raise ValueError(f"malformed global joint name: {global_name}")
        robot_name, local_name = global_name.split("/", 1)
        if not robot_name or not local_name or robot_name not in known_robots:
            raise ValueError(f"unknown global joint assignment: {global_name}")
        if robot_name in columns:
            if local_name not in expected_by_robot[robot_name]:
                raise ValueError(f"additional joint column for selected robot: {global_name}")
            if local_name in columns[robot_name]:
                raise ValueError(f"duplicate robot joint assignment: {global_name}")
            columns[robot_name][local_name] = index
    points = tuple(trajectory.points)
    for point in points:
        if len(point.positions) != len(names):
            raise ValueError("trajectory point dimension does not match joint names")
        if point.velocities and len(point.velocities) != len(names):
            raise ValueError("trajectory velocity dimension does not match joint names")

    entries: list[TaskEntry] = []
    for group_id, robot_name, task_name, _config, expected_local in selected_routes:
        robot_columns = columns[robot_name]
        missing = [local for local in expected_local if local not in robot_columns]
        if missing:
            raise ValueError(f"missing robot joints for {robot_name}: {missing}")
        intended = set(expected_local)
        local_order = tuple(local for local in robot_columns if local in intended)
        coordinator_names = tuple(
            topology.coordinator_joint_name(robot_name, local) for local in local_order
        )
        if len(set(coordinator_names)) != len(coordinator_names):
            raise ValueError(f"duplicate coordinator joint names for {robot_name}")
        local_trajectory = JointTrajectory(
            joint_names=list(coordinator_names),
            points=[
                TrajectoryPoint(
                    time_from_start=point.time_from_start,
                    positions=[point.positions[robot_columns[local]] for local in local_order],
                    velocities=(
                        [point.velocities[robot_columns[local]] for local in local_order]
                        if point.velocities
                        else None
                    ),
                )
                for point in points
            ],
        )
        entries.append(TaskEntry(group_id, task_name, {"trajectory": local_trajectory}, robot_name))
    return PreparedPlan(generated_plan, tuple(entries), topology, True)


prepare_plan = prepare_generated_plan
prepare_execution_plan = prepare_generated_plan
materialize_prepared_plan = prepare_generated_plan
