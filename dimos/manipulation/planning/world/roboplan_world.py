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

"""RoboPlan-backed manipulation world implementation.

This adapter imports RoboPlan at module load time. The factory imports this module
only when the RoboPlan backend is requested, so default planning paths do not need
the optional dependency installed.
"""

from __future__ import annotations

from collections.abc import Mapping
from contextlib import contextmanager
from dataclasses import dataclass, field, replace
from pathlib import Path
from threading import RLock
import time
from typing import TYPE_CHECKING, Any

import numpy as np

try:
    import roboplan.core as roboplan_core
    import roboplan.rrt as roboplan_rrt
except ImportError as exc:
    raise ImportError(
        "RoboPlanWorld requires the optional roboplan dependency. "
        "Install the manipulation extra before selecting the roboplan backend."
    ) from exc

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.groups.utils import joint_state_to_ordered_positions
from dimos.manipulation.planning.planners.selected_joint_space import normalize_selection_target
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import (
    Obstacle,
    PlanningGroupID,
    PlanningResult,
    RobotName,
    WorldRobotID,
)
from dimos.manipulation.planning.utils.path_utils import compute_path_length
from dimos.manipulation.planning.world.roboplan_model import (
    RoboPlanGroup,
    RoboPlanModel,
    build_roboplan_model,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

if TYPE_CHECKING:
    from collections.abc import Generator

    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec.protocols import WorldSpec

logger = setup_logger()

_WORLD_FRAME = ""


@dataclass
class _RoboPlanRobotData:
    robot_id: WorldRobotID
    config: RobotModelConfig
    lower_limits: NDArray[np.float64] | None = None
    upper_limits: NDArray[np.float64] | None = None


@dataclass
class RoboPlanContext:
    """DimOS context wrapper for RoboPlan world state."""

    q_by_robot: dict[WorldRobotID, NDArray[np.float64]] = field(default_factory=dict)


class RoboPlanWorld:
    """WorldSpec implementation backed by RoboPlan scene and collision queries."""

    def __init__(self, enable_viz: bool = False, **_: object) -> None:
        self._scene: Any | None = None
        self._model: RoboPlanModel | None = None
        self._enable_viz = enable_viz
        if enable_viz:
            logger.warning("RoboPlanWorld does not currently provide manipulation visualization")

        self._robots: dict[WorldRobotID, _RoboPlanRobotData] = {}
        self._planning_groups = PlanningGroupRegistry()
        self._obstacles: dict[str, Obstacle] = {}
        self._authoritative_robot_ids: set[WorldRobotID] = set()
        self._robot_counter = 0
        self._finalized = False
        self._live_context = RoboPlanContext()
        self._lock = RLock()

    # Robot Management

    def add_robot(self, config: RobotModelConfig) -> WorldRobotID:
        """Register a robot for the scene built by :meth:`finalize`."""
        if self._finalized:
            raise RuntimeError("Cannot add robot after world is finalized")
        if not Path(config.model_path).exists():
            raise FileNotFoundError(f"Robot model not found: {Path(config.model_path).resolve()}")
        if any(data.config.name == config.name for data in self._robots.values()):
            raise ValueError(f"Robot name '{config.name}' is already registered")
        self._validate_planning_group_config(config)
        self._validate_robot_config(config)
        self._robot_counter += 1
        robot_id = f"robot_{self._robot_counter}"
        self._robots[robot_id] = _RoboPlanRobotData(
            robot_id=robot_id,
            config=config,
        )
        self._planning_groups.add_robot(config)
        self._live_context.q_by_robot[robot_id] = np.zeros(
            len(config.joint_names), dtype=np.float64
        )
        return robot_id

    def get_robot_ids(self) -> list[WorldRobotID]:
        """Get all robot IDs in the world."""
        return list(self._robots.keys())

    def get_robot_config(self, robot_id: WorldRobotID) -> RobotModelConfig:
        """Get robot configuration by ID."""
        return self._get_robot(robot_id).config

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Get joint limits in DimOS joint order."""
        robot = self._get_robot(robot_id)
        if robot.lower_limits is None or robot.upper_limits is None:
            raise RuntimeError("Joint limits are available after RoboPlan finalization")
        return robot.lower_limits.copy(), robot.upper_limits.copy()

    # Obstacle Management

    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Add a supported obstacle to the RoboPlan scene."""
        obstacle_id = obstacle.name
        if obstacle_id in self._obstacles:
            return obstacle_id
        if self._finalized:
            self._add_obstacle_to_scene(obstacle, obstacle_id)
        self._obstacles[obstacle_id] = obstacle
        return obstacle_id

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle from the RoboPlan scene."""
        if obstacle_id not in self._obstacles:
            return False
        if self._finalized:
            self._require_scene().removeGeometry(obstacle_id)
        del self._obstacles[obstacle_id]
        return True

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        """Update an obstacle pose and invalidate collision scratch."""
        if obstacle_id not in self._obstacles:
            return False
        if self._finalized:
            self._require_scene().updateGeometryPlacement(
                obstacle_id, _WORLD_FRAME, pose_to_matrix(pose)
            )
        self._obstacles[obstacle_id] = replace(self._obstacles[obstacle_id], pose=pose)
        return True

    def clear_obstacles(self) -> None:
        """Remove all tracked obstacles."""
        for obstacle_id in list(self._obstacles.keys()):
            self.remove_obstacle(obstacle_id)

    def get_obstacles(self) -> list[Obstacle]:
        """Get all obstacles currently tracked by DimOS."""
        return list(self._obstacles.values())

    # Lifecycle

    def finalize(self) -> None:
        """Build one immutable robot model and materialize pending obstacles."""
        if self._finalized:
            return
        model = build_roboplan_model(
            list(self._robots.values()),
            self._planning_groups,
            roboplan_core.Scene,
        )
        self._model = model
        self._scene = model.scene
        try:
            for robot in self._robots.values():
                group = self._legacy_group(robot.config.name)
                lower, upper = self._extract_joint_limits(robot.config, group)
                robot.lower_limits = lower
                robot.upper_limits = upper
            for obstacle_id, obstacle in self._obstacles.items():
                self._add_obstacle_to_scene(obstacle, obstacle_id)
        except BaseException:
            self._scene = None
            self._model = None
            model.owner.cleanup()
            raise
        self._finalized = True

    @property
    def is_finalized(self) -> bool:
        """Check whether the scene is finalized."""
        return self._finalized

    # Context Management

    def get_live_context(self) -> RoboPlanContext:
        """Get the live context that mirrors robot state."""
        self._require_finalized()
        return self._live_context

    @contextmanager
    def scratch_context(self) -> Generator[RoboPlanContext, None, None]:
        """Create a per-consumer context with independent collision scratch."""
        self._require_finalized()
        ctx = RoboPlanContext(
            q_by_robot={robot_id: q.copy() for robot_id, q in self._live_context.q_by_robot.items()}
        )
        yield ctx

    def sync_from_joint_state(self, robot_id: WorldRobotID, joint_state: JointState) -> None:
        """Sync live context from a driver joint-state message."""
        if not self._finalized:
            return
        self.set_joint_state(self._live_context, robot_id, joint_state)
        self._authoritative_robot_ids.add(robot_id)

    # State Operations

    def set_joint_state(
        self, ctx: RoboPlanContext, robot_id: WorldRobotID, joint_state: JointState
    ) -> None:
        """Set robot joint state in a context."""
        self._require_finalized()
        ctx.q_by_robot[robot_id] = self._joint_state_to_q(robot_id, joint_state)

    def get_joint_state(self, ctx: RoboPlanContext, robot_id: WorldRobotID) -> JointState:
        """Get robot joint state from a context."""
        robot = self._get_robot(robot_id)
        q = ctx.q_by_robot.get(robot_id)
        if q is None:
            q = np.zeros(len(robot.config.joint_names), dtype=np.float64)
        return JointState(name=robot.config.joint_names, position=q.astype(float).tolist())

    # Collision Checking

    def is_collision_free(self, ctx: RoboPlanContext, robot_id: WorldRobotID) -> bool:
        """Check if the robot configuration in a context is collision-free."""
        self._require_finalized()
        q = ctx.q_by_robot.get(robot_id)
        if q is None:
            raise KeyError(f"Robot '{robot_id}' not found in context")
        return not self._has_collisions(ctx, robot_id, q)

    def get_min_distance(self, ctx: RoboPlanContext, robot_id: WorldRobotID) -> float:
        """Get minimum signed distance.

        RoboPlan signed-distance semantics are not verified yet, so do not return
        a misleading approximation.
        """
        raise NotImplementedError("RoboPlanWorld.get_min_distance is not implemented")

    def check_config_collision_free(self, robot_id: WorldRobotID, joint_state: JointState) -> bool:
        """Check a joint state using a scratch collision context."""
        with self.scratch_context() as ctx:
            self.set_joint_state(ctx, robot_id, joint_state)
            return self.is_collision_free(ctx, robot_id)

    def check_edge_collision_free(
        self,
        robot_id: WorldRobotID,
        start: JointState,
        end: JointState,
        step_size: float = 0.05,
    ) -> bool:
        """Check if an interpolated edge is collision-free."""
        self._require_finalized()
        q_start = self._joint_state_to_q(robot_id, start)
        q_end = self._joint_state_to_q(robot_id, end)
        with self.scratch_context() as ctx:
            return not self._call_path_collision_checker(ctx, robot_id, q_start, q_end, step_size)

    # Forward Kinematics

    def get_ee_pose(self, ctx: RoboPlanContext, robot_id: WorldRobotID) -> PoseStamped:
        """Get end-effector pose if RoboPlan exposes FK."""
        robot = self._get_robot(robot_id)
        group_id = self._primary_pose_group_id_for_config(robot.config)
        if group_id is None:
            raise ValueError(f"Robot '{robot.config.name}' has no pose-targetable planning group")
        return self.get_group_ee_pose(ctx, group_id)

    def get_group_ee_pose(self, ctx: RoboPlanContext, group_id: PlanningGroupID) -> PoseStamped:
        """Get planning-group tip pose if RoboPlan exposes FK."""
        group = self._planning_group_from_id(group_id)
        if group.tip_link is None:
            raise ValueError(f"Planning group '{group_id}' has no tip link")
        mat = self.get_link_pose(ctx, self._robot_id_for_group(group_id), group.tip_link)
        pose = matrix_to_pose(mat)
        return PoseStamped(
            frame_id="world",
            position=[pose.position.x, pose.position.y, pose.position.z],
            orientation=[
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
        )

    def get_link_pose(
        self, ctx: RoboPlanContext, robot_id: WorldRobotID, link_name: str
    ) -> NDArray[np.float64]:
        """Get link pose as a 4x4 homogeneous transform."""
        q = ctx.q_by_robot.get(robot_id)
        if q is None:
            raise KeyError(f"Robot '{robot_id}' not found in context")
        scene = self._require_scene()
        robot = self._get_robot(robot_id)
        with self._lock:
            scene_q = self._full_scene_q(ctx, overlay=(robot_id, q))
            scene.setJointPositions(scene_q)
            result = scene.forwardKinematics(
                scene_q,
                self._require_model().native_link(robot.config.name, link_name),
                "",
            )
        return np.asarray(result, dtype=np.float64)

    def get_jacobian(self, ctx: RoboPlanContext, robot_id: WorldRobotID) -> NDArray[np.float64]:
        """Get end-effector Jacobian if RoboPlan exposes a compatible API."""
        robot = self._get_robot(robot_id)
        group_id = self._primary_pose_group_id_for_config(robot.config)
        if group_id is None:
            raise ValueError(f"Robot '{robot.config.name}' has no pose-targetable planning group")
        return self.get_group_jacobian(ctx, group_id)

    def get_group_jacobian(
        self, ctx: RoboPlanContext, group_id: PlanningGroupID
    ) -> NDArray[np.float64]:
        """Get planning-group Jacobian projected to group-local joint order."""
        group = self._planning_group_from_id(group_id)
        if group.tip_link is None:
            raise ValueError(f"Planning group '{group_id}' has no tip link")
        robot_id = self._robot_id_for_group(group_id)
        robot = self._get_robot(robot_id)
        scene = self._require_scene()
        model = self._require_model()
        with self._lock:
            scene_q = self._full_scene_q(ctx)
            scene.setJointPositions(scene_q)
            result = scene.computeFrameJacobian(
                scene_q,
                model.native_link(robot.config.name, group.tip_link),
                True,
            )
        arr = np.asarray(result, dtype=np.float64)
        if arr.shape[0] != 6:
            raise ValueError(f"Unexpected RoboPlan Jacobian shape: {arr.shape}; expected 6 x n")
        scene_joint_order = list(scene.getJointNames())
        if arr.shape[1] == len(scene_joint_order):
            native_names = [
                model.native_joint(group.robot_name, name) for name in group.local_joint_names
            ]
            return arr[:, [scene_joint_order.index(name) for name in native_names]]
        raise ValueError(
            f"Unexpected RoboPlan Jacobian shape: {arr.shape}; cannot project group '{group_id}'"
        )

    # PlannerSpec for native RoboPlan planning

    def plan_joint_path(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan using the legacy robot-scoped local-name contract."""
        if world is not self:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                message="RoboPlan-native planner requires its RoboPlanWorld instance",
            )
        try:
            q_start = self._joint_state_to_q(robot_id, start)
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_START, message=str(exc))
        try:
            q_goal = self._joint_state_to_q(robot_id, goal)
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_GOAL, message=str(exc))
        if not self._is_ready():
            return PlanningResult(
                status=PlanningStatus.INVALID_START,
                message="RoboPlan planning scene is not ready: authoritative state is incomplete",
            )
        robot = self._get_robot(robot_id)
        current = self._live_context.q_by_robot[robot_id]
        if not np.allclose(q_start, current, atol=1e-6, rtol=0.0):
            return PlanningResult(
                status=PlanningStatus.INVALID_START,
                message="Requested start state does not match current scene state",
            )
        group = self._legacy_group(robot.config.name)
        return self._plan_group(
            group,
            dict(zip(robot.config.joint_names, q_start, strict=True)),
            dict(zip(robot.config.joint_names, q_goal, strict=True)),
            timeout,
            5000,
        )

    def plan_selected_joint_path(
        self,
        world: WorldSpec,
        selection: PlanningGroupSelection,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
        max_iterations: int = 5000,
    ) -> PlanningResult:
        """Plan one or more non-overlapping groups through RoboPlan RRT."""
        if world is not self:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="RoboPlan-native planner requires its RoboPlanWorld instance",
            )
        if not selection.groups:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message="No planning groups selected",
            )
        group = self._require_model().groups.get(frozenset(selection.group_ids))
        if group is None:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="RoboPlan has no generated group for this selection",
            )
        try:
            normalized_start = normalize_selection_target(selection, start, "start")
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_START, message=str(exc))
        try:
            normalized_goal = normalize_selection_target(selection, goal, "goal")
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_GOAL, message=str(exc))
        if not self._is_ready():
            return PlanningResult(
                status=PlanningStatus.INVALID_START,
                message="RoboPlan planning scene is not ready: authoritative state is incomplete",
            )
        start_by_name = dict(zip(normalized_start.name, normalized_start.position, strict=True))
        current_by_name = self._current_global_positions()
        if any(
            not np.isclose(start_by_name[name], current_by_name[name], atol=1e-6, rtol=0.0)
            for name in selection.joint_names
        ):
            return PlanningResult(
                status=PlanningStatus.INVALID_START,
                message="Requested start state does not match current scene state",
            )
        return self._plan_group(
            group,
            start_by_name,
            dict(zip(normalized_goal.name, normalized_goal.position, strict=True)),
            timeout,
            max_iterations,
        )

    def get_name(self) -> str:
        """Get planner name."""
        return "RoboPlan"

    # Internals

    def _validate_robot_config(self, config: RobotModelConfig) -> None:
        if not config.joint_names:
            raise ValueError("RoboPlanWorld requires explicit joint_names")
        if config.base_pose.frame_id not in ("", "world"):
            raise ValueError("RoboPlanWorld base_pose frame_id must be empty or 'world'")

    def _extract_joint_limits(
        self, config: RobotModelConfig, group: RoboPlanGroup
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        if config.joint_limits_lower is not None and config.joint_limits_upper is not None:
            lower = np.asarray(config.joint_limits_lower, dtype=np.float64)
            upper = np.asarray(config.joint_limits_upper, dtype=np.float64)
        else:
            lower, upper = self._require_scene().getPositionLimitVectors(group.name, False)
            lower = np.asarray(lower, dtype=np.float64)
            upper = np.asarray(upper, dtype=np.float64)
            by_name = dict(zip(group.public_names, zip(lower, upper, strict=True), strict=True))
            lower = np.asarray([by_name[name][0] for name in config.joint_names])
            upper = np.asarray([by_name[name][1] for name in config.joint_names])
        if len(lower) != len(config.joint_names) or len(upper) != len(config.joint_names):
            raise ValueError("Joint limit length must match joint_names length")
        if np.any(~np.isfinite(lower)) or np.any(~np.isfinite(upper)):
            raise ValueError("RoboPlanWorld requires finite joint limits")
        return lower, upper

    def _validate_planning_group_config(self, config: RobotModelConfig) -> None:
        """Validate planning groups before mutating backend state."""
        PlanningGroupRegistry([config])

    def _planning_group_from_id(self, group_id: PlanningGroupID) -> PlanningGroup:
        return self._planning_groups.get(group_id)

    def _primary_pose_group_id_for_config(self, config: RobotModelConfig) -> PlanningGroupID | None:
        return self._planning_groups.primary_pose_group_id_for_robot(config.name)

    def _get_robot(self, robot_id: WorldRobotID) -> _RoboPlanRobotData:
        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")
        return self._robots[robot_id]

    def _robot_id_for_group(self, group_id: PlanningGroupID) -> WorldRobotID:
        group = self._planning_group_from_id(group_id)
        matches = [
            rid for rid, data in self._robots.items() if data.config.name == group.robot_name
        ]
        if not matches:
            raise KeyError(f"No robot registered for planning group '{group_id}'")
        return matches[0]

    def _joint_state_to_q(
        self, robot_id: WorldRobotID, joint_state: JointState
    ) -> NDArray[np.float64]:
        robot = self._get_robot(robot_id)
        return joint_state_to_ordered_positions(
            joint_state,
            joint_names=robot.config.joint_names,
            joint_name_mapping=robot.config.joint_name_mapping,
        )

    def _require_finalized(self) -> None:
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

    def _require_scene(self) -> Any:
        if self._scene is None:
            raise RuntimeError("RoboPlan scene is not initialized; finalize the world first")
        return self._scene

    def _require_model(self) -> RoboPlanModel:
        if self._model is None:
            raise RuntimeError("RoboPlan model is not initialized; finalize the world first")
        return self._model

    def _full_scene_q(
        self,
        ctx: RoboPlanContext,
        overlay: tuple[WorldRobotID, NDArray[np.float64]] | None = None,
    ) -> NDArray[np.float64]:
        scene = self._require_scene()
        group = self._require_model().all_group
        positions = self._current_global_positions(ctx, overlay)
        q = np.asarray([positions[name] for name in group.public_names], dtype=np.float64)
        return np.asarray(scene.toFullJointPositions(group.name, q), dtype=np.float64)

    def _current_global_positions(
        self,
        ctx: RoboPlanContext | None = None,
        overlay: tuple[WorldRobotID, NDArray[np.float64]] | None = None,
    ) -> dict[str, float]:
        context = ctx if ctx is not None else self._live_context
        positions: dict[str, float] = {}
        for robot_id, robot in self._robots.items():
            q = (
                overlay[1]
                if overlay is not None and overlay[0] == robot_id
                else context.q_by_robot.get(robot_id)
            )
            if q is None or len(q) != len(robot.config.joint_names):
                raise RuntimeError(f"Missing authoritative state for robot '{robot_id}'")
            positions.update(
                {
                    f"{robot.config.name}/{name}": float(value)
                    for name, value in zip(robot.config.joint_names, q, strict=True)
                }
            )
        return positions

    def _has_collisions(
        self,
        ctx: RoboPlanContext,
        robot_id: WorldRobotID,
        q: NDArray[np.float64],
    ) -> bool:
        with self._lock:
            scene = self._require_scene()
            scene_q = self._full_scene_q(ctx, overlay=(robot_id, q))
            scene.setJointPositions(scene_q)
            return bool(scene.hasCollisions(scene_q))

    def _call_path_collision_checker(
        self,
        ctx: RoboPlanContext,
        robot_id: WorldRobotID,
        q_start: NDArray[np.float64],
        q_end: NDArray[np.float64],
        step_size: float,
    ) -> bool:
        with self._lock:
            scene = self._require_scene()
            scene_q_start = self._full_scene_q(ctx, overlay=(robot_id, q_start))
            scene_q_end = self._full_scene_q(ctx, overlay=(robot_id, q_end))
            scene.setJointPositions(scene_q_start)
            return bool(
                roboplan_core.hasCollisionsAlongPath(
                    scene,
                    scene_q_start,
                    scene_q_end,
                    step_size,
                    False,
                    True,
                )
            )

    def _add_obstacle_to_scene(self, obstacle: Obstacle, obstacle_id: str) -> None:
        scene = self._require_scene()
        matrix = pose_to_matrix(obstacle.pose)
        color = np.asarray(obstacle.color, dtype=np.float64)
        if obstacle.obstacle_type == ObstacleType.BOX:
            self._require_dimensions(obstacle, 3)
            width, height, depth = obstacle.dimensions
            scene.addBoxGeometry(
                obstacle_id,
                _WORLD_FRAME,
                roboplan_core.Box(width, height, depth),
                matrix,
                color,
            )
            return
        if obstacle.obstacle_type == ObstacleType.SPHERE:
            self._require_dimensions(obstacle, 1)
            (radius,) = obstacle.dimensions
            scene.addSphereGeometry(
                obstacle_id, _WORLD_FRAME, roboplan_core.Sphere(radius), matrix, color
            )
            return
        if obstacle.obstacle_type == ObstacleType.CYLINDER:
            self._require_dimensions(obstacle, 2)
            radius, length = obstacle.dimensions
            scene.addCylinderGeometry(
                obstacle_id,
                _WORLD_FRAME,
                roboplan_core.Cylinder(radius, length),
                matrix,
                color,
            )
            return
        if obstacle.obstacle_type == ObstacleType.MESH:
            if not obstacle.mesh_path:
                raise ValueError("MESH obstacle requires mesh_path")
            scene.addMeshGeometry(
                obstacle_id,
                _WORLD_FRAME,
                roboplan_core.Mesh(obstacle.mesh_path),
                matrix,
                color,
            )
            return
        raise ValueError(f"Unsupported obstacle type: {obstacle.obstacle_type}")

    def _require_dimensions(self, obstacle: Obstacle, n_dims: int) -> None:
        if len(obstacle.dimensions) != n_dims:
            raise ValueError(
                f"{obstacle.obstacle_type.name} obstacle requires {n_dims} dimensions, "
                f"got {len(obstacle.dimensions)}"
            )

    def _legacy_group(self, robot_name: RobotName) -> RoboPlanGroup:
        model = self._require_model()
        group_id = model.legacy_group_ids[robot_name]
        return model.groups[frozenset((group_id,))]

    def _is_ready(self) -> bool:
        return bool(self._robots) and self._authoritative_robot_ids == set(self._robots)

    def _plan_group(
        self,
        group: RoboPlanGroup,
        start_by_name: Mapping[str, float],
        goal_by_name: Mapping[str, float],
        timeout: float,
        max_iterations: int,
    ) -> PlanningResult:
        started = time.time()
        try:
            q_start = np.asarray(
                [start_by_name[name] for name in group.public_names],
                dtype=np.float64,
            )
            q_goal = np.asarray(
                [goal_by_name[name] for name in group.public_names],
                dtype=np.float64,
            )
        except KeyError as exc:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message=f"Joint target is missing '{exc.args[0]}'",
            )
        try:
            with self._lock:
                scene = self._require_scene()
                scene.setJointPositions(self._full_scene_q(self._live_context))
                result = self._run_native_rrt(
                    group,
                    q_start,
                    q_goal,
                    timeout,
                    max_iterations,
                )
            path = self._path_from_native(group, result)
        except ValueError as exc:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - started,
                message=f"RoboPlan-native planning failed: {exc}",
            )
        if not path:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - started,
                message="RoboPlan-native planning failed: returned an empty path",
            )
        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=path,
            planning_time=time.time() - started,
            path_length=compute_path_length(path),
            message="RoboPlan path found",
        )

    def _run_native_rrt(
        self,
        group: RoboPlanGroup,
        q_start: NDArray[np.float64],
        q_goal: NDArray[np.float64],
        timeout: float,
        max_iterations: int,
    ) -> Any:
        options: Any = roboplan_rrt.RRTOptions()
        options.group_name = group.name
        options.max_planning_time = timeout
        options.max_nodes = max_iterations
        options.collision_check_use_bisection = True
        planner = roboplan_rrt.RRT(self._require_scene(), options)
        start = roboplan_core.JointConfiguration(list(group.native_names), q_start)
        goal = roboplan_core.JointConfiguration(list(group.native_names), q_goal)
        result = planner.plan(start, goal)
        if result is None:
            raise ValueError("RoboPlan RRT returned no path")
        return result

    def _path_from_native(self, group: RoboPlanGroup, result: Any) -> list[JointState]:
        result_names = tuple(getattr(result, "joint_names", ()) or group.native_names)
        if set(result_names) != set(group.native_names):
            raise ValueError("RoboPlan path joint names do not match the selected group")
        public_by_native = dict(zip(group.native_names, group.public_names, strict=True))
        source_names = tuple(public_by_native[name] for name in result_names)
        path: list[JointState] = []
        for waypoint in result.positions:
            values = np.asarray(waypoint, dtype=np.float64)
            if len(values) != len(source_names):
                raise ValueError("RoboPlan path waypoint length does not match its names")
            positions = dict(zip(source_names, values, strict=True))
            path.append(
                JointState(
                    name=list(group.output_names),
                    position=[float(positions[name]) for name in group.output_names],
                )
            )
        return path
