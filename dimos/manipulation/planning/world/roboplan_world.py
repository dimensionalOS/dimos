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

from contextlib import contextmanager
from dataclasses import dataclass, field, replace
from pathlib import Path
import tempfile
import time
from typing import TYPE_CHECKING
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

import numpy as np

try:
    import roboplan.core as roboplan_core
    import roboplan.rrt as roboplan_rrt
except ImportError as exc:
    raise ImportError(
        "RoboPlanWorld requires the optional roboplan dependency. "
        "Install the manipulation extra before selecting the roboplan backend."
    ) from exc

from dimos.manipulation.planning.groups.identifiers import (
    make_global_joint_names,
    make_planning_group_id,
)
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.groups.utils import joint_state_to_ordered_positions
from dimos.manipulation.planning.planners.selected_joint_space import normalize_selection_target
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import (
    Obstacle,
    PlanningGroupID,
    PlanningResult,
    WorldRobotID,
)
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.manipulation.planning.utils.path_utils import compute_path_length
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
    lower_limits: NDArray[np.float64]
    upper_limits: NDArray[np.float64]


@dataclass
class RoboPlanContext:
    """DimOS context wrapper for RoboPlan world state."""

    q_by_robot: dict[WorldRobotID, NDArray[np.float64]] = field(default_factory=dict)


class RoboPlanWorld:
    """WorldSpec implementation backed by RoboPlan scene and collision queries."""

    def __init__(self, enable_viz: bool = False, **_: object) -> None:
        self._scene: roboplan_core.Scene | None = None
        self._enable_viz = enable_viz
        if enable_viz:
            logger.warning("RoboPlanWorld does not currently provide manipulation visualization")

        self._robots: dict[WorldRobotID, _RoboPlanRobotData] = {}
        self._obstacles: dict[str, Obstacle] = {}
        self._robot_counter = 0
        self._finalized = False
        self._live_context = RoboPlanContext()
        self._srdf_tempdirs: list[tempfile.TemporaryDirectory[str]] = []

    # Robot Management

    def add_robot(self, config: RobotModelConfig) -> WorldRobotID:
        """Add a supported robot model to the RoboPlan scene."""
        if self._finalized:
            raise RuntimeError("Cannot add robot after world is finalized")
        if self._robots:
            raise ValueError("RoboPlanWorld currently supports one robot per Scene")
        if not Path(config.model_path).exists():
            raise FileNotFoundError(f"Robot model not found: {Path(config.model_path).resolve()}")
        if any(data.config.name == config.name for data in self._robots.values()):
            raise ValueError(f"Robot name '{config.name}' is already registered")
        self._validate_planning_group_config(config)

        self._validate_robot_config(config)
        self._robot_counter += 1
        robot_id = f"robot_{self._robot_counter}"
        self._scene = self._create_scene(config)
        lower, upper = self._extract_joint_limits(config)
        self._robots[robot_id] = _RoboPlanRobotData(
            robot_id=robot_id,
            config=config,
            lower_limits=lower,
            upper_limits=upper,
        )
        self._live_context.q_by_robot[robot_id] = np.zeros(
            len(config.joint_names), dtype=np.float64
        )
        logger.info(f"Added RoboPlan robot '{robot_id}' ({config.name})")
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
        return robot.lower_limits.copy(), robot.upper_limits.copy()

    # Obstacle Management

    def add_obstacle(self, obstacle: Obstacle) -> str:
        """Add a supported obstacle to the RoboPlan scene."""
        obstacle_id = obstacle.name
        if obstacle_id in self._obstacles:
            return obstacle_id
        self._add_obstacle_to_scene(obstacle, obstacle_id)
        self._obstacles[obstacle_id] = obstacle
        return obstacle_id

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle from the RoboPlan scene."""
        if obstacle_id not in self._obstacles:
            return False
        scene = self._require_scene()
        scene.removeGeometry(obstacle_id)
        del self._obstacles[obstacle_id]
        return True

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        """Update an obstacle pose and invalidate collision scratch."""
        if obstacle_id not in self._obstacles:
            return False
        scene = self._require_scene()
        scene.updateGeometryPlacement(obstacle_id, _WORLD_FRAME, pose_to_matrix(pose))
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
        """Mark the RoboPlan scene ready for DimOS planning queries.

        RoboPlan Python bindings construct a query-ready Scene directly; v0.4.0
        exposes no Scene.finalize() lifecycle method.
        """
        self._require_scene()
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
        return not self._has_collisions(robot_id, q)

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
        return not self._call_path_collision_checker(robot_id, q_start, q_end, step_size)

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
        result = scene.forwardKinematics(self._to_scene_q(robot_id, q), link_name, "")
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
        q = ctx.q_by_robot.get(robot_id)
        if q is None:
            raise KeyError(f"Robot '{robot_id}' not found in context")
        scene = self._require_scene()
        result = scene.computeFrameJacobian(self._to_scene_q(robot_id, q), group.tip_link, True)
        arr = np.asarray(result, dtype=np.float64)
        if arr.shape[0] != 6:
            raise ValueError(f"Unexpected RoboPlan Jacobian shape: {arr.shape}; expected 6 x n")
        scene_joint_order = self._query_scene_joint_order(scene, robot.config)
        if scene_joint_order is not None and arr.shape[1] == len(scene_joint_order):
            missing = [name for name in group.local_joint_names if name not in scene_joint_order]
            if missing:
                raise ValueError(f"Unknown joints for planning group '{group_id}': {missing}")
            indices = [scene_joint_order.index(name) for name in group.local_joint_names]
            return arr[:, indices]
        if arr.shape[1] == len(robot.config.joint_names):
            missing = [
                name for name in group.local_joint_names if name not in robot.config.joint_names
            ]
            if missing:
                raise ValueError(f"Unknown joints for planning group '{group_id}': {missing}")
            indices = [robot.config.joint_names.index(name) for name in group.local_joint_names]
            return arr[:, indices]
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
        """Plan a path using RoboPlan-native RRT when selected as planner."""
        if world is not self:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                message="RoboPlan-native planner requires its RoboPlanWorld instance",
            )
        start_time = time.time()
        q_start = self._joint_state_to_q(robot_id, start)
        q_goal = self._joint_state_to_q(robot_id, goal)
        try:
            path_arrays = self._run_native_rrt(robot_id, q_start, q_goal, timeout)
        except ValueError as exc:
            # _run_native_rrt raises ValueError for the known "no path" case; let any
            # unexpected error propagate instead of swallowing its traceback.
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message=f"RoboPlan-native planning failed: {exc}",
            )
        if not path_arrays:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message="RoboPlan-native planning failed: returned an empty path",
            )
        robot = self._get_robot(robot_id)
        path = [
            JointState(
                name=list(robot.config.joint_names),
                position=np.asarray(q).astype(float).tolist(),
            )
            for q in path_arrays
        ]
        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=path,
            planning_time=time.time() - start_time,
            path_length=compute_path_length(path),
            message="RoboPlan path found",
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
        """Plan a single planning group using RoboPlan's native RRT."""
        if world is not self:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                message="RoboPlan-native planner requires its RoboPlanWorld instance",
            )
        start_time = time.time()
        if not selection.groups:
            return PlanningResult(
                status=PlanningStatus.INVALID_GOAL,
                message="No planning groups selected",
            )
        if len(selection.groups) != 1:
            return PlanningResult(
                status=PlanningStatus.UNSUPPORTED,
                message="RoboPlan-native planning supports exactly one selected planning group",
            )

        group = selection.groups[0]
        try:
            normalized_start = normalize_selection_target(selection, start, "start")
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_START, message=str(exc))
        try:
            normalized_goal = normalize_selection_target(selection, goal, "goal")
        except ValueError as exc:
            return PlanningResult(status=PlanningStatus.INVALID_GOAL, message=str(exc))

        robot_id = self._robot_id_for_group(group.id)
        q_start = np.asarray(normalized_start.position, dtype=np.float64)
        q_goal = np.asarray(normalized_goal.position, dtype=np.float64)
        try:
            path_arrays = self._run_native_rrt(
                robot_id,
                q_start,
                q_goal,
                timeout,
                group_name=group.group_name,
                joint_names=list(group.local_joint_names),
            )
        except ValueError as exc:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message=f"RoboPlan-native planning failed: {exc}",
            )
        if not path_arrays:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message="RoboPlan-native planning failed: returned an empty path",
            )
        path = [
            JointState(
                name=list(selection.joint_names),
                position=np.asarray(q).astype(float).tolist(),
            )
            for q in path_arrays
        ]
        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=path,
            planning_time=time.time() - start_time,
            path_length=compute_path_length(path),
            message="RoboPlan path found",
        )

    def get_name(self) -> str:
        """Get planner name."""
        return "RoboPlan"

    # Internals

    def _create_scene(self, config: RobotModelConfig) -> roboplan_core.Scene:
        urdf_path = self._prepare_robot_urdf(config)
        srdf_path = self._prepare_robot_srdf(config, urdf_path)
        package_paths = [str(path) for path in config.package_paths.values()]
        scene = roboplan_core.Scene(config.name, str(urdf_path), str(srdf_path), package_paths)
        self._apply_collision_exclusions(scene, config, urdf_path)
        return scene

    def _validate_robot_config(self, config: RobotModelConfig) -> None:
        if not config.joint_names:
            raise ValueError("RoboPlanWorld requires explicit joint_names")
        if not np.allclose(pose_to_matrix(config.base_pose), np.eye(4)):
            raise ValueError("RoboPlanWorld does not yet support non-identity robot base_pose")

    def _prepare_robot_urdf(self, config: RobotModelConfig) -> Path:
        return Path(
            prepare_urdf_for_drake(
                config.model_path,
                package_paths=config.package_paths,
                xacro_args=config.xacro_args,
                convert_meshes=config.auto_convert_meshes,
            )
        )

    def _prepare_robot_srdf(self, config: RobotModelConfig, urdf_path: Path) -> Path:
        srdf = self._generate_srdf(config, urdf_path)
        srdf_tempdir = tempfile.TemporaryDirectory(prefix="dimos_roboplan_srdf_")
        self._srdf_tempdirs.append(srdf_tempdir)
        cache_dir = Path(srdf_tempdir.name)
        srdf_path = cache_dir / f"{config.name}.srdf"
        srdf_path.write_text(srdf)
        return srdf_path

    def _generate_srdf(self, config: RobotModelConfig, urdf_path: Path) -> str:
        lines = [f'<robot name="{escape(config.name)}">']
        lines.append(f'  <group name="{escape(config.name)}">')
        for joint_name in config.joint_names:
            lines.append(f'    <joint name="{escape(joint_name)}"/>')
        lines.append("  </group>")
        for group in config.planning_groups:
            lines.append(f'  <group name="{escape(group.name)}">')
            for joint_name in group.joint_names:
                lines.append(f'    <joint name="{escape(joint_name)}"/>')
            lines.append("  </group>")
        for link1, link2 in self._collision_exclusion_pairs(config, urdf_path):
            lines.append(
                f'  <disable_collisions link1="{escape(link1)}" link2="{escape(link2)}" '
                'reason="DimOS configured"/>'
            )
        lines.append("</robot>")
        return "\n".join(lines) + "\n"

    def _collision_exclusion_pairs(
        self, config: RobotModelConfig, urdf_path: Path
    ) -> list[tuple[str, str]]:
        pairs = set(config.collision_exclusion_pairs)
        pairs.update(self._adjacent_link_pairs_from_urdf(urdf_path))
        return sorted(pairs)

    def _adjacent_link_pairs_from_urdf(self, urdf_path: Path) -> list[tuple[str, str]]:
        try:
            root = ET.parse(urdf_path).getroot()
        except ET.ParseError as exc:
            raise ValueError(
                f"Unable to parse prepared URDF for SRDF generation: {urdf_path}"
            ) from exc

        pairs: list[tuple[str, str]] = []
        for joint in root.findall("joint"):
            parent = joint.find("parent")
            child = joint.find("child")
            parent_link = parent.get("link") if parent is not None else None
            child_link = child.get("link") if child is not None else None
            if parent_link and child_link:
                pairs.append((parent_link, child_link))
        return pairs

    def _apply_collision_exclusions(
        self, scene: roboplan_core.Scene, config: RobotModelConfig, urdf_path: Path
    ) -> None:
        for link1, link2 in self._collision_exclusion_pairs(config, urdf_path):
            try:
                scene.setCollisions(link1, link2, False)
            except RuntimeError:
                logger.warning(f"RoboPlan rejected collision exclusion pair: {link1} <-> {link2}")

    def _extract_joint_limits(
        self, config: RobotModelConfig
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        if config.joint_limits_lower is not None and config.joint_limits_upper is not None:
            lower = np.asarray(config.joint_limits_lower, dtype=np.float64)
            upper = np.asarray(config.joint_limits_upper, dtype=np.float64)
        else:
            limits = self._query_scene_joint_limits(config)
            if limits is None:
                raise ValueError(
                    "RoboPlanWorld requires explicit joint_limits_lower/joint_limits_upper "
                    "when limits cannot be read from RoboPlan bindings"
                )
            lower, upper = limits
        if len(lower) != len(config.joint_names) or len(upper) != len(config.joint_names):
            raise ValueError("Joint limit length must match joint_names length")
        if np.any(~np.isfinite(lower)) or np.any(~np.isfinite(upper)):
            raise ValueError("RoboPlanWorld requires finite joint limits")
        return lower, upper

    def _query_scene_joint_limits(
        self, config: RobotModelConfig
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
        scene = self._require_scene()
        joint_order = self._query_scene_joint_order(scene, config)
        if joint_order is None:
            return None
        lower, upper = scene.getPositionLimitVectors(config.name, False)
        lower_array = np.asarray(lower, dtype=np.float64)
        upper_array = np.asarray(upper, dtype=np.float64)
        if len(joint_order) != len(lower_array) or len(joint_order) != len(upper_array):
            raise ValueError(
                "RoboPlan joint limit order length does not match returned limit vectors"
            )
        if len(set(joint_order)) != len(joint_order):
            raise ValueError("RoboPlan returned duplicate joint names for joint limits")
        if set(joint_order) != set(config.joint_names):
            raise ValueError(
                "RoboPlan joint limit names do not match configured joint_names: "
                f"RoboPlan={joint_order}, configured={config.joint_names}"
            )
        order_indices = [joint_order.index(joint_name) for joint_name in config.joint_names]
        return lower_array[order_indices], upper_array[order_indices]

    def _query_scene_joint_order(
        self, scene: roboplan_core.Scene, config: RobotModelConfig
    ) -> list[str] | None:
        try:
            group_info = scene.getJointGroupInfo(config.name)
        except AttributeError:
            return None
        return list(group_info.joint_names)

    def _validate_planning_group_config(self, config: RobotModelConfig) -> None:
        """Validate planning groups before mutating backend state."""
        seen_group_names: set[str] = set()
        for definition in config.planning_groups:
            group_id = make_planning_group_id(config.name, definition.name)
            if definition.name in seen_group_names:
                raise ValueError(f"Planning group '{group_id}' is already registered")
            make_global_joint_names(config.name, definition.joint_names)
            seen_group_names.add(definition.name)

    def _planning_group_from_config(
        self, config: RobotModelConfig, group_id: PlanningGroupID
    ) -> PlanningGroup:
        for definition in config.planning_groups:
            if make_planning_group_id(config.name, definition.name) == group_id:
                return PlanningGroup(
                    id=group_id,
                    robot_name=config.name,
                    group_name=definition.name,
                    joint_names=tuple(make_global_joint_names(config.name, definition.joint_names)),
                    local_joint_names=definition.joint_names,
                    base_link=definition.base_link,
                    tip_link=definition.tip_link,
                    source=definition.source,
                )
        raise KeyError(f"Unknown planning group ID: {group_id}")

    def _planning_group_from_id(self, group_id: PlanningGroupID) -> PlanningGroup:
        for robot in self._robots.values():
            try:
                return self._planning_group_from_config(robot.config, group_id)
            except KeyError:
                continue
        raise KeyError(f"Unknown planning group ID: {group_id}")

    def _primary_pose_group_id_for_config(self, config: RobotModelConfig) -> PlanningGroupID | None:
        pose_group_ids = [
            make_planning_group_id(config.name, group.name)
            for group in config.planning_groups
            if group.has_pose_target
        ]
        if not pose_group_ids:
            return None
        if len(pose_group_ids) > 1:
            raise ValueError(
                f"Robot '{config.name}' has {len(pose_group_ids)} pose-targetable "
                "planning groups; use an explicit planning group ID"
            )
        return pose_group_ids[0]

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

    def _require_scene(self) -> roboplan_core.Scene:
        if self._scene is None:
            raise RuntimeError("RoboPlan scene is not initialized; add a robot first")
        return self._scene

    def _to_scene_q(self, robot_id: WorldRobotID, q: NDArray[np.float64]) -> NDArray[np.float64]:
        """Expand DimOS group positions to RoboPlan's full scene vector when available."""
        scene = self._require_scene()
        robot = self._get_robot(robot_id)
        if len(q) != len(robot.config.joint_names):
            return q
        return np.asarray(scene.toFullJointPositions(robot.config.name, q), dtype=np.float64)

    def _has_collisions(self, robot_id: WorldRobotID, q: NDArray[np.float64]) -> bool:
        scene = self._require_scene()
        scene_q = self._to_scene_q(robot_id, q)
        return bool(scene.hasCollisions(scene_q))

    def _call_path_collision_checker(
        self,
        robot_id: WorldRobotID,
        q_start: NDArray[np.float64],
        q_end: NDArray[np.float64],
        step_size: float,
    ) -> bool:
        scene = self._require_scene()
        scene_q_start = self._to_scene_q(robot_id, q_start)
        scene_q_end = self._to_scene_q(robot_id, q_end)
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

    def _run_native_rrt(
        self,
        robot_id: WorldRobotID,
        q_start: NDArray[np.float64],
        q_goal: NDArray[np.float64],
        timeout: float,
        *,
        group_name: str | None = None,
        joint_names: list[str] | None = None,
    ) -> list[NDArray[np.float64]]:
        scene = self._require_scene()
        robot = self._get_robot(robot_id)
        options = roboplan_rrt.RRTOptions()
        native_group_name = robot.config.name if group_name is None else group_name
        native_joint_names = robot.config.joint_names if joint_names is None else joint_names
        options.group_name = native_group_name
        options.max_planning_time = timeout
        options.collision_check_use_bisection = False
        planner = roboplan_rrt.RRT(scene, options)
        start_config = self._to_native_joint_configuration(robot_id, q_start, native_joint_names)
        goal_config = self._to_native_joint_configuration(robot_id, q_goal, native_joint_names)
        result = planner.plan(start_config, goal_config)
        if result is None:
            raise ValueError("RoboPlan RRT returned no path")
        return self._extract_native_path(result)

    def _to_native_joint_configuration(
        self, robot_id: WorldRobotID, q: NDArray[np.float64], joint_names: list[str] | None = None
    ) -> roboplan_core.JointConfiguration:
        robot = self._get_robot(robot_id)
        return roboplan_core.JointConfiguration(
            robot.config.joint_names if joint_names is None else joint_names,
            np.asarray(q, dtype=np.float64),
        )

    def _extract_native_path(self, result: roboplan_core.JointPath) -> list[NDArray[np.float64]]:
        return [np.asarray(q, dtype=np.float64) for q in result.positions]
