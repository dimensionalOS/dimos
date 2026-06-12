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

This adapter intentionally keeps RoboPlan optional and reflective at the boundary
because the Python bindings are split across namespace packages and still evolving.
Planning-critical features fail loudly when a safe mapping is unavailable.
"""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
import hashlib
from importlib import import_module
from pathlib import Path
import tempfile
import time
from typing import TYPE_CHECKING, Any
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

import numpy as np

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import Obstacle, PlanningResult, WorldRobotID
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.manipulation.planning.utils.path_utils import compute_path_length
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

if TYPE_CHECKING:
    from collections.abc import Generator

    from numpy.typing import NDArray

logger = setup_logger()

ROBOPLAN_INSTALL_HINT = (
    "RoboPlan is not installed. Install the optional backend with `uv sync --extra roboplan`."
)


class RoboPlanImportError(ImportError):
    """Raised when optional RoboPlan packages are requested but unavailable."""


def _import_roboplan_core() -> Any:
    """Import `roboplan.core` with an actionable optional-extra hint."""
    try:
        return import_module("roboplan.core")
    except ImportError as exc:
        raise RoboPlanImportError(f"{ROBOPLAN_INSTALL_HINT} Missing module: roboplan.core") from exc


def _import_roboplan_rrt() -> Any:
    """Import `roboplan.rrt` with an actionable optional-extra hint."""
    try:
        return import_module("roboplan.rrt")
    except ImportError as exc:
        raise RoboPlanImportError(f"{ROBOPLAN_INSTALL_HINT} Missing module: roboplan.rrt") from exc


@dataclass
class _RoboPlanRobotData:
    robot_id: WorldRobotID
    config: RobotModelConfig
    lower_limits: NDArray[np.float64]
    upper_limits: NDArray[np.float64]
    model_handle: Any = None


@dataclass
class RoboPlanContext:
    """DimOS context wrapper with per-context RoboPlan collision scratch."""

    q_by_robot: dict[WorldRobotID, NDArray[np.float64]] = field(default_factory=dict)
    collision_context: Any = None
    geometry_revision: int = 0


class RoboPlanWorld:
    """WorldSpec implementation backed by RoboPlan scene and collision queries."""

    def __init__(self, enable_viz: bool = False, **_: Any) -> None:
        self._core = _import_roboplan_core()
        self._scene: Any | None = None
        self._enable_viz = enable_viz
        if enable_viz:
            logger.warning("RoboPlanWorld does not currently provide manipulation visualization")

        self._robots: dict[WorldRobotID, _RoboPlanRobotData] = {}
        self._obstacles: dict[str, Obstacle] = {}
        self._obstacle_handles: dict[str, Any] = {}
        self._robot_counter = 0
        self._finalized = False
        self._geometry_revision = 0
        self._live_context = RoboPlanContext(geometry_revision=self._geometry_revision)

    # Robot Management

    def add_robot(self, config: RobotModelConfig) -> WorldRobotID:
        """Add a supported robot model to the RoboPlan scene."""
        if self._finalized:
            raise RuntimeError("Cannot add robot after world is finalized")
        if self._robots:
            raise ValueError("RoboPlanWorld currently supports one robot per Scene")
        if not Path(config.model_path).exists():
            raise FileNotFoundError(f"Robot model not found: {Path(config.model_path).resolve()}")

        self._validate_robot_config(config)
        self._robot_counter += 1
        robot_id = f"robot_{self._robot_counter}"
        self._scene = self._create_scene(config)
        model_handle = config.name
        lower, upper = self._extract_joint_limits(config, model_handle)
        self._robots[robot_id] = _RoboPlanRobotData(
            robot_id=robot_id,
            config=config,
            lower_limits=lower,
            upper_limits=upper,
            model_handle=model_handle,
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
        handle = self._add_obstacle_to_scene(obstacle, obstacle_id)
        self._obstacles[obstacle_id] = obstacle
        self._obstacle_handles[obstacle_id] = handle
        self._bump_geometry_revision()
        return obstacle_id

    def remove_obstacle(self, obstacle_id: str) -> bool:
        """Remove an obstacle from the RoboPlan scene."""
        if obstacle_id not in self._obstacles:
            return False
        handle = self._obstacle_handles.get(obstacle_id, obstacle_id)
        remove = self._lookup_method(self._scene, ("removeGeometry", "remove_geometry"))
        if remove is None:
            raise ValueError("RoboPlan scene does not expose obstacle removal")
        remove(handle)
        del self._obstacles[obstacle_id]
        self._obstacle_handles.pop(obstacle_id, None)
        self._bump_geometry_revision()
        return True

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> bool:
        """Update an obstacle pose and invalidate collision scratch."""
        if obstacle_id not in self._obstacles:
            return False
        handle = self._obstacle_handles.get(obstacle_id, obstacle_id)
        update = self._lookup_method(
            self._scene, ("updateGeometryPlacement", "update_geometry_placement")
        )
        if update is None:
            raise ValueError("RoboPlan scene does not expose obstacle pose updates")
        for args in ((handle, "world", pose_to_matrix(pose)), (handle, pose_to_matrix(pose))):
            try:
                update(*args)
                break
            except TypeError:
                continue
        else:
            raise ValueError("RoboPlan obstacle update signature is unsupported")
        obstacle = self._obstacles[obstacle_id]
        self._obstacles[obstacle_id] = Obstacle(
            name=obstacle.name,
            obstacle_type=obstacle.obstacle_type,
            pose=pose,
            dimensions=obstacle.dimensions,
            color=obstacle.color,
            mesh_path=obstacle.mesh_path,
        )
        self._bump_geometry_revision()
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
        """Finalize the RoboPlan scene for collision queries."""
        self._require_scene()
        finalize = self._lookup_method(self._scene, ("finalize", "Finalize"))
        if finalize is not None:
            finalize()
        self._finalized = True
        self._live_context.collision_context = self._create_collision_context()
        self._live_context.geometry_revision = self._geometry_revision

    @property
    def is_finalized(self) -> bool:
        """Check whether the scene is finalized."""
        return self._finalized

    # Context Management

    def get_live_context(self) -> RoboPlanContext:
        """Get the live context that mirrors robot state."""
        self._require_finalized()
        self._refresh_context_if_needed(self._live_context)
        return self._live_context

    @contextmanager
    def scratch_context(self) -> Generator[RoboPlanContext, None, None]:
        """Create a per-consumer context with independent collision scratch."""
        self._require_finalized()
        ctx = RoboPlanContext(
            q_by_robot={
                robot_id: q.copy() for robot_id, q in self._live_context.q_by_robot.items()
            },
            collision_context=self._create_collision_context(),
            geometry_revision=self._geometry_revision,
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
        self._refresh_context_if_needed(ctx)
        q = ctx.q_by_robot.get(robot_id)
        if q is None:
            raise KeyError(f"Robot '{robot_id}' not found in context")
        return not self._has_collisions(robot_id, q, ctx)

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
        q_start = self._joint_state_to_q(robot_id, start)
        q_end = self._joint_state_to_q(robot_id, end)
        with self.scratch_context() as ctx:
            path_check = self._lookup_path_collision_checker()
            if path_check is not None:
                return not bool(
                    self._call_path_collision_checker(
                        path_check, ctx, robot_id, q_start, q_end, step_size
                    )
                )

            # Safe fallback: explicit interpolation using RoboPlan config collision queries.
            dist = float(np.linalg.norm(q_end - q_start))
            if dist < 1e-8:
                ctx.q_by_robot[robot_id] = q_start
                return self.is_collision_free(ctx, robot_id)
            n_steps = max(2, int(np.ceil(dist / step_size)) + 1)
            for i in range(n_steps):
                t = i / (n_steps - 1)
                ctx.q_by_robot[robot_id] = q_start + t * (q_end - q_start)
                if not self.is_collision_free(ctx, robot_id):
                    return False
            return True

    # Forward Kinematics

    def get_ee_pose(self, ctx: RoboPlanContext, robot_id: WorldRobotID) -> PoseStamped:
        """Get end-effector pose if RoboPlan exposes FK."""
        robot = self._get_robot(robot_id)
        mat = self.get_link_pose(ctx, robot_id, robot.config.end_effector_link)
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
        fk = self._lookup_method(self._scene, ("forwardKinematics", "forward_kinematics"))
        if fk is None:
            raise NotImplementedError("RoboPlan scene does not expose forward kinematics")
        try:
            result = fk(self._to_scene_q(robot_id, q), link_name, "")
        except TypeError:
            result = fk(self._to_scene_q(robot_id, q), link_name)
        return np.asarray(result, dtype=np.float64)

    def get_jacobian(self, ctx: RoboPlanContext, robot_id: WorldRobotID) -> NDArray[np.float64]:
        """Get end-effector Jacobian if RoboPlan exposes a compatible API."""
        robot = self._get_robot(robot_id)
        q = ctx.q_by_robot.get(robot_id)
        if q is None:
            raise KeyError(f"Robot '{robot_id}' not found in context")
        jac = self._lookup_method(self._scene, ("computeFrameJacobian", "compute_frame_jacobian"))
        if jac is None:
            raise NotImplementedError("RoboPlan scene does not expose frame Jacobian")
        try:
            result = jac(self._to_scene_q(robot_id, q), robot.config.end_effector_link, True)
        except TypeError:
            result = jac(self._to_scene_q(robot_id, q), robot.config.end_effector_link)
        arr = np.asarray(result, dtype=np.float64)
        if arr.shape[0] != 6:
            raise ValueError(f"Unexpected RoboPlan Jacobian shape: {arr.shape}; expected 6 x n")
        return arr

    # PlannerSpec for native RoboPlan planning

    def plan_joint_path(
        self,
        world: Any,
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
        except Exception as exc:
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                planning_time=time.time() - start_time,
                message=f"RoboPlan-native planning failed: {exc}",
            )
        path = [
            JointState(name=start.name, position=np.asarray(q).astype(float).tolist())
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

    def _create_scene(self, config: RobotModelConfig) -> Any:
        scene_cls = getattr(self._core, "Scene", None)
        if scene_cls is None:
            scene_module = getattr(self._core, "scene", None)
            scene_cls = getattr(scene_module, "Scene", None)
        if scene_cls is None:
            raise ValueError("roboplan.core does not expose Scene")
        urdf_path = self._prepare_robot_urdf(config)
        srdf_path = self._prepare_robot_srdf(config, urdf_path)
        package_paths = [str(path) for path in config.package_paths.values()]
        try:
            scene = scene_cls(config.name, urdf_path, srdf_path, package_paths)
        except TypeError:
            scene = scene_cls(config.name, str(urdf_path), str(srdf_path), package_paths)
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
        cache_key = hashlib.md5(srdf.encode()).hexdigest()[:16]
        cache_dir = Path(tempfile.gettempdir()) / "dimos_roboplan_srdf" / cache_key
        cache_dir.mkdir(parents=True, exist_ok=True)
        srdf_path = cache_dir / f"{config.name}.srdf"
        srdf_path.write_text(srdf)
        return srdf_path

    def _generate_srdf(self, config: RobotModelConfig, urdf_path: Path) -> str:
        lines = [f'<robot name="{escape(config.name)}">']
        lines.append(f'  <group name="{escape(config.name)}">')
        for joint_name in config.joint_names:
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
        self, scene: Any, config: RobotModelConfig, urdf_path: Path
    ) -> None:
        set_collisions = self._lookup_method(scene, ("setCollisions", "set_collisions"))
        if set_collisions is None:
            return
        for link1, link2 in self._collision_exclusion_pairs(config, urdf_path):
            try:
                set_collisions(link1, link2, False)
            except RuntimeError:
                logger.debug(
                    f"RoboPlan did not accept collision exclusion pair: {link1} <-> {link2}"
                )

    def _extract_joint_limits(
        self, config: RobotModelConfig, model_handle: Any
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        if config.joint_limits_lower is not None and config.joint_limits_upper is not None:
            lower = np.asarray(config.joint_limits_lower, dtype=np.float64)
            upper = np.asarray(config.joint_limits_upper, dtype=np.float64)
        else:
            limits = self._query_scene_joint_limits(config, model_handle)
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
        self, config: RobotModelConfig, model_handle: Any
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]] | None:
        for name in ("getJointLimits", "get_joint_limits"):
            method = getattr(self._scene, name, None)
            if method is None:
                continue
            try:
                result = method(config.joint_names)
            except TypeError:
                result = method(model_handle, config.joint_names)
            lower, upper = result
            return np.asarray(lower, dtype=np.float64), np.asarray(upper, dtype=np.float64)
        limits = self._lookup_method(
            self._scene, ("getPositionLimitVectors", "get_position_limit_vectors")
        )
        if limits is not None:
            limit_arg_options: list[tuple[Any, ...]] = [(config.name, False), (config.name,), ()]
            for args in limit_arg_options:
                try:
                    lower, upper = limits(*args)
                    return np.asarray(lower, dtype=np.float64), np.asarray(upper, dtype=np.float64)
                except TypeError:
                    continue
        return None

    def _get_robot(self, robot_id: WorldRobotID) -> _RoboPlanRobotData:
        if robot_id not in self._robots:
            raise KeyError(f"Robot '{robot_id}' not found")
        return self._robots[robot_id]

    def _joint_state_to_q(
        self, robot_id: WorldRobotID, joint_state: JointState
    ) -> NDArray[np.float64]:
        robot = self._get_robot(robot_id)
        if len(joint_state.position) != len(robot.config.joint_names):
            raise ValueError("JointState position length must match configured joint count")
        if not joint_state.name:
            return np.asarray(joint_state.position, dtype=np.float64)
        name_to_pos = {
            robot.config.get_urdf_joint_name(name): position
            for name, position in zip(joint_state.name, joint_state.position, strict=True)
        }
        missing = [name for name in robot.config.joint_names if name not in name_to_pos]
        if missing:
            raise ValueError(f"JointState missing joints for RoboPlanWorld: {missing}")
        return np.asarray(
            [name_to_pos[name] for name in robot.config.joint_names], dtype=np.float64
        )

    def _require_finalized(self) -> None:
        if not self._finalized:
            raise RuntimeError("World must be finalized first")

    def _require_scene(self) -> None:
        if self._scene is None:
            raise RuntimeError("RoboPlan scene is not initialized; add a robot first")

    def _to_scene_q(self, robot_id: WorldRobotID, q: NDArray[np.float64]) -> NDArray[np.float64]:
        """Expand DimOS group positions to RoboPlan's full scene vector when available."""
        self._require_scene()
        robot = self._get_robot(robot_id)
        if len(q) != len(robot.config.joint_names):
            return q
        to_full = self._lookup_method(
            self._scene, ("toFullJointPositions", "to_full_joint_positions")
        )
        if to_full is None:
            return q
        try:
            return np.asarray(to_full(robot.config.name, q), dtype=np.float64)
        except TypeError:
            return q

    def _create_collision_context(self) -> Any:
        self._require_scene()
        collision_context_cls = getattr(self._core, "CollisionContext", None)
        if collision_context_cls is None:
            return None
        return collision_context_cls(self._scene)

    def _refresh_context_if_needed(self, ctx: RoboPlanContext) -> None:
        if ctx.geometry_revision == self._geometry_revision:
            return
        ctx.collision_context = self._create_collision_context()
        ctx.geometry_revision = self._geometry_revision

    def _bump_geometry_revision(self) -> None:
        self._geometry_revision += 1
        self._live_context.collision_context = None
        self._live_context.geometry_revision = -1

    def _has_collisions(
        self, robot_id: WorldRobotID, q: NDArray[np.float64], ctx: RoboPlanContext
    ) -> bool:
        self._require_scene()
        scene_q = self._to_scene_q(robot_id, q)
        for target, names in (
            (ctx.collision_context, ("hasCollisions", "has_collisions")),
            (self._scene, ("hasCollisions", "has_collisions")),
        ):
            if target is None:
                continue
            method = self._lookup_method(target, names)
            if method is None:
                continue
            try:
                return bool(method(scene_q))
            except TypeError:
                return bool(method(self._scene, scene_q))
        raise ValueError("RoboPlan collision checking is unavailable from installed bindings")

    def _lookup_path_collision_checker(self) -> Any:
        return self._lookup_method(
            self._core,
            ("hasCollisionsAlongPath", "has_collisions_along_path"),
        )

    def _call_path_collision_checker(
        self,
        path_check: Any,
        ctx: RoboPlanContext,
        robot_id: WorldRobotID,
        q_start: NDArray[np.float64],
        q_end: NDArray[np.float64],
        step_size: float,
    ) -> bool:
        self._require_scene()
        scene_q_start = self._to_scene_q(robot_id, q_start)
        scene_q_end = self._to_scene_q(robot_id, q_end)
        for args in (
            (
                self._scene,
                ctx.collision_context,
                scene_q_start,
                scene_q_end,
                step_size,
                False,
                True,
            ),
            (self._scene, ctx.collision_context, scene_q_start, scene_q_end, step_size),
            (self._scene, scene_q_start, scene_q_end, step_size, False, True),
            (self._scene, scene_q_start, scene_q_end, step_size),
        ):
            try:
                return bool(path_check(*args))
            except TypeError:
                continue
        raise ValueError("RoboPlan path collision checker signature is unsupported")

    def _add_obstacle_to_scene(self, obstacle: Obstacle, obstacle_id: str) -> Any:
        self._require_scene()
        matrix = pose_to_matrix(obstacle.pose)
        if obstacle.obstacle_type == ObstacleType.BOX:
            self._require_dimensions(obstacle, 3)
            return self._call_first_obstacle_method(
                ("addBoxGeometry", "add_box_geometry"), obstacle_id, obstacle.dimensions, matrix
            )
        if obstacle.obstacle_type == ObstacleType.SPHERE:
            self._require_dimensions(obstacle, 1)
            return self._call_first_obstacle_method(
                ("addSphereGeometry", "add_sphere_geometry"),
                obstacle_id,
                obstacle.dimensions,
                matrix,
            )
        if obstacle.obstacle_type == ObstacleType.CYLINDER:
            self._require_dimensions(obstacle, 2)
            return self._call_first_obstacle_method(
                ("addCylinderGeometry", "add_cylinder_geometry"),
                obstacle_id,
                obstacle.dimensions,
                matrix,
            )
        if obstacle.obstacle_type == ObstacleType.MESH:
            if not obstacle.mesh_path:
                raise ValueError("MESH obstacle requires mesh_path")
            return self._call_first_obstacle_method(
                ("addMeshGeometry", "add_mesh_geometry", "addGeometry", "add_geometry"),
                obstacle_id,
                (obstacle.mesh_path,),
                matrix,
            )
        raise ValueError(f"Unsupported obstacle type: {obstacle.obstacle_type}")

    def _call_first_obstacle_method(
        self,
        names: tuple[str, ...],
        obstacle_id: str,
        geometry_args: tuple[Any, ...],
        matrix: NDArray[np.float64],
    ) -> Any:
        method = self._lookup_method(self._scene, names)
        if method is None:
            raise ValueError(f"RoboPlan scene does not support obstacle method(s): {names}")
        color = np.asarray([1.0, 0.6, 0.2, 1.0], dtype=np.float64)
        geometry = self._make_geometry(names, geometry_args)
        for args in (
            (obstacle_id, "world", geometry, matrix, color),
            (obstacle_id, *geometry_args, matrix),
            (obstacle_id, geometry_args, matrix),
            (*geometry_args, matrix, obstacle_id),
        ):
            try:
                return method(*args)
            except TypeError:
                continue
        raise ValueError(f"RoboPlan obstacle method signature is unsupported: {names}")

    def _make_geometry(self, names: tuple[str, ...], geometry_args: tuple[Any, ...]) -> Any:
        if any("Box" in name for name in names):
            box = getattr(self._core, "Box", None)
            return box(*geometry_args) if box is not None else geometry_args
        if any("Sphere" in name for name in names):
            sphere = getattr(self._core, "Sphere", None)
            return sphere(*geometry_args) if sphere is not None else geometry_args
        if any("Cylinder" in name for name in names):
            cylinder = getattr(self._core, "Cylinder", None)
            if cylinder is None:
                return geometry_args
            radius, length = geometry_args
            return cylinder(radius, length)
        if any("Mesh" in name for name in names):
            mesh = getattr(self._core, "Mesh", None)
            return mesh(str(geometry_args[0])) if mesh is not None else geometry_args
        return geometry_args

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
    ) -> list[NDArray[np.float64]]:
        rrt_module = _import_roboplan_rrt()
        options_cls = getattr(rrt_module, "RRTOptions", None)
        options = options_cls() if options_cls is not None else None
        if options is not None:
            robot = self._get_robot(robot_id)
            for attr, value in (
                ("group_name", robot.config.name),
                ("timeout", timeout),
                ("max_time", timeout),
                ("max_planning_time", timeout),
                ("collision_check_use_bisection", False),
            ):
                if hasattr(options, attr):
                    setattr(options, attr, value)
        rrt_cls = getattr(rrt_module, "RRT", None)
        if rrt_cls is None:
            raise ValueError("roboplan.rrt does not expose RRT")
        self._require_scene()
        planner = rrt_cls(self._scene, options) if options is not None else rrt_cls(self._scene)
        start_config = self._to_native_joint_configuration(robot_id, q_start)
        goal_config = self._to_native_joint_configuration(robot_id, q_goal)
        result = planner.plan(start_config, goal_config)
        if result is None:
            raise ValueError("RoboPlan RRT returned no path")
        return self._extract_native_path(result)

    def _to_native_joint_configuration(self, robot_id: WorldRobotID, q: NDArray[np.float64]) -> Any:
        joint_config_cls = getattr(self._core, "JointConfiguration", None)
        if joint_config_cls is None:
            raise ValueError("roboplan.core does not expose JointConfiguration")
        robot = self._get_robot(robot_id)
        return joint_config_cls(robot.config.joint_names, np.asarray(q, dtype=np.float64))

    def _extract_native_path(self, result: Any) -> list[NDArray[np.float64]]:
        if result is None:
            raise ValueError("RoboPlan RRT returned no path")
        if isinstance(result, (list, tuple)):
            return [np.asarray(q, dtype=np.float64) for q in result]
        for attr in ("positions", "path", "joint_path", "waypoints"):
            value = getattr(result, attr, None)
            if value is not None:
                return [np.asarray(q, dtype=np.float64) for q in value]
        raise ValueError("RoboPlan RRT result does not expose a path")

    @staticmethod
    def _lookup_method(target: Any, names: tuple[str, ...]) -> Any:
        for name in names:
            method = getattr(target, name, None)
            if method is not None:
                return method
        return None
