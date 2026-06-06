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

"""VAMP (Vector-Accelerated Motion Planning) planner implementing PlannerSpec.

VAMP uses SIMD-accelerated collision checking (CAPT) and FK internally,
achieving microsecond-level planning times. Unlike RRTConnectPlanner which
delegates collision checking to WorldSpec, VampPlanner uses VAMP's own
collision engine during planning. Obstacles are pulled FROM WorldSpec and
converted to VAMP format to keep the world state synchronized.

Supported robots (pre-compiled in VAMP):
    - panda: Franka Emika Panda (7 DOF)
    - ur5: Universal Robots UR5 (6 DOF)
    - fetch: Fetch Mobile Manipulator
    - baxter: Rethink Robotics Baxter

Requires: pip install vamp-planner
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import PlanningResult, WorldRobotID
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.utils.path_utils import compute_path_length
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.manipulation.planning.spec.models import Obstacle

logger = setup_logger()

try:
    import vamp

    _VAMP_AVAILABLE = True
except ImportError:
    _VAMP_AVAILABLE = False

# Mapping from common robot names to VAMP robot module names.
# Keys are lowercase patterns that may appear in RobotModelConfig.name or URDF path.
VAMP_ROBOT_MAP: dict[str, str] = {
    "panda": "panda",
    "franka": "panda",
    "ur5": "ur5",
    "fetch": "fetch",
    "baxter": "baxter",
}


def _get_vamp_robot_module(robot_name: str) -> Any:
    """Resolve a dimos robot name to a VAMP robot submodule.

    Args:
        robot_name: The robot name to resolve. Matched case-insensitively
            against known VAMP robot names.

    Returns:
        The VAMP robot submodule (e.g., vamp.panda).

    Raises:
        ValueError: If the robot name cannot be mapped to a VAMP robot.
    """
    name_lower = robot_name.lower()
    for pattern, vamp_name in VAMP_ROBOT_MAP.items():
        if pattern in name_lower:
            return getattr(vamp, vamp_name)

    available = ", ".join(sorted(VAMP_ROBOT_MAP.keys()))
    raise ValueError(
        f"Cannot map robot '{robot_name}' to a VAMP robot. "
        f"Known robots: [{available}]. "
        f"Set vamp_robot_name explicitly in VampPlanner constructor."
    )


def _obstacle_to_vamp(obstacle: Obstacle) -> list[Any]:
    """Convert a dimos Obstacle to VAMP geometry objects.

    Returns a list of (geometry, add_method_name) tuples since VAMP uses
    different add methods for different geometry types.
    """
    from dimos.manipulation.planning.spec.enums import ObstacleType

    pose = obstacle.pose
    position = [pose.position.x, pose.position.y, pose.position.z]

    # Convert quaternion to Euler XYZ for VAMP cuboid/cylinder constructors.
    # VAMP uses [roll, pitch, yaw] Euler angles.
    q = pose.orientation
    euler = _quaternion_to_euler_xyz(q.x, q.y, q.z, q.w)

    results = []
    if obstacle.obstacle_type == ObstacleType.SPHERE:
        radius = obstacle.dimensions[0] if obstacle.dimensions else 0.05
        s = vamp.Sphere(position, radius)
        s.name = obstacle.name
        results.append((s, "add_sphere"))

    elif obstacle.obstacle_type == ObstacleType.BOX:
        # dimensions are (width, height, depth) -> half extents for VAMP
        dims = obstacle.dimensions if obstacle.dimensions else (0.1, 0.1, 0.1)
        half_extents = [d / 2.0 for d in dims]
        b = vamp.Cuboid(position, euler, half_extents)
        b.name = obstacle.name
        results.append((b, "add_cuboid"))

    elif obstacle.obstacle_type == ObstacleType.CYLINDER:
        radius = obstacle.dimensions[0] if len(obstacle.dimensions) > 0 else 0.05
        height = obstacle.dimensions[1] if len(obstacle.dimensions) > 1 else 0.2
        c = vamp.Cylinder(position, euler, radius, height)
        c.name = obstacle.name
        results.append((c, "add_capsule"))

    else:
        logger.warning(
            "VAMP does not support obstacle type %s for '%s', skipping",
            obstacle.obstacle_type,
            obstacle.name,
        )

    return results


def _quaternion_to_euler_xyz(x: float, y: float, z: float, w: float) -> list[float]:
    """Convert quaternion (x, y, z, w) to Euler XYZ (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return [float(roll), float(pitch), float(yaw)]


class VampPlanner:
    """SIMD-accelerated motion planner using VAMP.

    VAMP achieves microsecond-level planning by using its own internal
    SIMD-accelerated collision checking (CAPT) and FK. This planner
    conforms to PlannerSpec but uses VAMP internally for the actual
    planning, pulling obstacle data from WorldSpec.

    Supported algorithms: rrtc (default), prm, fcit, aorrtc.

    Args:
        vamp_robot_name: Explicit VAMP robot name (e.g., "panda").
            If None, auto-detected from RobotModelConfig.name.
        algorithm: Planning algorithm. One of "rrtc", "prm", "fcit", "aorrtc".
        rrt_range: Step size for RRT-Connect (default: 1.0).
        max_iterations: Maximum planner iterations.
        simplify: Whether to simplify the planned path.
        simplify_operations: Simplification routines to apply.
    """

    def __init__(
        self,
        vamp_robot_name: str | None = None,
        algorithm: str = "rrtc",
        rrt_range: float = 1.0,
        max_iterations: int = 100_000,
        simplify: bool = True,
        simplify_operations: list[str] | None = None,
    ):
        if not _VAMP_AVAILABLE:
            raise ImportError(
                "vamp-planner is not installed. Install with: pip install vamp-planner"
            )

        self._vamp_robot_name = vamp_robot_name
        self._algorithm = algorithm
        self._rrt_range = rrt_range
        self._max_iterations = max_iterations
        self._simplify = simplify
        self._simplify_operations = simplify_operations or ["SHORTCUT", "BSPLINE"]

        # Cached per-robot module (resolved on first plan call)
        self._robot_module: Any = None
        self._resolved_robot_name: str | None = vamp_robot_name

    def plan_joint_path(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        start: JointState,
        goal: JointState,
        timeout: float = 10.0,
    ) -> PlanningResult:
        """Plan a collision-free joint-space path using VAMP.

        Obstacles are pulled from WorldSpec and converted to VAMP format.
        VAMP's internal SIMD collision checking is used during planning.
        """
        start_time = time.time()

        # Resolve robot module
        robot_module = self._resolve_robot_module(world, robot_id)

        # Build VAMP environment with obstacles from WorldSpec
        env = self._build_vamp_environment(world)

        # Convert JointState to numpy arrays
        q_start = list(start.position)
        q_goal = list(goal.position)
        joint_names = start.name

        # Validate start/goal with VAMP
        if not robot_module.validate(q_start, env):
            return PlanningResult(
                status=PlanningStatus.COLLISION_AT_START,
                message="Start configuration is in collision (VAMP validation)",
                planning_time=time.time() - start_time,
            )

        if not robot_module.validate(q_goal, env):
            return PlanningResult(
                status=PlanningStatus.COLLISION_AT_GOAL,
                message="Goal configuration is in collision (VAMP validation)",
                planning_time=time.time() - start_time,
            )

        # Configure planner and plan
        try:
            result = self._run_planner(robot_module, q_start, q_goal, env)
        except Exception:
            logger.exception("VAMP planning failed")
            return PlanningResult(
                status=PlanningStatus.NO_SOLUTION,
                message="VAMP planning raised an exception",
                planning_time=time.time() - start_time,
            )

        planning_time = time.time() - start_time

        if not result.solved:
            status = (
                PlanningStatus.TIMEOUT if planning_time >= timeout else PlanningStatus.NO_SOLUTION
            )
            return PlanningResult(
                status=status,
                message=f"VAMP {self._algorithm} failed after {result.iterations} iterations",
                planning_time=planning_time,
                iterations=result.iterations,
            )

        # Optionally simplify
        path_obj = result.path
        if self._simplify:
            try:
                path_obj = self._simplify_path(robot_module, path_obj, env)
            except Exception:
                logger.warning("VAMP path simplification failed, using raw path")

        # Convert VAMP path to JointState list
        path_np = path_obj.numpy()  # (n_waypoints, n_dof)
        joint_path = [
            JointState(name=joint_names, position=path_np[i].tolist())
            for i in range(path_np.shape[0])
        ]

        return PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=joint_path,
            planning_time=planning_time,
            path_length=compute_path_length(joint_path),
            iterations=result.iterations,
            message=f"VAMP {self._algorithm} found path ({result.nanoseconds / 1e6:.2f} ms)",
        )

    def get_name(self) -> str:
        """Get planner name."""
        robot = self._resolved_robot_name or "auto"
        return f"VAMP-{self._algorithm.upper()}({robot})"

    def _resolve_robot_module(self, world: WorldSpec, robot_id: WorldRobotID) -> Any:
        """Resolve the VAMP robot module, caching it for future calls."""
        if self._robot_module is not None:
            return self._robot_module

        if self._vamp_robot_name:
            self._robot_module = getattr(vamp, self._vamp_robot_name)
            self._resolved_robot_name = self._vamp_robot_name
        else:
            config = world.get_robot_config(robot_id)
            self._robot_module = _get_vamp_robot_module(config.name)
            # Extract the resolved name for logging
            name_lower = config.name.lower()
            for pattern, vamp_name in VAMP_ROBOT_MAP.items():
                if pattern in name_lower:
                    self._resolved_robot_name = vamp_name
                    break

        logger.info("VAMP resolved robot module: %s", self._resolved_robot_name)
        return self._robot_module

    def _build_vamp_environment(self, world: WorldSpec) -> Any:
        """Build a VAMP Environment from WorldSpec obstacles.

        Pulls all obstacles from the world and converts them to VAMP
        geometry objects (Sphere, Cuboid, Cylinder).
        """
        env = vamp.Environment()

        # WorldSpec doesn't expose a list-all-obstacles method directly.
        # We use get_live_context to introspect obstacles if available.
        # The obstacle sync is handled by the caller adding obstacles
        # to WorldSpec before planning.
        if hasattr(world, "_obstacles"):
            # DrakeWorld stores obstacles in _obstacles dict
            for obs_data in world._obstacles.values():
                if hasattr(obs_data, "obstacle"):
                    self._add_obstacle_to_env(env, obs_data.obstacle)
        elif hasattr(world, "get_obstacles"):
            # Future WorldSpec implementations may expose this
            for obstacle in world.get_obstacles():
                self._add_obstacle_to_env(env, obstacle)

        return env

    def _add_obstacle_to_env(self, env: Any, obstacle: Obstacle) -> None:
        """Add a single dimos Obstacle to a VAMP Environment."""
        try:
            geom_pairs = _obstacle_to_vamp(obstacle)
            for geom, method_name in geom_pairs:
                getattr(env, method_name)(geom)
        except Exception:
            logger.warning("Failed to convert obstacle '%s' to VAMP format", obstacle.name)

    def _run_planner(
        self, robot_module: Any, start: list[float], goal: list[float], env: Any
    ) -> Any:
        """Run the configured VAMP planning algorithm."""
        rng = robot_module.halton()

        if self._algorithm == "rrtc":
            settings = vamp.RRTCSettings()
            settings.range = self._rrt_range
            settings.max_iterations = self._max_iterations
            settings.max_samples = self._max_iterations
            return robot_module.rrtc(start, goal, env, settings, rng)

        elif self._algorithm == "prm":
            neighbor_params = vamp.PRMNeighborParams(
                robot_module.dimension(), robot_module.space_measure()
            )
            settings = vamp.PRMSettings(neighbor_params)
            settings.max_iterations = self._max_iterations
            settings.max_samples = self._max_iterations
            return robot_module.prm(start, goal, env, settings, rng)

        elif self._algorithm == "fcit":
            neighbor_params = vamp.FCITNeighborParams(
                robot_module.dimension(), robot_module.space_measure()
            )
            settings = vamp.FCITSettings(neighbor_params)
            settings.max_iterations = self._max_iterations
            settings.max_samples = self._max_iterations
            return robot_module.fcit(start, goal, env, settings, rng)

        elif self._algorithm == "aorrtc":
            settings = vamp.AORRTCSettings()
            settings.rrtc = vamp.RRTCSettings()
            settings.rrtc.range = self._rrt_range
            settings.max_iterations = self._max_iterations
            settings.max_samples = self._max_iterations
            return robot_module.aorrtc(start, goal, env, settings, rng)

        else:
            raise ValueError(
                f"Unknown VAMP algorithm: {self._algorithm}. "
                f"Available: ['rrtc', 'prm', 'fcit', 'aorrtc']"
            )

    def _simplify_path(self, robot_module: Any, path: Any, env: Any) -> Any:
        """Simplify a VAMP path using configured simplification routines."""
        simp_settings = vamp.SimplifySettings()

        routine_map = {
            "SHORTCUT": vamp.SimplifyRoutine.SHORTCUT,
            "BSPLINE": vamp.SimplifyRoutine.BSPLINE,
            "REDUCE": vamp.SimplifyRoutine.REDUCE,
            "PERTURB": vamp.SimplifyRoutine.PERTURB,
        }
        simp_settings.operations = [
            routine_map[op.upper()] for op in self._simplify_operations if op.upper() in routine_map
        ]

        rng = robot_module.halton()
        result = robot_module.simplify(path, env, simp_settings, rng)
        return result.path
