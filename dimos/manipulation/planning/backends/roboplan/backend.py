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

from __future__ import annotations

from collections.abc import Sequence
from itertools import pairwise
import time
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.planning.backends.base import (
    BackendCapabilities,
    BackendDiagnostics,
    BackendRobot,
    PlannedMotion,
    SceneUpdateResult,
)
from dimos.manipulation.planning.backends.roboplan.config import (
    RoboPlanBackendConfig,
    extract_roboplan_options,
    parse_backend_config,
)
from dimos.manipulation.planning.backends.roboplan.conversion import (
    dimos_path_from_roboplan,
    dimos_trajectory_from_roboplan,
    joint_state_to_positions,
    make_joint_state,
    pose_from_transform,
    transform_from_pose,
)
from dimos.manipulation.planning.backends.roboplan.imports import (
    RoboPlanBackendUnavailableError,
    RoboPlanComponents,
    load_roboplan_components,
    probe_optional_toppra,
)
from dimos.manipulation.planning.spec.enums import IKStatus, ObstacleType, PlanningStatus
from dimos.manipulation.planning.spec.models import IKResult, PlanningResult, WorldRobotID
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import (
        CollisionObjectMessage,
        JointPath,
        Obstacle,
    )
    from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
        JointTrajectoryGenerator,
    )
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.vision_msgs.Detection3D import Detection3D
    from dimos.perception.detection.type.detection3d.object import Object


class RoboPlanPlanningBackend:
    name = "roboplan"

    def __init__(
        self,
        *,
        enable_viz: bool = False,
        planner_name: str = "rrt_connect",
        kinematics_name: str = "jacobian",
        options: dict[str, Any] | None = None,
    ) -> None:
        self._options = extract_roboplan_options(options)
        self._enable_viz = enable_viz
        self._planner_name = planner_name
        self._kinematics_name = kinematics_name
        self._diagnostics = BackendDiagnostics(backend_name=self.name)
        self._capabilities = BackendCapabilities(backend_name=self.name)
        self._components: RoboPlanComponents | None = None
        self._scene: Any | None = None
        self._rrt: Any | None = None
        self._ik: Any | None = None
        self._toppra: Any | None = None
        self._robot: BackendRobot | None = None
        self._config: RoboPlanBackendConfig | None = None
        self._current_state: dict[WorldRobotID, JointState] = {}
        self._current_velocities: dict[WorldRobotID, JointState] = {}
        self._state_time: dict[WorldRobotID, float] = {}
        self._obstacles: dict[str, Obstacle] = {}
        self._applied_obstacles: set[str] = set()
        self._cached_objects: list[Object] = []
        self._cached_detections: list[Detection3D] = []

    def scene(self) -> RoboPlanPlanningBackend:
        return self

    def planner(self) -> RoboPlanPlanningBackend:
        return self

    def visualization(self) -> None:
        return None

    def capabilities(self) -> BackendCapabilities:
        return self._capabilities

    def diagnostics(self) -> BackendDiagnostics:
        return self._diagnostics

    @property
    def world_monitor(self) -> None:
        return None

    @property
    def world(self) -> None:
        return None

    @property
    def legacy_planner(self) -> None:
        return None

    @property
    def legacy_kinematics(self) -> None:
        return None

    def add_robot(self, config: RobotModelConfig) -> BackendRobot:
        if self._robot is not None:
            raise ValueError(
                "RoboPlan backend currently supports one active robot/group per module"
            )
        backend_config = parse_backend_config(config, self._options)
        robot_id = f"roboplan_{config.name}"
        robot = BackendRobot(name=config.name, robot_id=robot_id, config=config)
        self._robot = robot
        self._config = backend_config
        self._diagnostics.add("robot_config", "configured", "RoboPlan robot config validated")
        return robot

    def finalize(self) -> None:
        robot = self._require_robot()
        config = self._require_config()
        require_toppra = config.retiming == "toppra"
        self._components = load_roboplan_components(require_toppra=require_toppra)
        self._scene = self._components.core.Scene(
            robot.config.name,
            str(config.model_path),
            str(config.srdf_path),
            [str(path) for path in config.package_paths],
        )

        self._rrt = self._components.rrt.RRT(self._scene, self._make_rrt_options())
        rrt = self._rrt
        if config.rrt.seed is not None and rrt is not None and hasattr(rrt, "setRngSeed"):
            rrt.setRngSeed(config.rrt.seed)
        self._ik = self._components.simple_ik.SimpleIk(self._scene, self._make_ik_options())
        if require_toppra:
            assert self._components.toppra is not None
            self._toppra = self._components.toppra.PathParameterizerTOPPRA(
                self._scene, config.planning_group
            )
        elif probe_optional_toppra() is None:
            self._diagnostics.add(
                "toppra",
                "unavailable",
                "RoboPlan TOPPRA module is not installed; retiming='dimos' remains available",
            )

        self._capabilities = BackendCapabilities(
            backend_name=self.name,
            joint_planning=True,
            pose_planning=True,
            inverse_kinematics=True,
            forward_kinematics=hasattr(self._scene, "forwardKinematics"),
            jacobian=hasattr(self._scene, "computeFrameJacobian"),
            collision_checking=hasattr(self._scene, "hasCollisions"),
            distance_query=False,
            primitive_obstacles=config.scene.primitive_obstacles,
            mesh_obstacles=config.scene.mesh_obstacles,
            pointcloud_layers=config.scene.pointcloud_layers,
            attached_objects=config.scene.attached_objects,
            visualization=False,
            path_preview=False,
            drake_native_access=False,
        )
        self._diagnostics.add("scene", "ready", "RoboPlan scene and planner initialized")

    def stop(self) -> None:
        self._scene = None
        self._rrt = None
        self._ik = None
        self._toppra = None

    def get_robot_ids(self) -> list[WorldRobotID]:
        return [self._robot.robot_id] if self._robot is not None else []

    def get_robot_config(self, robot_id: WorldRobotID) -> RobotModelConfig:
        robot = self._require_robot_id(robot_id)
        return robot.config

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        robot = self._require_robot_id(robot_id)
        lower = robot.config.joint_limits_lower
        upper = robot.config.joint_limits_upper
        if lower is None or upper is None:
            raise ValueError("RoboPlan backend requires configured joint limits")
        return np.asarray(lower, dtype=np.float64), np.asarray(upper, dtype=np.float64)

    def start_state_monitor(self, robot_id: WorldRobotID) -> None:
        self._require_robot_id(robot_id)

    def start_obstacle_monitor(self) -> None:
        return None

    def on_joint_state(self, msg: JointState, robot_id: WorldRobotID) -> None:
        robot = self._require_robot_id(robot_id)
        public_positions = joint_state_to_positions(msg, robot.config.get_coordinator_joint_names())
        public_state = make_joint_state(robot.config.joint_names, public_positions)
        public_state.velocity = list(msg.velocity)
        self._current_state[robot_id] = public_state
        if msg.velocity:
            velocity_state = make_joint_state(robot.config.joint_names, [])
            velocity_state.velocity = list(msg.velocity)
            self._current_velocities[robot_id] = velocity_state
        self._state_time[robot_id] = time.time()
        self._try_set_scene_positions(public_state)

    def on_collision_object(self, msg: CollisionObjectMessage) -> None:
        if msg.operation == "remove":
            self.remove_obstacle(msg.id)
            return
        if msg.pose is None or msg.primitive_type is None:
            self._diagnostics.add(
                "obstacle", "unsupported", "CollisionObjectMessage missing pose/type"
            )
            return
        obstacle_type = {
            "box": ObstacleType.BOX,
            "sphere": ObstacleType.SPHERE,
            "cylinder": ObstacleType.CYLINDER,
        }.get(msg.primitive_type)
        if obstacle_type is None:
            self._diagnostics.add(
                "obstacle", "unsupported", f"Unsupported obstacle type {msg.primitive_type}"
            )
            return
        from dimos.manipulation.planning.spec.models import Obstacle

        obstacle = Obstacle(
            name=msg.id,
            obstacle_type=obstacle_type,
            pose=msg.pose,
            dimensions=msg.dimensions or (),
            color=msg.color,
        )
        if msg.operation == "update":
            self.update_obstacle_pose(msg.id, msg.pose)
        else:
            self.add_obstacle(obstacle)

    def on_detections(self, detections: list[Detection3D]) -> None:
        self._cached_detections = list(detections)
        self._diagnostics.add(
            "perception", "cached", "Detections cached but not projected to RoboPlan"
        )

    def on_objects(self, objects: list[Object]) -> None:
        self._cached_objects = list(objects)
        self._diagnostics.add(
            "perception", "cached", "Objects cached but not projected to RoboPlan"
        )

    def refresh_obstacles(self, min_duration: float = 0.0) -> list[dict[str, Any]]:
        return self.list_added_obstacles()

    def clear_perception_obstacles(self) -> int:
        count = len(self._cached_objects) + len(self._cached_detections)
        self._cached_objects = []
        self._cached_detections = []
        return count

    def get_perception_status(self) -> dict[str, int]:
        return {"objects": len(self._cached_objects), "detections": len(self._cached_detections)}

    def get_cached_objects(self) -> list[Object]:
        return list(self._cached_objects)

    def list_cached_detections(self) -> list[dict[str, Any]]:
        return [
            {"index": idx, "backend": self.name} for idx, _ in enumerate(self._cached_detections)
        ]

    def list_added_obstacles(self) -> list[dict[str, Any]]:
        return [
            {
                "id": obstacle_id,
                "type": obstacle.obstacle_type.name.lower(),
                "applied": obstacle_id in self._applied_obstacles,
            }
            for obstacle_id, obstacle in self._obstacles.items()
        ]

    def get_current_joint_state(self, robot_id: WorldRobotID) -> JointState | None:
        return self._current_state.get(robot_id)

    def get_current_velocities(self, robot_id: WorldRobotID) -> JointState | None:
        return self._current_velocities.get(robot_id)

    def wait_for_state(self, robot_id: WorldRobotID, timeout: float = 1.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if robot_id in self._current_state:
                return True
            time.sleep(0.01)
        return robot_id in self._current_state

    def is_state_stale(self, robot_id: WorldRobotID, max_age: float = 1.0) -> bool:
        timestamp = self._state_time.get(robot_id)
        return timestamp is None or time.time() - timestamp > max_age

    def add_obstacle(self, obstacle: Obstacle) -> str:
        self._obstacles[obstacle.name] = obstacle
        result = self._apply_obstacle(obstacle)
        if result.applied:
            self._applied_obstacles.add(obstacle.name)
        return obstacle.name

    def remove_obstacle(self, obstacle_id: str) -> bool:
        self._obstacles.pop(obstacle_id, None)
        self._applied_obstacles.discard(obstacle_id)
        if self._scene is not None and hasattr(self._scene, "removeGeometry"):
            try:
                self._scene.removeGeometry(obstacle_id)
            except RuntimeError:
                return False
        return True

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> SceneUpdateResult:
        obstacle = self._obstacles.get(obstacle_id)
        if obstacle is None:
            return SceneUpdateResult(
                False, "missing", f"Obstacle '{obstacle_id}' not found", obstacle_id
            )
        obstacle.pose = pose
        if obstacle_id not in self._applied_obstacles:
            return SceneUpdateResult(
                False,
                "unsupported",
                "Obstacle is cached but not applied to RoboPlan collision geometry",
                obstacle_id,
            )
        if self._scene is None or not hasattr(self._scene, "updateGeometryPlacement"):
            return SceneUpdateResult(
                False, "unsupported", "RoboPlan geometry update unavailable", obstacle_id
            )
        parent_frame = pose.frame_id or self._require_config().base_frame
        self._scene.updateGeometryPlacement(obstacle_id, parent_frame, transform_from_pose(pose))
        return SceneUpdateResult(
            True, "applied_live", "RoboPlan geometry pose updated", obstacle_id
        )

    def clear_obstacles(self) -> None:
        for obstacle_id in list(self._obstacles):
            self.remove_obstacle(obstacle_id)

    def get_obstacles(self) -> list[Obstacle]:
        return list(self._obstacles.values())

    def is_state_valid(self, robot_id: WorldRobotID, joint_state: JointState) -> bool:
        self._require_robot_id(robot_id)
        if self._scene is None:
            return False
        positions = self._full_positions_from_state(joint_state)
        return bool(self._scene.isValidConfiguration(positions)) and not bool(
            self._scene.hasCollisions(positions)
        )

    def is_path_valid(
        self, robot_id: WorldRobotID, path: JointPath, step_size: float = 0.05
    ) -> bool:
        self._require_robot_id(robot_id)
        if self._scene is None or self._components is None:
            return False
        if len(path) < 2:
            return True
        for start, end in pairwise(path):
            start_positions = self._full_positions_from_state(start)
            end_positions = self._full_positions_from_state(end)
            if self._components.core.hasCollisionsAlongPath(
                self._scene, start_positions, end_positions, step_size, False, True
            ):
                return False
        return True

    def get_min_distance(self, robot_id: WorldRobotID) -> None:
        self._require_robot_id(robot_id)
        self._diagnostics.add(
            "distance_query", "unsupported", "RoboPlan distance query is not wired"
        )
        return None

    def get_ee_pose(
        self, robot_id: WorldRobotID, joint_state: JointState | None = None
    ) -> PoseStamped | None:
        return self.get_link_pose(robot_id, self._require_config().end_effector_frame, joint_state)

    def get_link_pose(
        self, robot_id: WorldRobotID, link_name: str, joint_state: JointState | None = None
    ) -> PoseStamped | None:
        self._require_robot_id(robot_id)
        if self._scene is None or not hasattr(self._scene, "forwardKinematics"):
            self._diagnostics.add("forward_kinematics", "unsupported", "RoboPlan FK unavailable")
            return None
        state = joint_state or self.get_current_joint_state(robot_id)
        if state is None:
            return None
        transform = self._scene.forwardKinematics(
            self._full_positions_from_state(state),
            link_name,
            self._require_config().base_frame,
        )
        return pose_from_transform(transform, frame_id=self._require_config().base_frame)

    def get_jacobian(
        self, robot_id: WorldRobotID, joint_state: JointState
    ) -> NDArray[np.float64] | None:
        self._require_robot_id(robot_id)
        if self._scene is None or not hasattr(self._scene, "computeFrameJacobian"):
            self._diagnostics.add("jacobian", "unsupported", "RoboPlan Jacobian unavailable")
            return None
        jacobian = self._scene.computeFrameJacobian(
            self._full_positions_from_state(joint_state),
            self._require_config().end_effector_frame,
            True,
        )
        return np.asarray(jacobian, dtype=np.float64)

    def plan_to_joints(
        self,
        robot_id: WorldRobotID,
        goal: JointState,
        timeout: float,
        trajectory_generator: JointTrajectoryGenerator,
    ) -> PlannedMotion | PlanningResult:
        robot = self._require_robot_id(robot_id)
        start = self.get_current_joint_state(robot_id)
        if start is None:
            return _planning_failure("No current joint state for RoboPlan planning")
        if self.is_state_stale(robot_id, max_age=max(1.0, timeout)):
            return _planning_failure("Current joint state is stale for RoboPlan planning")
        if self._components is None or self._rrt is None:
            return _planning_failure("RoboPlan backend is not finalized")
        try:
            start_config = self._make_joint_config(start)
            goal_config = self._make_joint_config(goal)
            if np.allclose(
                self._active_positions_from_state(start),
                self._active_positions_from_state(goal),
                atol=1e-9,
            ):
                path = [
                    make_joint_state(robot.config.joint_names, list(start.position)),
                    make_joint_state(robot.config.joint_names, list(goal.position)),
                ]
                trajectory = trajectory_generator.generate([list(state.position) for state in path])
                result = PlanningResult(
                    status=PlanningStatus.SUCCESS,
                    path=path,
                    path_length=0.0,
                    message="RoboPlan joint planning skipped because start equals goal",
                )
                return PlannedMotion(path=path, trajectory=trajectory, planning_result=result)
            native_path = self._rrt.plan(start_config, goal_config)
            path = dimos_path_from_roboplan(
                native_path,
                robot.config.joint_names,
                self._require_config().active_joint_names,
                start.position,
            )
            if len(path) < 2:
                return _planning_failure("RoboPlan did not return a usable path")
            if not self.is_path_valid(
                robot_id, path, self._require_config().rrt.collision_check_step_size
            ):
                return PlanningResult(
                    status=PlanningStatus.COLLISION_AT_GOAL,
                    path=[],
                    message="RoboPlan returned an invalid path",
                )
            trajectory = self._retime(native_path, path, trajectory_generator)
            result = PlanningResult(
                status=PlanningStatus.SUCCESS,
                path=path,
                path_length=_path_length(path),
                message="RoboPlan joint planning succeeded",
            )
            return PlannedMotion(path=path, trajectory=trajectory, planning_result=result)
        except Exception as exc:
            self._diagnostics.add("joint_planning", "failed", str(exc))
            return _planning_failure(f"RoboPlan joint planning failed: {exc}")

    def plan_to_pose(
        self,
        robot_id: WorldRobotID,
        target_pose: PoseStamped,
        timeout: float,
        trajectory_generator: JointTrajectoryGenerator,
    ) -> PlannedMotion | IKResult | PlanningResult:
        start = self.get_current_joint_state(robot_id)
        if start is None:
            return _planning_failure("No current joint state for RoboPlan pose planning")
        if self._components is None or self._ik is None:
            return IKResult(IKStatus.NO_SOLUTION, message="RoboPlan IK is not initialized")
        normalized_target = self._target_pose_for_roboplan(target_pose)
        if normalized_target is None:
            return IKResult(
                IKStatus.NO_SOLUTION,
                message=(
                    f"RoboPlan target frame does not match configured base frame "
                    f"'{self._require_config().base_frame}'"
                ),
            )
        target_pose = normalized_target
        solution = self._components.core.JointConfiguration()
        goal = self._components.core.CartesianConfiguration(
            self._require_config().base_frame,
            self._require_config().end_effector_frame,
            transform_from_pose(target_pose),
        )
        try:
            ok = bool(self._ik.solveIk(goal, self._make_joint_config(start), solution))
        except Exception as exc:
            self._diagnostics.add("inverse_kinematics", "failed", str(exc))
            return IKResult(IKStatus.NO_SOLUTION, message=f"RoboPlan IK failed: {exc}")
        if not ok:
            return IKResult(IKStatus.NO_SOLUTION, message="RoboPlan IK found no solution")
        solution_names = list(solution.joint_names) or list(
            self._require_config().active_joint_names
        )
        ik_state = make_joint_state(solution_names, solution.positions)
        result = self.plan_to_joints(robot_id, ik_state, timeout, trajectory_generator)
        ik_result = IKResult(
            IKStatus.SUCCESS, joint_state=ik_state, message="RoboPlan IK succeeded"
        )
        if isinstance(result, PlannedMotion):
            return PlannedMotion(result.path, result.trajectory, result.planning_result, ik_result)
        return result

    def validate_path(
        self, robot_id: WorldRobotID, path: JointPath, step_size: float = 0.05
    ) -> bool:
        return self.is_path_valid(robot_id, path, step_size)

    def _make_rrt_options(self) -> Any:
        assert self._components is not None
        config = self._require_config()
        rrt = config.rrt
        return self._components.rrt.RRTOptions(
            group_name=config.planning_group,
            max_nodes=rrt.max_nodes,
            max_connection_distance=rrt.max_connection_distance,
            collision_check_step_size=rrt.collision_check_step_size,
            collision_check_use_bisection=rrt.collision_check_use_bisection,
            goal_biasing_probability=rrt.goal_biasing_probability,
            max_planning_time=rrt.max_planning_time,
            rrt_connect=rrt.rrt_connect,
        )

    def _make_ik_options(self) -> Any:
        assert self._components is not None
        config = self._require_config()
        ik = config.ik
        return self._components.simple_ik.SimpleIkOptions(
            group_name=config.planning_group,
            max_iters=ik.max_iters,
            max_time=ik.max_time,
            max_restarts=ik.max_restarts,
            step_size=ik.step_size,
            damping=ik.damping,
            max_linear_error_norm=ik.max_linear_error_norm,
            max_angular_error_norm=ik.max_angular_error_norm,
            check_collisions=ik.check_collisions,
            fast_return=ik.fast_return,
        )

    def _make_joint_config(self, state: JointState) -> Any:
        assert self._components is not None
        config = self._require_config()
        return self._components.core.JointConfiguration(
            config.active_joint_names,
            np.asarray(self._active_positions_from_state(state), dtype=np.float64),
        )

    def _active_positions_from_state(self, state: JointState) -> list[float]:
        if not state.name:
            state = make_joint_state(self._require_robot().config.joint_names, state.position)
        return joint_state_to_positions(state, self._require_config().active_joint_names)

    def _full_positions_from_state(self, state: JointState) -> NDArray[np.float64]:
        active_positions = np.asarray(self._active_positions_from_state(state), dtype=np.float64)
        if self._scene is not None and hasattr(self._scene, "toFullJointPositions"):
            full_positions = self._scene.toFullJointPositions(
                self._require_config().planning_group, active_positions
            )
            return np.asarray(full_positions, dtype=np.float64)
        return active_positions

    def _retime(
        self,
        native_path: Any,
        path: JointPath,
        trajectory_generator: JointTrajectoryGenerator,
    ) -> Any:
        config = self._require_config()
        if config.retiming == "none":
            return trajectory_generator.generate([list(state.position) for state in path])
        if config.retiming == "toppra":
            if self._toppra is None or self._components is None or self._components.toppra is None:
                raise RoboPlanBackendUnavailableError(
                    "RoboPlan TOPPRA retiming selected but unavailable"
                )
            mode = getattr(self._components.toppra.SplineFittingMode, config.toppra.mode)
            native_traj = self._toppra.generate(
                native_path,
                config.toppra.dt,
                mode,
                config.toppra.velocity_scale,
                config.toppra.acceleration_scale,
                config.toppra.max_adaptive_iterations,
                config.toppra.max_adaptive_step_size,
            )
            return dimos_trajectory_from_roboplan(
                native_traj, self._require_robot().config.joint_names
            )
        return trajectory_generator.generate([list(state.position) for state in path])

    def _target_pose_for_roboplan(self, target_pose: PoseStamped) -> PoseStamped | None:
        config = self._require_config()
        frame_id = target_pose.frame_id or config.base_frame
        if frame_id == config.base_frame:
            return target_pose
        if frame_id != "world":
            return None
        base_pose = self._require_robot().config.base_pose
        if base_pose.frame_id and base_pose.frame_id != "world":
            return None
        target_in_base = np.linalg.inv(transform_from_pose(base_pose)) @ transform_from_pose(
            target_pose
        )
        return pose_from_transform(target_in_base, frame_id=config.base_frame)

    def _apply_obstacle(self, obstacle: Obstacle) -> SceneUpdateResult:
        config = self._require_config()
        if self._scene is None or self._components is None:
            return SceneUpdateResult(
                False, "cached", "Obstacle cached before RoboPlan finalize", obstacle.name
            )
        if not config.scene.primitive_obstacles:
            return SceneUpdateResult(
                False, "unsupported", "Primitive obstacles disabled", obstacle.name
            )
        parent_frame = obstacle.pose.frame_id or config.base_frame
        color = np.asarray(obstacle.color, dtype=np.float64)
        transform = transform_from_pose(obstacle.pose)
        try:
            if obstacle.obstacle_type == ObstacleType.BOX and len(obstacle.dimensions) == 3:
                self._scene.addBoxGeometry(
                    obstacle.name,
                    parent_frame,
                    self._components.core.Box(*obstacle.dimensions),
                    transform,
                    color,
                )
                return SceneUpdateResult(
                    True, "applied", "RoboPlan box obstacle applied", obstacle.name
                )
            if obstacle.obstacle_type == ObstacleType.SPHERE and len(obstacle.dimensions) == 1:
                self._scene.addSphereGeometry(
                    obstacle.name,
                    parent_frame,
                    self._components.core.Sphere(obstacle.dimensions[0]),
                    transform,
                    color,
                )
                return SceneUpdateResult(
                    True, "applied", "RoboPlan sphere obstacle applied", obstacle.name
                )
            if obstacle.obstacle_type == ObstacleType.CYLINDER and len(obstacle.dimensions) == 2:
                self._scene.addCylinderGeometry(
                    obstacle.name,
                    parent_frame,
                    self._components.core.Cylinder(obstacle.dimensions[0], obstacle.dimensions[1]),
                    transform,
                    color,
                )
                return SceneUpdateResult(
                    True, "applied", "RoboPlan cylinder obstacle applied", obstacle.name
                )
        except Exception as exc:
            self._diagnostics.add("obstacle", "failed", str(exc), obstacle_id=obstacle.name)
            return SceneUpdateResult(False, "failed", str(exc), obstacle.name)
        return SceneUpdateResult(
            False,
            "unsupported",
            f"RoboPlan obstacle type {obstacle.obstacle_type.name} is not enabled or dimensioned",
            obstacle.name,
        )

    def _try_set_scene_positions(self, state: JointState) -> None:
        if self._scene is None:
            return
        try:
            self._scene.setJointPositions(self._full_positions_from_state(state))
        except Exception as exc:
            self._diagnostics.add("state", "partial", f"Could not set RoboPlan scene state: {exc}")

    def _require_robot(self) -> BackendRobot:
        if self._robot is None:
            raise ValueError("RoboPlan backend has no registered robot")
        return self._robot

    def _require_config(self) -> RoboPlanBackendConfig:
        if self._config is None:
            raise ValueError("RoboPlan backend has no parsed config")
        return self._config

    def _require_robot_id(self, robot_id: WorldRobotID) -> BackendRobot:
        robot = self._require_robot()
        if robot.robot_id != robot_id:
            raise ValueError(f"Unknown RoboPlan robot_id: {robot_id}")
        return robot


def _planning_failure(message: str) -> PlanningResult:
    return PlanningResult(status=PlanningStatus.NO_SOLUTION, message=message)


def _path_length(path: Sequence[JointState]) -> float:
    total = 0.0
    for start, end in pairwise(path):
        total += float(np.linalg.norm(np.asarray(end.position) - np.asarray(start.position)))
    return total
