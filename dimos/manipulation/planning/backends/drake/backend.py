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

"""Drake-backed active manipulation planning backend."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from dimos.manipulation.planning.backends.base import (
    BackendCapabilities,
    BackendDiagnostics,
    BackendRobot,
    PlannedMotion,
    SceneUpdateResult,
)
from dimos.manipulation.planning.factory import create_kinematics, create_planner
from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
from dimos.manipulation.planning.spec.models import IKResult, PlanningResult, WorldRobotID
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import (
        CollisionObjectMessage,
        JointPath,
        Obstacle,
    )
    from dimos.manipulation.planning.spec.protocols import KinematicsSpec, PlannerSpec, WorldSpec
    from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
        JointTrajectoryGenerator,
    )
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.vision_msgs.Detection3D import Detection3D
    from dimos.perception.detection.type.detection3d.object import Object

logger = setup_logger()


class DrakePlanningBackend:
    """Compatibility backend wrapping the existing Drake WorldMonitor planning stack."""

    name = "drake"

    def __init__(
        self,
        *,
        planner_name: str = "rrt_connect",
        kinematics_name: str = "jacobian",
        options: dict[str, Any] | None = None,
    ) -> None:
        self._options = options or {}
        world_kwargs = dict(self._options.get("world", {}))
        self._world_monitor = WorldMonitor(
            backend="drake",
            enable_viz=False,
            **world_kwargs,
        )
        self._planner = create_planner(name=planner_name, **self._options.get("planner", {}))
        self._kinematics = create_kinematics(
            name=kinematics_name,
            **self._options.get("kinematics", {}),
        )
        self._capabilities = BackendCapabilities(
            backend_name=self.name,
            joint_planning=True,
            pose_planning=True,
            inverse_kinematics=True,
            forward_kinematics=True,
            jacobian=True,
            collision_checking=True,
            distance_query=True,
            primitive_obstacles=True,
            mesh_obstacles=True,
            drake_native_access=True,
        )
        self._diagnostics = BackendDiagnostics(backend_name=self.name)

    def scene(self) -> DrakePlanningBackend:
        return self

    def planner(self) -> DrakePlanningBackend:
        return self

    def capabilities(self) -> BackendCapabilities:
        return self._capabilities

    def diagnostics(self) -> BackendDiagnostics:
        return self._diagnostics

    @property
    def world_monitor(self) -> WorldMonitor:
        return self._world_monitor

    @property
    def world(self) -> WorldSpec:
        return self._world_monitor.world

    @property
    def legacy_planner(self) -> PlannerSpec:
        return self._planner

    @property
    def legacy_kinematics(self) -> KinematicsSpec:
        return self._kinematics

    def add_robot(self, config: RobotModelConfig) -> BackendRobot:
        robot_id = self._world_monitor.add_robot(config)
        return BackendRobot(name=config.name, robot_id=robot_id, config=config)

    def finalize(self) -> None:
        self._world_monitor.finalize()

    def stop(self) -> None:
        self._world_monitor.stop_all_monitors()

    def get_robot_ids(self) -> list[WorldRobotID]:
        return self._world_monitor.get_robot_ids()

    def get_robot_config(self, robot_id: WorldRobotID) -> RobotModelConfig:
        return self._world_monitor.get_robot_config(robot_id)

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return self._world_monitor.get_joint_limits(robot_id)

    def start_state_monitor(self, robot_id: WorldRobotID) -> None:
        self._world_monitor.start_state_monitor(robot_id)

    def start_obstacle_monitor(self) -> None:
        self._world_monitor.start_obstacle_monitor()

    def on_joint_state(self, msg: JointState, robot_id: WorldRobotID) -> None:
        self._world_monitor.on_joint_state(msg, robot_id=robot_id)

    def on_collision_object(self, msg: CollisionObjectMessage) -> None:
        self._world_monitor.on_collision_object(msg)

    def on_detections(self, detections: list[Detection3D]) -> None:
        self._world_monitor.on_detections(detections)

    def on_objects(self, objects: list[Object]) -> None:
        self._world_monitor.on_objects(objects)

    def refresh_obstacles(self, min_duration: float = 0.0) -> list[dict[str, Any]]:
        return self._world_monitor.refresh_obstacles(min_duration)

    def clear_perception_obstacles(self) -> int:
        return self._world_monitor.clear_perception_obstacles()

    def get_perception_status(self) -> dict[str, int]:
        return self._world_monitor.get_perception_status()

    def get_cached_objects(self) -> list[Object]:
        return self._world_monitor.get_cached_objects()

    def list_cached_detections(self) -> list[dict[str, Any]]:
        return self._world_monitor.list_cached_detections()

    def list_added_obstacles(self) -> list[dict[str, Any]]:
        return self._world_monitor.list_added_obstacles()

    def get_current_joint_state(self, robot_id: WorldRobotID) -> JointState | None:
        return self._world_monitor.get_current_joint_state(robot_id)

    def get_current_velocities(self, robot_id: WorldRobotID) -> JointState | None:
        return self._world_monitor.get_current_velocities(robot_id)

    def wait_for_state(self, robot_id: WorldRobotID, timeout: float = 1.0) -> bool:
        return self._world_monitor.wait_for_state(robot_id, timeout)

    def is_state_stale(self, robot_id: WorldRobotID, max_age: float = 1.0) -> bool:
        return self._world_monitor.is_state_stale(robot_id, max_age)

    def add_obstacle(self, obstacle: Obstacle) -> str:
        return self._world_monitor.add_obstacle(obstacle)

    def remove_obstacle(self, obstacle_id: str) -> bool:
        return self._world_monitor.remove_obstacle(obstacle_id)

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> SceneUpdateResult:
        obstacles = self._world_monitor.world.get_obstacles()
        obstacle = next(
            (candidate for candidate in obstacles if candidate.name == obstacle_id), None
        )
        if obstacle is None:
            return SceneUpdateResult(
                applied=False,
                status="missing",
                message=f"Obstacle '{obstacle_id}' not found",
                obstacle_id=obstacle_id,
            )

        obstacle.pose = pose
        if self._world_monitor.remove_obstacle(obstacle_id):
            new_id = self._world_monitor.add_obstacle(obstacle)
            return SceneUpdateResult(
                applied=True,
                status="applied_live",
                message="Obstacle removed and re-added so Drake collision geometry uses the new pose",
                obstacle_id=new_id,
            )

        return SceneUpdateResult(
            applied=False,
            status="failed",
            message=f"Obstacle '{obstacle_id}' could not be removed for re-add",
            obstacle_id=obstacle_id,
        )

    def clear_obstacles(self) -> None:
        self._world_monitor.clear_obstacles()

    def get_obstacles(self) -> list[Obstacle]:
        return self._world_monitor.world.get_obstacles()

    def is_state_valid(self, robot_id: WorldRobotID, joint_state: JointState) -> bool:
        return self._world_monitor.is_state_valid(robot_id, joint_state)

    def is_path_valid(
        self, robot_id: WorldRobotID, path: JointPath, step_size: float = 0.05
    ) -> bool:
        return self._world_monitor.is_path_valid(robot_id, path, step_size)

    def get_min_distance(self, robot_id: WorldRobotID) -> float | None:
        return self._world_monitor.get_min_distance(robot_id)

    def get_ee_pose(
        self, robot_id: WorldRobotID, joint_state: JointState | None = None
    ) -> PoseStamped | None:
        return self._world_monitor.get_ee_pose(robot_id, joint_state=joint_state)

    def get_link_pose(
        self, robot_id: WorldRobotID, link_name: str, joint_state: JointState | None = None
    ) -> PoseStamped | None:
        return self._world_monitor.get_link_pose(robot_id, link_name, joint_state=joint_state)

    def get_jacobian(
        self, robot_id: WorldRobotID, joint_state: JointState
    ) -> NDArray[np.float64] | None:
        return self._world_monitor.get_jacobian(robot_id, joint_state)

    def plan_to_joints(
        self,
        robot_id: WorldRobotID,
        goal: JointState,
        timeout: float,
        trajectory_generator: JointTrajectoryGenerator,
    ) -> PlannedMotion | PlanningResult:
        start = self.get_current_joint_state(robot_id)
        if start is None:
            return _planning_failure("No joint state")

        planner_dof = len(start.position)
        if len(goal.position) > planner_dof:
            goal = JointState(
                name=list(goal.name[:planner_dof]) if goal.name else [],
                position=list(goal.position[:planner_dof]),
            )

        result = self._planner.plan_joint_path(
            world=self.world,
            robot_id=robot_id,
            start=start,
            goal=goal,
            timeout=timeout,
        )
        if not result.is_success():
            return result

        trajectory = trajectory_generator.generate([list(state.position) for state in result.path])
        return PlannedMotion(path=result.path, trajectory=trajectory, planning_result=result)

    def plan_to_pose(
        self,
        robot_id: WorldRobotID,
        target_pose: PoseStamped,
        timeout: float,
        trajectory_generator: JointTrajectoryGenerator,
    ) -> PlannedMotion | IKResult | PlanningResult:
        current = self.get_current_joint_state(robot_id)
        if current is None:
            return _planning_failure("No joint state")

        ik = self._kinematics.solve(
            world=self.world,
            robot_id=robot_id,
            target_pose=target_pose,
            seed=current,
            check_collision=True,
        )
        if not ik.is_success() or ik.joint_state is None:
            return ik

        planned = self.plan_to_joints(
            robot_id=robot_id,
            goal=ik.joint_state,
            timeout=timeout,
            trajectory_generator=trajectory_generator,
        )
        if isinstance(planned, PlannedMotion):
            return PlannedMotion(
                path=planned.path,
                trajectory=planned.trajectory,
                planning_result=planned.planning_result,
                ik_result=ik,
            )
        return planned

    def validate_path(
        self, robot_id: WorldRobotID, path: JointPath, step_size: float = 0.05
    ) -> bool:
        return self.is_path_valid(robot_id, path, step_size)


def _planning_failure(message: str) -> PlanningResult:
    from dimos.manipulation.planning.spec.enums import PlanningStatus

    return PlanningResult(status=PlanningStatus.NO_SOLUTION, message=message)
