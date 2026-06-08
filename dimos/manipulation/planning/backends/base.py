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

"""Backend-facing manipulation planning protocols and result models."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Protocol, runtime_checkable

from dimos.manipulation.planning.spec.models import (
    IKResult,
    JointPath,
    PlanningResult,
    RobotName,
    WorldRobotID,
)

if TYPE_CHECKING:
    import numpy as np
    from numpy.typing import NDArray

    from dimos.manipulation.planning.monitor.world_monitor import WorldMonitor
    from dimos.manipulation.planning.spec.config import RobotModelConfig
    from dimos.manipulation.planning.spec.models import CollisionObjectMessage, Obstacle
    from dimos.manipulation.planning.spec.protocols import KinematicsSpec, PlannerSpec, WorldSpec
    from dimos.manipulation.planning.trajectory_generator.joint_trajectory_generator import (
        JointTrajectoryGenerator,
    )
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.sensor_msgs.JointState import JointState
    from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
    from dimos.msgs.vision_msgs.Detection3D import Detection3D
    from dimos.perception.detection.type.detection3d.object import Object


@dataclass(frozen=True)
class BackendRobot:
    """Robot registered with an active planning backend."""

    name: RobotName
    robot_id: WorldRobotID
    config: RobotModelConfig


@dataclass(frozen=True)
class BackendCapabilities:
    """Capability flags for the active manipulation planning backend."""

    backend_name: str
    joint_planning: bool = False
    pose_planning: bool = False
    inverse_kinematics: bool = False
    forward_kinematics: bool = False
    jacobian: bool = False
    collision_checking: bool = False
    distance_query: bool = False
    primitive_obstacles: bool = False
    mesh_obstacles: bool = False
    pointcloud_layers: bool = False
    attached_objects: bool = False
    visualization: bool = False
    path_preview: bool = False
    drake_native_access: bool = False


@dataclass(frozen=True)
class BackendDiagnostic:
    """Single backend diagnostic event."""

    feature: str
    status: str
    message: str
    backend_name: str
    details: dict[str, Any] = field(default_factory=dict)


@dataclass
class BackendDiagnostics:
    """Accumulated backend diagnostics."""

    backend_name: str
    entries: list[BackendDiagnostic] = field(default_factory=list)

    def add(self, feature: str, status: str, message: str, **details: Any) -> None:
        """Record a diagnostic entry."""
        self.entries.append(
            BackendDiagnostic(
                feature=feature,
                status=status,
                message=message,
                backend_name=self.backend_name,
                details=details,
            )
        )

    def as_dict(self) -> dict[str, Any]:
        """Return diagnostics as RPC-friendly data."""
        return {
            "backend": self.backend_name,
            "entries": [
                {
                    "feature": entry.feature,
                    "status": entry.status,
                    "message": entry.message,
                    "backend": entry.backend_name,
                    "details": entry.details,
                }
                for entry in self.entries
            ],
        }


@dataclass(frozen=True)
class SceneUpdateResult:
    """Result of applying a scene mutation to the active backend."""

    applied: bool
    status: str
    message: str = ""
    obstacle_id: str | None = None


@dataclass(frozen=True)
class PlannedMotion:
    """Normalized backend planning output for manipulation callers."""

    path: JointPath
    trajectory: JointTrajectory
    planning_result: PlanningResult
    ik_result: IKResult | None = None


@runtime_checkable
class SceneFacade(Protocol):
    """Scene/state/query facade exposed by an active planning backend."""

    def add_robot(self, config: RobotModelConfig) -> BackendRobot: ...

    def finalize(self) -> None: ...

    def get_robot_ids(self) -> list[WorldRobotID]: ...

    def get_robot_config(self, robot_id: WorldRobotID) -> RobotModelConfig: ...

    def get_joint_limits(
        self, robot_id: WorldRobotID
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]: ...

    def start_state_monitor(self, robot_id: WorldRobotID) -> None: ...

    def start_obstacle_monitor(self) -> None: ...

    def on_joint_state(self, msg: JointState, robot_id: WorldRobotID) -> None: ...

    def on_collision_object(self, msg: CollisionObjectMessage) -> None: ...

    def on_detections(self, detections: list[Detection3D]) -> None: ...

    def on_objects(self, objects: list[Object]) -> None: ...

    def refresh_obstacles(self, min_duration: float = 0.0) -> list[dict[str, Any]]: ...

    def clear_perception_obstacles(self) -> int: ...

    def get_perception_status(self) -> dict[str, int]: ...

    def get_cached_objects(self) -> list[Object]: ...

    def list_cached_detections(self) -> list[dict[str, Any]]: ...

    def list_added_obstacles(self) -> list[dict[str, Any]]: ...

    def get_current_joint_state(self, robot_id: WorldRobotID) -> JointState | None: ...

    def get_current_velocities(self, robot_id: WorldRobotID) -> JointState | None: ...

    def wait_for_state(self, robot_id: WorldRobotID, timeout: float = 1.0) -> bool: ...

    def is_state_stale(self, robot_id: WorldRobotID, max_age: float = 1.0) -> bool: ...

    def add_obstacle(self, obstacle: Obstacle) -> str: ...

    def remove_obstacle(self, obstacle_id: str) -> bool: ...

    def update_obstacle_pose(self, obstacle_id: str, pose: PoseStamped) -> SceneUpdateResult: ...

    def clear_obstacles(self) -> None: ...

    def get_obstacles(self) -> list[Obstacle]: ...

    def is_state_valid(self, robot_id: WorldRobotID, joint_state: JointState) -> bool: ...

    def is_path_valid(
        self, robot_id: WorldRobotID, path: JointPath, step_size: float = 0.05
    ) -> bool: ...

    def get_min_distance(self, robot_id: WorldRobotID) -> float | None: ...

    def get_ee_pose(
        self, robot_id: WorldRobotID, joint_state: JointState | None = None
    ) -> PoseStamped | None: ...

    def get_link_pose(
        self, robot_id: WorldRobotID, link_name: str, joint_state: JointState | None = None
    ) -> PoseStamped | None: ...

    def get_jacobian(
        self, robot_id: WorldRobotID, joint_state: JointState
    ) -> NDArray[np.float64] | None: ...


@runtime_checkable
class PlannerFacade(Protocol):
    """Planner facade exposed by an active planning backend."""

    def plan_to_joints(
        self,
        robot_id: WorldRobotID,
        goal: JointState,
        timeout: float,
        trajectory_generator: JointTrajectoryGenerator,
    ) -> PlannedMotion | PlanningResult: ...

    def plan_to_pose(
        self,
        robot_id: WorldRobotID,
        target_pose: PoseStamped,
        timeout: float,
        trajectory_generator: JointTrajectoryGenerator,
    ) -> PlannedMotion | IKResult | PlanningResult: ...

    def validate_path(
        self, robot_id: WorldRobotID, path: JointPath, step_size: float = 0.05
    ) -> bool: ...


@runtime_checkable
class VisualizationFacade(Protocol):
    """Visualization facade exposed by an active planning backend."""

    def start_visualization_thread(self, rate_hz: float = 10.0) -> None: ...

    def get_visualization_url(self) -> str | None: ...

    def publish_visualization(self) -> None: ...

    def preview_path(
        self, robot_id: WorldRobotID, path: JointPath, duration: float = 3.0
    ) -> bool: ...

    def dismiss_preview(self, robot_id: WorldRobotID) -> None: ...


@runtime_checkable
class PlanningBackend(Protocol):
    """Coarse active planning backend boundary for ManipulationModule."""

    name: str

    def scene(self) -> SceneFacade: ...

    def planner(self) -> PlannerFacade: ...

    def visualization(self) -> VisualizationFacade | None: ...

    def capabilities(self) -> BackendCapabilities: ...

    def diagnostics(self) -> BackendDiagnostics: ...

    def stop(self) -> None: ...

    @property
    def world_monitor(self) -> WorldMonitor | None: ...

    @property
    def world(self) -> WorldSpec | None: ...

    @property
    def legacy_planner(self) -> PlannerSpec | None: ...

    @property
    def legacy_kinematics(self) -> KinematicsSpec | None: ...
