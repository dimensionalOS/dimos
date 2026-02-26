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

"""Path follower task for ControlCoordinator.

Implements path-following control as a ControlTask that runs at 100Hz
within the ControlCoordinator tick loop.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.nav_msgs import Path
from dimos.navigation.replanning_a_star.controllers import (
    PurePursuitController,
    PIDCrossTrackController,
)
from dimos.navigation.replanning_a_star.path_distancer import PathDistancer
from dimos.navigation.replanning_a_star.velocity_profiler import VelocityProfiler
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

logger = setup_logger()


@dataclass
class PathFollowerTaskConfig:
    """Configuration for path follower task.
    
    Attributes:
        joint_names: Joint names for base control ["base_vx", "base_vy", "base_wz"]
        priority: Task priority for arbitration
        max_linear_speed: Maximum forward speed (m/s)
        goal_tolerance: Distance tolerance for goal (m)
        orientation_tolerance: Orientation tolerance for goal (rad)
    """
    
    joint_names: list[str] = None  # type: ignore[assignment]
    priority: int = 10
    max_linear_speed: float = 0.8
    goal_tolerance: float = 0.3
    orientation_tolerance: float = 0.35
    
    def __post_init__(self) -> None:
        """Set default joint names if not provided."""
        if self.joint_names is None:
            self.joint_names = ["base_vx", "base_vy", "base_wz"]


class PathFollowerTask(BaseControlTask):
    """Path-following control task for mobile base.
    
    Follows a Path using Pure Pursuit controller with velocity profiling.
    Outputs JointCommandOutput with base velocities [vx, vy, wz].
    
    CRITICAL: Uses state.t_now from CoordinatorState, never calls time.time()
    
    State Machine:
        IDLE ──start_path()──► FOLLOWING ──goal_reached──► COMPLETED
          ▲                        │                          │
          │                    cancel()                    reset()
          │                        ▼                          │
          └─────reset()───── ABORTED ◄──────────────────────┘
    """
    
    # Task states
    State: type = Literal["idle", "following", "completed", "aborted"]
    
    def __init__(
        self,
        name: str,
        config: PathFollowerTaskConfig,
        global_config: GlobalConfig,
    ):
        """Initialize path follower task.
        
        Args:
            name: Unique task name
            config: Task configuration
            global_config: Global configuration
        """
        if len(config.joint_names) != 3:
            raise ValueError(
                f"PathFollowerTask '{name}' requires exactly 3 joints "
                f"(base_vx, base_vy, base_wz), got {len(config.joint_names)}"
            )
        
        self._name = name
        self._config = config
        self._global_config = global_config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        
        # State machine
        self._state: PathFollowerTask.State = "idle"
        self._path: Path | None = None
        self._path_distancer: PathDistancer | None = None
        self._velocity_profiler: VelocityProfiler | None = None
        self._current_odom: PoseStamped | None = None
        self._closest_index: int = 0
        
        # Controller
        self._controller = PurePursuitController(
            global_config,
            control_frequency=100.0, 
            min_lookahead=0.2,
            max_lookahead=0.5,  
            lookahead_gain=0.3,  
        )
        # Cross-track PID controller for tighter lateral tracking
        self._cross_track_controller = PIDCrossTrackController(
            control_frequency=100.0,
            k_p=1.5,  
            k_i=0.1, 
            k_d=0.2,  
            max_correction=1.0,  
            max_integral=0.3,
        )
        
        logger.info(
            f"PathFollowerTask {name} initialized for joints: {config.joint_names}"
        )
    
    @property
    def name(self) -> str:
        """Unique task identifier."""
        return self._name
    
    def claim(self) -> ResourceClaim:
        """Declare resource requirements."""
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.VELOCITY,
        )
    
    def is_active(self) -> bool:
        """Check if task should run this tick."""
        return self._state == "following"
    
    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Compute path-following command for this tick.
        
        CRITICAL: Uses state.t_now for timing, NOT time.time()!
        
        Args:
            state: Current coordinator state
            
        Returns:
            JointCommandOutput with [base_vx, base_vy, base_wz], or None if not active
        """
        # Check if task is active and has a path and path distancer
        if self._state != "following" or self._path is None or self._path_distancer is None:
            return None
        if self._current_odom is None:
            return None
        
        path_distancer = self._path_distancer
        velocity_profiler = self._velocity_profiler
        current_odom = self._current_odom
        closest_index = self._closest_index
        
        # Check if goal reached
        current_pos = np.array([current_odom.position.x, current_odom.position.y])
        
        # Distance to goal
        distance_to_goal = path_distancer.distance_to_goal(current_pos)
        
        # --- FINAL ROTATION MODE: inside positional tolerance ---
        if distance_to_goal < self._config.goal_tolerance and len(self._path.poses) > 0:
            goal_yaw = self._path.poses[-1].orientation.euler[2]
            robot_yaw = current_odom.orientation.euler[2]
            yaw_error = angle_diff(goal_yaw, robot_yaw)

            # If orientation is also aligned, we are done
            if abs(yaw_error) < self._config.orientation_tolerance:
                self._state = "completed"
                logger.info(f"PathFollowerTask {self._name} completed - goal reached")
                return JointCommandOutput(
                    joint_names=self._joint_names_list,
                    velocities=[0.0, 0.0, 0.0],
                    mode=ControlMode.VELOCITY,
                )

            # Otherwise, rotate to the goal orientation
            twist = self._controller.rotate(yaw_error)

            # Clamp yaw rate
            max_wz = 1.2
            twist.angular.z = float(np.clip(twist.angular.z, -max_wz, max_wz))

            # Return JointCommandOutput
            return JointCommandOutput(
                joint_names=self._joint_names_list,
                velocities=[
                    float(twist.linear.x),
                    float(twist.linear.y),
                    float(twist.angular.z),
                ],
                mode=ControlMode.VELOCITY,
            )
        
        # --- NORMAL PATH FOLLOWING (Pure Pursuit + cross-track PID) ---
        # Update closest point on path
        closest_index = path_distancer.find_closest_point_index(current_pos)
        self._closest_index = closest_index
        
        # Get velocity from profiler
        target_velocity = velocity_profiler.get_velocity_at_index(self._path, closest_index)
        
        # Get curvature for better control
        curvature = path_distancer.get_curvature_at_index(closest_index)
        
        # Find adaptive lookahead point
        lookahead_point = path_distancer.find_adaptive_lookahead_point(
            closest_index,
            target_velocity,
            min_lookahead=0.3,
            max_lookahead=2.0,
        )
        
        # Compute control command using Pure Pursuit
        twist = self._controller.advance(
            lookahead_point,
            current_odom,
            current_speed=target_velocity,
            path_curvature=curvature,
        )

        # Cross-track correction: tighten lateral tracking around the path
        # (except when very close to goal to avoid conflicting with orientation alignment)
        cross_track_error = 0.0
        ct_correction = 0.0
        if distance_to_goal >= self._config.goal_tolerance:
            cross_track_error = path_distancer.get_signed_cross_track_error(current_pos)
            ct_correction = self._cross_track_controller.compute_correction(cross_track_error)
            twist.angular.z -= ct_correction

        # Re-clamp yaw rate to match controller limits
        max_wz = 1.2
        twist.angular.z = float(np.clip(twist.angular.z, -max_wz, max_wz))
        
        # Convert Twist to JointCommandOutput
        # Twist: linear.x (forward), linear.y (left), angular.z (yaw)
        # Joints: base_vx (forward), base_vy (left), base_wz (yaw)
        return JointCommandOutput(
            joint_names=self._joint_names_list,
            velocities=[
                float(twist.linear.x),  # base_vx
                float(twist.linear.y),  # base_vy
                float(twist.angular.z),  # base_wz
            ],
            mode=ControlMode.VELOCITY,
        )
    
    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Handle preemption by higher-priority task.
        
        Args:
            by_task: Name of preempting task
            joints: Joints that were preempted
        """
        if joints & self._joint_names:
            logger.warning(
                f"PathFollowerTask {self._name} preempted by {by_task} on joints {joints}"
            )
            if self._state == "following":
                self._state = "aborted"
    
    # =========================================================================
    # Task-specific methods
    # =========================================================================
    
    def start_path(self, path: Path, current_odom: PoseStamped) -> bool:
        """Start following a new path.
        
        Args:
            path: Path to follow
            current_odom: Current robot pose
            
        Returns:
            True if accepted, False if invalid
        """
        if path is None or len(path.poses) < 2:
            logger.warning(f"PathFollowerTask {self._name}: invalid path")
            return False
        
        self._path = path
        self._path_distancer = PathDistancer(path)
        self._velocity_profiler = VelocityProfiler(
            max_linear_speed=self._config.max_linear_speed,
            max_angular_speed=2.0,
            max_linear_accel=2.0,
            max_linear_decel=3.0,
            max_centripetal_accel=2.5,
            min_speed=0.1,
        )

        self._cross_track_controller.reset()
        
        self._current_odom = current_odom
        self._closest_index = self._path_distancer.find_closest_point_index(
            np.array([current_odom.position.x, current_odom.position.y])
        )
        self._state = "following"
        
        logger.info(
            f"PathFollowerTask {self._name} started following path with "
            f"{len(path.poses)} points"
        )
        return True
    
    def update_odom(self, odom: PoseStamped) -> None:
        """Update current robot pose.
        
        Args:
            odom: Current robot pose
        """
        self._current_odom = odom
    
    def cancel(self) -> bool:
        """Cancel current path following.
        
        Returns:
            True if cancelled, False if not following
        """
        if self._state != "following":
            return False
        self._state = "aborted"
        logger.info(f"PathFollowerTask {self._name} cancelled")
        return True
    
    def reset(self) -> bool:
        """Reset to idle state.
        
        Returns:
            True if reset, False if currently following
        """
        if self._state == "following":
            logger.warning(f"Cannot reset {self._name} while following")
            return False
        self._state = "idle"
        self._path = None
        self._path_distancer = None
        self._velocity_profiler = None
        self._current_odom = None
        logger.info(f"PathFollowerTask {self._name} reset to IDLE")
        return True
    
    def get_state(self) -> State:
        """Get current state."""
        return self._state