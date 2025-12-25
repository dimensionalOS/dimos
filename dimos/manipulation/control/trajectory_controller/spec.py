# Copyright 2025 Dimensional Inc.
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

"""
Joint Trajectory Controller Specification

A simple joint-space trajectory executor. Does NOT:
- Use Cartesian space
- Compute error
- Apply PID
- Call IK

Just samples a trajectory at time t and sends joint positions to the driver.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Protocol

if TYPE_CHECKING:
    from dimos.core import In, Out
    from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState
    from dimos.msgs.trajectory_msgs import JointTrajectory as JointTrajectoryMsg, TrajectoryState

# Input topics
joint_state: In[JointState] | None = None  # Feedback from arm driver
robot_state: In[RobotState] | None = None  # Robot status from arm driver
trajectory: In[JointTrajectoryMsg] | None = None  # Desired trajectory

# Output topics
joint_position_command: Out[JointCommand] | None = None  # To arm driver


def execute_trajectory() -> bool:
    """
    Set and start executing a new trajectory immediately.
    Returns True if accepted, False if controller busy or traj invalid.
    """
    raise NotImplementedError("Protocol method")


def cancel() -> bool:
    """
    Cancel the currently executing trajectory.
    Returns True if cancelled, False if no active trajectory.
    """
    raise NotImplementedError("Protocol method")


def get_status() -> TrajectoryStatusProtocol:
    """
    Get the current status of the trajectory execution.
    Returns a TrajectoryStatus message with details.
     "state": "IDLE" | "EXECUTING" | "COMPLETED" | "ABORTED" | "FAULT",
      "progress": float in [0,1],
      "active_traj_id": Optional[str],
      "error": Optional[str],
    """
    raise NotImplementedError("Protocol method")
    ...


class JointTrajectoryProtocol(Protocol):
    """Protocol for a joint trajectory object."""

    duration: float  # Total duration in seconds

    def sample(self, t: float) -> tuple[list[float], list[float]]:
        """
        Sample the trajectory at time t.

        Args:
            t: Time in seconds (0 <= t <= duration)

        Returns:
            Tuple of (q_ref, qd_ref):
                - q_ref: Joint positions (radians)
                - qd_ref: Joint velocities (rad/s)
        """
        ...


class TrajectoryStatusProtocol(Protocol):
    """Status of trajectory execution."""

    state: TrajectoryState  # Current state
    progress: float  # Progress 0.0 to 1.0
    time_elapsed: float  # Seconds since trajectory start
    time_remaining: float  # Estimated seconds remaining
    error: str | None  # Error message if FAULT state
