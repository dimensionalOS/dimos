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

State Machine:
    IDLE ──execute()──► EXECUTING ──done──► COMPLETED
      ▲                     │                    │
      │                  cancel()             reset()
      │                     ▼                    │
      └─────reset()───── ABORTED ◄──────────────┘
                            │
                         error
                            ▼
                          FAULT ──reset()──► IDLE
"""

from dimos.core import In, Out
from dimos.msgs.sensor_msgs import JointCommand, JointState, RobotState
from dimos.msgs.trajectory_msgs import JointTrajectory, TrajectoryStatus


class JointTrajectoryControllerSpec:
    """Specification for joint trajectory controller.

    Executes joint trajectories at a fixed rate by sampling and forwarding
    joint positions to the arm driver.
    """

    # Input topics
    joint_state: In[JointState]  # Feedback from arm driver
    robot_state: In[RobotState]  # Robot status from arm driver
    trajectory: In[JointTrajectory]  # Trajectory to execute (topic-based trigger)

    # Output topics
    joint_position_command: Out[JointCommand]  # To arm driver

    # RPC Methods
    def start(self) -> None:
        """Start the trajectory controller and begin listening for trajectories."""
        ...

    def stop(self) -> None:
        """Stop the trajectory controller and execution loop."""
        ...

    def execute_trajectory(self, trajectory: JointTrajectory) -> bool:
        """
        Set and start executing a new trajectory immediately.
        If currently executing, preempts and starts new trajectory.

        Args:
            trajectory: JointTrajectory to execute

        Returns:
            True if accepted, False if in FAULT state or trajectory invalid
        """
        ...

    def cancel(self) -> bool:
        """
        Cancel the currently executing trajectory.
        Robot stops at current position.

        Returns:
            True if cancelled, False if no active trajectory
        """
        ...

    def reset(self) -> bool:
        """
        Reset from FAULT, COMPLETED, or ABORTED state back to IDLE.
        Required before executing new trajectories after a fault.

        Returns:
            True if reset successful, False if currently EXECUTING
        """
        ...

    def get_status(self) -> TrajectoryStatus:
        """
        Get the current status of the trajectory execution.

        Returns:
            TrajectoryStatus with state, progress, time_elapsed, time_remaining, error
        """
        ...
