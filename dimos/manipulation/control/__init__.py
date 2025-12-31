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
Manipulation Control Modules

Hardware-agnostic controllers for robotic manipulation tasks.

Submodules:
- servo_control: Real-time servo-level controllers (Cartesian motion control)
- trajectory_controller: Trajectory planning and execution
"""

# Re-export from servo_control for backwards compatibility
from dimos.manipulation.control.servo_control import (
    CartesianMotionController,
    CartesianMotionControllerConfig,
    cartesian_motion_controller,
)

# Re-export from trajectory_controller
from dimos.manipulation.control.trajectory_controller import (
    JointTrajectoryController,
    JointTrajectoryControllerConfig,
    joint_trajectory_controller,
)

__all__ = [
    # Servo control
    "CartesianMotionController",
    "CartesianMotionControllerConfig",
    # Trajectory control
    "JointTrajectoryController",
    "JointTrajectoryControllerConfig",
    "cartesian_motion_controller",
    "joint_trajectory_controller",
]
