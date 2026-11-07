# Copyright 2026 Dimensional Inc.
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

"""Shared R1 Pro arm IK tasks for hardware and simulation."""

from collections.abc import Mapping

from dimos.control.coordinator import TaskConfig
from dimos.control.teleop_coordinator import TeleopControlCoordinator
from dimos.core.stream import In, Out
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel


def arm_tasks(models: Mapping[str, RobotModel], frame_prefix: str = "") -> list[TaskConfig]:
    """Independent grip/clutch sessions, using the same tasks as dual OpenYAM."""
    return [
        TaskConfig(
            name=f"teleop_{side}",
            type="teleop_ik",
            joint_names=[f"r1pro/{side}_arm_joint{i}" for i in range(1, 8)],
            priority=20,
            params={
                "robot_model": RobotModelConfig(
                    model=models[side].with_default_joint_acceleration_limit(2.0),
                    joint_names=[f"r1pro/{side}_arm_joint{i}" for i in range(1, 8)],
                    base_link=f"{frame_prefix}{side}_arm_base_link",
                ),
                "bindings": [{"hand": side, "target_frame": f"{frame_prefix}{side}_gripper_link"}],
                "pink": PinkKinematicsConfig(
                    # ProxQP falsely reports infeasibility for feasible R1
                    # differential steps, including with preconditioning off.
                    solver="quadprog",
                    dt=0.02,
                    position_cost=8.0,
                    orientation_cost=2.0,
                    posture_cost=0.01,
                    joint_limit_posture_margin=0.3,
                    lm_damping=0.01,
                    gain=1.0,
                ),
                "timeout": 0.3,
                "max_joint_velocity_rad_s": 0.5,
                "max_command_tracking_error_deg": 10.0,
                # MuJoCo soft stops can overshoot by a few milliradians. Permit
                # recovery, while commanding 0.03 rad inside physical limits.
                "feedback_limit_tolerance": 0.01,
                "command_limit_margin": 0.03,
                "joint_command_filter_cutoff_hz": 10.0,
            },
        )
        for side in ("left", "right")
    ]


class R1HardwareTeleopCoordinator(TeleopControlCoordinator):
    """Arm IK on measured hardware joints; actuation is owned by R1SpatialControl."""

    dedicated_worker = True
    r1pro_joint_state: In[JointState]
    r1pro_position_command: Out[JointState]
