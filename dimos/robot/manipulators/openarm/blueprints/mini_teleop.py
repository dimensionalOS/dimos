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

"""OpenArm Mini leader teleop blueprints for the bimanual OpenArm follower."""

from __future__ import annotations

from dimos.control.coordinator import TaskConfig
from dimos.control.tasks.trajectory_task.trajectory_task import JOINT_TRAJECTORY_TASK_NAME
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.openarm.blueprints.teleop import (
    OpenArmTeleopCoordinator,
    _OpenArmManipulationModule,
)
from dimos.robot.manipulators.openarm.config import (
    openarm_bimanual_model_config,
    openarm_urdf_joints,
)
from dimos.teleop.openarm_mini.calibration import OpenArmMiniSide
from dimos.teleop.openarm_mini.teleop_module import OpenArmMiniTeleopModule

# Per-joint follower speed cap. Leader-follower needs human arm speeds, well
# above the WebXR IK profile; a rejected leader jump (0.75 rad) still plays
# out over a quarter second instead of stepping the PD loop.
_OPENARM_ARM_VELOCITY_PROFILE_RAD_S = (3.0, 3.0, 3.0, 3.0, 6.0, 6.0, 6.0)


def _trajectory_task(sides: tuple[OpenArmMiniSide, ...]) -> TaskConfig:
    velocity_limits = {
        joint_name: velocity_limit
        for side in sides
        for joint_name, velocity_limit in zip(
            openarm_urdf_joints(side),
            _OPENARM_ARM_VELOCITY_PROFILE_RAD_S,
            strict=True,
        )
    }
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(velocity_limits),
        priority=10,
        params={
            "start_position_tolerance": 0.05,
            "velocity_limits": velocity_limits,
        },
    )


# Leader joint N drives follower joint N through the coordinator's streamed
# joint_command path. The follower is in-memory unless both CAN ports are set.
teleop_openarm_mini = autoconnect(
    OpenArmMiniTeleopModule.blueprint(enabled_sides=("left", "right")),
    OpenArmTeleopCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=[_trajectory_task(("left", "right"))],
    ),
    _OpenArmManipulationModule.blueprint(
        model=openarm_bimanual_model_config(),
        visualization={"backend": "viser"},
    ),
)

teleop_openarm_mini_left = autoconnect(
    OpenArmMiniTeleopModule.blueprint(enabled_sides=("left",)),
    OpenArmTeleopCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=[_trajectory_task(("left",))],
    ),
    _OpenArmManipulationModule.blueprint(
        model=openarm_bimanual_model_config(),
        visualization={"backend": "viser"},
    ),
)

teleop_openarm_mini_right = autoconnect(
    OpenArmMiniTeleopModule.blueprint(enabled_sides=("right",)),
    OpenArmTeleopCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=[_trajectory_task(("right",))],
    ),
    _OpenArmManipulationModule.blueprint(
        model=openarm_bimanual_model_config(),
        visualization={"backend": "viser"},
    ),
)

# Split deployment: leaders on an operator machine, follower on the robot.
# Both sides name the stream joint_command, so it rides one zenoh topic once
# the operator machine joins the robot's bus (a zenoh router on the robot,
# --zenoh-mode client --robot-ip <robot> here). The robot stack owns the
# bus-wide Coordinator name, so the leader half does not claim it.
teleop_openarm_mini_leader = OpenArmMiniTeleopModule.blueprint(
    enabled_sides=("left", "right")
).global_config(serve_coordinator_rpc=False)

teleop_openarm_mini_leader_left = OpenArmMiniTeleopModule.blueprint(
    enabled_sides=("left",)
).global_config(serve_coordinator_rpc=False)

teleop_openarm_mini_leader_right = OpenArmMiniTeleopModule.blueprint(
    enabled_sides=("right",)
).global_config(serve_coordinator_rpc=False)

teleop_openarm_mini_follower = autoconnect(
    OpenArmTeleopCoordinator.blueprint(
        instance_name="ControlCoordinator",
        tasks=[_trajectory_task(("left", "right"))],
    ),
    _OpenArmManipulationModule.blueprint(
        model=openarm_bimanual_model_config(),
        visualization={"backend": "viser"},
    ),
)
