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

"""Simulation-first G1 watering demo with no navigation stack.

MuJoCo publishes privileged plant and robot poses in ``world`` for the sim
regression only; this migration does not add a hardware odometry source.
The pose adapter turns the simulator-specific pose into the same typed target
contract that perception will eventually provide on hardware. ``WateringTaskModule``
then owns the complete approach/latch/verify/pour lifecycle.

There is deliberately no mapper, costmap, global planner, MovementManager, or
teleop producer in this graph. The task's last-mile velocity command is the
only source connected to the coordinator, which makes sim parity deterministic
and removes the old last-message-wins behavior.

Usage::

    dimos --simulation mujoco run unitree-g1-water-demo-sim
    dimos shell
    app.WateringTaskModule.start_watering()
    app.WateringTaskModule.get_status()
    app.WateringTaskModule.cancel_watering()
"""

from __future__ import annotations

from typing import Any, cast

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.mobile.pose_target_observation_module import (
    PoseTargetObservationModule,
)
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc import (
    _G1_NUM_MOTORS,
    _MJCF_PATH,
    _ROBOT_MESHDIR,
    _SCENE_PROPS,
    _g1_sim_spec,
    _G1GrootCoordinator,
    _n_workers,
    _viewer,
    g1_groot_coordinator,
    g1_groot_task_config,
)
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_groot_wbc_manip import (
    _ARM_TRAJECTORY_TASK,
    g1_manipulation,
)
from dimos.robot.unitree.g1.watering_task import WateringTaskModule
from dimos.simulation.engines.mujoco_sim_module import MujocoSimModule
from dimos.simulation.engines.sim_body_pose import SimBodyPose

if global_config.simulation != "mujoco":
    raise ValueError(
        "unitree-g1-water-demo-sim requires --simulation mujoco; "
        "the hardware graph is intentionally migrated separately"
    )

_demo_remappings = [
    (_G1GrootCoordinator, "twist_command", "cmd_vel"),
    (WateringTaskModule, "base_command", "cmd_vel"),
    (WateringTaskModule, "base_pose", "odom"),
    (WateringTaskModule, "operator_command", "tele_cmd_vel"),
    (WateringTaskModule, "approach_path", "path"),
    (WateringTaskModule, "approach_goal", "goal_request"),
]

_TELEOP_GROOT_TASK = g1_groot_task_config(
    name="teleop_groot_wbc",
    priority=60,
    stream_bind={"twist_command": "tele_cmd_vel"},
    yield_when_idle=True,
)

_WATERING_GROOT_TASK = g1_groot_task_config(timeout=0.25)

_watering_backend = MujocoSimModule.blueprint(
    address=_MJCF_PATH,
    extra_mjcf=_SCENE_PROPS,
    robot_meshdir=_ROBOT_MESHDIR,
    headless=global_config.viewer == "none",
    dof=_G1_NUM_MOTORS,
    enable_color=False,
    enable_depth=False,
    enable_pointcloud=False,
    wait_for_control_command=True,
    robot_sim_spec=_g1_sim_spec,
)

unitree_g1_water_demo_sim = (
    autoconnect(
        _watering_backend,
        g1_groot_coordinator(
            extra_tasks=(_ARM_TRAJECTORY_TASK, _TELEOP_GROOT_TASK),
            locomotion_task=_WATERING_GROOT_TASK,
        ),
        SimBodyPose.blueprint(body_name="plant_pot_1"),
        PoseTargetObservationModule.blueprint(
            object_id="plant_pot_1",
            label="plant pot",
            source="sim_ground_truth",
        ),
        g1_manipulation(),
        WateringTaskModule.blueprint(target_id="plant_pot_1"),
        _viewer(),
    )
    .remappings(cast("Any", _demo_remappings))
    .global_config(robot_model="unitree_g1", n_workers=_n_workers + 1)
)
