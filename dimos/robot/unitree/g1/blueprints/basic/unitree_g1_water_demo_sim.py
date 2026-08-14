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

There is deliberately no mapper, costmap, global planner, or MovementManager
in this graph. A path-follower module produces the autonomy Twist while the
coordinator arbitrates it against Rerun teleop before the single GR00T task.

Usage::

    dimos --simulation mujoco run unitree-g1-water-demo-sim
    dimos shell
    app.WateringTaskModule.start_approach()
    app.WateringTaskModule.get_status()
    app.WateringTaskModule.start_pour()
    app.WateringTaskModule.cancel_watering()
"""

from __future__ import annotations

import math
from typing import Any, cast

from dimos.control.coordinator import TwistSourceConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.manipulation.mobile.pose_target_observation_module import (
    PoseTargetObservationModule,
)
from dimos.navigation.rpp_path_follower.module import RPPPathFollower
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
    (_G1GrootCoordinator, "operator_twist_command", "tele_cmd_vel"),
    (_G1GrootCoordinator, "autonomy_twist_command", "autonomy_cmd_vel"),
    (RPPPathFollower, "base_pose", "odom"),
    (RPPPathFollower, "path", "approach_command_path"),
    (RPPPathFollower, "stop_movement", "stop_approach"),
    (RPPPathFollower, "nav_cmd_vel", "autonomy_cmd_vel"),
    (WateringTaskModule, "base_pose", "odom"),
    (WateringTaskModule, "operator_command", "tele_cmd_vel"),
    (WateringTaskModule, "approach_path", "path"),
    (WateringTaskModule, "approach_goal", "goal_request"),
]

_WATERING_GROOT_TASK = g1_groot_task_config(timeout=0.25)

_TWIST_SOURCES = (
    TwistSourceConfig("operator", "operator_twist_command", priority=100, timeout=0.35),
    TwistSourceConfig("autonomy", "autonomy_twist_command", priority=50, timeout=0.25),
)

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
            extra_tasks=(_ARM_TRAJECTORY_TASK,),
            locomotion_task=_WATERING_GROOT_TASK,
            twist_sources=_TWIST_SOURCES,
        ),
        SimBodyPose.blueprint(body_name="plant_pot_1"),
        PoseTargetObservationModule.blueprint(
            object_id="plant_pot_1",
            label="plant pot",
            source="sim_ground_truth",
        ),
        g1_manipulation(),
        RPPPathFollower.blueprint(
            speed=0.25,
            min_linear_speed=0.15,
            slowdown_distance=0.4,
            control_frequency=10.0,
            goal_tolerance=0.06,
            orientation_tolerance=math.radians(5.0),
            k_angular=1.5,
            max_yaw_rate=0.25,
            min_angular_speed=0.12,
            rotation_threshold=math.radians(20.0),
            lookahead_dist=0.3,
            lookahead_min=0.2,
            lookahead_max=0.4,
            lookahead_speed_scale=1.0,
            pose_timeout=0.5,
            max_centripetal_accel=0.2,
            max_linear_accel=0.3,
            max_linear_decel=0.5,
            synthesize_tangent_headings=True,
        ),
        WateringTaskModule.blueprint(
            target_id="plant_pot_1",
            motion_enabled=False,
            approach_motion_enabled=True,
            pour_motion_enabled=True,
        ),
        _viewer(),
    )
    .remappings(cast("Any", _demo_remappings))
    .global_config(robot_model="unitree_g1", n_workers=_n_workers + 1)
)
