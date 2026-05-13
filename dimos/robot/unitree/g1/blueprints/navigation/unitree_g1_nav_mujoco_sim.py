#!/usr/bin/env python3
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

"""G1 nav sim (MuJoCo) — SimplePlanner + PGO + local obstacle avoidance.

MuJoCo counterpart to ``unitree_g1_nav_sim`` (Unity).  Same nav-stack
composition, same goal-following pipeline; the only difference is the
sim source.  Where the Unity bridge publishes ``registered_scan``,
``odometry``, and ``terrain_map`` directly (ground truth), MuJoCo's G1
connection only publishes raw ``lidar`` and a ``PoseStamped`` pose.  We
bridge the two ports:

- ``lidar`` is already in world frame from the MuJoCo lidar sensor
  process (see ``mujoco_process.py``), so a remap from ``lidar`` →
  ``registered_scan`` is sufficient — no transform needed.
- ``PoseStamped`` is converted to ``nav_msgs/Odometry`` via
  ``MujocoPoseToOdometryAdapter`` (twist fields zeroed; the connection
  doesn't expose linear/angular velocity).
- ``terrain_map`` is computed by ``TerrainAnalysis`` inside
  ``create_nav_stack`` from the registered scan, just like onboard.

Data flow:
    Click → MovementManager → goal → SimplePlanner
    → way_point → LocalPlanner → path → PathFollower
    → nav_cmd_vel → MovementManager → cmd_vel → G1SimConnection (MuJoCo)

    G1SimConnection.lidar (= registered_scan) → TerrainAnalysis → terrain_map
    G1SimConnection.odom → MujocoPoseToOdometryAdapter → odometry → PGO
"""

from __future__ import annotations

from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.modules.movement_manager.movement_manager import MovementManager
from dimos.navigation.nav_stack import create_nav_stack
from dimos.robot.unitree.g1.blueprints.navigation._mujoco_pose_adapter import (
    MujocoPoseToOdometryAdapter,
)
from dimos.robot.unitree.g1.config import G1_LOCAL_PLANNER_PRECOMPUTED_PATHS, G1_VEHICLE_HEIGHT
from dimos.robot.unitree.g1.mujoco_sim import G1SimConnection


def _rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Spatial3DView(
                origin="world",
                name="3D",
                eye_controls=rrb.EyeControls3D(
                    position=(0.0, 0.0, 20.0),
                    look_target=(0.0, 0.0, 0.0),
                    eye_up=(0.0, 0.0, 1.0),
                ),
            ),
        ),
        collapse_panels=True,
    )


unitree_g1_nav_mujoco_sim = (
    autoconnect(
        G1SimConnection.blueprint(),
        MujocoPoseToOdometryAdapter.blueprint(),
        create_nav_stack(
            use_simple_planner=True,
            vehicle_height=G1_VEHICLE_HEIGHT,
            simple_planner={"body_frame": "base_link"},
            terrain_analysis={
                "obstacle_height_threshold": 0.1,
                "ground_height_threshold": 0.05,
                "max_relative_z": 0.3,
                "min_relative_z": -1.5,
            },
            local_planner={
                "paths_dir": str(G1_LOCAL_PLANNER_PRECOMPUTED_PATHS),
                "max_speed": 2.0,
                "autonomy_speed": 2.0,
                "obstacle_height_threshold": 0.1,
                "max_relative_z": 0.3,
                "min_relative_z": -1.5,
                "freeze_ang": 180.0,
                "two_way_drive": False,
            },
            path_follower={
                "max_speed": 2.0,
                "autonomy_speed": 2.0,
                "max_acceleration": 4.0,
                "slow_down_distance_threshold": 0.5,
                "omni_dir_goal_threshold": 0.5,
                "two_way_drive": False,
            },
        ),
        MovementManager.blueprint(),
        # vis_module(
        #     viewer_backend=global_config.viewer,
        #     rerun_config=nav_stack_rerun_config(
        #         {
        #             "blueprint": _rerun_blueprint,
        #             # Update tf#/sensor each tick from the live odometry so
        #             # the static robot wireframe (parent_frame=tf#/sensor)
        #             # tracks the robot in the scene instead of staying at
        #             # the world origin (z=0).
        #             "visual_override": {"world/odometry": g1_mujoco_sensor_tf_override},
        #             "static": {
        #                 "world/tf/robot": g1_static_robot,
        #             },
        #         }
        #     ),
        # ),
    )
    .remappings(
        [
            # MuJoCo's lidar is already in world frame, so it's the
            # `registered_scan` the nav stack expects.
            (G1SimConnection, "lidar", "registered_scan"),
            # Planner owns way_point — disconnect MovementManager's click relay.
            (MovementManager, "way_point", "_mgr_way_point_unused"),
        ]
    )
    .global_config(
        n_workers=8,
        robot_model="unitree_g1",
        simulation=True,
        # Multi-room HSSD-derived house scene (bedroom + bathroom).
        # See data/.lfs/hssd_house.tar.gz.  Override at the CLI with
        # --mujoco-room office1 to fall back to the original single-room
        # office, or --mujoco-room scene_empty for a featureless plane.
        mujoco_room="hssd_house",
        # Spawn inside the dining_room (interior centre — the room's
        # body frame).  Earlier placements on the open floor or in the
        # hallway corridor kept landing in wall bands; the dining_room
        # interior is a reliable known-walkable patch.
        mujoco_start_pos="0.2, -3.2",
        # Skip the ONNX walking policy and integrate cmd_vel directly
        # into the floating base each tick (joints frozen at the home
        # pose).  Nav-stack runs are about planner correctness, not
        # gait fidelity; gait dynamics just slow iteration without
        # changing what the planner sees.  Override with
        # mujoco_kinematic_robot=False to bring the gait back.
        mujoco_kinematic_robot=True,
        mujoco_camera_position="1.5, 9.5, 0.0, 35.0, 90.0, -89.0",
    )
)


__all__ = ["unitree_g1_nav_mujoco_sim"]
