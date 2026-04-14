#!/usr/bin/env python3
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

"""M20 native navigation blueprint — no Docker, no ROS.

All modules run on the NOS host via nix:
- DrddsLidarBridge: reads drdds SHM (lidar + IMU) and publishes to LCM
- AriseSLAM: C++ NativeModule for lidar-inertial SLAM
- SmartNav: full planning stack (terrain analysis, local planner, path
  follower, simple planner, PGO, CmdVelMux, ClickToGoal)
- NavCmdPub: raw FastDDS publisher for /NAV_CMD (no rclpy needed)
- M20Connection: camera (RTSP), robot state (UDP heartbeat, gait, mode)

Data flow:
    drdds_recv (host) → POSIX SHM
    → DrddsLidarBridge → raw_points + imu (LCM)
    → AriseSLAM → registered_scan + odometry
    → SmartNav (TerrainAnalysis → LocalPlanner → PathFollower)
    → CmdVelMux → cmd_vel → NavCmdPub → rt/NAV_CMD (FastDDS) → AOS motors
"""

from dimos.core.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.navigation.smart_nav.main import smart_nav, smart_nav_rerun_config
from dimos.navigation.smart_nav.modules.arise_slam.arise_slam import AriseSLAM
from dimos.robot.deeprobotics.m20.blueprints.nav.m20_rerun import (
    camera_info_override,
    m20_rerun_blueprint,
    static_robot,
)
from dimos.robot.deeprobotics.m20.connection import m20_connection
from dimos.robot.deeprobotics.m20.drdds_bridge.module import DrddsLidarBridge, NavCmdPub
from dimos.visualization.vis_module import vis_module

# M20 physical dimensions
M20_HEIGHT_CLEARANCE = 0.57  # 570mm standing height
M20_LIDAR_HEIGHT = 0.47  # lidar at 47cm in agile stance

m20_smartnav_native = (
    autoconnect(
        m20_connection(
            ip="10.21.31.103",
            enable_camera=False,  # TODO: re-enable once startup is fast
            lidar_height=M20_LIDAR_HEIGHT,
        ),
        NavCmdPub.blueprint(),
        DrddsLidarBridge.blueprint(build_command=None),
        AriseSLAM.blueprint(
            build_command=None,
            scan_voxel_size=0.1,
            max_range=50.0,
        ),
        smart_nav(
            use_simple_planner=True,
            vehicle_height=M20_HEIGHT_CLEARANCE,
            terrain_analysis={
                "build_command": None,
                "obstacle_height_threshold": 0.01,
                "ground_height_threshold": 0.01,
            },
            local_planner={
                "build_command": None,
            },
            path_follower={
                "build_command": None,
                "two_way_drive": False,
            },
            simple_planner={
                "cell_size": 0.3,
                "obstacle_height_threshold": 0.20,
                "inflation_radius": 0.4,
                "lookahead_distance": 2.0,
                "replan_rate": 5.0,
                "replan_cooldown": 2.0,
            },
        ),
        vis_module(
            viewer_backend=global_config.viewer,
            rerun_config=smart_nav_rerun_config(
                {
                    "blueprint": m20_rerun_blueprint,
                    "visual_override": {
                        "world/camera_info": camera_info_override,
                    },
                    "static": {
                        "world/tf/base_link": static_robot,
                    },
                }
            ),
        ),
    )
    .remappings(
        [
            # DrddsLidarBridge outputs "lidar"; AriseSLAM expects "raw_points"
            (DrddsLidarBridge, "lidar", "raw_points"),
        ]
    )
    .global_config(
        n_workers=4,
        robot_model="deeprobotics_m20",
        robot_ip="10.21.31.103",
        robot_width=0.45,
        robot_rotation_diameter=0.6,
    )
)


def main() -> None:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    coordinator = ModuleCoordinator.build(m20_smartnav_native)
    coordinator.loop()


__all__ = ["m20_smartnav_native"]

if __name__ == "__main__":
    main()
