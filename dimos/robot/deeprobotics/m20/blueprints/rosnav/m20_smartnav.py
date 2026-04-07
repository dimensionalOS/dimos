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

"""M20 SmartNav blueprint — native planning with Docker ARISE SLAM.

Adapts the G1 nav onboard pattern (unitree_g1_nav_onboard.py) for M20:
- ARISE SLAM runs inside Docker via M20ROSNav (outputs registered_scan + odometry)
- SmartNav handles all planning natively (terrain analysis, local/global planner,
  PGO loop closure, click-to-goal, cmd_vel mux)
- M20Connection sends velocity commands via /NAV_CMD DDS

Data flow:
    Docker (ARISE SLAM) → registered_scan + odometry
    → SmartNav (TerrainAnalysis → LocalPlanner → PathFollower → CmdVelMux)
    → cmd_vel → M20Connection → /NAV_CMD → robot motors
"""

from dimos.core.blueprints import autoconnect
from dimos.navigation.smart_nav.main import smart_nav
from dimos.robot.deeprobotics.m20.connection import m20_connection
from dimos.robot.deeprobotics.m20.rosnav_docker import M20ROSNav, m20_ros_nav
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

m20_smartnav = (
    autoconnect(
        m20_connection(
            ip="10.21.31.103",
            enable_ros=False,
            enable_lidar=False,
            lidar_height=0.47,
        ),
        m20_ros_nav(),
        smart_nav(
            vehicle_height=0.47,  # 47cm lidar height in agile stance
            terrain_analysis={
                "obstacle_height_threshold": 0.2,
                "ground_height_threshold": 0.1,
            },
            local_planner={
                "max_speed": 1.0,
                "autonomy_speed": 1.0,
                "two_way_drive": False,
            },
            path_follower={
                "max_speed": 1.0,
                "autonomy_speed": 1.0,
                "max_acceleration": 2.0,
                "two_way_drive": False,
            },
            far_planner={
                "sensor_range": 15.0,
                "is_static_env": False,
            },
        ),
        WebsocketVisModule.blueprint(),
    )
    .remappings(
        [
            # ROSNav outputs "lidar" (registered_scan from ARISE);
            # SmartNav expects "registered_scan"
            (M20ROSNav, "lidar", "registered_scan"),
        ]
    )
    .global_config(
        n_workers=2,
        robot_model="deeprobotics_m20",
        robot_ip="10.21.31.103",
        robot_width=0.45,
        robot_rotation_diameter=0.6,
    )
)


def main() -> None:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    coordinator = ModuleCoordinator.build(m20_smartnav)
    coordinator.loop()


__all__ = ["m20_smartnav"]

if __name__ == "__main__":
    main()
