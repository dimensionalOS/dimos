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

"""M20 navigation blueprint — Docker ARISE SLAM + planners, host Python modules.

The ROSNav Docker container runs C++ modules that need nix (ARISE SLAM,
TerrainAnalysis, LocalPlanner, PathFollower). The host runs Python-only
modules (PGO, CmdVelMux, ClickToGoal) that enhance the pipeline.

Data flow:
    Container (ARISE SLAM → TerrainAnalysis → LocalPlanner → PathFollower)
      → cmd_vel, registered_scan, odometry via ROSNav bridge
    Host:
      → PGO (loop closure) → corrected_odometry + global_map
      → CmdVelMux (nav cmd_vel + teleop) → final cmd_vel
      → M20Connection → /NAV_CMD → robot motors
"""

from dimos.core.blueprints import autoconnect
from dimos.navigation.cmd_vel_mux import CmdVelMux
from dimos.navigation.smart_nav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smart_nav.modules.pgo.pgo import PGO
from dimos.robot.deeprobotics.m20.connection import m20_connection
from dimos.robot.deeprobotics.m20.rosnav_docker import M20ROSNav, m20_ros_nav
from dimos.visualization.rerun.bridge import RerunBridgeModule
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
        PGO.blueprint(),
        ClickToGoal.blueprint(),
        CmdVelMux.blueprint(),
        RerunBridgeModule.blueprint(),
        WebsocketVisModule.blueprint(),
    )
    .remappings(
        [
            # ROSNav outputs "lidar" (registered_scan from ARISE);
            # PGO expects "registered_scan"
            (M20ROSNav, "lidar", "registered_scan"),
            # ROSNav cmd_vel (from container's planner) → CmdVelMux nav input
            (M20ROSNav, "cmd_vel", "nav_cmd_vel"),
            # PGO-corrected odometry for ClickToGoal (globally consistent goals)
            (ClickToGoal, "odometry", "corrected_odometry"),
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
