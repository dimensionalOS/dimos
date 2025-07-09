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

"""Minimal UnitreeGo2 implementation with module configuration support."""

from typing import Optional, cast
import os
import numpy as np
import threading

from dimos.robot.robot_clean import Robot
from dimos.robot.module_utils import robot_capability
from dimos.robot.unitree_webrtc.connection import UnitreeWebRTCConnection
from dimos.perception.spatial_perception import SpatialMemory
from dimos.perception.person_tracker import PersonTrackingStream
from dimos.perception.object_tracker import ObjectTrackingStream
from dimos.robot.global_planner.planner import AstarPlanner
from dimos.robot.local_planner.vfh_local_planner import VFHPurePursuitPlanner
from dimos.robot.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.skills.skills import SkillLibrary
from dimos.robot.capabilities import has_capability, Odometry


@robot_capability(
    SpatialMemory,
    PersonTrackingStream,
    ObjectTrackingStream,
    AstarPlanner,
    VFHPurePursuitPlanner,
    WavefrontFrontierExplorer,
)
class UnitreeGo2(Robot):
    """Minimal Unitree Go2 with dependency injection and config support."""

    def __init__(
        self,
        ip: str,
        mode: str = "ai",
        config_file: Optional[str] = None,
    ):
        """Initialize robot with minimal parameters.

        Args:
            ip: Robot IP address
            mode: Robot mode (ai/normal)
            config_file: Optional YAML config file for modules
        """
        # Create connection
        conn = UnitreeWebRTCConnection(ip=ip, mode=mode)
        conn.standup()

        # Store config file path for decorator to use
        if config_file:
            self.module_config_file = config_file
        else:
            # Default module config for testing
            self.module_config = {
                "spatialmemory": {
                    "new_memory": True,
                },
                "vfhpurepursuitplanner": {
                    "max_linear_vel": 0.8,
                },
            }

        # Camera params - modules can access via robot.camera_*
        self.camera_intrinsics = [819.553492, 820.646595, 625.284099, 336.808987]
        self.camera_pitch = np.deg2rad(0)
        self.camera_height = 0.44

        # Initialize base robot - modules are auto-created by decorator
        super().__init__(conn)

        # Skills can be added separately
        self.skill_library = SkillLibrary()

    def get_pose(self) -> dict:
        """Get current robot pose."""
        if has_capability(self.conn, Odometry):
            return cast(Odometry, self.conn).get_pose()
        raise RuntimeError("Connection doesn't support Odometry capability")

    def explore(self, stop_event: Optional[threading.Event] = None) -> bool:
        """Start autonomous frontier exploration.

        Args:
            stop_event: Optional event to signal stop

        Returns:
            bool: True if exploration completed successfully
        """
        # Access the WavefrontFrontierExplorer module
        explorer = self.get_module(WavefrontFrontierExplorer)
        if explorer:
            return explorer.explore(stop_event=stop_event)
        return False
