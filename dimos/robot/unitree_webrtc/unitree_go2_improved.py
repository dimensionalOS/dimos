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

"""Improved UnitreeGo2 implementation using capability-based architecture."""

from typing import Optional, List
import os
import numpy as np
from dimos.skills.skills import AbstractRobotSkill

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
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
import threading
import os


@robot_capability(
    SpatialMemory,
    PersonTrackingStream,
    ObjectTrackingStream,
    AstarPlanner,
    VFHPurePursuitPlanner,
    WavefrontFrontierExplorer,
)
class UnitreeGo2(Robot):
    """Unitree Go2 robot with all standard capabilities."""

    def __init__(
        self,
        mode: str = "normal",
        output_dir: str = os.path.join(os.getcwd(), "assets", "output"),
        skill_library: Optional[SkillLibrary] = None,
        # Module-specific parameters
        spatial_memory_collection: str = "spatial_memory",
        new_memory: bool = True,
        enable_perception: bool = True,
        camera_intrinsics: Optional[List[float]] = None,
        camera_pitch: Optional[float] = None,
        camera_height: Optional[float] = None,
        max_linear_vel: float = 0.7,
        max_angular_vel: float = 0.65,
        min_frontier_size: int = 10,
    ):
        """Initialize Unitree Go2 robot with improved architecture.

        Args:
            ip: IP address of the robot
            mode: Robot mode (ai, normal)
            output_dir: Directory for output files
            skill_library: Skill library instance
            spatial_memory_collection: Collection name for spatial memory
            new_memory: Whether to create new spatial memory
            enable_perception: Whether to enable perception modules
            camera_intrinsics: Camera intrinsic parameters [fx, fy, cx, cy]
            camera_pitch: Camera pitch angle in radians
            camera_height: Camera height from ground in meters
            max_linear_vel: Maximum linear velocity for local planner
            max_angular_vel: Maximum angular velocity for local planner
            min_frontier_size: Minimum frontier size for exploration
        """
        # Create connection
        conn = UnitreeWebRTCConnection(ip=os.getenv("ROBOT_IP"), mode=mode)

        # Stand up the robot
        print("Standing up...")
        conn.standup()

        # Default camera parameters for Go2
        if camera_intrinsics is None:
            camera_intrinsics = [819.553492, 820.646595, 625.284099, 336.808987]
        if camera_pitch is None:
            camera_pitch = np.deg2rad(0)
        if camera_height is None:
            camera_height = 0.44

        # Store camera parameters as instance attributes
        self.camera_intrinsics = camera_intrinsics
        self.camera_pitch = camera_pitch
        self.camera_height = camera_height

        # Create skill library if not provided
        if skill_library is None:
            skill_library = MyUnitreeSkills()

        super().__init__(conn)

        self.skill_library = skill_library

        if self.skill_library is not None:
            for skill in self.skill_library:
                if isinstance(skill, AbstractRobotSkill):
                    self.skill_library.create_instance(skill.__name__, robot=self)
            if isinstance(self.skill_library, MyUnitreeSkills):
                self.skill_library._robot = self
                self.skill_library.init()
                self.skill_library.initialize_skills()

        from dimos.robot.unitree_webrtc.type.map import Map  # local import
        import reactivex

        # TODO: This subscription should not be done here, handled in whatever module needs it
        self.map = Map(voxel_size=0.2)
        # Subscribe immediately; keep disposable so GC doesn’t cancel it
        self._map_subscription = (
            self.conn.lidar_stream()
            .pipe(reactivex.operators.map(lambda msg: self.map.add_frame(msg)))
            .subscribe(lambda _: None)
        )

        # Module configuration will be passed to the decorator
        # These parameters will be available during module initialization
        self._module_config = {
            # SpatialMemory config
            "collection_name": spatial_memory_collection,
            "new_memory": new_memory,
            "output_dir": output_dir,
            "visual_memory_path": os.path.join(output_dir, "visual_memory.pkl"),
            "db_path": os.path.join(output_dir, "chromadb_data"),
            "enable_perception": enable_perception,
            # Camera config for trackers
            "camera_intrinsics": camera_intrinsics,
            "camera_pitch": camera_pitch,
            "camera_height": camera_height,
            # Planner config
            "max_linear_vel": max_linear_vel,
            "max_angular_vel": max_angular_vel,
            "robot_width": 0.36,  # Unitree Go2 width
            "robot_length": 0.6,  # Unitree Go2 length
            # Explorer config
            "min_frontier_size": min_frontier_size,
        }

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
