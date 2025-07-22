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

"""Heavy version of Unitree Go2 with GPU-required modules."""

import asyncio
from typing import List, Optional

import numpy as np
from reactivex import Observable
from reactivex.disposable import CompositeDisposable
from reactivex.scheduler import ThreadPoolScheduler

from dimos.perception.object_tracker import ObjectTrackingStream
from dimos.perception.person_tracker import PersonTrackingStream
from dimos.robot.unitree_webrtc.multiprocess.unitree_go2 import UnitreeGo2Light
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.skills.skills import AbstractRobotSkill, SkillLibrary
from dimos.types.robot_capabilities import RobotCapability
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler

logger = setup_logger("dimos.robot.unitree_webrtc.multiprocess.unitree_go2_heavy")


class UnitreeGo2Heavy(UnitreeGo2Light):
    """Heavy version of Unitree Go2 with additional GPU-required modules.

    This class extends UnitreeGo2Light with:
    - Spatial memory with ChromaDB
    - Person tracking stream
    - Object tracking stream
    - Skill library integration
    - Full perception capabilities
    """

    def __init__(
        self,
        ip: str,
        skill_library: Optional[SkillLibrary] = None,
        robot_capabilities: Optional[List[RobotCapability]] = None,
        spatial_memory_collection: str = "spatial_memory",
        new_memory: bool = True,
        enable_perception: bool = True,
        pool_scheduler: Optional[ThreadPoolScheduler] = None,
    ):
        """Initialize Unitree Go2 Heavy with full capabilities.

        Args:
            ip: IP address of the robot
            output_dir: Directory for output files
            skill_library: Skill library instance
            robot_capabilities: List of robot capabilities
            spatial_memory_collection: Collection name for spatial memory
            new_memory: Whether to create new spatial memory
            enable_perception: Whether to enable perception streams
            pool_scheduler: Thread pool scheduler for async operations
        """
        super().__init__(ip)

        self.enable_perception = enable_perception
        self.disposables = CompositeDisposable()
        self.pool_scheduler = pool_scheduler if pool_scheduler else get_scheduler()

        # Initialize capabilities
        self.capabilities = robot_capabilities or [
            RobotCapability.LOCOMOTION,
            RobotCapability.VISION,
            RobotCapability.AUDIO,
        ]

        # Camera configuration for Unitree Go2
        self.camera_intrinsics = [819.553492, 820.646595, 625.284099, 336.808987]
        self.camera_pitch = np.deg2rad(0)  # negative for downward pitch
        self.camera_height = 0.44  # meters

        # Initialize skill library
        if skill_library is None:
            skill_library = MyUnitreeSkills()
        self.skill_library = skill_library

        # Initialize spatial memory module (will be deployed after connection is established)
        self._video_stream = None
        self.new_memory = new_memory

        # Tracking streams (initialized after start)
        self.person_tracker = None
        self.object_tracker = None
        self.person_tracking_stream = None
        self.object_tracking_stream = None

    async def start(self):
        """Start the robot modules and initialize heavy components."""
        # First start the lightweight components
        await super().start()

        await asyncio.sleep(0.5)

        # Now we have connection publishing to LCM, initialize video stream
        self._video_stream = self.get_video_stream(fps=10)  # Lower FPS for processing

        if self.enable_perception:
            # Initialize person and object tracking
            self.person_tracker = PersonTrackingStream(
                camera_intrinsics=self.camera_intrinsics,
                camera_pitch=self.camera_pitch,
                camera_height=self.camera_height,
            )
            self.object_tracker = ObjectTrackingStream(
                camera_intrinsics=self.camera_intrinsics,
                camera_pitch=self.camera_pitch,
                camera_height=self.camera_height,
            )

            # Create tracking streams
            self.person_tracking_stream = self.person_tracker.create_stream(self._video_stream)
            self.object_tracking_stream = self.object_tracker.create_stream(self._video_stream)

            logger.info("Person and object tracking initialized")
        else:
            logger.info("Perception disabled or video stream unavailable")

        # Initialize skills with robot reference
        if self.skill_library is not None:
            for skill in self.skill_library:
                if isinstance(skill, AbstractRobotSkill):
                    self.skill_library.create_instance(skill.__name__, robot=self)
            if isinstance(self.skill_library, MyUnitreeSkills):
                self.skill_library._robot = self
                self.skill_library.init()
                self.skill_library.initialize_skills()

        logger.info("UnitreeGo2Heavy initialized with all modules")

    @property
    def video_stream(self) -> Optional[Observable]:
        """Get the robot's video stream.

        Returns:
            Observable video stream or None if not available
        """
        return self._video_stream

    def get_skills(self):
        """Get the robot's skill library.

        Returns:
            The robot's skill library for adding/managing skills
        """
        return self.skill_library

    def has_capability(self, capability: RobotCapability) -> bool:
        """Check if the robot has a specific capability.

        Args:
            capability: The capability to check for

        Returns:
            bool: True if the robot has the capability
        """
        return capability in self.capabilities

    def cleanup(self):
        """Clean up resources used by the robot."""
        # Dispose of reactive resources
        if self.disposables:
            self.disposables.dispose()

        # Clean up tracking streams
        if self.person_tracker:
            self.person_tracker = None
        if self.object_tracker:
            self.object_tracker = None

        logger.info("UnitreeGo2Heavy cleanup completed")
