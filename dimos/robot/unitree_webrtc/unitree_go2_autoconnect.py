#!/usr/bin/env python3

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


from functools import cached_property
import logging
import os
import time
from typing import Optional

from reactivex import Observable
from reactivex.disposable import CompositeDisposable

from dimos.core.blueprints import autoconnect
from dimos.core.dimos import Dimos
from dimos.core.resource import Resource
from dimos.core.transport import LCMTransport
from dimos.mapping.types import LatLon
from dimos.msgs.geometry_msgs import PoseStamped, Twist
from dimos.msgs.sensor_msgs import Image
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.perception.spatial_perception import SpatialMemory, spatial_memory
from dimos.protocol import pubsub
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.robot.foxglove_bridge import foxglove_bridge
from dimos.robot.unitree_webrtc.unitree_go2 import ConnectionModule, connection
from dimos.utils.monitoring import utilization
from dimos.web.websocket_vis.websocket_vis_module import websocket_vis
from dimos.navigation.global_planner import astar_planner
from dimos.navigation.local_planner.holonomic_local_planner import (
    holonomic_local_planner,
)
from dimos.navigation.bt_navigator.navigator import (
    NavigatorState,
    behavior_tree_navigator,
    BehaviorTreeNavigator,
)
from dimos.navigation.frontier_exploration import (
    WavefrontFrontierExplorer,
    wavefront_frontier_explorer,
)
from dimos.robot.unitree_webrtc.type.map import mapper
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.robot.unitree_webrtc.depth_module import depth_module
from dimos.skills.skills import AbstractRobotSkill, SkillLibrary
from dimos.utils.logging_config import setup_logger
from dimos.perception.object_tracker import object_tracking
from dimos.robot.robot import UnitreeRobot
from dimos.types.robot_capabilities import RobotCapability


logger = setup_logger(__file__, level=logging.INFO)


class UnitreeGo2(UnitreeRobot, Resource):
    _dimos: Dimos
    _disposables: CompositeDisposable = CompositeDisposable()

    def __init__(
        self,
        ip: Optional[str],
        output_dir: str = None,
        websocket_port: int = 7779,
        skill_library: Optional[SkillLibrary] = None,
        connection_type: Optional[str] = "webrtc",
    ):
        super().__init__()
        self.ip = ip
        self.connection_type = connection_type or "webrtc"
        if ip is None and self.connection_type == "webrtc":
            self.connection_type = "fake"  # Auto-enable playback if no IP provided
        self.output_dir = output_dir or os.path.join(os.getcwd(), "assets", "output")
        self.websocket_port = websocket_port
        self.lcm = LCM()

        if skill_library is None:
            skill_library = MyUnitreeSkills()
        self.skill_library = skill_library

        self.capabilities = [RobotCapability.LOCOMOTION, RobotCapability.VISION]

        self._setup_directories()

    def _setup_directories(self):
        """Setup directories for spatial memory storage."""
        os.makedirs(self.output_dir, exist_ok=True)
        logger.info(f"Robot outputs will be saved to: {self.output_dir}")

        # Initialize memory directories
        self.memory_dir = os.path.join(self.output_dir, "memory")
        os.makedirs(self.memory_dir, exist_ok=True)

        # Initialize spatial memory properties
        self.spatial_memory_dir = os.path.join(self.memory_dir, "spatial_memory")
        self.db_path = os.path.join(self.spatial_memory_dir, "chromadb_data")
        self.visual_memory_path = os.path.join(self.spatial_memory_dir, "visual_memory.pkl")

        # Create spatial memory directories
        os.makedirs(self.spatial_memory_dir, exist_ok=True)
        os.makedirs(self.db_path, exist_ok=True)

    def start(self):
        self.lcm.start()

        min_height = 0.3 if self.connection_type == "mujoco" else 0.15
        gt_depth_scale = 1.0 if self.connection_type == "mujoco" else 0.5

        basic_robot = autoconnect(
            connection(self.ip, connection_type=self.connection_type),
            mapper(voxel_size=0.5, global_publish_interval=2.5, min_height=min_height),
            astar_planner(),
            holonomic_local_planner(),
            behavior_tree_navigator(),
            wavefront_frontier_explorer(),
            websocket_vis(self.websocket_port),
            foxglove_bridge(),
        )

        enhanced_robot = autoconnect(
            basic_robot,
            spatial_memory(
                db_path=self.db_path,
                visual_memory_path=self.visual_memory_path,
                output_dir=self.spatial_memory_dir,
            ),
            object_tracking(frame_id="camera_link"),
            depth_module(gt_depth_scale=gt_depth_scale),
            utilization(),
        )

        self._dimos = enhanced_robot.with_transports(
            {
                ("color_image", Image): LCMTransport("/go2/color_image", Image),
                ("depth_image", Image): LCMTransport("/go2/depth_image", Image),
                ("camera_pose", PoseStamped): LCMTransport("/go2/camera_pose", PoseStamped),
                ("camera_info", CameraInfo): LCMTransport("/go2/camera_info", CameraInfo),
            }
        ).build()

        self._start_skills()

    def stop(self):
        self._disposables.dispose()
        self._dimos.stop()
        self.lcm.stop()

    def _start_skills(self):
        # Initialize skills after connection is established
        if self.skill_library is not None:
            for skill in self.skill_library:
                if isinstance(skill, AbstractRobotSkill):
                    self.skill_library.create_instance(skill.__name__, robot=self)
            if isinstance(self.skill_library, MyUnitreeSkills):
                self.skill_library._robot = self
                self.skill_library.init()
                self.skill_library.initialize_skills()

    def get_single_rgb_frame(self, timeout: float = 2.0) -> Image:
        topic = Topic("/go2/color_image", Image)
        return self.lcm.wait_for_message(topic, timeout=timeout)

    def move(self, twist: Twist, duration: float = 0.0):
        self._dimos.get_instance(ConnectionModule).move(twist, duration)

    def explore(self) -> bool:
        return self._dimos.get_instance(WavefrontFrontierExplorer).explore()

    def navigate_to(self, pose: PoseStamped, blocking: bool = True):
        logger.info(
            f"Navigating to pose: ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f})"
        )
        self._dimos.get_instance(BehaviorTreeNavigator).set_goal(pose)
        time.sleep(1.0)

        if blocking:
            while (
                self._dimos.get_instance(BehaviorTreeNavigator).get_state()
                == NavigatorState.FOLLOWING_PATH
            ):
                time.sleep(0.25)

            time.sleep(1.0)
            if not self._dimos.get_instance(BehaviorTreeNavigator).is_goal_reached():
                logger.info("Navigation was cancelled or failed")
                return False
            else:
                logger.info("Navigation goal reached")
                return True

        return True

    def stop_exploration(self) -> bool:
        self._dimos.get_instance(BehaviorTreeNavigator).cancel_goal()
        return self._dimos.get_instance(WaveFrontFrontierExplorer).stop_exploration()

    def is_exploration_active(self) -> bool:
        return self._dimos.get_instance(WaveFrontFrontierExplorer).is_exploration_active()

    def cancel_navigation(self) -> bool:
        return self._dimos.get_instance(BehaviorTreeNavigator).cancel_goal()

    @property
    def spatial_memory(self) -> Optional[SpatialMemory]:
        return self._dimos.get_instance(SpatialMemory)

    @cached_property
    def gps_position_stream(self) -> Observable[LatLon]:
        return self._dimos.get_instance(ConnectionModule).gps_location.transport.pure_observable()

    def get_odom(self) -> PoseStamped:
        return self._dimos.get_instance(ConnectionModule).get_odom()

    def navigate_to_object(self, pose, blocking=True):
        pass


def main():
    ip = os.getenv("ROBOT_IP")
    connection_type = os.getenv("CONNECTION_TYPE", "webrtc")

    pubsub.lcm.autoconf()

    robot = UnitreeGo2(ip=ip, websocket_port=7779, connection_type=connection_type)
    robot.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
