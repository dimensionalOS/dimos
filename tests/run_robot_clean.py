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

import os
from dimos.perception.object_tracker import ObjectTrackingStream
from dimos.robot.module_utils import robot_capability
from dimos.robot.unitree_webrtc.connection import UnitreeWebRTCConnection
from dimos.robot.robot_clean import Robot
from dimos.perception.spatial_perception import SpatialMemory
from dimos.perception.person_tracker import PersonTrackingStream
from dimos.robot.local_planner.vfh_local_planner import VFHPurePursuitPlanner
from dimos.robot.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.robot.global_planner.planner import AstarPlanner
from dimos.skills.skills import SkillLibrary
from dimos.robot.unitree_webrtc.unitree_skills import MyUnitreeSkills
from dimos.skills.skills import AbstractRobotSkill
from dimos.perception.object_tracker import ObjectTrackingStream
import time
from dimos.agents.claude_agent import ClaudeAgent
from dimos.web.robot_web_interface import RobotWebInterface
import numpy as np
import threading
from dimos.skills.navigation import Explore
from dimos.robot.unitree_webrtc.unitree_go2_improved import UnitreeGo2 as UnitreeGo2Improved
from dimos.types.vector import Vector
from dimos.web.websocket_vis.server import WebsocketVis
import reactivex.operators as ops
import reactivex as rx
import asyncio
from typing import Optional
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
    ):
        # Create connection
        conn = UnitreeWebRTCConnection(ip=os.getenv("ROBOT_IP"), mode="normal")

        # Stand up the robot
        print("Standing up...")
        conn.standup()

        self.camera_intrinsics = [819.553492, 820.646595, 625.284099, 336.808987]
        self.camera_pitch = np.deg2rad(0)  # negative for downward pitch
        self.camera_height = 0.44  # meters

        self.skill_library = MyUnitreeSkills()

        super().__init__(conn)

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


robot = UnitreeGo2()

for module_name, module in robot._modules.items():
    print(f"  - {module_name}: {module.__class__.__name__}")
robot.skill_library.add(Explore)
robot.skill_library.create_instance("Explore", robot=robot)
streams = {
    "unitree_video": robot.video_stream(),
}
web_interface = RobotWebInterface(port=5555, **streams)

global_planner = robot.get_module(AstarPlanner)
# Initialize WebSocket visualization
websocket_vis = WebsocketVis()
websocket_vis.start()
websocket_vis.connect(global_planner.vis_stream())


def msg_handler(msgtype, data):
    if msgtype == "click":
        print(f"Received click at position: {data['position']}")

        try:
            print("Setting goal...")

            # Instead of disabling visualization, make it timeout-safe
            original_vis = global_planner.vis

            def safe_vis(name, drawable):
                """Visualization wrapper that won't block on timeouts"""
                try:
                    # Use a separate thread for visualization to avoid blocking
                    def vis_update():
                        try:
                            original_vis(name, drawable)
                        except Exception as e:
                            print(f"Visualization update failed (non-critical): {e}")

                    vis_thread = threading.Thread(target=vis_update)
                    vis_thread.daemon = True
                    vis_thread.start()
                    # Don't wait for completion - let it run asynchronously
                except Exception as e:
                    print(f"Visualization setup failed (non-critical): {e}")

            global_planner.vis = safe_vis
            global_planner.set_goal(Vector(data["position"]))
            global_planner.vis = original_vis

            print("Goal set successfully")
        except Exception as e:
            print(f"Error setting goal: {e}")
            import traceback

            traceback.print_exc()


def threaded_msg_handler(msgtype, data):
    print(f"Processing message: {msgtype}")

    # Create a dedicated event loop for goal setting to avoid conflicts
    def run_with_dedicated_loop():
        try:
            # Use asyncio.run which creates and manages its own event loop
            # This won't conflict with the robot's or websocket's event loops
            async def async_msg_handler():
                msg_handler(msgtype, data)

            asyncio.run(async_msg_handler())
            print("Goal setting completed successfully")
        except Exception as e:
            print(f"Error in goal setting thread: {e}")
            import traceback

            traceback.print_exc()

    thread = threading.Thread(target=run_with_dedicated_loop)
    thread.daemon = True
    thread.start()


websocket_vis.msg_handler = threaded_msg_handler


def newmap(msg):
    return ["costmap", robot.map.costmap.smudge()]


websocket_vis.connect(global_planner.map_stream.pipe(ops.map(newmap)))
websocket_vis.connect(
    robot.conn.odom_stream().pipe(ops.map(lambda pos: ["robot_pos", pos.pos.to_2d()]))
)


agent = ClaudeAgent(
    dev_name="test_agent",
    # input_query_stream=stt_node.emit_text(),
    input_query_stream=web_interface.query_stream,
    skills=robot.skill_library,
    system_query="You are a helpful robot",
    model_name="claude-3-7-sonnet-latest",
    thinking_budget_tokens=0,
)

web_thread = threading.Thread(target=web_interface.run)
web_thread.daemon = True
web_thread.start()

while True:
    time.sleep(0.01)
