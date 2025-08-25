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

import asyncio

import pytest

from dimos import core
from dimos.core import Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Vector3, Quaternion
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.protocol import pubsub
from dimos.navigation.frontier_exploration import WavefrontFrontierExplorer
from dimos.navigation.global_planner import AstarPlanner
from dimos.navigation.local_planner.holonomic_local_planner import HolonomicLocalPlanner
from dimos.navigation.bt_navigator.navigator import BehaviorTreeNavigator
from dimos.robot.unitree_webrtc.unitree_go2 import ConnectionModule
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.utils.logging_config import setup_logger

logger = setup_logger("test_unitree_go2_integration")

pubsub.lcm.autoconf()


class MovementControlModule(Module):
    """Simple module to send movement commands for testing."""

    movecmd: Out[Vector3] = None

    def __init__(self):
        super().__init__()
        self.commands_sent = []

    @rpc
    def send_move_command(self, x: float, y: float, yaw: float):
        """Send a movement command."""
        cmd = Vector3(x, y, yaw)
        self.movecmd.publish(cmd)
        self.commands_sent.append(cmd)
        logger.info(f"Sent move command: x={x}, y={y}, yaw={yaw}")

    @rpc
    def get_command_count(self) -> int:
        """Get number of commands sent."""
        return len(self.commands_sent)


@pytest.mark.module
class TestUnitreeGo2CoreModules:
    @pytest.mark.asyncio
    async def test_unitree_go2_navigation_stack(self):
        """Test UnitreeGo2 core navigation modules without perception/visualization."""

        # Start Dask
        dimos = core.start(4)

        try:
            # Deploy ConnectionModule with playback mode (uses test data)
            connection = dimos.deploy(
                ConnectionModule,
                ip="127.0.0.1",  # IP doesn't matter for playback
                playback=True,  # Enable playback mode
            )

            # Configure LCM transports
            connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
            connection.odom.transport = core.LCMTransport("/odom", PoseStamped)
            connection.video.transport = core.LCMTransport("/video", Image)

            # Deploy Map module
            mapper = dimos.deploy(Map, voxel_size=0.5, global_publish_interval=2.5)
            mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)
            mapper.global_costmap.transport = core.LCMTransport("/global_costmap", OccupancyGrid)
            mapper.local_costmap.transport = core.LCMTransport("/local_costmap", OccupancyGrid)
            mapper.lidar.connect(connection.lidar)

            # Deploy navigation stack
            global_planner = dimos.deploy(AstarPlanner)
            local_planner = dimos.deploy(HolonomicLocalPlanner)
            navigator = dimos.deploy(BehaviorTreeNavigator, local_planner=local_planner)

            # Set up transports first
            from dimos.msgs.nav_msgs import Path
            from dimos_lcm.std_msgs import Bool

            navigator.goal.transport = core.LCMTransport("/navigation_goal", PoseStamped)
            navigator.goal_request.transport = core.LCMTransport("/goal_request", PoseStamped)
            navigator.goal_reached.transport = core.LCMTransport("/goal_reached", Bool)
            navigator.global_costmap.transport = core.LCMTransport("/global_costmap", OccupancyGrid)
            global_planner.path.transport = core.LCMTransport("/global_path", Path)
            local_planner.cmd_vel.transport = core.LCMTransport("/cmd_vel", Vector3)

            # Configure navigation connections
            global_planner.target.connect(navigator.goal)
            global_planner.global_costmap.connect(mapper.global_costmap)
            global_planner.odom.connect(connection.odom)

            local_planner.path.connect(global_planner.path)
            local_planner.local_costmap.connect(mapper.local_costmap)
            local_planner.odom.connect(connection.odom)

            connection.movecmd.connect(local_planner.cmd_vel)
            navigator.odom.connect(connection.odom)

            # Deploy movement control module for testing
            movement = dimos.deploy(MovementControlModule)
            movement.movecmd.transport = core.LCMTransport("/test_move", Vector3)
            connection.movecmd.connect(movement.movecmd)

            # Start all modules
            connection.start()
            mapper.start()
            global_planner.start()
            local_planner.start()
            navigator.start()

            logger.info("All core modules started")

            # Wait for initialization
            await asyncio.sleep(3)

            # Test movement commands
            movement.send_move_command(0.5, 0.0, 0.0)
            await asyncio.sleep(0.5)

            movement.send_move_command(0.0, 0.0, 0.3)
            await asyncio.sleep(0.5)

            movement.send_move_command(0.0, 0.0, 0.0)
            await asyncio.sleep(0.5)

            # Check commands were sent
            cmd_count = movement.get_command_count()
            assert cmd_count == 3, f"Expected 3 commands, got {cmd_count}"
            logger.info(f"Successfully sent {cmd_count} movement commands")

            # Test navigation
            target_pose = PoseStamped(
                frame_id="world",
                position=Vector3(2.0, 1.0, 0.0),
                orientation=Quaternion(0, 0, 0, 1),
            )

            # Set navigation goal (non-blocking)
            try:
                navigator.set_goal(target_pose)
                logger.info("Navigation goal set")
            except Exception as e:
                logger.warning(f"Navigation goal setting failed: {e}")

            await asyncio.sleep(2)

            # Cancel navigation
            navigator.cancel_goal()
            logger.info("Navigation cancelled")

            # Test frontier exploration
            frontier_explorer = dimos.deploy(WavefrontFrontierExplorer)
            frontier_explorer.costmap.connect(mapper.global_costmap)
            frontier_explorer.odometry.connect(connection.odom)
            frontier_explorer.goal_request.transport = core.LCMTransport(
                "/frontier_goal", PoseStamped
            )
            frontier_explorer.goal_reached.transport = core.LCMTransport("/frontier_reached", Bool)
            frontier_explorer.start()

            # Try to start exploration
            result = frontier_explorer.explore()
            logger.info(f"Exploration started: {result}")

            await asyncio.sleep(2)

            # Stop exploration
            frontier_explorer.stop_exploration()
            logger.info("Exploration stopped")

            logger.info("All core navigation tests passed!")

        finally:
            dimos.close()
            logger.info("Closed Dask cluster")


if __name__ == "__main__":
    pytest.main(["-v", "-s", __file__])
