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
import threading
import time

import pytest

from dimos import core
from dimos.core import Module, Out, rpc
from dimos.msgs.geometry_msgs import Pose, PoseStamped, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.protocol import pubsub
from dimos.robot.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.robot.global_planner import AstarPlanner
from dimos.robot.local_planner.vfh_local_planner import VFHPurePursuitPlanner
from dimos.robot.unitree_webrtc.multiprocess.unitree_go2 import ConnectionModule, ControlModule
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map
from dimos.utils.logging_config import setup_logger

logger = setup_logger("test_unitree_go2_cpu_module")

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
    def send_explore_sequence(self):
        """Send a sequence of exploration commands."""

        def send_commands():
            commands = [
                (0.5, 0.0, 0.0),
                (0.0, 0.0, 0.3),
                (0.5, 0.0, 0.0),
                (0.0, 0.0, -0.3),
                (0.3, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            ]

            for x, y, yaw in commands:
                self.send_move_command(x, y, yaw)
                time.sleep(0.5)

        thread = threading.Thread(target=send_commands, daemon=True)
        thread.start()

    @rpc
    def get_command_count(self) -> int:
        """Get number of commands sent."""
        return len(self.commands_sent)


@pytest.mark.module
class TestUnitreeGo2CPUModule:
    @pytest.mark.asyncio
    async def test_unitree_go2_connection_explore_movement(self):
        """Test UnitreeGo2 modules with FakeRTC for exploration and movement without spatial memory."""

        # Start Dask
        dimos = core.start(4)

        try:
            # Deploy ConnectionModule with FakeRTC (uses test data)
            connection = dimos.deploy(
                ConnectionModule, "127.0.0.1"
            )  # IP doesn't matter for FakeRTC

            # Configure LCM transports
            connection.lidar.transport = core.LCMTransport("/lidar", LidarMessage)
            connection.odom.transport = core.LCMTransport("/odom", PoseStamped)
            connection.video.transport = core.LCMTransport("/video", Image)

            # Deploy Map module
            mapper = dimos.deploy(Map, voxel_size=0.5, global_publish_interval=2.5)
            mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)
            mapper.lidar.connect(connection.lidar)

            # Deploy Local planner
            local_planner = dimos.deploy(
                VFHPurePursuitPlanner,
                get_costmap=connection.get_local_costmap,
            )
            local_planner.odom.connect(connection.odom)
            local_planner.movecmd.transport = core.LCMTransport("/move", Vector3)
            connection.movecmd.connect(local_planner.movecmd)

            # Deploy Global planner
            global_planner = dimos.deploy(
                AstarPlanner,
                get_costmap=mapper.costmap,
                get_robot_pos=connection.get_pos,
                set_local_nav=local_planner.navigate_path_local,
            )
            global_planner.path.transport = core.pLCMTransport("/global_path")

            # Deploy Control module for testing
            ctrl = dimos.deploy(ControlModule)
            ctrl.plancmd.transport = core.LCMTransport("/global_target", Pose)
            global_planner.target.connect(ctrl.plancmd)

            # Deploy movement control module
            movement = dimos.deploy(MovementControlModule)
            movement.movecmd.transport = core.LCMTransport("/test_move", Vector3)

            # Connect movement commands to connection module as well
            connection.movecmd.connect(movement.movecmd)

            # Start all modules
            mapper.start()
            connection.start()
            local_planner.start()
            global_planner.start()

            logger.info("All modules started")

            # Wait for initialization
            await asyncio.sleep(3)

            # Test get methods
            odom = connection.get_odom()
            assert odom is not None, "Should get odometry"
            logger.info(f"Got odometry: position={odom.position}")

            pos = connection.get_pos()
            assert pos is not None, "Should get position"
            logger.info(f"Got position: {pos}")

            local_costmap = connection.get_local_costmap()
            assert local_costmap is not None, "Should get local costmap"
            logger.info(f"Got local costmap with shape: {local_costmap.grid.shape}")

            # Test mapper costmap
            global_costmap = mapper.costmap()
            assert global_costmap is not None, "Should get global costmap"
            logger.info(f"Got global costmap with shape: {global_costmap.grid.shape}")

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

            # Test explore sequence
            logger.info("Testing explore sequence")
            movement.send_explore_sequence()

            # Wait for sequence to complete
            await asyncio.sleep(4)

            # Verify explore commands were sent
            final_count = movement.get_command_count()
            assert final_count == 9, f"Expected 9 total commands, got {final_count}"

            # Test frontier exploration setup
            frontier_explorer = WavefrontFrontierExplorer(
                set_goal=global_planner.set_goal,
                get_costmap=mapper.costmap,
                get_robot_pos=connection.get_pos,
            )
            logger.info("Frontier explorer created successfully")

            # Start control module to trigger planning
            ctrl.start()
            logger.info("Control module started - will trigger planning in 4 seconds")

            await asyncio.sleep(5)

            logger.info("All UnitreeGo2 CPU module tests passed!")

        finally:
            dimos.close()
            logger.info("Closed Dask cluster")


if __name__ == "__main__":
    pytest.main(["-v", "-s", __file__])
