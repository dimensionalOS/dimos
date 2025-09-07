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


import os
import sys
import time
import logging
import argparse

from dimos.hardware.zed_camera_single import ZedModuleSingle

sys.path.append(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
)

from dimos import core
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.local_splice_map import LocalSpliceMap
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3, Quaternion
from dimos.protocol import pubsub
from dimos.protocol.pubsub.lcmpubsub import LCM
from dimos.protocol.tf import TF
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__, level=logging.INFO)


class StereoMapper:
    """
    Stereo mapping system using ZED camera
    """

    def __init__(self, websocket_port: int = 7779):
        """
        Initialize StereoMapper with ZED camera.

        Args:
            websocket_port: Port for Foxglove websocket connection
        """
        self.websocket_port = websocket_port
        self.lcm = LCM()
        self.tf = None

        self.dimos = None
        self.zed_module = None
        self.mapper = None
        self.foxglove_bridge = None

        self.voxel_size = 0.1  # ZED spatial mapping voxel size
        self.map_publish_interval = 2.5  # seconds

        logger.info(f"StereoMapper initialized - websocket port {websocket_port}")

    def start(self):
        logger.info("Starting StereoMapper system...")

        self.dimos = core.start(1, memory_limit="30GiB")

        # Initialize TF after Dimos is started
        self.tf = TF()

        self._deploy_zed_and_connection()
        self._deploy_mapping()
        self._deploy_visualization()

        self._start_modules()

        self.lcm.start()

        logger.info("=" * 60)
        logger.info("StereoMapper started successfully!")
        logger.info(f"Foxglove visualization: http://localhost:{self.websocket_port}")
        logger.info("")
        logger.info("Publishing topics:")
        logger.info("  /lidar - Pointcloud from ZED (as LidarMessage)")
        logger.info("  /odom - Camera pose from ZED visual odometry")
        logger.info("  /global_map")
        logger.info("  /global_costmap - Navigation costmap")
        logger.info("")
        logger.info("Foxglove display frame: 'world'")
        logger.info("TF chain: world → base_link → camera_link")
        logger.info("=" * 60)

    def _deploy_zed_and_connection(self):
        logger.info("Deploying ZED camera module with spatial mapping...")

        self.zed_module = self.dimos.deploy(
            ZedModuleSingle,
            voxel_size=self.voxel_size,
            map_publish_interval=self.map_publish_interval,
        )

        self.zed_module.pointcloud_msg.transport = core.LCMTransport("/lidar", LidarMessage)
        self.zed_module.pose.transport = core.LCMTransport("/odom", PoseStamped)
        self.zed_module.pose.subscribe(self._publish_tf)

        logger.info("✓ ZED camera module deployed and configured")

    def _publish_tf(self, msg: PoseStamped):
        # Only publish TF if we have a valid pose (ZED tracking is working)
        if msg is None:
            logger.debug("No valid pose from ZED yet, skipping TF publish")
            return

        # Check if position is valid (not None or all zeros)
        if not hasattr(msg, "position") or msg.position is None:
            logger.debug("Invalid pose position, skipping TF publish")
            return

        if not msg.frame_id or msg.frame_id == "":
            msg.frame_id = "world"

        # Publish world -> base_link transform from ZED pose
        # The ZED gives us the camera pose in world, but we treat it as base_link for simplicity
        self.tf.publish(Transform.from_pose("base_link", msg))

        # base_link -> camera_link (physical offset of camera on robot)
        # For a handheld or stationary camera, this can be identity
        camera_link = Transform(
            translation=Vector3(0.0, 0.0, 0.0),  # Camera IS the base for now
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),  # No rotation
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=time.time(),
        )
        self.tf.publish(camera_link)

        # No need for camera_optical transform - ZED already outputs in ROS frame

    def _deploy_mapping(self):
        logger.info("Deploying mapping module...")

        self.mapper = self.dimos.deploy(
            LocalSpliceMap,
            cylinder_radius=3.0,
            voxel_size=self.voxel_size,
            cost_resolution=self.voxel_size,
            global_publish_interval=self.map_publish_interval,
            frame_id="world",
        )

        # Configure transports
        self.mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)
        self.mapper.global_costmap.transport = core.LCMTransport("/global_costmap", OccupancyGrid)
        self.mapper.local_costmap.transport = core.LCMTransport("/local_costmap", OccupancyGrid)

        # Connect lidar input - connect directly to ZED module's pointcloud output
        self.mapper.lidar.connect(self.zed_module.pointcloud_msg)

        # Connect odometry input to mapper
        self.mapper.odom.connect(self.zed_module.pose)

        logger.info("✓ Mapping module deployed and connected to ZED with odometry")

    def _deploy_visualization(self):
        logger.info("Deploying visualization...")

        self.foxglove_bridge = FoxgloveBridge()

        logger.info(f"✓ Foxglove bridge ready on port {self.websocket_port}")

    def _start_modules(self):
        logger.info("Starting all modules...")

        logger.info("  Starting ZED module...")
        self.zed_module.start()
        time.sleep(3)  # Give ZED time to initialize

        logger.info("  Starting mapper module...")
        self.mapper.start()

        logger.info("  Starting Foxglove bridge...")
        self.foxglove_bridge.start()

        logger.info("✓ All modules started")

    def stop(self):
        logger.info("Stopping StereoMapper...")

        if self.zed_module:
            self.zed_module.stop()

        if self.foxglove_bridge:
            self.foxglove_bridge.stop()

        self.lcm.stop()

        if self.dimos:
            try:
                core.stop(self.dimos)
            except:
                pass

        logger.info("StereoMapper stopped")


def main():
    parser = argparse.ArgumentParser(description="Stereo Mapping with ZED Camera")
    parser.add_argument("--port", type=int, default=7779, help="Foxglove websocket port")
    args = parser.parse_args()

    pubsub.lcm.autoconf()

    mapper = StereoMapper(websocket_port=args.port)
    mapper.start()

    try:
        logger.info("\nStereoMapper running. Press Ctrl+C to stop...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\nShutting down...")
        mapper.stop()


if __name__ == "__main__":
    main()
