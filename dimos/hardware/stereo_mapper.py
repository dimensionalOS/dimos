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

"""
StereoMapper - Exactly like UnitreeGo2 but using ZED camera data.
Uses ZED for both lidar (pointcloud) and odometry (pose).
"""

import os
import sys
import time
import logging

from dimos.hardware.zed_camera_single import ZedModuleSingle

sys.path.append(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
)

from dimos import core
from dimos.hardware.zed_filtered_module import FilteredZEDModule
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map  # Use regular Map, not LimitedMap
from dimos.robot.unitree_webrtc.type.passthrough_map import PassthroughMap
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.geometry_msgs import PoseStamped, Transform, Vector3, Quaternion
from dimos.msgs.sensor_msgs import Image
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.protocol import pubsub
from dimos.protocol.pubsub.lcmpubsub import LCM
from dimos.protocol.tf import TF
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.utils.logging_config import setup_logger
import numpy as np
from dimos.hardware.zed_camera import ZEDModule
from dimos.hardware.fake_zed_module import FakeZEDModule
from dimos.utils.testing import TimedSensorStorage

logger = setup_logger(__name__, level=logging.INFO)


class StereoMapper:
    """
    Stereo mapping system using ZED camera - structured exactly like UnitreeGo2.
    """

    def __init__(
        self,
        websocket_port: int = 7779,
        record_path: str = None,
        replay_path: str = None,
    ):
        """
        Initialize StereoMapper with ZED camera.

        Args:
            websocket_port: Port for Foxglove websocket connection
            record_path: Optional path to record sensor data
            replay_path: Optional path to replay recorded data from
        """
        self.websocket_port = websocket_port
        self.record_path = record_path
        self.replay_path = replay_path
        self.lcm = LCM()
        self.tf = None  # Initialize TF later to avoid conflicts

        self.dimos = None
        self.zed_module = None
        self.mapper = None
        self.foxglove_bridge = None
        self.storages = None  # For recording

        logger.info(f"StereoMapper initialized - websocket port {websocket_port}")

    def start(self):
        """Start the stereo mapping system - exactly like UnitreeGo2.start()."""
        logger.info("Starting StereoMapper system...")

        # Start Dimos
        self.dimos = core.start(1, memory_limit="30GiB")

        # Initialize TF after Dimos is started
        self.tf = TF()

        # Deploy modules in same order as UnitreeGo2
        self._deploy_zed_and_connection()
        self._deploy_mapping()
        self._deploy_visualization()

        # Start all modules
        self._start_modules()

        # Start LCM
        self.lcm.start()

        logger.info("=" * 60)
        logger.info("StereoMapper started successfully!")
        logger.info(f"Foxglove visualization: http://localhost:{self.websocket_port}")
        logger.info("")
        logger.info("Publishing topics:")
        logger.info("  /lidar - Pointcloud from ZED (as LidarMessage)")
        logger.info("  /odom - Camera pose from ZED visual odometry")
        logger.info("  /zed/color_image - RGB camera feed")
        logger.info("  /global_map")
        logger.info("  /global_costmap - Navigation costmap")
        logger.info("  /local_costmap - Local costmap")
        logger.info("")
        logger.info("Foxglove display frame: 'world'")
        logger.info("TF chain: world → base_link → camera_link")
        logger.info("=" * 60)

    def _deploy_zed_and_connection(self):
        """Deploy ZED camera module and configure connections - combines _deploy_connection from UnitreeGo2."""

        if self.replay_path:
            # Deploy fake ZED module for replay
            logger.info(f"Deploying FakeZEDModule for replay from: {self.replay_path}")
            self.zed_module = self.dimos.deploy(FakeZEDModule, recording_path=self.replay_path)
        else:
            # Deploy real ZED module
            logger.info("Deploying ZED camera module with spatial mapping...")
            self.zed_module = self.dimos.deploy(ZedModuleSingle)

        # Configure transports - use same topic names as UnitreeGo2
        # These are the main topics that UnitreeGo2 uses
        self.zed_module.pointcloud_msg.transport = core.LCMTransport("/lidar", LidarMessage)
        self.zed_module.pose.transport = core.LCMTransport("/odom", PoseStamped)
        # self.zed_module.color_image.transport = core.LCMTransport("/zed/color_image", Image)

        # Additional ZED-specific topics for debugging
        # self.zed_module.depth_image.transport = core.LCMTransport("/zed/depth_image", Image)
        # self.zed_module.camera_info.transport = core.LCMTransport("/zed/camera_info", CameraInfo)

        # Subscribe to pose messages to publish TF transforms
        self.zed_module.pose.subscribe(self._publish_tf)
        # Subscribe to lidar messages to log metadata
        logger.info("✓ ZED camera module deployed and configured")

        # Set up recording if requested (not in replay mode)
        if self.record_path and not self.replay_path:
            self._setup_recording()

    def _publish_tf(self, msg: PoseStamped):
        """Publish TF transforms - exactly like UnitreeGo2's ConnectionModule._publish_tf."""
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

    def _setup_recording(self):
        """Set up recording for ZED sensor streams."""
        logger.info(f"Setting up recording to {self.record_path}")

        # Create storage instances for each stream
        # TimedSensorStorage automatically saves with timestamps
        self.storages = {
            "lidar": TimedSensorStorage(f"{self.record_path}/lidar"),
            "pose": TimedSensorStorage(f"{self.record_path}/pose"),
            "color_image": TimedSensorStorage(f"{self.record_path}/color_image"),
            "depth_image": TimedSensorStorage(f"{self.record_path}/depth_image"),
            "camera_info": TimedSensorStorage(f"{self.record_path}/camera_info"),
        }

        # Record lidar pointcloud
        self.zed_module.pointcloud_msg.transport.subscribe(
            lambda msg: self.storages["lidar"].save_one(msg)
        )
        logger.info("  Recording /lidar stream")

        # # Record pose/odometry
        # self.zed_module.pose.transport.subscribe(lambda msg: self.storages["pose"].save_one(msg))
        # logger.info("  Recording /odom stream")

        # # Record color image
        # self.zed_module.color_image.transport.subscribe(
        #     lambda msg: self.storages["color_image"].save_one(msg)
        # )
        # logger.info("  Recording /zed/color_image stream")

        # # Record depth image
        # self.zed_module.depth_image.transport.subscribe(
        #     lambda msg: self.storages["depth_image"].save_one(msg)
        # )
        # logger.info("  Recording /zed/depth_image stream")

        # # Record camera info
        # self.zed_module.camera_info.transport.subscribe(
        #     lambda msg: self.storages["camera_info"].save_one(msg)
        # )
        # logger.info("  Recording /zed/camera_info stream")

        logger.info(f"✓ Recording setup complete - saving to {self.record_path}")

    def _deploy_mapping(self):
        """Deploy and configure the mapping module."""
        logger.info("Deploying mapping module...")

        self.mapper = self.dimos.deploy(
            PassthroughMap,
            voxel_size=0.05,  # Match ZED downsampling
            global_publish_interval=2.5,
            min_height=0.15,
            max_height=1.5,
            frame_id="world",
        )

        # Configure transports - same topics as UnitreeGo2
        self.mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)
        self.mapper.global_costmap.transport = core.LCMTransport("/global_costmap", OccupancyGrid)
        self.mapper.local_costmap.transport = core.LCMTransport("/local_costmap", OccupancyGrid)

        # Connect lidar input - connect directly to ZED module's pointcloud output
        self.mapper.lidar.connect(self.zed_module.pointcloud_msg)
        # self.mapper.odom.connect(self.zed_module.pose)

        logger.info("✓ Mapping module deployed and connected to ZED")

    def _deploy_visualization(self):
        """Deploy visualization (Foxglove bridge)."""
        logger.info("Deploying visualization...")

        self.foxglove_bridge = FoxgloveBridge()

        logger.info(f"✓ Foxglove bridge ready on port {self.websocket_port}")

    def _start_modules(self):
        """Start all deployed modules."""
        logger.info("Starting all modules...")

        # Start ZED first
        logger.info("  Starting ZED module...")
        self.zed_module.start()
        time.sleep(3)  # Give ZED time to initialize

        # Start mapper
        logger.info("  Starting mapper module...")
        self.mapper.start()

        # Start Foxglove
        logger.info("  Starting Foxglove bridge...")
        self.foxglove_bridge.start()

        logger.info("✓ All modules started")

    def stop(self):
        """Stop the stereo mapping system."""
        logger.info("Stopping StereoMapper...")

        # Recordings are saved automatically by TimedSensorStorage
        if self.storages:
            logger.info(f"✓ Recording saved to {self.record_path}")

        if self.zed_module:
            self.zed_module.stop()

        # Map module doesn't have a stop method, skip

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
    """Main entry point for StereoMapper."""
    import argparse

    parser = argparse.ArgumentParser(description="Stereo Mapping with ZED Camera")
    parser.add_argument("--port", type=int, default=7779, help="Foxglove websocket port")
    parser.add_argument("--record", type=str, help="Recording name/path for saving ZED data")
    parser.add_argument(
        "--replay", type=str, help="Path to recorded data to replay instead of using real camera"
    )
    parser.add_argument(
        "--no-spatial-mapping",
        action="store_true",
        help="Disable ZED spatial mapping and use traditional accumulation instead",
    )
    args = parser.parse_args()

    # Configure LCM
    pubsub.lcm.autoconf()

    # Create and start mapper
    mapper = StereoMapper(
        websocket_port=args.port,
        record_path=args.record,
        replay_path=args.replay,
    )
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
