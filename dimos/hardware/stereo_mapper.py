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

sys.path.append(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
)

from dimos import core
from dimos.hardware.zed_filtered_module import FilteredZEDModule
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map  # Use regular Map, not LimitedMap
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

logger = setup_logger(__name__, level=logging.INFO)


class StereoMapper:
    """
    Stereo mapping system using ZED camera - structured exactly like UnitreeGo2.
    """

    def __init__(
        self, websocket_port: int = 7779, record_path: str = None, replay_path: str = None
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
        self.dimos = core.start(4)

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
        logger.info("  /global_map - Accumulated map")
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
            self.zed_module = self.dimos.deploy(
                FakeZEDModule, recording_path=self.replay_path, frame_id="camera_link"
            )
        else:
            # Deploy real ZED module
            logger.info("Deploying ZED camera module...")
            self.zed_module = self.dimos.deploy(
                FilteredZEDModule,
                camera_id=0,
                resolution="HD720",
                depth_mode="NEURAL",
                fps=15,
                enable_tracking=True,  # Enable visual odometry
                enable_imu_fusion=True,
                set_floor_as_origin=True,
                publish_rate=10.0,
                frame_id="camera_link",
                recording_path=self.record_path if self.record_path else None,
                # Filtering parameters - match Unitree specs
                filter_voxel_size=0.05,  # Initial downsampling to reduce lag
                filter_max_distance=5.0,  # Slightly larger than Unitree for better coverage
                filter_min_distance=0.1,
                filter_min_z=-1.0,  # Allow more ground points
                filter_max_z=2.0,  # Allow ceiling/higher objects
                filter_ground_threshold=-0.45,  # Not used anymore but kept for compatibility
                filter_target_points=50000,  # More points for better map quality
            )
            # self.zed_module = self.dimos.deploy(ZEDModule)

        # Configure transports - use same topic names as UnitreeGo2
        # These are the main topics that UnitreeGo2 uses
        self.zed_module.pointcloud_msg.transport = core.LCMTransport("/lidar", LidarMessage)
        self.zed_module.pose.transport = core.LCMTransport("/odom", PoseStamped)
        self.zed_module.color_image.transport = core.LCMTransport("/zed/color_image", Image)

        # Additional ZED-specific topics for debugging
        self.zed_module.depth_image.transport = core.LCMTransport("/zed/depth_image", Image)
        self.zed_module.camera_info.transport = core.LCMTransport("/zed/camera_info", CameraInfo)

        # Subscribe to pose messages to publish TF transforms
        self.zed_module.pose.subscribe(self._publish_tf)
        # Subscribe to lidar messages to log metadata
        # self.zed_module.pointcloud_msg.subscribe(self._log_lidar_metadata)
        logger.info("✓ ZED camera module deployed and configured")

    def _log_lidar_metadata(self, msg: LidarMessage):
        """Log lidar metadata - exactly like UnitreeGo2's ConnectionModule."""
        # Ensure origin is set correctly for ZED (camera is at origin in optical frame)
        if msg.origin is None or msg.origin == [0.0, 0.0, 0.0]:
            msg.origin = [0.0, 0.0, 0.0]  # ZED pointcloud is relative to camera optical frame

        # Log metadata about the lidar message
        if hasattr(msg, "pointcloud") and msg.pointcloud is not None:
            pcd = msg.pointcloud
            num_points = len(pcd.points) if hasattr(pcd, "points") else 0

            # Get bounds if available
            if num_points > 0:
                points = np.asarray(pcd.points)
                x_min, y_min, z_min = points.min(axis=0)
                x_max, y_max, z_max = points.max(axis=0)
                bounds_str = f"X:[{x_min:.2f},{x_max:.2f}] Y:[{y_min:.2f},{y_max:.2f}] Z:[{z_min:.2f},{z_max:.2f}]"
            else:
                bounds_str = "No bounds (0 points)"
        else:
            num_points = 0
            bounds_str = "No pointcloud"

        logger.info(
            f"[ZED LIDAR] Points: {num_points}, Resolution: {msg.resolution:.3f}, "
            f"Origin: {msg.origin}, Bounds: {bounds_str}"
        )

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
        """Deploy and configure the mapping module - exactly like UnitreeGo2."""
        logger.info("Deploying mapping module...")

        # Use regular Map class with same parameters as UnitreeGo2
        self.mapper = self.dimos.deploy(
            Map,
            voxel_size=0.05,  # VERY SPARSE - 30cm voxels for testing (was 0.05)
            global_publish_interval=2.5,
            min_height=0.15,
            max_height=1.5,
        )

        # Configure transports - same topics as UnitreeGo2
        self.mapper.global_map.transport = core.LCMTransport("/global_map", LidarMessage)
        self.mapper.global_costmap.transport = core.LCMTransport("/global_costmap", OccupancyGrid)
        self.mapper.local_costmap.transport = core.LCMTransport("/local_costmap", OccupancyGrid)

        # Connect lidar input - connect directly to ZED module's pointcloud output
        self.mapper.lidar.connect(self.zed_module.pointcloud_msg)

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
    args = parser.parse_args()

    # Configure LCM
    pubsub.lcm.autoconf()

    # Create and start mapper
    mapper = StereoMapper(
        websocket_port=args.port, record_path=args.record, replay_path=args.replay
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
