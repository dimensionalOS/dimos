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

"""Test script for ZED Module with LCM visualization."""

import asyncio
import threading
import time

import cv2
from dimos_lcm.geometry_msgs import PoseStamped

# Import LCM message types
from dimos_lcm.sensor_msgs import CameraInfo, Image as LCMImage
import numpy as np

from dimos import core
from dimos.hardware.zed_camera import ZEDModule
from dimos.perception.common.utils import colorize_depth
from dimos.protocol import pubsub
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class ZEDVisualizationNode:
    """Node that subscribes to ZED topics and visualizes the data."""

    def __init__(self):
        self.lcm = LCM()
        self.latest_color = None
        self.latest_depth = None
        self.latest_pose = None
        self.camera_info = None
        self._running = False

        # Subscribe to topics
        self.color_topic = Topic("/zed/color_image", LCMImage)
        self.depth_topic = Topic("/zed/depth_image", LCMImage)
        self.camera_info_topic = Topic("/zed/camera_info", CameraInfo)
        self.pose_topic = Topic("/zed/pose", PoseStamped)

    def start(self):
        """Start the visualization node."""
        self._running = True
        self.lcm.start()

        # Subscribe to topics
        self.lcm.subscribe(self.color_topic, self._on_color_image)
        self.lcm.subscribe(self.depth_topic, self._on_depth_image)
        self.lcm.subscribe(self.camera_info_topic, self._on_camera_info)
        self.lcm.subscribe(self.pose_topic, self._on_pose)

        logger.info("Visualization node started, subscribed to ZED topics")

    def stop(self):
        """Stop the visualization node."""
        self._running = False
        cv2.destroyAllWindows()

    def _on_color_image(self, msg: LCMImage, topic: str):
        """Handle color image messages."""
        try:
            # Convert LCM message to numpy array
            data = np.frombuffer(msg.data, dtype=np.uint8)

            if msg.encoding == "rgb8":
                image = data.reshape((msg.height, msg.width, 3))
                # Convert RGB to BGR for OpenCV
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "mono8":
                image = data.reshape((msg.height, msg.width))
            else:
                logger.warning(f"Unsupported encoding: {msg.encoding}")
                return

            self.latest_color = image
            logger.debug(f"Received color image: {msg.width}x{msg.height}")

        except Exception as e:
            logger.error(f"Error processing color image: {e}")

    def _on_depth_image(self, msg: LCMImage, topic: str):
        """Handle depth image messages."""
        try:
            # Convert LCM message to numpy array
            if msg.encoding == "32FC1":
                data = np.frombuffer(msg.data, dtype=np.float32)
                depth = data.reshape((msg.height, msg.width))
            else:
                logger.warning(f"Unsupported depth encoding: {msg.encoding}")
                return

            self.latest_depth = depth
            logger.debug(f"Received depth image: {msg.width}x{msg.height}")

        except Exception as e:
            logger.error(f"Error processing depth image: {e}")

    def _on_camera_info(self, msg: CameraInfo, topic: str):
        """Handle camera info messages."""
        self.camera_info = msg
        logger.info(
            f"Received camera info: {msg.width}x{msg.height}, distortion model: {msg.distortion_model}"
        )

    def _on_pose(self, msg: PoseStamped, topic: str):
        """Handle pose messages."""
        self.latest_pose = msg
        pos = msg.pose.position
        ori = msg.pose.orientation
        logger.debug(
            f"Pose: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), "
            + f"ori=({ori.x:.2f}, {ori.y:.2f}, {ori.z:.2f}, {ori.w:.2f})"
        )

    def visualize(self):
        """Run visualization loop."""
        while self._running:
            # Create visualization
            vis_images = []

            # Color image
            if self.latest_color is not None:
                color_vis = self.latest_color.copy()

                # Add pose text if available
                if self.latest_pose is not None:
                    pos = self.latest_pose.pose.position
                    text = f"Pose: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})"
                    cv2.putText(
                        color_vis, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                    )

                vis_images.append(("ZED Color", color_vis))

            # Depth image
            if self.latest_depth is not None:
                depth_colorized = colorize_depth(self.latest_depth, max_depth=5.0)
                if depth_colorized is not None:
                    # Convert RGB to BGR for OpenCV
                    depth_colorized = cv2.cvtColor(depth_colorized, cv2.COLOR_RGB2BGR)

                    # Add depth stats
                    valid_mask = np.isfinite(self.latest_depth) & (self.latest_depth > 0)
                    if np.any(valid_mask):
                        min_depth = np.min(self.latest_depth[valid_mask])
                        max_depth = np.max(self.latest_depth[valid_mask])
                        mean_depth = np.mean(self.latest_depth[valid_mask])

                        text = f"Depth: min={min_depth:.2f}m, max={max_depth:.2f}m, mean={mean_depth:.2f}m"
                        cv2.putText(
                            depth_colorized,
                            text,
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 255),
                            1,
                        )

                    vis_images.append(("ZED Depth", depth_colorized))

            # Show windows
            for name, image in vis_images:
                cv2.imshow(name, image)

            # Handle key press
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                logger.info("Quit requested")
                self._running = False
                break
            elif key == ord("s"):
                # Save images
                if self.latest_color is not None:
                    cv2.imwrite("zed_color.png", self.latest_color)
                    logger.info("Saved color image to zed_color.png")
                if self.latest_depth is not None:
                    np.save("zed_depth.npy", self.latest_depth)
                    logger.info("Saved depth data to zed_depth.npy")

            time.sleep(0.03)  # ~30 FPS


async def test_zed_module():
    """Test the ZED Module with visualization."""
    logger.info("Starting ZED Module test")

    # Start Dask
    dimos = core.start(1)

    # Enable LCM auto-configuration
    pubsub.lcm.autoconf()

    try:
        # Deploy ZED module
        logger.info("Deploying ZED module...")
        zed = dimos.deploy(
            ZEDModule,
            camera_id=0,
            resolution="HD720",
            depth_mode="NEURAL",
            fps=30,
            enable_tracking=True,
            publish_rate=10.0,  # 10 Hz for testing
            frame_id="zed_camera",
        )

        # Configure LCM transports
        zed.color_image.transport = core.LCMTransport("/zed/color_image", LCMImage)
        zed.depth_image.transport = core.LCMTransport("/zed/depth_image", LCMImage)
        zed.camera_info.transport = core.LCMTransport("/zed/camera_info", CameraInfo)
        zed.pose.transport = core.LCMTransport("/zed/pose", PoseStamped)

        # Print module info
        logger.info("ZED Module configured:")

        # Start ZED module
        logger.info("Starting ZED module...")
        zed.start()

        # Give module time to initialize
        await asyncio.sleep(2)

        # Create and start visualization node
        viz_node = ZEDVisualizationNode()
        viz_node.start()

        # Run visualization in separate thread
        viz_thread = threading.Thread(target=viz_node.visualize, daemon=True)
        viz_thread.start()

        logger.info("ZED Module running. Press 'q' in image window to quit, 's' to save images.")

        # Keep running until visualization stops
        while viz_node._running:
            await asyncio.sleep(0.1)

        # Stop ZED module
        logger.info("Stopping ZED module...")
        zed.stop()

        # Stop visualization
        viz_node.stop()

    except Exception as e:
        logger.error(f"Error in test: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Clean up
        dimos.close()
        logger.info("Test completed")


if __name__ == "__main__":
    # Run the test
    asyncio.run(test_zed_module())
