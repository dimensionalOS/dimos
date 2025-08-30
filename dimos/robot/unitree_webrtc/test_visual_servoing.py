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

"""Test script for Mobile Base PBVS module with Unitree Go2 robot."""

import os
import time
import cv2
import logging
import warnings
from typing import Optional

from dimos import core
from dimos.core import In, Out
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.geometry_msgs import Twist, PoseStamped
from dimos_lcm.sensor_msgs import CameraInfo
from dimos_lcm.vision_msgs import Detection2DArray, Detection3DArray
from dimos_lcm.std_msgs import String
from dimos.protocol import pubsub
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.manipulation.visual_servoing.mobile_base_pbvs import MobileBasePBVS
from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2
from dimos.robot.foxglove_bridge import FoxgloveBridge
from dimos.utils.logging_config import setup_logger

logger = setup_logger("test_visual_servoing", level=logging.INFO)

# Suppress verbose loggers
logging.getLogger("aiortc.codecs.h264").setLevel(logging.ERROR)
logging.getLogger("lcm_foxglove_bridge").setLevel(logging.ERROR)
logging.getLogger("websockets.server").setLevel(logging.ERROR)
logging.getLogger("FoxgloveServer").setLevel(logging.ERROR)
logging.getLogger("asyncio").setLevel(logging.ERROR)
warnings.filterwarnings("ignore", message="coroutine.*was never awaited")
warnings.filterwarnings("ignore", message="H264Decoder.*failed to decode")


class VisualizationHandler:
    """Handles visualization and user interaction for visual servoing."""

    def __init__(self):
        self.lcm = LCM()
        self.latest_rgb = None
        self.latest_viz = None

        # Mouse interaction state
        self.selecting = False
        self.click_point = None
        self.tracking_active = False

        # Subscribe to topics
        self.rgb_topic = Topic("/go2/color_image", Image)
        self.viz_topic = Topic("/mobile_pbvs/viz", Image)

    def start(self):
        """Start the visualization handler."""
        self.lcm.start()

        # Subscribe to image topics
        self.lcm.subscribe(self.rgb_topic, self._on_rgb_image)
        self.lcm.subscribe(self.viz_topic, self._on_viz_image)

        logger.info("Visualization handler started")

    def _on_rgb_image(self, msg: Image, _: str):
        """Handle RGB image messages."""
        try:
            self.latest_rgb = msg.data
        except Exception as e:
            logger.error(f"Error processing RGB image: {e}")

    def _on_viz_image(self, msg: Image, _: str):
        """Handle visualization image messages."""
        try:
            self.latest_viz = msg.data
        except Exception as e:
            logger.error(f"Error processing viz image: {e}")

    def mouse_callback(self, event, x, y, _, param):
        """Handle mouse events for target selection."""
        pbvs_module = param.get("pbvs")

        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_point = (x, y)
            logger.info(f"Clicked at ({x}, {y})")

            # Start tracking via RPC
            if pbvs_module:
                result = pbvs_module.track(target_x=x, target_y=y)
                if result["status"] == "success":
                    logger.info(f"Started tracking: {result['message']}")
                    self.tracking_active = True
                else:
                    logger.error(f"Failed to start tracking: {result['message']}")

    def draw_interface(self, frame):
        """Draw UI elements on the frame."""
        # Draw click point if exists
        if self.click_point and self.tracking_active:
            cv2.circle(frame, self.click_point, 5, (0, 255, 0), -1)
            cv2.circle(frame, self.click_point, 10, (0, 255, 0), 2)

        # Draw instructions
        cv2.putText(
            frame,
            "Click on an object to track",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            "Press 's' to stop tracking, 'q' to quit",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        # Show tracking status
        if self.tracking_active:
            status = "Tracking Active"
            color = (0, 255, 0)
        else:
            status = "No Target"
            color = (0, 0, 255)
        cv2.putText(frame, f"Status: {status}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        return frame


def main():
    """Main test function."""
    logger.info("Starting Mobile Base PBVS Test")

    # Get robot IP from environment
    ip = os.getenv("ROBOT_IP")
    connection_type = os.getenv("CONNECTION_TYPE", "webrtc")

    # Enable LCM auto-configuration
    pubsub.lcm.autoconf()

    # Initialize components
    robot = None
    dimos = None
    pbvs_module = None
    viz_handler = None
    foxglove_bridge = None

    try:
        # Initialize and start robot
        logger.info("Initializing Unitree Go2 robot...")
        robot = UnitreeGo2(
            ip=ip,
            websocket_port=7779,
            connection_type=connection_type,
            use_metric3d_depth=os.getenv("USE_METRIC3D_DEPTH", "false").lower() == "true",
        )
        robot.start()
        time.sleep(3)  # Wait for robot to initialize

        logger.info("Robot initialized successfully")

        # Deploy Mobile Base PBVS module
        logger.info("Deploying Mobile Base PBVS module...")
        pbvs_module = robot.dimos.deploy(
            MobileBasePBVS,
            position_gain=0.4,
            rotation_gain=0.5,
            max_linear_velocity=0.6,
            max_angular_velocity=0.8,
            target_distance=1.2,
            target_tolerance=0.2,
            min_confidence=0.5,
            camera_frame_id="camera_link_optical",
            base_frame_id="world",
            tracking_loss_timeout=2.0,
        )

        # Configure input transports
        pbvs_module.rgb_image.transport = core.LCMTransport("/go2/color_image", Image)
        pbvs_module.depth_image.transport = core.LCMTransport("/go2/depth_image", Image)
        pbvs_module.camera_info.transport = core.LCMTransport("/go2/camera_info", CameraInfo)

        # Configure output transports
        pbvs_module.viz_image.transport = core.LCMTransport("/mobile_pbvs/viz", Image)
        pbvs_module.cmd_vel.transport = core.LCMTransport("/cmd_vel", Twist)
        pbvs_module.tracking_state.transport = core.LCMTransport("/mobile_pbvs/state", String)
        pbvs_module.detection3d_array.transport = core.LCMTransport(
            "/mobile_pbvs/detection3d", Detection3DArray
        )
        pbvs_module.detection2d_array.transport = core.LCMTransport(
            "/mobile_pbvs/detection2d", Detection2DArray
        )

        # Connect inputs to robot outputs
        pbvs_module.rgb_image.connect(robot.connection.video)

        # Connect to appropriate depth source based on robot configuration
        if robot.use_metric3d_depth:
            pbvs_module.depth_image.connect(robot.depth_module.depth_image)
        else:
            pbvs_module.depth_image.connect(robot.pointcloud_to_depth.depth_image)

        pbvs_module.camera_info.connect(robot.connection.camera_info)

        # Connect velocity commands to robot
        robot.connection.movecmd.connect(pbvs_module.cmd_vel)

        # Start the PBVS module
        pbvs_module.start()
        logger.info("Mobile Base PBVS module started")

        # Create visualization handler
        viz_handler = VisualizationHandler()
        viz_handler.start()

        # Start Foxglove bridge for additional visualization
        foxglove_bridge = FoxgloveBridge()
        foxglove_bridge.start()

        # Give modules time to initialize
        time.sleep(2)

        # Create OpenCV window and set mouse callback
        cv2.namedWindow("Mobile Base Visual Servoing")
        cv2.setMouseCallback(
            "Mobile Base Visual Servoing", viz_handler.mouse_callback, {"pbvs": pbvs_module}
        )

        logger.info("=" * 60)
        logger.info("Mobile Base PBVS Test Ready!")
        logger.info("Click on an object to start tracking")
        logger.info("Press 's' to stop tracking")
        logger.info("Press 'q' to quit")
        logger.info(f"Foxglove visualization at http://localhost:8765")
        logger.info(f"Web interface at http://localhost:{robot.websocket_port}")
        logger.info("=" * 60)

        # Main visualization loop
        while True:
            # Get the frame to display
            # When tracking is active and viz is available, use viz; otherwise use RGB
            display_frame = None
            if viz_handler.tracking_active and viz_handler.latest_viz is not None:
                display_frame = viz_handler.latest_viz.copy()
            elif viz_handler.latest_rgb is not None:
                display_frame = viz_handler.latest_rgb.copy()
            else:
                time.sleep(0.03)
                continue

            # Draw UI elements
            display_frame = viz_handler.draw_interface(display_frame)

            # Show frame
            cv2.imshow("Mobile Base Visual Servoing", display_frame)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                logger.info("Quit requested")
                break
            elif key == ord("s"):
                # Stop tracking
                if pbvs_module and viz_handler.tracking_active:
                    result = pbvs_module.stop_track()
                    logger.info(f"Stopped tracking: {result['message']}")
                    viz_handler.tracking_active = False
                    viz_handler.click_point = None
                    viz_handler.latest_viz = None  # Clear viz to force RGB display

            time.sleep(0.03)  # ~30 FPS

    except KeyboardInterrupt:
        logger.info("Keyboard interrupt received")
    except Exception as e:
        logger.error(f"Error in test: {e}")
        import traceback

        traceback.print_exc()
    finally:
        # Clean up
        logger.info("Cleaning up...")
        cv2.destroyAllWindows()

        if pbvs_module:
            pbvs_module.cleanup()

        if foxglove_bridge:
            foxglove_bridge.stop()

        if dimos:
            dimos.close()

        logger.info("Test completed")


if __name__ == "__main__":
    main()
