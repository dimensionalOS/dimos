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

"""Test script for Object Tracking module with ZED camera."""

import asyncio
import cv2

from dimos import core
from dimos.hardware.zed_camera import ZEDModule
from dimos.perception.object_tracker import ObjectTracking
from dimos.protocol import pubsub
from dimos.utils.logging_config import setup_logger
from dimos.robot.foxglove_bridge import FoxgloveBridge

# Import message types
from dimos.msgs.sensor_msgs import Image
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic

logger = setup_logger("test_object_tracking_module")

# Suppress verbose Foxglove bridge warnings
import logging

logging.getLogger("lcm_foxglove_bridge").setLevel(logging.ERROR)
logging.getLogger("FoxgloveServer").setLevel(logging.ERROR)


class TrackingVisualization:
    """Handles visualization and user interaction for object tracking."""

    def __init__(self):
        self.lcm = LCM()
        self.latest_color = None

        # Mouse interaction state
        self.selecting_bbox = False
        self.bbox_start = None
        self.current_bbox = None
        self.tracking_active = False

        # Subscribe to color image topic only
        self.color_topic = Topic("/zed/color_image", Image)

    def start(self):
        """Start the visualization node."""
        self.lcm.start()

        # Subscribe to color image only
        self.lcm.subscribe(self.color_topic, self._on_color_image)

        logger.info("Visualization started, subscribed to color image topic")

    def _on_color_image(self, msg: Image, _: str):
        """Handle color image messages."""
        try:
            # Convert dimos Image to OpenCV format (BGR) for display
            self.latest_color = msg.to_opencv()
            logger.debug(f"Received color image: {msg.width}x{msg.height}, format: {msg.format}")
        except Exception as e:
            logger.error(f"Error processing color image: {e}")

    def mouse_callback(self, event, x, y, _, param):
        """Handle mouse events for bbox selection."""
        tracker_module = param.get("tracker")

        if event == cv2.EVENT_LBUTTONDOWN:
            self.selecting_bbox = True
            self.bbox_start = (x, y)
            self.current_bbox = None

        elif event == cv2.EVENT_MOUSEMOVE and self.selecting_bbox:
            # Update current selection for visualization
            x1, y1 = self.bbox_start
            self.current_bbox = [min(x1, x), min(y1, y), max(x1, x), max(y1, y)]

        elif event == cv2.EVENT_LBUTTONUP and self.selecting_bbox:
            self.selecting_bbox = False
            if self.bbox_start:
                x1, y1 = self.bbox_start
                x2, y2 = x, y
                # Ensure valid bbox
                bbox = [min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2)]

                # Check if bbox is valid (has area)
                if bbox[2] > bbox[0] and bbox[3] > bbox[1]:
                    # Call track RPC on the tracker module
                    if tracker_module:
                        result = tracker_module.track(bbox)
                        logger.info(f"Tracking initialized: {result}")
                        self.tracking_active = True
                        self.current_bbox = None

    def draw_interface(self, frame):
        """Draw UI elements on the frame."""
        # Draw bbox selection if in progress
        if self.selecting_bbox and self.current_bbox:
            x1, y1, x2, y2 = self.current_bbox
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)

        # Draw instructions
        cv2.putText(
            frame,
            "Click and drag to select object",
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


async def test_object_tracking_module():
    """Test object tracking with ZED camera module."""
    logger.info("Starting Object Tracking Module test")

    # Start Dimos
    dimos = core.start(2)

    # Enable LCM auto-configuration
    pubsub.lcm.autoconf()

    viz = None
    tracker = None
    zed = None
    foxglove_bridge = None

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
            publish_rate=15.0,
            frame_id="zed_camera_link",
        )

        # Configure ZED LCM transports
        zed.color_image.transport = core.LCMTransport("/zed/color_image", Image)
        zed.depth_image.transport = core.LCMTransport("/zed/depth_image", Image)
        zed.camera_info.transport = core.LCMTransport("/zed/camera_info", CameraInfo)
        zed.pose.transport = core.LCMTransport("/zed/pose", PoseStamped)

        # Start ZED to begin publishing
        zed.start()
        await asyncio.sleep(2)  # Wait for camera to initialize

        # Deploy Object Tracking module
        logger.info("Deploying Object Tracking module...")
        tracker = dimos.deploy(
            ObjectTracking,
            camera_intrinsics=None,  # Will get from camera_info topic
            reid_threshold=5,
            reid_fail_tolerance=10,
            frame_id="zed_camera_link",
        )

        # Configure tracking LCM transports
        tracker.color_image.transport = core.LCMTransport("/zed/color_image", Image)
        tracker.depth.transport = core.LCMTransport("/zed/depth_image", Image)
        tracker.camera_info.transport = core.LCMTransport("/zed/camera_info", CameraInfo)

        # Configure output transports
        from dimos_lcm.vision_msgs import Detection2DArray, Detection3DArray

        tracker.detection2darray.transport = core.LCMTransport(
            "/detection2darray", Detection2DArray
        )
        tracker.detection3darray.transport = core.LCMTransport(
            "/detection3darray", Detection3DArray
        )
        tracker.tracked_overlay.transport = core.LCMTransport("/tracked_overlay", Image)

        # Connect inputs
        tracker.color_image.connect(zed.color_image)
        tracker.depth.connect(zed.depth_image)
        tracker.camera_info.connect(zed.camera_info)

        # Start tracker
        tracker.start()

        # Create visualization
        viz = TrackingVisualization()
        viz.start()

        # Start Foxglove bridge for visualization
        foxglove_bridge = FoxgloveBridge()
        foxglove_bridge.start()

        # Give modules time to initialize
        await asyncio.sleep(1)

        # Create OpenCV window and set mouse callback
        cv2.namedWindow("Object Tracking")
        cv2.setMouseCallback("Object Tracking", viz.mouse_callback, {"tracker": tracker})

        logger.info("System ready. Click and drag to select an object to track.")
        logger.info("Foxglove visualization available at http://localhost:8765")

        # Main visualization loop
        while True:
            # Get the color frame to display
            if viz.latest_color is not None:
                display_frame = viz.latest_color.copy()
            else:
                # Wait for frames
                await asyncio.sleep(0.03)
                continue

            # Draw UI elements
            display_frame = viz.draw_interface(display_frame)

            # Show frame
            cv2.imshow("Object Tracking", display_frame)

            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                logger.info("Quit requested")
                break
            elif key == ord("s"):
                # Stop tracking
                if tracker:
                    tracker.stop_track()
                    viz.tracking_active = False
                    logger.info("Tracking stopped")

            await asyncio.sleep(0.03)  # ~30 FPS

    except Exception as e:
        logger.error(f"Error in test: {e}")
        import traceback

        traceback.print_exc()

    finally:
        # Clean up
        cv2.destroyAllWindows()

        if tracker:
            tracker.stop()
        if zed:
            zed.stop()
        if foxglove_bridge:
            foxglove_bridge.stop()

        dimos.close()
        logger.info("Test completed")


if __name__ == "__main__":
    asyncio.run(test_object_tracking_module())
