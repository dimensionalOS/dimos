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

import cv2
import numpy as np
import time
import threading
from typing import Dict, List, Optional
import logging

from dimos.core import In, Out, Module, rpc
from dimos.msgs.std_msgs import Header
from dimos.msgs.sensor_msgs import Image, ImageFormat, CameraInfo
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.utils.logging_config import setup_logger
from reactivex.disposable import Disposable

# Import LCM messages
from dimos_lcm.vision_msgs import (
    BoundingBox2D,
    Detection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)

logger = setup_logger("dimos.perception.csrt_tracker", level=logging.INFO)


class CSRTTracker(Module):
    """2D object tracking module using CSRT."""

    color_image: In[Image] = None

    detection2darray: Out[Detection2DArray] = None
    tracked_overlay: Out[Image] = None  # Visualization output

    def __init__(
        self,
        camera_info: CameraInfo,
        frame_id: str = "camera_link",
        enable_visualization: bool = True,
    ):
        """
        Initialize 2D object tracking module using OpenCV's CSRT tracker.

        Args:
            camera_info: Camera calibration information
            frame_id: Frame ID for the camera (default: "camera_link")
            enable_visualization: Whether to publish visualization overlay (default: True)
        """
        super().__init__()

        self.camera_info = camera_info
        self.frame_id = frame_id
        self.enable_visualization = enable_visualization

        # Tracker state
        self.tracker = None
        self.tracking_bbox = None  # Stores (x, y, w, h)
        self.tracking_initialized = False

        # Stuck detection
        self._last_bbox = None
        self._stuck_count = 0
        self._max_stuck_frames = 30  # 6 seconds at 5Hz before considering stuck

        # Frame management
        self._frame_lock = threading.Lock()
        self._latest_rgb_frame: Optional[np.ndarray] = None
        self._frame_arrival_time: Optional[float] = None

        # Tracking thread control
        self.tracking_thread: Optional[threading.Thread] = None
        self.stop_tracking_event = threading.Event()
        self.tracking_rate = 1.0  # Hz - balanced for CSRT performance
        self.tracking_period = 1.0 / self.tracking_rate

        # Store latest detection for RPC access
        self._latest_detection2d: Optional[Detection2DArray] = None

    @rpc
    def start(self):
        super().start()

        def on_frame(frame_msg: Image):
            # Use actual frame timestamp, not perf_counter!
            arrival_time = frame_msg.ts if frame_msg.ts else time.time()
            with self._frame_lock:
                self._latest_rgb_frame = frame_msg.data
                self._frame_arrival_time = arrival_time
            logger.debug(f"Frame received with arrival_time={arrival_time:.3f} (Unix timestamp)")

        unsub = self.color_image.subscribe(on_frame)
        self._disposables.add(Disposable(unsub))
        logger.info(f"CSRTTracker started ({self.camera_info.width}x{self.camera_info.height})")

    @rpc
    def stop(self) -> None:
        self.stop_track()
        if self.tracking_thread and self.tracking_thread.is_alive():
            self.stop_tracking_event.set()
            self.tracking_thread.join(timeout=2.0)

        super().stop()

    @rpc
    def track(self, bbox: List[float]) -> Dict:
        """
        Initialize tracking with a bounding box.

        Args:
            bbox: Bounding box in format [x1, y1, x2, y2]

        Returns:
            Dict containing tracking status
        """
        logger.debug(f"track() called with bbox: {bbox}")

        if self._latest_rgb_frame is None:
            logger.warning("No RGB frame available for tracking")
            return {"status": "no_frame"}

        # Initialize tracking
        x1, y1, x2, y2 = map(int, bbox)
        w, h = x2 - x1, y2 - y1
        if w <= 0 or h <= 0:
            logger.warning(f"Invalid initial bbox provided: {bbox}. Tracking not started.")
            return {"status": "invalid_bbox"}

        self.tracking_bbox = (x1, y1, w, h)
        self.tracker = cv2.legacy.TrackerCSRT_create()  # CSRT is more robust
        self.tracking_initialized = False
        logger.info(f"Tracking target set with bbox: {self.tracking_bbox}")

        # Convert RGB to BGR for tracker (OpenCV expects BGR)
        frame_bgr = cv2.cvtColor(self._latest_rgb_frame, cv2.COLOR_RGB2BGR)
        init_success = self.tracker.init(frame_bgr, self.tracking_bbox)

        if init_success:
            self.tracking_initialized = True
            logger.info("Tracker initialized successfully.")
        else:
            logger.error("Tracker initialization failed.")
            self.stop_track()
            return {"status": "init_failed"}

        # Start tracking thread
        self._start_tracking_thread()

        return {"status": "tracking_started", "bbox": self.tracking_bbox}

    def _start_tracking_thread(self):
        """Start the tracking thread."""
        self.stop_tracking_event.clear()
        self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.tracking_thread.start()
        logger.info("Started tracking thread")

    def _tracking_loop(self):
        """Main tracking loop that runs in a separate thread."""
        logger.debug(f"Tracking loop started, rate={self.tracking_rate}Hz")
        while not self.stop_tracking_event.is_set() and self.tracking_initialized:
            self._process_tracking()
            time.sleep(self.tracking_period)
        logger.debug("Tracking loop ended")

    def _reset_tracking_state(self):
        """Reset tracking state without stopping the thread."""
        self.tracker = None
        self.tracking_bbox = None
        self.tracking_initialized = False
        self._last_bbox = None
        self._stuck_count = 0

        # Publish empty detection
        empty_2d = Detection2DArray(
            detections_length=0, header=Header(time.time(), self.frame_id), detections=[]
        )
        self._latest_detection2d = empty_2d
        self.detection2darray.publish(empty_2d)

    @rpc
    def stop_track(self) -> bool:
        """
        Stop tracking the current object.

        Returns:
            bool: True if tracking was successfully stopped
        """
        self._reset_tracking_state()

        # Stop tracking thread if running
        if self.tracking_thread and self.tracking_thread.is_alive():
            if threading.current_thread() != self.tracking_thread:
                self.stop_tracking_event.set()
                self.tracking_thread.join(timeout=1.0)
                self.tracking_thread = None
            else:
                self.stop_tracking_event.set()

        logger.info("Tracking stopped")
        return True

    @rpc
    def is_tracking(self) -> bool:
        """
        Check if the tracker is currently tracking an object.

        Returns:
            bool: True if tracking is active
        """
        return self.tracking_initialized

    def _process_tracking(self):
        """Process current frame for tracking and publish 2D detections."""
        if self.tracker is None or not self.tracking_initialized:
            return

        # Get frame reference (only copy if we need visualization)
        with self._frame_lock:
            if self._latest_rgb_frame is None:
                return
            frame_rgb = self._latest_rgb_frame  # Use reference, not copy
            frame_timestamp = self._frame_arrival_time
            logger.debug(f"Processing frame with arrival time: {frame_timestamp}")

        # Convert RGB to BGR for tracker (OpenCV expects BGR)
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # Safety check for tracker
        if self.tracker is None:
            logger.warning("Tracker became None during processing, skipping")
            return

        # Update tracker
        tracker_succeeded, bbox_cv = self.tracker.update(frame_bgr)

        if not tracker_succeeded:
            logger.info("Tracker update failed. Stopping track.")
            self._reset_tracking_state()
            return

        # Extract bbox
        x, y, w, h = map(int, bbox_cv)
        current_bbox_x1y1x2y2 = [x, y, x + w, y + h]
        x1, y1, x2, y2 = current_bbox_x1y1x2y2

        # Check if tracker is stuck
        if self._last_bbox is not None:
            if (x1, y1, x2, y2) == self._last_bbox:
                self._stuck_count += 1
                if self._stuck_count >= self._max_stuck_frames:
                    logger.warning(f"Tracker stuck for {self._stuck_count} frames. Stopping track.")
                    self._reset_tracking_state()
                    return
            else:
                self._stuck_count = 0

        self._last_bbox = (x1, y1, x2, y2)

        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        width = float(x2 - x1)
        height = float(y2 - y1)

        # Create 2D detection header
        header = Header(time.time(), self.frame_id)

        # Create Detection2D with all fields in constructors
        detection_2d = Detection2D(
            id="0",
            results_length=1,
            header=header,
            bbox=BoundingBox2D(
                center=Pose2D(position=Point2D(x=center_x, y=center_y), theta=0.0),
                size_x=width,
                size_y=height,
            ),
            results=[
                ObjectHypothesisWithPose(
                    hypothesis=ObjectHypothesis(class_id="tracked_object", score=1.0)
                )
            ],
        )

        detection2darray = Detection2DArray(
            detections_length=1, header=header, detections=[detection_2d]
        )

        # Store and publish detection
        self._latest_detection2d = detection2darray
        self.detection2darray.publish(detection2darray)
        logger.debug(
            f"Published detection2d for tracked object at ({center_x:.1f}, {center_y:.1f})"
        )

        # Only create visualization if enabled
        if self.enable_visualization:
            # Only copy frame now for visualization
            frame_copy = frame_rgb.copy()
            # Convert to BGR for OpenCV drawing
            frame_bgr_viz = cv2.cvtColor(frame_copy, cv2.COLOR_RGB2BGR)
            viz_image_bgr = self._draw_visualization(frame_bgr_viz, current_bbox_x1y1x2y2)
            # Convert back to RGB for publishing
            viz_image_rgb = cv2.cvtColor(viz_image_bgr, cv2.COLOR_BGR2RGB)
            viz_msg = Image.from_numpy(viz_image_rgb, format=ImageFormat.RGB)
            self.tracked_overlay.publish(viz_msg)
            logger.debug(f"Published tracked overlay with bbox {current_bbox_x1y1x2y2}")

    def _draw_visualization(self, image: np.ndarray, bbox: List[int]) -> np.ndarray:
        """Draw tracking visualization directly on the image."""
        x1, y1, x2, y2 = bbox
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(image, "TRACKING", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return image
