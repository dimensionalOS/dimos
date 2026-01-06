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

from dataclasses import dataclass
import logging
import threading
import time

import cv2

# Import LCM messages
from dimos_lcm.vision_msgs import (  # type: ignore[import-untyped]
    BoundingBox2D,
    Detection2D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
import numpy as np
from reactivex.disposable import Disposable

from dimos.core import In, Module, ModuleConfig, Out, rpc
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.msgs.std_msgs import Header
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.utils.logging_config import setup_logger

logger = setup_logger(level=logging.INFO)


@dataclass
class ObjectTracker2DConfig(ModuleConfig):
    frame_id: str = "camera_link"


class ObjectTracker2D(Module[ObjectTracker2DConfig]):
    """Pure 2D object tracking module using OpenCV's CSRT tracker."""

    color_image: In[Image]

    detection2darray: Out[Detection2DArray]
    tracked_overlay: Out[Image]  # Visualization output

    default_config = ObjectTracker2DConfig
    config: ObjectTracker2DConfig

    def __init__(self, **kwargs: object) -> None:
        """Initialize 2D object tracking module using OpenCV's CSRT tracker."""
        super().__init__(**kwargs)

        # Tracker state
        self.tracker = None
        self.tracking_bbox = None  # Stores (x, y, w, h)
        self.tracking_initialized = False

        # Stuck detection
        self._last_bbox = None
        self._stuck_count = 0
        self._max_stuck_frames = 10  # Higher threshold for stationary objects

        # Frame management
        self._frame_lock = threading.Lock()
        self._latest_rgb_frame: np.ndarray | None = None  # type: ignore[type-arg]
        self._frame_arrival_time: float | None = None

        # Tracking thread control
        self.tracking_thread: threading.Thread | None = None
        self.stop_tracking_event = threading.Event()
        self.tracking_rate = 5.0  # Hz
        self.tracking_period = 1.0 / self.tracking_rate

        # Store latest detection for RPC access
        self._latest_detection2d: Detection2DArray | None = None

    @rpc
    def start(self) -> None:
        super().start()

        def on_frame(frame_msg: Image) -> None:
            arrival_time = time.perf_counter()
            with self._frame_lock:
                self._latest_rgb_frame = frame_msg.data
                self._frame_arrival_time = arrival_time

        unsub = self.color_image.subscribe(on_frame)
        self._disposables.add(Disposable(unsub))
        logger.info("ObjectTracker2D module started")

    @rpc
    def stop(self) -> None:
        self.stop_track()
        if self.tracking_thread and self.tracking_thread.is_alive():
            self.stop_tracking_event.set()
            self.tracking_thread.join(timeout=2.0)

        super().stop()

    @rpc
    def track(self, bbox: list[float]) -> dict:  # type: ignore[type-arg]
        """
        Initialize tracking with a bounding box.

        Args:
            bbox: Bounding box in format [x1, y1, x2, y2]

        Returns:
            Dict containing tracking status
        """
        if self._latest_rgb_frame is None:
            logger.warning("No RGB frame available for tracking")
            return {"status": "no_frame"}

        # Initialize tracking
        x1, y1, x2, y2 = map(int, bbox)
        w, h = x2 - x1, y2 - y1
        if w <= 0 or h <= 0:
            logger.warning(f"Invalid initial bbox provided: {bbox}. Tracking not started.")
            return {"status": "invalid_bbox"}

        self.tracking_bbox = (x1, y1, w, h)  # type: ignore[assignment]
        self.tracker = cv2.legacy.TrackerCSRT_create()  # type: ignore[attr-defined]
        self.tracking_initialized = False
        logger.info(f"Tracking target set with bbox: {self.tracking_bbox}")

        # Convert RGB to BGR for CSRT (OpenCV expects BGR)
        frame_bgr = cv2.cvtColor(self._latest_rgb_frame, cv2.COLOR_RGB2BGR)
        init_success = self.tracker.init(frame_bgr, self.tracking_bbox)  # type: ignore[attr-defined]
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

    def _start_tracking_thread(self) -> None:
        """Start the tracking thread."""
        self.stop_tracking_event.clear()
        self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.tracking_thread.start()
        logger.info("Started tracking thread")

    def _tracking_loop(self) -> None:
        """Main tracking loop that runs in a separate thread."""
        while not self.stop_tracking_event.is_set() and self.tracking_initialized:
            self._process_tracking()
            time.sleep(self.tracking_period)
        logger.info("Tracking loop ended")

    def _reset_tracking_state(self) -> None:
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

    def _process_tracking(self) -> None:
        """Process current frame for tracking and publish 2D detections."""
        if self.tracker is None or not self.tracking_initialized:
            return

        # Get frame copy
        with self._frame_lock:
            if self._latest_rgb_frame is None:
                return
            frame = self._latest_rgb_frame.copy()

        # Convert RGB to BGR for CSRT (OpenCV expects BGR)
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

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

        # Store and publish
        self._latest_detection2d = detection2darray
        self.detection2darray.publish(detection2darray)

        # Create visualization
        viz_image = self._draw_visualization(frame, current_bbox_x1y1x2y2)
        viz_copy = viz_image.copy()  # Force copy needed to prevent frame reuse
        viz_msg = Image.from_numpy(viz_copy, format=ImageFormat.RGB)
        self.tracked_overlay.publish(viz_msg)

    def _draw_visualization(self, image: np.ndarray, bbox: list[int]) -> np.ndarray:  # type: ignore[type-arg]
        """Draw tracking visualization."""
        viz_image = image.copy()
        x1, y1, x2, y2 = bbox
        cv2.rectangle(viz_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(viz_image, "TRACKING", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return viz_image
