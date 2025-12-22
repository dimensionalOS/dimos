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

from dimos.core import In, Out, Module, rpc
from dimos.msgs.std_msgs import Header
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.utils.logging_config import setup_logger

# Import LCM messages
from dimos_lcm.vision_msgs import Detection2D, ObjectHypothesisWithPose

logger = setup_logger("dimos.perception.object_tracker_2d")


class ObjectTracker2D(Module):
    """Pure 2D object tracking module using CSRT tracker with ORB re-identification."""

    # Inputs
    color_image: In[Image] = None

    # Outputs
    detection2darray: Out[Detection2DArray] = None
    tracked_overlay: Out[Image] = None  # Visualization output

    def __init__(
        self,
        reid_threshold: int = 10,
        reid_fail_tolerance: int = 5,
        frame_id: str = "camera_link",
    ):
        """
        Initialize 2D object tracking module using OpenCV's CSRT tracker with ORB re-ID.

        Args:
            reid_threshold: Minimum good feature matches needed to confirm re-ID.
            reid_fail_tolerance: Number of consecutive frames Re-ID can fail before tracking stops.
            frame_id: TF frame ID for the camera (default: "camera_link")
        """
        super().__init__()

        self.reid_threshold = reid_threshold
        self.reid_fail_tolerance = reid_fail_tolerance
        self.frame_id = frame_id

        # Tracker state
        self.tracker = None
        self.tracking_bbox = None  # Stores (x, y, w, h)
        self.tracking_initialized = False

        # ORB Re-ID
        self.orb = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.original_des = None
        self.original_kps = None
        self.reid_fail_count = 0

        # Visualization state
        self.last_good_matches = []
        self.last_roi_kps = None
        self.last_roi_bbox = None
        self.reid_confirmed = False
        self.tracking_frame_count = 0
        self.reid_warmup_frames = 3

        # Frame management
        self._frame_lock = threading.Lock()
        self._latest_rgb_frame: Optional[np.ndarray] = None

        # Tracking thread control
        self.tracking_thread: Optional[threading.Thread] = None
        self.stop_tracking_event = threading.Event()
        self.tracking_rate = 30.0  # Hz
        self.tracking_period = 1.0 / self.tracking_rate

        # Store latest detection for RPC access
        self._latest_detection2d: Optional[Detection2DArray] = None

    @rpc
    def start(self):
        """Start the object tracking module and subscribe to video stream."""

        def on_frame(frame_msg: Image):
            with self._frame_lock:
                self._latest_rgb_frame = frame_msg.data

        self.color_image.subscribe(on_frame)
        logger.info("ObjectTracker2D module started")

    @rpc
    def track(self, bbox: List[float]) -> Dict:
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

        # Set tracking parameters
        self.tracking_bbox = (x1, y1, w, h)
        self.tracker = cv2.legacy.TrackerCSRT_create()
        self.tracking_initialized = False
        self.original_des = None
        self.reid_fail_count = 0
        logger.info(f"Tracking target set with bbox: {self.tracking_bbox}")

        # Extract initial ORB features
        roi = self._latest_rgb_frame[y1:y2, x1:x2]
        if roi.size > 0:
            self.original_kps, self.original_des = self.orb.detectAndCompute(roi, None)
            if self.original_des is None:
                logger.warning("No ORB features found in initial ROI. REID will be disabled.")
            else:
                logger.info(f"Initial ORB features extracted: {len(self.original_des)}")

            # Initialize the tracker
            init_success = self.tracker.init(self._latest_rgb_frame, self.tracking_bbox)
            if init_success:
                self.tracking_initialized = True
                self.tracking_frame_count = 0
                logger.info("Tracker initialized successfully.")
            else:
                logger.error("Tracker initialization failed.")
                self.stop_track()
                return {"status": "init_failed"}
        else:
            logger.error("Empty ROI during tracker initialization.")
            self.stop_track()
            return {"status": "empty_roi"}

        # Start tracking thread
        self._start_tracking_thread()

        return {"status": "tracking_started", "bbox": self.tracking_bbox}

    def _reid(self, frame, current_bbox) -> bool:
        """Check if features in current_bbox match stored original features."""
        # During warm-up period, always return True
        if self.tracking_frame_count < self.reid_warmup_frames:
            return True

        if self.original_des is None:
            return False

        x1, y1, x2, y2 = map(int, current_bbox)
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return False

        kps_current, des_current = self.orb.detectAndCompute(roi, None)
        if des_current is None or len(des_current) < 2:
            return False

        # Store for visualization
        self.last_roi_kps = kps_current
        self.last_roi_bbox = [x1, y1, x2, y2]

        # Handle single descriptor case
        if len(self.original_des) < 2:
            matches = self.bf.match(self.original_des, des_current)
            self.last_good_matches = matches
            good_matches = len(matches)
        else:
            matches = self.bf.knnMatch(self.original_des, des_current, k=2)
            good_matches_list = []
            good_matches = 0
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.75 * n.distance:
                        good_matches_list.append(m)
                        good_matches += 1
            self.last_good_matches = good_matches_list

        return good_matches >= self.reid_threshold

    def _start_tracking_thread(self):
        """Start the tracking thread."""
        self.stop_tracking_event.clear()
        self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.tracking_thread.start()
        logger.info("Started tracking thread")

    def _tracking_loop(self):
        """Main tracking loop that runs in a separate thread."""
        while not self.stop_tracking_event.is_set() and self.tracking_initialized:
            self._process_tracking()
            time.sleep(self.tracking_period)
        logger.info("Tracking loop ended")

    def _reset_tracking_state(self):
        """Reset tracking state without stopping the thread."""
        self.tracker = None
        self.tracking_bbox = None
        self.tracking_initialized = False
        self.original_des = None
        self.original_kps = None
        self.reid_fail_count = 0
        self.last_good_matches = []
        self.last_roi_kps = None
        self.last_roi_bbox = None
        self.reid_confirmed = False
        self.tracking_frame_count = 0

        # Publish empty detection
        empty_2d = Detection2DArray(
            detections_length=0, header=Header(self.frame_id), detections=[]
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
        Check if the tracker is currently tracking an object successfully.

        Returns:
            bool: True if tracking is active and REID is confirmed
        """
        return self.tracking_initialized and self.reid_confirmed

    def _process_tracking(self):
        """Process current frame for tracking and publish 2D detections."""
        if self.tracker is None or not self.tracking_initialized:
            return

        # Get frame copy
        with self._frame_lock:
            if self._latest_rgb_frame is None:
                return
            frame = self._latest_rgb_frame.copy()

        # Perform tracker update
        tracker_succeeded, bbox_cv = self.tracker.update(frame)

        current_bbox_x1y1x2y2 = None
        final_success = False

        if tracker_succeeded:
            x, y, w, h = map(int, bbox_cv)
            current_bbox_x1y1x2y2 = [x, y, x + w, y + h]

            # Perform re-ID check
            reid_confirmed_this_frame = self._reid(frame, current_bbox_x1y1x2y2)
            self.reid_confirmed = reid_confirmed_this_frame

            if reid_confirmed_this_frame:
                self.reid_fail_count = 0
            else:
                self.reid_fail_count += 1
        else:
            self.reid_confirmed = False

        # Determine final success
        if tracker_succeeded:
            if self.reid_fail_count >= self.reid_fail_tolerance:
                logger.warning(
                    f"Re-ID failed consecutively {self.reid_fail_count} times. Target lost."
                )
                self._reset_tracking_state()
                return
            else:
                final_success = True
        else:
            if self.tracking_initialized:
                logger.info("Tracker update failed. Stopping track.")
                self._reset_tracking_state()
            return

        self.tracking_frame_count += 1

        # Skip publishing if REID not confirmed after warmup
        if not self.reid_confirmed and self.tracking_frame_count >= self.reid_warmup_frames:
            return

        # Create 2D detection
        header = Header(self.frame_id)
        detection2darray = Detection2DArray(detections_length=0, header=header, detections=[])

        if final_success and current_bbox_x1y1x2y2 is not None:
            x1, y1, x2, y2 = current_bbox_x1y1x2y2
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            width = float(x2 - x1)
            height = float(y2 - y1)

            # Create Detection2D
            detection_2d = Detection2D()
            detection_2d.id = "0"
            detection_2d.results_length = 1
            detection_2d.header = header

            # Create hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = "tracked_object"
            hypothesis.hypothesis.score = 1.0
            detection_2d.results = [hypothesis]

            # Create bounding box
            detection_2d.bbox.center.position.x = center_x
            detection_2d.bbox.center.position.y = center_y
            detection_2d.bbox.center.theta = 0.0
            detection_2d.bbox.size_x = width
            detection_2d.bbox.size_y = height

            detection2darray = Detection2DArray()
            detection2darray.detections_length = 1
            detection2darray.header = header
            detection2darray.detections = [detection_2d]

        # Store and publish
        self._latest_detection2d = detection2darray
        self.detection2darray.publish(detection2darray)

        # Create visualization if tracking is active
        if self.tracking_initialized and current_bbox_x1y1x2y2:
            viz_image = self._draw_visualization(frame, current_bbox_x1y1x2y2)
            viz_msg = Image.from_numpy(viz_image)
            self.tracked_overlay.publish(viz_msg)

    def _draw_visualization(self, image: np.ndarray, bbox: List[int]) -> np.ndarray:
        """Draw tracking visualization."""
        viz_image = image.copy()

        x1, y1, x2, y2 = bbox

        # Draw bbox
        cv2.rectangle(viz_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Draw Re-ID matches if available
        if self.last_good_matches and self.last_roi_kps and self.last_roi_bbox:
            roi_x1, roi_y1, _roi_x2, _roi_y2 = self.last_roi_bbox

            # Draw keypoints
            for kp in self.last_roi_kps:
                pt = (int(kp.pt[0] + roi_x1), int(kp.pt[1] + roi_y1))
                cv2.circle(viz_image, pt, 3, (0, 255, 0), -1)

            # Draw matched keypoints
            for match in self.last_good_matches:
                current_kp = self.last_roi_kps[match.trainIdx]
                pt_current = (int(current_kp.pt[0] + roi_x1), int(current_kp.pt[1] + roi_y1))
                cv2.circle(viz_image, pt_current, 5, (0, 255, 255), 2)

                intensity = int(255 * (1.0 - min(match.distance / 100.0, 1.0)))
                cv2.circle(viz_image, pt_current, 2, (intensity, intensity, 255), -1)

            # Draw match count
            text = f"REID: {len(self.last_good_matches)}/{len(self.last_roi_kps)}"
            cv2.putText(viz_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Draw status
        if self.tracking_frame_count < self.reid_warmup_frames:
            status = f"WARMUP ({self.tracking_frame_count}/{self.reid_warmup_frames})"
            color = (255, 255, 0)
        elif self.reid_confirmed:
            status = "TRACKING"
            color = (0, 255, 0)
        else:
            status = f"WEAK ({self.reid_fail_count}/{self.reid_fail_tolerance})"
            color = (0, 165, 255)

        cv2.putText(viz_image, status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        return viz_image

    @rpc
    def cleanup(self):
        """Clean up resources."""
        self.stop_track()
        if self.tracking_thread and self.tracking_thread.is_alive():
            self.stop_tracking_event.set()
            self.tracking_thread.join(timeout=2.0)
