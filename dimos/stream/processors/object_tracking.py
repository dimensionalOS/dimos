# Copyright 2025-2026 Dimensional Inc.
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

"""Object tracking stream processor.

This module provides a modular object tracking processor that can be optionally
loaded if Metric3D and OpenCV dependencies are available.
"""

from typing import Any, Dict, Optional
import numpy as np
import cv2
import logging

from ..stream_processor import StreamProcessor
from dimos.perception.common.ibvs import ObjectDistanceEstimator
from dimos.models.depth.metric3d import Metric3D
from dimos.perception.detection2d.utils import calculate_depth_from_bbox

logger = logging.getLogger(__name__)


class ObjectTrackingProcessor(StreamProcessor):
    """Stream processor for object tracking using OpenCV CSRT tracker with depth estimation."""

    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the object tracking processor.

        Args:
            name: Unique name for this processor
            config: Configuration dictionary with the following keys:
                - camera_intrinsics: Camera intrinsics [fx, fy, cx, cy]
                - camera_pitch: Camera pitch in radians (default: 0.0)
                - camera_height: Camera height in meters (default: 1.0)
                - reid_threshold: Minimum good feature matches for re-ID (default: 5)
                - reid_fail_tolerance: Max consecutive re-ID failures (default: 10)
                - gt_depth_scale: Ground truth depth scale factor (default: 1000.0)
                - device: Device for depth model (default: "cpu")
        """
        super().__init__(name, config)

        # Set default configuration
        self.camera_intrinsics = self.config.get("camera_intrinsics")
        self.camera_pitch = self.config.get("camera_pitch", 0.0)
        self.camera_height = self.config.get("camera_height", 1.0)
        self.reid_threshold = self.config.get("reid_threshold", 5)
        self.reid_fail_tolerance = self.config.get("reid_fail_tolerance", 10)
        self.gt_depth_scale = self.config.get("gt_depth_scale", 1000.0)
        self.device = self.config.get("device", "cpu")

        # Initialize tracking state
        self.tracker = None
        self.tracking_bbox = None
        self.tracking_initialized = False
        self.orb = None
        self.bf = None
        self.original_des = None
        self.reid_fail_count = 0

        # Initialize components
        self.distance_estimator = None
        self.depth_model = None

    def initialize(self) -> bool:
        """Initialize the object tracker and depth estimation."""
        try:
            # Initialize ORB feature detector and matcher
            self.orb = cv2.ORB_create()
            self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

            # Initialize distance estimator if camera intrinsics are provided
            if self.camera_intrinsics is not None:
                if (
                    not isinstance(self.camera_intrinsics, (list, tuple, np.ndarray))
                    or len(self.camera_intrinsics) != 4
                ):
                    raise ValueError("Camera intrinsics must be provided as [fx, fy, cx, cy]")

                fx, fy, cx, cy = self.camera_intrinsics
                K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

                self.distance_estimator = ObjectDistanceEstimator(
                    K=K, camera_pitch=self.camera_pitch, camera_height=self.camera_height
                )

                # Initialize depth model
                self.depth_model = Metric3D(self.gt_depth_scale)
                self.depth_model.update_intrinsic(self.camera_intrinsics)

                logger.info(f"Initialized object tracking with depth estimation")
            else:
                logger.info(
                    f"Initialized object tracking without depth estimation (no camera intrinsics)"
                )

            return True

        except Exception as e:
            logger.error(f"Failed to initialize object tracking processor: {e}")
            return False

    def track(self, bbox, frame=None, distance=None, size=None):
        """Set the initial bounding box for tracking.

        Args:
            bbox: Bounding box in format [x1, y1, x2, y2]
            frame: Optional - Current frame for depth estimation and feature extraction
            distance: Optional - Known distance to object (meters)
            size: Optional - Known size of object (meters)

        Returns:
            bool: True if intention to track is set (bbox is valid)
        """
        x1, y1, x2, y2 = map(int, bbox)
        w, h = x2 - x1, y2 - y1
        if w <= 0 or h <= 0:
            logger.warning(f"Invalid initial bbox provided: {bbox}. Tracking not started.")
            self.stop_track()
            return False

        self.tracking_bbox = (x1, y1, w, h)  # Store in (x, y, w, h) format
        self.tracker = cv2.legacy.TrackerCSRT_create()
        self.tracking_initialized = False
        self.original_des = None
        self.reid_fail_count = 0

        logger.info(f"Tracking target set with bbox: {self.tracking_bbox}")

        # Calculate depth only if distance and size not provided
        depth_estimate = None
        if frame is not None and distance is None and size is None and self.depth_model is not None:
            depth_estimate = calculate_depth_from_bbox(self.depth_model, frame, bbox)
            if depth_estimate is not None:
                logger.info(f"Estimated depth for object: {depth_estimate:.2f}m")

        # Update distance estimator if needed
        if self.distance_estimator is not None:
            if size is not None:
                self.distance_estimator.set_estimated_object_size(size)
            elif distance is not None:
                self.distance_estimator.estimate_object_size(bbox, distance)
            elif depth_estimate is not None:
                self.distance_estimator.estimate_object_size(bbox, depth_estimate)
            else:
                logger.info("No distance or size provided. Cannot estimate object size.")

        return True

    def stop_track(self):
        """Stop tracking the current object."""
        self.tracker = None
        self.tracking_bbox = None
        self.tracking_initialized = False
        self.original_des = None
        self.reid_fail_count = 0
        return True

    def reid(self, frame, current_bbox) -> bool:
        """Check if features in current_bbox match stored original features."""
        if self.original_des is None:
            return True  # Cannot re-id if no original features

        x1, y1, x2, y2 = map(int, current_bbox)
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return False

        _, des_current = self.orb.detectAndCompute(roi, None)
        if des_current is None or len(des_current) < 2:
            return False

        # Handle case where original_des has only 1 descriptor
        if len(self.original_des) < 2:
            matches = self.bf.match(self.original_des, des_current)
            good_matches = len(matches)
        else:
            matches = self.bf.knnMatch(self.original_des, des_current, k=2)
            good_matches = 0
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.75 * n.distance:
                        good_matches += 1

        return good_matches >= self.reid_threshold

    def process_frame(
        self, frame: Any, metadata: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Process a frame for object tracking.

        Args:
            frame: Input video frame
            metadata: Optional metadata

        Returns:
            Dict containing tracking results and visualization
        """
        viz_frame = frame.copy()
        tracker_succeeded = False
        reid_confirmed_this_frame = False
        final_success = False
        target_data = None
        current_bbox_x1y1x2y2 = None

        if self.tracker is not None and self.tracking_bbox is not None:
            if not self.tracking_initialized:
                # Extract initial features and initialize tracker on first frame
                x_init, y_init, w_init, h_init = self.tracking_bbox
                roi = frame[y_init : y_init + h_init, x_init : x_init + w_init]

                if roi.size > 0:
                    _, self.original_des = self.orb.detectAndCompute(roi, None)
                    if self.original_des is None:
                        logger.warning(
                            "No ORB features found in initial ROI during stream processing."
                        )
                    else:
                        logger.info(f"Initial ORB features extracted: {len(self.original_des)}")

                    # Initialize the tracker
                    init_success = self.tracker.init(frame, self.tracking_bbox)
                    if init_success:
                        self.tracking_initialized = True
                        tracker_succeeded = True
                        reid_confirmed_this_frame = True
                        current_bbox_x1y1x2y2 = [x_init, y_init, x_init + w_init, y_init + h_init]
                        logger.info("Tracker initialized successfully.")
                    else:
                        logger.error("Tracker initialization failed in stream.")
                        self.stop_track()
                else:
                    logger.error("Empty ROI during tracker initialization in stream.")
                    self.stop_track()

            else:  # Tracker already initialized, perform update and re-id
                tracker_succeeded, bbox_cv = self.tracker.update(frame)
                if tracker_succeeded:
                    x, y, w, h = map(int, bbox_cv)
                    current_bbox_x1y1x2y2 = [x, y, x + w, y + h]
                    reid_confirmed_this_frame = self.reid(frame, current_bbox_x1y1x2y2)

                    if reid_confirmed_this_frame:
                        self.reid_fail_count = 0
                    else:
                        self.reid_fail_count += 1
                        logger.debug(
                            f"Re-ID failed ({self.reid_fail_count}/{self.reid_fail_tolerance})"
                        )

        # Determine final success
        if tracker_succeeded:
            if self.reid_fail_count >= self.reid_fail_tolerance:
                logger.info(
                    f"Re-ID failed consecutively {self.reid_fail_count} times. Target lost."
                )
                final_success = False
            else:
                final_success = True
        else:
            final_success = False
            if self.tracking_initialized:
                logger.info("Tracker update failed. Stopping track.")

        # Post-processing based on final_success
        if final_success and current_bbox_x1y1x2y2 is not None:
            x1, y1, x2, y2 = current_bbox_x1y1x2y2
            viz_color = (0, 255, 0) if reid_confirmed_this_frame else (0, 165, 255)
            cv2.rectangle(viz_frame, (x1, y1), (x2, y2), viz_color, 2)

            target_data = {
                "target_id": 0,
                "bbox": current_bbox_x1y1x2y2,
                "confidence": 1.0,
                "reid_confirmed": reid_confirmed_this_frame,
                "class_name": "object",
            }

            dist_text = "Object Tracking"
            if not reid_confirmed_this_frame:
                dist_text += " (Re-ID Failed - Tolerated)"

            if (
                self.distance_estimator is not None
                and self.distance_estimator.estimated_object_size is not None
            ):
                distance, angle = self.distance_estimator.estimate_distance_angle(
                    current_bbox_x1y1x2y2
                )
                if distance is not None:
                    target_data["distance"] = distance
                    target_data["angle"] = angle
                    dist_text = f"Object: {distance:.2f}m, {np.rad2deg(angle):.1f} deg"
                    if not reid_confirmed_this_frame:
                        dist_text += " (Re-ID Failed - Tolerated)"

            text_size = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
            label_bg_y = max(y1 - text_size[1] - 5, 0)
            cv2.rectangle(viz_frame, (x1, label_bg_y), (x1 + text_size[0], y1), (0, 0, 0), -1)
            cv2.putText(
                viz_frame,
                dist_text,
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

        elif self.tracking_initialized:
            self.stop_track()

        return {
            "frame": frame,
            "viz_frame": viz_frame,
            "targets": [target_data] if target_data else [],
            "processor": self.name,
            "metadata": metadata or {},
        }

    def cleanup(self) -> None:
        """Clean up resources."""
        self.stop_track()
        self.orb = None
        self.bf = None
        self.distance_estimator = None
        self.depth_model = None
