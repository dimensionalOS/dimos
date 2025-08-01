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
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.geometry_msgs import Vector3, Quaternion, Transform, Pose
from dimos.protocol.tf import TF
from dimos.utils.logging_config import setup_logger

# Import LCM messages
from dimos_lcm.std_msgs import Header
from dimos_lcm.vision_msgs import (
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from dimos_lcm.geometry_msgs import Point
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.utils.transform_utils import (
    yaw_towards_point,
    optical_to_robot_frame,
    euler_to_quaternion,
)
from dimos.manipulation.visual_servoing.utils import visualize_detections_3d

logger = setup_logger("dimos.perception.object_tracker")


class ObjectTracking(Module):
    """Module for object tracking with LCM input/output."""

    # LCM inputs
    rgb_image: In[Image] = None
    depth: In[Image] = None
    camera_info: In[CameraInfo] = None

    # LCM outputs
    detection2darray: Out[Detection2DArray] = None
    detection3darray: Out[Detection3DArray] = None
    tracked_overlay: Out[Image] = None  # Visualization output

    def __init__(
        self,
        camera_intrinsics: Optional[List[float]] = None,  # [fx, fy, cx, cy]
        reid_threshold: int = 5,
        reid_fail_tolerance: int = 10,
        frame_id: str = "camera_link",
    ):
        """
        Initialize an object tracking module using OpenCV's CSRT tracker with ORB re-ID.

        Args:
            camera_intrinsics: Optional [fx, fy, cx, cy] camera parameters.
                              If None, will use camera_info input.
            reid_threshold: Minimum good feature matches needed to confirm re-ID.
            reid_fail_tolerance: Number of consecutive frames Re-ID can fail before
                                 tracking is stopped.
            frame_id: TF frame ID for the camera (default: "camera_link")
        """
        # Call parent Module init
        super().__init__()

        self.camera_intrinsics = camera_intrinsics
        self._camera_info_received = False
        self.reid_threshold = reid_threshold
        self.reid_fail_tolerance = reid_fail_tolerance
        self.frame_id = frame_id

        self.tracker = None
        self.tracking_bbox = None  # Stores (x, y, w, h) for tracker initialization
        self.tracking_initialized = False
        self.orb = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.original_des = None  # Store original ORB descriptors
        self.reid_fail_count = 0  # Counter for consecutive re-id failures

        # For tracking latest frame data
        self._latest_rgb_frame: Optional[np.ndarray] = None
        self._latest_depth_frame: Optional[np.ndarray] = None
        self._latest_camera_info: Optional[CameraInfo] = None

        # Tracking thread control
        self.tracking_thread: Optional[threading.Thread] = None
        self.stop_tracking = threading.Event()
        self.tracking_rate = 30.0  # Hz
        self.tracking_period = 1.0 / self.tracking_rate

        # Initialize TF publisher
        self.tf = TF()

    @rpc
    def start(self):
        """Start the object tracking module and subscribe to LCM streams."""

        # Subscribe to rgb image stream
        def on_rgb(image_msg: Image):
            self._latest_rgb_frame = image_msg.data

        self.rgb_image.subscribe(on_rgb)

        # Subscribe to depth stream
        def on_depth(image_msg: Image):
            self._latest_depth_frame = image_msg.data

        self.depth.subscribe(on_depth)

        # Subscribe to camera info stream
        def on_camera_info(camera_info_msg: CameraInfo):
            self._latest_camera_info = camera_info_msg
            # Extract intrinsics from camera info K matrix
            # K is a 3x3 matrix in row-major order: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.camera_intrinsics = [
                camera_info_msg.K[0],
                camera_info_msg.K[4],
                camera_info_msg.K[2],
                camera_info_msg.K[5],
            ]
            if not self._camera_info_received:
                self._camera_info_received = True
                logger.info(
                    f"Camera intrinsics received from camera_info: {self.camera_intrinsics}"
                )

        self.camera_info.subscribe(on_camera_info)

        logger.info("ObjectTracking module started and subscribed to LCM streams")

    @rpc
    def track(
        self,
        bbox: List[float],
    ) -> Dict:
        """
        Initialize tracking with a bounding box and process current frame.

        Args:
            bbox: Bounding box in format [x1, y1, x2, y2]

        Returns:
            Dict containing tracking results with 2D and 3D detections
        """
        if self._latest_rgb_frame is None:
            logger.warning("No RGB frame available for tracking")
            return {"detection2darray": Detection2DArray(), "detection3darray": Detection3DArray()}

        # Initialize tracking
        x1, y1, x2, y2 = map(int, bbox)
        w, h = x2 - x1, y2 - y1
        if w <= 0 or h <= 0:
            logger.warning(f"Invalid initial bbox provided: {bbox}. Tracking not started.")
            return {"detection2darray": Detection2DArray(), "detection3darray": Detection3DArray()}

        # Set tracking parameters
        self.tracking_bbox = (x1, y1, w, h)  # Store in (x, y, w, h) format
        self.tracker = cv2.legacy.TrackerCSRT_create()
        self.tracking_initialized = False
        self.original_des = None
        self.reid_fail_count = 0
        logger.info(f"Tracking target set with bbox: {self.tracking_bbox}")

        # Extract initial features
        roi = self._latest_rgb_frame[y1:y2, x1:x2]
        if roi.size > 0:
            _, self.original_des = self.orb.detectAndCompute(roi, None)
            if self.original_des is None:
                logger.warning("No ORB features found in initial ROI.")
            else:
                logger.info(f"Initial ORB features extracted: {len(self.original_des)}")

            # Initialize the tracker
            init_success = self.tracker.init(self._latest_rgb_frame, self.tracking_bbox)
            if init_success:
                self.tracking_initialized = True
                logger.info("Tracker initialized successfully.")
            else:
                logger.error("Tracker initialization failed.")
                self.stop_track()
        else:
            logger.error("Empty ROI during tracker initialization.")
            self.stop_track()

        # Start tracking thread
        self._start_tracking_thread()

        # Return initial tracking result
        return {"status": "tracking_started", "bbox": self.tracking_bbox}

    def reid(self, frame, current_bbox) -> bool:
        """Check if features in current_bbox match stored original features."""
        if self.original_des is None:
            return True  # Cannot re-id if no original features
        x1, y1, x2, y2 = map(int, current_bbox)
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return False  # Empty ROI cannot match

        _, des_current = self.orb.detectAndCompute(roi, None)
        if des_current is None or len(des_current) < 2:
            return False  # Need at least 2 descriptors for knnMatch

        # Handle case where original_des has only 1 descriptor (cannot use knnMatch with k=2)
        if len(self.original_des) < 2:
            matches = self.bf.match(self.original_des, des_current)
            good_matches = len(matches)
        else:
            matches = self.bf.knnMatch(self.original_des, des_current, k=2)
            # Apply Lowe's ratio test robustly
            good_matches = 0
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.75 * n.distance:
                        good_matches += 1

        return good_matches >= self.reid_threshold

    def _start_tracking_thread(self):
        """Start the tracking thread."""
        self.stop_tracking.clear()
        self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.tracking_thread.start()
        logger.info("Started tracking thread")

    def _tracking_loop(self):
        """Main tracking loop that runs in a separate thread."""
        while not self.stop_tracking.is_set() and self.tracking_initialized:
            # Process tracking for current frame
            self._process_tracking()

            # Sleep to maintain tracking rate
            time.sleep(self.tracking_period)

        logger.info("Tracking loop ended")

    def _reset_tracking_state(self):
        """Reset tracking state without stopping the thread."""
        self.tracker = None
        self.tracking_bbox = None
        self.tracking_initialized = False
        self.original_des = None
        self.reid_fail_count = 0  # Reset counter

        # Publish empty detections to clear any visualizations
        empty_2d = Detection2DArray(detections_length=0, header=Header(), detections=[])
        empty_3d = Detection3DArray(detections_length=0, header=Header(), detections=[])
        self.detection2darray.publish(empty_2d)
        self.detection3darray.publish(empty_3d)

    @rpc
    def stop_track(self) -> bool:
        """
        Stop tracking the current object.
        This resets the tracker and all tracking state.

        Returns:
            bool: True if tracking was successfully stopped
        """
        # Reset tracking state first
        self._reset_tracking_state()

        # Stop tracking thread if running (only if called from outside the thread)
        if self.tracking_thread and self.tracking_thread.is_alive():
            # Check if we're being called from within the tracking thread
            if threading.current_thread() != self.tracking_thread:
                self.stop_tracking.set()
                self.tracking_thread.join(timeout=1.0)
                self.tracking_thread = None
            else:
                # If called from within thread, just set the stop flag
                self.stop_tracking.set()

        logger.info("Tracking stopped")
        return True

    def _process_tracking(self):
        """Process current frame for tracking and publish detections."""
        if self._latest_rgb_frame is None or self.tracker is None or not self.tracking_initialized:
            return

        frame = self._latest_rgb_frame
        tracker_succeeded = False
        reid_confirmed_this_frame = False
        final_success = False
        current_bbox_x1y1x2y2 = None

        # Perform tracker update
        tracker_succeeded, bbox_cv = self.tracker.update(frame)
        if tracker_succeeded:
            x, y, w, h = map(int, bbox_cv)
            current_bbox_x1y1x2y2 = [x, y, x + w, y + h]
            # Perform re-ID check
            reid_confirmed_this_frame = self.reid(frame, current_bbox_x1y1x2y2)

            if reid_confirmed_this_frame:
                self.reid_fail_count = 0
            else:
                self.reid_fail_count += 1

        # Determine final success
        if tracker_succeeded:
            if self.reid_fail_count >= self.reid_fail_tolerance:
                logger.warning(
                    f"Re-ID failed consecutively {self.reid_fail_count} times. Target lost."
                )
                final_success = False
                self._reset_tracking_state()
            else:
                final_success = True
        else:
            final_success = False
            if self.tracking_initialized:
                logger.info("Tracker update failed. Stopping track.")
                self._reset_tracking_state()

        # Create detections if tracking succeeded
        detection2darray = Detection2DArray(detections_length=0, header=Header(), detections=[])
        detection3darray = Detection3DArray(detections_length=0, header=Header(), detections=[])

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
            detection_2d.header = Header()

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
            detection2darray.header = Header()
            detection2darray.detections = [detection_2d]

            # Create Detection3D if depth is available
            if self._latest_depth_frame is not None:
                # Calculate 3D position using depth and camera intrinsics
                depth_value = self._calculate_depth_at_center(current_bbox_x1y1x2y2)
                if (
                    depth_value is not None
                    and depth_value > 0
                    and self.camera_intrinsics is not None
                ):
                    fx, fy, cx, cy = self.camera_intrinsics

                    # Convert pixel coordinates to 3D in optical frame
                    z_optical = depth_value
                    x_optical = (center_x - cx) * z_optical / fx
                    y_optical = (center_y - cy) * z_optical / fy

                    # Create pose in optical frame
                    optical_pose = Pose()
                    optical_pose.position = Point(x_optical, y_optical, z_optical)
                    optical_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)  # Identity for now

                    # Convert to robot frame
                    robot_pose = optical_to_robot_frame(optical_pose)

                    # Calculate orientation: object facing towards camera (origin)
                    yaw = yaw_towards_point(robot_pose.position)
                    euler = Vector3(0.0, 0.0, yaw)  # Only yaw, no roll/pitch
                    robot_pose.orientation = euler_to_quaternion(euler)

                    # Estimate object size in meters
                    size_x = width * z_optical / fx
                    size_y = height * z_optical / fy
                    size_z = 0.1  # Default depth size

                    # Create Detection3D
                    detection_3d = Detection3D()
                    detection_3d.id = "0"
                    detection_3d.results_length = 1
                    detection_3d.header = Header()

                    # Reuse hypothesis from 2D
                    detection_3d.results = [hypothesis]

                    # Create 3D bounding box with robot frame pose
                    detection_3d.bbox.center = Pose()
                    detection_3d.bbox.center.position = robot_pose.position
                    detection_3d.bbox.center.orientation = robot_pose.orientation
                    detection_3d.bbox.size = Vector3(size_x, size_y, size_z)

                    detection3darray = Detection3DArray()
                    detection3darray.detections_length = 1
                    detection3darray.header = Header()
                    detection3darray.detections = [detection_3d]

                    # Publish transform for tracked object
                    # The optical pose is in camera optical frame, so publish it relative to the camera frame
                    tracked_object_tf = Transform(
                        translation=robot_pose.position,
                        rotation=robot_pose.orientation,
                        frame_id=self.frame_id,  # Use configured camera frame
                        child_frame_id=f"tracked_object",
                        ts=time.time(),
                    )
                    self.tf.publish(tracked_object_tf)

        # Publish detections
        self.detection2darray.publish(detection2darray)
        self.detection3darray.publish(detection3darray)

        # Create and publish visualization if tracking is active
        if self.tracking_initialized and self._latest_rgb_frame is not None:
            # Convert single detection to list for visualization
            detections_3d = (
                detection3darray.detections if detection3darray.detections_length > 0 else []
            )
            detections_2d = (
                detection2darray.detections if detection2darray.detections_length > 0 else []
            )

            if detections_3d and detections_2d:
                # Extract 2D bbox for visualization
                det_2d = detections_2d[0]
                bbox_2d = []
                if det_2d.bbox:
                    x1 = det_2d.bbox.center.position.x - det_2d.bbox.size_x / 2
                    y1 = det_2d.bbox.center.position.y - det_2d.bbox.size_y / 2
                    x2 = det_2d.bbox.center.position.x + det_2d.bbox.size_x / 2
                    y2 = det_2d.bbox.center.position.y + det_2d.bbox.size_y / 2
                    bbox_2d = [[x1, y1, x2, y2]]

                # Create visualization
                viz_image = visualize_detections_3d(
                    self._latest_rgb_frame, detections_3d, show_coordinates=True, bboxes_2d=bbox_2d
                )

                # Convert to Image message and publish
                viz_msg = Image.from_numpy(viz_image)
                self.tracked_overlay.publish(viz_msg)

    def _calculate_depth_at_center(self, bbox: List[int]) -> Optional[float]:
        """Calculate depth at the center of the bounding box using a square region."""
        if self._latest_depth_frame is None:
            return None

        x1, y1, x2, y2 = bbox
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)

        # Use a square region around the center
        margin = 5
        y_start = max(0, center_y - margin)
        y_end = min(self._latest_depth_frame.shape[0], center_y + margin)
        x_start = max(0, center_x - margin)
        x_end = min(self._latest_depth_frame.shape[1], center_x + margin)

        roi_depth = self._latest_depth_frame[y_start:y_end, x_start:x_end]
        valid_depths = roi_depth[np.isfinite(roi_depth) & (roi_depth > 0)]

        if len(valid_depths) > 0:
            return float(np.median(valid_depths))

        return None

    @rpc
    def cleanup(self):
        """Clean up resources."""
        self.stop_track()

        # Ensure thread is stopped
        if self.tracking_thread and self.tracking_thread.is_alive():
            self.stop_tracking.set()
            self.tracking_thread.join(timeout=2.0)
