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
import threading
import time

import cv2

# Import LCM messages
from dimos_lcm.vision_msgs import (  # type: ignore[import-untyped]
    Detection2D,
    Detection3D,
    ObjectHypothesisWithPose,
)
import numpy as np
from reactivex.disposable import Disposable

from dimos.core import In, Module, ModuleConfig, Out, rpc
from dimos.manipulation.visual_servoing.utils import visualize_detections_3d
from dimos.msgs.geometry_msgs import Pose, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import (
    CameraInfo,  # type: ignore[import-untyped]
    Image,
    ImageFormat,
)
from dimos.msgs.std_msgs import Header
from dimos.msgs.vision_msgs import Detection2DArray, Detection3DArray
from dimos.protocol.tf import TF
from dimos.types.timestamped import align_timestamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import (
    euler_to_quaternion,
    optical_to_robot_frame,
    yaw_towards_point,
)

logger = setup_logger()


@dataclass
class ObjectTrackingConfig(ModuleConfig):
    frame_id: str = "camera_link"


class ObjectTracking(Module[ObjectTrackingConfig]):
    """Module for object tracking with LCM input/output."""

    # LCM inputs
    color_image: In[Image]
    depth: In[Image]
    camera_info: In[CameraInfo]

    # LCM outputs
    detection2darray: Out[Detection2DArray]
    detection3darray: Out[Detection3DArray]
    tracked_overlay: Out[Image]  # Visualization output

    default_config = ObjectTrackingConfig
    config: ObjectTrackingConfig

    def __init__(
        self, reid_threshold: int = 10, reid_fail_tolerance: int = 5, **kwargs: object
    ) -> None:
        """
        Initialize an object tracking module using OpenCV's CSRT tracker with ORB re-ID.

        Args:
            camera_intrinsics: Optional [fx, fy, cx, cy] camera parameters.
                              If None, will use camera_info input.
            reid_threshold: Minimum good feature matches needed to confirm re-ID.
            reid_fail_tolerance: Number of consecutive frames Re-ID can fail before
                                 tracking is stopped.
        """
        # Call parent Module init
        super().__init__(**kwargs)

        self.camera_intrinsics = None
        self.reid_threshold = reid_threshold
        self.reid_fail_tolerance = reid_fail_tolerance

        self.tracker = None
        self.tracking_bbox = None  # Stores (x, y, w, h) for tracker initialization
        self.tracking_initialized = False
        self.orb = cv2.ORB_create()  # type: ignore[attr-defined]
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.original_des = None  # Store original ORB descriptors
        self.original_kps = None  # Store original ORB keypoints
        self.reid_fail_count = 0  # Counter for consecutive re-id failures
        self.last_good_matches = []  # type: ignore[var-annotated]  # Store good matches for visualization
        self.last_roi_kps = None  # Store last ROI keypoints for visualization
        self.last_roi_bbox = None  # Store last ROI bbox for visualization
        self.reid_confirmed = False  # Store current reid confirmation state
        self.tracking_frame_count = 0  # Count frames since tracking started
        self.reid_warmup_frames = 3  # Number of frames before REID starts

        self._frame_lock = threading.Lock()
        self._latest_rgb_frame: np.ndarray | None = None  # type: ignore[type-arg]
        self._latest_depth_frame: np.ndarray | None = None  # type: ignore[type-arg]
        self._latest_camera_info: CameraInfo | None = None

        # Tracking thread control
        self.tracking_thread: threading.Thread | None = None
        self.stop_tracking = threading.Event()
        self.tracking_rate = 30.0  # Hz
        self.tracking_period = 1.0 / self.tracking_rate

        # Initialize TF publisher
        self.tf = TF()

        # Store latest detections for RPC access
        self._latest_detection2d: Detection2DArray | None = None
        self._latest_detection3d: Detection3DArray | None = None
        self._detection_event = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()

        # Subscribe to aligned rgb and depth streams
        def on_aligned_frames(frames_tuple) -> None:  # type: ignore[no-untyped-def]
            rgb_msg, depth_msg = frames_tuple
            with self._frame_lock:
                self._latest_rgb_frame = rgb_msg.data

                depth_data = depth_msg.data
                # Convert from millimeters to meters if depth is DEPTH16 format
                if depth_msg.format == ImageFormat.DEPTH16:
                    depth_data = depth_data.astype(np.float32) / 1000.0
                self._latest_depth_frame = depth_data

        # Create aligned observable for RGB and depth
        aligned_frames = align_timestamped(
            self.color_image.observable(),  # type: ignore[no-untyped-call]
            self.depth.observable(),  # type: ignore[no-untyped-call]
            buffer_size=2.0,  # 2 second buffer
            match_tolerance=0.5,  # 500ms tolerance
        )
        unsub = aligned_frames.subscribe(on_aligned_frames)
        self._disposables.add(unsub)

        # Subscribe to camera info stream separately (doesn't need alignment)
        def on_camera_info(camera_info_msg: CameraInfo) -> None:
            self._latest_camera_info = camera_info_msg
            # Extract intrinsics from camera info K matrix
            # K is a 3x3 matrix in row-major order: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.camera_intrinsics = [  # type: ignore[assignment]
                camera_info_msg.K[0],
                camera_info_msg.K[4],
                camera_info_msg.K[2],
                camera_info_msg.K[5],
            ]

        unsub = self.camera_info.subscribe(on_camera_info)  # type: ignore[assignment]
        self._disposables.add(Disposable(unsub))  # type: ignore[arg-type]

    @rpc
    def stop(self) -> None:
        self.stop_track()

        self.stop_tracking.set()

        if self.tracking_thread and self.tracking_thread.is_alive():
            self.tracking_thread.join(timeout=2.0)

        super().stop()

    @rpc
    def track(
        self,
        bbox: list[float],
    ) -> dict:  # type: ignore[type-arg]
        """
        Initialize tracking with a bounding box and process current frame.

        Args:
            bbox: Bounding box in format [x1, y1, x2, y2]

        Returns:
            Dict containing tracking results with 2D and 3D detections
        """
        if self._latest_rgb_frame is None:
            logger.warning("No RGB frame available for tracking")

        # Initialize tracking
        x1, y1, x2, y2 = map(int, bbox)
        w, h = x2 - x1, y2 - y1
        if w <= 0 or h <= 0:
            logger.warning(f"Invalid initial bbox provided: {bbox}. Tracking not started.")

        # Set tracking parameters
        self.tracking_bbox = (x1, y1, w, h)  # type: ignore[assignment]  # Store in (x, y, w, h) format
        self.tracker = cv2.legacy.TrackerCSRT_create()  # type: ignore[attr-defined]
        self.tracking_initialized = False
        self.original_des = None
        self.reid_fail_count = 0
        logger.info(f"Tracking target set with bbox: {self.tracking_bbox}")

        # Extract initial features
        roi = self._latest_rgb_frame[y1:y2, x1:x2]  # type: ignore[index]
        if roi.size > 0:
            self.original_kps, self.original_des = self.orb.detectAndCompute(roi, None)
            if self.original_des is None:
                logger.warning("No ORB features found in initial ROI. REID will be disabled.")
            else:
                logger.info(f"Initial ORB features extracted: {len(self.original_des)}")

            # Initialize the tracker
            init_success = self.tracker.init(self._latest_rgb_frame, self.tracking_bbox)  # type: ignore[attr-defined]
            if init_success:
                self.tracking_initialized = True
                self.tracking_frame_count = 0  # Reset frame counter
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

    def reid(self, frame, current_bbox) -> bool:  # type: ignore[no-untyped-def]
        """Check if features in current_bbox match stored original features."""
        # During warm-up period, always return True
        if self.tracking_frame_count < self.reid_warmup_frames:
            return True

        if self.original_des is None:
            return False
        x1, y1, x2, y2 = map(int, current_bbox)
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return False  # Empty ROI cannot match

        kps_current, des_current = self.orb.detectAndCompute(roi, None)
        if des_current is None or len(des_current) < 2:
            return False  # Need at least 2 descriptors for knnMatch

        # Store ROI keypoints and bbox for visualization
        self.last_roi_kps = kps_current
        self.last_roi_bbox = [x1, y1, x2, y2]

        # Handle case where original_des has only 1 descriptor (cannot use knnMatch with k=2)
        if len(self.original_des) < 2:
            matches = self.bf.match(self.original_des, des_current)
            self.last_good_matches = matches  # Store all matches for visualization
            good_matches = len(matches)
        else:
            matches = self.bf.knnMatch(self.original_des, des_current, k=2)
            # Apply Lowe's ratio test robustly
            good_matches_list = []
            good_matches = 0
            for match_pair in matches:
                if len(match_pair) == 2:
                    m, n = match_pair
                    if m.distance < 0.75 * n.distance:
                        good_matches_list.append(m)
                        good_matches += 1
            self.last_good_matches = good_matches_list  # Store good matches for visualization

        return good_matches >= self.reid_threshold

    def _start_tracking_thread(self) -> None:
        """Start the tracking thread."""
        self.stop_tracking.clear()
        self.tracking_thread = threading.Thread(target=self._tracking_loop, daemon=True)
        self.tracking_thread.start()
        logger.info("Started tracking thread")

    def _tracking_loop(self) -> None:
        """Main tracking loop that runs in a separate thread."""
        while not self.stop_tracking.is_set() and self.tracking_initialized:
            # Process tracking for current frame
            self._process_tracking()

            # Sleep to maintain tracking rate
            time.sleep(self.tracking_period)

        logger.info("Tracking loop ended")

    def _reset_tracking_state(self) -> None:
        """Reset tracking state without stopping the thread."""
        self.tracker = None
        self.tracking_bbox = None
        self.tracking_initialized = False
        self.original_des = None
        self.original_kps = None
        self.reid_fail_count = 0  # Reset counter
        self.last_good_matches = []
        self.last_roi_kps = None
        self.last_roi_bbox = None
        self.reid_confirmed = False  # Reset reid confirmation state
        self.tracking_frame_count = 0  # Reset frame counter

        # Publish empty detections to clear any visualizations
        empty_2d = Detection2DArray(detections_length=0, header=Header(), detections=[])
        empty_3d = Detection3DArray(detections_length=0, header=Header(), detections=[])
        self._latest_detection2d = empty_2d
        self._latest_detection3d = empty_3d
        self._detection_event.clear()
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

    @rpc
    def is_tracking(self) -> bool:
        """
        Check if the tracker is currently tracking an object successfully.

        Returns:
            bool: True if tracking is active and REID is confirmed, False otherwise
        """
        return self.tracking_initialized and self.reid_confirmed

    def _process_tracking(self) -> None:
        """Process current frame for tracking and publish detections."""
        if self.tracker is None or not self.tracking_initialized:
            return

        # Get local copies of frames under lock
        with self._frame_lock:
            if self._latest_rgb_frame is None or self._latest_depth_frame is None:
                return
            frame = self._latest_rgb_frame.copy()
            depth_frame = self._latest_depth_frame.copy()
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
            self.reid_confirmed = reid_confirmed_this_frame  # Store for is_tracking() RPC

            if reid_confirmed_this_frame:
                self.reid_fail_count = 0
            else:
                self.reid_fail_count += 1
        else:
            self.reid_confirmed = False  # No tracking if tracker failed

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

        self.tracking_frame_count += 1

        if not reid_confirmed_this_frame and self.tracking_frame_count >= self.reid_warmup_frames:
            return

        # Create detections if tracking succeeded
        header = Header(self.frame_id)
        detection2darray = Detection2DArray(detections_length=0, header=header, detections=[])
        detection3darray = Detection3DArray(detections_length=0, header=header, detections=[])

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

            # Create Detection3D if depth is available
            if depth_frame is not None:
                # Calculate 3D position using depth and camera intrinsics
                depth_value = self._get_depth_from_bbox(current_bbox_x1y1x2y2, depth_frame)
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
                    optical_pose.position = Vector3(x_optical, y_optical, z_optical)
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
                    detection_3d.header = header

                    # Reuse hypothesis from 2D
                    detection_3d.results = [hypothesis]

                    # Create 3D bounding box with robot frame pose
                    detection_3d.bbox.center = Pose()
                    detection_3d.bbox.center.position = robot_pose.position
                    detection_3d.bbox.center.orientation = robot_pose.orientation
                    detection_3d.bbox.size = Vector3(size_x, size_y, size_z)

                    detection3darray = Detection3DArray()
                    detection3darray.detections_length = 1
                    detection3darray.header = header
                    detection3darray.detections = [detection_3d]

                    # Publish transform for tracked object
                    # The optical pose is in camera optical frame, so publish it relative to the camera frame
                    tracked_object_tf = Transform(
                        translation=robot_pose.position,
                        rotation=robot_pose.orientation,
                        frame_id=self.frame_id,  # Use configured camera frame
                        child_frame_id="tracked_object",
                        ts=header.ts,
                    )
                    self.tf.publish(tracked_object_tf)

        # Store latest detections for RPC access
        self._latest_detection2d = detection2darray
        self._latest_detection3d = detection3darray

        # Signal that new detections are available
        if detection2darray.detections_length > 0 or detection3darray.detections_length > 0:
            self._detection_event.set()

        # Publish detections
        self.detection2darray.publish(detection2darray)
        self.detection3darray.publish(detection3darray)

        # Create and publish visualization if tracking is active
        if self.tracking_initialized:
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
                    frame, detections_3d, show_coordinates=True, bboxes_2d=bbox_2d
                )

                # Overlay REID feature matches if available
                if self.last_good_matches and self.last_roi_kps and self.last_roi_bbox:
                    viz_image = self._draw_reid_matches(viz_image)

                # Convert to Image message and publish
                viz_msg = Image.from_numpy(viz_image)
                self.tracked_overlay.publish(viz_msg)

    def _draw_reid_matches(self, image: np.ndarray) -> np.ndarray:  # type: ignore[type-arg]
        """Draw REID feature matches on the image."""
        viz_image = image.copy()

        x1, y1, _x2, _y2 = self.last_roi_bbox  # type: ignore[misc]

        # Draw keypoints from current ROI in green
        for kp in self.last_roi_kps:  # type: ignore[attr-defined]
            pt = (int(kp.pt[0] + x1), int(kp.pt[1] + y1))  # type: ignore[has-type]
            cv2.circle(viz_image, pt, 3, (0, 255, 0), -1)

        for match in self.last_good_matches:
            current_kp = self.last_roi_kps[match.trainIdx]  # type: ignore[index]
            pt_current = (int(current_kp.pt[0] + x1), int(current_kp.pt[1] + y1))  # type: ignore[has-type]

            # Draw a larger circle for matched points in yellow
            cv2.circle(viz_image, pt_current, 5, (0, 255, 255), 2)  # Yellow for matched points

            # Draw match strength indicator (smaller circle with intensity based on distance)
            # Lower distance = better match = brighter color
            intensity = int(255 * (1.0 - min(match.distance / 100.0, 1.0)))
            cv2.circle(viz_image, pt_current, 2, (intensity, intensity, 255), -1)

        text = f"REID Matches: {len(self.last_good_matches)}/{len(self.last_roi_kps) if self.last_roi_kps else 0}"
        cv2.putText(viz_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if self.tracking_frame_count < self.reid_warmup_frames:
            status_text = (
                f"REID: WARMING UP ({self.tracking_frame_count}/{self.reid_warmup_frames})"
            )
            status_color = (255, 255, 0)  # Yellow
        elif len(self.last_good_matches) >= self.reid_threshold:
            status_text = "REID: CONFIRMED"
            status_color = (0, 255, 0)  # Green
        else:
            status_text = f"REID: WEAK ({self.reid_fail_count}/{self.reid_fail_tolerance})"
            status_color = (0, 165, 255)  # Orange

        cv2.putText(
            viz_image, status_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2
        )

        return viz_image

    def _get_depth_from_bbox(self, bbox: list[int], depth_frame: np.ndarray) -> float | None:  # type: ignore[type-arg]
        """Calculate depth from bbox using the 25th percentile of closest points.

        Args:
            bbox: Bounding box coordinates [x1, y1, x2, y2]
            depth_frame: Depth frame to extract depth values from

        Returns:
            Depth value or None if not available
        """
        if depth_frame is None:
            return None

        x1, y1, x2, y2 = bbox

        # Ensure bbox is within frame bounds
        y1 = max(0, y1)
        y2 = min(depth_frame.shape[0], y2)
        x1 = max(0, x1)
        x2 = min(depth_frame.shape[1], x2)

        # Extract depth values from the entire bbox
        roi_depth = depth_frame[y1:y2, x1:x2]

        # Get valid (finite and positive) depth values
        valid_depths = roi_depth[np.isfinite(roi_depth) & (roi_depth > 0)]

        if len(valid_depths) > 0:
            depth_25th_percentile = float(np.percentile(valid_depths, 25))
            return depth_25th_percentile

        return None


object_tracking = ObjectTracking.blueprint

__all__ = ["ObjectTracking", "object_tracking"]
