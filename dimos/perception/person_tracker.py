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
from reactivex import Observable, interval, operators as ops
from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.msgs.sensor_msgs import Image
from dimos.perception.common.ibvs import PersonDistanceEstimator
from dimos.perception.detection2d.utils import filter_detections
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector  # type: ignore[import-untyped]
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PersonTrackingStream(Module):
    """Module for person tracking with LCM input/output."""

    # LCM inputs
    video: In[Image]

    # LCM outputs
    tracking_data: Out[dict]  # type: ignore[type-arg]

    def __init__(  # type: ignore[no-untyped-def]
        self,
        camera_intrinsics=None,
        camera_pitch: float = 0.0,
        camera_height: float = 1.0,
    ) -> None:
        """
        Initialize a person tracking stream using Yolo2DDetector and PersonDistanceEstimator.

        Args:
            camera_intrinsics: List in format [fx, fy, cx, cy] where:
                - fx: Focal length in x direction (pixels)
                - fy: Focal length in y direction (pixels)
                - cx: Principal point x-coordinate (pixels)
                - cy: Principal point y-coordinate (pixels)
            camera_pitch: Camera pitch angle in radians (positive is up)
            camera_height: Height of the camera from the ground in meters
        """
        # Call parent Module init
        super().__init__()

        self.camera_intrinsics = camera_intrinsics
        self.camera_pitch = camera_pitch
        self.camera_height = camera_height

        self.detector = Yolo2DDetector()

        # Initialize distance estimator
        if camera_intrinsics is None:
            raise ValueError("Camera intrinsics are required for distance estimation")

        # Validate camera intrinsics format [fx, fy, cx, cy]
        if (
            not isinstance(camera_intrinsics, list | tuple | np.ndarray)
            or len(camera_intrinsics) != 4
        ):
            raise ValueError("Camera intrinsics must be provided as [fx, fy, cx, cy]")

        # Convert [fx, fy, cx, cy] to 3x3 camera matrix
        fx, fy, cx, cy = camera_intrinsics
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)

        self.distance_estimator = PersonDistanceEstimator(
            K=K, camera_pitch=camera_pitch, camera_height=camera_height
        )

        # For tracking latest frame data
        self._latest_frame: np.ndarray | None = None  # type: ignore[type-arg]
        self._process_interval = 0.1  # Process at 10Hz

        # Tracking state - starts disabled
        self._tracking_enabled = False

    @rpc
    def start(self) -> None:
        """Start the person tracking module and subscribe to LCM streams."""

        super().start()

        # Subscribe to video stream
        def set_video(image_msg: Image) -> None:
            if hasattr(image_msg, "data"):
                self._latest_frame = image_msg.data
            else:
                logger.warning("Received image message without data attribute")

        unsub = self.video.subscribe(set_video)
        self._disposables.add(Disposable(unsub))

        # Start periodic processing
        unsub = interval(self._process_interval).subscribe(lambda _: self._process_frame())  # type: ignore[assignment]
        self._disposables.add(unsub)  # type: ignore[arg-type]

        logger.info("PersonTracking module started and subscribed to LCM streams")

    @rpc
    def stop(self) -> None:
        super().stop()

    def _process_frame(self) -> None:
        """Process the latest frame if available."""
        if self._latest_frame is None:
            return

        # Only process and publish if tracking is enabled
        if not self._tracking_enabled:
            return

        # Process frame through tracking pipeline
        result = self._process_tracking(self._latest_frame)  # type: ignore[no-untyped-call]

        # Publish result to LCM
        if result:
            self.tracking_data.publish(result)

    def _process_tracking(self, frame):  # type: ignore[no-untyped-def]
        """Process a single frame for person tracking."""
        # Detect people in the frame
        bboxes, track_ids, class_ids, confidences, names = self.detector.process_image(frame)

        # Filter to keep only person detections using filter_detections
        (
            filtered_bboxes,
            filtered_track_ids,
            filtered_class_ids,
            filtered_confidences,
            filtered_names,
        ) = filter_detections(
            bboxes,
            track_ids,
            class_ids,
            confidences,
            names,
            class_filter=[0],  # 0 is the class_id for person
            name_filter=["person"],
        )

        # Create visualization
        viz_frame = self.detector.visualize_results(
            frame,
            filtered_bboxes,
            filtered_track_ids,
            filtered_class_ids,
            filtered_confidences,
            filtered_names,
        )

        # Calculate distance and angle for each person
        targets = []
        for i, bbox in enumerate(filtered_bboxes):
            target_data = {
                "target_id": filtered_track_ids[i] if i < len(filtered_track_ids) else -1,
                "bbox": bbox,
                "confidence": filtered_confidences[i] if i < len(filtered_confidences) else None,
            }

            distance, angle = self.distance_estimator.estimate_distance_angle(bbox)
            target_data["distance"] = distance
            target_data["angle"] = angle

            # Add text to visualization
            _x1, y1, x2, _y2 = map(int, bbox)
            dist_text = f"{distance:.2f}m, {np.rad2deg(angle):.1f} deg"

            # Add black background for better visibility
            text_size = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            # Position at top-right corner
            cv2.rectangle(
                viz_frame, (x2 - text_size[0], y1 - text_size[1] - 5), (x2, y1), (0, 0, 0), -1
            )

            # Draw text in white at top-right
            cv2.putText(
                viz_frame,
                dist_text,
                (x2 - text_size[0], y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                2,
            )

            targets.append(target_data)

        # Create the result dictionary
        return {"frame": frame, "viz_frame": viz_frame, "targets": targets}

    @rpc
    def enable_tracking(self) -> bool:
        """Enable person tracking.

        Returns:
            bool: True if tracking was enabled successfully
        """
        self._tracking_enabled = True
        logger.info("Person tracking enabled")
        return True

    @rpc
    def disable_tracking(self) -> bool:
        """Disable person tracking.

        Returns:
            bool: True if tracking was disabled successfully
        """
        self._tracking_enabled = False
        logger.info("Person tracking disabled")
        return True

    @rpc
    def is_tracking_enabled(self) -> bool:
        """Check if tracking is currently enabled.

        Returns:
            bool: True if tracking is enabled
        """
        return self._tracking_enabled

    @rpc
    def get_tracking_data(self) -> dict:  # type: ignore[type-arg]
        """Get the latest tracking data.

        Returns:
            Dictionary containing tracking results
        """
        if self._latest_frame is not None:
            return self._process_tracking(self._latest_frame)  # type: ignore[no-any-return, no-untyped-call]
        return {"frame": None, "viz_frame": None, "targets": []}

    def create_stream(self, video_stream: Observable) -> Observable:  # type: ignore[type-arg]
        """
        Create an Observable stream of person tracking results from a video stream.

        Args:
            video_stream: Observable that emits video frames

        Returns:
            Observable that emits dictionaries containing tracking results and visualizations
        """

        return video_stream.pipe(ops.map(self._process_tracking))
