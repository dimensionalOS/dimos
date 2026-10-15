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


# Generated detection values
from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    Quaternion,
    Transform,
    TransformStamped,
    Vector3,
)
from dimos_generated.sensor_msgs.msg import CameraInfo, Image
from dimos_generated.std_msgs.msg import Header
from dimos_generated.tf2_msgs.msg import TFMessage
from dimos_generated.vision_msgs.msg import (
    BoundingBox3D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
)
import numpy as np
from reactivex import operators as ops

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.image import image_from_array, image_to_rgb, image_view
from dimos.msgs.time import to_seconds
from dimos.perception.experimental.object_tracker_2d import ObjectTracker2D
from dimos.types.timestamped import TimestampedData, align_timestamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import (
    euler_to_quaternion,
    optical_to_robot_frame,
    yaw_towards_point,
)


def _timed_image(image: Image) -> TimestampedData[Image]:
    return TimestampedData(image, to_seconds(image.header.stamp))


logger = setup_logger()


class ObjectTracker3D(ObjectTracker2D):
    """3D object tracking module extending ObjectTracker2D with depth capabilities."""

    # Additional inputs (2D tracker already has color_image)
    depth: In[Image]
    camera_info: In[CameraInfo]

    # Additional outputs (2D tracker already has detection2darray and tracked_overlay)
    detection3darray: Out[Detection3DArray]
    tf: Out[TFMessage]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        """
        Initialize 3D object tracking module.

        Args:
            **kwargs: Arguments passed to parent ObjectTracker2D
        """
        super().__init__(**kwargs)

        # Additional state for 3D tracking
        self.camera_intrinsics = None
        self._latest_depth_frame: np.ndarray | None = None
        self._latest_camera_info: CameraInfo | None = None

        # Store latest 3D detection
        self._latest_detection3d: Detection3DArray | None = None

    @rpc
    def start(self) -> None:
        super().start()

        # Subscribe to aligned RGB and depth streams
        def on_aligned_frames(frames_tuple) -> None:  # type: ignore[no-untyped-def]
            rgb_msg, depth_msg = (item.value for item in frames_tuple)
            with self._frame_lock:
                self._latest_rgb_frame = image_to_rgb(rgb_msg)

                depth_data = image_view(depth_msg)
                # Convert from millimeters to meters if depth is DEPTH16 format
                if depth_msg.encoding in ("16UC1", "mono16"):
                    depth_data = depth_data.astype(np.float32) / 1000.0
                self._latest_depth_frame = depth_data

        # Create aligned observable for RGB and depth
        aligned_frames = align_timestamped(
            self.color_image.pure_observable().pipe(ops.map(_timed_image)),
            self.depth.pure_observable().pipe(ops.map(_timed_image)),
            buffer_size=2.0,  # 2 second buffer
            match_tolerance=0.5,  # 500ms tolerance
        )
        unsub = aligned_frames.subscribe(on_aligned_frames)
        self.register_disposable(unsub)

        # Subscribe to camera info
        def on_camera_info(camera_info_msg: CameraInfo) -> None:
            self._latest_camera_info = camera_info_msg
            # Extract intrinsics: K is [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.camera_intrinsics = [  # type: ignore[assignment]
                camera_info_msg.k[0],
                camera_info_msg.k[4],
                camera_info_msg.k[2],
                camera_info_msg.k[5],
            ]

        self.camera_info.subscribe(on_camera_info)

        logger.info("ObjectTracker3D module started with aligned frame subscription")

    @rpc
    def stop(self) -> None:
        super().stop()

    def _process_tracking(self) -> None:
        """Override to add 3D detection creation after 2D tracking."""
        import cv2

        # Call parent 2D tracking
        super()._process_tracking()

        # Enhance with 3D if we have depth and a valid 2D detection
        if (
            self._latest_detection2d
            and bool(self._latest_detection2d.detections)
            and self._latest_depth_frame is not None
            and self.camera_intrinsics is not None
        ):
            detection_3d = self._create_detection3d_from_2d(self._latest_detection2d)
            if detection_3d:
                self._latest_detection3d = detection_3d
                self.detection3darray.publish(detection_3d)

                # Update visualization with 3D info
                with self._frame_lock:
                    if self._latest_rgb_frame is not None:
                        frame = self._latest_rgb_frame.copy()

                # Extract 2D bbox for visualization
                det_2d = self._latest_detection2d.detections[0]
                x1 = det_2d.bbox.center.position.x - det_2d.bbox.size_x / 2
                y1 = det_2d.bbox.center.position.y - det_2d.bbox.size_y / 2
                x2 = det_2d.bbox.center.position.x + det_2d.bbox.size_x / 2
                y2 = det_2d.bbox.center.position.y + det_2d.bbox.size_y / 2
                bbox_2d = [[x1, y1, x2, y2]]

                # Use frame directly for visualization
                viz_image = frame.copy()

                # Draw bounding boxes
                for bbox in bbox_2d:
                    x1, y1, x2, y2 = map(int, bbox)
                    cv2.rectangle(viz_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Overlay Re-ID matches
                if self.last_good_matches and self.last_roi_kps and self.last_roi_bbox:
                    viz_image = self._draw_reid_overlay(viz_image)

                viz_msg = image_from_array(
                    viz_image, encoding="rgb8", header=self._latest_detection2d.header
                )
                self.tracked_overlay.publish(viz_msg)

    def _create_detection3d_from_2d(self, detection2d: Detection2DArray) -> Detection3DArray | None:
        """Create 3D detection from 2D detection using depth."""
        if not detection2d.detections:
            return None

        det_2d = detection2d.detections[0]

        # Get bbox center
        center_x = det_2d.bbox.center.position.x
        center_y = det_2d.bbox.center.position.y
        width = det_2d.bbox.size_x
        height = det_2d.bbox.size_y

        # Convert to bbox coordinates
        x1 = int(center_x - width / 2)
        y1 = int(center_y - height / 2)
        x2 = int(center_x + width / 2)
        y2 = int(center_y + height / 2)

        # Get depth value
        depth_value = self._get_depth_from_bbox([x1, y1, x2, y2], self._latest_depth_frame)  # type: ignore[arg-type]

        if depth_value is None or depth_value <= 0:
            return None

        fx, fy, cx, cy = self.camera_intrinsics  # type: ignore[misc]

        # Convert pixel coordinates to 3D in optical frame
        z_optical = depth_value
        x_optical = (center_x - cx) * z_optical / fx  # type: ignore[has-type]
        y_optical = (center_y - cy) * z_optical / fy  # type: ignore[has-type]

        # Create pose in optical frame
        optical_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        optical_pose.position = Point(x=x_optical, y=y_optical, z=z_optical)
        optical_pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)

        # Convert to robot frame
        robot_pose = optical_to_robot_frame(optical_pose)

        # Calculate orientation: object facing towards camera
        yaw = yaw_towards_point(robot_pose.position)
        euler = Vector3(z=yaw, x=0.0, y=0.0)
        robot_pose.orientation = euler_to_quaternion(euler)

        # Estimate object size in meters
        size_x = width * z_optical / fx  # type: ignore[has-type]
        size_y = height * z_optical / fy  # type: ignore[has-type]
        size_z = 0.1  # Default depth size

        # Create Detection3D
        header = Header(stamp=detection2d.header.stamp, frame_id=self.config.frame_id)
        detection_3d = Detection3D(
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            results=[],
            bbox=BoundingBox3D(
                center=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                size=Vector3(x=0.0, y=0.0, z=0.0),
            ),
            id="",
        )
        detection_3d.id = "0"
        detection_3d.header = header

        # Create hypothesis
        hypothesis = ObjectHypothesisWithPose(
            hypothesis=ObjectHypothesis(class_id="", score=0.0),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=0.0, y=0.0, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                covariance=np.zeros(36, dtype=np.float64),
            ),
        )
        hypothesis.hypothesis.class_id = "tracked_object"
        hypothesis.hypothesis.score = 1.0
        detection_3d.results = [hypothesis]

        # Create 3D bounding box
        detection_3d.bbox.center = Pose(
            position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        detection_3d.bbox.center.position = robot_pose.position
        detection_3d.bbox.center.orientation = robot_pose.orientation
        detection_3d.bbox.size = Vector3(x=size_x, y=size_y, z=size_z)

        detection3darray = Detection3DArray(
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""), detections=[]
        )
        detection3darray.header = header
        detection3darray.detections = [detection_3d]

        # Publish TF for tracked object
        tracked_object_tf = TransformStamped(
            header=header,
            child_frame_id="tracked_object",
            transform=Transform(
                translation=Vector3(
                    x=robot_pose.position.x, y=robot_pose.position.y, z=robot_pose.position.z
                ),
                rotation=robot_pose.orientation,
            ),
        )
        self.tf.publish(TFMessage(transforms=[tracked_object_tf]))

        return detection3darray

    def _get_depth_from_bbox(self, bbox: list[int], depth_frame: np.ndarray) -> float | None:
        """
        Calculate depth from bbox using the 25th percentile of closest points.

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

        # Extract depth values from the bbox
        roi_depth = depth_frame[y1:y2, x1:x2]

        # Get valid (finite and positive) depth values
        valid_depths = roi_depth[np.isfinite(roi_depth) & (roi_depth > 0)]

        if len(valid_depths) > 0:
            return float(np.percentile(valid_depths, 25))

        return None

    def _draw_reid_overlay(self, image: np.ndarray) -> np.ndarray:
        """Draw Re-ID feature matches on visualization."""
        import cv2

        viz_image: np.ndarray = image.copy()
        x1, y1, _x2, _y2 = self.last_roi_bbox  # type: ignore[attr-defined]

        # Draw keypoints
        for kp in self.last_roi_kps:  # type: ignore[attr-defined]
            pt = (int(kp.pt[0] + x1), int(kp.pt[1] + y1))
            cv2.circle(viz_image, pt, 3, (0, 255, 0), -1)

        # Draw matches
        for match in self.last_good_matches:  # type: ignore[attr-defined]
            current_kp = self.last_roi_kps[match.trainIdx]  # type: ignore[attr-defined]
            pt_current = (int(current_kp.pt[0] + x1), int(current_kp.pt[1] + y1))
            cv2.circle(viz_image, pt_current, 5, (0, 255, 255), 2)

            intensity = int(255 * (1.0 - min(match.distance / 100.0, 1.0)))
            cv2.circle(viz_image, pt_current, 2, (intensity, intensity, 255), -1)

        # Draw match count
        text = f"REID: {len(self.last_good_matches)}/{len(self.last_roi_kps)}"  # type: ignore[attr-defined]
        cv2.putText(viz_image, text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        return viz_image
