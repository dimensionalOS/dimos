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

"""
Real-time 3D object detection processor that extracts object poses from RGB-D data.
"""

import cv2
from dimos_lcm.vision_msgs import (  # type: ignore[import-untyped]
    BoundingBox2D,
    BoundingBox3D,
    Detection2D,
    Detection3D,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
import numpy as np

from dimos.manipulation.visual_servoing.utils import (
    estimate_object_depth,
    transform_pose,
    visualize_detections_3d,
)
from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.msgs.std_msgs import Header
from dimos.msgs.vision_msgs import Detection2DArray, Detection3DArray
from dimos.perception.common.utils import bbox2d_to_corners
from dimos.perception.detection2d.utils import calculate_object_size_from_bbox
from dimos.perception.pointcloud.utils import extract_centroids_from_masks
from dimos.perception.segmentation.sam_2d_seg import Sam2DSegmenter
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Detection3DProcessor:
    """
    Real-time 3D detection processor optimized for speed.

    Uses Sam (FastSAM) for segmentation and mask generation, then extracts
    3D centroids from depth data.
    """

    def __init__(
        self,
        camera_intrinsics: list[float],  # [fx, fy, cx, cy]
        min_confidence: float = 0.6,
        min_points: int = 30,
        max_depth: float = 1.0,
        max_object_size: float = 0.15,
    ) -> None:
        """
        Initialize the real-time 3D detection processor.

        Args:
            camera_intrinsics: [fx, fy, cx, cy] camera parameters
            min_confidence: Minimum detection confidence threshold
            min_points: Minimum 3D points required for valid detection
            max_depth: Maximum valid depth in meters
        """
        self.camera_intrinsics = camera_intrinsics
        self.min_points = min_points
        self.max_depth = max_depth
        self.max_object_size = max_object_size

        # Initialize Sam segmenter with tracking enabled but analysis disabled
        self.detector = Sam2DSegmenter(
            use_tracker=False,
            use_analyzer=False,
            use_filtering=True,
        )

        self.min_confidence = min_confidence

        logger.info(
            f"Initialized Detection3DProcessor with Sam segmenter, confidence={min_confidence}, "
            f"min_points={min_points}, max_depth={max_depth}m, max_object_size={max_object_size}m"
        )

    def process_frame(
        self,
        rgb_image: np.ndarray,  # type: ignore[type-arg]
        depth_image: np.ndarray,  # type: ignore[type-arg]
        transform: np.ndarray | None = None,  # type: ignore[type-arg]
    ) -> tuple[Detection3DArray, Detection2DArray]:
        """
        Process a single RGB-D frame to extract 3D object detections.

        Args:
            rgb_image: RGB image (H, W, 3)
            depth_image: Depth image (H, W) in meters
            transform: Optional 4x4 transformation matrix to transform objects from camera frame to desired frame

        Returns:
            Tuple of (Detection3DArray, Detection2DArray) with 3D and 2D information
        """

        # Convert RGB to BGR for Sam (OpenCV format)
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)

        # Run Sam segmentation with tracking
        masks, bboxes, track_ids, probs, names = self.detector.process_image(bgr_image)  # type: ignore[no-untyped-call]

        if not masks or len(masks) == 0:
            return Detection3DArray(
                detections_length=0, header=Header(), detections=[]
            ), Detection2DArray(detections_length=0, header=Header(), detections=[])

        # Convert CUDA tensors to numpy arrays if needed
        numpy_masks = []
        for mask in masks:
            if hasattr(mask, "cpu"):  # PyTorch tensor
                numpy_masks.append(mask.cpu().numpy())
            else:  # Already numpy array
                numpy_masks.append(mask)

        # Extract 3D centroids from masks
        poses = extract_centroids_from_masks(
            rgb_image=rgb_image,
            depth_image=depth_image,
            masks=numpy_masks,
            camera_intrinsics=self.camera_intrinsics,
        )

        detections_3d = []
        detections_2d = []
        pose_dict = {p["mask_idx"]: p for p in poses if p["centroid"][2] < self.max_depth}

        for i, (bbox, name, prob, track_id) in enumerate(
            zip(bboxes, names, probs, track_ids, strict=False)
        ):
            if i not in pose_dict:
                continue

            pose = pose_dict[i]
            obj_cam_pos = pose["centroid"]

            if obj_cam_pos[2] > self.max_depth:
                continue

            # Calculate object size from bbox and depth
            width_m, height_m = calculate_object_size_from_bbox(
                bbox, obj_cam_pos[2], self.camera_intrinsics
            )

            # Calculate depth dimension using segmentation mask
            depth_m = estimate_object_depth(
                depth_image, numpy_masks[i] if i < len(numpy_masks) else None, bbox
            )

            size_x = max(width_m, 0.01)  # Minimum 1cm width
            size_y = max(height_m, 0.01)  # Minimum 1cm height
            size_z = max(depth_m, 0.01)  # Minimum 1cm depth

            if min(size_x, size_y, size_z) > self.max_object_size:
                continue

            # Transform to desired frame if transform matrix is provided
            if transform is not None:
                # Get orientation as euler angles, default to no rotation if not available
                obj_cam_orientation = pose.get(
                    "rotation", np.array([0.0, 0.0, 0.0])
                )  # Default to no rotation
                transformed_pose = transform_pose(
                    obj_cam_pos, obj_cam_orientation, transform, to_robot=True
                )
                center_pose = transformed_pose
            else:
                # If no transform, use camera coordinates
                center_pose = Pose(
                    position=Vector3(obj_cam_pos[0], obj_cam_pos[1], obj_cam_pos[2]),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),  # Default orientation
                )

            # Create Detection3D object
            detection = Detection3D(
                results_length=1,
                header=Header(),  # Empty header
                results=[
                    ObjectHypothesisWithPose(
                        hypothesis=ObjectHypothesis(class_id=name, score=float(prob))
                    )
                ],
                bbox=BoundingBox3D(center=center_pose, size=Vector3(size_x, size_y, size_z)),
                id=str(track_id),
            )

            detections_3d.append(detection)

            # Create corresponding Detection2D
            x1, y1, x2, y2 = bbox
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            width = x2 - x1
            height = y2 - y1

            detection_2d = Detection2D(
                results_length=1,
                header=Header(),
                results=[
                    ObjectHypothesisWithPose(
                        hypothesis=ObjectHypothesis(class_id=name, score=float(prob))
                    )
                ],
                bbox=BoundingBox2D(
                    center=Pose2D(position=Point2D(center_x, center_y), theta=0.0),
                    size_x=float(width),
                    size_y=float(height),
                ),
                id=str(track_id),
            )
            detections_2d.append(detection_2d)

        # Create and return both arrays
        return (
            Detection3DArray(
                detections_length=len(detections_3d), header=Header(), detections=detections_3d
            ),
            Detection2DArray(
                detections_length=len(detections_2d), header=Header(), detections=detections_2d
            ),
        )

    def visualize_detections(
        self,
        rgb_image: np.ndarray,  # type: ignore[type-arg]
        detections_3d: list[Detection3D],
        detections_2d: list[Detection2D],
        show_coordinates: bool = True,
    ) -> np.ndarray:  # type: ignore[type-arg]
        """
        Visualize detections with 3D position overlay next to bounding boxes.

        Args:
            rgb_image: Original RGB image
            detections_3d: List of Detection3D objects
            detections_2d: List of Detection2D objects (must be 1:1 correspondence)
            show_coordinates: Whether to show 3D coordinates

        Returns:
            Visualization image
        """
        # Extract 2D bboxes from Detection2D objects

        bboxes_2d = []
        for det_2d in detections_2d:
            if det_2d.bbox:
                x1, y1, x2, y2 = bbox2d_to_corners(det_2d.bbox)
                bboxes_2d.append([x1, y1, x2, y2])

        return visualize_detections_3d(rgb_image, detections_3d, show_coordinates, bboxes_2d)

    def get_closest_detection(
        self, detections: list[Detection3D], class_filter: str | None = None
    ) -> Detection3D | None:
        """
        Get the closest detection with valid 3D data.

        Args:
            detections: List of Detection3D objects
            class_filter: Optional class name to filter by

        Returns:
            Closest Detection3D or None
        """
        valid_detections = []
        for d in detections:
            # Check if has valid bbox center position
            if d.bbox and d.bbox.center and d.bbox.center.position:
                # Check class filter if specified
                if class_filter is None or (
                    d.results_length > 0 and d.results[0].hypothesis.class_id == class_filter
                ):
                    valid_detections.append(d)

        if not valid_detections:
            return None

        # Sort by depth (Z coordinate)
        def get_z_coord(d):  # type: ignore[no-untyped-def]
            return abs(d.bbox.center.position.z)

        return min(valid_detections, key=get_z_coord)

    def cleanup(self) -> None:
        """Clean up resources."""
        if hasattr(self.detector, "cleanup"):
            self.detector.cleanup()
        logger.info("Detection3DProcessor cleaned up")
