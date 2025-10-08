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
from typing import List, Tuple, Optional, Any, Union
from dimos.types.manipulation import ObjectData
from dimos.types.vector import Vector
from dimos.utils.logging_config import setup_logger
from dimos_lcm.vision_msgs import Detection3D, Detection2D, BoundingBox2D
from dimos_lcm.sensor_msgs import CameraInfo
from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.msgs.std_msgs import Header
from dimos.msgs.sensor_msgs import Image
import torch
import yaml

logger = setup_logger("dimos.perception.common.utils")


def load_camera_info(yaml_path: str, frame_id: str = "camera_link") -> CameraInfo:
    """
    Load ROS-style camera_info YAML file and convert to CameraInfo LCM message.

    Args:
        yaml_path: Path to camera_info YAML file (ROS format)
        frame_id: Frame ID for the camera (default: "camera_link")

    Returns:
        CameraInfo: LCM CameraInfo message with all calibration data
    """
    with open(yaml_path, "r") as f:
        camera_info_data = yaml.safe_load(f)

    # Extract image dimensions
    width = camera_info_data.get("image_width", 1280)
    height = camera_info_data.get("image_height", 720)

    # Extract camera matrix (K) - already in row-major format
    K = camera_info_data["camera_matrix"]["data"]

    # Extract distortion coefficients
    D = camera_info_data["distortion_coefficients"]["data"]

    # Extract rectification matrix (R) if available, else use identity
    R = camera_info_data.get("rectification_matrix", {}).get("data", [1, 0, 0, 0, 1, 0, 0, 0, 1])

    # Extract projection matrix (P) if available
    P = camera_info_data.get("projection_matrix", {}).get("data", None)

    # If P not provided, construct from K
    if P is None:
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]
        P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]

    # Create header
    header = Header(frame_id)

    # Create and return CameraInfo message
    return CameraInfo(
        D_length=len(D),
        header=header,
        height=height,
        width=width,
        distortion_model=camera_info_data.get("distortion_model", "plumb_bob"),
        D=D,
        K=K,
        R=R,
        P=P,
        binning_x=0,
        binning_y=0,
    )


def load_camera_info_opencv(yaml_path: str) -> Tuple[np.ndarray, np.ndarray]:
    """
    Load ROS-style camera_info YAML file and convert to OpenCV camera matrix and distortion coefficients.

    Args:
        yaml_path: Path to camera_info YAML file (ROS format)

    Returns:
        K: 3x3 camera intrinsic matrix
        dist: 1xN distortion coefficients array (for plumb_bob model)
    """
    with open(yaml_path, "r") as f:
        camera_info = yaml.safe_load(f)

    # Extract camera matrix (K)
    camera_matrix_data = camera_info["camera_matrix"]["data"]
    K = np.array(camera_matrix_data).reshape(3, 3)

    # Extract distortion coefficients
    dist_coeffs_data = camera_info["distortion_coefficients"]["data"]
    dist = np.array(dist_coeffs_data)

    # Ensure dist is 1D array for OpenCV compatibility
    if dist.ndim == 2:
        dist = dist.flatten()

    return K, dist


def rectify_image(image: Image, camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> Image:
    """
    Rectify (undistort) an image using camera calibration parameters.

    Args:
        image: Input Image object to rectify
        camera_matrix: 3x3 camera intrinsic matrix (K)
        dist_coeffs: Distortion coefficients array

    Returns:
        Image: Rectified Image object with same format and metadata
    """
    # Apply undistortion using OpenCV
    rectified_data = cv2.undistort(image.data, camera_matrix, dist_coeffs)

    # Create new Image object with rectified data, preserving all other properties
    return Image(data=rectified_data, format=image.format, frame_id=image.frame_id, ts=image.ts)


def project_3d_points_to_2d(
    points_3d: np.ndarray, camera_intrinsics: Union[List[float], np.ndarray]
) -> np.ndarray:
    """
    Project 3D points to 2D image coordinates using camera intrinsics.

    Args:
        points_3d: Nx3 array of 3D points (X, Y, Z)
        camera_intrinsics: Camera parameters as [fx, fy, cx, cy] list or 3x3 matrix

    Returns:
        Nx2 array of 2D image coordinates (u, v)
    """
    if len(points_3d) == 0:
        return np.zeros((0, 2), dtype=np.int32)

    # Filter out points with zero or negative depth
    valid_mask = points_3d[:, 2] > 0
    if not np.any(valid_mask):
        return np.zeros((0, 2), dtype=np.int32)

    valid_points = points_3d[valid_mask]

    # Extract camera parameters
    if isinstance(camera_intrinsics, list) and len(camera_intrinsics) == 4:
        fx, fy, cx, cy = camera_intrinsics
    else:
        camera_matrix = np.array(camera_intrinsics)
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

    # Project to image coordinates
    u = (valid_points[:, 0] * fx / valid_points[:, 2]) + cx
    v = (valid_points[:, 1] * fy / valid_points[:, 2]) + cy

    # Round to integer pixel coordinates
    points_2d = np.column_stack([u, v]).astype(np.int32)

    return points_2d


def project_2d_points_to_3d(
    points_2d: np.ndarray,
    depth_values: np.ndarray,
    camera_intrinsics: Union[List[float], np.ndarray],
) -> np.ndarray:
    """
    Project 2D image points to 3D coordinates using depth values and camera intrinsics.

    Args:
        points_2d: Nx2 array of 2D image coordinates (u, v)
        depth_values: N-length array of depth values (Z coordinates) for each point
        camera_intrinsics: Camera parameters as [fx, fy, cx, cy] list or 3x3 matrix

    Returns:
        Nx3 array of 3D points (X, Y, Z)
    """
    if len(points_2d) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    # Ensure depth_values is a numpy array
    depth_values = np.asarray(depth_values)

    # Filter out points with zero or negative depth
    valid_mask = depth_values > 0
    if not np.any(valid_mask):
        return np.zeros((0, 3), dtype=np.float32)

    valid_points_2d = points_2d[valid_mask]
    valid_depths = depth_values[valid_mask]

    # Extract camera parameters
    if isinstance(camera_intrinsics, list) and len(camera_intrinsics) == 4:
        fx, fy, cx, cy = camera_intrinsics
    else:
        camera_matrix = np.array(camera_intrinsics)
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

    # Back-project to 3D coordinates
    # X = (u - cx) * Z / fx
    # Y = (v - cy) * Z / fy
    # Z = depth
    X = (valid_points_2d[:, 0] - cx) * valid_depths / fx
    Y = (valid_points_2d[:, 1] - cy) * valid_depths / fy
    Z = valid_depths

    # Stack into 3D points
    points_3d = np.column_stack([X, Y, Z]).astype(np.float32)

    return points_3d


def colorize_depth(
    depth_img: np.ndarray, max_depth: float = 5.0, overlay_stats: bool = True
) -> Optional[np.ndarray]:
    """
    Normalize and colorize depth image using COLORMAP_JET with optional statistics overlay.

    Args:
        depth_img: Depth image (H, W) in meters
        max_depth: Maximum depth value for normalization
        overlay_stats: Whether to overlay depth statistics on the image

    Returns:
        Colorized depth image (H, W, 3) in RGB format, or None if input is None
    """
    if depth_img is None:
        return None

    valid_mask = np.isfinite(depth_img) & (depth_img > 0)
    depth_norm = np.zeros_like(depth_img)
    depth_norm[valid_mask] = np.clip(depth_img[valid_mask] / max_depth, 0, 1)
    depth_colored = cv2.applyColorMap((depth_norm * 255).astype(np.uint8), cv2.COLORMAP_JET)
    depth_rgb = cv2.cvtColor(depth_colored, cv2.COLOR_BGR2RGB)

    # Make the depth image less bright by scaling down the values
    depth_rgb = (depth_rgb * 0.6).astype(np.uint8)

    if overlay_stats and valid_mask.any():
        # Calculate statistics
        valid_depths = depth_img[valid_mask]
        min_depth = np.min(valid_depths)
        max_depth_actual = np.max(valid_depths)

        # Get center depth
        h, w = depth_img.shape
        center_y, center_x = h // 2, w // 2
        # Sample a small region around center for robustness
        center_region = depth_img[
            max(0, center_y - 2) : min(h, center_y + 3), max(0, center_x - 2) : min(w, center_x + 3)
        ]
        center_mask = np.isfinite(center_region) & (center_region > 0)
        if center_mask.any():
            center_depth = np.median(center_region[center_mask])
        else:
            center_depth = depth_img[center_y, center_x] if valid_mask[center_y, center_x] else 0.0

        # Prepare text overlays
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 1
        line_type = cv2.LINE_AA

        # Text properties
        text_color = (255, 255, 255)  # White
        bg_color = (0, 0, 0)  # Black background
        padding = 5

        # Min depth (top-left)
        min_text = f"Min: {min_depth:.2f}m"
        (text_w, text_h), _ = cv2.getTextSize(min_text, font, font_scale, thickness)
        cv2.rectangle(
            depth_rgb,
            (padding, padding),
            (padding + text_w + 4, padding + text_h + 6),
            bg_color,
            -1,
        )
        cv2.putText(
            depth_rgb,
            min_text,
            (padding + 2, padding + text_h + 2),
            font,
            font_scale,
            text_color,
            thickness,
            line_type,
        )

        # Max depth (top-right)
        max_text = f"Max: {max_depth_actual:.2f}m"
        (text_w, text_h), _ = cv2.getTextSize(max_text, font, font_scale, thickness)
        cv2.rectangle(
            depth_rgb,
            (w - padding - text_w - 4, padding),
            (w - padding, padding + text_h + 6),
            bg_color,
            -1,
        )
        cv2.putText(
            depth_rgb,
            max_text,
            (w - padding - text_w - 2, padding + text_h + 2),
            font,
            font_scale,
            text_color,
            thickness,
            line_type,
        )

        # Center depth (center)
        if center_depth > 0:
            center_text = f"{center_depth:.2f}m"
            (text_w, text_h), _ = cv2.getTextSize(center_text, font, font_scale, thickness)
            center_text_x = center_x - text_w // 2
            center_text_y = center_y + text_h // 2

            # Draw crosshair
            cross_size = 10
            cross_color = (255, 255, 255)
            cv2.line(
                depth_rgb,
                (center_x - cross_size, center_y),
                (center_x + cross_size, center_y),
                cross_color,
                1,
            )
            cv2.line(
                depth_rgb,
                (center_x, center_y - cross_size),
                (center_x, center_y + cross_size),
                cross_color,
                1,
            )

            # Draw center depth text with background
            cv2.rectangle(
                depth_rgb,
                (center_text_x - 2, center_text_y - text_h - 2),
                (center_text_x + text_w + 2, center_text_y + 2),
                bg_color,
                -1,
            )
            cv2.putText(
                depth_rgb,
                center_text,
                (center_text_x, center_text_y),
                font,
                font_scale,
                text_color,
                thickness,
                line_type,
            )

    return depth_rgb


def draw_bounding_box(
    image: np.ndarray,
    bbox: List[float],
    color: Tuple[int, int, int] = (0, 255, 0),
    thickness: int = 2,
    label: Optional[str] = None,
    confidence: Optional[float] = None,
    object_id: Optional[int] = None,
    font_scale: float = 0.6,
) -> np.ndarray:
    """
    Draw a bounding box with optional label on an image.

    Args:
        image: Image to draw on (H, W, 3)
        bbox: Bounding box [x1, y1, x2, y2]
        color: RGB color tuple for the box
        thickness: Line thickness for the box
        label: Optional class label
        confidence: Optional confidence score
        object_id: Optional object ID
        font_scale: Font scale for text

    Returns:
        Image with bounding box drawn
    """
    x1, y1, x2, y2 = map(int, bbox)

    # Draw bounding box
    cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)

    # Create label text
    text_parts = []
    if label is not None:
        text_parts.append(str(label))
    if object_id is not None:
        text_parts.append(f"ID: {object_id}")
    if confidence is not None:
        text_parts.append(f"({confidence:.2f})")

    if text_parts:
        text = ", ".join(text_parts)

        # Draw text background
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)[0]
        cv2.rectangle(
            image,
            (x1, y1 - text_size[1] - 5),
            (x1 + text_size[0], y1),
            (0, 0, 0),
            -1,
        )

        # Draw text
        cv2.putText(
            image,
            text,
            (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            1,
        )

    return image


def draw_segmentation_mask(
    image: np.ndarray,
    mask: np.ndarray,
    color: Tuple[int, int, int] = (0, 200, 200),
    alpha: float = 0.5,
    draw_contours: bool = True,
    contour_thickness: int = 2,
) -> np.ndarray:
    """
    Draw segmentation mask overlay on an image.

    Args:
        image: Image to draw on (H, W, 3)
        mask: Segmentation mask (H, W) - boolean or uint8
        color: RGB color for the mask
        alpha: Transparency factor (0.0 = transparent, 1.0 = opaque)
        draw_contours: Whether to draw mask contours
        contour_thickness: Thickness of contour lines

    Returns:
        Image with mask overlay drawn
    """
    if mask is None:
        return image

    try:
        # Ensure mask is uint8
        mask = mask.astype(np.uint8)

        # Create colored mask overlay
        colored_mask = np.zeros_like(image)
        colored_mask[mask > 0] = color

        # Apply the mask with transparency
        mask_area = mask > 0
        image[mask_area] = cv2.addWeighted(
            image[mask_area], 1 - alpha, colored_mask[mask_area], alpha, 0
        )

        # Draw mask contours if requested
        if draw_contours:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(image, contours, -1, color, contour_thickness)

    except Exception as e:
        logger.warning(f"Error drawing segmentation mask: {e}")

    return image


def draw_object_detection_visualization(
    image: np.ndarray,
    objects: List[ObjectData],
    draw_masks: bool = False,
    bbox_color: Tuple[int, int, int] = (0, 255, 0),
    mask_color: Tuple[int, int, int] = (0, 200, 200),
    font_scale: float = 0.6,
) -> np.ndarray:
    """
    Create object detection visualization with bounding boxes and optional masks.

    Args:
        image: Base image to draw on (H, W, 3)
        objects: List of ObjectData with detection information
        draw_masks: Whether to draw segmentation masks
        bbox_color: Default color for bounding boxes
        mask_color: Default color for segmentation masks
        font_scale: Font scale for text labels

    Returns:
        Image with detection visualization
    """
    viz_image = image.copy()

    for obj in objects:
        try:
            # Draw segmentation mask first (if enabled and available)
            if draw_masks and "segmentation_mask" in obj and obj["segmentation_mask"] is not None:
                viz_image = draw_segmentation_mask(
                    viz_image, obj["segmentation_mask"], color=mask_color, alpha=0.5
                )

            # Draw bounding box
            if "bbox" in obj and obj["bbox"] is not None:
                # Use object's color if available, otherwise default
                color = bbox_color
                if "color" in obj and obj["color"] is not None:
                    obj_color = obj["color"]
                    if isinstance(obj_color, np.ndarray):
                        color = tuple(int(c) for c in obj_color)
                    elif isinstance(obj_color, (list, tuple)):
                        color = tuple(int(c) for c in obj_color[:3])

                viz_image = draw_bounding_box(
                    viz_image,
                    obj["bbox"],
                    color=color,
                    label=obj.get("label"),
                    confidence=obj.get("confidence"),
                    object_id=obj.get("object_id"),
                    font_scale=font_scale,
                )

        except Exception as e:
            logger.warning(f"Error drawing object visualization: {e}")

    return viz_image


def detection_results_to_object_data(
    bboxes: List[List[float]],
    track_ids: List[int],
    class_ids: List[int],
    confidences: List[float],
    names: List[str],
    masks: Optional[List[np.ndarray]] = None,
    source: str = "detection",
) -> List[ObjectData]:
    """
    Convert detection/segmentation results to ObjectData format.

    Args:
        bboxes: List of bounding boxes [x1, y1, x2, y2]
        track_ids: List of tracking IDs
        class_ids: List of class indices
        confidences: List of detection confidences
        names: List of class names
        masks: Optional list of segmentation masks
        source: Source type ("detection" or "segmentation")

    Returns:
        List of ObjectData dictionaries
    """
    objects = []

    for i in range(len(bboxes)):
        # Calculate basic properties from bbox
        bbox = bboxes[i]
        width = bbox[2] - bbox[0]
        height = bbox[3] - bbox[1]
        center_x = bbox[0] + width / 2
        center_y = bbox[1] + height / 2

        # Create ObjectData
        object_data: ObjectData = {
            "object_id": track_ids[i] if i < len(track_ids) else i,
            "bbox": bbox,
            "depth": -1.0,  # Will be populated by depth estimation or point cloud processing
            "confidence": confidences[i] if i < len(confidences) else 1.0,
            "class_id": class_ids[i] if i < len(class_ids) else 0,
            "label": names[i] if i < len(names) else f"{source}_object",
            "movement_tolerance": 1.0,  # Default to freely movable
            "segmentation_mask": masks[i].cpu().numpy()
            if masks and i < len(masks) and isinstance(masks[i], torch.Tensor)
            else masks[i]
            if masks and i < len(masks)
            else None,
            # Initialize 3D properties (will be populated by point cloud processing)
            "position": Vector(0, 0, 0),
            "rotation": Vector(0, 0, 0),
            "size": {
                "width": 0.0,
                "height": 0.0,
                "depth": 0.0,
            },
        }
        objects.append(object_data)

    return objects


def combine_object_data(
    list1: List[ObjectData], list2: List[ObjectData], overlap_threshold: float = 0.8
) -> List[ObjectData]:
    """
    Combine two ObjectData lists, removing duplicates based on segmentation mask overlap.
    """
    combined = list1.copy()
    used_ids = set(obj.get("object_id", 0) for obj in list1)
    next_id = max(used_ids) + 1 if used_ids else 1

    for obj2 in list2:
        obj_copy = obj2.copy()

        # Handle duplicate object_id
        if obj_copy.get("object_id", 0) in used_ids:
            obj_copy["object_id"] = next_id
            next_id += 1
        used_ids.add(obj_copy["object_id"])

        # Check mask overlap
        mask2 = obj2.get("segmentation_mask")
        if mask2 is None or np.sum(mask2 > 0) == 0:
            combined.append(obj_copy)
            continue

        mask2_area = np.sum(mask2 > 0)
        is_duplicate = False

        for obj1 in list1:
            mask1 = obj1.get("segmentation_mask")
            if mask1 is None:
                continue

            intersection = np.sum((mask1 > 0) & (mask2 > 0))
            if intersection / mask2_area >= overlap_threshold:
                is_duplicate = True
                break

        if not is_duplicate:
            combined.append(obj_copy)

    return combined


def point_in_bbox(point: Tuple[int, int], bbox: List[float]) -> bool:
    """
    Check if a point is inside a bounding box.

    Args:
        point: (x, y) coordinates
        bbox: Bounding box [x1, y1, x2, y2]

    Returns:
        True if point is inside bbox
    """
    x, y = point
    x1, y1, x2, y2 = bbox
    return x1 <= x <= x2 and y1 <= y <= y2


def bbox2d_to_corners(bbox_2d: BoundingBox2D) -> Tuple[float, float, float, float]:
    """
    Convert BoundingBox2D from center format to corner format.

    Args:
        bbox_2d: BoundingBox2D with center and size

    Returns:
        Tuple of (x1, y1, x2, y2) corner coordinates
    """
    center_x = bbox_2d.center.position.x
    center_y = bbox_2d.center.position.y
    half_width = bbox_2d.size_x / 2.0
    half_height = bbox_2d.size_y / 2.0

    x1 = center_x - half_width
    y1 = center_y - half_height
    x2 = center_x + half_width
    y2 = center_y + half_height

    return x1, y1, x2, y2


def find_clicked_detection(
    click_pos: Tuple[int, int], detections_2d: List[Detection2D], detections_3d: List[Detection3D]
) -> Optional[Detection3D]:
    """
    Find which detection was clicked based on 2D bounding boxes.

    Args:
        click_pos: (x, y) click position
        detections_2d: List of Detection2D objects
        detections_3d: List of Detection3D objects (must be 1:1 correspondence)

    Returns:
        Corresponding Detection3D object if found, None otherwise
    """
    click_x, click_y = click_pos

    for i, det_2d in enumerate(detections_2d):
        if det_2d.bbox and i < len(detections_3d):
            x1, y1, x2, y2 = bbox2d_to_corners(det_2d.bbox)

            if x1 <= click_x <= x2 and y1 <= click_y <= y2:
                return detections_3d[i]

    return None


def extract_pose_from_detection3d(detection3d: Detection3D):
    """Extract PoseStamped from Detection3D message.

    Args:
        detection3d: Detection3D message

    Returns:
        Pose or None if no valid detection
    """
    if not detection3d or not detection3d.bbox or not detection3d.bbox.center:
        return None

    # Extract position
    pos = detection3d.bbox.center.position
    position = Vector3(pos.x, pos.y, pos.z)

    # Extract orientation
    orient = detection3d.bbox.center.orientation
    orientation = Quaternion(orient.x, orient.y, orient.z, orient.w)

    pose = Pose(position=position, orientation=orientation)
    return pose
