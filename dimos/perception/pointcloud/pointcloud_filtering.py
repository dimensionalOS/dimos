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
import open3d as o3d
import torch

from dimos.perception.pointcloud.cuboid_fit import fit_cuboid
from dimos.perception.pointcloud.utils import (
    create_point_cloud_and_extract_masks,
    load_camera_matrix_from_yaml,
)
from dimos.types.manipulation import ObjectData
from dimos.types.vector import Vector
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.perception.pointcloud.filtering")


class PointcloudFiltering:
    """
    A production-ready point cloud filtering pipeline for segmented objects.

    This class takes segmentation results and produces clean, filtered point clouds
    for each object with consistent coloring and optional outlier removal.
    """

    def __init__(
        self,
        color_intrinsics: str | list[float] | np.ndarray | None = None,
        depth_intrinsics: str | list[float] | np.ndarray | None = None,
        color_weight: float = 0.3,
        enable_statistical_filtering: bool = True,
        statistical_neighbors: int = 20,
        statistical_std_ratio: float = 1.5,
        enable_radius_filtering: bool = True,
        radius_filtering_radius: float = 0.015,
        radius_filtering_min_neighbors: int = 25,
        enable_subsampling: bool = True,
        voxel_size: float = 0.005,
        max_num_objects: int = 10,
        min_points_for_cuboid: int = 10,
        cuboid_method: str = "oriented",
        max_bbox_size_percent: float = 30.0,
    ) -> None:
        """
        Initialize the point cloud filtering pipeline.

        Args:
            color_intrinsics: Camera intrinsics for color image
            depth_intrinsics: Camera intrinsics for depth image
            color_weight: Weight for blending generated color with original (0.0-1.0)
            enable_statistical_filtering: Enable/disable statistical outlier filtering
            statistical_neighbors: Number of neighbors for statistical filtering
            statistical_std_ratio: Standard deviation ratio for statistical filtering
            enable_radius_filtering: Enable/disable radius outlier filtering
            radius_filtering_radius: Search radius for radius filtering (meters)
            radius_filtering_min_neighbors: Min neighbors within radius
            enable_subsampling: Enable/disable point cloud subsampling
            voxel_size: Voxel size for downsampling (meters, when subsampling enabled)
            max_num_objects: Maximum number of objects to process (top N by confidence)
            min_points_for_cuboid: Minimum points required for cuboid fitting
            cuboid_method: Method for cuboid fitting ('minimal', 'oriented', 'axis_aligned')
            max_bbox_size_percent: Maximum percentage of image size for object bboxes (0-100)

        Raises:
            ValueError: If invalid parameters are provided
        """
        # Validate parameters
        if not 0.0 <= color_weight <= 1.0:
            raise ValueError(f"color_weight must be between 0.0 and 1.0, got {color_weight}")
        if not 0.0 <= max_bbox_size_percent <= 100.0:
            raise ValueError(
                f"max_bbox_size_percent must be between 0.0 and 100.0, got {max_bbox_size_percent}"
            )

        # Store settings
        self.color_weight = color_weight
        self.enable_statistical_filtering = enable_statistical_filtering
        self.statistical_neighbors = statistical_neighbors
        self.statistical_std_ratio = statistical_std_ratio
        self.enable_radius_filtering = enable_radius_filtering
        self.radius_filtering_radius = radius_filtering_radius
        self.radius_filtering_min_neighbors = radius_filtering_min_neighbors
        self.enable_subsampling = enable_subsampling
        self.voxel_size = voxel_size
        self.max_num_objects = max_num_objects
        self.min_points_for_cuboid = min_points_for_cuboid
        self.cuboid_method = cuboid_method
        self.max_bbox_size_percent = max_bbox_size_percent

        # Load camera matrices
        self.color_camera_matrix = load_camera_matrix_from_yaml(color_intrinsics)
        self.depth_camera_matrix = load_camera_matrix_from_yaml(depth_intrinsics)

        # Store the full point cloud
        self.full_pcd = None

    def generate_color_from_id(self, object_id: int) -> np.ndarray:
        """Generate consistent RGB color from object ID."""
        # Fix the seed range error
        seed = abs(object_id) % (2**32 - 1)  # Ensure valid range
        np.random.seed(seed)
        return np.random.randint(0, 255, 3)

    def _validate_inputs(
        self, color_img: np.ndarray, depth_img: np.ndarray, objects: list[ObjectData]
    ):
        """Validate input parameters."""
        if color_img.shape[:2] != depth_img.shape:
            raise ValueError("Color and depth image dimensions don't match")

    def _prepare_masks(self, masks: list[np.ndarray], target_shape: tuple) -> list[np.ndarray]:
        """Prepare and validate masks to match target shape."""
        processed_masks = []
        for mask in masks:
            # Convert mask to numpy if it's a tensor
            if hasattr(mask, "cpu"):
                mask = mask.cpu().numpy()

            mask = mask.astype(bool)

            # Handle shape mismatches
            if mask.shape != target_shape:
                if len(mask.shape) > 2:
                    mask = mask[:, :, 0]

                if mask.shape != target_shape:
                    mask = cv2.resize(
                        mask.astype(np.uint8),
                        (target_shape[1], target_shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    ).astype(bool)

            processed_masks.append(mask)

        return processed_masks

    def _apply_color_mask(
        self, pcd: o3d.geometry.PointCloud, rgb_color: np.ndarray
    ) -> o3d.geometry.PointCloud:
        """Apply weighted color mask to point cloud."""
        if len(np.asarray(pcd.colors)) > 0:
            original_colors = np.asarray(pcd.colors)
            generated_color = rgb_color.astype(np.float32) / 255.0
            colored_mask = (
                1.0 - self.color_weight
            ) * original_colors + self.color_weight * generated_color
            colored_mask = np.clip(colored_mask, 0.0, 1.0)
            pcd.colors = o3d.utility.Vector3dVector(colored_mask)
        return pcd

    def _apply_filtering(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Apply optional filtering to point cloud based on enabled flags."""
        current_pcd = pcd

        # Apply statistical filtering if enabled
        if self.enable_statistical_filtering:
            current_pcd, _ = current_pcd.remove_statistical_outlier(
                nb_neighbors=self.statistical_neighbors, std_ratio=self.statistical_std_ratio
            )

        # Apply radius filtering if enabled
        if self.enable_radius_filtering:
            current_pcd, _ = current_pcd.remove_radius_outlier(
                nb_points=self.radius_filtering_min_neighbors, radius=self.radius_filtering_radius
            )

        return current_pcd

    def _apply_subsampling(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Apply subsampling to limit point cloud size using Open3D's voxel downsampling."""
        if self.enable_subsampling and pcd is not None and len(pcd.points) > 0:
            return pcd.voxel_down_sample(self.voxel_size)
        return pcd

    def _extract_masks_from_objects(self, objects: list[ObjectData], image_shape: tuple) -> list:
        """Extract masks from ObjectData, handling None masks."""
        masks = []
        for obj in objects:
            if obj.get("mask") is not None:
                masks.append(obj["mask"])
            elif obj.get("bbox") is not None:
                # CREATE A MASK FROM BBOX if no mask exists
                bbox = obj["bbox"]
                # Create a rectangular mask from bbox
                mask = np.zeros(image_shape[:2], dtype=np.uint8)
                x1, y1, x2, y2 = [int(coord) for coord in bbox]
                # Clamp to image bounds
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(image_shape[1], x2), min(image_shape[0], y2)
                mask[y1:y2, x1:x2] = 255
                masks.append(mask)
            else:
                # Empty mask if neither mask nor bbox
                masks.append(np.zeros(image_shape[:2], dtype=np.uint8))
        return masks

    def get_full_point_cloud(self) -> o3d.geometry.PointCloud:
        """Get the full point cloud."""
        if self.full_pcd is None:
            return o3d.geometry.PointCloud()
        return self._apply_subsampling(self.full_pcd)

    def process_images(
        self, color_img: np.ndarray, depth_img: np.ndarray, objects: list[ObjectData]
    ) -> list[ObjectData]:
        """
        Process color and depth images with object detection results to create filtered point clouds.

        Args:
            color_img: RGB image as numpy array (H, W, 3)
            depth_img: Depth image as numpy array (H, W) in meters
            objects: List of ObjectData from object detection stream

        Returns:
            List of updated ObjectData with pointcloud and 3D information
        """

        # ADD THESE CHECKS FIRST - Before _validate_inputs
        if color_img is None:
            logger.error("Color image is None in process_images!")
            return []

        if depth_img is None:
            logger.error("Depth image is None in process_images!")
            return []

        # Check dimensions match
        if color_img.shape[:2] != depth_img.shape[:2]:
            logger.error(
                f"Color and depth image dimensions don't match: "
                f"Color={color_img.shape}, Depth={depth_img.shape}"
            )
            return []  # Return empty list instead of raising error

        # Check depth is in correct format
        if len(depth_img.shape) != 2:
            logger.error(f"Depth image has wrong dimensions: {depth_img.shape}, expected 2D")
            return []

        # Ensure depth is float32
        if depth_img.dtype != np.float32:
            logger.warning(f"Converting depth from {depth_img.dtype} to float32")
            depth_img = depth_img.astype(np.float32)

        # NOW do the existing validation
        try:
            self._validate_inputs(color_img, depth_img, objects)
        except Exception as e:
            logger.error(f"Validation failed: {e}")
            return []

        if not objects:
            logger.info("No objects to process")
            return []

        # Filter to top N objects by confidence
        if len(objects) > self.max_num_objects:
            # Sort objects by confidence (highest first), handle None confidences
            sorted_objects = sorted(
                objects,
                key=lambda obj: obj.get("confidence", 0.0)
                if obj.get("confidence") is not None
                else 0.0,
                reverse=True,
            )
            objects = sorted_objects[: self.max_num_objects]

        # Filter out objects with bboxes too large
        image_area = color_img.shape[0] * color_img.shape[1]
        max_bbox_area = image_area * (self.max_bbox_size_percent / 100.0)

        filtered_objects = []
        for obj in objects:
            if "bbox" in obj and obj["bbox"] is not None:
                bbox = obj["bbox"]
                # Calculate bbox area (assuming bbox format [x1, y1, x2, y2])
                bbox_area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
                if bbox_area <= max_bbox_area:
                    filtered_objects.append(obj)
            else:
                filtered_objects.append(obj)

        objects = filtered_objects

        # Extract masks from ObjectData - PASS IMAGE SHAPE
        masks = self._extract_masks_from_objects(objects, depth_img.shape)

        # Prepare masks
        processed_masks = self._prepare_masks(masks, depth_img.shape)

        # Create point clouds efficiently
        self.full_pcd, masked_pcds = create_point_cloud_and_extract_masks(
            color_img, depth_img, processed_masks, self.depth_camera_matrix, depth_scale=1.0
        )

        # Process each object and update ObjectData
        updated_objects = []

        for i, (obj, _mask, pcd) in enumerate(
            zip(objects, processed_masks, masked_pcds, strict=False)
        ):
            # Skip empty point clouds
            if pcd is None or len(np.asarray(pcd.points)) == 0:
                continue

            # Create a copy of the object data to avoid modifying the original
            updated_obj = obj.copy()

            # Generate consistent color
            object_id = obj.get("object_id", i)
            rgb_color = self.generate_color_from_id(object_id)

            # Apply color mask
            pcd = self._apply_color_mask(pcd, rgb_color)

            # Apply subsampling to control point cloud size
            pcd = self._apply_subsampling(pcd)

            # Apply filtering (optional based on flags)
            pcd_filtered = self._apply_filtering(pcd)

            # Fit cuboid and extract 3D information
            points = np.asarray(pcd_filtered.points)
            if len(points) >= self.min_points_for_cuboid:
                cuboid_params = fit_cuboid(points, method=self.cuboid_method)
                if cuboid_params is not None:
                    # Update position, rotation, and size from cuboid
                    center = cuboid_params["center"]
                    dimensions = cuboid_params["dimensions"]
                    rotation_matrix = cuboid_params["rotation"]

                    # Convert rotation matrix to euler angles (roll, pitch, yaw)
                    sy = np.sqrt(
                        rotation_matrix[0, 0] * rotation_matrix[0, 0]
                        + rotation_matrix[1, 0] * rotation_matrix[1, 0]
                    )
                    singular = sy < 1e-6

                    if not singular:
                        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
                        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
                        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                    else:
                        roll = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
                        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
                        yaw = 0

                    # Update position, rotation, and size from cuboid
                    updated_obj["position"] = Vector(center[0], center[1], center[2])
                    updated_obj["rotation"] = Vector(roll, pitch, yaw)
                    updated_obj["size"] = {
                        "width": float(dimensions[0]),
                        "height": float(dimensions[1]),
                        "depth": float(dimensions[2]),
                    }

            # Add point cloud data to ObjectData
            updated_obj["point_cloud"] = pcd_filtered
            updated_obj["color"] = rgb_color

            # Extract numpy arrays for grasp generation
            points_array = np.asarray(pcd_filtered.points).astype(np.float32)  # Nx3 XYZ coordinates
            if pcd_filtered.has_colors():
                colors_array = np.asarray(pcd_filtered.colors).astype(
                    np.float32
                )  # Nx3 RGB (0-1 range)
            else:
                # If no colors, create array of zeros
                colors_array = np.zeros((len(points_array), 3), dtype=np.float32)

            updated_obj["point_cloud_numpy"] = points_array
            updated_obj["colors_numpy"] = colors_array

            updated_objects.append(updated_obj)

        return updated_objects

    def cleanup(self) -> None:
        """Clean up resources."""
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
