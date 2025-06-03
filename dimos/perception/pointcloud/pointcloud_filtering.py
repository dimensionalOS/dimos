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

import numpy as np
import cv2
import os
import torch
import open3d as o3d
from typing import Dict, List, Optional, Union

from dimos.perception.segmentation import Sam2DSegmenter
from dimos.types.segmentation import SegmentationType
from dimos.perception.pointcloud.utils import (
    load_camera_matrix_from_yaml,
    create_point_cloud_and_extract_masks,
    o3d_point_cloud_to_numpy,
)
from dimos.perception.pointcloud.cuboid_fit import fit_cuboid


class PointcloudFiltering:
    """
    A production-ready point cloud filtering pipeline for segmented objects.

    This class takes segmentation results and produces clean, filtered point clouds
    for each object with consistent coloring and optional outlier removal.
    """

    def __init__(
        self,
        color_intrinsics: Optional[Union[str, List[float], np.ndarray]] = None,
        depth_intrinsics: Optional[Union[str, List[float], np.ndarray]] = None,
        color_weight: float = 0.3,
        statistical_neighbors: int = 40,
        statistical_std_ratio: float = 1.5,
        radius_filtering_radius: float = 0.015,
        radius_filtering_min_neighbors: int = 100,
        min_points_for_cuboid: int = 10,
        cuboid_method: str = "oriented",
    ):
        """
        Initialize the point cloud filtering pipeline.

        Args:
            color_intrinsics: Camera intrinsics for color image
            depth_intrinsics: Camera intrinsics for depth image
            color_weight: Weight for blending generated color with original (0.0-1.0)
            statistical_neighbors: Number of neighbors for statistical filtering
            statistical_std_ratio: Standard deviation ratio for statistical filtering
            radius_filtering_radius: Search radius for radius filtering (meters)
            radius_filtering_min_neighbors: Min neighbors within radius
            min_points_for_cuboid: Minimum points required for cuboid fitting
            cuboid_method: Method for cuboid fitting ('minimal', 'oriented', 'axis_aligned')

        Raises:
            ValueError: If invalid parameters are provided
        """
        # Validate parameters
        if not 0.0 <= color_weight <= 1.0:
            raise ValueError(f"color_weight must be between 0.0 and 1.0, got {color_weight}")
        if statistical_neighbors < 1:
            raise ValueError(f"statistical_neighbors must be >= 1, got {statistical_neighbors}")
        if statistical_std_ratio <= 0:
            raise ValueError(f"statistical_std_ratio must be > 0, got {statistical_std_ratio}")
        if radius_filtering_radius <= 0:
            raise ValueError(f"radius_filtering_radius must be > 0, got {radius_filtering_radius}")
        if radius_filtering_min_neighbors < 1:
            raise ValueError(
                f"radius_filtering_min_neighbors must be >= 1, got {radius_filtering_min_neighbors}"
            )
        if min_points_for_cuboid < 4:
            raise ValueError(f"min_points_for_cuboid must be >= 4, got {min_points_for_cuboid}")
        if cuboid_method not in ["minimal", "oriented", "axis_aligned"]:
            raise ValueError(
                f"cuboid_method must be 'minimal', 'oriented', or 'axis_aligned', got {cuboid_method}"
            )

        # Store settings
        self.color_weight = color_weight
        self.statistical_neighbors = statistical_neighbors
        self.statistical_std_ratio = statistical_std_ratio
        self.radius_filtering_radius = radius_filtering_radius
        self.radius_filtering_min_neighbors = radius_filtering_min_neighbors
        self.min_points_for_cuboid = min_points_for_cuboid
        self.cuboid_method = cuboid_method

        # Load camera matrices
        try:
            self.color_camera_matrix = load_camera_matrix_from_yaml(color_intrinsics)
            self.depth_camera_matrix = load_camera_matrix_from_yaml(depth_intrinsics)
        except Exception as e:
            raise ValueError(f"Failed to load camera matrices: {e}")

    def generate_color_from_id(self, object_id: int) -> np.ndarray:
        """Generate a consistent color for a given object ID."""
        np.random.seed(object_id)
        color = np.random.randint(0, 255, 3, dtype=np.uint8)
        np.random.seed(None)
        return color

    def _validate_inputs(
        self, color_img: np.ndarray, depth_img: np.ndarray, segmentation_result: SegmentationType
    ):
        """Validate input parameters."""
        if not isinstance(color_img, np.ndarray) or len(color_img.shape) != 3:
            raise ValueError("color_img must be a 3D numpy array")
        if not isinstance(depth_img, np.ndarray) or len(depth_img.shape) != 2:
            raise ValueError("depth_img must be a 2D numpy array")
        if color_img.shape[:2] != depth_img.shape:
            raise ValueError(
                f"Color and depth image dimensions don't match: {color_img.shape[:2]} vs {depth_img.shape}"
            )
        if not isinstance(segmentation_result, SegmentationType):
            raise ValueError("segmentation_result must be a SegmentationType object")
        if self.depth_camera_matrix is None:
            raise ValueError("Depth camera matrix must be provided")

    def _prepare_masks(self, masks: List[np.ndarray], target_shape: tuple) -> List[np.ndarray]:
        """Prepare and validate masks to match target shape."""
        processed_masks = []
        for i, mask in enumerate(masks):
            try:
                # Convert mask to numpy if it's a tensor
                if hasattr(mask, "cpu"):
                    mask = mask.cpu().numpy()

                # Ensure mask is proper boolean array
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
            except Exception as e:
                raise ValueError(f"Failed to process mask {i}: {e}")

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
        """Apply statistical and radius filtering to point cloud."""
        # Statistical filtering
        pcd_filtered, _ = pcd.remove_statistical_outlier(
            nb_neighbors=self.statistical_neighbors, std_ratio=self.statistical_std_ratio
        )
        pcd = pcd_filtered

        # Radius filtering
        pcd_filtered, _ = pcd.remove_radius_outlier(
            nb_points=self.radius_filtering_min_neighbors, radius=self.radius_filtering_radius
        )

        return pcd_filtered

    def process_images(
        self, color_img: np.ndarray, depth_img: np.ndarray, segmentation_result: SegmentationType
    ) -> Dict:
        """
        Process color and depth images with segmentation results to create filtered point clouds.

        Args:
            color_img: RGB image as numpy array (H, W, 3)
            depth_img: Depth image as numpy array (H, W) in meters
            segmentation_result: SegmentationType object containing masks and metadata

        Returns:
            Dictionary containing:
                - objects: List of object dictionaries with point clouds and metadata
                - full_point_cloud: Complete Open3D point cloud for reference
                - processing_stats: Overall processing statistics

        Raises:
            ValueError: If inputs are invalid
            RuntimeError: If processing fails
        """
        # Validate inputs
        self._validate_inputs(color_img, depth_img, segmentation_result)

        try:
            # Extract data from segmentation result
            masks = segmentation_result.masks
            metadata = segmentation_result.metadata
            objects_metadata = metadata.get("objects", [])

            # Prepare masks
            processed_masks = self._prepare_masks(masks, depth_img.shape)

            # Create point clouds efficiently
            full_pcd, masked_pcds = create_point_cloud_and_extract_masks(
                color_img, depth_img, processed_masks, self.depth_camera_matrix, depth_scale=1.0
            )

            # Process each object
            objects = []
            total_points_processed = 0

            for i, (mask, pcd) in enumerate(zip(processed_masks, masked_pcds)):
                # Skip empty point clouds
                if len(np.asarray(pcd.points)) == 0:
                    continue

                # Get object metadata
                obj_meta = objects_metadata[i] if i < len(objects_metadata) else {}
                object_id = obj_meta.get("object_id", i)
                bbox = obj_meta.get("bbox", [0, 0, 0, 0])
                confidence = obj_meta.get("prob", 1.0)
                label = obj_meta.get("label", "")

                # Generate consistent color
                rgb_color = self.generate_color_from_id(object_id)

                # Apply color mask
                pcd = self._apply_color_mask(pcd, rgb_color)

                # Apply filtering
                pcd_filtered = self._apply_filtering(pcd)

                # Fit cuboid if enough points
                cuboid_params = None
                points = np.asarray(pcd_filtered.points)
                if len(points) >= self.min_points_for_cuboid:
                    try:
                        cuboid_params = fit_cuboid(points, method=self.cuboid_method)
                    except Exception as e:
                        # Log warning but don't fail the entire process
                        print(f"Warning: Cuboid fitting failed for object {object_id}: {e}")

                # Create object data
                obj_data = {
                    "object_id": object_id,
                    "mask": mask,
                    "bbox": bbox,
                    "confidence": float(confidence),
                    "label": label,
                    "point_cloud": pcd_filtered,
                    "point_cloud_numpy": o3d_point_cloud_to_numpy(pcd_filtered),
                    "color": rgb_color,
                }

                if cuboid_params is not None:
                    obj_data["cuboid_params"] = cuboid_params

                objects.append(obj_data)
                total_points_processed += len(points)

            # Processing statistics
            processing_stats = {
                "total_objects_processed": len(objects),
                "total_input_objects": len(masks),
                "total_points_processed": total_points_processed,
                "full_point_cloud_size": len(np.asarray(full_pcd.points)),
            }

            return {
                "objects": objects,
                "full_point_cloud": full_pcd,
                "processing_stats": processing_stats,
            }

        except Exception as e:
            raise RuntimeError(f"Failed to process images: {e}")
        finally:
            # Clean up GPU memory
            if torch.cuda.is_available():
                torch.cuda.empty_cache()

    def cleanup(self):
        """Clean up resources."""
        if torch.cuda.is_available():
            torch.cuda.empty_cache()


def create_test_pipeline(data_dir: str) -> tuple:
    """
    Create a test pipeline with default settings.

    Args:
        data_dir: Directory containing camera info files

    Returns:
        Tuple of (filter_pipeline, color_info_path, depth_info_path)
    """
    color_info_path = os.path.join(data_dir, "color_camera_info.yaml")
    depth_info_path = os.path.join(data_dir, "depth_camera_info.yaml")

    filter_pipeline = PointcloudFiltering(
        color_intrinsics=color_info_path,
        depth_intrinsics=depth_info_path,
    )

    return filter_pipeline, color_info_path, depth_info_path


def load_test_images(data_dir: str) -> tuple:
    """
    Load the first available test images from data directory.

    Args:
        data_dir: Directory containing color and depth subdirectories

    Returns:
        Tuple of (color_img, depth_img) or raises FileNotFoundError
    """

    def find_first_image(directory):
        """Find the first image file in the given directory."""
        if not os.path.exists(directory):
            return None

        image_extensions = [".jpg", ".jpeg", ".png", ".bmp"]
        for filename in sorted(os.listdir(directory)):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                return os.path.join(directory, filename)
        return None

    color_dir = os.path.join(data_dir, "color")
    depth_dir = os.path.join(data_dir, "depth")

    color_img_path = find_first_image(color_dir)
    depth_img_path = find_first_image(depth_dir)

    if not color_img_path or not depth_img_path:
        raise FileNotFoundError(f"Could not find color or depth images in {data_dir}")

    # Load color image
    color_img = cv2.imread(color_img_path)
    if color_img is None:
        raise FileNotFoundError(f"Could not load color image from {color_img_path}")
    color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

    # Load depth image
    depth_img = cv2.imread(depth_img_path, cv2.IMREAD_UNCHANGED)
    if depth_img is None:
        raise FileNotFoundError(f"Could not load depth image from {depth_img_path}")

    # Convert depth to meters if needed
    if depth_img.dtype == np.uint16:
        depth_img = depth_img.astype(np.float32) / 1000.0

    return color_img, depth_img


def run_segmentation(color_img: np.ndarray, device: str = "auto") -> SegmentationType:
    """
    Run segmentation on color image.

    Args:
        color_img: RGB color image
        device: Device to use ('auto', 'cuda', or 'cpu')

    Returns:
        SegmentationType object with segmentation results
    """
    if device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"

    segmenter = Sam2DSegmenter(
        model_path="FastSAM-s.pt", device=device, use_tracker=False, use_analyzer=False
    )

    try:
        masks, bboxes, target_ids, probs, names = segmenter.process_image(np.array(color_img))

        # Create metadata
        objects_metadata = []
        for i in range(len(bboxes)):
            obj_data = {
                "object_id": target_ids[i] if i < len(target_ids) else i,
                "bbox": bboxes[i],
                "prob": probs[i] if i < len(probs) else 1.0,
                "label": names[i] if i < len(names) else "",
            }
            objects_metadata.append(obj_data)

        metadata = {"frame": color_img, "objects": objects_metadata}

        numpy_masks = [mask.cpu().numpy() if hasattr(mask, "cpu") else mask for mask in masks]
        return SegmentationType(masks=numpy_masks, metadata=metadata)

    finally:
        segmenter.cleanup()


def visualize_results(results: Dict):
    """
    Visualize point cloud filtering results.

    Args:
        results: Results dictionary from process_images
    """
    all_pcds = []

    for obj in results["objects"]:
        pcd = obj["point_cloud"]
        all_pcds.append(pcd)

        # Add cuboid visualization if available
        if "cuboid_params" in obj:
            cuboid = obj["cuboid_params"]
            if "bounding_box" in cuboid:
                obb = cuboid["bounding_box"]
                obb.color = [1, 0, 0]  # Red bounding boxes
                all_pcds.append(obb)

    # Add coordinate frame
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    all_pcds.append(coordinate_frame)

    # Visualize
    if all_pcds:
        o3d.visualization.draw_geometries(
            all_pcds, window_name="Filtered Point Clouds", width=1280, height=720
        )


def main():
    """Main function to demonstrate the PointcloudFiltering pipeline."""
    try:
        # Setup paths
        script_dir = os.path.dirname(os.path.abspath(__file__))
        dimos_dir = os.path.abspath(os.path.join(script_dir, "../../../"))
        data_dir = os.path.join(dimos_dir, "assets/rgbd_data")

        # Load test data
        print("Loading test images...")
        color_img, depth_img = load_test_images(data_dir)
        print(f"Loaded images: color {color_img.shape}, depth {depth_img.shape}")

        # Run segmentation
        print("Running segmentation...")
        segmentation_result = run_segmentation(color_img)
        print(f"Found {len(segmentation_result.masks)} objects")

        # Create filtering pipeline
        print("Creating filtering pipeline...")
        filter_pipeline, _, _ = create_test_pipeline(data_dir)

        # Process images
        print("Processing point clouds...")
        results = filter_pipeline.process_images(color_img, depth_img, segmentation_result)

        # Print results
        stats = results["processing_stats"]
        print(f"Processing complete:")
        print(
            f"  Objects processed: {stats['total_objects_processed']}/{stats['total_input_objects']}"
        )
        print(f"  Total points: {stats['total_points_processed']}")

        # Print per-object stats
        for i, obj in enumerate(results["objects"]):
            print(
                f"  Object {i + 1} (ID: {obj['object_id']}): {len(np.asarray(obj['point_cloud'].points))} points"
            )

        # Visualize results
        print("Visualizing results...")
        visualize_results(results)

    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
