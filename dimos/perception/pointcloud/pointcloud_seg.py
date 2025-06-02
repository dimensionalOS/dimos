import numpy as np
import cv2
import yaml
import os
import sys
from PIL import Image, ImageDraw
from dimos.perception.segmentation import Sam2DSegmenter
from dimos.types.segmentation import SegmentationType
from dimos.perception.pointcloud.utils import (
    load_camera_matrix_from_yaml,
    create_masked_point_cloud,
    o3d_point_cloud_to_numpy,
    create_o3d_point_cloud_from_rgbd,
)
from dimos.perception.pointcloud.cuboid_fit import fit_cuboid, visualize_fit
import torch
import open3d as o3d


class PointcloudFiltering:
    def __init__(
        self,
        color_intrinsics=None,
        depth_intrinsics=None,
        enable_statistical_filtering=True,
        enable_cuboid_fitting=True,
        color_weight=0.3,
        statistical_neighbors=20,
        statistical_std_ratio=2.0,
    ):
        """
        Initialize processor to filter point clouds from segmented objects.

        Args:
            color_intrinsics: Path to YAML file or list with color camera intrinsics [fx, fy, cx, cy]
            depth_intrinsics: Path to YAML file or list with depth camera intrinsics [fx, fy, cx, cy]
            enable_statistical_filtering: Whether to apply statistical outlier filtering
            enable_cuboid_fitting: Whether to fit 3D cuboids to objects
            color_weight: Weight for blending generated color with original color (0.0 = original, 1.0 = generated)
            statistical_neighbors: Number of neighbors for statistical filtering
            statistical_std_ratio: Standard deviation ratio for statistical filtering
        """
        # Store settings
        self.enable_statistical_filtering = enable_statistical_filtering
        self.enable_cuboid_fitting = enable_cuboid_fitting
        self.color_weight = color_weight
        self.statistical_neighbors = statistical_neighbors
        self.statistical_std_ratio = statistical_std_ratio

        # Load camera matrices
        self.color_camera_matrix = load_camera_matrix_from_yaml(color_intrinsics)
        self.depth_camera_matrix = load_camera_matrix_from_yaml(depth_intrinsics)

    def generate_color_from_id(self, object_id):
        """Generate a consistent color for a given object ID."""
        np.random.seed(object_id)
        color = np.random.randint(0, 255, 3)
        np.random.seed(None)
        return color

    def process_images(self, color_img, depth_img, segmentation_result):
        """
        Process color and depth images with segmentation results to create filtered point clouds.

        Args:
            color_img: RGB image as numpy array (H, W, 3)
            depth_img: Depth image as numpy array (H, W) in meters
            segmentation_result: SegmentationType object containing masks and metadata

        Returns:
            dict: Dictionary containing:
                - objects: List of dicts for each object with:
                    - object_id: Object tracking ID
                    - mask: Segmentation mask (H, W, bool)
                    - bbox: Bounding box [x1, y1, x2, y2]
                    - confidence: Detection confidence
                    - label: Object label/name
                    - point_cloud: Open3D point cloud object (filtered and colored)
                    - point_cloud_numpy: Nx6 array of XYZRGB points (for compatibility)
                    - color: RGB color for visualization
                    - cuboid_params: Cuboid parameters (if enabled)
                    - filtering_stats: Filtering statistics (if filtering enabled)
        """
        if self.depth_camera_matrix is None:
            raise ValueError("Depth camera matrix must be provided to process images")

        # Extract masks and metadata from segmentation result
        masks = segmentation_result.masks
        metadata = segmentation_result.metadata
        objects_metadata = metadata.get("objects", [])

        # Process each object
        objects = []
        for i, mask in enumerate(masks):
            # Get object metadata if available
            obj_meta = objects_metadata[i] if i < len(objects_metadata) else {}
            object_id = obj_meta.get("object_id", i)
            bbox = obj_meta.get("bbox", [0, 0, 0, 0])
            confidence = obj_meta.get("prob", 1.0)
            label = obj_meta.get("label", "")

            # Convert mask to numpy if it's a tensor
            if hasattr(mask, "cpu"):
                mask = mask.cpu().numpy()

            # Ensure mask is proper boolean array with correct dimensions
            mask = mask.astype(bool)

            # Ensure mask has the same shape as the depth image
            if mask.shape != depth_img.shape[:2]:
                if len(mask.shape) > 2:
                    mask = mask[:, :, 0] if mask.shape[2] > 0 else mask[:, :, 0]

                if mask.shape != depth_img.shape[:2]:
                    mask = cv2.resize(
                        mask.astype(np.uint8),
                        (depth_img.shape[1], depth_img.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    ).astype(bool)

            try:
                # Create point cloud using Open3D
                pcd = create_masked_point_cloud(
                    color_img, depth_img, mask, self.depth_camera_matrix, depth_scale=1.0
                )

                # Skip if no points
                if len(np.asarray(pcd.points)) == 0:
                    continue

                # Generate color for visualization
                rgb_color = self.generate_color_from_id(object_id)

                # Apply weighted colored mask to the point cloud
                if len(np.asarray(pcd.colors)) > 0:
                    original_colors = np.asarray(pcd.colors)
                    generated_color = np.array(rgb_color) / 255.0
                    colored_mask = (
                        1.0 - self.color_weight
                    ) * original_colors + self.color_weight * generated_color
                    colored_mask = np.clip(colored_mask, 0.0, 1.0)
                    pcd.colors = o3d.utility.Vector3dVector(colored_mask)

                # Apply statistical outlier filtering if enabled
                filtering_stats = None
                if self.enable_statistical_filtering:
                    num_points_before = len(np.asarray(pcd.points))
                    pcd_filtered, outlier_indices = pcd.remove_statistical_outlier(
                        nb_neighbors=self.statistical_neighbors,
                        std_ratio=self.statistical_std_ratio,
                    )
                    num_points_after = len(np.asarray(pcd_filtered.points))
                    num_outliers_removed = num_points_before - num_points_after

                    pcd = pcd_filtered

                    filtering_stats = {
                        "points_before": num_points_before,
                        "points_after": num_points_after,
                        "outliers_removed": num_outliers_removed,
                        "outlier_percentage": 100.0 * num_outliers_removed / num_points_before
                        if num_points_before > 0
                        else 0,
                    }

                # Create object data
                obj_data = {
                    "object_id": object_id,
                    "mask": mask,
                    "bbox": bbox,
                    "confidence": float(confidence),
                    "label": label,
                    "point_cloud": pcd,
                    "point_cloud_numpy": o3d_point_cloud_to_numpy(pcd),
                    "color": rgb_color,
                }

                # Add optional data if available
                if filtering_stats is not None:
                    obj_data["filtering_stats"] = filtering_stats

                # Fit 3D cuboid if enabled
                if self.enable_cuboid_fitting:
                    points = np.asarray(pcd.points)
                    cuboid_params = fit_cuboid(points)
                    if cuboid_params is not None:
                        obj_data["cuboid_params"] = cuboid_params

                objects.append(obj_data)

            except Exception as e:
                continue

        # Clean up GPU memory if using CUDA
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        return {
            "objects": objects,
        }

    def cleanup(self):
        """Clean up resources."""
        if torch.cuda.is_available():
            torch.cuda.empty_cache()


def main():
    """
    Main function to test the PointcloudFiltering class with data from rgbd_data folder.
    """

    def find_first_image(directory):
        """Find the first image file in the given directory."""
        image_extensions = [".jpg", ".jpeg", ".png", ".bmp"]
        for filename in sorted(os.listdir(directory)):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                return os.path.join(directory, filename)
        return None

    # Define paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    dimos_dir = os.path.abspath(os.path.join(script_dir, "../../../"))
    data_dir = os.path.join(dimos_dir, "assets/rgbd_data")

    color_info_path = os.path.join(data_dir, "color_camera_info.yaml")
    depth_info_path = os.path.join(data_dir, "depth_camera_info.yaml")

    color_dir = os.path.join(data_dir, "color")
    depth_dir = os.path.join(data_dir, "depth")

    # Find first color and depth images
    color_img_path = find_first_image(color_dir)
    depth_img_path = find_first_image(depth_dir)

    if not color_img_path or not depth_img_path:
        print(f"Error: Could not find color or depth images in {data_dir}")
        return

    # Load images
    color_img = cv2.imread(color_img_path)
    if color_img is None:
        print(f"Error: Could not load color image from {color_img_path}")
        return

    color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

    depth_img = cv2.imread(depth_img_path, cv2.IMREAD_UNCHANGED)
    if depth_img is None:
        print(f"Error: Could not load depth image from {depth_img_path}")
        return

    # Convert depth to meters if needed
    if depth_img.dtype == np.uint16:
        depth_img = depth_img.astype(np.float32) / 1000.0

    # Run segmentation
    segmenter = Sam2DSegmenter(
        model_path="FastSAM-s.pt",
        device="cuda" if torch.cuda.is_available() else "cpu",
        use_tracker=False,
        use_analyzer=True,
    )

    masks, bboxes, target_ids, probs, names = segmenter.process_image(color_img)
    segmenter.run_analysis(color_img, bboxes, target_ids)
    names = segmenter.get_object_names(target_ids, names)

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
    segmentation_result = SegmentationType(masks=numpy_masks, metadata=metadata)

    # Initialize filtering pipeline
    filter_pipeline = PointcloudFiltering(
        color_intrinsics=color_info_path,
        depth_intrinsics=depth_info_path,
        enable_statistical_filtering=True,
        enable_cuboid_fitting=True,
        color_weight=0.3,
        statistical_neighbors=20,
        statistical_std_ratio=2.0,
    )

    # Process images through filtering pipeline
    try:
        results = filter_pipeline.process_images(color_img, depth_img, segmentation_result)

        # Visualize filtered point clouds
        all_pcds = []
        for i, obj in enumerate(results["objects"]):
            pcd = obj["point_cloud"]

            # Add cuboid visualization if available
            if "cuboid_params" in obj and obj["cuboid_params"] is not None:
                cuboid = obj["cuboid_params"]
                center = cuboid["center"]
                dimensions = cuboid["dimensions"]
                rotation = cuboid["rotation"]

                obb = o3d.geometry.OrientedBoundingBox(center=center, R=rotation, extent=dimensions)
                obb.color = [1, 0, 0]
                all_pcds.append(obb)

                coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                    size=min(dimensions) * 0.5, origin=center
                )
                all_pcds.append(coord_frame)

            all_pcds.append(pcd)

        # Add coordinate frame at origin
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        all_pcds.append(coordinate_frame)

        # Show filtered point clouds
        if all_pcds:
            o3d.visualization.draw_geometries(
                all_pcds,
                window_name="Filtered Point Clouds",
                width=1280,
                height=720,
                left=50,
                top=50,
            )

    except Exception as e:
        print(f"Error during processing: {str(e)}")
        import traceback

        traceback.print_exc()

    # Clean up resources
    segmenter.cleanup()
    filter_pipeline.cleanup()


if __name__ == "__main__":
    main()
