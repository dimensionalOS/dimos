import os
import cv2
import numpy as np
import open3d as o3d
from pathlib import Path
from PIL import Image
import logging

from dimos.models.segmentation.segment_utils import apply_mask_to_image
from dimos.models.pointcloud.pointcloud_utils import (
    create_point_cloud_from_rgbd,
    canonicalize_point_cloud
)
from dimos.types.pointcloud import PointCloudType

# Setup logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class PointCloudProcessor:
    def __init__(self, output_dir, intrinsic_parameters=None):
        """
        Initializes the PointCloudProcessor.

        Args:
            output_dir (str): The directory where point clouds will be saved.
            intrinsic_parameters (dict, optional): Camera intrinsic parameters.
                Defaults to None, in which case default parameters are used.
        """
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        self.logger = logger

        # Default intrinsic parameters
        self.default_intrinsic_parameters = {
            'width': 640,
            'height': 480,
            'fx': 960.0,
            'fy': 960.0,
            'cx': 320.0,
            'cy': 240.0,
        }
        self.intrinsic_parameters = intrinsic_parameters if intrinsic_parameters else self.default_intrinsic_parameters

    def process_frame(self, image, depth_map, masks):
        """
        Process a single frame to generate point clouds.

        Args:
            image (PIL.Image.Image or np.ndarray): The RGB image.
            depth_map (PIL.Image.Image or np.ndarray): The depth map corresponding to the image.
            masks (list of np.ndarray): A list of binary masks for segmentation.

        Returns:
            list of PointCloudType: A list of point clouds for each mask.
            bool: A flag indicating if the point clouds were canonicalized.
        """
        try:
            self.logger.info("STARTING POINT CLOUD PROCESSING ---------------------------------------")

            # Convert images to OpenCV format if they are PIL Images
            if isinstance(image, Image.Image):
                original_image_cv = cv2.cvtColor(np.array(image.convert('RGB')), cv2.COLOR_RGB2BGR)
            else:
                original_image_cv = image

            if isinstance(depth_map, Image.Image):
                depth_image_cv = cv2.cvtColor(np.array(depth_map.convert('RGB')), cv2.COLOR_RGB2BGR)
            else:
                depth_image_cv = depth_map

            width, height = original_image_cv.shape[1], original_image_cv.shape[0]
            intrinsic_parameters = self.intrinsic_parameters.copy()
            intrinsic_parameters.update({
                'width': width,
                'height': height,
                'cx': width / 2,
                'cy': height / 2,      
            })

            point_clouds = []
            point_cloud_data = []

            # Create original point cloud
            original_pcd = create_point_cloud_from_rgbd(original_image_cv, depth_image_cv, intrinsic_parameters)
            pcd, canonicalized, transformation = canonicalize_point_cloud(original_pcd, canonicalize_threshold=0.3)

            for idx, mask in enumerate(masks):
                mask_binary = mask > 0

                masked_rgb = apply_mask_to_image(original_image_cv, mask_binary)
                masked_depth = apply_mask_to_image(depth_image_cv, mask_binary)

                pcd = create_point_cloud_from_rgbd(masked_rgb, masked_depth, intrinsic_parameters)
                # Remove outliers
                cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
                inlier_cloud = pcd.select_by_index(ind)
                if canonicalized:
                    inlier_cloud.transform(transformation)

                point_clouds.append(PointCloudType(point_cloud=inlier_cloud, metadata={"mask_index": idx}))
                # Save point cloud to file
                pointcloud_filename = f"pointcloud_{idx}.pcd"
                pointcloud_filepath = os.path.join(self.output_dir, pointcloud_filename)
                o3d.io.write_point_cloud(pointcloud_filepath, inlier_cloud)
                point_cloud_data.append(pointcloud_filepath)
                self.logger.info(f"Saved point cloud {pointcloud_filepath}")

            self.logger.info("DONE POINT CLOUD PROCESSING ---------------------------------------")
            return point_clouds, canonicalized
        except Exception as e:
            self.logger.error(f"Error processing frame: {e}")
            return [], False
