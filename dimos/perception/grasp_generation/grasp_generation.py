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
ContactGraspNet-based grasp generation for manipulation pipeline.
"""

import os
import sys
import time
import logging
import numpy as np
import torch
import open3d as o3d
from typing import Dict, List, Optional, Tuple, Any, Union

from dimos.types.manipulation import ObjectData
from dimos.utils.logging_config import setup_logger

from dimos.models.manipulation.contact_graspnet_pytorch.contact_graspnet_pytorch.contact_grasp_estimator import (
    GraspEstimator,
)
from dimos.models.manipulation.contact_graspnet_pytorch.contact_graspnet_pytorch.checkpoints import (
    CheckpointIO,
)
from dimos.models.manipulation.contact_graspnet_pytorch.contact_graspnet_pytorch import config_utils

logger = setup_logger("dimos.perception.grasp_generation")


class ContactGraspNetGenerator:
    """
    ContactGraspNet-based grasp generator for manipulation pipeline.

    Takes ObjectData with point clouds from pointcloud filtering and generates
    6-DoF grasp poses using ContactGraspNet.
    """

    def __init__(
        self,
        checkpoint_dir: str = "dimos/models/manipulation/contact_graspnet_pytorch/checkpoints",
        local_regions: bool = True,
        filter_grasps: bool = True,
        forward_passes: int = 1,
        z_range: List[float] = [0.2, 1.8],
        min_points_for_grasp: int = 100,
    ):
        """
        Initialize ContactGraspNet grasp generator.

        Args:
            checkpoint_dir: Path to ContactGraspNet checkpoint directory
            device: Device to use ('cuda', 'cpu', or None for auto)
            local_regions: Crop 3D local regions around object segments for prediction
            filter_grasps: Filter grasp contacts to lie within object segments
            forward_passes: Number of forward passes for prediction
            z_range: Z distance range [min, max] in meters to filter point clouds
            min_points_for_grasp: Minimum points required for grasp generation
            max_objects: Maximum number of objects to process for grasps
        """
        self.checkpoint_dir = checkpoint_dir
        self.local_regions = local_regions
        self.filter_grasps = filter_grasps
        self.forward_passes = forward_passes
        self.z_range = z_range
        self.min_points_for_grasp = min_points_for_grasp

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        logger.info(f"Initializing ContactGraspNet on device: {self.device}")

        # Load configuration
        self._load_config()

        # Initialize grasp estimator
        self._initialize_model()

        logger.info("ContactGraspNet grasp generator initialized successfully")

    def _load_config(self):
        """Load ContactGraspNet configuration."""
        try:
            self.global_config = config_utils.load_config(
                self.checkpoint_dir, batch_size=self.forward_passes
            )
            logger.info(f"Loaded config from {self.checkpoint_dir}")
        except Exception as e:
            logger.error(f"Failed to load config: {e}")
            raise

    def _initialize_model(self):
        """Initialize the ContactGraspNet model."""
        try:
            self.grasp_estimator = GraspEstimator(self.global_config)
            logger.info("ContactGraspNet model initialized")
        except Exception as e:
            logger.error(f"Failed to initialize model: {e}")
            raise

    def generate_grasps_from_objects(
        self, objects: List[ObjectData], full_pcd: o3d.geometry.PointCloud
    ) -> Tuple[Dict, Dict, Dict, Dict]:
        """
        Generate grasps from ObjectData objects.

        Args:
            objects: List of ObjectData with point clouds from pointcloud filtering
            full_pcd: Open3D point cloud of full scene

        Returns:
            Tuple of (pred_grasps_cam, scores, contact_pts, gripper_openings) as dicts
            mapping object_id to corresponding arrays, as specified in line 180 of contact_grasp_estimator.py
        """
        try:
            start_time = time.time()

            # Convert ObjectData to point cloud format
            full_pc = np.asarray(full_pcd.points)
            pc_segments = {}

            # Build pc_segments dictionary as expected by ContactGraspNet
            for i, obj in enumerate(objects):
                object_id = obj.get("object_id", i)
                pc_segments[object_id] = obj["point_cloud_numpy"]

            # Generate grasps using ContactGraspNet
            pred_grasps_cam, scores, contact_pts, gripper_openings = (
                self.grasp_estimator.predict_scene_grasps(
                    pc_full=full_pc,
                    pc_segments=pc_segments,
                    local_regions=self.local_regions,
                    filter_grasps=self.filter_grasps,
                    forward_passes=self.forward_passes,
                )
            )

            processing_time = time.time() - start_time

            # Log results
            total_grasps = (
                sum(len(grasps) for grasps in pred_grasps_cam.values()) if pred_grasps_cam else 0
            )
            logger.info(
                f"Generated {total_grasps} grasps across {len(pred_grasps_cam)} objects in {processing_time:.2f}s"
            )

            return pred_grasps_cam, scores, contact_pts, gripper_openings

        except Exception as e:
            logger.error(f"Grasp generation failed: {e}")
            return {}, {}, {}, {}

    def cleanup(self):
        """Clean up resources."""
        if hasattr(self, "grasp_estimator"):
            del self.grasp_estimator
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        logger.info("ContactGraspNet grasp generator cleaned up")
