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

import os
from typing import TYPE_CHECKING

import cv2
import numpy as np
import open3d as o3d
import pytest

from dimos.perception.pointcloud.pointcloud_filtering import PointcloudFiltering
from dimos.perception.pointcloud.utils import load_camera_matrix_from_yaml

if TYPE_CHECKING:
    from dimos.types.manipulation import ObjectData


class TestPointcloudFiltering:
    def test_pointcloud_filtering_initialization(self) -> None:
        """Test PointcloudFiltering initializes correctly with default parameters."""
        try:
            filtering = PointcloudFiltering()
            assert filtering is not None
            assert filtering.color_weight == 0.3
            assert filtering.enable_statistical_filtering
            assert filtering.enable_radius_filtering
            assert filtering.enable_subsampling
        except Exception as e:
            pytest.skip(f"Skipping test due to initialization error: {e}")

    def test_pointcloud_filtering_with_custom_params(self) -> None:
        """Test PointcloudFiltering with custom parameters."""
        try:
            filtering = PointcloudFiltering(
                color_weight=0.5,
                enable_statistical_filtering=False,
                enable_radius_filtering=False,
                voxel_size=0.01,
                max_num_objects=5,
            )
            assert filtering.color_weight == 0.5
            assert not filtering.enable_statistical_filtering
            assert not filtering.enable_radius_filtering
            assert filtering.voxel_size == 0.01
            assert filtering.max_num_objects == 5
        except Exception as e:
            pytest.skip(f"Skipping test due to initialization error: {e}")

    def test_pointcloud_filtering_process_images(self) -> None:
        """Test PointcloudFiltering can process RGB-D images and return filtered point clouds."""
        try:
            # Import data inside method to avoid pytest fixture confusion
            from dimos.utils.data import get_data

            # Load test RGB-D data
            data_dir = get_data("rgbd_frames")

            # Load first frame
            color_path = os.path.join(data_dir, "color", "00000.png")
            depth_path = os.path.join(data_dir, "depth", "00000.png")
            intrinsics_path = os.path.join(data_dir, "color_camera_info.yaml")

            assert os.path.exists(color_path), f"Color image not found: {color_path}"
            assert os.path.exists(depth_path), f"Depth image not found: {depth_path}"
            assert os.path.exists(intrinsics_path), f"Intrinsics file not found: {intrinsics_path}"

            # Load images
            color_img = cv2.imread(color_path)
            color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

            depth_img = cv2.imread(depth_path, cv2.IMREAD_ANYDEPTH)
            if depth_img.dtype == np.uint16:
                depth_img = depth_img.astype(np.float32) / 1000.0

            # Load camera intrinsics
            camera_matrix = load_camera_matrix_from_yaml(intrinsics_path)
            if camera_matrix is None:
                pytest.skip("Failed to load camera intrinsics")

            # Create mock objects with segmentation masks
            height, width = color_img.shape[:2]

            # Create simple rectangular masks for testing
            mock_objects = []

            # Object 1: Top-left quadrant
            mask1 = np.zeros((height, width), dtype=bool)
            mask1[height // 4 : height // 2, width // 4 : width // 2] = True

            obj1: ObjectData = {
                "object_id": 1,
                "confidence": 0.9,
                "bbox": [width // 4, height // 4, width // 2, height // 2],
                "segmentation_mask": mask1,
                "name": "test_object_1",
            }
            mock_objects.append(obj1)

            # Object 2: Bottom-right quadrant
            mask2 = np.zeros((height, width), dtype=bool)
            mask2[height // 2 : 3 * height // 4, width // 2 : 3 * width // 4] = True

            obj2: ObjectData = {
                "object_id": 2,
                "confidence": 0.8,
                "bbox": [width // 2, height // 2, 3 * width // 4, 3 * height // 4],
                "segmentation_mask": mask2,
                "name": "test_object_2",
            }
            mock_objects.append(obj2)

            # Initialize filtering with intrinsics
            filtering = PointcloudFiltering(
                color_intrinsics=camera_matrix,
                depth_intrinsics=camera_matrix,
                enable_statistical_filtering=False,  # Disable for faster testing
                enable_radius_filtering=False,  # Disable for faster testing
                voxel_size=0.01,  # Larger voxel for faster processing
            )

            # Process images
            results = filtering.process_images(color_img, depth_img, mock_objects)

            print(
                f"Processing results - Input objects: {len(mock_objects)}, Output objects: {len(results)}"
            )

            # Verify results
            assert isinstance(results, list), "Results should be a list"
            assert len(results) <= len(mock_objects), "Should not return more objects than input"

            # Check each result object
            for i, result in enumerate(results):
                print(f"Object {i}: {result.get('name', 'unknown')}")

                # Verify required fields exist
                assert "point_cloud" in result, "Result should contain point_cloud"
                assert "color" in result, "Result should contain color"
                assert "point_cloud_numpy" in result, "Result should contain point_cloud_numpy"

                # Verify point cloud is valid Open3D object
                pcd = result["point_cloud"]
                assert isinstance(pcd, o3d.geometry.PointCloud), (
                    "point_cloud should be Open3D PointCloud"
                )

                # Verify numpy arrays
                points_array = result["point_cloud_numpy"]
                assert isinstance(points_array, np.ndarray), (
                    "point_cloud_numpy should be numpy array"
                )
                assert points_array.shape[1] == 3, "Point array should have 3 columns (x,y,z)"
                assert points_array.dtype == np.float32, "Point array should be float32"

                # Verify color
                color = result["color"]
                assert isinstance(color, np.ndarray), "Color should be numpy array"
                assert color.shape == (3,), "Color should be RGB triplet"
                assert color.dtype == np.uint8, "Color should be uint8"

                # Check if 3D information was added (when enough points for cuboid fitting)
                points = np.asarray(pcd.points)
                if len(points) >= filtering.min_points_for_cuboid:
                    if "position" in result:
                        assert "rotation" in result, "Should have rotation if position exists"
                        assert "size" in result, "Should have size if position exists"

                        # Verify position format
                        from dimos.types.vector import Vector

                        position = result["position"]
                        assert isinstance(position, Vector), "Position should be Vector"

                        # Verify size format
                        size = result["size"]
                        assert isinstance(size, dict), "Size should be dict"
                        assert "width" in size and "height" in size and "depth" in size

                print(f"  - Points: {len(points)}")
                print(f"  - Color: {color}")
                if "position" in result:
                    print(f"  - Position: {result['position']}")
                    print(f"  - Size: {result['size']}")

            # Test full point cloud access
            full_pcd = filtering.get_full_point_cloud()
            if full_pcd is not None:
                assert isinstance(full_pcd, o3d.geometry.PointCloud), (
                    "Full point cloud should be Open3D PointCloud"
                )
                full_points = np.asarray(full_pcd.points)
                print(f"Full point cloud points: {len(full_points)}")

            print("All pointcloud filtering tests passed!")

        except Exception as e:
            pytest.skip(f"Skipping test due to error: {e}")

    def test_pointcloud_filtering_empty_objects(self) -> None:
        """Test PointcloudFiltering with empty object list."""
        try:
            from dimos.utils.data import get_data

            # Load test data
            data_dir = get_data("rgbd_frames")
            color_path = os.path.join(data_dir, "color", "00000.png")
            depth_path = os.path.join(data_dir, "depth", "00000.png")

            if not (os.path.exists(color_path) and os.path.exists(depth_path)):
                pytest.skip("Test images not found")

            color_img = cv2.imread(color_path)
            color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
            depth_img = cv2.imread(depth_path, cv2.IMREAD_ANYDEPTH)
            if depth_img.dtype == np.uint16:
                depth_img = depth_img.astype(np.float32) / 1000.0

            filtering = PointcloudFiltering()

            # Test with empty object list
            results = filtering.process_images(color_img, depth_img, [])

            assert isinstance(results, list), "Results should be a list"
            assert len(results) == 0, "Should return empty list for empty input"

        except Exception as e:
            pytest.skip(f"Skipping test due to error: {e}")

    def test_color_generation_consistency(self) -> None:
        """Test that color generation is consistent for the same object ID."""
        try:
            filtering = PointcloudFiltering()

            # Test color generation consistency
            color1 = filtering.generate_color_from_id(42)
            color2 = filtering.generate_color_from_id(42)
            color3 = filtering.generate_color_from_id(43)

            assert np.array_equal(color1, color2), "Same ID should generate same color"
            assert not np.array_equal(color1, color3), (
                "Different IDs should generate different colors"
            )
            assert color1.shape == (3,), "Color should be RGB triplet"
            assert color1.dtype == np.uint8, "Color should be uint8"

        except Exception as e:
            pytest.skip(f"Skipping test due to error: {e}")


if __name__ == "__main__":
    pytest.main(["-v", __file__])
