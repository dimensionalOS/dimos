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

import open3d as o3d
from typing import Any

class PointCloudType:
    def __init__(self, point_cloud: o3d.geometry.PointCloud, metadata: Any = None):
        """
        Initializes a standardized point cloud type.

        Args:
            point_cloud (o3d.geometry.PointCloud): The point cloud data.
            metadata (Any, optional): Additional metadata related to the point cloud.
        """
        self.point_cloud = point_cloud
        self.metadata = metadata 

    def downsample(self, voxel_size: float):
        """Downsample the point cloud using a voxel grid filter."""
        self.point_cloud = self.point_cloud.voxel_down_sample(voxel_size)

    def save_to_file(self, filepath: str):
        """Save the point cloud to a file."""
        o3d.io.write_point_cloud(filepath, self.point_cloud) 