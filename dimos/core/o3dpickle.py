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

import copyreg

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]


def reduce_external(obj):  # type: ignore[no-untyped-def]
    # Convert Vector3dVector to numpy array for pickling
    points_array = np.asarray(obj.points)
    return (reconstruct_pointcloud, (points_array,))


def reconstruct_pointcloud(points_array):  # type: ignore[no-untyped-def]
    # Create new PointCloud and assign the points
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points_array)
    return pc


def register_picklers() -> None:
    # Register for the actual PointCloud class that gets instantiated
    # We need to create a dummy PointCloud to get its actual class
    _dummy_pc = o3d.geometry.PointCloud()
    copyreg.pickle(_dummy_pc.__class__, reduce_external)
