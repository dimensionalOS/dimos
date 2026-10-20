# Copyright 2025-2026 Dimensional Inc.
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
import sys

import numpy as np


def reduce_external(obj):  # type: ignore[no-untyped-def]
    return (reconstruct_pointcloud, (np.asarray(obj.points),))


# Recorded lidar pickles under data/ reference this function by module path.
def reconstruct_pointcloud(points_array):  # type: ignore[no-untyped-def]
    import open3d as o3d  # type: ignore[import-untyped]

    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points_array)
    return pc


def register_picklers() -> None:
    """Teach pickle about open3d point clouds if open3d is loaded.

    Only a process that imported open3d can hold a point cloud, so skipping the
    2 s import when it is absent loses nothing.
    """
    o3d = sys.modules.get("open3d")
    if o3d is not None:
        copyreg.pickle(o3d.geometry.PointCloud, reduce_external)
