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

"""Rerun visual-override helpers shared across camera-nav blueprints."""

from __future__ import annotations


def costmap_viz(grid):
    """Render occupancy grid as a 3D textured floor overlay in Rerun.

    OccupancyGrid convention: -1=unknown (grey), 0=free (green), 100=lethal (red).
    """
    return grid.to_rerun()


def cloud_points(cloud):
    """Render a PointCloud2 as coloured 3D points in Rerun."""
    import numpy as np
    import rerun as rr
    pts, cols = cloud.as_numpy()
    if len(pts) == 0:
        return rr.Points3D([])
    if cols is not None and len(cols) == len(pts):
        return rr.Points3D(positions=pts, colors=(cols * 255).astype(np.uint8))
    return rr.Points3D(positions=pts)


def pinhole_setup(info):
    """Log camera intrinsics as a static Rerun Pinhole so images display in 3D.

    Logs to both world/color_image and world/depth_image so neither generates
    the 'pinhole ancestor required' warning.  Returns None to suppress the
    default world/camera_info entity.
    """
    import rerun as rr
    K = info.get_K_matrix()
    if info.width == 0 or info.height == 0:
        return None
    pinhole = rr.Pinhole(image_from_camera=K, width=info.width, height=info.height)
    rr.log("world/color_image", pinhole, static=True)
    rr.log("world/depth_image", pinhole, static=True)
    return None
