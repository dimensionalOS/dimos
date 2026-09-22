# Copyright 2026 Dimensional Inc.
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

"""A perspective depth render of the cloud from a pose."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray
from PIL import Image as PILImage

from dimos.experimental.agent_encode.pointcloud.image.base import Image, draw_items
from dimos.experimental.agent_encode.pointcloud.image.lib.canvas import Drawable
from dimos.experimental.agent_encode.pointcloud.image.lib.colour import (
    colour_scale,
    depth_range,
    depth_rgb,
)
from dimos.experimental.agent_encode.pointcloud.image.lib.files import (
    artifact,
    file_stem,
    output_dir,
)
from dimos.experimental.agent_encode.pointcloud.image.lib.splat import (
    DepthRaster,
    depth_raster,
    project,
)
from dimos.experimental.agent_encode.pointcloud.queries.base import Query
from dimos.experimental.agent_encode.pointcloud.queries.lib.points import as_cloud, finite_points

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True, eq=False, repr=False)
class CameraImage(Image):
    """A perspective depth image: near is red through yellow and green to blue far away
    (``scale`` reads depths off it); black is no return."""

    pose: tuple[float, float, float, float, float]
    """x, y, z, yaw_deg, pitch_deg of the camera."""
    fov_deg: float
    max_depth: float | None
    raster: DepthRaster
    points: NDArray[np.float32]
    """The finite returns of ``cloud``, indexed by ``raster.point_ids``."""
    cloud: PointCloud2
    """The cloud the image was rendered from."""

    def pixel(self, point: tuple[float, ...]) -> tuple[float, float] | None:
        """Where world point (x, y, z) lands; None behind the camera or beyond max_depth."""
        u, v = project(
            np.array([point], dtype=np.float64), self.pose, self.fov_deg, self.size, self.max_depth
        )[0]
        return None if np.isnan(u) else (round(float(u), 2), round(float(v), 2))

    def world(self, uv: tuple[int, int]) -> tuple[float, float, float] | None:
        """The return drawn at pixel ``uv``; None where no return reached it (black, or
        a gap filled from a neighbouring pixel) or outside the image."""
        u, v = uv
        if not (0 <= u < self.size[0] and 0 <= v < self.size[1]):
            return None
        if self.raster.provenance[v, u] not in (1, 2):
            return None
        x, y, z = self.points[self.raster.point_ids[v, u]].tolist()
        return (x, y, z)

    def returns_under(self, pixels: NDArray[np.int64]) -> PointCloud2:
        """The returns drawn at ``pixels``; pixels filled from a neighbour add nothing."""
        drawn = np.isin(self.raster.provenance[pixels[:, 1], pixels[:, 0]], (1, 2))
        ids = self.raster.point_ids[pixels[drawn, 1], pixels[drawn, 0]]
        return as_cloud(self.points[np.unique(ids)], self.cloud)


@dataclass(frozen=True)
class CameraView(Query[CameraImage]):
    """A pinhole-camera depth render of the cloud from ``pose``, the way the scene would
    look from there. Each return is drawn as wide as the gap to its nearest neighbour, so
    surfaces close at the cloud's own resolution; the nearest return wins."""

    pose: tuple[float, float, float, float, float]
    """x, y, z, yaw_deg, pitch_deg. Yaw 0 looks along +x, positive yaw turns toward +y;
    positive pitch looks up."""
    fov_deg: float = 90.0
    """Horizontal field of view."""
    size: tuple[int, int] = (768, 480)
    """Width, height in pixels; large enough for a VLM to read detail."""
    max_depth: float | None = None
    """No cap: every return in front of the camera is drawn."""
    point_size_m: float | None = None
    """One width for every return; None draws each as wide as the gap to its nearest
    neighbour."""
    draw: tuple[Drawable | PointCloud2, ...] = ()
    """Shapes, results and clouds drawn over the image in distinct colours."""
    out_dir: Path | None = None
    """Where the file goes; the run's log directory by default."""

    def __post_init__(self) -> None:
        if (
            not isinstance(self.pose, tuple)
            or len(self.pose) != 5
            or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in self.pose)
        ):
            raise ValueError(
                f"pose must be five finite numbers (x, y, z, yaw_deg, pitch_deg), not {self.pose!r}"
            )

    def run(self, cloud: PointCloud2) -> CameraImage:
        points = finite_points(cloud)
        raster = depth_raster(
            points, self.pose, self.fov_deg, self.size, self.max_depth, self.point_size_m
        )
        near, far = depth_range(raster.depth, self.max_depth)
        picture = PILImage.fromarray(depth_rgb(raster.depth, near, far))

        def project_many(world: NDArray[np.float64]) -> NDArray[np.float64]:
            return project(world, self.pose, self.fov_deg, self.size, self.max_depth)

        z = points[:, 2]
        z_extent = (float(z.min()), float(z.max())) if len(z) else (0.0, 0.0)
        drawn = draw_items(picture, self.draw, project_many, z_extent)
        path = output_dir(self.out_dir) / f"{file_stem(self, cloud)}_camera.png"
        with artifact(path) as staging:
            picture.save(staging)
        return CameraImage(
            path,
            self.size,
            colour_scale(max(near, 0.05), far, log=True, reverse=True),
            drawn,
            self.pose,
            self.fov_deg,
            self.max_depth,
            raster,
            points,
            cloud,
        )
