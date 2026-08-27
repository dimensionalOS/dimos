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

"""Robot-model point-cloud self exclusion and map-clear-mask generation."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from functools import partial
from io import BytesIO

import numpy as np
from pydantic import Field
import trimesh
import yourdfpy  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass(frozen=True)
class _CollisionGeometry:
    link: str
    link_from_geometry: np.ndarray
    mesh: trimesh.Trimesh
    shape: str
    dimensions: tuple[float, ...]
    clear_samples: np.ndarray


class PointCloudSelfFilterConfig(ModuleConfig):
    robot_model: RobotModelConfig
    padding_m: float = Field(default=0.01, ge=0.0)
    # Must match the mapper's voxel_size, or the mask names cells the map does
    # not hold and clears nothing.
    voxel_size: float = Field(default=0.05, gt=0.0)
    planning_frame: str = "world"
    tf_tolerance_s: float = Field(default=0.02, ge=0.0)
    tf_forward_tolerance_s: float = Field(default=0.05, ge=0.0)


class PointCloudSelfFilter(Module):
    """Remove the modeled robot from a cloud and emit its map clear mask.

    A wrist camera sees its own arm. Two things follow, and this module does
    both: the arm's returns must not enter the map, and the volume the arm
    occupies must be erased from it. Ray tracing cannot do the second - the arm
    occludes whatever is behind it, so no ray ever passes through that volume to
    clear it - so the mask says outright which cells are free.
    """

    config: PointCloudSelfFilterConfig  # type: ignore[assignment]

    pointcloud: In[PointCloud2]
    tf: In[TFMessage]
    filtered_pointcloud: Out[PointCloud2]
    voxel_clear_mask: Out[PointCloud2]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._collision_geometry = self._load_collision_geometry()
        if not self._collision_geometry:
            raise ValueError("Robot model contains no collision geometry")
        self._previous_clear_keys: set[tuple[int, int, int]] = set()

    @rpc
    def start(self) -> None:
        # Subscribe to TF before accepting clouds. Filtering is dispatched off
        # the transport thread so TF can keep filling while geometry work runs.
        _ = self.tfbuffer
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    async def handle_pointcloud(self, cloud: PointCloud2) -> None:
        """Filter the latest capture without starving TF transport callbacks."""
        await asyncio.to_thread(self._on_pointcloud, cloud)

    def filter_cloud(self, cloud: PointCloud2) -> tuple[PointCloud2, PointCloud2] | None:
        """Filter one capture and build the matching world-frame clear mask."""
        config = self.filter_config
        points = cloud.points_f32()
        keep = np.ones(len(points), dtype=bool)
        current_clear_keys: set[tuple[int, int, int]] = set()

        for geometry in self._collision_geometry:
            sensor_from_link = self._lookup(cloud.frame_id, geometry.link, cloud.ts)
            world_from_link = self._lookup(config.planning_frame, geometry.link, cloud.ts)
            if sensor_from_link is None or world_from_link is None:
                logger.warning(
                    "Dropping cloud: capture-time TF unavailable for robot link %s", geometry.link
                )
                return None

            if len(points):
                sensor_from_geometry = sensor_from_link.to_matrix() @ geometry.link_from_geometry
                local = _transform_points(points, np.linalg.inv(sensor_from_geometry))
                keep &= ~_points_inside_geometry(local, geometry, config.padding_m)

            world_from_geometry = world_from_link.to_matrix() @ geometry.link_from_geometry
            world_samples = _transform_points(geometry.clear_samples, world_from_geometry)
            keys = np.floor(world_samples / config.voxel_size).astype(np.int32)
            current_clear_keys.update(map(tuple, keys.tolist()))

        # Where the arm was plus where it is: a link that moved between frames
        # leaves a ghost behind it that nothing else will ever clear.
        clear_keys = self._previous_clear_keys | current_clear_keys
        self._previous_clear_keys = current_clear_keys
        clear_points = (
            np.asarray(sorted(clear_keys), dtype=np.float32).reshape((-1, 3)) + 0.5
        ) * config.voxel_size
        clear_mask = PointCloud2.from_numpy(
            clear_points,
            frame_id=config.planning_frame,
            timestamp=cloud.ts,
        )

        intensities = cloud.intensities_f32()
        filtered = PointCloud2.from_numpy(
            points[keep],
            frame_id=cloud.frame_id,
            timestamp=cloud.ts,
            intensities=intensities[keep] if intensities is not None else None,
        )
        for name, values in cloud.pointcloud_tensor.point.items():
            if name not in ("positions", "intensities"):
                filtered.pointcloud_tensor.point[name] = values[keep]
        return filtered, clear_mask

    def _lookup(self, parent_frame: str, child_frame: str, stamp: float) -> Transform | None:
        config = self.filter_config
        return self.tfbuffer.get(
            parent_frame,
            child_frame,
            time_point=stamp,
            time_tolerance=config.tf_tolerance_s,
            forward_tolerance=config.tf_forward_tolerance_s,
        )

    def _on_pointcloud(self, cloud: PointCloud2) -> None:
        result = self.filter_cloud(cloud)
        if result is None:
            return
        filtered, clear_mask = result
        # The mask is independent authoritative free-space evidence. Publishing
        # it first minimizes cleanup latency without making cloud processing
        # depend on cross-topic ordering.
        self.voxel_clear_mask.publish(clear_mask)
        self.filtered_pointcloud.publish(filtered)

    def _load_collision_geometry(self) -> list[_CollisionGeometry]:
        config = self.filter_config.robot_model
        description = prepare_urdf_for_drake(
            config.model.load(),
            convert_meshes=bool(config.auto_convert_meshes),
        )
        mesh_dir = str(description.source_path.parent)
        robot = yourdfpy.URDF.load(
            BytesIO(description.xml.encode()),
            build_scene_graph=False,
            build_collision_scene_graph=False,
            load_meshes=False,
            load_collision_meshes=False,
        )
        resolve = partial(yourdfpy.filename_handler_magic, dir=mesh_dir)
        result: list[_CollisionGeometry] = []
        for link in robot.robot.links:
            for collision in link.collisions:
                shape = _geometry_mesh(collision.geometry, resolve)
                if shape is None:
                    continue
                mesh, shape_name, dimensions = shape
                result.append(
                    _CollisionGeometry(
                        link=link.name,
                        link_from_geometry=(
                            np.eye(4, dtype=np.float64)
                            if collision.origin is None
                            else np.asarray(collision.origin, dtype=np.float64)
                        ),
                        mesh=mesh,
                        shape=shape_name,
                        dimensions=dimensions,
                        clear_samples=self._clear_samples(mesh),
                    )
                )
        return result

    def _clear_samples(self, mesh: trimesh.Trimesh) -> np.ndarray:
        """Grid points covering the geometry, at map resolution.

        A cell whose center is outside the mesh can still be occupied by it, so
        the margin reaches out by half a cell diagonal.
        """
        pitch = self.filter_config.voxel_size
        margin = self.filter_config.padding_m + (np.sqrt(3.0) * pitch / 2.0)
        lower = np.floor((mesh.bounds[0] - margin) / pitch).astype(int)
        upper = np.ceil((mesh.bounds[1] + margin) / pitch).astype(int)
        axes = [
            np.arange(lo, hi + 1, dtype=np.float64) * pitch
            for lo, hi in zip(lower, upper, strict=True)
        ]
        grid = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape((-1, 3))
        signed_distance = trimesh.proximity.signed_distance(  # type: ignore[no-untyped-call]
            mesh, grid
        )
        return np.asarray(grid[signed_distance >= -margin], dtype=np.float64)

    @property
    def filter_config(self) -> PointCloudSelfFilterConfig:
        return self.config  # type: ignore[return-value]


def _geometry_mesh(
    geometry: object,
    resolve: object,
) -> tuple[trimesh.Trimesh, str, tuple[float, ...]] | None:
    if geometry.box is not None:  # type: ignore[attr-defined]
        size = tuple(float(value) for value in geometry.box.size)  # type: ignore[attr-defined]
        return trimesh.creation.box(extents=size), "box", size
    if geometry.sphere is not None:  # type: ignore[attr-defined]
        radius = float(geometry.sphere.radius)  # type: ignore[attr-defined]
        return trimesh.creation.icosphere(radius=radius), "sphere", (radius,)
    if geometry.cylinder is not None:  # type: ignore[attr-defined]
        radius = float(geometry.cylinder.radius)  # type: ignore[attr-defined]
        length = float(geometry.cylinder.length)  # type: ignore[attr-defined]
        return (
            trimesh.creation.cylinder(radius=radius, height=length),
            "cylinder",
            (radius, length),
        )
    if geometry.mesh is None:  # type: ignore[attr-defined]
        return None
    filename = resolve(geometry.mesh.filename)  # type: ignore[operator, attr-defined]
    loaded = trimesh.load_mesh(filename, force="mesh")
    if not isinstance(loaded, trimesh.Trimesh):
        raise ValueError(f"Collision mesh is not a single mesh: {filename}")
    mesh = loaded.copy()
    if geometry.mesh.scale is not None:  # type: ignore[attr-defined]
        mesh.apply_scale(  # type: ignore[no-untyped-call]
            np.asarray(geometry.mesh.scale, dtype=np.float64)  # type: ignore[attr-defined]
        )
    return mesh, "mesh", ()


def _points_inside_geometry(
    points: np.ndarray, geometry: _CollisionGeometry, padding: float
) -> np.ndarray:
    if geometry.shape == "box":
        half_size = np.asarray(geometry.dimensions, dtype=np.float64) / 2.0
        return np.asarray(np.all(np.abs(points) <= half_size + padding, axis=1))
    if geometry.shape == "sphere":
        radius = geometry.dimensions[0] + padding
        return np.asarray(np.einsum("ij,ij->i", points, points) <= radius**2)
    if geometry.shape == "cylinder":
        radius, length = geometry.dimensions
        radial_sq = np.einsum("ij,ij->i", points[:, :2], points[:, :2])
        return np.asarray(
            (radial_sq <= (radius + padding) ** 2)
            & (np.abs(points[:, 2]) <= length / 2.0 + padding)
        )

    # Exact mesh distance is expensive for a full RGB-D cloud. Reject points
    # outside the padded mesh bounds first; robot links occupy only a small
    # fraction of the camera view.
    padded_lower = geometry.mesh.bounds[0] - padding
    padded_upper = geometry.mesh.bounds[1] + padding
    candidates = np.all((points >= padded_lower) & (points <= padded_upper), axis=1)
    inside = np.zeros(len(points), dtype=bool)
    if np.any(candidates):
        signed_distance = trimesh.proximity.signed_distance(  # type: ignore[no-untyped-call]
            geometry.mesh, points[candidates]
        )
        inside[candidates] = signed_distance >= -padding
    return inside


def _transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    if not len(points):
        return np.empty((0, 3), dtype=np.float64)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    return np.asarray(points @ rotation.T + translation, dtype=np.float64)


point_cloud_self_filter = PointCloudSelfFilter.blueprint
