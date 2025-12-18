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

from collections.abc import Callable
from dataclasses import dataclass
import functools
import time

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]
from reactivex import interval
from reactivex.disposable import Disposable

from dimos.core import DimosCluster, In, LCMTransport, Module, Out, rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleConfig
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.robot.unitree.connection.go2 import Go2ConnectionProtocol
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.metrics import timed


def ensure_tensor_pcd(
    pcd_any: o3d.t.geometry.PointCloud | o3d.geometry.PointCloud,
    device: o3c.Device,
) -> o3d.t.geometry.PointCloud:
    """Convert legacy / cuda.pybind point clouds into o3d.t.geometry.PointCloud on `device`."""

    if isinstance(pcd_any, o3d.t.geometry.PointCloud):
        return pcd_any.to(device)

    assert isinstance(pcd_any, o3d.geometry.PointCloud), (
        "Input must be a legacy PointCloud or a tensor PointCloud"
    )

    # Legacy CPU point cloud -> tensor
    if isinstance(pcd_any, o3d.geometry.PointCloud):
        return o3d.t.geometry.PointCloud.from_legacy(pcd_any, o3c.float32, device)

    pts = np.asarray(pcd_any.points, dtype=np.float32)
    pcd_t = o3d.t.geometry.PointCloud(device=device)
    pcd_t.point["positions"] = o3c.Tensor(pts, o3c.float32, device)
    return pcd_t


def ensure_legacy_pcd(
    pcd_any: o3d.t.geometry.PointCloud | o3d.geometry.PointCloud,
) -> o3d.geometry.PointCloud:
    if isinstance(pcd_any, o3d.geometry.PointCloud):
        return pcd_any

    assert isinstance(pcd_any, o3d.t.geometry.PointCloud), (
        "Input must be a legacy PointCloud or a tensor PointCloud"
    )

    return pcd_any.to_legacy()


@dataclass
class Config(ModuleConfig):
    publish_interval: float = 0
    voxel_size: float = 0.05
    block_count: int = 2_000_000
    device: str = "CUDA:0"


class SparseVoxelGridMapper(Module):
    default_config = Config
    config: Config

    lidar: In[LidarMessage]
    global_map: Out[LidarMessage]
    global_costmap: Out[OccupancyGrid]
    local_costmap: Out[OccupancyGrid]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        dev = (
            o3c.Device(self.config.device)
            if (self.config.device.startswith("CUDA") and o3c.cuda.is_available())
            else o3c.Device("CPU:0")
        )

        print(f"SparseVoxelGridMapper using device: {dev}")

        self.vbg = o3d.t.geometry.VoxelBlockGrid(
            attr_names=("dummy",),
            attr_dtypes=(o3c.uint8,),
            attr_channels=(o3c.SizeVector([1]),),
            voxel_size=self.config.voxel_size,
            block_resolution=1,
            block_count=self.config.block_count,
            device=dev,
        )

        self._dev = dev
        self._hm = self.vbg.hashmap()
        self._key_dtype = self._hm.key_tensor().dtype

    @rpc
    def start(self) -> None:
        super().start()

        lidar_unsub = self.lidar.subscribe(self._on_frame)
        self._disposables.add(Disposable(lidar_unsub))

        # If publish_interval > 0, publish on timer; otherwise publish on each frame
        if self.config.publish_interval > 0:

            def publish(_: int) -> None:
                self.global_map.publish(self.get_global_pointcloud2())

            interval_disposable = interval(self.config.publish_interval).subscribe(publish)
            self._disposables.add(interval_disposable)

    def _on_frame(self, frame: LidarMessage) -> None:
        self.add_frame(frame)
        if self.config.publish_interval <= 0:
            self.global_map.publish(self.get_global_pointcloud2())

    # @timed()
    def add_frame(self, frame: LidarMessage) -> None:
        # we are potentially moving into CUDA here
        pcd = ensure_tensor_pcd(frame.pointcloud, self._dev)

        if pcd.is_empty():
            return

        pts = pcd.point["positions"].to(self._dev, o3c.float32)
        vox = (pts / self.config.voxel_size).floor().to(self._key_dtype)
        keys_Nx3 = vox.contiguous()
        self._hm.activate(keys_Nx3)

    def get_global_pointcloud2(self) -> PointCloud2:
        return PointCloud2(
            # we are potentially moving out of CUDA here
            ensure_legacy_pcd(self.get_global_pointcloud()),
            frame_id="map",
            ts=time.time(),
        )

    def get_global_pointcloud(self) -> o3d.t.geometry.PointCloud:
        voxel_coords, _ = self.vbg.voxel_coordinates_and_flattened_indices()
        pts = voxel_coords + (self.config.voxel_size * 0.5)
        out = o3d.t.geometry.PointCloud(device=self._dev)
        out.point["positions"] = pts
        return out


# @timed()
def splice_cylinder(
    map_pcd: o3d.geometry.PointCloud,
    patch_pcd: o3d.geometry.PointCloud,
    axis: int = 2,
    shrink: float = 0.95,
) -> o3d.geometry.PointCloud:
    center = patch_pcd.get_center()
    patch_pts = np.asarray(patch_pcd.points)

    # Axes perpendicular to cylinder
    axes = [0, 1, 2]
    axes.remove(axis)

    planar_dists = np.linalg.norm(patch_pts[:, axes] - center[axes], axis=1)
    radius = planar_dists.max() * shrink

    axis_min = (patch_pts[:, axis].min() - center[axis]) * shrink + center[axis]
    axis_max = (patch_pts[:, axis].max() - center[axis]) * shrink + center[axis]

    map_pts = np.asarray(map_pcd.points)
    planar_dists_map = np.linalg.norm(map_pts[:, axes] - center[axes], axis=1)

    victims = np.nonzero(
        (planar_dists_map < radius)
        & (map_pts[:, axis] >= axis_min)
        & (map_pts[:, axis] <= axis_max)
    )[0]

    survivors = map_pcd.select_by_index(victims, invert=True)
    return survivors + patch_pcd


mapper = SparseVoxelGridMapper.blueprint
