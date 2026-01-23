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

from dataclasses import dataclass
import time

import numpy as np
import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]
from reactivex import interval, operators as ops
from reactivex.disposable import Disposable
from reactivex.subject import Subject
import rerun as rr
import rerun.blueprint as rrb

from dimos.core import In, Module, Out, rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import ModuleConfig
from dimos.dashboard.rerun_init import connect_rerun
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.utils.decorators import simple_mcache
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()


@dataclass
class Config(ModuleConfig):
    frame_id: str = "world"
    # -1 never publishes, 0 publishes on every frame, >0 publishes at interval in seconds
    publish_interval: float = 0
    voxel_size: float = 0.05
    block_count: int = 2_000_000
    device: str = "CUDA:0"
    carve_columns: bool = True


class VoxelGridMapper(Module):
    default_config = Config
    config: Config

    lidar: In[PointCloud2]
    global_map: Out[PointCloud2]

    @classmethod
    def rerun_views(cls):  # type: ignore[no-untyped-def]
        """Return Rerun view blueprints for voxel map visualization."""
        return [
            rrb.TimeSeriesView(
                name="Voxel Pipeline (ms)",
                origin="/metrics/voxel_map",
                contents=[
                    "+ /metrics/voxel_map/extract_ms",
                    "+ /metrics/voxel_map/transport_ms",
                    "+ /metrics/voxel_map/publish_ms",
                ],
            ),
            rrb.TimeSeriesView(
                name="Voxel Count",
                origin="/metrics/voxel_map",
                contents=["+ /metrics/voxel_map/voxel_count"],
            ),
        ]

    def __init__(self, global_config: GlobalConfig | None = None, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._global_config = global_config or GlobalConfig()

        dev = (
            o3c.Device(self.config.device)
            if (self.config.device.startswith("CUDA") and o3c.cuda.is_available())
            else o3c.Device("CPU:0")
        )

        print(f"VoxelGridMapper using device: {dev}")

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
        self._voxel_hashmap = self.vbg.hashmap()
        self._key_dtype = self._voxel_hashmap.key_tensor().dtype
        self._latest_frame_ts: float = 0.0
        # Monotonic timestamp of last received frame (for accurate latency in replay)
        self._latest_frame_rx_monotonic: float | None = None

    @rpc
    def start(self) -> None:
        super().start()

        # Only start Rerun logging if Rerun backend is selected
        if self._global_config.viewer_backend.startswith("rerun"):
            connect_rerun(global_config=self._global_config)
            logger.info("VoxelGridMapper: Rerun logging enabled (sync)")

        # Subject to trigger publishing, with backpressure to drop if busy
        self._publish_trigger: Subject[None] = Subject()
        self._disposables.add(
            backpressure(self._publish_trigger)
            .pipe(ops.map(lambda _: self.publish_global_map()))
            .subscribe()
        )

        lidar_unsub = self.lidar.subscribe(self._on_frame)
        self._disposables.add(Disposable(lidar_unsub))

        # If publish_interval > 0, publish on timer; otherwise publish on each frame
        if self.config.publish_interval > 0:
            self._disposables.add(
                interval(self.config.publish_interval).subscribe(
                    lambda _: self._publish_trigger.on_next(None)
                )
            )

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_frame(self, frame: PointCloud2) -> None:
        # Track receipt time with monotonic clock (works correctly in replay)
        self._latest_frame_rx_monotonic = time.monotonic()
        self.add_frame(frame)
        if self.config.publish_interval == 0:
            self._publish_trigger.on_next(None)

    def publish_global_map(self) -> None:
        # Snapshot monotonic timestamp once (won't be overwritten during slow publish)
        rx_monotonic = self._latest_frame_rx_monotonic

        start_total = time.perf_counter()

        # 1. Extract pointcloud from GPU hashmap
        t1 = time.perf_counter()
        pc = self.get_global_pointcloud2()
        extract_ms = (time.perf_counter() - t1) * 1000

        # 2. Publish to downstream (NO auto-logging - fast!)
        t2 = time.perf_counter()
        self.global_map.publish(pc)
        publish_ms = (time.perf_counter() - t2) * 1000
        # 3. Synchronous Rerun logging (no queues/threads).
        if self._global_config.viewer_backend.startswith("rerun"):
            try:
                rr.log(
                    "world/map",
                    pc.to_rerun(
                        mode="boxes",
                        size=self.config.voxel_size,
                        colormap="turbo",
                    ),
                )
            except Exception as e:
                logger.warning(f"Rerun logging error: {e}")

        # Log detailed timing breakdown to Rerun
        total_ms = (time.perf_counter() - start_total) * 1000
        rr.log("metrics/voxel_map/publish_ms", rr.Scalars(total_ms))
        rr.log("metrics/voxel_map/extract_ms", rr.Scalars(extract_ms))
        rr.log("metrics/voxel_map/transport_ms", rr.Scalars(publish_ms))
        rr.log("metrics/voxel_map/voxel_count", rr.Scalars(float(len(pc))))

        # Log pipeline latency (time from frame receipt to publish complete)
        if rx_monotonic is not None:
            latency_ms = (time.monotonic() - rx_monotonic) * 1000
            rr.log("metrics/voxel_map/latency_ms", rr.Scalars(latency_ms))

    def size(self) -> int:
        return self._voxel_hashmap.size()  # type: ignore[no-any-return]

    def __len__(self) -> int:
        return self.size()

    # @timed()  # TODO: fix thread leak in timed decorator
    def add_frame(self, frame: PointCloud2) -> None:
        # Track latest frame timestamp for proper latency measurement
        if hasattr(frame, "ts") and frame.ts:
            self._latest_frame_ts = frame.ts

        # we are potentially moving into CUDA here
        pcd = ensure_tensor_pcd(frame.pointcloud, self._dev)

        if pcd.is_empty():
            return

        pts = pcd.point["positions"].to(self._dev, o3c.float32)
        vox = (pts / self.config.voxel_size).floor().to(self._key_dtype)
        keys_Nx3 = vox.contiguous()

        if self.config.carve_columns:
            self._carve_and_insert(keys_Nx3)
        else:
            self._voxel_hashmap.activate(keys_Nx3)

        self.get_global_pointcloud.invalidate_cache(self)  # type: ignore[attr-defined]
        self.get_global_pointcloud2.invalidate_cache(self)  # type: ignore[attr-defined]

    def _carve_and_insert(self, new_keys: o3c.Tensor) -> None:
        """Column carving: remove all existing voxels sharing (X,Y) with new_keys, then insert."""
        if new_keys.shape[0] == 0:
            self._voxel_hashmap.activate(new_keys)
            return

        # Extract (X, Y) from incoming keys
        xy_keys = new_keys[:, :2].contiguous()

        # Build temp hashmap for O(1) (X,Y) membership lookup
        xy_hashmap = o3c.HashMap(
            init_capacity=xy_keys.shape[0],
            key_dtype=self._key_dtype,
            key_element_shape=o3c.SizeVector([2]),
            value_dtypes=[o3c.uint8],
            value_element_shapes=[o3c.SizeVector([1])],
            device=self._dev,
        )
        dummy_vals = o3c.Tensor.zeros((xy_keys.shape[0], 1), o3c.uint8, self._dev)
        xy_hashmap.insert(xy_keys, dummy_vals)

        # Get existing keys from main hashmap
        active_indices = self._voxel_hashmap.active_buf_indices()
        if active_indices.shape[0] == 0:
            self._voxel_hashmap.activate(new_keys)
            return

        existing_keys = self._voxel_hashmap.key_tensor()[active_indices]
        existing_xy = existing_keys[:, :2].contiguous()

        # Find which existing keys have (X,Y) in the incoming set
        _, found_mask = xy_hashmap.find(existing_xy)

        # Erase those columns
        to_erase = existing_keys[found_mask]
        if to_erase.shape[0] > 0:
            self._voxel_hashmap.erase(to_erase)

        # Insert new keys
        self._voxel_hashmap.activate(new_keys)

    # returns PointCloud2 message (ready to send off down the pipeline)
    @simple_mcache
    def get_global_pointcloud2(self) -> PointCloud2:
        return PointCloud2(
            # we are potentially moving out of CUDA here
            ensure_legacy_pcd(self.get_global_pointcloud()),
            frame_id=self.frame_id,
            ts=self._latest_frame_ts if self._latest_frame_ts else time.time(),
        )

    @simple_mcache
    # @timed()
    def get_global_pointcloud(self) -> o3d.t.geometry.PointCloud:
        voxel_coords, _ = self.vbg.voxel_coordinates_and_flattened_indices()
        pts = voxel_coords + (self.config.voxel_size * 0.5)
        out = o3d.t.geometry.PointCloud(device=self._dev)
        out.point["positions"] = pts
        return out


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


voxel_mapper = VoxelGridMapper.blueprint
