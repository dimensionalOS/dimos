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

"""Real-time RGB-D TSDF 3D mesh reconstruction module.

Integrates aligned color + depth frames and ``tf`` poses into spatially-chunked
Open3D ``ScalableTSDFVolume`` instances and publishes dirty region meshes onto
``mesh`` for live Rerun 3D visualization.
"""

from __future__ import annotations

from collections import deque
import queue
import threading
import time
from typing import TYPE_CHECKING, Any

import numpy as np
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.shape_msgs.TriangleMesh import TriangleMesh
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import TF
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray
    import open3d as o3d  # type: ignore[import-untyped]

logger = setup_logger()

DEPTH_UNITS_PER_M = 1000.0
# ScalableTSDFVolume allocates 16x16x16 voxel volume units; padding by one unit plus
# sdf_trunc ensures adjacent cells share identical TSDF values across cell boundaries.
_VOLUME_UNIT_RES = 16


def _transform_to_matrix(transform: Transform) -> NDArray[np.float64]:
    mat = np.eye(4, dtype=np.float64)
    q = transform.rotation
    t = transform.translation
    mat[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


class RgbdMeshConfig(ModuleConfig):
    world_frame: str = "world"
    voxel_size: float = 0.02
    depth_min: float = 0.3
    depth_max: float = 3.5
    min_fill: float = 0.10
    region_m: float = 3.0
    emit_every: int = 3
    min_emit_interval_s: float = 0.20
    pair_tolerance_s: float = 0.02
    tf_tolerance_s: float = 0.25
    tf_wait_s: float = 0.15
    min_cell_pixels: int = 400
    queue_size: int = 64


class RgbdMesh(Module):
    """Real-time RGB-D TSDF mesh reconstructor."""

    config: RgbdMeshConfig
    dedicated_worker = True

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]
    tf: In[TFMessage]

    mesh: Out[TriangleMesh]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._pair_lock = threading.Lock()
        self._colors: deque[Image] = deque(maxlen=30)
        self._depths: deque[Image] = deque(maxlen=30)
        self._pair_queue: queue.Queue[tuple[Image, Image] | None] = queue.Queue(
            maxsize=self.config.queue_size
        )
        self._info_lock = threading.Lock()
        self._info_ready = threading.Event()
        self._info: CameraInfo | None = None
        self._intrinsic: o3d.camera.PinholeCameraIntrinsic | None = None
        self._ray_grid: tuple[NDArray[np.float32], NDArray[np.float32]] | None = None
        self._cells: dict[tuple[int, int], Any] = {}
        self._dirty: set[tuple[int, int]] = set()
        self._integrated_frames = 0
        self._last_emit = 0.0
        self._stop_event = threading.Event()
        self._worker_thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        if self._tf is None:
            self._tf = TF(self.tf, buffer_size=30.0)
        self.register_disposable(Disposable(self.camera_info.subscribe(self._on_camera_info)))
        self.register_disposable(Disposable(self.color_image.subscribe(self._on_color)))
        self.register_disposable(Disposable(self.depth_image.subscribe(self._on_depth)))
        self._worker_thread = threading.Thread(
            target=self._worker_loop, name="rgbd-mesh-worker", daemon=True
        )
        self._worker_thread.start()
        logger.info(
            "RgbdMesh started (voxel=%.3fm, region=%.1fm, depth=%.1f-%.1fm)",
            self.config.voxel_size,
            self.config.region_m,
            self.config.depth_min,
            self.config.depth_max,
        )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        self._info_ready.set()
        if self._tf is not None:
            with self._tf._cv:
                self._tf._cv.notify_all()
        try:
            self._pair_queue.put_nowait(None)
        except queue.Full:
            pass
        if self._worker_thread is not None and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._worker_thread = None
        super().stop()

    def _on_camera_info(self, msg: CameraInfo) -> None:
        import open3d as o3d  # type: ignore[import-untyped]

        with self._info_lock:
            if (
                self._info is None
                or self._info.width != msg.width
                or self._info.height != msg.height
                or self._info.K != msg.K
            ):
                k = msg.K
                fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
                if fx <= 0 or fy <= 0 or msg.width <= 0 or msg.height <= 0:
                    return
                self._info = msg
                self._intrinsic = o3d.camera.PinholeCameraIntrinsic(
                    msg.width, msg.height, fx, fy, cx, cy
                )
                u = (np.arange(msg.width, dtype=np.float32) - np.float32(cx)) / np.float32(fx)
                v = (np.arange(msg.height, dtype=np.float32) - np.float32(cy)) / np.float32(fy)
                self._ray_grid = (np.meshgrid(u, v)[0], np.meshgrid(u, v)[1])
                self._info_ready.set()

    def _on_color(self, color: Image) -> None:
        pair: tuple[Image, Image] | None = None
        with self._pair_lock:
            best_idx = -1
            best_dt = self.config.pair_tolerance_s
            for i, d in enumerate(self._depths):
                dt = abs(d.ts - color.ts)
                if dt <= best_dt:
                    best_dt = dt
                    best_idx = i
            if best_idx >= 0:
                depth = self._depths[best_idx]
                del self._depths[best_idx]
                pair = (color, depth)
            else:
                self._colors.append(color)
        if pair is not None:
            self._enqueue_pair(pair)

    def _on_depth(self, depth: Image) -> None:
        pair: tuple[Image, Image] | None = None
        with self._pair_lock:
            best_idx = -1
            best_dt = self.config.pair_tolerance_s
            for i, c in enumerate(self._colors):
                dt = abs(c.ts - depth.ts)
                if dt <= best_dt:
                    best_dt = dt
                    best_idx = i
            if best_idx >= 0:
                color = self._colors[best_idx]
                del self._colors[best_idx]
                pair = (color, depth)
            else:
                self._depths.append(depth)
        if pair is not None:
            self._enqueue_pair(pair)

    def _enqueue_pair(self, pair: tuple[Image, Image]) -> None:
        try:
            self._pair_queue.put_nowait(pair)
        except queue.Full:
            try:
                self._pair_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self._pair_queue.put_nowait(pair)
            except queue.Full:
                pass

    def _worker_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                item = self._pair_queue.get(timeout=0.1)
            except queue.Empty:
                if self._dirty:
                    self._emit_dirty(time.time())
                continue
            if item is None or self._stop_event.is_set():
                break
            color, depth = item
            if not self._info_ready.wait(timeout=2.0) or self._stop_event.is_set():
                continue
            try:
                self._integrate_pair(color, depth)
            except Exception:
                logger.error("RgbdMesh failed to integrate frame", exc_info=True)

    def _integrate_pair(self, color: Image, depth: Image) -> None:
        import open3d as o3d  # type: ignore[import-untyped]

        with self._info_lock:
            info = self._info
            intrinsic = self._intrinsic
            ray_grid = self._ray_grid
        if info is None or intrinsic is None or ray_grid is None:
            return

        frame_id = color.frame_id or depth.frame_id or info.frame_id
        if not frame_id:
            return

        transform = self.tfbuffer.get(
            self.config.world_frame,
            frame_id,
            time_point=color.ts,
            time_tolerance=self.config.tf_tolerance_s,
            forward_tolerance=self.config.tf_wait_s,
            warn=False,
        )
        if transform is None:
            return

        d_raw = np.ascontiguousarray(depth.data)
        if d_raw.dtype == np.uint16:
            lo = self.config.depth_min * DEPTH_UNITS_PER_M
            hi = self.config.depth_max * DEPTH_UNITS_PER_M
            valid = (d_raw >= lo) & (d_raw < hi)
            if float(np.mean(valid)) < self.config.min_fill:
                return
            z = d_raw.astype(np.float32) * np.float32(1.0 / DEPTH_UNITS_PER_M)
            depth_scale = DEPTH_UNITS_PER_M
            zero_val: Any = np.uint16(0)
        else:
            d_raw = d_raw.astype(np.float32, copy=False)
            valid = (
                np.isfinite(d_raw)
                & (d_raw >= self.config.depth_min)
                & (d_raw < self.config.depth_max)
            )
            if float(np.mean(valid)) < self.config.min_fill:
                return
            z = d_raw
            depth_scale = 1.0
            zero_val = np.float32(0.0)

        rgb = np.ascontiguousarray(color.to_rgb().data, dtype=np.uint8)
        pose = _transform_to_matrix(transform)
        extrinsic = np.linalg.inv(pose)

        rx, ry = ray_grid
        R = pose[:3, :3].astype(np.float32)
        t = pose[:3, 3].astype(np.float32)
        xw = z * (R[0, 0] * rx + R[0, 1] * ry + R[0, 2]) + t[0]
        yw = z * (R[1, 0] * rx + R[1, 1] * ry + R[1, 2]) + t[1]

        voxel_size = self.config.voxel_size
        sdf_trunc = voxel_size * 3.0
        region_m = self.config.region_m
        pad = _VOLUME_UNIT_RES * voxel_size + sdf_trunc

        xw_valid = xw[valid]
        yw_valid = yw[valid]
        ix_min = int(np.floor((float(xw_valid.min()) - pad) / region_m))
        ix_max = int(np.floor((float(xw_valid.max()) + pad) / region_m))
        iy_min = int(np.floor((float(yw_valid.min()) - pad) / region_m))
        iy_max = int(np.floor((float(yw_valid.max()) + pad) / region_m))

        color_o3d = o3d.geometry.Image(rgb)
        depth_trunc = self.config.depth_max + 0.1

        for ix in range(ix_min, ix_max + 1):
            x0 = ix * region_m - pad
            x1 = (ix + 1) * region_m + pad
            in_x = valid & (xw >= x0) & (xw <= x1)
            if not np.any(in_x):
                continue
            for iy in range(iy_min, iy_max + 1):
                y0 = iy * region_m - pad
                y1 = (iy + 1) * region_m + pad
                mask = in_x & (yw >= y0) & (yw <= y1)
                if int(np.count_nonzero(mask)) < self.config.min_cell_pixels:
                    continue
                key = (ix, iy)
                vol = self._cells.get(key)
                if vol is None:
                    vol = o3d.pipelines.integration.ScalableTSDFVolume(
                        voxel_length=voxel_size,
                        sdf_trunc=sdf_trunc,
                        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
                    )
                    self._cells[key] = vol
                d_cell = np.where(mask, d_raw, zero_val)
                rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    color_o3d,
                    o3d.geometry.Image(d_cell),
                    depth_scale=depth_scale,
                    depth_trunc=depth_trunc,
                    convert_rgb_to_intensity=False,
                )
                vol.integrate(rgbd, intrinsic, extrinsic)
                self._dirty.add(key)

        self._integrated_frames += 1
        now = time.monotonic()
        if (
            self._dirty
            and self._integrated_frames % self.config.emit_every == 0
            and now - self._last_emit >= self.config.min_emit_interval_s
        ):
            self._emit_dirty(color.ts)

    def _emit_dirty(self, ts: float) -> None:
        self._last_emit = time.monotonic()
        region_m = self.config.region_m
        dirty_keys = list(self._dirty)
        self._dirty.clear()

        for ix, iy in dirty_keys:
            if self._stop_event.is_set():
                break
            vol = self._cells.get((ix, iy))
            if vol is None:
                continue
            o3d_mesh = vol.extract_triangle_mesh()
            verts = np.asarray(o3d_mesh.vertices, dtype=np.float32)
            tris = np.asarray(o3d_mesh.triangles, dtype=np.uint32)
            if len(verts) == 0 or len(tris) == 0:
                continue
            o3d_mesh.compute_vertex_normals()
            normals = np.asarray(o3d_mesh.vertex_normals, dtype=np.float32)
            colors = (np.asarray(o3d_mesh.vertex_colors) * 255.0).astype(np.uint8)

            # Keep triangles whose centroids lie in [ix*region_m, (ix+1)*region_m) x [iy*region_m, (iy+1)*region_m)
            # so adjacent regions stitch seamlessly without overlapping duplicate bands.
            centroids = verts[tris].mean(axis=1)
            x0, x1 = ix * region_m, (ix + 1) * region_m
            y0, y1 = iy * region_m, (iy + 1) * region_m
            keep = (
                (centroids[:, 0] >= x0)
                & (centroids[:, 0] < x1)
                & (centroids[:, 1] >= y0)
                & (centroids[:, 1] < y1)
            )
            tris = tris[keep]
            if len(tris) == 0:
                continue

            used = np.zeros(len(verts), dtype=bool)
            used[tris.ravel()] = True
            remap = np.empty(len(verts), dtype=np.uint32)
            remap[used] = np.arange(int(np.count_nonzero(used)), dtype=np.uint32)
            verts = verts[used]
            normals = normals[used]
            colors = colors[used]
            tris = remap[tris]

            self.mesh.publish(
                TriangleMesh(
                    verts,
                    tris,
                    normals,
                    key=(ix, iy, 0),
                    ts=ts,
                    colors=colors,
                )
            )
