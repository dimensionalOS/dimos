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

"""Voxel occupancy mapper over recorded world-frame lidar — the first real pure module.

Accumulates ``PointCloud2`` scans into a sparse voxel grid, reusing
:class:`dimos.mapping.voxels.VoxelGrid` (the repo's Open3D voxel-hashmap core),
and emits the occupied-voxel-centers cloud every ``emit_every`` scans plus a
final row at stream exhaustion.

Shape — ``fold``, not the sketch3 §3 Mealy: the grid is a mutable, disposable
Open3D accumulator, inadmissible as Mealy ``State`` (immutable plain data), and
a plain-data grid re-copied per tick is O(grid) at every scan (~4k ticks x 1e5
voxels on the reference dataset). ``@resource`` (T7, the sketch's answer) has
not landed; fold's generator-local state with dispose-in-``finally`` is the
sanctioned offline-natural shape and mirrors the proven
``dimos.mapping.voxels.VoxelMapTransformer`` one-to-one. Once T7 lands, this
module can migrate to the sketch's Mealy-plus-resource spelling.

No pose port: the reference dataset's scans arrive already transformed to the
``world`` frame (LIO pipeline output) — verified empirically: per-scan bounding
boxes track the robot's world-frame trajectory (scan centroids match the
interpolated odom position throughout the run) and static geometry stays
globally consistent. A sensor-frame dataset would add
``pose: PoseStamped = pm.interpolate()`` (after
``dimos.pure.interpolators.install()``) and transform each scan in step; this
module deliberately keeps the simpler world-frame shape.

Reference dataset ``get_data("go2_hongkong_office.db")`` — Go2 office session,
558 s, 2026-05-06:

- ``lidar``: ``PointCloud2``, 4235 @ 7.6 Hz, frame ``world``, ~20k pts/scan
  locally clipped to ~+-3 m around the robot, z in [-0.2, 1.1]
- ``odom``: ``PoseStamped``, 10437 @ 18.7 Hz, frame ``world`` (trajectory
  ~24 x 28 m, ~180 m path)
- ``color_image`` / ``color_image_embedded``: ``Image``, 14.3 / 1.8 Hz, frame
  ``camera_optical``

Payloads carry their own ``ts`` (``PointCloud2.ts`` is set; payload ts
slightly precedes the envelope ts — payload wins at the ``over()`` boundary).
Payload ts is NOT strictly monotonic (2 regressions in the first 300 scans);
alignment drops such items (``dropped_nonmonotonic``), so 300 stored scans
become 298 ticks. Ground truth
``get_data("go2_hongkong_office_twopass_map.pc2.lcm")`` — PGO twopass map,
490k points, frame ``world``, load via ``PointCloud2.lcm_decode``; the
first-300-scan map agrees 0.89 @ 1 voxel / 0.97 @ 2 voxels (0.1 m voxels).
The full session (4227 ticks -> 120k voxels) agrees 0.73 / 0.85 / 0.94 @
1 / 2 / 5 voxels — online LIO drifts from the PGO solution late in the run.

``device`` defaults to ``"CPU:0"``: measured 5x faster than CUDA on this
workload (~20k-point scans — GPU launch overhead dominates) and deterministic
across machines; both devices produce identical grids.
"""

from __future__ import annotations

from collections.abc import Iterator

from dimos import pure as pm
from dimos.mapping.voxels import VoxelGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

__all__ = ["VoxelMapper"]


class VoxelMapper(pm.PureModule):
    """Fold world-frame lidar scans into a sparse voxel occupancy map."""

    voxel_size: float = 0.1  # metric voxel edge length
    emit_every: int = 20  # emit every n scans; <= 0 emits only at exhaustion
    device: str = "CPU:0"  # o3d device; CPU measured faster than CUDA here
    carve_columns: bool = True  # clear re-observed (x, y) columns before insert
    frame_id: str = "world"  # frame stamped on the emitted map

    class In(pm.In):
        scan: PointCloud2 = pm.tick(expect_hz=8)

    class Out(pm.Out):
        global_map: PointCloud2 = pm.contract(min_hz=0.25)  # occupied-voxel centers
        n_voxels: int
        n_scans: int

    def fold(self, rows: Iterator[In]) -> Iterator[Out]:
        """Accumulate scans generator-locally; dispose the grid on any exit."""
        grid = VoxelGrid(
            voxel_size=self.voxel_size,
            device=self.device,
            carve_columns=self.carve_columns,
            frame_id=self.frame_id,
            show_startup_log=False,
        )
        try:
            n = 0
            last_ts: float | None = None
            for r in rows:
                grid.add_frame(r.scan)
                n += 1
                last_ts = r.ts
                if self.emit_every > 0 and n % self.emit_every == 0:
                    yield self._snapshot(grid, r.ts, n)
            if last_ts is not None and (self.emit_every <= 0 or n % self.emit_every != 0):
                yield self._snapshot(grid, last_ts, n)
        finally:
            grid.dispose()

    def _snapshot(self, grid: VoxelGrid, ts: float, n: int) -> Out:
        """Current occupied-voxel-centers cloud as a stamped Out row."""
        return VoxelMapper.Out(
            ts=ts,
            global_map=grid.get_global_pointcloud2(),
            n_voxels=len(grid),
            n_scans=n,
        )
