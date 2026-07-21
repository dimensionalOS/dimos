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

"""Voxel occupancy mapper over recorded lidar — the first real pure module.

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

Frame-agnostic input via an optional ``pose`` tf() port: it samples the
``frame_id <- sensor_frame`` chain at each tick and every scan is transformed
into ``frame_id`` (the grid/output frame, one source of truth) before insertion.
Leave tf unwired — as the reference dataset does, its scans already world-frame
LIO output — and ``pose`` defaults None, scans insert as-is (the historical
shape). Wire ``over(tf=<stream>)`` (or a live ``m.i.tf`` feed) with
``sensor_frame`` set to the scan's own frame to map a sensor-frame dataset in;
the buffer composes the full chain, so only the two endpoints are named here. An
unresolved tick falls through untransformed (optional-port passthrough), not
dropped — wire tf only for genuinely sensor-framed data.

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
"""

from __future__ import annotations

from collections.abc import Iterator

from dimos import pure as pm
from dimos.mapping.voxels import VoxelGrid
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

__all__ = ["VoxelMapper", "VoxelMapper2"]

logger = setup_logger()


class VoxelMapper(pm.PureModule):
    """Fold lidar scans into a sparse voxel occupancy map (optional tf into frame_id)."""

    voxel_size: float = 0.1  # metric voxel edge length
    emit_every: int = 20  # emit every n scans; <= 0 emits only at exhaustion
    device: str = "CUDA:0"
    carve_columns: bool = True  # clear re-observed (x, y) columns before insert
    frame_id: str = "world"  # grid/output frame; also the pose tf() target
    sensor_frame: str = "sensor"  # scan's own frame; pose samples frame_id <- sensor_frame

    class In(pm.In):
        scan: PointCloud2 = pm.tick(expect_hz=8)
        pose: Transform | None = pm.tf("{frame_id}", "{sensor_frame}", default=None)

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
            show_startup_log=True,
        )
        try:
            n = 0
            last_ts: float | None = None
            for row in rows:
                grid.add_frame(row.scan if row.pose is None else row.scan.transform(row.pose))
                n += 1
                last_ts = row.ts
                if self.emit_every > 0 and n % self.emit_every == 0:
                    yield self._snapshot(grid, row.ts, n)
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


class VoxelMapper2(pm.PureModule):
    """Mealy-plus-``@resource`` respelling of :class:`VoxelMapper` — the sketch3 §3 shape.

    Same VoxelGrid core, same config, same ``Out`` — a side-by-side of two module
    shapes, not a replacement (:class:`VoxelMapper` stays). The grid is now an
    engine-owned ``@resource`` (T7): built fresh per run at warmup, disposed at
    teardown (``VoxelGrid.dispose`` is sniffed), lazily cached outside a run so a
    unit test can touch ``self.grid`` with no engine. ``State`` is plain data —
    a scan counter — so it is checkpointable (T10-compatible); the heavy mutable
    accumulator lives in the resource, exactly where resources belong.

    The ``finish(state)`` hook (T14+) restores full fold parity: a step never
    learns the stream ended, but ``finish`` is called once at exhaustion, so the
    trailing partial batch flushes exactly as the fold's tail does. With
    ``emit_every=50`` over 298 ticks both shapes emit at 50..250 and a 298 tail.
    What the Mealy shape still buys over the fold: checkpointable plain-data
    ``State`` and engine-managed grid lifecycle — the live-mapping trade (resume a
    long mapping run from a checkpoint; the engine owns warmup and teardown
    instead of a generator-local ``try/finally``).

    Comparison (fold ``VoxelMapper`` vs Mealy ``VoxelMapper2``)::

        aspect            VoxelMapper (fold)          VoxelMapper2 (Mealy + resource)
        ─────────────────────────────────────────────────────────────────────────
        shape             fold(rows) -> Iterator      step(state, i) -> (state, Out?)
        grid home         generator-local var         @resource (engine-owned)
        state home        closure locals (n, last_ts) State NamedTuple (plain data)
        grid lifecycle    try/finally dispose          warmup create / teardown dispose
        exhaustion flush  YES (final partial batch)    YES (finish() tail flush)
        checkpointable    no (state is in the stack)   yes (State is plain data)
        emit at 298?      yes                          yes (finish flushes the tail)
        logic LOC         28 (fold + _snapshot)        26 (State + grid + step + finish)

    At every emit point (the shared cadence + the tail) the two emit identical
    maps — same VoxelGrid, same insertion order. See ``test_voxel_mapper.py`` for
    the full-parity test.
    """

    voxel_size: float = 0.1  # metric voxel edge length
    emit_every: int = 20  # emit every n scans; <= 0 never emits (no exhaustion flush)
    device: str = "CUDA:0"
    carve_columns: bool = True  # clear re-observed (x, y) columns before insert
    frame_id: str = "world"  # grid/output frame; also the pose tf() target
    sensor_frame: str = "sensor"  # scan's own frame; pose samples frame_id <- sensor_frame

    class In(pm.In):
        scan: PointCloud2 = pm.tick(expect_hz=8)
        pose: Transform | None = pm.tf("{frame_id}", "{sensor_frame}", default=None)

    class Out(pm.Out):
        global_map: PointCloud2 = pm.contract(min_hz=0.25)  # occupied-voxel centers
        n_voxels: int
        n_scans: int

    class State(pm.State):
        n_scans: int = 0  # scans folded so far — plain data, checkpointable

    @pm.resource
    def grid(self) -> VoxelGrid:
        """Fresh per-run VoxelGrid from config; engine disposes it at teardown."""
        return VoxelGrid(
            voxel_size=self.voxel_size,
            device=self.device,
            carve_columns=self.carve_columns,
            frame_id=self.frame_id,
            show_startup_log=True,
        )

    def step(self, s: State, i: In) -> tuple[State, Out | None]:
        """Add the scan to the grid, bump the counter, emit a snapshot on cadence."""
        self.grid.add_frame(i.scan if i.pose is None else i.scan.transform(i.pose))
        n = s.n_scans + 1
        s = s.replace(n_scans=n)
        if self.emit_every > 0 and n % self.emit_every == 0:
            return s, VoxelMapper2.Out(
                global_map=self.grid.get_global_pointcloud2(),
                n_voxels=len(self.grid),
                n_scans=n,
            )  # engine stamps ts from the tick row
        return s, None

    def finish(self, s: State) -> Out | None:
        """Flush the trailing partial batch at stream end (fold-tail parity)."""
        n = s.n_scans
        if n == 0 or (self.emit_every > 0 and n % self.emit_every == 0):
            return None  # nothing consumed, or the last tick already emitted on cadence
        return VoxelMapper2.Out(
            global_map=self.grid.get_global_pointcloud2(),
            n_voxels=len(self.grid),
            n_scans=n,
        )  # engine stamps ts from the last consumed tick row
