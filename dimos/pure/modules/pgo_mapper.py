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

"""PGOVoxelMapper: a :class:`VoxelMapper2` whose map self-corrects on loop closure.

The offline flow (``dimos map global <db> --pgo``) is two-pass: run PGO over the
whole recording, then re-accumulate every scan with the final corrections. Live
there is no second pass — when a loop closes, the already-built map is wrong and
whoever holds it must rebuild it. :class:`_PGOState` (the incremental ISAM2 core
behind the offline ``PGO`` transformer) already keeps exactly what a rebuild
needs — per-keyframe body clouds and optimized poses — so the mapper and PGO
live in one module rather than duplicating that history across two.

The map IS the keyframes: ``global_map`` is the union of keyframe body clouds
(downsampled at ``submap_resolution``) registered at their optimized poses.
Non-keyframe scans never touch the grid — a tick between keyframes costs one
cheap pose-delta check. Per tick: feed the scan+pose to PGO (keyframe gating,
ICP loop search, ISAM2 — all internal to ``_PGOState``), then

- new keyframe, no loop → insert just that keyframe's cloud: incremental;
- a loop landed → reset the grid and re-register every keyframe cloud at its
  now-moved optimized pose, then emit immediately (the map just jumped —
  downstream should refresh now, not at the next cadence point).

Incremental insert and closure rebuild are the same operation (register
keyframe clouds), just one vs all. The full rebuild per accepted loop is the
irreducible cost; it blocks only this module's step, and the ``min_hz``
contract on ``global_map`` tolerates the hitch. If it ever bites, the next
step is an iterative background rebuild prioritizing the robot's surroundings.

``correction`` (world_corrected <- world_raw, a proper msg — wire-safe) lets
downstream consumers lift raw-frame data into the corrected frame, e.g. a
planner composing it onto the robot's odom pose.
"""

from __future__ import annotations

from dimos import pure as pm
from dimos.mapping.loop_closure.pgo import PGOConfig, _PGOState, _transform_to_pose3
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.nav_msgs.GraphNodes3D import GraphNode, GraphNodes3D
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.pure.modules.voxel_mapper import VoxelMapper2

__all__ = ["PGOVoxelMapper"]


class PGOVoxelMapper(VoxelMapper2):
    """Keyframe voxel map with online pose-graph optimization: rebuild on loop closure."""

    # PGO knobs (a curated subset; everything else uses PGOConfig defaults)
    key_pose_delta_trans: float = 1.0  # keyframe every this many meters of travel
    key_pose_delta_deg: float = 90.0  # ... or degrees of rotation
    loop_search_radius: float = 2.0  # candidate keyframes within this distance
    loop_time_thresh: float = 20.0  # ... and at least this many seconds old
    loop_score_thresh: float = 0.3  # accept ICP loops below this fitness (m^2)
    submap_resolution: float = 0.1  # keyframe cloud voxel size — the map's density floor

    class In(VoxelMapper2.In):
        # pose becomes REQUIRED: PGO unregisters world scans into body frame with it.
        pose: Transform = pm.tf("{frame_id}", "{sensor_frame}")

    class Out(VoxelMapper2.Out):
        correction: Transform  # world_corrected <- world_raw at the current tick
        n_keyframes: int
        n_loops: int
        # debug viz (rendered by any rerun sink via to_rerun): optimized keyframe
        # positions as graph nodes; accepted loop edges as line segments.
        keyframes: GraphNodes3D | None = None
        loop_closures: LineSegments3D | None = None

    @pm.resource
    def pgo(self) -> _PGOState:
        """Fresh per-run incremental PGO (ISAM2 + ICP loop closure) from config."""
        return _PGOState(
            PGOConfig(
                key_pose_delta_trans=self.key_pose_delta_trans,
                key_pose_delta_deg=self.key_pose_delta_deg,
                loop_search_radius=self.loop_search_radius,
                loop_time_thresh=self.loop_time_thresh,
                loop_score_thresh=self.loop_score_thresh,
                submap_resolution=self.submap_resolution,
            )
        )

    # Deliberate In narrowing (pose becomes required): a pure-module subclass redefines
    # its own step contract, and the engine dispatches on this class's step spec.
    def step(  # type: ignore[override]
        self, s: VoxelMapper2.State, i: In
    ) -> tuple[VoxelMapper2.State, Out | None]:
        """Feed PGO; insert new keyframe clouds; rebuild the grid when a loop lands."""
        pgo = self.pgo
        kfs_before = pgo.n_keyframes
        loops_before = pgo.n_loops
        # Placeholder filter (same as the offline PGO transformer): zero translation
        # OR uninitialized all-zero quaternion. Identity rotation (qw=1) is valid.
        if not (i.pose.translation.is_zero() or i.pose.rotation.is_zero()):
            pgo.process(_transform_to_pose3(i.pose), i.ts, i.scan)
        closed = pgo.n_loops > loops_before

        if closed:
            # the loop moved old keyframes — re-register all of them
            self.grid.reset()
            for cloud in pgo.keyframe_world_clouds():
                self.grid.add_frame(cloud)
        elif pgo.n_keyframes > kfs_before:
            # new keyframe, no loop — register just the new cloud
            for cloud in pgo.keyframe_world_clouds(start=kfs_before):
                self.grid.add_frame(cloud)

        n = s.n_scans + 1
        s = s.replace(n_scans=n)
        if closed or (self.emit_every > 0 and n % self.emit_every == 0):
            snap = pgo.snapshot()
            return s, PGOVoxelMapper.Out(
                global_map=self.grid.get_global_pointcloud2(),
                n_voxels=len(self.grid),
                n_scans=n,
                correction=pgo.correction(i.ts),
                n_keyframes=pgo.n_keyframes,
                n_loops=pgo.n_loops,
                keyframes=GraphNodes3D(
                    ts=i.ts,
                    frame_id=self.frame_id,
                    nodes=[
                        GraphNode(t.x, t.y, t.z)
                        for kf in snap.keyframes
                        for t in (kf.optimized.translation,)
                    ],
                ),
                loop_closures=LineSegments3D(
                    ts=i.ts,
                    frame_id=self.frame_id,
                    segments=[
                        ((a.x, a.y, a.z), (b.x, b.y, b.z))
                        for lp in snap.loops
                        for a, b in ((lp.source.translation, lp.target.translation),)
                    ],
                ),
            )  # engine stamps ts from the tick row
        return s, None
