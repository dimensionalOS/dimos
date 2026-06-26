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

"""Post-traversal PGO map correction for camera-only navigation.

Reads a traversal database recorded by ``CameraNavRecorder``, runs
pose-graph optimization (ICP loop closure via ISAM2), then re-accumulates
all frame clouds using the drift-corrected poses.  The result is a global
point cloud that is globally consistent even after long traversals.

Usage::

    python -m dimos.navigation.camera_nav.correct_map traversal.db

Options::

    --output corrected.pcd   Output file (default: <db_stem>_corrected.pcd)
    --stream frame_cloud     Stream name inside the database (default: frame_cloud)
    --voxel 0.05             Voxel size in metres for output downsampling
    --rerun                  Open Rerun viewer to inspect raw vs corrected map

Prerequisites
-------------
- ``gtsam`` must be installed (``pip install gtsam``)
- The database must have been recorded with ``CameraNavRecorder`` so that
  each ``frame_cloud`` observation has a ``world <- base_link`` pose attached
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import open3d as o3d

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _load_stream(db_path: Path, stream_name: str):  # type: ignore[return]
    """Open a SQLite store and return the named PointCloud2 stream."""
    from dimos.mapping.loop_closure.pgo import PGO
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    store = SqliteStore(path=str(db_path))
    stream = store.stream(stream_name, PointCloud2)
    return store, stream, PGO


def _run_pgo(stream):
    """Run PGO on the stream; return the PoseGraph."""
    from dimos.mapping.loop_closure.pgo import PGO

    logger.info("Running pose graph optimization (pass 1)…")
    t0 = time.perf_counter()
    graph = stream.transform(PGO()).last().data
    elapsed = time.perf_counter() - t0
    n_kf = len(graph.keyframes)
    n_lc = len(graph.loops)
    logger.info(
        "PGO done in %.1fs — %d keyframes, %d loop closures", elapsed, n_kf, n_lc
    )
    if n_lc == 0:
        logger.warning(
            "No loop closures detected.  The robot may not have revisited any area, "
            "or the trajectory is too short for ICP to find matches.  "
            "The output map will have the same drift as the raw traversal."
        )
    return graph


def _accumulate_corrected(stream, graph, voxel: float) -> o3d.geometry.PointCloud:
    """Re-accumulate all frame clouds using PGO-corrected poses.

    Each cloud is transformed from raw-world to corrected-world by the
    per-timestamp drift correction from the PoseGraph, then merged into
    a single voxel-downsampled point cloud.
    """
    logger.info("Accumulating corrected map (pass 2)…")
    t0 = time.perf_counter()

    pts_list: list[np.ndarray] = []
    cols_list: list[np.ndarray] = []
    n_frames = 0

    for obs in stream:
        if obs.pose_tuple is None:
            # No pose → cloud was captured without odometry; skip.
            continue
        if len(obs.data) == 0:
            continue

        correction = graph.correction_at(obs.ts)
        corrected = obs.data.transform(correction)
        pts, cols = corrected.as_numpy()
        if len(pts) == 0:
            continue

        pts_list.append(pts.astype(np.float64))
        if cols is not None and len(cols) == len(pts):
            cols_list.append(cols.astype(np.float64))
        n_frames += 1

    if not pts_list:
        logger.error("No posed frames found in the recording.  Was CameraNavRecorder used?")
        return o3d.geometry.PointCloud()

    logger.info("Merging %d frames…", n_frames)
    merged = o3d.geometry.PointCloud()
    merged.points = o3d.utility.Vector3dVector(np.concatenate(pts_list))
    if len(cols_list) == len(pts_list):
        merged.colors = o3d.utility.Vector3dVector(np.concatenate(cols_list))

    downsampled = merged.voxel_down_sample(voxel)
    elapsed = time.perf_counter() - t0
    logger.info(
        "Done in %.1fs — %d points (%.2f m voxel)",
        elapsed,
        len(downsampled.points),
        voxel,
    )
    return downsampled


def _save(pcd: o3d.geometry.PointCloud, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(output), pcd)
    logger.info("Saved corrected map → %s", output)


def _visualize(raw_stream, graph, corrected: o3d.geometry.PointCloud) -> None:
    """Log raw and corrected maps to a Rerun web viewer."""
    import rerun as rr

    rr.init("camera_nav_correct_map")
    server_uri = rr.serve_grpc()
    rr.serve_web_viewer(connect_to=server_uri, open_browser=True)

    # Raw (drifted) map
    raw_pts_list: list[np.ndarray] = []
    for obs in raw_stream:
        if len(obs.data) == 0:
            continue
        pts, _ = obs.data.as_numpy()
        if len(pts) > 0:
            raw_pts_list.append(pts.astype(np.float32))
    if raw_pts_list:
        raw_pts = np.concatenate(raw_pts_list)
        rr.log("/raw_map", rr.Points3D(positions=raw_pts, colors=[180, 180, 180]), static=True)

    # Corrected map
    pts_corr = np.asarray(corrected.points, dtype=np.float32)
    cols_corr = None
    if corrected.has_colors():
        cols_corr = (np.asarray(corrected.colors, dtype=np.float32) * 255).astype(np.uint8)
    if cols_corr is not None:
        rr.log("/corrected_map", rr.Points3D(positions=pts_corr, colors=cols_corr), static=True)
    else:
        rr.log("/corrected_map", rr.Points3D(positions=pts_corr), static=True)

    # PGO trajectory
    if graph.keyframes:
        traj = [
            (kf.optimized.translation.x, kf.optimized.translation.y, kf.optimized.translation.z)
            for kf in graph.keyframes
        ]
        rr.log(
            "/pgo_path",
            rr.LineStrips3D(strips=[traj], colors=[[255, 255, 0]], radii=[0.02]),
            static=True,
        )
        loop_pairs = [
            [
                (lc.source.translation.x, lc.source.translation.y, lc.source.translation.z),
                (lc.target.translation.x, lc.target.translation.y, lc.target.translation.z),
            ]
            for lc in graph.loops
        ]
        if loop_pairs:
            rr.log(
                "/loop_closures",
                rr.LineStrips3D(strips=loop_pairs, colors=[[255, 0, 128]], radii=[0.015]),
                static=True,
            )

    logger.info("Rerun viewer opened — press Ctrl+C to exit")
    try:
        import time as _time
        while True:
            _time.sleep(1.0)
    except KeyboardInterrupt:
        pass


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Post-traversal PGO map correction for camera-only navigation"
    )
    parser.add_argument("db", type=Path, help="SQLite recording from CameraNavRecorder")
    parser.add_argument(
        "--output", "-o", type=Path, default=None,
        help="Output .pcd path (default: <db_stem>_corrected.pcd next to the db)"
    )
    parser.add_argument(
        "--stream", default="frame_cloud",
        help="Stream name in the database (default: frame_cloud)"
    )
    parser.add_argument(
        "--voxel", type=float, default=0.05,
        help="Output voxel size in metres (default: 0.05)"
    )
    parser.add_argument(
        "--rerun", action="store_true",
        help="Open Rerun viewer to compare raw and corrected maps"
    )
    args = parser.parse_args()

    db_path: Path = args.db
    if not db_path.exists():
        raise FileNotFoundError(f"Database not found: {db_path}")

    output: Path = args.output or db_path.with_name(f"{db_path.stem}_corrected.pcd")

    store, stream, _ = _load_stream(db_path, args.stream)
    n_total = stream.count()
    logger.info("Loaded %d observations from '%s' in %s", n_total, args.stream, db_path)

    if n_total == 0:
        logger.error("Stream '%s' is empty — nothing to correct", args.stream)
        return

    graph = _run_pgo(stream)
    corrected = _accumulate_corrected(stream, graph, voxel=args.voxel)

    if len(corrected.points) == 0:
        logger.error("Corrected map is empty — no posed frames were found")
        return

    _save(corrected, output)

    if args.rerun:
        _visualize(stream, graph, corrected)

    store.stop()


if __name__ == "__main__":
    main()
