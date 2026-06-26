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

"""Post-traversal PGO map correction.

Reads a traversal.db recorded by CameraNavRecorder, runs pose-graph
optimisation (ICP loop closure via ISAM2), then re-accumulates all frame
clouds using drift-corrected poses.

Usage::

    python -m dimos.navigation.camera_nav.correct_map traversal.db
    python -m dimos.navigation.camera_nav.correct_map traversal.db --output corrected.pcd --rerun
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
    from dimos.mapping.loop_closure.pgo import PGO
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    store = SqliteStore(path=str(db_path))
    stream = store.stream(stream_name, PointCloud2)
    return store, stream, PGO


def _run_pgo(stream):
    from dimos.mapping.loop_closure.pgo import PGO

    logger.info("Running pose graph optimization…")
    t0 = time.perf_counter()
    graph = stream.transform(PGO()).last().data
    n_kf = len(graph.keyframes)
    n_lc = len(graph.loops)
    logger.info("PGO done in %.1fs — %d keyframes, %d loop closures", time.perf_counter() - t0, n_kf, n_lc)
    if n_lc == 0:
        logger.warning(
            "No loop closures detected — output will have the same drift as the raw traversal."
        )
    return graph


def _accumulate_corrected(stream, graph, voxel: float) -> o3d.geometry.PointCloud:
    logger.info("Accumulating corrected map…")
    t0 = time.perf_counter()

    pts_list: list[np.ndarray] = []
    cols_list: list[np.ndarray] = []
    n_frames = 0

    for obs in stream:
        if obs.pose_tuple is None or len(obs.data) == 0:
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
        logger.error("No posed frames found — was CameraNavRecorder used?")
        return o3d.geometry.PointCloud()

    merged = o3d.geometry.PointCloud()
    merged.points = o3d.utility.Vector3dVector(np.concatenate(pts_list))
    if len(cols_list) == len(pts_list):
        merged.colors = o3d.utility.Vector3dVector(np.concatenate(cols_list))

    downsampled = merged.voxel_down_sample(voxel)
    logger.info(
        "Done in %.1fs — %d frames → %d points",
        time.perf_counter() - t0,
        n_frames,
        len(downsampled.points),
    )
    return downsampled


def _save(pcd: o3d.geometry.PointCloud, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(output), pcd)
    logger.info("Saved corrected map → %s", output)


def _visualize(raw_stream, graph, corrected: o3d.geometry.PointCloud) -> None:
    import rerun as rr

    rr.init("camera_nav_correct_map")
    server_uri = rr.serve_grpc()
    rr.serve_web_viewer(connect_to=server_uri, open_browser=True)

    raw_pts_list: list[np.ndarray] = []
    for obs in raw_stream:
        if len(obs.data) == 0:
            continue
        pts, _ = obs.data.as_numpy()
        if len(pts) > 0:
            raw_pts_list.append(pts.astype(np.float32))
    if raw_pts_list:
        rr.log("/raw_map", rr.Points3D(positions=np.concatenate(raw_pts_list), colors=[180, 180, 180]), static=True)

    pts_corr = np.asarray(corrected.points, dtype=np.float32)
    cols_corr = (
        (np.asarray(corrected.colors, dtype=np.float32) * 255).astype(np.uint8)
        if corrected.has_colors()
        else None
    )
    rr.log(
        "/corrected_map",
        rr.Points3D(positions=pts_corr, colors=cols_corr) if cols_corr is not None else rr.Points3D(positions=pts_corr),
        static=True,
    )

    if graph.keyframes:
        traj = [(kf.optimized.translation.x, kf.optimized.translation.y, kf.optimized.translation.z) for kf in graph.keyframes]
        rr.log("/pgo_path", rr.LineStrips3D(strips=[traj], colors=[[255, 255, 0]], radii=[0.02]), static=True)

        loop_pairs = [
            [(lc.source.translation.x, lc.source.translation.y, lc.source.translation.z),
             (lc.target.translation.x, lc.target.translation.y, lc.target.translation.z)]
            for lc in graph.loops
        ]
        if loop_pairs:
            rr.log("/loop_closures", rr.LineStrips3D(strips=loop_pairs, colors=[[255, 0, 128]], radii=[0.015]), static=True)

    logger.info("Rerun viewer opened — press Ctrl+C to exit")
    try:
        import time as _time
        while True:
            _time.sleep(1.0)
    except KeyboardInterrupt:
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description="Post-traversal PGO map correction")
    parser.add_argument("db", type=Path, help="SQLite recording from CameraNavRecorder")
    parser.add_argument("--output", "-o", type=Path, default=None)
    parser.add_argument("--stream", default="frame_cloud")
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--rerun", action="store_true")
    args = parser.parse_args()

    if not args.db.exists():
        raise FileNotFoundError(f"Database not found: {args.db}")

    output: Path = args.output or args.db.with_name(f"{args.db.stem}_corrected.pcd")

    store, stream, _ = _load_stream(args.db, args.stream)
    n_total = stream.count()
    logger.info("Loaded %d observations from '%s' in %s", n_total, args.stream, args.db)

    if n_total == 0:
        logger.error("Stream '%s' is empty", args.stream)
        return

    graph = _run_pgo(stream)
    corrected = _accumulate_corrected(stream, graph, voxel=args.voxel)

    if len(corrected.points) == 0:
        logger.error("Corrected map is empty — no posed frames found")
        return

    _save(corrected, output)

    if args.rerun:
        _visualize(stream, graph, corrected)

    store.stop()


if __name__ == "__main__":
    main()
