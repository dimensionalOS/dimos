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

from collections.abc import Callable, Iterable
import time
from typing import Any

import rerun as rr
import rerun.blueprint as rrb
import typer

from dimos.mapping.voxels import VoxelMapTransformer
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.memory2.transform import QualityWindow, SpeedLimit
from dimos.memory2.type.observation import Observation
from dimos.memory2.vis.color import Color
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.fiducial.marker_transformer import DetectMarkers
from dimos.robot.unitree.go2.connection import _camera_info_static
from dimos.utils.data import resolve_named_path
from dimos.visualization.rerun.init import rerun_init

PATH_THICKNESS = 0.01


def _accumulate(
    obs_iter: Iterable[Observation[PointCloud2]],
    *,
    voxel: float,
    block_count: int,
    device: str,
    interp: Callable[[float], Transform] | None = None,
    progress_cb: Callable[[Observation[Any]], None] | None = None,
) -> PointCloud2 | None:
    """Accumulate a voxel map from `obs_iter`, optionally PGO-correcting each frame.

    Returns the final ``PointCloud2`` (or ``None`` if the input was empty).
    Disposal of the underlying ``VoxelGrid`` is handled by ``VoxelMapTransformer``.
    """

    def prepared() -> Iterable[Observation[PointCloud2]]:
        for obs in obs_iter:
            if progress_cb is not None:
                progress_cb(obs)
            if len(obs.data) == 0:
                continue
            if interp is not None:
                if obs.pose_tuple is None:
                    continue
                yield obs.derive(data=obs.data.transform(interp(obs.ts)))
            else:
                yield obs

    vmt = VoxelMapTransformer(
        emit_every=0,  # batch mode: emit once on exhaustion
        voxel_size=voxel,
        block_count=block_count,
        device=device,
    )
    result = next(iter(vmt(iter(prepared()))), None)
    return result.data if result is not None else None


def progress(total: int, label: str = "") -> Callable[[Observation[Any]], None]:
    seen = 0
    wall_start: float | None = None
    last_wall: float | None = None
    first_ts: float | None = None

    def _progress(obs: Observation[Any]) -> None:
        nonlocal seen, wall_start, last_wall, first_ts
        now = time.monotonic()
        if wall_start is None:
            wall_start = now
            first_ts = obs.ts
        assert first_ts is not None  # narrowed by the same `if` above
        frame_ms = (now - last_wall) * 1000 if last_wall is not None else 0.0
        last_wall = now
        seen += 1
        pct = 100 * seen // total if total else 100
        wall = now - wall_start
        data = obs.ts - first_ts
        speed = data / wall if wall > 0 else 0.0
        end = "\n" if seen >= total else ""
        prefix = f"{label} " if label else ""
        print(
            f"\r{prefix}{pct:>3}% [{seen}/{total}] {data:.1f}s ({speed:.1f} x rt) {frame_ms:.0f}ms/frame",
            end=end,
            flush=True,
        )

    return _progress


def main(
    dataset: str = typer.Argument(..., help="Dataset .db: bare name (cwd or data/) or path"),
    voxel: float = typer.Option(0.05, "--voxel", help="Voxel size for the rebuild"),
    device: str = typer.Option(
        "CUDA:0", "--device", help="Open3D compute device (e.g. CUDA:0, CPU:0)"
    ),
    pgo: bool = typer.Option(
        False,
        "--pgo",
        help="Run pose graph optimization and rebuild from spatially-deduped frames",
    ),
    pgo_tol: float = typer.Option(
        0.3,
        "--pgo-tol",
        help="Spatial dedup tolerance (meters); applies to both raw and --pgo maps",
    ),
    block_count: int = typer.Option(2_000_000, "--block-count", help="VoxelBlockGrid capacity"),
    export: bool = typer.Option(
        False,
        "--export",
        help="Export PGO map to ./<dataset>.pc2.lcm in cwd (implies --pgo)",
    ),
    full_pgo: bool = typer.Option(
        False,
        "--full-pgo",
        help="Also build a full-replay PGO map (every frame) for comparison (implies --pgo)",
    ),
    no_gui: bool = typer.Option(False, "--no-gui", help="Skip rerun visualization"),
    markers: bool = typer.Option(
        False,
        "--markers",
        help="Detect AprilTag markers in color_image and overlay them in rerun",
    ),
    marker_size: float = typer.Option(
        0.1, "--marker-size", help="Physical marker edge length in meters (--markers only)"
    ),
    marker_max_speed: float = typer.Option(
        0.5,
        "--marker-max-speed",
        help="Skip frames where robot is moving faster than this (m/s); 0 disables",
    ),
    marker_max_rot_rate: float = typer.Option(
        50.0,
        "--marker-max-rot-rate",
        help="Skip frames where robot is rotating faster than this (deg/s); 0 disables",
    ),
    marker_quality_window: float = typer.Option(
        0.1,
        "--marker-quality-window",
        help="Sharpest-frame window for marker detection (s)",
    ),
    marker_smoothing: float = typer.Option(
        7.5,
        "--marker-smoothing",
        help="Sliding-window track buffer for marker pose averaging (s); 0 disables (one box per raw detection)",
    ),
) -> None:
    db_path = resolve_named_path(dataset, ".db")
    if export or full_pgo:
        pgo = True

    store = SqliteStore(path=db_path)
    lidar = store.streams.lidar

    print(lidar.summary())

    total = lidar.count()

    # Spatial dedup: bucket frames by 3D cell using the raw pose, keep the
    # latest per cell. Shared by raw and PGO rebuilds. Doesn't touch obs.data
    # so it stays cheap (no pointcloud loading).
    seen: dict[tuple[int, int, int], Observation[Any]] = {}
    for obs in lidar:
        p = obs.pose_tuple
        if p is None:
            continue
        # Reject placeholder poses at the world origin.
        if p[0] == 0 and p[1] == 0 and p[2] == 0:
            continue
        cell = (int(p[0] / pgo_tol), int(p[1] / pgo_tol), int(p[2] / pgo_tol))
        seen[cell] = obs

    n_kept = len(seen)
    pct = 100 * n_kept / total if total else 0
    print(f"dedup: kept [{n_kept}/{total}] frames ({pct:.1f}%) at tol={pgo_tol}m")

    # Dict insertion order = lidar iteration order = chronological.
    # `seen` only contains entries with non-None poses (filtered above).
    path: list[tuple[float, float, float]] = [
        (p[0], p[1], p[2]) for obs in seen.values() if (p := obs.pose_tuple) is not None
    ]

    pgo_map = None
    pgo_path: list[tuple[float, float, float]] = []
    loops: list[Any] = []
    interp: Any | None = None
    if pgo:
        from dimos.mapping.loop_closure.pgo import (
            LoopClosure,
            keyframes_to_corrections,
            make_interpolator,
            pgo_keyframes,
        )

        print("running PGO twopass map...")
        pgo_loops: list[LoopClosure] = []
        keyframes = pgo_keyframes(
            lidar,
            on_frame=progress(total, "pgo pass 1 (optimizing)"),
            loop_closures_out=pgo_loops,
        )
        loops = list(pgo_loops)
        corrections = keyframes_to_corrections(keyframes)
        interp = make_interpolator(corrections)

        for kf_obs in keyframes:
            kf_t = kf_obs.data.optimized.translation
            pgo_path.append((kf_t.x, kf_t.y, kf_t.z))

        pgo_map = _accumulate(
            seen.values(),
            voxel=voxel,
            block_count=block_count,
            device=device,
            interp=interp,
            progress_cb=progress(n_kept, "pgo pass 2 (rebuilding)"),
        )

    full_pgo_map = None
    if full_pgo:
        assert interp is not None
        full_pgo_map = _accumulate(
            lidar,
            voxel=voxel,
            block_count=block_count,
            device=device,
            interp=interp,
            progress_cb=progress(total, "full pgo (rebuilding)"),
        )

    # Raw map: same dedup'd frames, no PGO correction.
    global_map = _accumulate(
        seen.values(),
        voxel=voxel,
        block_count=block_count,
        device=device,
        progress_cb=progress(n_kept, "reconstructing global map"),
    )

    marker_dets: list[Observation[Any]] = []
    if markers:
        # Image observations in dimos recordings are stamped with
        # frame_id="camera_optical", so obs.pose is already optical-in-world
        # (verified: matches lidar_base_pose + BASE_TO_OPTICAL to ~1mm).
        # No mount composition needed.
        color_image = store.stream("color_image", Image)
        xf = DetectMarkers(
            camera_info=_camera_info_static(),
            marker_length_m=marker_size,
            smoothing_window=marker_smoothing,
        )
        # Keep the sharpest frame per --marker-quality-window window, then
        # drop frames where the robot was moving (linear + rotational) faster
        # than the limits. Defaults match markers_rrd.py so positions agree.
        pipeline: Stream[Image] = color_image.tap(
            progress(color_image.count(), "detecting markers")
        ).transform(QualityWindow(lambda img: img.sharpness, window=marker_quality_window))
        if marker_max_speed > 0:
            pipeline = pipeline.transform(
                SpeedLimit(
                    max_mps=marker_max_speed,
                    max_dps=marker_max_rot_rate if marker_max_rot_rate > 0 else None,
                )
            )
        all_dets = pipeline.transform(xf).to_list()
        if marker_smoothing > 0:
            # Keep only the latest emission per track_id — that's the most
            # averaged pose, drawn once per tracked marker session.
            by_track: dict[int, Observation[Any]] = {}
            for d in all_dets:
                by_track[d.data.track_id] = d
            marker_dets = list(by_track.values())
        else:
            marker_dets = all_dets
        unique_ids = sorted({obs.data.marker_id for obs in marker_dets})
        print(
            f"markers: {len(marker_dets)} entries from {len(all_dets)} raw detections "
            f"across {len(unique_ids)} unique ids {unique_ids}"
        )

    if not no_gui:
        rerun_init("dimos map tool", spawn=True)
        rr.send_blueprint(rrb.Blueprint(rrb.Spatial3DView(origin="world")))
        if global_map is not None:
            rr.log(
                "world/raw_map/pointcloud", global_map.to_rerun(voxel_size=voxel / 2), static=True
            )
        if path:
            rr.log(
                "world/raw_map/path",
                rr.LineStrips3D(strips=[path], colors=[[231, 76, 60]], radii=[PATH_THICKNESS]),
                static=True,
            )
        if pgo_map is not None:
            rr.log("world/pgo_map/pointcloud", pgo_map.to_rerun(voxel_size=voxel / 2), static=True)
        if full_pgo_map is not None:
            rr.log(
                "world/full_pgo_map/pointcloud",
                full_pgo_map.to_rerun(voxel_size=voxel / 2),
                static=True,
            )
        STEM_HEIGHT = 0  # lift pose-graph viz above the map for legibility
        if pgo_path:
            rr.log(
                "world/pgo_map/path",
                rr.LineStrips3D(
                    strips=[pgo_path], colors=[[255, 255, 255]], radii=[PATH_THICKNESS]
                ),
                static=True,
            )
            hovered = [(x, y, z + STEM_HEIGHT) for (x, y, z) in pgo_path]
            rr.log(
                "world/pgo_map/pgo/keyframes",
                rr.Points3D(positions=hovered, colors=[[255, 0, 0]], radii=[0.025]),
                static=True,
            )
        if pgo and loops:
            loop_strips = [
                [
                    (
                        lc.source.translation.x,
                        lc.source.translation.y,
                        lc.source.translation.z + STEM_HEIGHT,
                    ),
                    (
                        lc.target.translation.x,
                        lc.target.translation.y,
                        lc.target.translation.z + STEM_HEIGHT,
                    ),
                ]
                for lc in loops
            ]
            rr.log(
                "world/pgo_map/pgo/loop_closures",
                rr.LineStrips3D(strips=loop_strips, colors=[[231, 76, 60]], radii=[0.025]),
                static=True,
            )
        if marker_dets:
            half = marker_size / 2.0
            n = len(marker_dets)
            fill_half = [(half, half, 0.005)] * n
            # Outline sits just outside the fill so both stay visible.
            outline_bump = marker_size * 0.05
            outline_half = [(half + outline_bump, half + outline_bump, 0.006)] * n
            centers = [(d.data.center.x, d.data.center.y, d.data.center.z) for d in marker_dets]
            quaternions = [
                (
                    d.data.orientation.x,
                    d.data.orientation.y,
                    d.data.orientation.z,
                    d.data.orientation.w,
                )
                for d in marker_dets
            ]
            # One entry per tracked marker session — color stable per track_id.
            colors = [
                Color.from_cmap("tab10", (d.data.track_id % 10) / 10.0).rgb_u8()
                for d in marker_dets
            ]
            labels = [f"track={d.data.track_id} id={d.data.marker_id}" for d in marker_dets]
            # Pin pattern (from dimos/memory2/vis/space/rerun.py): thin
            # vertical line from each marker with the label floating at the
            # top so multi-marker labels never overlap the boxes.
            MARKER_STEM = 1.0
            pin_strips = [[(cx, cy, cz), (cx, cy, cz + MARKER_STEM)] for (cx, cy, cz) in centers]
            label_positions = [(cx, cy, cz + MARKER_STEM + 0.01) for (cx, cy, cz) in centers]
            rr.log(
                "world/raw_map/markers/fill",
                rr.Boxes3D(
                    centers=centers,
                    half_sizes=fill_half,
                    quaternions=quaternions,
                    colors=colors,
                    fill_mode=rr.components.FillMode.Solid,
                ),
                static=True,
            )
            rr.log(
                "world/raw_map/markers/outline",
                rr.Boxes3D(
                    centers=centers,
                    half_sizes=outline_half,
                    quaternions=quaternions,
                    colors=[(255, 255, 255)] * n,
                    fill_mode=rr.components.FillMode.MajorWireframe,
                    radii=0.002,
                ),
                static=True,
            )
            rr.log(
                "world/raw_map/markers/pin",
                rr.LineStrips3D(strips=pin_strips, colors=colors, radii=[0.005]),
                static=True,
            )
            rr.log(
                "world/raw_map/markers/label",
                rr.Points3D(
                    positions=label_positions, labels=labels, colors=colors, radii=[0.001] * n
                ),
                static=True,
            )

            if interp is not None:
                # PGO-corrected marker poses. interp(ts) maps raw_world →
                # pgo_world; composing with each raw marker transform lifts
                # it into the corrected frame so it lines up with pgo_map.
                pgo_centers: list[tuple[float, float, float]] = []
                pgo_quaternions: list[tuple[float, float, float, float]] = []
                for d in marker_dets:
                    raw_tf = Transform(
                        translation=d.data.center,
                        rotation=d.data.orientation,
                        frame_id="world",
                        child_frame_id=f"marker_{d.data.marker_id}",
                        ts=d.ts,
                    )
                    corrected = interp(d.ts) + raw_tf
                    pgo_centers.append(
                        (
                            corrected.translation.x,
                            corrected.translation.y,
                            corrected.translation.z,
                        )
                    )
                    pgo_quaternions.append(
                        (
                            corrected.rotation.x,
                            corrected.rotation.y,
                            corrected.rotation.z,
                            corrected.rotation.w,
                        )
                    )
                pgo_pin_strips = [
                    [(cx, cy, cz), (cx, cy, cz + MARKER_STEM)] for (cx, cy, cz) in pgo_centers
                ]
                pgo_label_positions = [
                    (cx, cy, cz + MARKER_STEM + 0.01) for (cx, cy, cz) in pgo_centers
                ]
                rr.log(
                    "world/pgo_map/markers/fill",
                    rr.Boxes3D(
                        centers=pgo_centers,
                        half_sizes=fill_half,
                        quaternions=pgo_quaternions,
                        colors=colors,
                        fill_mode=rr.components.FillMode.Solid,
                    ),
                    static=True,
                )
                rr.log(
                    "world/pgo_map/markers/outline",
                    rr.Boxes3D(
                        centers=pgo_centers,
                        half_sizes=outline_half,
                        quaternions=pgo_quaternions,
                        colors=[(255, 255, 255)] * n,
                        fill_mode=rr.components.FillMode.MajorWireframe,
                        radii=0.002,
                    ),
                    static=True,
                )
                rr.log(
                    "world/pgo_map/markers/pin",
                    rr.LineStrips3D(strips=pgo_pin_strips, colors=colors, radii=[0.005]),
                    static=True,
                )
                rr.log(
                    "world/pgo_map/markers/label",
                    rr.Points3D(
                        positions=pgo_label_positions,
                        labels=labels,
                        colors=colors,
                        radii=[0.001] * n,
                    ),
                    static=True,
                )

    if export and pgo_map is not None:
        from pathlib import Path

        out_path = Path.cwd() / f"{db_path.stem}.pc2.lcm"
        print(f"exporting PGO twopass map to {out_path}...")
        out_path.write_bytes(pgo_map.lcm_encode())
        print(f"wrote {out_path}")
        print()
        print("load back with:")
        print("    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2")
        print(f'    pcd = PointCloud2.lcm_decode(open("{out_path.name}", "rb").read())')


if __name__ == "__main__":
    typer.run(main)
