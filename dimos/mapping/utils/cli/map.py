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

from __future__ import annotations

from collections.abc import Callable, Iterable
import math
from pathlib import Path
import subprocess
from typing import TYPE_CHECKING, Any

import rerun as rr
import rerun.blueprint as rrb
import typer

# Heavy dimos imports (mapping/memory2 → torch, transformers, open3d, sklearn) are
# deferred into the function bodies below so that `dimos --help` — which imports this
# module just to register the `map` subcommand — stays fast. See test_cli_startup.py.
if TYPE_CHECKING:
    from dimos.mapping.loop_closure.pgo import PoseGraph
    from dimos.memory2.stream import Stream
    from dimos.memory2.type.observation import Observation
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

PATH_THICKNESS = 0.01
# Pin pattern (from dimos/memory2/vis/space/rerun.py): thin vertical line
# from each marker with the label floating at the top so multi-marker
# labels never overlap the boxes.
MARKER_STEM = 1.0


def _log_markers(
    prefix: str,
    centers: list[tuple[float, float, float]],
    quats: list[tuple[float, float, float, float]],
    *,
    fill_half: list[tuple[float, float, float]],
    outline_half: list[tuple[float, float, float]],
    colors: list[tuple[int, int, int]],
    labels: list[str],
) -> None:
    """Render per-marker fill + outline + pin-stem + label as four static entities."""
    n = len(centers)
    pin_strips = [[(cx, cy, cz), (cx, cy, cz + MARKER_STEM)] for (cx, cy, cz) in centers]
    label_positions = [(cx, cy, cz + MARKER_STEM + 0.01) for (cx, cy, cz) in centers]
    rr.log(
        f"{prefix}/fill",
        rr.Boxes3D(
            centers=centers,
            half_sizes=fill_half,
            quaternions=quats,
            colors=colors,
            fill_mode=rr.components.FillMode.Solid,
        ),
        static=True,
    )
    rr.log(
        f"{prefix}/outline",
        rr.Boxes3D(
            centers=centers,
            half_sizes=outline_half,
            quaternions=quats,
            colors=[(255, 255, 255)] * n,
            fill_mode=rr.components.FillMode.MajorWireframe,
            radii=0.002,
        ),
        static=True,
    )
    rr.log(
        f"{prefix}/pin",
        rr.LineStrips3D(strips=pin_strips, colors=colors, radii=[0.005]),
        static=True,
    )
    rr.log(
        f"{prefix}/label",
        rr.Points3D(positions=label_positions, labels=labels, colors=colors, radii=[0.001] * n),
        static=True,
    )


def _accumulate(
    obs_iter: Iterable[Observation[PointCloud2]],
    *,
    voxel: float,
    block_count: int,
    device: str,
    graph: PoseGraph | None = None,
    progress_cb: Callable[[Observation[Any]], None] | None = None,
) -> PointCloud2 | None:
    """Accumulate a voxel map from `obs_iter`, optionally PGO-correcting each frame.

    Returns the final ``PointCloud2`` (or ``None`` if the input was empty).
    Disposal of the underlying ``VoxelGrid`` is handled by ``VoxelMapTransformer``.
    """
    from dimos.mapping.voxels import VoxelMapTransformer

    def prepared() -> Iterable[Observation[PointCloud2]]:
        for obs in obs_iter:
            if progress_cb is not None:
                progress_cb(obs)
            if len(obs.data) == 0:
                continue
            if graph is not None:
                if obs.pose_tuple is None:
                    continue
                yield obs.derive(data=obs.data.transform(graph.correction_at(obs.ts)))
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


def _log_reconstruction(
    *,
    voxel: float,
    global_map: PointCloud2 | None,
    path: list[tuple[float, float, float]],
    pgo_map: PointCloud2 | None,
    full_pgo_map: PointCloud2 | None,
    pgo_path: list[tuple[float, float, float]],
    graph: PoseGraph | None,
    marker_dets: list[Observation[Any]],
    marker_size: float,
    bottom_cutoff: float | None = None,
) -> None:
    """Log maps, paths, the PGO graph, and markers to the active rerun recording."""
    from dimos.memory2.vis.color import Color
    from dimos.msgs.geometry_msgs.Transform import Transform

    rr.send_blueprint(rrb.Blueprint(rrb.Spatial3DView(origin="world")))
    if global_map is not None:
        rr.log(
            "world/raw_map/pointcloud",
            global_map.to_rerun(voxel_size=voxel / 2, bottom_cutoff=bottom_cutoff),
            static=True,
        )
    if path:
        rr.log(
            "world/raw_map/path",
            rr.LineStrips3D(strips=[path], colors=[[231, 76, 60]], radii=[PATH_THICKNESS]),
            static=True,
        )
    if pgo_map is not None:
        rr.log(
            "world/pgo_map/pointcloud",
            pgo_map.to_rerun(voxel_size=voxel / 2, bottom_cutoff=bottom_cutoff),
            static=True,
        )
    if full_pgo_map is not None:
        rr.log(
            "world/full_pgo_map/pointcloud",
            full_pgo_map.to_rerun(voxel_size=voxel / 2, bottom_cutoff=bottom_cutoff),
            static=True,
        )
    if pgo_path:
        rr.log(
            "world/pgo_map/path",
            rr.LineStrips3D(strips=[pgo_path], colors=[[255, 255, 255]], radii=[PATH_THICKNESS]),
            static=True,
        )
        rr.log(
            "world/pgo_map/pgo/keyframes",
            rr.Points3D(positions=pgo_path, colors=[[255, 0, 0]], radii=[0.025]),
            static=True,
        )
    if graph is not None and graph.loops:
        loop_strips = [
            [
                (lc.source.translation.x, lc.source.translation.y, lc.source.translation.z),
                (lc.target.translation.x, lc.target.translation.y, lc.target.translation.z),
            ]
            for lc in graph.loops
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
        raw_centers = [(d.data.center.x, d.data.center.y, d.data.center.z) for d in marker_dets]
        raw_quats = [
            (d.data.orientation.x, d.data.orientation.y, d.data.orientation.z, d.data.orientation.w)
            for d in marker_dets
        ]
        # One entry per tracked marker session — color stable per track_id.
        colors = [
            Color.from_cmap("tab10", (d.data.track_id % 10) / 10.0).rgb_u8() for d in marker_dets
        ]
        labels = [f"track={d.data.track_id} id={d.data.marker_id}" for d in marker_dets]

        _log_markers(
            "world/raw_map/markers",
            raw_centers,
            raw_quats,
            fill_half=fill_half,
            outline_half=outline_half,
            colors=colors,
            labels=labels,
        )

        if graph is not None:
            # PGO-correct each raw marker pose: lift it from world_raw into
            # world_corrected so it lines up with pgo_map.
            pgo_centers: list[tuple[float, float, float]] = []
            pgo_quats: list[tuple[float, float, float, float]] = []
            for d in marker_dets:
                raw_tf = Transform(
                    translation=d.data.center,
                    rotation=d.data.orientation,
                    frame_id="world",
                    child_frame_id=f"marker_{d.data.marker_id}",
                    ts=d.ts,
                )
                corrected = graph.correct(raw_tf)
                pgo_centers.append(
                    (corrected.translation.x, corrected.translation.y, corrected.translation.z)
                )
                pgo_quats.append(
                    (
                        corrected.rotation.x,
                        corrected.rotation.y,
                        corrected.rotation.z,
                        corrected.rotation.w,
                    )
                )
            _log_markers(
                "world/pgo_map/markers",
                pgo_centers,
                pgo_quats,
                fill_half=fill_half,
                outline_half=outline_half,
                colors=colors,
                labels=labels,
            )


def main(
    dataset: str = typer.Argument(..., help="Dataset .db: bare name (cwd or data/) or path"),
    lidar_stream: str = typer.Option(
        "lidar", "--lidar", help="Lidar point-cloud stream to reconstruct"
    ),
    seek: float = typer.Option(0.0, "--seek", help="Skip the first N seconds of the recording"),
    duration: float | None = typer.Option(
        None, "--duration", help="Use only N seconds from --seek (default: to the end)"
    ),
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
        help="Spatial dedup tolerance (meters); applies to both raw and --pgo maps. 0 disables dedup (keep every posed frame)",
    ),
    block_count: int = typer.Option(
        2_000_000, "--block-count", help="VoxelBlockGrid capacity (raw and PGO rebuilds)"
    ),
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
    out: Path | None = typer.Option(
        None, "--out", help="Output .rrd path (default: ./<dataset>.rrd)"
    ),
    no_gui: bool = typer.Option(False, "--no-gui", help="Write the .rrd but don't launch rerun"),
    markers: bool = typer.Option(
        False,
        "--markers",
        help="Detect AprilTag markers in color_image and overlay them in rerun",
    ),
    camera_info: Path | None = typer.Option(
        None,
        "--camera-info",
        help="YAML calibration file for --markers; defaults to Go2 builtin",
    ),
    image_pose: str | None = typer.Option(
        None,
        "--image-pose",
        help="Re-pose color_image from this stream's pose (composed with the camera "
        "optical mount) before marker detection, instead of the image's stored pose",
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
    bottom_cutoff: float | None = typer.Option(
        None,
        "--bottom-cutoff",
        help="Drop global-map points below this Z (m) when rendering; e.g. 0 strips the floor",
    ),
) -> None:
    """Rebuild a voxel map from a recorded SQLite dataset, write a .rrd, and open it in rerun."""
    from dimos.mapping.loop_closure.pgo import PGO
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.memory2.transform import QualityWindow, SpeedLimit
    from dimos.memory2.utils.progress import progress
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.perception.fiducial.marker_transformer import DetectMarkers
    from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL, _camera_info_static
    from dimos.utils.data import resolve_named_path
    from dimos.visualization.rerun.init import rerun_init

    db_path = resolve_named_path(dataset, ".db")
    if out is None:
        out = Path.cwd() / f"{db_path.stem}.rrd"
    if export or full_pgo:
        pgo = True

    store = SqliteStore(path=db_path)
    lidar = store.stream(lidar_stream, PointCloud2).from_time(seek or None).to_time(duration)

    print(lidar.summary())

    total = lidar.count()

    # Spatial dedup: bucket frames by 3D cell using the raw pose, keep the
    # latest per cell. Shared by raw and PGO rebuilds. Doesn't touch obs.data
    # so it stays cheap (no pointcloud loading). With pgo_tol<=0 the bucketing
    # is disabled and every posed frame is kept (keyed by index).
    seen: dict[Any, Observation[Any]] = {}
    for i, obs in enumerate(lidar):
        pose = obs.pose
        if pose is None:
            continue
        # Reject placeholder poses: zero translation OR uninitialized rotation.
        # Same condition as pgo_keyframes so dedup and PGO see the same frames.
        if pose.position.is_zero() or pose.orientation.is_zero():
            continue
        if pgo_tol > 0:
            t = pose.position
            # math.floor so negative coords bucket consistently; int() truncates
            # toward zero and silently folds -0.5 and 0.5 into the same cell.
            key: Any = (
                math.floor(t.x / pgo_tol),
                math.floor(t.y / pgo_tol),
                math.floor(t.z / pgo_tol),
            )
        else:
            key = i
        seen[key] = obs

    n_kept = len(seen)
    pct = 100 * n_kept / total if total else 0
    if pgo_tol > 0:
        print(f"dedup: kept [{n_kept}/{total}] frames ({pct:.1f}%) at tol={pgo_tol}m")
    else:
        print(f"dedup: disabled, kept all [{n_kept}/{total}] posed frames")

    # Dict insertion order = lidar iteration order = chronological.
    # `seen` only contains entries with non-None poses (filtered above).
    path: list[tuple[float, float, float]] = [
        (p[0], p[1], p[2]) for obs in seen.values() if (p := obs.pose_tuple) is not None
    ]

    pgo_map = None
    pgo_path: list[tuple[float, float, float]] = []
    graph: PoseGraph | None = None
    if pgo:
        print("running PGO twopass map...")
        prog = progress(total, "pgo pass 1 (optimizing)")
        graph = lidar.tap(prog).transform(PGO()).last().data

        pgo_path = [
            (kf.optimized.translation.x, kf.optimized.translation.y, kf.optimized.translation.z)
            for kf in graph.keyframes
        ]

        pgo_map = _accumulate(
            seen.values(),
            voxel=voxel,
            block_count=block_count,
            device=device,
            graph=graph,
            progress_cb=progress(n_kept, "pgo pass 2 (rebuilding)"),
        )

    full_pgo_map = None
    if full_pgo:
        assert graph is not None
        full_pgo_map = _accumulate(
            lidar,
            voxel=voxel,
            block_count=block_count,
            device=device,
            graph=graph,
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
        # (verified: matches lidar_base_pose + BASE_TO_OPTICAL to ~1mm). With
        # --image-pose, swap that stored pose for a different source (e.g.
        # fastlio_odometry), composing the base→optical mount onto it first.
        color_image = store.stream("color_image", Image).from_time(seek or None).to_time(duration)
        n_images = color_image.count()
        if image_pose is not None:
            from dimos.mapping.utils.cli.pose_fill import pose_fill

            src_pose: Stream[Any] = (
                store.stream(image_pose).from_time(seek or None).to_time(duration)
            )
            print(f"re-posing color_image from {image_pose!r} + camera optical mount")
            color_image = pose_fill(color_image, src_pose, tolerance=0.1, mount=BASE_TO_OPTICAL)
        cam_info = CameraInfo.from_yaml(str(camera_info)) if camera_info else _camera_info_static()
        xf = DetectMarkers(
            camera_info=cam_info,
            marker_length_m=marker_size,
            smoothing_window=marker_smoothing,
        )
        # Keep the sharpest frame per --marker-quality-window window, then
        # drop frames where the robot was moving (linear + rotational) faster
        # than the limits. Defaults match replay_marker.py so positions agree.
        pipeline: Stream[Image] = color_image.tap(
            progress(n_images, "detecting markers")
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

    rerun_init("dimos map tool")
    rr.save(str(out))
    _log_reconstruction(
        voxel=voxel,
        global_map=global_map,
        path=path,
        pgo_map=pgo_map,
        full_pgo_map=full_pgo_map,
        pgo_path=pgo_path,
        graph=graph,
        marker_dets=marker_dets,
        marker_size=marker_size,
        bottom_cutoff=bottom_cutoff,
    )
    print(f"wrote {out}")
    if no_gui:
        print(f"open with: rerun {out}")
    else:
        subprocess.Popen(["rerun", str(out)])

    if export and pgo_map is not None:
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
