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

from dimos_generated.builtin_interfaces.msg import Time
from dimos_message_build.registry import encode as cdr_encode
import typer

from dimos.mapping.cli.streams import select_stream

if TYPE_CHECKING:
    from dimos_generated.geometry_msgs.msg import TransformStamped
    from dimos_generated.sensor_msgs.msg import Image, PointCloud2

    from dimos.mapping.loop_closure.pgo import PoseGraph
    from dimos.memory.stream import Stream
    from dimos.memory.type.observation import Observation

PATH_THICKNESS = 0.01
# Pin pattern (from dimos/memory/vis/space/rerun.py): thin vertical line
# from each marker with the label floating at the top so multi-marker
# labels never overlap the boxes.
MARKER_STEM = 1.0

# Conventional world frames tried in order when --frame isn't given.
_WORLD_FRAMES = ("world", "map", "odom")


def _detect_world(tf_buf: Any, cloud_frame: str, ts: float) -> str | None:
    """Pick the first conventional world frame that resolves the cloud frame via tf."""
    if cloud_frame in _WORLD_FRAMES:
        return cloud_frame
    if tf_buf is not None:
        for cand in _WORLD_FRAMES:
            if tf_buf.get(cand, cloud_frame, time_point=ts) is not None:
                return cand
    return None


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
    import rerun as rr

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
    register: Callable[[Observation[Any]], TransformStamped | None] | None = None,
    carve_columns: bool = False,
    progress_cb: Callable[[Observation[Any]], None] | None = None,
) -> PointCloud2 | None:
    """Accumulate a voxel map from `obs_iter`, optionally PGO-correcting each frame.

    ``register`` maps each observation to the transform lifting its cloud into
    the world frame; ``None`` means no transform is available and the frame is
    skipped. With ``register=None`` all clouds are assumed world-registered.

    Returns the final ``PointCloud2`` (or ``None`` if the input was empty).
    Disposal of the underlying ``VoxelGrid`` is handled by ``VoxelMapTransformer``.
    """
    from dimos_generated.geometry_msgs.msg import TransformStamped
    from dimos_generated.std_msgs.msg import Header
    import numpy as np

    from dimos.mapping.voxels.module import VoxelMapTransformer
    from dimos.msgs.geometry import transform_from_matrix, transform_matrix
    from dimos.msgs.pointcloud import transform_cloud

    def prepared() -> Iterable[Observation[PointCloud2]]:
        for obs in obs_iter:
            if progress_cb is not None:
                progress_cb(obs)
            if obs.data.width * obs.data.height == 0:
                continue
            # sensor->world via `register`, unless the clouds are already
            # world-registered. graph adds the PGO correction on top
            # (correction ∘ tf), applied after the registration.
            tf: TransformStamped | None = None
            if register is not None:
                tf = register(obs)
                if tf is None:
                    continue
            if graph is not None:
                if obs.pose_tuple is None:
                    continue
                correction = transform_matrix(graph.correction_at(obs.ts).transform)
                matrix = np.eye(4) if tf is None else transform_matrix(tf.transform)
                tf = TransformStamped(
                    header=Header(stamp=obs.data.header.stamp, frame_id="world_corrected"),
                    child_frame_id=obs.data.header.frame_id,
                    transform=transform_from_matrix(correction @ matrix),
                )
            yield obs if tf is None else obs.derive(data=transform_cloud(obs.data, tf))

    vmt = VoxelMapTransformer(
        emit_every=0,  # batch mode: emit once on exhaustion
        voxel_size=voxel,
        block_count=block_count,
        device=device,
        carve_columns=carve_columns,
    )
    result = next(iter(vmt(iter(prepared()))), None)
    return result.data if result is not None else None


def _denoise(cloud: PointCloud2 | None) -> PointCloud2 | None:
    """Statistical outlier removal via o3d; drops sparse floaters, keeps colors."""
    if cloud is None or cloud.width * cloud.height < 20:
        return cloud
    import numpy as np
    import open3d as o3d

    from dimos.msgs.pointcloud import (
        pointcloud_from_xyz,
        pointcloud_from_xyz_rgb,
        pointcloud_to_open3d,
    )

    if {field.name for field in cloud.fields} - {"x", "y", "z", "rgb"}:
        raise ValueError("map denoising supports only XYZ/RGB point fields")
    tensor = o3d.t.geometry.PointCloud.from_legacy(pointcloud_to_open3d(cloud))
    clean, _ = tensor.remove_statistical_outliers(nb_neighbors=20, std_ratio=2.0)
    points = clean.point.positions.numpy()
    if "colors" in clean.point:
        colors = (np.clip(clean.point.colors.numpy(), 0.0, 1.0) * 255).astype(np.uint8)
        return pointcloud_from_xyz_rgb(points, colors, header=cloud.header)
    return pointcloud_from_xyz(points, header=cloud.header)


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
    from dimos_generated.geometry_msgs.msg import Point, Pose
    import rerun as rr
    import rerun.blueprint as rrb

    from dimos.memory.vis.color import Color
    from dimos.msgs.geometry import pose_from_matrix, pose_matrix, transform_matrix
    from dimos.visualization.rerun.message_helpers import cloud_archetype

    rr.send_blueprint(rrb.Blueprint(rrb.Spatial3DView(origin="world")))
    if global_map is not None:
        rr.log(
            "world/raw_map/pointcloud",
            cloud_archetype(global_map, ui_radius=voxel / 2, bottom_cutoff=bottom_cutoff),
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
            cloud_archetype(pgo_map, ui_radius=voxel / 2, bottom_cutoff=bottom_cutoff),
            static=True,
        )
    if full_pgo_map is not None:
        rr.log(
            "world/full_pgo_map/pointcloud",
            cloud_archetype(full_pgo_map, ui_radius=voxel / 2, bottom_cutoff=bottom_cutoff),
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
                (
                    lc.source.transform.translation.x,
                    lc.source.transform.translation.y,
                    lc.source.transform.translation.z,
                ),
                (
                    lc.target.transform.translation.x,
                    lc.target.transform.translation.y,
                    lc.target.transform.translation.z,
                ),
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
                center = d.data.center
                raw_pose = Pose(
                    position=Point(x=center.x, y=center.y, z=center.z),
                    orientation=d.data.orientation,
                )
                corrected = pose_from_matrix(
                    transform_matrix(graph.correction_at(d.ts).transform) @ pose_matrix(raw_pose)
                )
                pgo_centers.append(
                    (corrected.position.x, corrected.position.y, corrected.position.z)
                )
                pgo_quats.append(
                    (
                        corrected.orientation.x,
                        corrected.orientation.y,
                        corrected.orientation.z,
                        corrected.orientation.w,
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
    dataset: str = typer.Argument(..., help="Dataset .db or .mcap: bare name or path"),
    lidar_stream: str | None = typer.Option(
        None, "--lidar", help="PointCloud2 stream; auto-select only a unique candidate"
    ),
    image_stream: str | None = typer.Option(
        None, "--image", help="Image stream for --markers; auto-select only a unique candidate"
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
        0.0,
        "--pgo-tol",
        help="Spatial dedup tolerance (meters); applies to both raw and --pgo maps. 0 disables dedup (default; keep every registered frame)",
    ),
    block_count: int = typer.Option(
        2_000_000, "--block-count", help="VoxelBlockGrid capacity (raw and PGO rebuilds)"
    ),
    export: bool = typer.Option(
        False,
        "--export",
        help="Export PGO map to ./<dataset>.pc2.cdr in cwd (implies --pgo)",
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
    frame: str | None = typer.Option(
        None,
        "--frame",
        help="World frame to register clouds into. Default: auto-detect — the "
        "first of 'world', 'map', 'odom' that resolves the cloud frame via the "
        "dataset's tf stream. Clouds whose frame_id differs from it are "
        "registered via tf; clouds already in it pass through verbatim.",
    ),
    tf_tolerance: float | None = typer.Option(
        None,
        "--tf-tolerance",
        help="Max |Δts| (s) for tf lookups; default unlimited (nearest message), "
        "which also serves static/rarely-published transforms",
    ),
    carve: bool = typer.Option(
        False,
        "--carve/--no-carve",
        help="Column carving: keep only the latest frame's points per (X,Y) column. "
        "Off by default (full 3D accumulation); on collapses vertical structure "
        "(stairs, revisited columns) to the most recent observation.",
    ),
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
    marker_inverted: bool = typer.Option(
        False,
        "--marker-inverted",
        help="Also detect colour-inverted (light-on-dark) markers, e.g. a 3D print "
        "whose filaments were swapped",
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
    denoise: bool = typer.Option(
        False,
        "--denoise",
        help="Statistical outlier removal on the finished maps (o3d, nb_neighbors=20, "
        "std_ratio=2.0): drops sparse floaters before rendering/export",
    ),
) -> None:
    """Rebuild a voxel map from a recorded SQLite dataset, write a .rrd, and open it in rerun."""
    from dimos_generated.sensor_msgs.msg import Image, PointCloud2
    from dimos_generated.std_msgs.msg import Header
    import rerun as rr

    from dimos.mapping.loop_closure.pgo import PGO
    from dimos.memory.cli.dataset import open_store, resolve_dataset, stream_payload_types
    from dimos.memory.transform import QualityWindow, SpeedLimit
    from dimos.memory.utils.progress import progress
    from dimos.msgs.camera_info import camera_info_from_yaml
    from dimos.msgs.image import image_sharpness
    from dimos.perception.fiducial.marker_transformer import DetectMarkers
    from dimos.robot.unitree.go2.camera_calibration import front_camera_calibration
    from dimos.robot.unitree.go2.connection import BASE_TO_OPTICAL
    from dimos.visualization.rerun.init import rerun_init

    db_path = resolve_dataset(dataset)
    store = open_store(db_path)
    try:
        types = stream_payload_types(store)
        selected_lidar = select_stream(types, PointCloud2, lidar_stream, "--lidar")
        selected_image = (
            select_stream(types, Image, image_stream, "--image")
            if markers or image_stream is not None
            else None
        )
        assert selected_lidar is not None
    except Exception:
        store.stop()
        raise
    if out is None:
        out = Path.cwd() / f"{db_path.stem}.rrd"
    if export or full_pgo:
        pgo = True

    lidar = store.stream(selected_lidar, PointCloud2).from_time(seek or None).to_time(duration)

    print(lidar.summary())

    total = lidar.count()

    # Register clouds into the world frame via the dataset's tf stream. Clouds
    # already stamped with the world frame pass through verbatim; sensor-frame
    # clouds with no tf lookup are dropped. Stored per-frame poses are never
    # used for registration — only as trajectory metadata (dedup/path) when
    # the tf stream can't provide a position.
    from dimos.memory.tf import StreamTF

    tf_buf = StreamTF.from_store(store)
    # Streams are homogeneous: read the cloud frame from the first observation.
    first_obs = next(iter(lidar), None)
    cloud_frame: str | None = first_obs.data.header.frame_id if first_obs is not None else None

    world = frame
    if world is None and first_obs is not None and cloud_frame is not None:
        world = _detect_world(tf_buf, cloud_frame, first_obs.ts)
        if world is None:
            frames = tf_buf.get_frames() if tf_buf is not None else set()
            known = ", ".join(sorted(frames)) or "dataset has no tf stream"
            raise typer.BadParameter(
                f"none of {', '.join(_WORLD_FRAMES)} resolves {cloud_frame!r} clouds; "
                f"pass --frame (tf frames: {known})",
                param_hint="--frame",
            )
    if world is None:
        world = "world"  # empty lidar stream; the frame is moot

    # Registration: sensor-frame clouds get a per-frame tf lookup lifting them
    # into the world frame (frames with no tf answer are dropped); clouds
    # already stamped with the world frame accumulate verbatim (register=None).
    register: Callable[[Observation[Any]], TransformStamped | None] | None = None
    if first_obs is not None and cloud_frame is not None and cloud_frame != world:
        # Fail fast when registration is impossible: probe the first cloud's
        # timestamp (unbounded tolerance — "possible at all", not "in range").
        probe = (
            tf_buf.get(world, cloud_frame, time_point=first_obs.ts) if tf_buf is not None else None
        )
        if tf_buf is None or probe is None:
            frames = tf_buf.get_frames() if tf_buf is not None else set()
            known = ", ".join(sorted(frames)) or "dataset has no tf stream"
            raise typer.BadParameter(
                f"cannot register {cloud_frame!r} clouds into {world!r} (tf frames: {known})",
                param_hint="--frame",
            )
        print(f"registering clouds {world!r} ← {cloud_frame!r} via tf")
        buf = tf_buf

        def _register(obs: Observation[Any]) -> TransformStamped | None:
            return buf.get(
                world, obs.data.header.frame_id, time_point=obs.ts, time_tolerance=tf_tolerance
            )

        register = _register
    elif cloud_frame is not None:
        print(f"clouds already in world frame {world!r}; accumulating verbatim")

    def _position(obs: Observation[Any]) -> tuple[float, float, float] | None:
        """Trajectory position for dedup/path: registration tf, else the stored pose."""
        if register is not None:
            tf = register(obs)
            if tf is None:
                return None
            return (
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z,
            )
        pose = obs.pose
        # Reject placeholder poses: zero translation OR uninitialized rotation.
        # Same condition as pgo_keyframes so dedup and PGO see the same frames.
        if (
            pose is not None
            and any((pose.position.x, pose.position.y, pose.position.z))
            and any(
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            )
        ):
            return (pose.position.x, pose.position.y, pose.position.z)
        return None

    # World-frame accumulation needs no trajectory. Dedup and PGO do.
    seen: dict[Any, Observation[Any]] = {}
    path: list[tuple[float, float, float]] = []
    for i, obs in enumerate(lidar):
        pos = _position(obs)
        if register is not None and pos is None:
            raise typer.BadParameter(
                f"Missing registration TF at timestamp {obs.ts}", param_hint="--frame"
            )
        if pgo_tol > 0 and pos is None:
            raise typer.BadParameter(
                "Spatial dedup requires trajectory positions; use --pgo-tol 0 for plain accumulation",
                param_hint="--pgo-tol",
            )
        if pgo and (
            obs.pose is None
            or not any((obs.pose.position.x, obs.pose.position.y, obs.pose.position.z))
            or not any(
                (
                    obs.pose.orientation.x,
                    obs.pose.orientation.y,
                    obs.pose.orientation.z,
                    obs.pose.orientation.w,
                )
            )
        ):
            raise typer.BadParameter(
                "PGO requires valid stored trajectory poses; registration TF alone does not populate them",
                param_hint="--pgo",
            )
        key: Any = i
        if pgo_tol > 0:
            assert pos is not None
            key = tuple(math.floor(value / pgo_tol) for value in pos)
        seen[key] = obs

    kept = list(seen.values())
    path = [pos for obs in kept if (pos := _position(obs)) is not None]
    n_kept = len(kept)
    if not n_kept:
        raise typer.BadParameter("No point-cloud observations in the selected interval")
    print(f"dedup: kept [{n_kept}/{total}] frames at tol={pgo_tol}m")

    pgo_map = None
    pgo_path: list[tuple[float, float, float]] = []
    graph: PoseGraph | None = None
    if pgo:
        print("running PGO twopass map...")
        with progress(total, "pgo pass 1 (optimizing)") as bar:
            graph = lidar.tap(bar).transform(PGO()).last().data

        pgo_path = [
            (
                kf.optimized.transform.translation.x,
                kf.optimized.transform.translation.y,
                kf.optimized.transform.translation.z,
            )
            for kf in graph.keyframes
        ]

        with progress(n_kept, "pgo pass 2 (rebuilding)") as bar:
            pgo_map = _accumulate(
                kept,
                voxel=voxel,
                block_count=block_count,
                device=device,
                graph=graph,
                register=register,
                carve_columns=carve,
                progress_cb=bar,
            )

    full_pgo_map = None
    if full_pgo:
        assert graph is not None
        with progress(total, "full pgo (rebuilding)") as bar:
            full_pgo_map = _accumulate(
                lidar,
                voxel=voxel,
                block_count=block_count,
                device=device,
                graph=graph,
                register=register,
                carve_columns=carve,
                progress_cb=bar,
            )

    # Raw map: same dedup'd frames, no PGO correction.
    with progress(n_kept, "reconstructing global map") as bar:
        global_map = _accumulate(
            kept,
            voxel=voxel,
            block_count=block_count,
            device=device,
            register=register,
            carve_columns=carve,
            progress_cb=bar,
        )

    if global_map is None:
        raise typer.BadParameter("Selected observations contain no points to accumulate")

    if denoise:
        print("denoising maps (statistical outlier removal)...")
        global_map = _denoise(global_map)
        pgo_map = _denoise(pgo_map)
        full_pgo_map = _denoise(full_pgo_map)

    marker_dets: list[Observation[Any]] = []
    if markers:
        # Image observations in dimos recordings are stamped with
        # frame_id="camera_optical", so obs.pose is already optical-in-world
        # (verified: matches lidar_base_pose + BASE_TO_OPTICAL to ~1mm). With
        # --image-pose, swap that stored pose for a different source (e.g.
        # fastlio_odometry), composing the base→optical mount onto it first.
        assert selected_image is not None
        color_image = store.stream(selected_image, Image).from_time(seek or None).to_time(duration)
        n_images = color_image.count()
        if image_pose is not None:
            from dimos.mapping.cli.pose_fill import pose_fill

            src_pose: Stream[Any] = (
                store.stream(image_pose).from_time(seek or None).to_time(duration)
            )
            print(f"re-posing color_image from {image_pose!r} + camera optical mount")
            color_image = pose_fill(color_image, src_pose, tolerance=0.1, mount=BASE_TO_OPTICAL)
        cam_info = (
            camera_info_from_yaml(
                camera_info, header=Header(frame_id="camera_optical", stamp=Time(sec=0, nanosec=0))
            )
            if camera_info
            else front_camera_calibration()
        )
        xf = DetectMarkers(
            camera_info=cam_info,
            marker_length_m=marker_size,
            smoothing_window=marker_smoothing,
            detect_inverted=marker_inverted,
        )
        # Keep the sharpest frame per --marker-quality-window window, then
        # drop frames where the robot was moving (linear + rotational) faster
        # than the limits. Defaults match replay_marker.py so positions agree.
        with progress(n_images, "detecting markers") as bar:
            pipeline: Stream[Image] = color_image.tap(bar).transform(
                QualityWindow(image_sharpness, window=marker_quality_window)
            )
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
        out_path = Path.cwd() / f"{db_path.stem}.pc2.cdr"
        print(f"exporting PGO twopass map to {out_path}...")
        out_path.write_bytes(cdr_encode(pgo_map))
        print(f"wrote {out_path}")
        print()
        print("load back with:")
        print("    from dimos_generated.sensor_msgs.msg import PointCloud2")
        print("    from dimos_message_build.registry import decode")
        print(f'    pcd = decode(open("{out_path.name}", "rb").read(), PointCloud2)')


if __name__ == "__main__":
    typer.run(main)
