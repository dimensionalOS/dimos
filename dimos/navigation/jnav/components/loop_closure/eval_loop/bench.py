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

"""Benchmark Ivan's go2 PGO (dimos/navigation/go2/loop_closure/pgo.py) against gsc_pgo (pgo-eval).

Both get the exact same input: the recording's `lidar` stream (world-frame scans, each
stamped with its odom pose). Ivan's runs twice: at `PGO()` defaults (what his eval.py uses)
and at `LIVE_PGO` (what his PGOVoxelMapper uses). gsc runs at its defaults. Each yields a
keyframe table (ts, raw pose, optimized pose); the same slerp/lerp turns that into a
per-scan correction, and the same renderer draws the top-downs.

usage: bench.py <dataset> [<dataset> ...]
"""

import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from paths import OUT as HERE, PGO_EVAL, REPO, import_ivan
from recordings import BY_NAME
from scipy.spatial.transform import Rotation, Slerp

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

_ivan_pgo, _ivan_pgo_map = import_ivan()[1:]
PGO, LIVE_PGO = _ivan_pgo.PGO, _ivan_pgo_map.LIVE_PGO
IVAN_CONFIGS = {"ivan": {}, "ivan_live": LIVE_PGO.model_dump()}
# image poses must be in the same world as the scans, or the corrections don't apply to the markers
MAX_IMAGE_POSE_OFFSET_M = 0.5

DATA = REPO / "data"
RENDER_POINTS_PER_SCAN = 4000
VOXEL_M = 0.2


def load(db: Path):
    """Scans in time order: ts, (N,7) pose rows [x,y,z,qx,qy,qz,qw], world points (subsampled)."""
    rng = np.random.default_rng(0)
    times, poses, clouds = [], [], []
    with SqliteStore(path=str(db)) as store:
        per_scan = max(
            200, min(RENDER_POINTS_PER_SCAN, int(6e6 / max(1, store.streams.lidar.count())))
        )
        for obs in store.streams.lidar.order_by("ts"):
            pose = obs.pose
            if pose is None or pose.position.is_zero() or pose.orientation.is_zero():
                continue
            pts = np.asarray(obs.data.points_f32(), np.float64)
            if len(pts) == 0:
                continue
            if len(pts) > per_scan:
                pts = pts[rng.choice(len(pts), per_scan, replace=False)]
            p, q = pose.position, pose.orientation
            times.append(obs.ts)
            poses.append([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            clouds.append(pts)
    return np.asarray(times), np.asarray(poses), clouds


def guarded_icp(*args, **kwargs):
    """Ivan's `_icp`, but Open3D's singular-system error counts as a failed ICP (his own
    `(identity, inf)` rejection) instead of ending the run."""
    try:
        return _unguarded_icp(*args, **kwargs)
    except RuntimeError as error:
        if "Singular" not in str(error):
            raise
        return Transform.identity(), float("inf")


_unguarded_icp = _ivan_pgo._icp


def run_ivan(db: Path, config: dict, guarded: bool = False):
    _ivan_pgo._icp = guarded_icp if guarded else _unguarded_icp
    try:
        with SqliteStore(path=str(db)) as store:
            started = time.perf_counter()
            graph = store.streams.lidar.transform(PGO(**config)).last().data
            runtime = time.perf_counter() - started
    finally:
        _ivan_pgo._icp = _unguarded_icp
    rows_raw, rows_opt = [], []
    for kf in graph.keyframes:
        for rows, tf in ((rows_raw, kf.local), (rows_opt, kf.optimized)):
            t, r = tf.translation, tf.rotation
            rows.append([kf.ts, t.x, t.y, t.z, r.x, r.y, r.z, r.w])
    return np.asarray(rows_raw), np.asarray(rows_opt), len(graph.loops), runtime, graph


def write_gsc_input(times, poses, db: Path, out: Path):
    """Body-frame copy of the full-res lidar + an Odometry stream of the scans' own poses."""
    if out.exists() and out.stat().st_mtime >= db.stat().st_mtime:
        return
    tmp = out.with_suffix(".tmp.db")
    tmp.unlink(missing_ok=True)
    pose_by_ts = dict(zip(times.tolist(), poses, strict=True))
    with SqliteStore(path=str(db)) as src, SqliteStore(path=str(tmp)) as dst:
        odom = dst.stream("bench_odom", Odometry)
        lidar = dst.stream("bench_lidar", PointCloud2)
        for obs in src.streams.lidar.order_by("ts"):
            row = pose_by_ts.get(obs.ts)
            if row is None:
                continue
            x, y, z, qx, qy, qz, qw = row
            rotation = Rotation.from_quat([qx, qy, qz, qw])
            body = rotation.inv().apply(np.asarray(obs.data.points_f32(), np.float64) - [x, y, z])
            odom.append(
                Odometry(
                    ts=obs.ts,
                    frame_id="world",
                    child_frame_id="base_link",
                    pose=Pose(x, y, z, qx, qy, qz, qw),
                ),
                ts=obs.ts,
                pose=(x, y, z, qx, qy, qz, qw),
            )
            lidar.append(
                PointCloud2.from_numpy(
                    body.astype(np.float32), frame_id="base_link", timestamp=obs.ts
                ),
                ts=obs.ts,
                pose=(x, y, z, qx, qy, qz, qw),
            )
    tmp.rename(out)


def check_gsc_defaults(used: dict, overrides: dict | None = None) -> None:
    """pgo-eval's effective config must equal the live module's defaults (gsc_pgo/module.py),
    apart from any explicit `overrides`."""
    from dimos.core.native_module import NativeModuleConfig
    from dimos.navigation.jnav.components.loop_closure.gsc_pgo.module import PGOConfig

    live = {
        name: field.default
        for name, field in PGOConfig.model_fields.items()
        if name not in NativeModuleConfig.model_fields
    }
    differ = {
        name: (value, live.get(name, "<absent>"))
        for name, value in used.items()
        if (overrides or {}).get(name, live.get(name, "<absent>")) != value
    }
    assert not differ, f"pgo-eval config != gsc_pgo/module.py defaults (eval, module): {differ}"


def run_gsc(gsc_db: Path, out_json: Path, config: Path | None = None):
    overrides = json.loads(config.read_text()) if config else None
    started = time.perf_counter()
    args = [str(PGO_EVAL), "--db", str(gsc_db), "--odom-stream", "bench_odom"]
    args += ["--lidar-stream", "bench_lidar", "--output", str(out_json)]
    if config:
        args += ["--config", str(config)]
    subprocess.run(args, check=True)
    runtime = time.perf_counter() - started
    result = json.loads(out_json.read_text())
    check_gsc_defaults(result["config"], overrides)
    odom = np.asarray(result["odom_rows"], float)
    graph = np.asarray(result["graph"], float)
    # raw pose at each keyframe = the odom row with that exact timestamp
    index = np.clip(np.searchsorted(odom[:, 0], graph[:, 0]), 0, len(odom) - 1)
    assert np.array_equal(odom[index, 0], graph[:, 0]), "a gsc keyframe has no odom row at its ts"
    # like Ivan's graph.loops: only closures still in the final graph (GNC weight > 0)
    kept = sum(1 for edge in result["committed_loop_edges"] if edge["gnc_weight"] > 0)
    return odom[index], graph, kept, runtime, result


def correction_table(raw: np.ndarray, opt: np.ndarray):
    """Per-keyframe correction C = optimized · raw^-1, as (ts, Rotation, translations)."""
    r_raw, r_opt = Rotation.from_quat(raw[:, 4:8]), Rotation.from_quat(opt[:, 4:8])
    r_c = r_opt * r_raw.inv()
    t_c = opt[:, 1:4] - r_c.apply(raw[:, 1:4])
    return raw[:, 0], r_c, t_c


def correction_at(table, times):
    """Slerp/lerp between keyframe corrections, clipped to the ends: Ivan's own
    `PoseGraph._corrections` (go2/loop_closure/pgo.py), applied to both PGOs."""
    ts_k, r_c, t_c = table
    order = np.argsort(ts_k)
    ts_k, r_c, t_c = ts_k[order], r_c[order], t_c[order]
    keep = np.concatenate([[True], np.diff(ts_k) > 0])
    ts_k, r_c, t_c = ts_k[keep], r_c[keep], t_c[keep]
    clipped = np.clip(times, ts_k[0], ts_k[-1])
    translations = np.stack([np.interp(clipped, ts_k, t_c[:, i]) for i in range(3)], axis=1)
    return Slerp(ts_k, r_c)(clipped), translations


def apply_correction(table, times, poses, clouds):
    rotations, translations = correction_at(table, np.asarray(times))
    positions = rotations.apply(poses[:, :3]) + translations
    new_clouds = [rotations[i].apply(c) + translations[i] for i, c in enumerate(clouds)]
    return positions, new_clouds


def occupied_voxels(clouds) -> int:
    allpts = np.vstack(clouds)
    return len(np.unique(np.floor(allpts / VOXEL_M).astype(np.int64), axis=0))


def image_pose_offset(db: Path, times, poses) -> float:
    """Median distance (m) between `color_image` poses and the scan poses interpolated to the same time."""
    from dimos.msgs.sensor_msgs.Image import Image

    with SqliteStore(path=str(db)) as store:
        stream = store.stream("color_image", Image)
        step = max(1, stream.count() // 300)
        offsets = []
        for i, obs in enumerate(stream.order_by("ts")):
            if i % step or obs.pose is None or not (times[0] <= obs.ts <= times[-1]):
                continue
            scan_xyz = [np.interp(obs.ts, times, poses[:, axis]) for axis in range(3)]
            p = obs.pose.position
            offsets.append(np.linalg.norm(np.array([p.x, p.y, p.z]) - scan_xyz))
    return float(np.median(offsets)) if offsets else float("inf")


def detect_markers(db: Path, camera: str, camera_pose=None):
    """(ts, raw world xyz, orientation, marker_id) of each final smoothed marker track, built like
    Ivan's go2/loop_closure/eval.py `_eval_recording` at its CLI defaults. camera "go2" uses his
    hard-coded go2 intrinsics; "stream" uses the recording's own `camera_info`. `camera_pose(ts)`, when
    given, replaces each image's stored pose (before the speed limit and marker placement use it)."""
    from dimos.memory.transform import QualityWindow, SpeedLimit
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.perception.fiducial.marker_transformer import DetectMarkers
    from dimos.robot.unitree.go2.connection import _camera_info_static

    with SqliteStore(path=str(db)) as store:
        camera_info = (
            _camera_info_static()
            if camera == "go2"
            else store.stream("camera_info", CameraInfo).first().data
        )
        images = store.stream("color_image", Image)
        if camera_pose is not None:
            images = images.map(lambda obs: obs.with_pose(camera_pose(obs.ts)))
        detections = (
            images.transform(QualityWindow(lambda img: img.sharpness, window=0.1))
            .transform(SpeedLimit(max_mps=0.5, max_dps=50.0))
            .transform(
                DetectMarkers(camera_info=camera_info, marker_length_m=0.1, smoothing_window=7.5)
            )
            .to_list()
        )
    tracks = {d.data.track_id: d for d in detections}.values()
    return [(d.ts, d.data.center, d.data.orientation, d.data.marker_id) for d in tracks]


def _pairwise_sum(points) -> float:
    points = np.asarray(points)
    return float(
        sum(np.linalg.norm(points[i + 1 :] - p, axis=1).sum() for i, p in enumerate(points))
    )


def tag_spread(markers, table) -> float:
    """Ivan's TOTAL_SPREAD for one recording: per marker id, the summed pairwise distance
    between its corrected track positions (m, lower = tighter loop closure)."""
    times = np.array([ts for ts, _, _, _ in markers])
    raw = np.array([[c.x, c.y, c.z] for _, c, _, _ in markers])
    if table is None:
        corrected = raw
    else:
        rotations, translations = correction_at(table, times)
        corrected = rotations.apply(raw) + translations
    by_marker: dict[int, list] = {}
    for (_, _, _, marker_id), xyz in zip(markers, corrected, strict=True):
        by_marker.setdefault(marker_id, []).append(xyz)
    return sum(_pairwise_sum(v) for v in by_marker.values())


def ivan_own_spread(markers, graph) -> float:
    """The same spread through Ivan's own `PoseGraph.correct`, to check `tag_spread` matches it."""
    by_marker: dict[int, list] = {}
    for ts, center, orientation, marker_id in markers:
        t = graph.correct(
            Transform(translation=center, rotation=orientation, frame_id="world", ts=ts)
        ).translation
        by_marker.setdefault(marker_id, []).append([t.x, t.y, t.z])
    return sum(_pairwise_sum(v) for v in by_marker.values())


def score(k_raw, k_opt, loops, runtime, times, poses, clouds, markers, maps, method):
    table = correction_table(k_raw, k_opt)
    positions, corrected = apply_correction(table, times, poses, clouds)
    maps[method], maps[f"{method}_path"] = np.vstack(corrected), positions
    return {
        "keyframes": len(k_raw),
        "loops": loops,
        "runtime_s": runtime,
        "voxels": occupied_voxels(corrected),
        "end_gap_m": float(np.linalg.norm(positions[-1] - positions[0])),
        "max_shift_m": float(np.max(np.linalg.norm(positions - poses[:, :3], axis=1))),
        "tag_spread_m": None if markers is None else tag_spread(markers, table),
    }


def source_for(name: str) -> Path:
    """The db `main` benchmarks for a dataset name."""
    prepped = HERE / f"{name}.src.db"
    return prepped if prepped.exists() else DATA / f"{name}.db"


def odom_poses(db: Path, stream: str):
    """An odometry stream's ts and (N,7) pose rows [x,y,z,qx,qy,qz,qw], like `load`'s."""
    times, poses = [], []
    with SqliteStore(path=str(db)) as store:
        for obs in store.stream(stream).order_by("ts"):
            pose = getattr(obs.data, "pose", obs.data)
            p, q = getattr(pose, "pose", pose).position, getattr(pose, "pose", pose).orientation
            times.append(obs.ts)
            poses.append([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    times, unique = np.unique(times, return_index=True)
    return times, np.asarray(poses)[unique]


def pose_at(times, poses, at):
    """`poses` interpolated to the times `at`, as (Rotation, (N,3) translation)."""
    rotation = Slerp(times, Rotation.from_quat(poses[:, 3:]))(at)
    return rotation, np.stack([np.interp(at, times, poses[:, axis]) for axis in range(3)], 1)


def rebase_markers(markers, from_times, from_poses, to_times, to_poses, offset_s: float):
    """Markers sighted in `from`'s world (camera clock) re-expressed in `to`'s world and clock, through
    the robot's pose in each at the sighting: the marker stays put relative to the robot."""
    camera_times = np.array([ts for ts, _, _, _ in markers])
    from_r, from_t = pose_at(from_times, from_poses, camera_times)
    to_r, to_t = pose_at(to_times, to_poses, camera_times + offset_s)
    rebased = []
    for i, (ts, center, orientation, marker_id) in enumerate(markers):
        relative = to_r[i] * from_r[i].inv()
        xyz = relative.apply([center.x, center.y, center.z] - from_t[i]) + to_t[i]
        q = (
            relative
            * Rotation.from_quat([orientation.x, orientation.y, orientation.z, orientation.w])
        ).as_quat()
        rebased.append((ts + offset_s, Vector3(*xyz), Quaternion(*q), marker_id))
    return rebased


def camera_pose_from_tf(source: Path, dataset, times, poses):
    """Camera-in-scan-world pose at a time: the scan pose interpolated there, times the recording's
    static tf from the scan pose's frame to `dataset.tag_camera_frame` (None outside the scans)."""
    from prep import static_chain

    with SqliteStore(path=str(source)) as store:
        odom = store.stream(dataset.odom).first().data
        body = getattr(odom, "child_frame_id", "") or "base_link"
        lidar_frame = store.stream(dataset.lidar).first().data.frame_id
        body_r_camera, body_t_camera = static_chain(store, body, dataset.tag_camera_frame)
        body_r_lidar, body_t_lidar = (
            (Rotation.identity(), np.zeros(3))
            if lidar_frame in ("world", "odom", "map")
            else static_chain(store, body, lidar_frame)
        )
    # scan poses are body * mount (prep.py); camera = scan pose * mount^-1 * body->camera
    scan_r_camera = body_r_lidar.inv() * body_r_camera
    scan_t_camera = body_r_lidar.inv().apply(body_t_camera - body_t_lidar)
    times, unique = np.unique(times, return_index=True)
    poses = poses[unique]
    slerp = Slerp(times, Rotation.from_quat(poses[:, 3:]))

    def at(ts: float):
        if not times[0] <= ts <= times[-1]:
            return None
        r = slerp([ts])[0]
        t = np.array([np.interp(ts, times, poses[:, axis]) for axis in range(3)])
        return (*(r.apply(scan_t_camera) + t), *(r * scan_r_camera).as_quat())

    return at


def markers_for(name: str, times, poses):
    """A dataset's marker tracks in its scans' world and clock (None if not scorable), and why."""
    dataset = BY_NAME.get(name)
    if dataset is None or not dataset.tag_camera:
        return None, "no camera configured"
    source = dataset.source()
    if dataset.tag_camera_frame:
        camera_pose = camera_pose_from_tf(source, dataset, times, poses)
        markers = [
            m
            for m in detect_markers(source, dataset.tag_camera, camera_pose)
            if m[3] not in dataset.tag_ids_moving
        ]
        note = f"camera poses from the scans' odometry and tf -> {dataset.tag_camera_frame}"
        return (markers or None), note + ("" if markers else "; 0 marker tracks")
    camera_times, camera_poses = (
        odom_poses(source, dataset.tag_odom)
        if dataset.tag_odom
        else (times - dataset.clock_offset_s, poses)
    )
    offset = image_pose_offset(source, camera_times, camera_poses)
    note = f"image poses {offset:.2f} m from {dataset.tag_odom or 'scan'} poses (median)"
    if offset == float("inf"):
        return None, "no images inside the scans' time range (different clock), skipped"
    if offset > MAX_IMAGE_POSE_OFFSET_M:
        return None, note + "; different world, skipped"
    markers = [
        m for m in detect_markers(source, dataset.tag_camera) if m[3] not in dataset.tag_ids_moving
    ]
    if dataset.tag_ids_moving:
        note += f"; ids {list(dataset.tag_ids_moving)} left out (moving tags)"
    if not markers:
        return None, note + "; 0 marker tracks"
    if dataset.tag_odom:
        markers = rebase_markers(
            markers, camera_times, camera_poses, times, poses, dataset.clock_offset_s
        )
        note += f"; rebased from {dataset.tag_odom} onto the scans"
    return markers, note


def main():
    summary = []
    for name in sys.argv[1:]:
        name, _, explicit = name.partition("=")
        db = Path(explicit) if explicit else HERE / f"{name}.src.db"
        if not db.exists():
            db = DATA / f"{name}.db"
        print(f"== {name}", flush=True)
        times, poses, clouds = load(db)
        markers, tag_note = markers_for(name, times, poses)
        print(f"tags: {tag_note}", flush=True)
        row = {
            "dataset": name,
            "scans": len(times),
            "duration_s": float(times[-1] - times[0]),
            "raw_end_gap_m": float(np.linalg.norm(poses[-1, :3] - poses[0, :3])),
            "voxels_raw": occupied_voxels(clouds),
            "tag_spread_raw_m": None if markers is None else tag_spread(markers, None),
            "tag_tracks": None if markers is None else len(markers),
            "tag_note": tag_note,
        }
        maps = {"raw": np.vstack(clouds), "raw_path": poses[:, :3], "times": times}
        for method, config in IVAN_CONFIGS.items():
            error, graph = None, None
            try:
                k_raw, k_opt, loops, runtime, graph = run_ivan(db, config)
            except RuntimeError as crash:  # Ivan's PGO raising ends his whole run
                error = f"{type(crash).__name__}: {str(crash).strip()[-160:]}"
                print(f"{method} CRASHED: {error}; rerunning with the ICP guard", flush=True)
                k_raw, k_opt, loops, runtime, graph = run_ivan(db, config, guarded=True)
            print(f"{method}: {len(k_raw)} keyframes, {loops} loops, {runtime:.1f}s", flush=True)
            row[method] = score(
                k_raw, k_opt, loops, runtime, times, poses, clouds, markers, maps, method
            )
            row[method]["crashed_unguarded"] = error
            if markers is not None:
                row[method]["tag_spread_ivan_correct_m"] = ivan_own_spread(markers, graph)
        gsc_db = HERE / f"{name}.gsc.db"
        write_gsc_input(times, poses, db, gsc_db)
        g_raw, g_opt, gsc_loops, gsc_time, gsc_result = run_gsc(gsc_db, HERE / f"{name}.gsc.json")
        print(f"gsc: {len(g_raw)} keyframes, {gsc_loops} loops, {gsc_time:.1f}s", flush=True)
        row["gsc"] = score(
            g_raw, g_opt, gsc_loops, gsc_time, times, poses, clouds, markers, maps, "gsc"
        )
        row["gsc"]["loops_before_gnc"] = gsc_result["closures"]
        row["gsc"]["gate_counts"] = gsc_result.get("gate_counts")
        np.savez_compressed(HERE / f"{name}.maps.npz", **maps)
        print(json.dumps(row), flush=True)
        (HERE / f"{name}.summary.json").write_text(json.dumps(row, indent=2))
        summary.append(row)


if __name__ == "__main__":
    main()
