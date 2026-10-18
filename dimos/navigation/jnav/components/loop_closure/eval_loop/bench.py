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

"""Shared scoring for the live replays, and the per-recording baseline.

Both PGOs get the exact same input: the recording's `lidar` stream (world-frame scans, each
stamped with its odom pose). Each live replay (variant.py for gsc, ivan_live.py for Ivan's module)
yields a keyframe table (ts, raw pose, optimized pose); `score` turns it into a per-scan correction
with the same slerp/lerp and measures the corrected map.

usage: bench.py <dataset> [<dataset> ...]   -> <dataset>.summary.json (raw stats) and <dataset>.gsc.db
"""

import json
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from paths import OUT as HERE, PGO_EVAL, REPO
from recordings import BY_NAME
from scipy.spatial.transform import Rotation, Slerp

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

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
    """(ts, raw world xyz, orientation, marker_id) per sighting cluster, through jnav's apriltags.py:
    each glimpse passes its gates (blur, reprojection, size, distance, view angle, camera speed), is
    placed in the world by its camera pose, and each tag's glimpses split where they're more than 5 s
    apart, each cluster reduced to a Huber-refined medoid. Timed by the store ts, never the image's own
    header stamp. camera "go2" uses the hard-coded go2 intrinsics; "stream" uses the recording's own
    `camera_info`. `camera_pose(ts)`, when given, replaces each image's stored pose."""
    import cv2

    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.navigation.jnav.utils import apriltags
    from dimos.perception.fiducial.marker_pose import (
        camera_info_to_cv_matrices,
        create_aruco_detector,
        estimate_marker_pose,
        marker_reprojection_error,
    )
    from dimos.robot.unitree.go2.connection import _camera_info_static

    marker_length_m = 0.1
    detector = create_aruco_detector("DICT_APRILTAG_36h11")
    glimpses = []
    previous = None
    with SqliteStore(path=str(db)) as store:
        camera_info = (
            _camera_info_static()
            if camera == "go2"
            else store.stream("camera_info", CameraInfo).first().data
        )
        intrinsics, distortion = camera_info_to_cv_matrices(camera_info)
        for obs in store.stream("color_image", Image).order_by("ts"):
            if camera_pose is not None:
                pose = camera_pose(obs.ts)
            elif obs.pose is not None:
                p, q = obs.pose.position, obs.pose.orientation
                pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            else:
                pose = None
            if pose is None:
                previous = None
                continue
            pose = np.asarray(pose, float)
            speed = None
            if previous is not None and obs.ts > previous[0]:
                dt = obs.ts - previous[0]
                cos_half = min(1.0, abs(float(pose[3:] @ previous[1][3:])))
                speed = (
                    float(np.linalg.norm(pose[:3] - previous[1][:3])) / dt,
                    np.degrees(2.0 * np.arccos(cos_half)) / dt,
                )
            previous = (obs.ts, pose)
            bgr = obs.data.to_opencv()
            all_corners, marker_ids, _ = detector.detectMarkers(bgr)
            if marker_ids is None:
                continue
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
            world_r_camera = Rotation.from_quat(pose[3:])
            for corners, marker_id in zip(all_corners, marker_ids.flatten(), strict=True):
                solved = estimate_marker_pose(corners, marker_length_m, intrinsics, distortion)
                if solved is None:
                    continue
                rotation_vector, translation_vector = solved
                camera_r_marker = Rotation.from_rotvec(rotation_vector.reshape(3))
                camera_t_marker = translation_vector.reshape(3)
                glimpse = {
                    "ts": float(obs.ts),
                    "marker_id": int(marker_id),
                    "t_cam_marker": [*camera_t_marker, *camera_r_marker.as_quat()],
                    "sharpness": apriltags.tag_sharpness(gray, corners),
                    "reproj_px": marker_reprojection_error(
                        corners,
                        marker_length_m,
                        intrinsics,
                        distortion,
                        rotation_vector,
                        translation_vector,
                    ),
                    "tag_px": apriltags.tag_pixel_size(corners),
                    "speed": speed,
                }
                if apriltags.glimpse_passes(glimpse) is not None:
                    continue
                # robust_cluster_pose only reads "t_cam_marker", so it carries the world pose from here on
                world_t_marker = world_r_camera.apply(camera_t_marker) + pose[:3]
                world_q_marker = (world_r_camera * camera_r_marker).as_quat()
                glimpses.append({**glimpse, "t_cam_marker": [*world_t_marker, *world_q_marker]})
    markers = []
    for cluster in apriltags.cluster_by_time(glimpses, apriltags.DEFAULT_CLUSTER_GAP_SEC):
        if len(cluster) < apriltags.DEFAULT_MIN_OBSERVATIONS:
            continue
        medoid = apriltags.robust_cluster_pose(
            cluster,
            apriltags.DEFAULT_ROTATION_WEIGHT_M_PER_RAD,
            apriltags.DEFAULT_HUBER_DELTA_M,
        )
        x, y, z, qx, qy, qz, qw = medoid["t_cam_marker"]
        markers.append(
            (medoid["ts"], Vector3(x, y, z), Quaternion(qx, qy, qz, qw), medoid["marker_id"])
        )
    return sorted(markers, key=lambda marker: marker[0])


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
    """A dataset's marker tracks in its scans' world and clock (None if not scorable), and why.
    Cached in <name>.markers.json, so a machine without the camera recording can still score tags."""
    cache = HERE / f"{name}.markers.json"
    if cache.exists():
        saved = json.loads(cache.read_text())
        markers = [
            (ts, Vector3(*center), Quaternion(*orientation), marker_id)
            for ts, center, orientation, marker_id in saved["markers"]
        ]
        return (markers or None), saved["note"]
    markers, note = _detect_markers_for(name, times, poses)
    rows = [
        (ts, [c.x, c.y, c.z], [q.x, q.y, q.z, q.w], int(marker_id))
        for ts, c, q, marker_id in markers or []
    ]
    cache.write_text(json.dumps({"markers": rows, "note": note}))
    return markers, note


def _detect_markers_for(name: str, times, poses):
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
        write_gsc_input(times, poses, db, HERE / f"{name}.gsc.db")
        print(json.dumps(row), flush=True)
        (HERE / f"{name}.summary.json").write_text(json.dumps(row, indent=2))


if __name__ == "__main__":
    main()
