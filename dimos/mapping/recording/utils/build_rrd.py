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

"""Dump a go2 mem2.db to a Rerun .rrd (adapted from dimos
mapping/loop_closure/utils/map_rrd.py), focused on the post-processed result.

For each lidar stream it logs both the per-frame clouds and a single aggregated,
voxel-downsampled "map" (corrected fastlio re-anchored on `gtsam_odom`; the Go2
`lidar` stays in its own `odom` frame). Clouds get a slight height-color
gradient; trajectories get a start->end gradient. Each AprilTag is placed in 3D
(via the corrected trajectory) with its detections, a labeled marker, basis-
vector axes at the perceived pose, and the robot's-eye camera image at the
recognition moment.
"""

from __future__ import annotations

from datetime import datetime
import json
import math
from pathlib import Path
import sqlite3

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from scipy.spatial.transform import Rotation

from dimos.mapping.recording.go2_mid360.static_transforms import (
    GO2_BODY_HALF_SIZES,
    MID360_TO_BASE,
)
from dimos.mapping.recording.utils import stream_names
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

TIMELINE = "ts"
# one distinct base color per point cloud (height modulates brightness within each)
_CLOUD_PALETTE = [(0, 180, 170), (240, 160, 40), (220, 90, 180), (150, 200, 60), (90, 150, 235)]
_TAG_PALETTE = [
    (255, 80, 80),
    (80, 200, 255),
    (255, 220, 60),
    (160, 100, 255),
    (90, 230, 130),
    (255, 150, 60),
]


def _mat(trans, quat) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = Rotation.from_quat(quat).as_matrix()
    M[:3, 3] = trans
    return M


def _mat7(p) -> np.ndarray:
    return _mat(p[:3], p[3:7])


# fastlio reports the mid360 (lidar) frame, which is mounted pitched 44 deg down
# relative to the front camera; place tags/camera by chaining mid360 -> camera.
# (base_link cancels out, so we only need the camera<->mid360 relationship.)
_PITCH_HALF = math.radians(44.0) / 2.0
_M_FC_MID360 = _mat([-0.032, 0.0, 0.12], [0.0, math.sin(_PITCH_HALF), 0.0, math.cos(_PITCH_HALF)])
_M_FC_OPTICAL = _mat([0.0, 0.0, 0.0], [-0.5, 0.5, -0.5, 0.5])  # REP-103 optical in camera body
MID360_TO_OPTICAL = np.linalg.inv(_M_FC_MID360) @ _M_FC_OPTICAL  # legacy: optical pts -> mid360 pts


def _pose7(p):
    if hasattr(p, "orientation"):
        o = p.orientation
        return [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
    return list(p)


def _voxel_keys(pts: np.ndarray, voxel: float) -> np.ndarray:
    """One int64 voxel id per point (the cell it falls in at `voxel` resolution)."""
    cell = np.floor(pts / voxel).astype(np.int64) + (1 << 20)
    return cell[:, 0] | (cell[:, 1] << 21) | (cell[:, 2] << 42)


def _down(pts: np.ndarray, voxel: float) -> np.ndarray:
    if voxel <= 0 or len(pts) == 0:
        return pts
    _, idx = np.unique(_voxel_keys(pts, voxel), return_index=True)
    return pts[idx]


def _shaded(pts: np.ndarray, base) -> np.ndarray:
    """Per-cloud color: the cloud's base color, brightness modulated by height
    (darker low -> brighter high). Distinct hue per cloud, subtle within-cloud."""
    z = pts[:, 2]
    lo, hi = np.percentile(z, 5), np.percentile(z, 95)
    t = np.clip((z - lo) / (hi - lo + 1e-9), 0, 1)[:, None]
    return (np.array(base, np.float32) * (0.5 + 0.5 * t)).astype(np.uint8)


def _log_frames(store, stream, entity, stride, voxel, base):
    if stream not in store.list_streams():
        return
    print(f"   rrd: logging {stream} frames -> {entity} (stride {stride}) ...", flush=True)
    n = 0
    for k, obs in enumerate(store.stream(stream, PointCloud2)):
        if k % stride:
            continue
        pts = _down(obs.data.points_f32(), voxel)
        if len(pts) == 0:
            continue
        rr.set_time(TIMELINE, timestamp=obs.ts)
        rr.log(entity, rr.Points3D(pts, colors=_shaded(pts, base)))
        n += 1
    print(f"   rrd: {entity} <- {stream} ({n} frames, stride {stride}, voxel {voxel}m)")


def _log_map(store, stream, entity, voxel, base):
    """Aggregate a lidar stream into one voxel-downsampled map. Streams cloud by
    cloud, downsampling each and keeping only points in not-yet-seen voxels, so
    peak memory is bounded by the *final* map size (not the sum of all frames)."""
    if stream not in store.list_streams():
        return
    print(f"   rrd: aggregating {stream} map -> {entity} (this is the slow one) ...", flush=True)
    if voxel <= 0:
        chunks = [
            cloud
            for cloud in (obs.data.points_f32() for obs in store.stream(stream, PointCloud2))
            if len(cloud)
        ]
        if not chunks:
            return
        pts = np.concatenate(chunks)
        rr.log(entity, rr.Points3D(pts, colors=_shaded(pts, base)), static=True)
        print(f"   rrd: {entity} <- {stream} ({len(pts):,} pts, no voxel)")
        return

    seen: set[int] = set()
    kept: list[np.ndarray] = []
    for obs in store.stream(stream, PointCloud2):
        pts = _down(obs.data.points_f32(), voxel)
        if len(pts) == 0:
            continue
        keys = _voxel_keys(pts, voxel)
        fresh = np.array([key not in seen for key in keys.tolist()], dtype=bool)
        if not fresh.any():
            continue
        kept.append(pts[fresh])
        seen.update(keys[fresh].tolist())
    if not kept:
        return
    pts = np.concatenate(kept)
    rr.log(entity, rr.Points3D(pts, colors=_shaded(pts, base)), static=True)
    print(f"   rrd: {entity} <- {stream} ({len(pts):,} pts, voxel {voxel}m)")


def _path(db_path, stream):
    try:
        conn = sqlite3.connect(db_path)
        rows = conn.execute(
            f'SELECT pose_x,pose_y,pose_z FROM "{stream}" WHERE pose_x IS NOT NULL ORDER BY ts'
        ).fetchall()
        conn.close()
    except sqlite3.OperationalError:
        return None
    return np.array(rows, float) if rows else None


def _log_odom_frames(db_path, stride=5):
    """A moving XYZ basis-vector triad per odom stream (Transform3D over time +
    a static axis triad). Doubles as the eye's tracking target."""
    conn = sqlite3.connect(db_path)
    present = {row[0] for row in conn.execute("SELECT name FROM _streams")}
    conn.close()
    is_go2 = stream_names.ODOM in present or stream_names.LIDAR in present
    for stream, name in [
        (stream_names.GTSAM_ODOM, "gtsam"),
        (stream_names.FASTLIO_ODOM, "fastlio"),
        (stream_names.ODOM, "odom"),  # Go2 onboard leg odometry
    ]:
        try:
            conn = sqlite3.connect(db_path)
            rows = conn.execute(
                f"SELECT ts,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw "
                f'FROM "{stream}" WHERE pose_qw IS NOT NULL ORDER BY ts'
            ).fetchall()
            conn.close()
        except sqlite3.OperationalError:
            continue
        if not rows:
            continue
        ent = f"world/{name}_frame"
        rr.log(
            ent,
            rr.Arrows3D(
                vectors=[[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]],
                colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
            ),
            static=True,
        )
        # translucent box marking the odom source (child -> inherits the moving
        # transform); the fastlio_odometry box is yellow, the others green. On a Go2
        # the fastlio (mid360) frame's box is the dog trunk, statically offset to
        # base_link (mid360->base). reframe_go2 normalizes fastlio_odometry's pose to
        # mid360, so this offset always lands the box on the (level) base_link.
        box_color = [220, 220, 0, 128] if name == "fastlio" else [0, 220, 0, 128]
        if name == "fastlio" and is_go2:
            offset = MID360_TO_BASE
            rr.log(
                f"{ent}/box",
                rr.Transform3D(
                    translation=[offset.translation.x, offset.translation.y, offset.translation.z],
                    quaternion=rr.Quaternion(
                        xyzw=[
                            offset.rotation.x,
                            offset.rotation.y,
                            offset.rotation.z,
                            offset.rotation.w,
                        ]
                    ),
                ),
                static=True,
            )
            rr.log(
                f"{ent}/box",
                rr.Boxes3D(half_sizes=[list(GO2_BODY_HALF_SIZES)], colors=[box_color]),
                static=True,
            )
        else:
            rr.log(
                f"{ent}/box",
                rr.Boxes3D(half_sizes=[[0.35, 0.2, 0.15]], colors=[box_color]),
                static=True,
            )
        for r in rows[::stride]:
            rr.set_time(TIMELINE, timestamp=r[0])
            rr.log(
                ent, rr.Transform3D(translation=r[1:4], quaternion=rr.Quaternion(xyzw=list(r[4:8])))
            )
        print(f"   rrd: {ent} <- {stream} ({len(rows[::stride])} frames, basis axes)")


def _log_path_gradient(db_path, stream, entity, base):
    """Log a trajectory as per-segment line strips with a start->end brightness
    gradient (dim at start, full color at end) so direction/time is visible."""
    pts = _path(db_path, stream)
    if pts is None or len(pts) < 2:
        return
    segs = [pts[i : i + 2] for i in range(len(pts) - 1)]
    t = np.linspace(0.0, 1.0, len(segs))[:, None]
    base = np.array(base, np.float32)
    colors = ((0.25 + 0.75 * t) * base).astype(np.uint8)
    rr.log(entity, rr.LineStrips3D(segs, colors=colors), static=True)
    print(f"   rrd: {entity} <- {stream} ({len(pts)} poses, gradient)")


def _has_rows(conn, stream):
    try:
        return (
            conn.execute(f'SELECT 1 FROM "{stream}" WHERE pose_qw IS NOT NULL LIMIT 1').fetchone()
            is not None
        )
    except sqlite3.OperationalError:
        return False


def _log_apriltags(store, db_path, cam_xform, intrinsics, resolution, max_views_per_tag=40):
    """Place every AprilTag recognition in 3D via the corrected trajectory:
    T_world_tag = T_world_base(t) . T_base_optical . T_cam_tag. Per marker logs
    the detection cloud + a labeled marker, and for a sample of recognitions:
    XYZ basis axes at the perceived tag pose and the robot's-eye camera image on
    a pinhole frustum at the camera pose (3D only — see the blueprint)."""
    if stream_names.APRIL_TAGS not in store.list_streams():
        return
    print("   rrd: placing april_tags in 3D ...", flush=True)
    connection = sqlite3.connect(db_path)
    traj_stream = (
        stream_names.GTSAM_ODOM
        if _has_rows(connection, stream_names.GTSAM_ODOM)
        else stream_names.FASTLIO_ODOM
    )
    pose_rows = connection.execute(
        f"SELECT ts,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw "
        f'FROM "{traj_stream}" WHERE pose_qw IS NOT NULL ORDER BY ts'
    ).fetchall()
    connection.close()
    if not pose_rows:
        return
    traj_timestamps = np.array([row[0] for row in pose_rows])
    traj_poses = np.array([row[1:8] for row in pose_rows], float)

    def optical_in_world(timestamp):
        """T_world_optical = T_world_traj . (traj-frame -> optical)."""
        index = int(np.searchsorted(traj_timestamps, timestamp))
        return _mat7(traj_poses[min(max(index, 0), len(traj_timestamps) - 1)]) @ cam_xform

    detections_by_marker: dict[int, dict] = {}
    for detection_obs in store.stream(stream_names.APRIL_TAGS, PoseStamped):
        marker_id = (detection_obs.tags or {}).get("marker_id")
        if marker_id is None or detection_obs.pose is None:
            continue
        tag_world = optical_in_world(detection_obs.ts) @ _mat7(_pose7(detection_obs.pose))
        entry = detections_by_marker.setdefault(int(marker_id), {"poses": [], "ts": []})
        entry["poses"].append(tag_world)
        entry["ts"].append(detection_obs.ts)

    camera_targets = []  # (entity, T_world_optical, ts) for image logging
    for palette_index, (marker_id, entry) in enumerate(sorted(detections_by_marker.items())):
        tag_poses = np.array(entry["poses"])
        timestamps = np.array(entry["ts"])
        positions = tag_poses[:, :3, 3]
        color = _TAG_PALETTE[palette_index % len(_TAG_PALETTE)]
        center = np.median(positions, 0)
        rr.log(
            f"world/april_tags/marker_{marker_id}/detections",
            rr.Points3D(positions, colors=color, radii=0.1),
            static=True,
        )
        rr.log(
            f"world/april_tags/marker_{marker_id}",
            rr.Points3D(
                [center], colors=color, radii=0.5, labels=[f"tag {marker_id}"], show_labels=True
            ),
            static=True,
        )
        # sample recognitions: axes at the perceived tag pose + a camera frustum
        sample_stride = max(1, len(tag_poses) // max_views_per_tag)
        sampled = range(0, len(tag_poses), sample_stride)
        for sample_index in sampled:
            tag_pose = tag_poses[sample_index]
            tag_entity = f"world/april_tags/marker_{marker_id}/recognitions/{sample_index:04d}/tag"
            rr.log(
                tag_entity,
                rr.Transform3D(translation=tag_pose[:3, 3], mat3x3=tag_pose[:3, :3]),
                static=True,
            )
            rr.log(
                tag_entity,
                rr.Arrows3D(
                    vectors=[[0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.3]],
                    colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                ),
                static=True,
            )
            cam_entity = f"world/april_tags/marker_{marker_id}/recognitions/{sample_index:04d}/cam"
            camera_targets.append(
                (cam_entity, optical_in_world(timestamps[sample_index]), timestamps[sample_index])
            )
        print(
            f"   rrd: world/april_tags/marker_{marker_id} @ {center.round(1).tolist()} "
            f"({len(positions)} detections, {len(sampled)} views, via {traj_stream})"
        )

    _log_cam_frustums(store, camera_targets, intrinsics, resolution)


def _log_cam_frustums(store, camera_targets, intrinsics, resolution):
    """Place the robot's-eye color image on a pinhole frustum at the camera pose
    for each (entity, T_world_optical, ts) target — rendered only in 3D."""
    if not camera_targets or stream_names.COLOR_IMAGE not in store.list_streams():
        return
    nearest = [(1e18, None) for _ in camera_targets]  # (time delta, image obs) per target
    for image_obs in store.stream(stream_names.COLOR_IMAGE, Image):
        for target_index, (_entity, _pose, target_ts) in enumerate(camera_targets):
            delta = abs(image_obs.ts - target_ts)
            if delta < nearest[target_index][0]:
                nearest[target_index] = (delta, image_obs)
    logged = 0
    for (entity, optical_pose, _target_ts), (_delta, image_obs) in zip(
        camera_targets, nearest, strict=False
    ):
        if image_obs is None:
            continue
        rr.log(
            entity,
            rr.Transform3D(translation=optical_pose[:3, 3], mat3x3=optical_pose[:3, :3]),
            static=True,
        )
        rr.log(
            entity,
            rr.Pinhole(
                image_from_camera=intrinsics,
                resolution=list(resolution),
                camera_xyz=rr.ViewCoordinates.RDF,
                image_plane_distance=0.6,
            ),
            static=True,
        )
        try:
            rr.log(f"{entity}/rgb", image_obs.data.to_rerun(), static=True)
            logged += 1
        except Exception:
            pass
    print(f"   rrd: {logged} recognition camera frustums (robot's-eye images, 3D only)")


# dimos jsonl level -> rerun TextLog level
_LOG_LEVELS = {
    "debug": "DEBUG",
    "info": "INFO",
    "warning": "WARN",
    "error": "ERROR",
    "critical": "CRITICAL",
}
# keys rendered structurally; everything else is appended as key=value context
_LOG_STD_KEYS = {"event", "level", "logger", "timestamp", "func_name", "lineno"}


def _find_jsonl(db_path: str) -> Path | None:
    """A dimos `main.jsonl` for this recording — next to the db."""
    candidate = Path(db_path).parent / "main.jsonl"
    return candidate if candidate.exists() else None


def _iso_to_epoch(value: str) -> float:
    return datetime.fromisoformat(value.replace("Z", "+00:00")).timestamp()


def _log_jsonl(jsonl_path: Path) -> None:
    """Replay a dimos `main.jsonl` as rerun TextLog entries on the `ts` timeline."""
    n = 0
    for line in jsonl_path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            entry = json.loads(line)
        except json.JSONDecodeError:
            continue
        timestamp = entry.get("timestamp")
        if not timestamp:
            continue
        body = str(entry.get("event", ""))
        extra = "  ".join(f"{k}={entry[k]}" for k in entry if k not in _LOG_STD_KEYS)
        if extra:
            body = f"{body}  ({extra})"
        if entry.get("logger"):
            body = f"[{entry['logger']}] {body}"
        rr.set_time(TIMELINE, timestamp=_iso_to_epoch(timestamp))
        rr.log(
            "logs", rr.TextLog(body, level=_LOG_LEVELS.get(str(entry.get("level")).lower(), "INFO"))
        )
        n += 1
    print(f"   rrd: logs <- {jsonl_path.name} ({n} entries)")


def build_rrd(
    db_path: str,
    out_path: str,
    intrinsics,
    optical_in_base,
    resolution,
    *,
    map_voxel: float = 0.1,
    cloud_stride: int = 3,
    camera_stride: int = 30,
    mid360_pitch: bool = False,
):
    rr.init("recording_post_process", recording_id=str(out_path))
    rr.save(str(out_path))
    cam_xform = MID360_TO_OPTICAL if mid360_pitch else _mat7(optical_in_base)
    jsonl_path = _find_jsonl(db_path)

    with SqliteStore(path=db_path) as store:
        streams = store.list_streams()
        # Explicit blueprint: a 3D view (incl. the recognition camera frustums) +
        # a 2D panel for the live camera, so rerun doesn't auto-make a panel per
        # pinhole. Heavy aggregated maps default to hidden (toggle in the entity
        # panel); the eye tracks the primary odom frame; left/bottom panels open.
        track = (
            "/world/odom_frame"
            if stream_names.ODOM in streams
            else "/world/fastlio_frame"
            if stream_names.FASTLIO_ODOM in streams
            else "/world/gtsam_frame"
        )
        hide = {
            f"/world/{m}": rrb.EntityBehavior(visible=False)
            for m in (
                "go2_map",
                "corrected_go2_map",
                "fastlio_map",
                "corrected_fastlio_map",
                "onboard_map",
            )
        }
        views = rrb.Horizontal(
            rrb.Spatial3DView(
                origin="/world",
                name="3D",
                overrides=hide,
                eye_controls=rrb.EyeControls3D(kind=rrb.Eye3DKind.Orbital, tracking_entity=track),
            ),
            rrb.Spatial2DView(origin="/world/camera", name="camera"),
            column_shares=[3, 1],
        )
        # When a dimos main.jsonl is present, dock its log replay below the views.
        layout = (
            rrb.Vertical(views, rrb.TextLogView(origin="/logs", name="logs"), row_shares=[4, 1])
            if jsonl_path is not None
            else views
        )
        rr.send_blueprint(
            rrb.Blueprint(
                layout,
                rrb.BlueprintPanel(state=rrb.PanelState.Expanded),
                rrb.TimePanel(state=rrb.PanelState.Expanded),
                rrb.SelectionPanel(state=rrb.PanelState.Collapsed),
            )
        )

        ci = 0  # rotate a distinct base color through each point cloud
        for raw, corr, name in [
            # Go2 onboard L1 (re-anchored on gtsam)
            (stream_names.LIDAR, stream_names.corrected(stream_names.LIDAR), "go2"),
            # mid360
            (
                stream_names.FASTLIO_LIDAR,
                stream_names.corrected(stream_names.FASTLIO_LIDAR),
                "fastlio",
            ),
        ]:
            for stream, ent in [(raw, name), (corr, f"corrected_{name}")]:
                if stream and stream in streams:
                    base = _CLOUD_PALETTE[ci % len(_CLOUD_PALETTE)]
                    ci += 1
                    _log_frames(store, stream, f"world/{ent}_lidar", cloud_stride, map_voxel, base)
                    _log_map(store, stream, f"world/{ent}_map", map_voxel, base)

        _log_apriltags(store, db_path, cam_xform, intrinsics, resolution)

        for stream, entity, base in [
            (stream_names.GTSAM_ODOM, "world/gtsam_path", (0, 220, 0)),  # corrected GT -> green
            (stream_names.FASTLIO_ODOM, "world/fastlio_path", (0, 200, 220)),  # cyan
            (stream_names.ODOM, "world/odom_path", (255, 165, 0)),  # Go2 onboard odom -> orange
        ]:
            _log_path_gradient(db_path, stream, entity, base)
        _log_odom_frames(db_path)  # moving XYZ basis triads (+ eye tracking target)

        if stream_names.COLOR_IMAGE in streams:
            n = 0
            for k, obs in enumerate(store.stream(stream_names.COLOR_IMAGE, Image)):
                if k % camera_stride:
                    continue
                rr.set_time(TIMELINE, timestamp=obs.ts)
                try:
                    rr.log("world/camera/image", obs.data.to_rerun())
                    n += 1
                except Exception:
                    break
            print(f"   rrd: world/camera/image <- color_image ({n} frames, stride {camera_stride})")

    if jsonl_path is not None:
        _log_jsonl(jsonl_path)
    print(f"   wrote {out_path}")
