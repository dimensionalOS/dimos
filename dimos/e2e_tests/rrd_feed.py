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

"""Live LCM feeder for the full china-office session (.rrd + sibling mem2.db).

Neither file is natively readable by GO2Connection's own backends, and --
the hard-won part -- the session's many logged streams live in *mutually
inconsistent frames*, so the pieces must be picked carefully:

- Camera images: only in the .rrd (`/world/camera/image`, 558 frames). The
  sibling mem2.db has no color stream.
- Lidar: the .rrd's cloud entities are all unusable for world-frame work --
  `/world/go2_lidar`/`go2_map` are in the Go2's internal odom frame with the
  floors collapsed (z never exceeds ~1.4m across a 3-storey tour: its odom
  doesn't track height), and `/world/fastlio_lidar`/`fastlio_map` carry tens
  of meters of SLAM drift (z runs to -44m). The one internally-consistent
  pair in the session is mem2.db's ground-truth PointLIO pipeline:
  `gt_pointlio_lidar` (11,944 sensor-local scans; its frame_id claims 'odom'
  but the points are sensor-relative -- verified by scan centroids staying
  ~1-3m from origin while the pose walks the building) matched with
  `gt_pointlio_odometry` (35,699 base_link poses whose z-dwell shows the
  actual floors at ~0.1/1.1/3.6/5.4m).
- Rig geometry: the session's own `go2_mid360_rotated.urdf` -- scans live in
  its `mid360_gravity` frame, the camera in `camera_optical`, both chained
  off `base_link`. (GO2Connection's built-in chain is close but this URDF is
  the session-specific truth.)

Per camera frame, the feeder accumulates the last few seconds of scans
(transformed base->gravity->world via the matched pose), publishes that
window as a world-frame /lidar cloud stamped at the *camera's* timestamp
(so camera/lidar alignment pairs every frame exactly), publishes tf
(world->base_link + static base_link->camera_optical), and publishes the
image -- the same topics a live GO2Connection would use, consumed by the
connection-less `unitree-go2-scene-memory-feed` blueprint.

Validated by projecting detections through the full chain: 22/31 sampled
YOLO-E detections localize with the module's "sparse" filter preset (vs 0
with every .rrd-native cloud/pose combination tried first).
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator
import contextlib
from dataclasses import dataclass
import json
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np

from dimos.core.transport import LCMTransport
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.protocol.tf.tf import TF

# Fallback intrinsics (hand-copied from the china-office session's
# camera_intrinsics.json) — used only when the session dir carries no
# machine-readable calibration; see load_camera_intrinsics().
CAMERA_INTRINSICS = {
    "fx": 797.47561649,
    "fy": 796.48721128,
    "cx": 643.53521678,
    "cy": 349.27836053,
    "width": 1280,
    "height": 720,
}


def load_camera_intrinsics(session_dir: str | Path) -> dict[str, float | int]:
    """Intrinsics for CameraInfo.from_intrinsics, read from the session itself.

    Parses ``camera_intrinsics.json`` (same schema colorize_from_session
    reads: a 3x3 ``intrinsics`` K matrix + ``resolution``) so a re-calibrated
    or different session is fed with ITS calibration instead of silently
    projecting through the baked china-office constants. Falls back to those
    constants when the file is absent.
    """
    path = Path(session_dir).expanduser() / "camera_intrinsics.json"
    if not path.exists():
        return CAMERA_INTRINSICS
    intr = json.loads(path.read_text())
    k = np.asarray(intr["intrinsics"], dtype=np.float64)
    width, height = intr["resolution"]
    return {
        "fx": float(k[0, 0]),
        "fy": float(k[1, 1]),
        "cx": float(k[0, 2]),
        "cy": float(k[1, 2]),
        "width": int(width),
        "height": int(height),
    }

ODOMETRY_STREAM = "gt_pointlio_odometry"
SCAN_STREAM = "gt_pointlio_lidar"
# Camera images now live in mem2.db too (color_image: full-rate RGB stream on
# the *same* clock as the gt_pointlio pose/scan streams above), so the whole
# feed can come from one file -- see iter_mem2_camera_frames. The .rrd path
# (load_rrd_frames) predates those images being stored and only carries a
# decimated 558-frame subset in a different frame convention.
CAMERA_STREAM = "color_image"
DEFAULT_MEM2_CAMERA_HZ = 1.0  # decimation target for the mem2 image feed

SCAN_WINDOW_S = 3.0  # seconds of scans accumulated into each /lidar publish
PUBLISH_DELAY_S = 0.01  # small yield between messages; flow control does the pacing
CAMERA_INFO_PERIOD_S = 1.0  # matches GO2Connection.publish_camera_info's cadence
ACK_TIMEOUT_S = 8.0  # max wait for one frame's detections_2d ack

# Keep-alive phase (see session_keepalive): republish cadences chosen so any
# 60s live-collection window (generate_floorplan's default duration) observes
# the full building cloud and the full multi-floor trajectory.
KEEPALIVE_MAP_PERIOD_S = 4.0  # /global_map republish period (latest-wins downstream)
KEEPALIVE_MAP_VOXEL_M = 0.12  # downsample so each /global_map message stays ~1M pts
KEEPALIVE_ODOM_HZ = 20.0  # trajectory pose replay rate
KEEPALIVE_ODOM_POSES = 1200  # decimated trajectory length (~60s at KEEPALIVE_ODOM_HZ)


def _rpy_to_matrix(r: float, p: float, y: float) -> np.ndarray:
    """Fixed-axis roll-pitch-yaw (URDF convention) -> rotation matrix."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _tmat(rotation: np.ndarray, translation: list[float]) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = translation
    return m


def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Rotation matrix via the shared Quaternion type (scipy-backed)."""
    return Quaternion(qx, qy, qz, qw).to_rotation_matrix()


def _matrix_to_quat(rot: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) via the shared Quaternion type.

    scipy's extraction is robust at 180-degree rotations, where the naive
    w-only formula (divide by 4w) hits w = 0 and silently emits NaNs into
    every published /odom pose and tf for that frame.
    """
    q = Quaternion.from_rotation_matrix(rot)
    return q.x, q.y, q.z, q.w


# Rig chain from the session's go2_mid360_rotated.urdf (fixed joints):
#   base_link -> front_camera -> {mid360_link -> mid360_gravity, camera_optical}
_T_BASE_FRONT = _tmat(np.eye(3), [0.32715, -3e-05, 0.04297])
_T_FRONT_MID = _tmat(_rpy_to_matrix(-0.785398, 0, -1.570796), [-0.032, 0, 0.12])
_T_MID_GRAVITY = _tmat(_rpy_to_matrix(0.0, -0.785398, 1.570796), [0, 0, 0])
T_BASE_GRAVITY = _T_BASE_FRONT @ _T_FRONT_MID @ _T_MID_GRAVITY
T_BASE_OPTICAL = _T_BASE_FRONT @ _tmat(_rpy_to_matrix(-np.pi / 2, 0, -np.pi / 2), [0, 0, 0])


@dataclass
class RrdFrame:
    ts: float
    kind: str  # "camera" | "lidar"
    data: Any  # (H,W,3) uint8 image, or (N,3) float32 points


def load_rrd_frames(rrd_path: str | Path) -> list[RrdFrame]:
    """Camera + lidar frames from the .rrd, sorted by timestamp.

    Extracts buffers via pyarrow's array API (ListScalar.values -> numpy)
    rather than `.to_pydict()`, which materializes every element as a
    separate Python object -- ruinous for a 1280x720x3 = 2.7M-byte image
    buffer repeated over 558 frames (minutes instead of seconds).

    The lidar frames returned here (/world/go2_lidar) are kept for
    inspection tooling, but feed_rrd_live does NOT use them -- see the
    module docstring for why they're unusable for world-frame localization.
    """
    import rerun_bindings as rb

    rec = rb.load_recording(str(rrd_path))
    frames: list[RrdFrame] = []
    for chunk in rec.chunks():
        if chunk.entity_path == "/world/camera/image":
            table = chunk.to_record_batch()
            ts = table.column("ts")[0].as_py().timestamp()
            fmt = table.column("Image:format").to_pylist()[0][0]
            w, h = fmt["width"], fmt["height"]
            # Image:buffer is list<item: list<item: uint8>> (one row -> one
            # single-element outer list -> the actual byte list).
            buf = table.column("Image:buffer")[0].values[0].values.to_numpy(zero_copy_only=False)
            arr = buf.astype(np.uint8).reshape(h, w, 3)
            frames.append(RrdFrame(ts=ts, kind="camera", data=arr))
        elif chunk.entity_path == "/world/go2_lidar":
            table = chunk.to_record_batch()
            ts = table.column("ts")[0].as_py().timestamp()
            # Points3D:positions is list<item: fixed_size_list<float>[3]>.
            flat = table.column("Points3D:positions")[0].values.values.to_numpy(zero_copy_only=False)
            pts = flat.astype(np.float32).reshape(-1, 3)
            frames.append(RrdFrame(ts=ts, kind="lidar", data=pts))
    frames.sort(key=lambda f: f.ts)
    return frames


def iter_mem2_camera_frames(
    mem2_db_path: str | Path,
    target_hz: float = DEFAULT_MEM2_CAMERA_HZ,
) -> Iterator[RrdFrame]:
    """Yield decimated camera frames straight from mem2.db's ``color_image``
    stream, lazily -- only one image is decoded/resident at a time (the feed
    gates on YOLO-E per frame anyway, so there's no benefit to materializing
    all ~16.7k frames, and doing so would cost multiple GB of RAM).

    This supersedes ``load_rrd_frames``' camera path: the mem2 images are the
    full-rate RGB stream (1280x720) on the *same* clock as gt_pointlio
    pose/scan, so camera<->lidar<->pose alignment is exact -- no cross-file
    timestamp/frame reconciliation, and far richer than the .rrd's decimated
    558 frames. Decimation keeps the first frame past each ``1/target_hz``
    gap (target_hz<=0 keeps every frame).
    """
    min_dt = 1.0 / target_hz if target_hz > 0 else 0.0
    store = SqliteStore(path=str(mem2_db_path), must_exist=True)
    store.start()
    try:
        last_ts: float | None = None
        for ts, img in store.replay().stream(CAMERA_STREAM).iterate_ts():
            if last_ts is not None and ts - last_ts < min_dt:
                continue
            last_ts = ts
            arr = img.data
            if img.format != ImageFormat.RGB:
                import cv2

                arr = cv2.cvtColor(img.to_cv2(), cv2.COLOR_BGR2RGB)
            yield RrdFrame(ts=ts, kind="camera", data=np.ascontiguousarray(arr))
    finally:
        store.stop()


@dataclass
class GtSession:
    """The mem2.db ground-truth pipeline: base poses + world-frame scans."""

    pose_ts: np.ndarray  # (P,)
    pose_mats: np.ndarray  # (P, 4, 4) world_T_base
    scan_ts: np.ndarray  # (S,)
    scan_world: list[np.ndarray]  # (Ni, 3) float32, already in world frame

    def world_t_base(self, ts: float) -> np.ndarray | None:
        i = int(np.searchsorted(self.pose_ts, ts))
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.pose_ts):
                if best is None or abs(self.pose_ts[j] - ts) < abs(self.pose_ts[best] - ts):
                    best = j
        if best is None or abs(self.pose_ts[best] - ts) > 0.5:
            return None
        return self.pose_mats[best]

    def window_cloud(self, ts: float, window_s: float = SCAN_WINDOW_S) -> np.ndarray | None:
        """World-frame accumulation of the scans in [ts - window_s, ts]."""
        hi = int(np.searchsorted(self.scan_ts, ts, side="right"))
        lo = int(np.searchsorted(self.scan_ts, ts - window_s))
        if hi <= lo:
            return None
        return np.vstack(self.scan_world[lo:hi])


def load_gt_session(mem2_db_path: str | Path) -> GtSession:
    """Load gt_pointlio odometry + scans, pre-transforming scans to world.

    Scans are stored sensor-local (despite their 'odom' frame_id label);
    each is placed with the nearest-in-time pose via the URDF gravity-frame
    chain. ~600MB of float32 for the full session -- fine for a test run.
    """
    store = SqliteStore(path=str(mem2_db_path), must_exist=True)
    store.start()
    try:
        odom = sorted(store.replay().stream(ODOMETRY_STREAM).iterate(), key=lambda o: o.ts)
        pose_ts = np.array([o.ts for o in odom])
        pose_mats = np.stack(
            [
                _tmat(
                    _quat_to_matrix(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w),
                    [o.position.x, o.position.y, o.position.z],
                )
                for o in odom
            ]
        )

        scan_ts: list[float] = []
        scan_world: list[np.ndarray] = []
        for ts, pc in store.replay().stream(SCAN_STREAM).iterate_ts():
            i = int(np.searchsorted(pose_ts, ts))
            j = min(
                (k for k in (i - 1, i) if 0 <= k < len(pose_ts)),
                key=lambda k: abs(pose_ts[k] - ts),
                default=None,
            )
            if j is None:
                continue
            pts, _ = pc.as_numpy()
            world_t_gravity = pose_mats[j] @ T_BASE_GRAVITY
            world = (world_t_gravity[:3, :3] @ pts.T).T + world_t_gravity[:3, 3]
            scan_ts.append(ts)
            scan_world.append(world.astype(np.float32))
    finally:
        store.stop()

    return GtSession(
        pose_ts=pose_ts,
        pose_mats=pose_mats,
        scan_ts=np.array(scan_ts),
        scan_world=scan_world,
    )


@dataclass
class FeedStats:
    camera_frames_published: int = 0
    lidar_clouds_published: int = 0
    tf_published: int = 0
    frames_without_pose: int = 0
    acks_received: int = 0
    ack_timeouts: int = 0


class _DetectionAcks:
    """Tracks the newest /detections_2d header timestamp seen (the module
    publishes one per processed camera frame, so it doubles as a per-frame
    processing ack)."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._latest_ts = 0.0
        self.count = 0

    def on_message(self, msg: Detection2DArray) -> None:
        with self._cond:
            self._latest_ts = max(self._latest_ts, msg.ts)
            self.count += 1
            self._cond.notify_all()

    def wait_for(self, ts: float, timeout_s: float) -> bool:
        """Wait until a detections_2d ack for camera ts (or newer) arrives."""
        deadline = time.monotonic() + timeout_s
        with self._cond:
            while self._latest_ts < ts - 1e-4:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._cond.wait(remaining)
            return True


def feed_rrd_live(
    rrd_path: str | Path | None,
    mem2_db_path: str | Path,
    publish_delay_s: float = PUBLISH_DELAY_S,
    session: GtSession | None = None,
    cam_frames: Iterable[RrdFrame] | None = None,
) -> FeedStats:
    """Publish the session onto the real LCM topics a live GO2Connection
    would use: per camera frame, a world-frame accumulated lidar window
    stamped at the camera's exact ts (every frame pairs cleanly in the
    consumer's camera/lidar alignment), tf for the same instant, and the
    image itself.

    Flow-controlled, not fixed-pace: the consumer wraps its aligned stream
    in backpressure(), which keeps only the newest pending pair -- a feed
    pushed faster than YOLO-E inference silently drops almost every frame.
    Before publishing camera frame N+1, the feeder waits for the module's
    /detections_2d message for frame N (the module publishes exactly one per
    processed camera frame, a natural ack). The gate sits one frame behind
    rather than immediately after publishing because align_timestamped only
    emits a pair once the lidar stream has progressed past the camera ts:
    the next frame's lidar publish is what releases the previous pair.

    Frame timestamps are preserved exactly as recorded, so alignment and tf
    lookups (which match on each message's own .ts, not wall-clock arrival)
    behave identically to a real-time feed; total wall time becomes
    ~(camera frames x YOLO-E inference time) instead of the recording's
    real ~19.6 min duration.
    """
    if cam_frames is None:
        if rrd_path is None:
            raise ValueError("feed_rrd_live needs either rrd_path or cam_frames")
        cam_frames = [f for f in load_rrd_frames(rrd_path) if f.kind == "camera"]
    if session is None:
        session = load_gt_session(mem2_db_path)

    color_image: LCMTransport[Image] = LCMTransport("/color_image", Image)
    lidar: LCMTransport[PointCloud2] = LCMTransport("/lidar", PointCloud2)
    camera_info: LCMTransport[CameraInfo] = LCMTransport("/camera_info", CameraInfo)
    odom: LCMTransport[PoseStamped] = LCMTransport("/odom", PoseStamped)
    detections_2d: LCMTransport[Detection2DArray] = LCMTransport(
        "/detections_2d", Detection2DArray
    )
    acks = _DetectionAcks()
    color_image.start()
    lidar.start()
    camera_info.start()
    odom.start()
    unsubscribe = detections_2d.subscribe(acks.on_message)
    tf = TF()

    # The session dir (mem2.db's parent) carries the recording's calibration.
    intrinsics = load_camera_intrinsics(Path(mem2_db_path).parent)
    info_msg = CameraInfo.from_intrinsics(frame_id="camera_optical", **intrinsics)

    opt_q = _matrix_to_quat(T_BASE_OPTICAL[:3, :3])
    stats = FeedStats()
    last_camera_info_pub = 0.0
    awaiting_ack_ts: float | None = None
    try:
        for frame in cam_frames:
            now = time.monotonic()
            if now - last_camera_info_pub >= CAMERA_INFO_PERIOD_S:
                camera_info.broadcast(None, info_msg)
                last_camera_info_pub = now

            world_t_base = session.world_t_base(frame.ts)
            cloud = session.window_cloud(frame.ts)
            if world_t_base is None or cloud is None:
                stats.frames_without_pose += 1
                continue

            if awaiting_ack_ts is not None:
                if acks.wait_for(awaiting_ack_ts, ACK_TIMEOUT_S):
                    stats.acks_received += 1
                else:
                    stats.ack_timeouts += 1
                awaiting_ack_ts = None

            base_q = _matrix_to_quat(world_t_base[:3, :3])
            tf.publish(
                Transform(
                    translation=Vector3(*world_t_base[:3, 3]),
                    rotation=Quaternion(*base_q),
                    frame_id="world",
                    child_frame_id="base_link",
                    ts=frame.ts,
                ),
                Transform(
                    translation=Vector3(*T_BASE_OPTICAL[:3, 3]),
                    rotation=Quaternion(*opt_q),
                    frame_id="base_link",
                    child_frame_id="camera_optical",
                    ts=frame.ts,
                ),
            )
            stats.tf_published += 1
            # Pose consumers (LidarSignalSkills, floorplan's pose tracker)
            # listen on /odom, not tf.
            odom.broadcast(None, _pose_msg(world_t_base, frame.ts))

            pc = PointCloud2.from_numpy(
                cloud.astype(np.float64), frame_id="world", timestamp=frame.ts
            )
            lidar.broadcast(None, pc)
            stats.lidar_clouds_published += 1
            time.sleep(publish_delay_s)

            img = Image.from_numpy(
                frame.data, format=ImageFormat.RGB, frame_id="camera_optical", ts=frame.ts
            )
            color_image.broadcast(None, img)
            stats.camera_frames_published += 1
            awaiting_ack_ts = frame.ts
            time.sleep(publish_delay_s)

        # Wait out the final frame too, so callers know the whole feed has
        # been processed (not just published) when this returns.
        if awaiting_ack_ts is not None:
            if acks.wait_for(awaiting_ack_ts, ACK_TIMEOUT_S):
                stats.acks_received += 1
            else:
                stats.ack_timeouts += 1
    finally:
        unsubscribe()
        color_image.stop()
        lidar.stop()
        camera_info.stop()
        odom.stop()
        detections_2d.stop()
        tf.stop()

    return stats


def _pose_msg(world_t_base: np.ndarray, ts: float) -> PoseStamped:
    q = _matrix_to_quat(world_t_base[:3, :3])
    return PoseStamped(
        ts=ts,
        frame_id="world",
        position=[float(v) for v in world_t_base[:3, 3]],
        orientation=list(q),
    )


@contextlib.contextmanager
def session_keepalive(session: GtSession) -> Iterator[None]:
    """Keep the recorded session "live" on LCM while the caller runs Q&A.

    The feed itself ends before the questions start, but several skills
    collect *live* data at call time: generate_floorplan gathers /lidar +
    /global_map + /odom for its `duration` window, and LidarSignalSkills'
    robot-relative queries need /odom. Inside this context a daemon thread
    republishes:

    - /global_map every KEEPALIVE_MAP_PERIOD_S: the full-session cloud,
      voxel-downsampled once on entry. Downstream LidarPointCloudClient
      treats global_map as latest-wins (not accumulated), so repeats are
      safe, and any collection window sees the entire building.
    - /odom at KEEPALIVE_ODOM_HZ: cycling through a decimated copy of the
      full tour trajectory (~KEEPALIVE_ODOM_POSES poses / one cycle per
      ~60s), so a collection window observes the multi-floor trajectory that
      floor detection needs. Note this makes "the robot's current position"
      sweep the recording -- ego-now questions are meaningless against a
      recorded tour anyway (use the recorded question set).
    """
    import open3d as o3d  # type: ignore[import-untyped]

    full = np.vstack(session.scan_world)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(full.astype(np.float64))
    down = np.asarray(pcd.voxel_down_sample(KEEPALIVE_MAP_VOXEL_M).points)
    map_msg = PointCloud2.from_numpy(down, frame_id="world", timestamp=float(session.scan_ts[-1]))

    pose_idx = np.unique(
        np.linspace(0, len(session.pose_ts) - 1, KEEPALIVE_ODOM_POSES).astype(int)
    )
    pose_msgs = [
        _pose_msg(session.pose_mats[i], float(session.pose_ts[i])) for i in pose_idx
    ]

    global_map: LCMTransport[PointCloud2] = LCMTransport("/global_map", PointCloud2)
    odom: LCMTransport[PoseStamped] = LCMTransport("/odom", PoseStamped)
    global_map.start()
    odom.start()

    stop = threading.Event()

    def run() -> None:
        last_map_pub = 0.0
        i = 0
        while not stop.is_set():
            now = time.monotonic()
            if now - last_map_pub >= KEEPALIVE_MAP_PERIOD_S:
                global_map.broadcast(None, map_msg)
                last_map_pub = now
            odom.broadcast(None, pose_msgs[i % len(pose_msgs)])
            i += 1
            stop.wait(1.0 / KEEPALIVE_ODOM_HZ)

    thread = threading.Thread(target=run, name="rrd-feed-keepalive", daemon=True)
    thread.start()
    try:
        yield
    finally:
        stop.set()
        thread.join(timeout=5.0)
        global_map.stop()
        odom.stop()
