"""RealSense D435i → world-frame depth map using on-board IMU + point-cloud odometry.

VIO pipeline:
  MadgwickFilter         — D435i accel + gyro → orientation R  (every frame, ~1 ms)
  PointCloudOdometry     — translation-only ICP against previous world cloud
                           (runs in the map thread; live cloud uses t from last frame)

World-frame map:
  Same FastVoxelMap as the working ZED implementation. Each frame contributes
      xyz_world = xyz_cam @ R.T + t
  with R from IMU and t from ICP. As the camera moves, new voxels are added and
  previously seen areas receive additional observations at the same world positions.

Notes:
  - Accel + gyro require D435i firmware ≥ 5.12.  Both streams are enabled below.
  - IMU extrinsics (accel → depth optical) are read from the SDK and applied so
    Madgwick operates directly in camera_link frame (X-fwd, Y-left, Z-up).
  - ICP tracks translation with a one-frame lag — negligible at 15 fps.
  - For full drift-free world-frame mapping, pair with a T265 and swap in
    rs.stream.pose; the rest of the pipeline is unchanged.

Usage:
    python -m dimos.navigation.camera_nav.realsense_depth_map
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from scipy.ndimage import sobel
from scipy.spatial import cKDTree

if TYPE_CHECKING:
    import pyrealsense2 as rs


# ── Constants ─────────────────────────────────────────────────────────────────

_VOFF  = np.int64(100_000)
_VMASK = np.int64(0x3FFFF)

# Camera-relative height band (Z in camera_link = up direction from IMU).
# Height is measured relative to the camera, not world origin, so it is
# unaffected by absolute position error in the translation estimate.
_Z_REL_LO:    float = -1.4
_Z_REL_HI:    float =  1.5
_VOX_SIZE:    float =  0.020
_FLOOR_RAY_Z: float = -0.15   # tighter than ZED default: rejects rays >~9° below horizontal
_MAP_MAX_DEPTH: float = 4.5   # D435i stereo accuracy degrades beyond ~5 m — cap map pts here

# Optical frame (X=right, Y=down, Z=depth) → camera_link (X=fwd, Y=left, Z=up)
_R_OPT_TO_LINK = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)


# ── Helpers ───────────────────────────────────────────────────────────────────

def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + _VOFF) & _VMASK
    return (v[:, 0] << np.int64(36)) | (v[:, 1] << np.int64(18)) | v[:, 2]


def _height_color(z_rel: np.ndarray) -> np.ndarray:
    t = np.clip((z_rel - _Z_REL_LO) / (_Z_REL_HI - _Z_REL_LO), 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0.0, 1.0)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0.0, 1.0)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0.0, 1.0)
    return (np.column_stack([r, g, b]) * 255).astype(np.uint8)


def _filter_isolated(xyz: np.ndarray, voxel: float = 0.05, min_pts: int = 2) -> np.ndarray:
    if len(xyz) < min_pts:
        return np.zeros((0, 3), dtype=np.float32)
    vk = np.floor(xyz / voxel).astype(np.int32)
    _, inv, cnt = np.unique(_pack(vk), return_inverse=True, return_counts=True)
    return xyz[cnt[inv] >= min_pts]


def _nearest_per_ray(xyz: np.ndarray, cam_pos: np.ndarray, deg: float = 3.0) -> np.ndarray:
    """Keep only the nearest point per ~3° angular bin.

    Stereo cameras leak background depth for pixels near an obstacle edge —
    the two cameras see slightly different scenes and the matcher interpolates.
    These spurious behind-surface returns share the same ray direction as the
    real surface but sit at greater depth. Sorting by depth and keeping the
    nearest per bin discards them, leaving only actual obstacle surfaces.
    """
    if len(xyz) == 0:
        return xyz
    rays  = xyz - cam_pos
    dists = np.linalg.norm(rays, axis=1)
    ok    = dists > 0
    if not ok.any():
        return xyz[ok]

    idx_ok = np.where(ok)[0]
    dirs   = rays[ok] / dists[ok, None]

    rad = np.radians(deg)
    az  = np.floor(np.arctan2(dirs[:, 1], dirs[:, 0]) / rad).astype(np.int32)
    el  = np.floor(np.arcsin(np.clip(dirs[:, 2], -1.0, 1.0)) / rad).astype(np.int32)
    keys = az.astype(np.int64) * 100_000 + el.astype(np.int64)

    order = np.argsort(dists[ok])
    _, first = np.unique(keys[order], return_index=True)
    return xyz[idx_ok[order[first]]]


# ── Gradient stability filter ─────────────────────────────────────────────────

@dataclass
class GradientStabilityConfig:
    gradient_threshold: float = 0.30


class GradientStabilityFilter:
    def __init__(self, cfg: GradientStabilityConfig | None = None) -> None:
        self.cfg = cfg or GradientStabilityConfig()

    def compute(self, depth: np.ndarray) -> np.ndarray:
        valid    = np.isfinite(depth)
        depth_f  = np.where(valid, depth, 0.0).astype(np.float64)
        grad_mag = np.hypot(sobel(depth_f, axis=1), sobel(depth_f, axis=0))
        return valid & (grad_mag < self.cfg.gradient_threshold)


# ── Depth frame packet ────────────────────────────────────────────────────────

@dataclass
class DepthFramePacket:
    timestamp:  float
    depth:      np.ndarray       # H×W float32, metres, NaN=invalid
    intrinsics: np.ndarray       # [fx, fy, cx, cy] float64
    width:      int
    height:     int
    pose_R:     np.ndarray       # 3×3 float32  camera_link → world
    pose_t:     np.ndarray       # (3,) float32 camera world position
    colors_hw:  np.ndarray | None = field(default=None)  # H×W×3 uint8 RGB

    @property
    def K(self) -> np.ndarray:
        fx, fy, cx, cy = self.intrinsics
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


# ── IMU orientation — Madgwick AHRS ──────────────────────────────────────────

class MadgwickFilter:
    """Attitude estimation from gyroscope + accelerometer.

    Fuses gyro integration (fast, drifts over minutes) with gravity direction
    from accelerometer (stable, noisy during motion).  Outputs R: the rotation
    from the sensor frame to world (Z-up).

    beta: gradient-descent gain. 0.033 is a conservative default.
    Increase toward 0.1 if orientation drifts during slow motion.
    """

    def __init__(self, beta: float = 0.033) -> None:
        self._q    = np.array([1., 0., 0., 0.], dtype=np.float64)  # w x y z
        self._beta = beta
        self._t_prev: float | None = None

    def update(self, gyro: np.ndarray, accel: np.ndarray, t: float) -> None:
        """gyro: rad/s (x,y,z),  accel: m/s² (x,y,z),  t: monotonic seconds."""
        if self._t_prev is None:
            self._t_prev = t
            return
        dt = min(t - self._t_prev, 0.1)
        self._t_prev = t
        if dt <= 0:
            return

        q0, q1, q2, q3 = self._q
        gx, gy, gz = gyro.astype(np.float64)

        # Accelerometer correction (skip during near-freefall or strong vibration)
        a_n = np.linalg.norm(accel)
        if a_n > 0.5:
            ax, ay, az = accel.astype(np.float64) / a_n
            # Objective: align predicted gravity (R^T * [0,0,1]) with measured accel
            f1 = 2*(q1*q3 - q0*q2) - ax
            f2 = 2*(q0*q1 + q2*q3) - ay
            f3 = 2*(0.5 - q1**2 - q2**2) - az
            J  = np.array([
                [-2*q2,  2*q3, -2*q0, 2*q1],
                [ 2*q1,  2*q0,  2*q3, 2*q2],
                [    0, -4*q1, -4*q2,    0],
            ])
            grad = J.T @ np.array([f1, f2, f3])
            gn   = np.linalg.norm(grad)
            if gn > 1e-9:
                grad /= gn
        else:
            grad = np.zeros(4)

        q_dot = 0.5 * np.array([
            -q1*gx - q2*gy - q3*gz,
             q0*gx + q2*gz - q3*gy,
             q0*gy - q1*gz + q3*gx,
             q0*gz + q1*gy - q2*gx,
        ]) - self._beta * grad

        self._q = self._q + q_dot * dt
        self._q /= np.linalg.norm(self._q) + 1e-12

    @property
    def R(self) -> np.ndarray:
        """Rotation: sensor_frame → world (Z-up)."""
        q0, q1, q2, q3 = self._q
        return np.array([
            [1-2*(q2**2+q3**2), 2*(q1*q2-q0*q3),   2*(q1*q3+q0*q2)  ],
            [2*(q1*q2+q0*q3),   1-2*(q1**2+q3**2), 2*(q2*q3-q0*q1)  ],
            [2*(q1*q3-q0*q2),   2*(q2*q3+q0*q1),   1-2*(q1**2+q2**2)],
        ], dtype=np.float32)


# ── Translation odometry — rotation-decoupled ICP ────────────────────────────

class PointCloudOdometry:
    """Frame-to-frame translation via ICP with rotation already applied by IMU.

    Each frame's camera_link points are first rotated to a gravity-aligned
    frame using R from MadgwickFilter, then shifted by t to reach world frame.
    ICP finds the shift by aligning current gravity-aligned pts to the previous
    world-frame cloud.  Converges in 3–4 iterations since rotation is pre-applied.

    Updated in the map thread (daemon) so it does not slow live-cloud rendering.
    Live cloud uses t from the previous frame — one-frame lag, ~67 ms at 15 fps.
    """

    ITERS    = 4
    MAX_DIST = 0.40    # m  — reject correspondences further than this
    N_SRC    = 1_500   # subsample from current frame
    N_DST    = 8_000   # subsample stored from previous world cloud
    N_STORE  = 10_000  # how many pts to keep as "previous world cloud"

    def __init__(self) -> None:
        self._t          = np.zeros(3, dtype=np.float32)
        self._lock       = threading.Lock()
        self._prev_world: np.ndarray | None = None

    @property
    def t(self) -> np.ndarray:
        with self._lock:
            return self._t.copy()

    def update(self, xyz_cam: np.ndarray, R: np.ndarray) -> np.ndarray:
        """Estimate world-frame translation for the current frame.

        xyz_cam: (N, 3) float32, camera_link-frame stable filtered points
        R:       (3, 3) float32, orientation from MadgwickFilter
        Returns: (3,) float32 world translation
        """
        # Rotate to gravity-aligned frame (no translation yet)
        pts_ga = (xyz_cam @ R.T).astype(np.float32)

        with self._lock:
            t_est = self._t.copy()

        if self._prev_world is None or len(pts_ga) < 50:
            t_new = t_est
        else:
            n_src = min(self.N_SRC, len(pts_ga))
            n_dst = min(self.N_DST, len(self._prev_world))
            src   = pts_ga[np.random.choice(len(pts_ga), n_src, replace=False)]
            dst   = self._prev_world[np.random.choice(len(self._prev_world), n_dst, replace=False)]
            tree  = cKDTree(dst)

            for _ in range(self.ITERS):
                dists, idx = tree.query(src + t_est, k=1, workers=1)
                mask = dists < self.MAX_DIST
                if mask.sum() < 30:
                    break
                # Centroid shift of matched pairs = translation correction
                delta = dst[idx[mask]].mean(axis=0) - (src[mask] + t_est).mean(axis=0)
                t_est = (t_est + 0.7 * delta).astype(np.float32)  # damped update

            t_new = t_est

        # Store current world-frame pts as reference for next frame
        n_st  = min(self.N_STORE, len(pts_ga))
        idx_s = np.random.choice(len(pts_ga), n_st, replace=False)
        self._prev_world = (pts_ga[idx_s] + t_new).astype(np.float32)

        with self._lock:
            self._t = t_new
        return t_new


# ── Backprojector ─────────────────────────────────────────────────────────────

class DepthBackprojector:
    """Pinhole backproject → camera_link frame → world frame.

    Returns (xyz_world, xyz_cam, colors):
      xyz_cam:   camera_link-frame pts — passed to map worker for ICP
      xyz_world: world-frame pts — used for live-cloud display
    """

    def project(
        self,
        pkt:    DepthFramePacket,
        stable: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray | None]:
        valid = np.isfinite(pkt.depth) if stable is None else (np.isfinite(pkt.depth) & stable)
        d = pkt.depth
        H, W = d.shape
        fx, fy, cx, cy = pkt.intrinsics
        uu, vv = np.meshgrid(np.arange(W, dtype=np.float32),
                              np.arange(H, dtype=np.float32))
        dd      = d[valid]
        xyz_opt = np.column_stack([(uu[valid] - cx) * dd / fx,
                                   (vv[valid] - cy) * dd / fy,
                                   dd]).astype(np.float32)
        xyz_cam   = xyz_opt @ _R_OPT_TO_LINK.T                                # optical → camera_link
        xyz_world = (xyz_cam @ pkt.pose_R.T + pkt.pose_t).astype(np.float32)  # → world
        colors    = pkt.colors_hw[valid] if pkt.colors_hw is not None else None
        return xyz_world, colors


# ── Voxel map (same as ZED — the one that works) ─────────────────────────────

class FastVoxelMap:
    """Append-only world-frame point map backed entirely by numpy arrays."""

    COMPACT_EVERY: int = 30

    def __init__(self, voxel_size: float = 0.05, capacity: int = 500_000) -> None:
        self._v      = voxel_size
        self._cap    = capacity
        self._pts    = np.empty((capacity, 3), dtype=np.float32)
        self._n      = 0
        self._n_adds = 0

    def add(self, xyz: np.ndarray) -> None:
        if len(xyz) == 0:
            return
        vk = np.floor(xyz / self._v).astype(np.int32)
        _, first = np.unique(_pack(vk), return_index=True)
        new_pts = xyz[first]
        if self._n + len(new_pts) > self._cap:
            self._compact()
        if self._n + len(new_pts) > self._cap:
            self._grow()
        s = slice(self._n, self._n + len(new_pts))
        self._pts[s] = new_pts
        self._n += len(new_pts)
        self._n_adds += 1
        if self._n_adds % self.COMPACT_EVERY == 0:
            self._compact()

    def _compact(self) -> None:
        if self._n == 0:
            return
        vk = np.floor(self._pts[:self._n] / self._v).astype(np.int32)
        _, idx = np.unique(_pack(vk), return_index=True)
        n_u = len(idx)
        self._pts[:n_u] = self._pts[:self._n][idx]
        self._n = n_u

    def _grow(self) -> None:
        cap2 = self._cap * 2
        buf  = np.empty((cap2, 3), dtype=np.float32)
        buf[:self._n] = self._pts[:self._n]
        self._pts = buf
        self._cap = cap2

    def points(self) -> np.ndarray:
        return self._pts[:self._n]

    @property
    def count(self) -> int:
        return self._n


# ── RealSense source with IMU ─────────────────────────────────────────────────

class RealSenseDepthSource:
    """D435i → DepthFramePacket with VIO pose from IMU + point-cloud odometry.

    Enables accel + gyro streams alongside depth + color.  IMU extrinsics are
    read from the SDK so MadgwickFilter operates in camera_link frame directly.
    Translation comes from PointCloudOdometry (updated in the map thread,
    one frame behind — see DepthStreamer._map_worker).
    """

    MIN_DEPTH: float = 0.15
    MAX_DEPTH: float = 8.0

    def __init__(self, width: int = 848, height: int = 480, fps: int = 15) -> None:
        import pyrealsense2 as rs

        self._width  = width
        self._height = height

        self._pipeline = rs.pipeline()

        # Try to start with IMU streams (D435i).  Falls back to depth+colour only
        # if the device has no IMU (D435) or the firmware rejects the request.
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16,  fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f)   # let SDK pick fps
        cfg.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f)

        try:
            profile       = self._pipeline.start(cfg)
            self._has_imu = True
            print("IMU streams enabled (D435i) — orientation tracking active")
        except RuntimeError:
            cfg2 = rs.config()
            cfg2.enable_stream(rs.stream.depth, width, height, rs.format.z16,  fps)
            cfg2.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            profile       = self._pipeline.start(cfg2)
            self._has_imu = False
            print("IMU not available (D435?) — world-frame tracking disabled, using identity pose")

        self._depth_scale = (
            profile.get_device().first_depth_sensor().get_depth_scale()
        )
        self._align = rs.align(rs.stream.depth)

        # Intrinsics: depth stream (color is warped to it, preserving full depth FOV)
        intr = (profile.get_stream(rs.stream.depth)
                       .as_video_stream_profile()
                       .get_intrinsics())
        self._intrinsics = np.array(
            [intr.fx, intr.fy, intr.ppx, intr.ppy], dtype=np.float64
        )

        # Store stream enum refs so read() can use them without re-importing
        self._stream_gyro  = rs.stream.gyro
        self._stream_accel = rs.stream.accel

        if self._has_imu:
            # IMU-to-camera_link rotation: accel → depth-optical → camera_link
            ext            = (profile.get_stream(rs.stream.accel)
                                     .get_extrinsics_to(profile.get_stream(rs.stream.depth)))
            R_imu_to_depth = np.array(ext.rotation, dtype=np.float32).reshape(3, 3)
            self._R_imu_to_link = _R_OPT_TO_LINK @ R_imu_to_depth
            self._madgwick      = MadgwickFilter(beta=0.033)
        else:
            self._R_imu_to_link = None
            self._madgwick      = None

        self.odom         = PointCloudOdometry()
        self._pose_locked = not self._has_imu   # no IMU → identity pose is always "locked"

    @property
    def pose_locked(self) -> bool:
        return self._pose_locked

    def read(self, ts: float) -> DepthFramePacket:
        frames  = self._pipeline.wait_for_frames()
        aligned = self._align.process(frames)

        # ── IMU: update orientation ────────────────────────────────────
        if self._has_imu:
            gyro_f  = frames.first_or_default(self._stream_gyro)
            accel_f = frames.first_or_default(self._stream_accel)
            if gyro_f and accel_f:
                g = gyro_f.as_motion_frame().get_motion_data()
                a = accel_f.as_motion_frame().get_motion_data()
                gyro_imu  = np.array([g.x, g.y, g.z], dtype=np.float32)
                accel_imu = np.array([a.x, a.y, a.z], dtype=np.float32)
                gyro_link  = self._R_imu_to_link @ gyro_imu
                accel_link = self._R_imu_to_link @ accel_imu
                self._madgwick.update(gyro_link, accel_link, ts)
                if not self._pose_locked:
                    print("*** IMU locked ***")
                    self._pose_locked = True

        R = self._madgwick.R.copy() if self._has_imu else np.eye(3, dtype=np.float32)
        t = self.odom.t                     # translation from previous ICP update

        # ── Depth ──────────────────────────────────────────────────────
        depth = np.asarray(
            aligned.get_depth_frame().get_data(), dtype=np.float32
        ) * self._depth_scale
        invalid = (depth <= 0) | (depth < self.MIN_DEPTH) | (depth > self.MAX_DEPTH)
        depth[invalid] = np.nan

        # ── Color ──────────────────────────────────────────────────────
        bgr       = np.asarray(aligned.get_color_frame().get_data(), dtype=np.uint8)
        colors_hw = bgr[:, :, ::-1].copy()

        return DepthFramePacket(
            timestamp  = ts,
            depth      = depth,
            intrinsics = self._intrinsics,
            width      = self._width,
            height     = self._height,
            pose_R     = R,
            pose_t     = t,
            colors_hw  = colors_hw,
        )

    def stop(self) -> None:
        self._pipeline.stop()


# ── Streamer ──────────────────────────────────────────────────────────────────

class DepthStreamer:
    """Two-layer pipeline — mirrors the ZED implementation that worked.

    Main thread  (fast): gradient filter → backproject → live Rerun cloud
    Map thread (async):  height/floor/isolation filter → ICP (updates translation)
                         → xyz_world = xyz_cam @ R.T + t → FastVoxelMap → Rerun map
    """

    MAX_CLOUD  = 50_000
    MAX_MAP    = 200_000
    MAP_EVERY  = 5

    def __init__(
        self,
        source:      RealSenseDepthSource,
        backproj:    DepthBackprojector,
        grad_filter: GradientStabilityFilter | None = None,
    ) -> None:
        self._src            = source
        self._bp             = backproj
        self._grad           = grad_filter or GradientStabilityFilter()
        self._vox            = FastVoxelMap(voxel_size=0.05)
        self._vox_lock       = threading.Lock()
        self._live_vox       = FastVoxelMap(voxel_size=0.05)   # live_cloud_map accumulator
        self._last_n_valid   = 0
        self._last_n_stable  = 0
        self._last_n_vox     = 0
        self._pinhole_logged = False
        self._map_queue: queue.Queue = queue.Queue(maxsize=16)
        self._map_thread = threading.Thread(target=self._map_worker, daemon=True)
        self._map_thread.start()

    def assemble(self, ts: float) -> DepthFramePacket:
        return self._src.read(ts)

    def process(self, pkt: DepthFramePacket, frame: int) -> None:
        rr.set_time("frame", sequence=frame)
        if not self._pinhole_logged:
            rr.log("world/camera", rr.Pinhole(
                image_from_camera=pkt.K, width=pkt.width, height=pkt.height,
            ), static=True)
            self._pinhole_logged = True

        rr.log("world/camera/depth", rr.DepthImage(pkt.depth, meter=1.0))

        stable_mask = self._grad.compute(pkt.depth)
        self._last_n_valid  = int(np.isfinite(pkt.depth).sum())
        self._last_n_stable = int(stable_mask.sum())

        xyz, colors = self._bp.project(pkt, stable_mask)
        if len(xyz) == 0:
            return

        # Live cloud — instant, no map ops
        cam_z = float(pkt.pose_t[2])
        n     = min(len(xyz), self.MAX_CLOUD)
        idx   = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(n)
        cloud_colors = (colors[idx] if colors is not None
                        else _height_color(xyz[idx, 2] - cam_z))
        rr.log("world/cloud", rr.Points3D(positions=xyz[idx], colors=cloud_colors, radii=0.003))

        # Raw cloud (no gradient filter) — for comparison
        xyz_raw, colors_raw = self._bp.project(pkt, stable=None)
        if len(xyz_raw):
            n_r   = min(len(xyz_raw), self.MAX_CLOUD)
            idx_r = np.random.choice(len(xyz_raw), n_r, replace=False) if len(xyz_raw) > n_r else np.arange(len(xyz_raw))
            rr.log("world/raw_cloud", rr.Points3D(
                positions=xyz_raw[idx_r],
                colors=colors_raw[idx_r] if colors_raw is not None else _height_color(xyz_raw[idx_r, 2] - cam_z),
                radii=0.003,
            ))

        # Per-frame voxel map — mirrors ZED world/map pipeline exactly
        h_rel    = xyz[:, 2] - cam_z
        # Histogram-based floor plane detection.
        # Percentile is unreliable when few floor pixels are visible (angled camera,
        # etc.) — the histogram finds the *densest* Z cluster in the lower half of
        # the scene, which is always the floor regardless of what fraction of the
        # frame it occupies.  This is 1D RANSAC with a known normal (IMU gives us
        # world Z = up, so the floor plane normal is always [0,0,1]).
        z        = xyz[:, 2]
        z_lo     = float(z.min())
        z_search = float(np.percentile(z, 40))
        if z_search - z_lo > 0.05:
            hist, edges = np.histogram(z[(z >= z_lo) & (z <= z_search)], bins=30)
            floor_z = float(edges[np.argmax(hist) + 1]) + 0.15
        else:
            floor_z = z_lo + 0.15
        keep     = (xyz[:, 2] > floor_z) & (h_rel <= _Z_REL_HI)
        xyz_kept = xyz[keep]
        if len(xyz_kept):
            vk       = np.floor(xyz_kept / _VOX_SIZE).astype(np.int32)
            _, first = np.unique(_pack(vk), return_index=True)
            xyz_vox  = xyz_kept[first]
            self._last_n_vox = len(xyz_vox)
            rr.log("world/map", rr.Points3D(
                positions=xyz_vox,
                colors=_height_color(xyz_vox[:, 2] - cam_z),
                radii=0.010,
            ))
        else:
            self._last_n_vox = 0

    def _map_worker(self) -> None:
        while True:
            item = self._map_queue.get()
            if item is None:
                return
            xyz_world, cam_pos, cam_z, R, frame = item

            # ── Identical to ZED map worker ───────────────────────────
            h_rel    = xyz_world[:, 2] - cam_z
            rays     = xyz_world - cam_pos
            dist     = np.linalg.norm(rays, axis=1)
            d_z_norm = np.where(dist > 0, rays[:, 2] / dist, 0.0)
            keep     = (h_rel >= _Z_REL_LO) & (h_rel <= _Z_REL_HI) & (d_z_norm > _FLOOR_RAY_Z)
            xyz_map  = _filter_isolated(xyz_world[keep])
            if len(xyz_map) == 0:
                continue

            with self._vox_lock:
                self._vox.add(xyz_map)

            if frame % self.MAP_EVERY == 0:
                rr.set_time("frame", sequence=frame)
                self._log_map(cam_z)

            # ── ICP: update translation estimate for next frame ───────
            # Recover camera-link pts from world-frame: xyz_cam = (xyz_world - t) @ R
            xyz_cam_filt = (xyz_map - cam_pos) @ R
            self._src.odom.update(xyz_cam_filt, R)

    def _log_live_map(self, cam_z: float = 0.0, frame: int = 0) -> None:
        """Log accumulated obstacle-surface world map to Rerun."""
        pts = self._live_vox.points()
        if len(pts) == 0:
            return
        n   = min(len(pts), self.MAX_MAP)
        idx = np.random.choice(len(pts), n, replace=False) if len(pts) > n else np.arange(n)
        sub = pts[idx].copy()
        rr.log("world/live_cloud_map", rr.Points3D(
            positions=sub,
            colors=_height_color(sub[:, 2] - cam_z),
            radii=0.006,
        ))
        if frame % 30 == 0:
            mn, mx = pts.min(axis=0), pts.max(axis=0)
            print(
                f"  map={len(pts):6d} vox  "
                f"x=[{mn[0]:+.1f},{mx[0]:+.1f}] "
                f"y=[{mn[1]:+.1f},{mx[1]:+.1f}] "
                f"z=[{mn[2]:+.1f},{mx[2]:+.1f}]",
                flush=True,
            )

    def _log_map(self, cam_z: float = 0.0) -> None:
        with self._vox_lock:
            pts = self._vox.points().copy()
        if len(pts) == 0:
            return
        n   = min(len(pts), self.MAX_MAP)
        idx = np.random.choice(len(pts), n, replace=False) if len(pts) > n else np.arange(n)
        # world/map disabled — replaced by world/live_cloud_map above
        # rr.log("world/map", rr.Points3D(
        #     positions=pts[idx],
        #     colors=_height_color(pts[idx, 2] - cam_z),
        #     radii=0.005,
        # ))

    def log_stdout(self, pkt: DepthFramePacket, frame: int, fps: float) -> None:
        with self._vox_lock:
            n_map = self._vox.count
        t = self._src.odom.t
        pct = 100 * self._last_n_stable / self._last_n_valid if self._last_n_valid > 0 else 0.0
        vio = "LOCKED" if self._src.pose_locked else "init"
        print(
            f"frame={frame:5d}  "
            f"stable={self._last_n_stable:6d}/{self._last_n_valid:6d} ({pct:.0f}%)  "
            f"vox={self._last_n_vox:5d}  "
            f"t=[{t[0]:+.2f},{t[1]:+.2f},{t[2]:+.2f}]  "
            f"vio={vio}  fps={fps:.1f}",
            flush=True,
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    rr.init("realsense_depth_map", spawn=True)
    rr.send_blueprint(rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(name="live cloud", origin="world",
                              contents=["world/cloud", "world/camera/**"]),
            rrb.Spatial3DView(name="raw cloud", origin="world",
                              contents=["world/raw_cloud", "world/camera/**"]),
            rrb.Spatial3DView(name="voxel map", origin="world",
                              contents=["world/map"]),
        )
    ))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/cloud", rr.Points3D([[0, 0, 0]]), static=True)
    # live_cloud_map is logged temporally every frame — no static seed needed

    src = RealSenseDepthSource(width=848, height=480, fps=15)
    print("RealSense open — Ctrl-C to quit.")
    print("Hold still for 2–3 seconds while IMU initialises orientation.")

    streamer = DepthStreamer(
        src,
        DepthBackprojector(),
        GradientStabilityFilter(GradientStabilityConfig(gradient_threshold=0.30)),
    )

    frame = 0
    t0    = time.monotonic()

    try:
        while True:
            ts  = time.monotonic()
            pkt = streamer.assemble(ts)
            streamer.process(pkt, frame)
            streamer.log_stdout(pkt, frame, frame / max(ts - t0, 1e-6))
            frame += 1
    except KeyboardInterrupt:
        pass

    src.stop()
    print("done")


if __name__ == "__main__":
    main()
