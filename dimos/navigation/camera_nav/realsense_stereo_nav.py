"""RealSense D435i stereo depth pipeline — per-frame voxel map + persistent global map.

VIO pipeline:
  MadgwickFilter        — D435i accel + gyro → orientation R  (~1 ms per frame)
  PointCloudOdometry    — rotation-decoupled ICP for translation t
  RealSenseDepthSource  — depth + colour frames with IMU at native sensor rate
  _FloorCalibrator      — one-shot floor-height estimator for above-floor filtering

Two Rerun channels:
  world/map        per-frame voxel grid — refreshes every frame, never accumulates
  world/global_map persistent panoramic map — grows as the camera explores

Coordinate frames:
  Per-frame map    world frame: xyz_world = xyz_cam @ R.T + t
                   R from Madgwick IMU, t from ICP. Refreshed each frame so
                   ICP noise has no cumulative effect.

  Global map       rotation-only frame: xyz_ronly = xyz_world - t = xyz_cam @ R.T
                   ICP t oscillates ±2 cm even for a static camera (centroid
                   sub-sampling noise). At 2 cm voxels that maps the same surface
                   to different keys every frame → unbounded accumulation. Stripping
                   t gives a stable key: same camera orientation = same voxel, always.
                   The floor threshold is locked once at map-start so ICP Z drift
                   cannot corrupt it later.

Notes:
  - IMU is opened via the sensor API (not the pipeline synchroniser) so gyro
    fires at its native ~200 Hz rather than being throttled to the video rate.
    Without this, Madgwick barely integrates yaw — fast pans can be missed.
  - ICP translation trails by one frame (~67 ms at 15 fps) — negligible.
  - Accel + gyro require D435i firmware >= 5.12.

Usage:
    python -m dimos.navigation.camera_nav.realsense_stereo_nav
"""

from __future__ import annotations

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


class _FloorCalibrator:
    """One-shot floor height estimator using camera_link frame (Z always up).

    Works independently of Madgwick convergence because camera_link Z is defined
    by the fixed optical→link rotation, not by IMU attitude.  Skips the first
    SKIP frames so the IMU settles, then histograms the depth of floor-candidate
    points over CALIB frames and takes the mode — more robust than a percentile
    because sub-floor stereo noise has fewer hits than the real floor plane.

    After calibration, `floor_z` is the camera_link Z of the floor (negative,
    since floor is below the camera), and `cam_height` is the positive distance.
    """

    SKIP_FRAMES  = 30
    CALIB_FRAMES = 30
    BIN_M        = 0.02   # 2 cm histogram bins

    def __init__(self) -> None:
        self._frame   = 0
        self._samples: list[float] = []
        self.floor_z:   float | None = None  # camera_link Z of floor (negative)
        self.cam_height: float | None = None  # positive metres above floor

    @property
    def ready(self) -> bool:
        return self.floor_z is not None

    def update(self, xyz_cam: np.ndarray) -> None:
        self._frame += 1
        if self.ready or self._frame <= self.SKIP_FRAMES:
            return

        # Candidate floor points: at least 30 cm below camera, close enough
        # to have reliable stereo (within 4 m lateral distance)
        z    = xyz_cam[:, 2]
        dist = np.linalg.norm(xyz_cam[:, :2], axis=1)
        mask = (z < -0.30) & (dist < 4.0)
        if mask.sum() < 200:
            return

        z_floor = z[mask]
        lo, hi  = z_floor.min(), z_floor.max()
        if hi - lo < self.BIN_M:
            return
        bins          = np.arange(lo, hi + self.BIN_M, self.BIN_M)
        counts, edges = np.histogram(z_floor, bins=bins)
        mode_z        = float(edges[int(np.argmax(counts))] + self.BIN_M / 2)
        self._samples.append(mode_z)

        if len(self._samples) >= self.CALIB_FRAMES:
            self.floor_z    = float(np.median(self._samples))
            self.cam_height = -self.floor_z
            print(f"*** floor calibrated: camera {self.cam_height:.3f} m above floor ***")


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

        n_st  = min(self.N_STORE, len(pts_ga))
        idx_s = np.random.choice(len(pts_ga), n_st, replace=False)
        self._prev_world = (pts_ga[idx_s] + t_new).astype(np.float32)

        with self._lock:
            self._t = t_new
        return t_new


# ── Backprojector ─────────────────────────────────────────────────────────────

class DepthBackprojector:
    """Pinhole backproject → camera_link frame → world frame.

    Returns (xyz_world, colors):
      xyz_world: world-frame pts — used for live-cloud display and mapping
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


# ── Voxel map ─────────────────────────────────────────────────────────────────

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
    Translation comes from PointCloudOdometry (updated each frame after process,
    one frame behind — negligible at 15 fps).
    """

    MIN_DEPTH: float = 0.1
    MAX_DEPTH: float = 8.0

    def __init__(self, width: int = 848, height: int = 480, fps: int = 15) -> None:
        import pyrealsense2 as rs

        self._width  = width
        self._height = height

        self._pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16,  fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        profile = self._pipeline.start(cfg)

        self._depth_scale = (
            profile.get_device().first_depth_sensor().get_depth_scale()
        )
        self._align = rs.align(rs.stream.depth)

        intr = (profile.get_stream(rs.stream.depth)
                       .as_video_stream_profile()
                       .get_intrinsics())
        self._intrinsics = np.array(
            [intr.fx, intr.fy, intr.ppx, intr.ppy], dtype=np.float64
        )

        self._spatial_filter   = rs.spatial_filter()
        self._hole_fill_filter = rs.hole_filling_filter()

        self._imu_lock      = threading.Lock()
        self._last_accel    = np.array([0.0, 0.0, -9.81], dtype=np.float32)
        self._motion_sensor = None
        self._has_imu       = False
        self._madgwick      = None
        self._R_imu_to_link = None

        device = profile.get_device()
        for sensor in device.query_sensors():
            if not sensor.is_motion_sensor():
                continue
            all_profiles = sensor.get_stream_profiles()
            gyro_profiles  = [p for p in all_profiles if p.stream_type() == rs.stream.gyro]
            accel_profiles = [p for p in all_profiles if p.stream_type() == rs.stream.accel]
            if not gyro_profiles or not accel_profiles:
                break
            gyro_p  = max(gyro_profiles,  key=lambda p: p.fps())
            accel_p = max(accel_profiles, key=lambda p: p.fps())
            # Extrinsics: accel frame → depth optical frame → camera_link
            depth_stream   = profile.get_stream(rs.stream.depth)
            ext            = accel_p.get_extrinsics_to(depth_stream)
            R_imu_to_depth = np.array(ext.rotation, dtype=np.float32).reshape(3, 3)
            self._R_imu_to_link = _R_OPT_TO_LINK @ R_imu_to_depth
            self._madgwick      = MadgwickFilter(beta=0.033)
            sensor.open([gyro_p, accel_p])
            sensor.start(self._on_motion)
            self._motion_sensor = sensor
            self._has_imu       = True
            print(f"IMU sensor API: gyro@{gyro_p.fps()}Hz  accel@{accel_p.fps()}Hz  "
                  f"(independent of video pipeline)")
            break

        if not self._has_imu:
            print("IMU not available — world-frame tracking disabled, using identity pose")

        self.odom         = PointCloudOdometry()
        self._pose_locked = not self._has_imu

    def _on_motion(self, frame: object) -> None:
        """Sensor callback — gyro at ~200 Hz, accel at ~63 Hz, independent of video."""
        import pyrealsense2 as rs
        st = frame.get_profile().stream_type()
        if st == rs.stream.gyro:
            g         = frame.as_motion_frame().get_motion_data()
            gyro_link = self._R_imu_to_link @ np.array([g.x, g.y, g.z], dtype=np.float32)
            ts_hw     = frame.get_timestamp() / 1000.0   # SDK gives ms → convert to s
            with self._imu_lock:
                self._madgwick.update(gyro_link, self._last_accel, ts_hw)
                if not self._pose_locked:
                    self._pose_locked = True
                    print("*** IMU active — gyro integrating at native rate ***", flush=True)
        elif st == rs.stream.accel:
            a = frame.as_motion_frame().get_motion_data()
            with self._imu_lock:
                self._last_accel = self._R_imu_to_link @ np.array([a.x, a.y, a.z],
                                                                    dtype=np.float32)

    @property
    def pose_locked(self) -> bool:
        return self._pose_locked

    def read(self, ts: float) -> DepthFramePacket:
        frames  = self._pipeline.wait_for_frames()
        aligned = self._align.process(frames)

        with self._imu_lock:
            R = self._madgwick.R.copy() if self._has_imu else np.eye(3, dtype=np.float32)
        t = self.odom.t

        depth_frame = aligned.get_depth_frame()
        depth_frame = self._spatial_filter.process(depth_frame)
        depth_frame = self._hole_fill_filter.process(depth_frame)
        depth = np.asarray(depth_frame.get_data(), dtype=np.float32) * self._depth_scale
        invalid = (depth <= 0) | (depth < self.MIN_DEPTH) | (depth > self.MAX_DEPTH)
        depth[invalid] = np.nan

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
        if self._motion_sensor is not None:
            try:
                self._motion_sensor.stop()
                self._motion_sensor.close()
            except Exception:
                pass
        self._pipeline.stop()


# ── Ghost clearing via ray casting ───────────────────────────────────────────

def _raycast_free_keys(
    surface_pts: np.ndarray,
    vox_size: float,
    n_rays: int = 400,
) -> np.ndarray:
    """Return int64 voxel keys that are free space between camera and surface.

    Operates in the rotation-only frame where the camera is always at [0,0,0].
    Fully vectorised: builds all ray-step grids at once, no Python loop.
    Stops one voxel short of each surface point so the surface itself is
    never cleared.
    """
    origin = np.zeros(3, dtype=np.float32)

    n = min(len(surface_pts), n_rays)
    if n == 0:
        return np.empty(0, dtype=np.int64)

    idx  = (np.random.choice(len(surface_pts), n, replace=False)
            if len(surface_pts) > n else np.arange(n))
    pts   = surface_pts[idx]
    vecs  = pts - origin
    dists = np.linalg.norm(vecs, axis=1)
    ok    = dists > vox_size * 2
    vecs, dists = vecs[ok], dists[ok]
    if not len(vecs):
        return np.empty(0, dtype=np.int64)

    dirs      = vecs / dists[:, None]
    max_steps = int(np.ceil(dists.max() / vox_size))

    t_vals  = np.arange(max_steps, dtype=np.float32) * vox_size
    t_end   = (dists - vox_size * 1.5)[:, None]
    valid_m = (t_vals[None, :] >= 0) & (t_vals[None, :] < t_end)

    ray_pts  = origin + dirs[:, None, :] * t_vals[None, :, None]
    vk_flat  = np.floor(ray_pts.reshape(-1, 3) / vox_size).astype(np.int32)
    keys_all = _pack(vk_flat)
    return np.unique(keys_all[valid_m.ravel()])


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    rr.init("realsense_stereo_nav", spawn=True)
    rr.send_blueprint(rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(name="live cloud",  origin="world",
                              contents=["world/cloud", "world/camera/**"]),
            rrb.Spatial3DView(name="voxel map",   origin="world",
                              contents=["world/map"]),
            rrb.Spatial3DView(name="global map",  origin="world",
                              contents=["world/global_map"]),
        )
    ))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    src         = RealSenseDepthSource(width=848, height=480, fps=15)
    bp          = DepthBackprojector()
    grad        = GradientStabilityFilter()
    floor_calib = _FloorCalibrator()

    acc_pts:       np.ndarray = np.empty((0, 3), dtype=np.float32)
    map_ready:     bool       = False
    world_floor_z: float      = 0.0  # locked once at map start; never updated
    frame          = 0
    t0             = time.monotonic()
    pinhole_logged = False

    print("RealSense open — hold still ~4 s for IMU + floor calibration, then explore freely.")

    try:
        while True:
            ts  = time.monotonic()
            pkt = src.read(ts)

            rr.set_time("frame", sequence=frame)

            if not pinhole_logged:
                rr.log("world/camera", rr.Pinhole(
                    image_from_camera=pkt.K, width=pkt.width, height=pkt.height,
                ), static=True)
                pinhole_logged = True

            rr.log("world/camera/depth", rr.DepthImage(pkt.depth, meter=1.0))

            stable_mask = grad.compute(pkt.depth)
            xyz, colors = bp.project(pkt, stable_mask)
            if not len(xyz):
                frame += 1
                continue

            cam_z = float(pkt.pose_t[2])
            rr.log("world/cloud", rr.Points3D(
                positions=xyz,
                colors=colors if colors is not None else _height_color(xyz[:, 2] - cam_z),
                radii=0.003,
            ))

            xyz_cam = xyz @ pkt.pose_R
            floor_calib.update(xyz_cam)
            if floor_calib.ready:
                keep        = xyz_cam[:, 2] > (floor_calib.floor_z + 0.03)
                xyz_kept    = xyz[keep]
                xyz_cam_icp = xyz_cam[keep]
            else:
                xyz_kept    = xyz
                xyz_cam_icp = xyz_cam

            if len(xyz_kept):
                # ── Per-frame voxel map ───────────────────────────────────────
                vk       = np.floor(xyz_kept / _VOX_SIZE).astype(np.int32)
                _, first = np.unique(_pack(vk), return_index=True)
                xyz_vox  = xyz_kept[first]

                rr.log("world/map", rr.Points3D(
                    positions=xyz_vox,
                    colors=_height_color(xyz_vox[:, 2] - cam_z),
                    radii=0.010,
                ))

                # ── Global map (rotation-only frame, strips ICP t oscillation) ─────
                if floor_calib.ready:
                    if not map_ready:
                        acc_pts       = np.empty((0, 3), dtype=np.float32)
                        map_ready     = True
                        # Lock floor Z once (cam_z ≈ 0, ICP hasn't drifted yet).
                        world_floor_z = cam_z + floor_calib.floor_z
                        print(f"*** global map started — floor at Z ≈ {world_floor_z:.3f} m ***",
                              flush=True)

                    xyz_ronly  = xyz_vox - pkt.pose_t
                    vk_r       = np.floor(xyz_ronly / _VOX_SIZE).astype(np.int32)
                    _, first_r = np.unique(_pack(vk_r), return_index=True)
                    xyz_vox_r  = xyz_ronly[first_r]

                    # Floor filter in rotation-only frame (Z ≈ world Z since t_z ≈ 0).
                    xyz_for_map = xyz_vox_r[xyz_vox_r[:, 2] > world_floor_z + 0.04]

                    # Ray-cast BEFORE inserting; camera at origin in rotation-only frame.
                    if len(acc_pts) and len(xyz_for_map):
                        free_keys = _raycast_free_keys(xyz_for_map, _VOX_SIZE, n_rays=400)
                        if len(free_keys):
                            keys_acc = _pack(np.floor(acc_pts / _VOX_SIZE).astype(np.int32))
                            acc_pts  = acc_pts[~np.isin(keys_acc, free_keys)]

                    if len(xyz_for_map):
                        acc_pts = np.vstack([acc_pts, xyz_for_map]) if len(acc_pts) else xyz_for_map.copy()
                        _, ui   = np.unique(_pack(np.floor(acc_pts / _VOX_SIZE).astype(np.int32)),
                                            return_index=True)
                        acc_pts = acc_pts[ui]

                    rr.log("world/global_map", rr.Points3D(
                        positions=acc_pts,
                        colors=_height_color(acc_pts[:, 2] - cam_z),
                        radii=0.006,
                    ))

            if len(xyz_cam_icp) >= 50:
                src.odom.update(xyz_cam_icp, pkt.pose_R)

            fps   = frame / max(time.monotonic() - t0, 1e-6)
            t_icp = src.odom.t
            theta = np.degrees(np.arccos(np.clip((np.trace(pkt.pose_R) - 1.0) / 2.0, -1.0, 1.0)))
            print(
                f"frame={frame:5d}  rot={theta:.1f}deg  map={len(acc_pts):6d}vox  "
                f"t=[{t_icp[0]:+.3f},{t_icp[1]:+.3f},{t_icp[2]:+.3f}]  "
                f"floor={'ready' if floor_calib.ready else 'calibrating'}  fps={fps:.1f}",
                flush=True,
            )
            frame += 1

    except KeyboardInterrupt:
        pass

    src.stop()
    print(f"done — {len(acc_pts)} voxels in global map")


if __name__ == "__main__":
    main()
