"""ZED Mini → persistent world-frame 3D occupancy map.

Camera-agnostic pipeline:

  ZEDDepthSource     all ZED SDK calls; returns raw float32 depth (metres)
  DepthFramePacket   one frame: depth H×W + intrinsics + VIO pose
  GradientStabilityFilter  rejects depth-discontinuity pixels (|∇d| ≥ threshold)
  DepthBackprojector backprojects stable pixels → world-frame XYZ
  VoxelAccumulator   persistent map: deduplicates and exports voxels seen ≥ 2 frames
  DepthStreamer       drives the pipeline per frame, logs to Rerun

Swap ZEDDepthSource for any other DepthSource implementation to support a
different camera — no other changes required.

Rerun entities:
  world/camera        Pinhole model (static)
  world/camera/depth  raw depth image (float32, metres)
  world/camera/stable gradient stability mask (white = stable pixel)
  world/cloud         current-frame scan, height-colored
  world/map           persistent accumulated map, height-colored

Usage:
  python -m dimos.navigation.camera_nav.zed_depth_costmap
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import queue
import threading

import numpy as np
import pyzed.sl as sl
import rerun as rr
import rerun.blueprint as rrb
from scipy.ndimage import sobel


# ── Voxel key packing ────────────────────────────────────────────────────────
# 18-bit per axis → supports ±100 000 voxels/axis (±2 km at 2 cm resolution).

_VOFF  = np.int64(100_000)
_VMASK = np.int64(0x3FFFF)

# Camera-relative height band for obstacle accumulation.
# VIO absolute translation is unreliable (set_floor_as_origin calibration drifts),
# so all height filtering is camera-relative: h_rel = world_Z - cam_Z.
_Z_REL_LO: float = -1.4   # 1.4 m below camera
_Z_REL_HI: float =  0.5   # 0.5 m above camera

# Floor exclusion via ray angle.
# The floor is always below the camera, so rays to floor points point steeply
# downward. Ray directions are reliable even when VIO translation drifts because
# both the camera position and the surface point are in the same VIO frame —
# the relative vector between them is unaffected by translation error.
# _FLOOR_RAY_Z is the Z component of the normalised ray direction below which
# we treat the point as a floor hit. -0.30 ≈ 17° below horizontal; tune up
# (toward 0) to be more conservative, down (toward -1) to be more permissive.
_FLOOR_RAY_Z: float = -0.30


def _height_color(z_rel: np.ndarray) -> np.ndarray:
    """Jet colormap over camera-relative height band [_Z_REL_LO, _Z_REL_HI].

    blue  = floor level   (z_rel ≈ _Z_REL_LO, ~1.4 m below camera)
    green = mid-height    (z_rel ≈ halfway)
    red   = camera level  (z_rel ≈ 0) and above

    z_rel = world_Z − camera_world_Z so it is always valid even before
    set_floor_as_origin has calibrated the absolute floor height.
    """
    t = np.clip((z_rel - _Z_REL_LO) / (_Z_REL_HI - _Z_REL_LO), 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0.0, 1.0)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0.0, 1.0)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0.0, 1.0)
    return (np.column_stack([r, g, b]) * 255).astype(np.uint8)


def _pack(vkeys: np.ndarray) -> np.ndarray:
    """(N, 3) int32 voxel indices → unique int64 keys."""
    v = (vkeys.astype(np.int64) + _VOFF) & _VMASK
    return (v[:, 0] << 36) | (v[:, 1] << 18) | v[:, 2]


def _filter_isolated(xyz: np.ndarray, voxel: float = 0.05, min_pts: int = 2) -> np.ndarray:
    """Remove points that are the only hit in their voxel — stereo flying-pixel filter.

    A real surface produces many pixels per voxel; a stereo edge artifact produces one.
    voxel=5 cm gives enough area to catch the artifact without merging adjacent surfaces.
    """
    if len(xyz) < min_pts:
        return np.zeros((0, 3), dtype=np.float32)
    vk  = np.floor(xyz / voxel).astype(np.int32)
    _, inv, cnt = np.unique(_pack(vk), return_inverse=True, return_counts=True)
    return xyz[cnt[inv] >= min_pts]


# ── Stage 2: Geometric stability filter ──────────────────────────────────────

@dataclass
class GradientStabilityConfig:
    gradient_threshold: float = 0.10  # m/pixel; above this → depth discontinuity → reject


class GradientStabilityFilter:
    """Rejects depth pixels at geometric discontinuities via Sobel gradient magnitude.

    Stereo depth errors concentrate at depth discontinuities — object edges,
    foreground/background transitions — where the two cameras see different
    surfaces in adjacent pixels and the matcher interpolates across them.

    The Sobel gradient magnitude over the depth image directly measures this:

        |∇d| = sqrt( sobel_x(d)² + sobel_y(d)² )   [metres / pixel]

    Flat surfaces (walls, floors, tables) have |∇d| ≈ 0 and pass cleanly.
    Depth edges have |∇d| >> 0 and are rejected.

    This is deterministic and requires no probability, weighting, or smoothing.
    """

    def __init__(self, cfg: GradientStabilityConfig | None = None) -> None:
        self.cfg = cfg or GradientStabilityConfig()

    def compute(self, depth: np.ndarray) -> np.ndarray:
        """Return stable mask H×W (bool). True = geometrically stable pixel."""
        valid   = np.isfinite(depth)
        depth_f = np.where(valid, depth, 0.0).astype(np.float64)

        gx = sobel(depth_f, axis=1)
        gy = sobel(depth_f, axis=0)
        grad_mag = np.hypot(gx, gy)

        return valid & (grad_mag < self.cfg.gradient_threshold)


# ── Optical-frame → camera_link rotation ─────────────────────────────────────
# ZED get_position() returns camera_link pose in world (X=fwd, Y=left, Z=up).
# Pinhole backproject gives camera-OPTICAL frame (X=right, Y=down, Z=depth).
# Rotate optical→link before applying VIO pose so Z=up, not Z=depth.
_R_OPT_TO_LINK = np.array(
    [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32
)


# ── Packet contract ────────────────────────────────────────────────────────────

@dataclass
class DepthFramePacket:
    """One complete depth frame.

    depth:      H×W float32, metres (forward distance). NaN = invalid.
    xyz_cam_hw: H×W×3 float32, SDK XYZ in camera_link frame (RIGHT_HANDED_Z_UP_X_FWD).
                When present, DepthBackprojector uses it directly — no manual math.
    colors_hw:  H×W×3 uint8 RGB. Optional; enables colored live cloud in Rerun.
    """
    timestamp:  float
    depth:      np.ndarray                    # H×W float32, NaN=invalid
    intrinsics: np.ndarray                    # [fx, fy, cx, cy] float64
    width:      int
    height:     int
    pose_R:     np.ndarray                    # 3×3 float32  camera→world
    pose_t:     np.ndarray                    # (3,) float32 camera→world
    confidence: np.ndarray | None = field(default=None)   # H×W float32 0-100
    xyz_cam_hw: np.ndarray | None = field(default=None)   # H×W×3 float32
    colors_hw:  np.ndarray | None = field(default=None)   # H×W×3 uint8 RGB

    @property
    def valid_fraction(self) -> float:
        return float(np.isfinite(self.depth).mean())

    @property
    def K(self) -> np.ndarray:
        fx, fy, cx, cy = self.intrinsics
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


# ── Camera source protocol ────────────────────────────────────────────────────
#
# Any depth camera must implement `read(ts) -> DepthFramePacket` and expose
# `pose_locked` so the pipeline stages below are camera-agnostic.
# Swap ZEDDepthSource for a RealSenseDepthSource and nothing else changes.

# ── ZED Mini source ───────────────────────────────────────────────────────────

class ZEDDepthSource:
    """ZED SDK → DepthFramePacket.  No confidence filtering — raw depth only.

    Only physically impossible values (inf, below MIN_DEPTH, above MAX_DEPTH)
    are set to NaN.  All quality filtering happens downstream in the pipeline
    (GradientStabilityFilter, _filter_isolated, VoxelAccumulator).
    """

    MIN_DEPTH: float = 0.3
    MAX_DEPTH: float = 8.0

    def __init__(self, zed: sl.Camera) -> None:
        self._zed       = zed
        self._xyz_mat   = sl.Mat()
        self._img_mat   = sl.Mat()
        self._intr      = _IntrinsicsReader()
        self._pose      = _PoseReader()

    def enable_tracking(self) -> bool:
        return self._pose.enable(self._zed)

    @property
    def pose_locked(self) -> bool:
        return self._pose.locked

    def read(self, ts: float) -> DepthFramePacket:
        self._intr.read(self._zed)
        self._pose.update(self._zed)

        # XYZ: H×W×4 float32 in camera_link frame (RIGHT_HANDED_Z_UP_X_FWD)
        # Channel layout: [X=fwd, Y=left, Z=up, pad]
        self._zed.retrieve_measure(self._xyz_mat, sl.MEASURE.XYZ, sl.MEM.CPU)
        xyz_raw = self._xyz_mat.get_data()[:, :, :3].astype(np.float32)

        # Depth = forward distance = X channel; NaN where ZED has no valid match
        depth = xyz_raw[:, :, 0].copy()
        invalid = ~np.isfinite(depth) | (depth <= self.MIN_DEPTH) | (depth > self.MAX_DEPTH)
        depth[invalid] = np.nan
        xyz_raw[invalid] = np.nan

        # Left image for colored live cloud (BGRA → RGB)
        self._zed.retrieve_image(self._img_mat, sl.VIEW.LEFT, sl.MEM.CPU)
        img = self._img_mat.get_data()
        colors_hw = img[:, :, 2::-1].copy()  # BGRA → RGB uint8

        return DepthFramePacket(
            timestamp  = ts,
            depth      = depth,
            intrinsics = self._intr.intrinsics,
            width      = self._intr.width,
            height     = self._intr.height,
            pose_R     = self._pose.R.copy(),
            pose_t     = self._pose.t.copy(),
            xyz_cam_hw = xyz_raw,
            colors_hw  = colors_hw,
        )


# ── Internal ZED helpers (used only by ZEDDepthSource) ───────────────────────

class _IntrinsicsReader:
    """Reads and caches ZED left-camera calibration (fixed for the session)."""

    def __init__(self) -> None:
        self._fx = self._fy = self._cx = self._cy = 0.0
        self._w = self._h = 0
        self._ready = False

    def read(self, zed: sl.Camera) -> None:
        if self._ready:
            return
        ci  = zed.get_camera_information()
        cal = ci.camera_configuration.calibration_parameters.left_cam
        res = ci.camera_configuration.resolution
        self._fx, self._fy = cal.fx, cal.fy
        self._cx, self._cy = cal.cx, cal.cy
        self._w, self._h   = res.width, res.height
        self._ready = True

    @property
    def intrinsics(self) -> np.ndarray:
        return np.array([self._fx, self._fy, self._cx, self._cy], dtype=np.float64)

    @property
    def width(self) -> int: return self._w

    @property
    def height(self) -> int: return self._h

    @property
    def K(self) -> np.ndarray:
        return np.array(
            [[self._fx, 0, self._cx], [0, self._fy, self._cy], [0, 0, 1]],
            dtype=np.float64,
        )


class _PoseReader:
    """ZED VIO camera→world pose.  Identity passthrough until VIO locks."""

    def __init__(self) -> None:
        self._R         = np.eye(3, dtype=np.float32)
        self._t         = np.zeros(3, dtype=np.float32)
        self._active    = False
        self._locked    = False
        self._pose      = sl.Pose()
        self._t0        = time.monotonic()
        self._last_warn = 0.0

    def enable(self, zed: sl.Camera) -> bool:
        tp = sl.PositionalTrackingParameters()
        tp.enable_imu_fusion   = True
        tp.set_floor_as_origin = True
        ok = zed.enable_positional_tracking(tp) == sl.ERROR_CODE.SUCCESS
        self._active = ok
        print("VIO tracking enabled" if ok else "VIO tracking failed — running in camera frame")
        return ok

    def update(self, zed: sl.Camera) -> None:
        if not self._active:
            return
        now = time.monotonic()
        if zed.get_position(self._pose, sl.REFERENCE_FRAME.WORLD) == sl.POSITIONAL_TRACKING_STATE.OK:
            t = self._pose.get_translation()
            q = self._pose.get_orientation()
            self._t = np.array(t.get(), dtype=np.float32)
            x, y, z, w = np.array(q.get(), dtype=np.float32)
            self._R = np.array([
                [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)  ],
                [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)  ],
                [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
            ], dtype=np.float32)
            if not self._locked:
                print(f"*** VIO LOCKED after {now - self._t0:.1f}s ***")
            self._locked = True
        else:
            if now - max(self._last_warn, self._t0) >= 15.0:
                print(f"    VIO searching … {now - self._t0:.0f}s elapsed")
                self._last_warn = now

    @property
    def R(self) -> np.ndarray: return self._R

    @property
    def t(self) -> np.ndarray: return self._t

    @property
    def locked(self) -> bool: return self._locked


# ── Stage 4: Backprojector ────────────────────────────────────────────────────

class DepthBackprojector:
    """DepthFramePacket → (xyz N×3 world-frame, colors N×3 uint8).

    Fast path (ZEDDepthSource): pkt.xyz_cam_hw is already SDK-computed in
    camera_link frame (RIGHT_HANDED_Z_UP_X_FWD). Just apply the VIO transform.

    Fallback (any other source): manual pinhole backproject + _R_OPT_TO_LINK.
    """

    def project(
        self,
        pkt:    DepthFramePacket,
        stable: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (xyz (N,3) float32 world-frame, colors (N,3) uint8 or None)."""
        valid = np.isfinite(pkt.depth) if stable is None else (np.isfinite(pkt.depth) & stable)

        if pkt.xyz_cam_hw is not None:
            xyz_cam = pkt.xyz_cam_hw[valid]          # (N, 3), camera_link frame
        else:
            d = pkt.depth
            H, W = d.shape
            fx, fy, cx, cy = pkt.intrinsics
            uu, vv = np.meshgrid(np.arange(W, dtype=np.float32),
                                  np.arange(H, dtype=np.float32))
            dd      = d[valid]
            xyz_opt = np.column_stack([(uu[valid] - cx) * dd / fx,
                                       (vv[valid] - cy) * dd / fy,
                                       dd]).astype(np.float32)
            xyz_cam = xyz_opt @ _R_OPT_TO_LINK.T

        xyz_world = (xyz_cam @ pkt.pose_R.T + pkt.pose_t).astype(np.float32)

        colors = pkt.colors_hw[valid] if pkt.colors_hw is not None else None
        return xyz_world, colors


# ── Stage 5: Voxel map ────────────────────────────────────────────────────────

class FastVoxelMap:
    """Append-only world-frame point map backed entirely by numpy arrays.

    No Python dicts, no per-key loops.  All deduplication uses np.unique — a
    single vectorised C call — so throughput matches the live cloud pipeline.

    add() deduplicates the incoming batch and appends.
    Every COMPACT_EVERY adds, a full global dedup runs to bound memory.
    """

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
        # Dedup the incoming batch first
        vk = np.floor(xyz / self._v).astype(np.int32)
        _, first = np.unique(_pack(vk), return_index=True)
        new_pts = xyz[first]

        if self._n + len(new_pts) > self._cap:
            self._compact()
        if self._n + len(new_pts) > self._cap:
            self._grow()

        sl = slice(self._n, self._n + len(new_pts))
        self._pts[sl] = new_pts
        self._n += len(new_pts)

        self._n_adds += 1
        if self._n_adds % self.COMPACT_EVERY == 0:
            self._compact()

    def _compact(self) -> None:
        if self._n == 0:
            return
        pts = self._pts[:self._n]
        # Global voxel dedup
        vk = np.floor(pts / self._v).astype(np.int32)
        _, idx = np.unique(_pack(vk), return_index=True)
        pts = pts[idx]
        # Drop globally isolated voxels — noise with no neighbor within 10 cm
        pts = _filter_isolated(pts, voxel=0.10, min_pts=2)
        n_u = len(pts)
        self._pts[:n_u] = pts
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


# ── Stage 6: Streamer ─────────────────────────────────────────────────────────

class DepthStreamer:
    """Two-layer pipeline:
    - Main thread (fast):  gradient filter → backproject → live Rerun cloud
    - Map thread  (async): height filter → isolation filter → FastVoxelMap → map Rerun

    Map worker cost is now O(N log N) numpy only — same order as the live cloud.
    """

    MAX_CLOUD = 50_000   # Rerun per-frame point cap
    MAX_MAP   = 200_000  # Rerun map point cap
    MAP_EVERY = 5        # map-thread frames between map Rerun updates

    def __init__(
        self,
        source:      ZEDDepthSource,
        backproj:    DepthBackprojector,
        grad_filter: GradientStabilityFilter | None = None,
    ) -> None:
        self._src            = source
        self._bp             = backproj
        self._grad           = grad_filter or GradientStabilityFilter()
        self._vox            = FastVoxelMap(voxel_size=0.05)
        self._vox_lock       = threading.Lock()
        self._cam_z          = 0.0
        self._pinhole_logged = False
        self._last_n_valid   = 0
        self._last_n_stable  = 0
        self._map_queue: queue.Queue = queue.Queue(maxsize=16)
        self._map_thread = threading.Thread(target=self._map_worker, daemon=True)
        self._map_thread.start()

    def assemble(self, ts: float) -> DepthFramePacket:
        return self._src.read(ts)

    def process(self, pkt: DepthFramePacket, frame: int) -> None:
        """Fast path — target <10 ms. No voxel ops, no world-frame dict work."""
        rr.set_time("frame", sequence=frame)
        if not self._pinhole_logged:
            rr.log("world/camera", rr.Pinhole(
                image_from_camera=pkt.K, width=pkt.width, height=pkt.height,
            ), static=True)
            self._pinhole_logged = True

        rr.log("world/camera/depth", rr.DepthImage(pkt.depth, meter=1.0))

        # Stage 1+2: hard filter + gradient stability
        stable_mask = self._grad.compute(pkt.depth)
        self._last_n_valid  = int(np.isfinite(pkt.depth).sum())
        self._last_n_stable = int(stable_mask.sum())
        rr.log("world/camera/stable", rr.Image(stable_mask.astype(np.uint8) * 255))

        # Stage 3: backproject — SDK XYZ fast path, just VIO transform
        xyz, colors = self._bp.project(pkt, stable_mask)
        if len(xyz) == 0:
            return

        # Live cloud — instant, no dict work
        self._cam_z = float(pkt.pose_t[2])
        n   = min(len(xyz), self.MAX_CLOUD)
        idx = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(n)
        cloud_colors = colors[idx] if colors is not None else _height_color(xyz[idx, 2] - self._cam_z)
        rr.log("world/cloud", rr.Points3D(
            positions=xyz[idx],
            colors=cloud_colors,
            radii=0.003,
        ))

        # Hand off to map thread — non-blocking; drop frame if worker is behind
        if self._src.pose_locked:
            try:
                self._map_queue.put_nowait(
                    (xyz, pkt.pose_t.copy(), self._cam_z, frame)
                )
            except queue.Full:
                pass

    def _map_worker(self) -> None:
        """Background thread: height filter → isolation → FastVoxelMap → Rerun.

        All operations are vectorised numpy — cost is O(N log N), same order as
        the live cloud pipeline.  No Python loops, no ray carving.
        """
        while True:
            item = self._map_queue.get()
            if item is None:
                return
            xyz_world, cam_pos, cam_z, frame = item

            # Height band + floor-angle filter (all numpy)
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
                self._log_map(cam_z)

    def _log_map(self, cam_z: float | None = None) -> None:
        if cam_z is None:
            cam_z = self._cam_z
        with self._vox_lock:
            pts = self._vox.points().copy()
        if len(pts) == 0:
            return
        n   = min(len(pts), self.MAX_MAP)
        idx = np.random.choice(len(pts), n, replace=False) if len(pts) > n else np.arange(n)
        rr.log("world/map", rr.Points3D(
            positions=pts[idx],
            colors=_height_color(pts[idx, 2] - cam_z),
            radii=0.005,
        ))

    def log_stdout(
        self, pkt: DepthFramePacket, frame: int, fps: float,
        n_valid: int = 0, n_stable: int = 0,
    ) -> None:
        lock = "LOCKED" if self._src.pose_locked else "searching"
        with self._vox_lock:
            n_map = self._vox.count
        pct   = 100 * n_stable / n_valid if n_valid > 0 else 0.0
        print(
            f"frame={frame:5d}  "
            f"stable={n_stable:5d}/{n_valid:5d} ({pct:.0f}%)  "
            f"map={n_map:6d}  "
            f"cam_z={self._cam_z:+.2f}m  vio={lock}  fps={fps:.1f}",
            flush=True,
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    # Fork Rerun BEFORE zed.open() — ZED capture threads make post-open fork unsafe.
    rr.init("zed_depth_costmap", spawn=True)
    rr.send_blueprint(rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(name="live cloud", origin="world",
                              contents=["world/cloud", "world/camera/**"]),
            rrb.Spatial3DView(name="map", origin="world",
                              contents=["world/map"]),
        )
    ))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/cloud", rr.Points3D([[0, 0, 0]]), static=True)  # anchor 3D view before camera opens

    zed = sl.Camera()
    ip  = sl.InitParameters()
    ip.camera_resolution      = sl.RESOLUTION.HD720
    ip.camera_fps             = 15
    ip.depth_mode             = sl.DEPTH_MODE.NEURAL
    ip.coordinate_system      = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    ip.coordinate_units       = sl.UNIT.METER
    ip.depth_maximum_distance = 8.0

    if zed.open(ip) != sl.ERROR_CODE.SUCCESS:
        print("ZED failed to open"); return

    # Cap exposure to prevent bright light (windows, lamps) from washing out stereo
    # texture. Auto-exposure chases the bright source and under-exposes the scene;
    # a fixed value keeps geometry well-textured for stereo matching.
    # Range 0-100; lower = faster shutter. Tune up if the scene is too dark.
    zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 50)

    print("camera open — Ctrl-C to quit.")

    src = ZEDDepthSource(zed)
    src.enable_tracking()

    streamer = DepthStreamer(
        src,
        DepthBackprojector(),
        GradientStabilityFilter(GradientStabilityConfig(gradient_threshold=0.30)),
    )

    rt = sl.RuntimeParameters()
    rt.texture_confidence_threshold = 0     # no SDK-level texture gate; pipeline handles quality
    rt.remove_saturated_areas       = True  # pixels overexposed by bright lights have no valid signal

    frame = 0
    t0    = time.monotonic()

    try:
        while True:
            if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
                continue
            ts  = time.monotonic()
            pkt = streamer.assemble(ts)
            streamer.process(pkt, frame)
            streamer.log_stdout(pkt, frame, frame / max(ts - t0, 1e-6),
                                n_valid=streamer._last_n_valid,
                                n_stable=streamer._last_n_stable)
            frame += 1
    except KeyboardInterrupt:
        pass

    zed.close()
    print("done")


if __name__ == "__main__":
    main()
