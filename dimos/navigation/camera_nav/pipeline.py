"""Camera-agnostic depth pipeline: gradient filter → backproject → voxel map → Rerun.

Classes:
  DepthSource              Protocol — implement read() + pose_locked for any camera
  DepthFramePacket         one frame: depth H×W + intrinsics + pose (+ optional XYZ/RGB)
  GradientStabilityFilter  rejects depth-discontinuity pixels via Sobel gradient
  DepthBackprojector       stable pixels → world-frame XYZ  (SDK fast path or pinhole)
  FastVoxelMap             append-only numpy voxel map, no Python dict
  DepthStreamer            main thread (live cloud) + async map thread

To support a new camera, implement DepthSource and write a thin entry-point
module (see zed_stereo_nav.py and realsense_stereo_nav.py for examples).

Shared exports used by camera-specific pipelines:
  _pack, _height_color, _R_OPT_TO_LINK  — helper functions and constants
  DepthFramePacket, DepthBackprojector   — camera-agnostic packet and backprojector
"""

from __future__ import annotations

import queue
import threading
from dataclasses import dataclass, field
from typing import Protocol

import numpy as np
import rerun as rr
from scipy.ndimage import sobel


# ── Constants ────────────────────────────────────────────────────────────────

_VOFF  = np.int64(100_000)
_VMASK = np.int64(0x3FFFF)

_Z_REL_LO: float = -1.4
_Z_REL_HI: float =  0.5
_FLOOR_RAY_Z: float = -0.30

# Standard optical-frame → camera_link rotation.
# Pinhole backproject gives optical frame (X=right, Y=down, Z=depth).
# camera_link is (X=fwd, Y=left, Z=up).
_R_OPT_TO_LINK = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)


# ── Helpers ──────────────────────────────────────────────────────────────────

def _height_color(z_rel: np.ndarray) -> np.ndarray:
    t = np.clip((z_rel - _Z_REL_LO) / (_Z_REL_HI - _Z_REL_LO), 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0.0, 1.0)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0.0, 1.0)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0.0, 1.0)
    return (np.column_stack([r, g, b]) * 255).astype(np.uint8)


def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + _VOFF) & _VMASK
    return (v[:, 0] << 36) | (v[:, 1] << 18) | v[:, 2]


def _filter_isolated(xyz: np.ndarray, voxel: float = 0.05, min_pts: int = 2) -> np.ndarray:
    if len(xyz) < min_pts:
        return np.zeros((0, 3), dtype=np.float32)
    vk = np.floor(xyz / voxel).astype(np.int32)
    _, inv, cnt = np.unique(_pack(vk), return_inverse=True, return_counts=True)
    return xyz[cnt[inv] >= min_pts]


# ── Camera source protocol ────────────────────────────────────────────────────

class DepthSource(Protocol):
    """Interface any depth camera source must satisfy.

    Implement read() and pose_locked; the rest of the pipeline is camera-agnostic.
    See ZEDDepthSource and RealSenseDepthSource for reference implementations.
    """

    @property
    def pose_locked(self) -> bool:
        """True once pose estimates are reliable enough to build a world-frame map."""
        ...

    def read(self, ts: float) -> DepthFramePacket:
        """Capture one frame and return a populated DepthFramePacket."""
        ...


# ── Packet ───────────────────────────────────────────────────────────────────

@dataclass
class DepthFramePacket:
    """One complete depth frame from any camera.

    depth:      H×W float32, metres. NaN = invalid pixel.
    xyz_cam_hw: H×W×3 float32, SDK-native XYZ in camera_link frame
                (RIGHT_HANDED_Z_UP_X_FWD). When set, backprojector skips
                manual pinhole math and uses this directly.
    colors_hw:  H×W×3 uint8 RGB. Optional — enables coloured live cloud.
    """
    timestamp:  float
    depth:      np.ndarray                  # H×W float32
    intrinsics: np.ndarray                  # [fx, fy, cx, cy] float64
    width:      int
    height:     int
    pose_R:     np.ndarray                  # 3×3 float32  camera→world
    pose_t:     np.ndarray                  # (3,) float32 camera→world
    confidence: np.ndarray | None = field(default=None)
    xyz_cam_hw: np.ndarray | None = field(default=None)
    colors_hw:  np.ndarray | None = field(default=None)

    @property
    def valid_fraction(self) -> float:
        return float(np.isfinite(self.depth).mean())

    @property
    def K(self) -> np.ndarray:
        fx, fy, cx, cy = self.intrinsics
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


# ── Gradient stability filter ─────────────────────────────────────────────────

@dataclass
class GradientStabilityConfig:
    gradient_threshold: float = 0.30  # m/pixel; above → depth discontinuity → reject


class GradientStabilityFilter:
    """Rejects pixels at depth discontinuities via Sobel gradient magnitude.

    |∇d| = sqrt(sobel_x²  + sobel_y²)  [m/pixel]
    Flat surfaces (walls, floors) → near-zero gradient → pass.
    Stereo edge artifacts → high gradient → rejected.
    Fully deterministic — no probability, weighting, or smoothing.
    """

    def __init__(self, cfg: GradientStabilityConfig | None = None) -> None:
        self.cfg = cfg or GradientStabilityConfig()

    def compute(self, depth: np.ndarray) -> np.ndarray:
        """Return stable mask H×W bool (True = geometrically stable pixel)."""
        valid   = np.isfinite(depth)
        depth_f = np.where(valid, depth, 0.0).astype(np.float64)
        grad_mag = np.hypot(sobel(depth_f, axis=1), sobel(depth_f, axis=0))
        return valid & (grad_mag < self.cfg.gradient_threshold)


# ── Backprojector ─────────────────────────────────────────────────────────────

class DepthBackprojector:
    """DepthFramePacket → (xyz N×3 world-frame float32, colors N×3 uint8 | None).

    Fast path: if pkt.xyz_cam_hw is set (e.g. from ZEDDepthSource which uses
    sl.MEASURE.XYZ), skip manual math — just apply the VIO transform.

    Fallback: pinhole backproject + _R_OPT_TO_LINK rotation (any camera).
    """

    def project(
        self,
        pkt:    DepthFramePacket,
        stable: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray | None]:
        valid = np.isfinite(pkt.depth) if stable is None else (np.isfinite(pkt.depth) & stable)

        if pkt.xyz_cam_hw is not None:
            xyz_cam = pkt.xyz_cam_hw[valid]
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
        colors    = pkt.colors_hw[valid] if pkt.colors_hw is not None else None
        return xyz_world, colors


# ── Voxel map ─────────────────────────────────────────────────────────────────

class FastVoxelMap:
    """Append-only world-frame point map backed entirely by numpy arrays.

    No Python dicts — all deduplication uses np.unique (single vectorised C call).
    Each add() deduplicates the incoming batch then appends.
    Every COMPACT_EVERY adds a full global dedup runs to bound memory.
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


# ── Streamer ──────────────────────────────────────────────────────────────────

class DepthStreamer:
    """Two-layer pipeline, camera-agnostic.

    Main thread (fast):  gradient filter → backproject → live Rerun cloud
    Map thread  (async): height filter → isolation filter → FastVoxelMap → Rerun map

    Pass any DepthSource implementation — ZED, RealSense, or future cameras.
    """

    MAX_CLOUD = 50_000
    MAX_MAP   = 200_000
    MAP_EVERY = 5

    def __init__(
        self,
        source:      DepthSource,
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
        rr.log("world/camera/stable", rr.Image(stable_mask.astype(np.uint8) * 255))

        xyz, colors = self._bp.project(pkt, stable_mask)
        if len(xyz) == 0:
            return

        self._cam_z = float(pkt.pose_t[2])
        n   = min(len(xyz), self.MAX_CLOUD)
        idx = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(n)
        cloud_colors = colors[idx] if colors is not None else _height_color(xyz[idx, 2] - self._cam_z)
        rr.log("world/cloud", rr.Points3D(positions=xyz[idx], colors=cloud_colors, radii=0.003))

        if self._src.pose_locked:
            try:
                self._map_queue.put_nowait((xyz, pkt.pose_t.copy(), self._cam_z, frame))
            except queue.Full:
                pass

    def _map_worker(self) -> None:
        while True:
            item = self._map_queue.get()
            if item is None:
                return
            xyz_world, cam_pos, cam_z, frame = item

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
        pct = 100 * n_stable / n_valid if n_valid > 0 else 0.0
        print(
            f"frame={frame:5d}  "
            f"stable={n_stable:5d}/{n_valid:5d} ({pct:.0f}%)  "
            f"map={n_map:6d}  "
            f"cam_z={self._cam_z:+.2f}m  vio={lock}  fps={fps:.1f}",
            flush=True,
        )


# ── Rerun setup (shared) ──────────────────────────────────────────────────────

def init_rerun(app_name: str) -> None:
    """Initialise Rerun with the standard depth-pipeline blueprint."""
    import rerun.blueprint as rrb
    rr.init(app_name, spawn=True)
    rr.send_blueprint(rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(name="live cloud + map", origin="world",
                              contents=["world/cloud", "world/map", "world/camera/**"]),
            rrb.Spatial3DView(name="map only", origin="world",
                              contents=["world/map"]),
        )
    ))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/cloud", rr.Points3D([[0, 0, 0]]), static=True)
