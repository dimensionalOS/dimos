"""RealSense D435i → probabilistic occupancy map.

Live cloud:  gradient filter → backproject → Rerun (fast, same quality as ZED)
Map:         log-odds occupancy with raycasting — rays clear free space,
             depth measurements mark occupied. Voxels only appear in the map
             once occupied evidence exceeds free evidence.

Difference from ZED: no built-in VIO. The map is built in the camera's
starting frame (identity pose). Revisiting the same area updates the same
voxels correctly so long as the camera returns near its starting pose.
For a true world-frame map with arbitrary motion, add a T265 tracking
camera — swap the identity pose below for T265 pose data.

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

if TYPE_CHECKING:
    import pyrealsense2 as rs


# ── Constants ────────────────────────────────────────────────────────────────

_VOFF  = np.int64(100_000)
_VMASK = np.int64(0x3FFFF)

_Z_REL_LO:   float = -1.4
_Z_REL_HI:   float =  0.5
_FLOOR_RAY_Z: float = -0.30

# Optical frame (X=right, Y=down, Z=depth) → camera_link (X=fwd, Y=left, Z=up)
_R_OPT_TO_LINK = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)


# ── Helpers ──────────────────────────────────────────────────────────────────

def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + _VOFF) & _VMASK
    return (v[:, 0] << np.int64(36)) | (v[:, 1] << np.int64(18)) | v[:, 2]


def _unpack(keys: np.ndarray) -> np.ndarray:
    """int64 keys → (N, 3) int64 voxel indices."""
    vx = ((keys >> np.int64(36)) & _VMASK) - _VOFF
    vy = ((keys >> np.int64(18)) & _VMASK) - _VOFF
    vz = (keys & _VMASK) - _VOFF
    return np.column_stack([vx, vy, vz])


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
    depth:      np.ndarray       # H×W float32, NaN=invalid
    intrinsics: np.ndarray       # [fx, fy, cx, cy] float64
    width:      int
    height:     int
    pose_R:     np.ndarray       # 3×3 float32 camera→world
    pose_t:     np.ndarray       # (3,) float32 camera→world
    colors_hw:  np.ndarray | None = field(default=None)  # H×W×3 uint8 RGB

    @property
    def valid_fraction(self) -> float:
        return float(np.isfinite(self.depth).mean())

    @property
    def K(self) -> np.ndarray:
        fx, fy, cx, cy = self.intrinsics
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


# ── Backprojector ─────────────────────────────────────────────────────────────

class DepthBackprojector:
    """Pinhole backproject → rotate optical→camera_link → apply pose."""

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
        xyz_cam   = xyz_opt @ _R_OPT_TO_LINK.T
        xyz_world = (xyz_cam @ pkt.pose_R.T + pkt.pose_t).astype(np.float32)
        colors    = pkt.colors_hw[valid] if pkt.colors_hw is not None else None
        return xyz_world, colors


# ── Probabilistic occupancy voxel map ────────────────────────────────────────

class OccupancyVoxelMap:
    """Log-odds occupancy map updated by ray casting.

    For each stable depth measurement:
      - RAY_SAMPLES points sampled between camera and surface → free evidence
      - Terminal voxel (the measurement itself) → occupied evidence

    A voxel only appears in the output once accumulated occupied evidence
    exceeds free evidence (log_odds > RENDER_THRESH). This creates natural
    separation between free corridors and occupied surfaces.
    """

    LOG_FREE      = -0.4    # per-sample decrement (free ray hit)
    LOG_OCC       =  0.85   # per-measurement increment (occupied hit)
    LOG_MIN       = -2.0    # clamp — prevents over-confidence in free
    LOG_MAX       =  5.0    # clamp — prevents over-confidence in occupied
    RENDER_THRESH =  0.5    # log-odds above this → render as occupied
    RAY_SAMPLES   =  6      # free-space samples per ray (between cam and surface)

    def __init__(self, voxel_size: float = 0.05) -> None:
        self._v       = voxel_size
        self._data:   dict[int, float] = {}
        self._occupied: set[int]       = set()   # keys where log_odds > RENDER_THRESH

    def update(self, xyz_world: np.ndarray, cam_pos: np.ndarray) -> None:
        if len(xyz_world) == 0:
            return

        # ── Free-space samples along each ray ─────────────────────────
        # t in (0.05, 0.90): avoid the camera voxel and the surface voxel
        t    = np.linspace(0.05, 0.90, self.RAY_SAMPLES, dtype=np.float32)
        dirs = (xyz_world - cam_pos).astype(np.float32)            # (N, 3)
        free_pts = cam_pos + dirs[:, None, :] * t[None, :, None]  # (N, K, 3)
        free_pts = free_pts.reshape(-1, 3)

        free_vk = np.floor(free_pts / self._v).astype(np.int32)
        uf, fc  = np.unique(_pack(free_vk), return_counts=True)

        # ── Occupied at the measurement ────────────────────────────────
        occ_vk = np.floor(xyz_world / self._v).astype(np.int32)
        uo, oc = np.unique(_pack(occ_vk), return_counts=True)

        d   = self._data
        occ = self._occupied

        for k, c in zip(uf.tolist(), fc.tolist()):
            old = d.get(k, 0.0)
            new = max(self.LOG_MIN, old + c * self.LOG_FREE)
            if old > self.RENDER_THRESH and new <= self.RENDER_THRESH:
                occ.discard(k)
            d[k] = new

        for k, c in zip(uo.tolist(), oc.tolist()):
            old = d.get(k, 0.0)
            new = min(self.LOG_MAX, old + c * self.LOG_OCC)
            if new > self.RENDER_THRESH and old <= self.RENDER_THRESH:
                occ.add(k)
            d[k] = new

    def occupied_points(self) -> np.ndarray:
        if not self._occupied:
            return np.zeros((0, 3), dtype=np.float32)
        keys   = np.array(list(self._occupied), dtype=np.int64)
        voxels = _unpack(keys).astype(np.float32)
        return (voxels + 0.5) * self._v   # voxel centre coordinates

    @property
    def n_occupied(self) -> int:
        return len(self._occupied)

    @property
    def n_total(self) -> int:
        return len(self._data)


# ── RealSense depth source ───────────────────────────────────────────────────

class RealSenseDepthSource:
    """pyrealsense2 → DepthFramePacket.

    Depth is aligned to the color frame (same intrinsics, no pixel offset).
    No VIO: pose is always identity. Map accumulates in the camera's
    starting frame.
    """

    MIN_DEPTH: float = 0.3
    MAX_DEPTH: float = 8.0

    def __init__(self, width: int = 848, height: int = 480, fps: int = 15) -> None:
        import pyrealsense2 as rs

        self._width  = width
        self._height = height

        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16,  fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        self._pipeline = rs.pipeline()
        profile        = self._pipeline.start(cfg)

        self._depth_scale = (
            profile.get_device().first_depth_sensor().get_depth_scale()
        )
        self._align = rs.align(rs.stream.color)

        intr = (profile.get_stream(rs.stream.color)
                       .as_video_stream_profile()
                       .get_intrinsics())
        self._intrinsics = np.array(
            [intr.fx, intr.fy, intr.ppx, intr.ppy], dtype=np.float64
        )

        self._pose_R = np.eye(3, dtype=np.float32)
        self._pose_t = np.zeros(3, dtype=np.float32)

    @property
    def pose_locked(self) -> bool:
        return True   # identity pose is always "locked"

    def read(self, ts: float) -> DepthFramePacket:
        frames  = self._pipeline.wait_for_frames()
        aligned = self._align.process(frames)

        depth_raw = np.asarray(
            aligned.get_depth_frame().get_data(), dtype=np.float32
        ) * self._depth_scale
        invalid = (depth_raw <= 0) | (depth_raw < self.MIN_DEPTH) | (depth_raw > self.MAX_DEPTH)
        depth_raw[invalid] = np.nan

        bgr       = np.asarray(aligned.get_color_frame().get_data(), dtype=np.uint8)
        colors_hw = bgr[:, :, ::-1].copy()

        return DepthFramePacket(
            timestamp  = ts,
            depth      = depth_raw,
            intrinsics = self._intrinsics,
            width      = self._width,
            height     = self._height,
            pose_R     = self._pose_R,
            pose_t     = self._pose_t,
            colors_hw  = colors_hw,
        )

    def stop(self) -> None:
        self._pipeline.stop()


# ── Streamer ──────────────────────────────────────────────────────────────────

class DepthStreamer:
    """Two-layer pipeline.

    Main thread  (fast): gradient filter → backproject → live Rerun cloud
    Map thread (async):  height filter → isolation filter → OccupancyVoxelMap
                         with ray casting → periodic Rerun map update
    """

    MAX_CLOUD    = 50_000
    MAX_MAP      = 200_000
    MAP_EVERY    = 5    # frames between occupancy map updates
    MAP_LOG_EVERY = 30  # frames between Rerun map renders

    def __init__(
        self,
        source:      RealSenseDepthSource,
        backproj:    DepthBackprojector,
        grad_filter: GradientStabilityFilter | None = None,
    ) -> None:
        self._src            = source
        self._bp             = backproj
        self._grad           = grad_filter or GradientStabilityFilter()
        self._occ            = OccupancyVoxelMap(voxel_size=0.05)
        self._occ_lock       = threading.Lock()
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

        xyz, colors = self._bp.project(pkt, stable_mask)
        if len(xyz) == 0:
            return

        self._cam_z = float(pkt.pose_t[2])
        n   = min(len(xyz), self.MAX_CLOUD)
        idx = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(n)
        cloud_colors = (colors[idx] if colors is not None
                        else _height_color(xyz[idx, 2] - self._cam_z))
        rr.log("world/cloud", rr.Points3D(positions=xyz[idx], colors=cloud_colors, radii=0.003))

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

            # Height band + floor-angle filter
            h_rel    = xyz_world[:, 2] - cam_z
            rays     = xyz_world - cam_pos
            dist     = np.linalg.norm(rays, axis=1)
            d_z_norm = np.where(dist > 0, rays[:, 2] / dist, 0.0)
            keep     = (
                (h_rel >= _Z_REL_LO) & (h_rel <= _Z_REL_HI)
                & (d_z_norm > _FLOOR_RAY_Z)
            )
            xyz_filt = _filter_isolated(xyz_world[keep])
            if len(xyz_filt) == 0:
                continue

            if frame % self.MAP_EVERY == 0:
                with self._occ_lock:
                    self._occ.update(xyz_filt, cam_pos)

            if frame % self.MAP_LOG_EVERY == 0:
                rr.set_time("frame", sequence=frame)
                self._log_map(cam_z)

    def _log_map(self, cam_z: float) -> None:
        with self._occ_lock:
            pts = self._occ.occupied_points()
        if len(pts) == 0:
            return
        n   = min(len(pts), self.MAX_MAP)
        idx = np.random.choice(len(pts), n, replace=False) if len(pts) > n else np.arange(n)
        rr.log("world/map", rr.Points3D(
            positions=pts[idx],
            colors=_height_color(pts[idx, 2] - cam_z),
            radii=0.005,
        ))

    def log_stdout(self, pkt: DepthFramePacket, frame: int, fps: float) -> None:
        with self._occ_lock:
            n_occ   = self._occ.n_occupied
            n_total = self._occ.n_total
        pct = 100 * self._last_n_stable / self._last_n_valid if self._last_n_valid > 0 else 0.0
        print(
            f"frame={frame:5d}  "
            f"stable={self._last_n_stable:6d}/{self._last_n_valid:6d} ({pct:.0f}%)  "
            f"occ={n_occ:6d}  free+occ={n_total:6d}  "
            f"fps={fps:.1f}",
            flush=True,
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    rr.init("realsense_depth_map", spawn=True)
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

    src = RealSenseDepthSource(width=848, height=480, fps=15)
    print("RealSense open — Ctrl-C to quit.")

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
