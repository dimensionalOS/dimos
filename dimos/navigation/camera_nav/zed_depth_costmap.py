"""ZED Mini → persistent world-frame 3D occupancy map.

Pipeline stages:

  DepthFilter        sl.MEASURE.DEPTH + CONFIDENCE → depth H×W (NaN where bad)
  IntrinsicsReader   reads + caches ZED left-camera K matrix
  PoseReader         ZED VIO pose, camera→world (R, t)
  DepthFramePacket   one complete frame: depth + intrinsics + pose + confidence
  DepthBackprojector depth image → world-frame XYZ + per-point confidence (N×3, N)
  VoxelAccumulator   persistent world-frame map: deduplicates + refines across frames
  DepthStreamer      assembles packets, drives accumulation, logs to Rerun

Filtering before any point enters the map:
  1. NaN / out-of-range depth removed by DepthFilter
  2. Low-confidence stereo (conf < CONF_SURE=60) kept only in live cloud, not the map
  3. Isolated single-pixel artifacts removed per-frame before accumulation

Rerun entities:
  world/camera        Pinhole model (static)
  world/camera/depth  filtered depth image (float32, auto-colorised)
  world/cloud         current-frame scan: red = sure (≥60)  yellow = unsure (25–60)
  world/map           persistent accumulated map (white, grows as camera moves)

Usage:
  python -m dimos.navigation.camera_nav.zed_depth_costmap
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
import pyzed.sl as sl
import rerun as rr


# ── Voxel key packing ────────────────────────────────────────────────────────
# 18-bit per axis → supports ±100 000 voxels/axis (±2 km at 2 cm resolution).

_VOFF  = np.int64(100_000)
_VMASK = np.int64(0x3FFFF)

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

    depth: H×W float32, metres.  NaN = invalid/low-confidence/out-of-range.
    """
    timestamp:  float
    depth:      np.ndarray           # H×W float32, metres, NaN=invalid
    intrinsics: np.ndarray           # [fx, fy, cx, cy] float64
    width:      int
    height:     int
    pose_R:     np.ndarray           # 3×3 float32  camera→world rotation
    pose_t:     np.ndarray           # (3,) float32 camera→world translation
    confidence: np.ndarray | None = field(default=None)  # H×W float32 0-100

    @property
    def valid_fraction(self) -> float:
        return float(np.isfinite(self.depth).mean())

    @property
    def K(self) -> np.ndarray:
        fx, fy, cx, cy = self.intrinsics
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


# ── Stage 1: Depth filter ──────────────────────────────────────────────────────

class DepthFilter:
    """sl.MEASURE.DEPTH + CONFIDENCE → clean H×W float32 depth.

    Two confidence tiers are exposed downstream:
      CONF_SURE (≥60): high-confidence stereo match → shown red
      CONF_MIN  (≥25): lower-confidence but still usable → shown yellow
    Pixels below CONF_MIN are set to NaN and discarded entirely.
    """

    CONF_MIN:   float = 25.0   # discard below this — too noisy to use at all
    MIN_DEPTH:  float = 0.3
    MAX_DEPTH:  float = 8.0

    def __init__(self) -> None:
        self._depth_mat = sl.Mat()
        self._conf_mat  = sl.Mat()

    def filter(self, zed: sl.Camera) -> tuple[np.ndarray, np.ndarray]:
        """Return (depth H×W, conf H×W).  depth[pixel] = NaN where unusable."""
        zed.retrieve_measure(self._depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU)
        depth = self._depth_mat.get_data().astype(np.float32)
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        zed.retrieve_measure(self._conf_mat, sl.MEASURE.CONFIDENCE, sl.MEM.CPU)
        conf = self._conf_mat.get_data().astype(np.float32)
        if conf.ndim == 3:
            conf = conf[:, :, 0]

        invalid = (
            ~np.isfinite(depth)
            | (depth <= self.MIN_DEPTH)
            | (depth >  self.MAX_DEPTH)
            | ~np.isfinite(conf)
            | (conf < self.CONF_MIN)
        )
        depth[invalid] = np.nan
        return depth, conf


# ── Stage 2: Intrinsics reader ────────────────────────────────────────────────

class IntrinsicsReader:
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


# ── Stage 3: Pose reader ──────────────────────────────────────────────────────

class PoseReader:
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
    """DepthFramePacket → (xyz N×3, conf N) in camera-link frame.

    Three-step transform:
      1. Pinhole backproject (u,v,d) → camera-optical frame
      2. _R_OPT_TO_LINK            → camera_link frame (X=fwd, Y=left, Z=up)
      3. VIO pose_R / pose_t        → world frame  (identity before VIO locks)

    Also returns per-point confidence (same order as xyz rows) so the caller
    can color points by stereo reliability without a separate lookup.
    """

    def project(self, pkt: DepthFramePacket) -> tuple[np.ndarray, np.ndarray]:
        """Return (xyz (N,3) float32, conf (N,) float32)."""
        d = pkt.depth
        H, W = d.shape
        fx, fy, cx, cy = pkt.intrinsics

        uu, vv = np.meshgrid(
            np.arange(W, dtype=np.float32),
            np.arange(H, dtype=np.float32),
        )

        valid = np.isfinite(d)
        dd    = d[valid]
        x_opt = (uu[valid] - cx) * dd / fx
        y_opt = (vv[valid] - cy) * dd / fy
        z_opt = dd

        xyz_opt  = np.column_stack([x_opt, y_opt, z_opt]).astype(np.float32)
        xyz_link = xyz_opt @ _R_OPT_TO_LINK.T
        xyz_world = xyz_link @ pkt.pose_R.T + pkt.pose_t

        conf = (
            pkt.confidence[valid].astype(np.float32)
            if pkt.confidence is not None
            else np.full(len(xyz_world), 100.0, dtype=np.float32)
        )
        return xyz_world, conf


# ── Stage 5: Voxel accumulator ────────────────────────────────────────────────

class VoxelAccumulator:
    """Persistent world-frame 3D map.

    Identity layer (2 cm voxel hash): decides new voxel vs revisit in O(1).
    Fine layer (actual stereo positions): stored and refined via running average
    so geometry converges toward the true surface without collapsing to a grid centre.

    A voxel must be observed in at least min_obs separate frames to be exported —
    this is the cross-frame equivalent of the per-frame isolation filter.
    """

    def __init__(self, voxel_size: float = 0.02, capacity: int = 500_000) -> None:
        self._v   = voxel_size
        self._cap = capacity
        self._idx: dict[int, int] = {}
        self._xyz  = np.empty((capacity, 3), dtype=np.float32)
        self._obs  = np.zeros(capacity, dtype=np.uint16)   # frame observation count
        self._n    = 0

    def add(self, xyz: np.ndarray) -> None:
        """Merge one filtered, world-frame point array into the map."""
        if len(xyz) == 0:
            return
        vk   = np.floor(xyz / self._v).astype(np.int32)
        keys = _pack(vk)
        # One representative point per voxel within this batch
        _, first = np.unique(keys, return_index=True)
        keys = keys[first];  xyz = xyz[first]

        key_list = keys.tolist()
        exists   = np.array([k in self._idx for k in key_list], dtype=bool)

        if exists.any():
            rows  = np.array([self._idx[int(k)] for k in keys[exists].tolist()], dtype=np.int32)
            old_n = self._obs[rows].astype(np.float32)
            new_n = np.minimum(old_n + 1, 65_535)
            w     = (old_n / new_n)[:, None]          # weight of stored observation
            self._xyz[rows] = self._xyz[rows] * w + xyz[exists] * (1 - w)
            self._obs[rows] = new_n.astype(np.uint16)

        new_xyz  = xyz[~exists]
        new_keys = keys[~exists]
        n_new    = len(new_xyz)
        if n_new == 0:
            return
        if self._n + n_new > self._cap:
            self._grow()
        sl = slice(self._n, self._n + n_new)
        self._xyz[sl] = new_xyz
        self._obs[sl] = 1
        for i, k in enumerate(new_keys.tolist()):
            self._idx[int(k)] = self._n + i
        self._n += n_new

    def stable_xyz(self, min_obs: int = 2) -> np.ndarray:
        """Points seen in at least min_obs frames — cross-frame noise filter."""
        if self._n == 0:
            return np.zeros((0, 3), dtype=np.float32)
        mask = self._obs[:self._n] >= min_obs
        return self._xyz[:self._n][mask]

    @property
    def count(self) -> int:
        return self._n

    def _grow(self) -> None:
        cap2     = self._cap * 2
        new_xyz  = np.empty((cap2, 3), dtype=np.float32)
        new_obs  = np.zeros(cap2, dtype=np.uint16)
        new_xyz[:self._n] = self._xyz[:self._n]
        new_obs[:self._n] = self._obs[:self._n]
        self._xyz = new_xyz;  self._obs = new_obs;  self._cap = cap2


# ── Stage 6: Streamer ─────────────────────────────────────────────────────────

class DepthStreamer:
    """Assembles packets, builds persistent world-frame map, logs to Rerun."""

    CONF_SURE = 60       # confidence ≥ this → goes into map + shown red
    MAX_CLOUD = 50_000   # Rerun per-frame point cap
    MAX_MAP   = 100_000  # Rerun map point cap
    MAP_EVERY = 10       # frames between map updates

    def __init__(
        self,
        depth_filter: DepthFilter,
        intrinsics:   IntrinsicsReader,
        pose:         PoseReader,
        backproj:     DepthBackprojector,
    ) -> None:
        self._filt           = depth_filter
        self._intr           = intrinsics
        self._pose           = pose
        self._bp             = backproj
        self._vox            = VoxelAccumulator(voxel_size=0.02)
        self._pinhole_logged = False

    def assemble(self, zed: sl.Camera, ts: float) -> DepthFramePacket:
        self._intr.read(zed)
        self._pose.update(zed)
        depth, conf = self._filt.filter(zed)
        return DepthFramePacket(
            timestamp  = ts,
            depth      = depth,
            intrinsics = self._intr.intrinsics,
            width      = self._intr.width,
            height     = self._intr.height,
            pose_R     = self._pose.R.copy(),
            pose_t     = self._pose.t.copy(),
            confidence = conf,
        )

    def process(self, pkt: DepthFramePacket, frame: int) -> None:
        if not self._pinhole_logged:
            rr.log("world/camera", rr.Pinhole(
                image_from_camera=pkt.K, width=pkt.width, height=pkt.height,
            ), static=True)
            self._pinhole_logged = True

        rr.log("world/camera/depth", rr.DepthImage(pkt.depth, meter=1.0))

        xyz, conf = self._bp.project(pkt)
        if len(xyz) == 0:
            return

        # ── Live cloud: both tiers, every frame ──────────────────────────────
        n   = min(len(xyz), self.MAX_CLOUD)
        idx = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(n)
        sure   = conf[idx] >= self.CONF_SURE
        colors = np.where(
            sure[:, None],
            np.array([[220,  50,  50]], dtype=np.uint8),   # red    = sure
            np.array([[200, 180,   0]], dtype=np.uint8),   # yellow = unsure
        )
        rr.log("world/cloud", rr.Points3D(positions=xyz[idx], colors=colors, radii=0.003))

        # ── Persistent map: only sure points, only when VIO is locked ─────────
        if self._pose.locked:
            xyz_sure = xyz[conf >= self.CONF_SURE]
            xyz_clean = _filter_isolated(xyz_sure)   # drop single-pixel artifacts
            self._vox.add(xyz_clean)

        if frame % self.MAP_EVERY == 0:
            self._log_map()

    def _log_map(self) -> None:
        pts = self._vox.stable_xyz(min_obs=2)   # cross-frame noise filter
        if len(pts) == 0:
            return
        n   = min(len(pts), self.MAX_MAP)
        idx = np.random.choice(len(pts), n, replace=False) if len(pts) > n else np.arange(n)
        rr.log("world/map", rr.Points3D(
            positions=pts[idx],
            colors=np.full((n, 3), [220, 220, 220], dtype=np.uint8),  # white-gray
            radii=0.005,
        ))

    def log_stdout(self, pkt: DepthFramePacket, frame: int, fps: float) -> None:
        lock = "LOCKED" if self._pose.locked else "searching"
        print(
            f"frame={frame:5d}  "
            f"valid={pkt.valid_fraction * 100:5.1f}%  "
            f"map={self._vox.count:6d}  "
            f"vio={lock}  "
            f"fps={fps:.1f}"
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    # Fork Rerun BEFORE zed.open() — ZED capture threads make post-open fork unsafe.
    rr.init("zed_depth_costmap", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/cloud", rr.Points3D([[0, 0, 0]]), static=True)
    rr.log("world/map",   rr.Points3D([[0, 0, 0]]), static=True)

    zed = sl.Camera()
    ip  = sl.InitParameters()
    ip.camera_resolution      = sl.RESOLUTION.VGA
    ip.camera_fps             = 15
    ip.depth_mode             = sl.DEPTH_MODE.NEURAL
    ip.coordinate_system      = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    ip.coordinate_units       = sl.UNIT.METER
    ip.depth_maximum_distance = 8.0

    if zed.open(ip) != sl.ERROR_CODE.SUCCESS:
        print("ZED failed to open"); return

    print("camera open — Ctrl-C to quit.")

    streamer = DepthStreamer(
        DepthFilter(),
        IntrinsicsReader(),
        PoseReader(),
        DepthBackprojector(),
    )

    streamer._pose.enable(zed)

    rt    = sl.RuntimeParameters()
    frame = 0
    t0    = time.monotonic()

    try:
        while True:
            if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
                continue
            ts  = time.monotonic()
            pkt = streamer.assemble(zed, ts)
            streamer.process(pkt, frame)
            streamer.log_stdout(pkt, frame, frame / max(ts - t0, 1e-6))
            frame += 1
    except KeyboardInterrupt:
        pass

    zed.close()
    print("done")


if __name__ == "__main__":
    main()
