"""ZED Mini → filtered depth packets → world-frame voxel map → OccupancyGrid.

Pipeline stages (one class each):

  DepthFilter          sl.MEASURE.DEPTH + confidence → clean H×W float32 depth
  IntrinsicsReader     reads + caches ZED left-camera K matrix
  PoseReader           ZED VIO pose, camera→world (R, t)
  DepthFramePacket     one complete frame: depth + intrinsics + pose + optional conf
  ─── new stages built on top of the packet ────────────────────────────────────
  DepthBackprojector   depth image + intrinsics + pose → world-frame XYZ (N×3)
  VoxelAccumulator     hash-indexed voxel map, stores actual xyz (not centroids)
  CostGrid             2D height-cost grid from accumulated 3D points
  DepthStreamer        assembles packets, drives all stages, logs to Rerun + stdout

Rerun entities:
  world/camera              Pinhole model (static)
  world/camera/depth        filtered depth image (float32, auto-colorised)
  world/camera/confidence   per-pixel stereo confidence (greyscale)
  world/cloud               current frame in world frame (height-coloured)
  world/map                 accumulated voxel map (height-coloured, every 10 frames)
  world/costmap             2D occupancy grid as RGB image (green/yellow/red)

Usage:
  python -m dimos.navigation.camera_nav.zed_depth_costmap
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
import pyzed.sl as sl
import rerun as rr


# ── Optical-frame → camera_link rotation ─────────────────────────────────────
# ZED get_position() returns the pose of camera_link in world frame, where
# camera_link has X=forward, Y=left, Z=up (RIGHT_HANDED_Z_UP_X_FWD body frame).
# Backprojection with pinhole (u,v,d) gives camera-OPTICAL frame: X=right, Y=down, Z=depth.
# We must rotate optical→link before applying VIO pose, or the Z (height) axis is wrong.
# Matches OPTICAL_ROTATION = Quaternion(-0.5, 0.5, -0.5, 0.5) used in ZEDCamera TF.
# Derived from: x_link=z_opt, y_link=-x_opt, z_link=-y_opt
_R_OPT_TO_LINK = np.array(
    [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32
)

# ── Shared voxel key packing (same scheme as zed_global_map.py) ───────────────

_OFF  = np.int64(100_000)
_MASK = np.int64(0x3FFFF)

def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + _OFF) & _MASK
    return (v[:, 0] << 36) | (v[:, 1] << 18) | v[:, 2]


# ── Packet contract ────────────────────────────────────────────────────────────

@dataclass
class DepthFramePacket:
    """One complete depth frame.

    depth: H×W float32, metres.  NaN = invalid / low-confidence / out-of-range.
    Use np.isfinite(depth) to find valid pixels before projection.
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
    """sl.MEASURE.DEPTH + confidence → clean H×W float32 depth at full resolution.

    No downsampling — every valid pixel reaches the backprojector.
    Confidence threshold 40 removes clear stereo failures while keeping
    real geometry at low-texture surfaces and depth edges.
    """

    CONF_THRESHOLD: float = 40.0
    MIN_DEPTH:      float = 0.3
    MAX_DEPTH:      float = 8.0

    def __init__(self) -> None:
        self._depth_mat = sl.Mat()
        self._conf_mat  = sl.Mat()

    def filter(self, zed: sl.Camera) -> tuple[np.ndarray, np.ndarray]:
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
            | (conf < self.CONF_THRESHOLD)
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
    """ZED VIO camera→world pose.  Identity passthrough until enable() is called."""

    def __init__(self) -> None:
        self._R           = np.eye(3, dtype=np.float32)
        self._t           = np.zeros(3, dtype=np.float32)
        self._active      = False
        self._locked      = False
        self._pose        = sl.Pose()
        self._t0          = time.monotonic()
        self._last_warn   = 0.0

    def enable(self, zed: sl.Camera) -> bool:
        tp = sl.PositionalTrackingParameters()
        tp.enable_imu_fusion   = True
        tp.set_floor_as_origin = True   # Z=0 at floor → costmap Z thresholds are absolute
        ok = zed.enable_positional_tracking(tp) == sl.ERROR_CODE.SUCCESS
        self._active = ok
        if ok:
            print("VIO tracking enabled")
        else:
            print("VIO tracking failed — map will accumulate in camera frame")
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
                print(f"*** VIO LOCKED after {now - self._t0:.1f}s — costmap + world occ will now render ***")
            self._locked = True
        else:
            # Warn every 15 s while still searching so the user isn't left guessing
            if now - max(self._last_warn, self._t0) >= 15.0:
                print(f"    VIO searching … {now - self._t0:.0f}s elapsed  "
                      "— move camera slowly to help feature tracking")
                self._last_warn = now

    @property
    def R(self) -> np.ndarray: return self._R

    @property
    def t(self) -> np.ndarray: return self._t

    @property
    def active(self) -> bool: return self._active

    @property
    def locked(self) -> bool:
        """True once VIO has reported at least one OK fix. Before this, R=I and t=0
        (camera-optical frame mislabelled as world) — do not accumulate into the map."""
        return self._locked


# ── Stage 4: Backprojector ────────────────────────────────────────────────────

class DepthBackprojector:
    """DepthFramePacket → world-frame XYZ (N×3 float32).

    Three-step transform (matches ZEDNavBridge exactly):
      1. Pinhole backproject (u,v,d) → camera-optical frame (X=right, Y=down, Z=depth)
      2. _R_OPT_TO_LINK  → camera_link frame (X=forward, Y=left, Z=up)
      3. VIO pose_R / pose_t  → world frame (X=forward, Y=left, Z=up)

    Step 2 is the fix: without it, pose_R acts on optical-frame points where Z=depth
    instead of Z=up, producing a height map built on depth-from-camera rather than
    terrain elevation — the root cause of the sparse / meaningless costmap.
    """

    def project(self, pkt: DepthFramePacket) -> np.ndarray:
        d = pkt.depth                       # H×W float32, NaN=invalid
        H, W = d.shape
        fx, fy, cx, cy = pkt.intrinsics

        us = np.arange(W, dtype=np.float32)
        vs = np.arange(H, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)

        valid = np.isfinite(d)
        dd    = d[valid]
        x_opt = (uu[valid] - cx) * dd / fx   # right in image
        y_opt = (vv[valid] - cy) * dd / fy   # down in image
        z_opt = dd                            # depth into scene

        xyz_opt = np.column_stack([x_opt, y_opt, z_opt]).astype(np.float32)

        # Step 2: optical → camera_link (x_link=z_opt, y_link=-x_opt, z_link=-y_opt)
        xyz_link = xyz_opt @ _R_OPT_TO_LINK.T

        # Step 3: camera_link → world via VIO pose
        return xyz_link @ pkt.pose_R.T + pkt.pose_t


# ── Stage 5: Voxel accumulator ────────────────────────────────────────────────

class VoxelAccumulator:
    """Hash-indexed 3D voxel map storing actual observed xyz positions.

    Identity layer: 2 cm voxel hash decides new vs revisit.
    Fine layer:     actual stereo positions stored and refined via running average.
    Only xyz is stored (no colour) — the costmap only needs geometry.
    """

    def __init__(self, voxel_size: float = 0.02, capacity: int = 500_000) -> None:
        self._v   = voxel_size
        self._cap = capacity
        self._idx: dict[int, int] = {}
        self._xyz  = np.empty((capacity, 3), dtype=np.float32)
        self._conf = np.zeros(capacity, dtype=np.uint16)
        self._n    = 0

    def add(self, xyz: np.ndarray) -> None:
        if len(xyz) == 0:
            return
        vk   = np.floor(xyz / self._v).astype(np.int32)
        keys = _pack(vk)
        _, first = np.unique(keys, return_index=True)
        keys = keys[first]; xyz = xyz[first]

        key_list = keys.tolist()
        exists   = np.array([k in self._idx for k in key_list], dtype=bool)

        if exists.any():
            rows  = np.array([self._idx[int(k)] for k in keys[exists].tolist()], dtype=np.int32)
            old_c = self._conf[rows].astype(np.float32)
            new_c = np.minimum(old_c + 1, 65_535)
            w     = (old_c / new_c)[:, None]
            self._xyz[rows]  = self._xyz[rows] * w + xyz[exists] * (1 - w)
            self._conf[rows] = new_c.astype(np.uint16)

        new_xyz  = xyz[~exists]
        new_keys = keys[~exists]
        n_new    = len(new_xyz)
        if n_new == 0:
            return
        if self._n + n_new > self._cap:
            self._grow()
        rows = slice(self._n, self._n + n_new)
        self._xyz[rows]  = new_xyz
        self._conf[rows] = 1
        for i, k in enumerate(new_keys.tolist()):
            self._idx[int(k)] = self._n + i
        self._n += n_new

    def clear(self) -> None:
        self._idx.clear()
        self._n = 0

    @property
    def xyz(self) -> np.ndarray: return self._xyz[:self._n]

    @property
    def count(self) -> int: return self._n

    def _grow(self) -> None:
        new_cap  = self._cap * 2
        new_xyz  = np.empty((new_cap, 3), dtype=np.float32)
        new_conf = np.zeros(new_cap, dtype=np.uint16)
        new_xyz[:self._n]  = self._xyz[:self._n]
        new_conf[:self._n] = self._conf[:self._n]
        self._xyz = new_xyz; self._conf = new_conf; self._cap = new_cap


# ── Stage 6: Height-cost grid ─────────────────────────────────────────────────

class CostGrid:
    """2D occupancy grid from accumulated 3D points using height-gap cost.

    Z convention: set_floor_as_origin=True → Z=0 at floor.
      0 – FLOOR_Z   → floor / free
      FLOOR_Z – OBS_Z → obstacle band (robot body height)
      above OBS_Z    → overhead — ignored

    Cost logic (same as dimos CostMapper height_cost):
      per cell: max_z - min_z = height gap
      gap > CAN_PASS_UNDER → use min_z  (beam / overhead clearance → free)
      else: cost = clip((eff_z / OBS_Z) × 100, 0, 100)
    """

    RES           = 0.05   # metres per cell
    FLOOR_Z       = 0.12   # below → floor
    OBS_Z         = 1.8    # above → overhead (not mapped)
    CAN_PASS_UNDER = 0.6   # height gap above which low edge is used

    def build(self, xyz: np.ndarray) -> tuple[np.ndarray, float, float] | tuple[None, None, None]:
        """Return (grid H×W uint8 RGB, origin_x, origin_y) or (None, None, None)."""
        if len(xyz) == 0:
            return None, None, None

        # Only points in the meaningful height band
        ok  = (xyz[:, 2] > -0.3) & (xyz[:, 2] < self.OBS_Z)
        xyz = xyz[ok]
        if len(xyz) == 0:
            return None, None, None

        ox = xyz[:, 0].min() - 1.0
        oy = xyz[:, 1].min() - 1.0
        W  = max(1, int(np.ceil((xyz[:, 0].max() + 1.0 - ox) / self.RES)))
        H  = max(1, int(np.ceil((xyz[:, 1].max() + 1.0 - oy) / self.RES)))

        col = np.clip(((xyz[:, 0] - ox) / self.RES).astype(np.int32), 0, W - 1)
        row = np.clip(((xyz[:, 1] - oy) / self.RES).astype(np.int32), 0, H - 1)

        z_min = np.full((H, W),  np.inf, np.float32)
        z_max = np.full((H, W), -np.inf, np.float32)
        np.minimum.at(z_min, (row, col), xyz[:, 2])
        np.maximum.at(z_max, (row, col), xyz[:, 2])

        observed = np.isfinite(z_min) & np.isfinite(z_max)
        gap      = z_max - z_min
        eff_z    = np.where(gap > self.CAN_PASS_UNDER, z_min, z_max)

        # Cost 0–100
        cost = np.zeros((H, W), np.float32)
        cost[observed] = np.clip(
            (eff_z[observed] / self.OBS_Z) * 100.0, 0.0, 100.0
        )
        cost[eff_z <= self.FLOOR_Z] = 0.0   # floor → free

        # Colorise to RGB for Rerun
        rgb = np.zeros((H, W, 3), dtype=np.uint8)
        rgb[~observed] = (80, 80, 80)        # unknown → dark grey
        free_mask = observed & (cost < 10)
        rgb[free_mask] = (30, 180, 30)       # free → green
        obs_mask = observed & (cost >= 10)
        t  = (cost[obs_mask] / 100.0).clip(0, 1)
        r  = (255 * t).astype(np.uint8)
        g  = (180 * (1 - t)).astype(np.uint8)
        rgb[obs_mask] = np.column_stack([r, g, np.zeros_like(r)])

        return rgb, ox, oy


# ── Stage 7: Bird's-eye occupancy ────────────────────────────────────────────

class BirdsEyeOccupancy:
    """Single-frame bird's-eye occupancy image from depth.

    No VIO, no accumulation.  One depth frame → one colour image.

    Layout (camera at centre of image):
      forward  = up in image
      left     = left in image
      gray     = unknown (no ray reached here)
      green    = free   (ray passed through)
      red      = occupied (surface hit)
    """

    RES  = 0.10   # metres per cell
    HALF = 5.0    # grid covers ±HALF metres around camera

    def __init__(self) -> None:
        self._n   = int(self.HALF * 2 / self.RES)   # 100 × 100 cells
        self._mid = self._n // 2

    def build(
        self, depth: np.ndarray, fx: float, fy: float, cx: float, cy: float
    ) -> np.ndarray:
        n, mid = self._n, self._mid
        H, W   = depth.shape

        grid  = np.zeros((n, n), dtype=np.uint8)   # 0=unknown 1=free 2=occupied
        valid = np.isfinite(depth)

        if valid.any():
            u = np.tile(np.arange(W, dtype=np.float32), (H, 1))

            d      = depth[valid]
            x_fwd  = d                             # camera-link X = depth (forward)
            y_left = -(u[valid] - cx) * d / fx    # camera-link Y = lateral

            col_hit = np.clip((mid + y_left / self.RES).astype(np.int32), 0, n - 1)
            row_hit = np.clip((mid - x_fwd  / self.RES).astype(np.int32), 0, n - 1)

            # Mark free along each ray — fully vectorised, no Python loop.
            t        = np.linspace(0.1, 0.9, 9, dtype=np.float32)
            row_free = np.clip(mid - np.outer(t, x_fwd  / self.RES).astype(np.int32), 0, n - 1)
            col_free = np.clip(mid + np.outer(t, y_left / self.RES).astype(np.int32), 0, n - 1)
            grid[row_free.ravel(), col_free.ravel()] = 1   # free

            # Count how many depth pixels land in each cell.
            # Flying pixels (stereo edge artifacts) land in isolated cells — count=1.
            # Real surfaces (walls, furniture) produce many pixels per cell — count≥3.
            hits = np.zeros((n, n), dtype=np.uint16)
            np.add.at(hits, (row_hit, col_hit), 1)
            grid[hits >= 5] = 2   # occupied — overwrites free, requires 5+ hits

        rgb = np.full((n, n, 3), (200, 180, 0), dtype=np.uint8)  # unknown: yellow
        rgb[grid == 1] = (30, 180, 30)                          # free:     green
        rgb[grid == 2] = (220, 50,  50)                         # occupied: red
        rgb[mid,  mid] = (255, 255, 255)                        # camera:   white dot
        return rgb


# ── Stage 8: World occupancy grid ────────────────────────────────────────────

class WorldOccupancyGrid:
    """Persistent world-frame bird's-eye occupancy map.

    Accumulates evidence across frames using VIO pose. Each cell tracks:
      _hits — depth pixels that landed here  (occupied evidence)
      _rays — cast rays that passed through  (free evidence)

    Occupied if hits >= OCC_THRESH.
    Free     if rays >= FREE_THRESH and hits < OCC_THRESH.
    Unknown  if no evidence.

    Call update() only when VIO is locked — xyz_world must be in world frame.
    Image layout: forward = up, left = left, world origin = centre.
    """

    RES         = 0.10   # metres per cell
    HALF        = 10.0   # ±10 m  →  200×200 cells
    OCC_THRESH  = 5      # depth hits to confirm occupied
    FREE_THRESH = 3      # ray-through votes to confirm free

    def __init__(self) -> None:
        n           = int(self.HALF * 2 / self.RES)
        self._n     = n
        self._mid   = n // 2
        self._hits  = np.zeros((n, n), dtype=np.uint32)
        self._rays  = np.zeros((n, n), dtype=np.uint32)
        self._cam_t = np.zeros(3, dtype=np.float32)

    def _xy_to_rc(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        n, mid = self._n, self._mid
        row = np.clip((mid - x / self.RES).astype(np.int32), 0, n - 1)
        col = np.clip((mid - y / self.RES).astype(np.int32), 0, n - 1)
        return row, col

    def update(self, xyz_world: np.ndarray, cam_t: np.ndarray) -> None:
        """Accumulate one world-frame depth frame into the map.

        xyz_world: (N, 3) float32  world-frame hit points
        cam_t:     (3,)   float32  camera world position (pose_t)
        """
        self._cam_t = cam_t.copy()
        n = self._n

        row_hit, col_hit = self._xy_to_rc(xyz_world[:, 0], xyz_world[:, 1])

        # Occupied evidence: count hits per cell (bincount >> np.add.at)
        flat_hit    = row_hit * n + col_hit
        self._hits += np.bincount(flat_hit, minlength=n * n).reshape(n, n).astype(np.uint32)

        # Free evidence: ray cast from camera to each hit point
        cam_row = int(np.clip(self._mid - cam_t[0] / self.RES, 0, n - 1))
        cam_col = int(np.clip(self._mid - cam_t[1] / self.RES, 0, n - 1))

        t        = np.linspace(0.1, 0.9, 9, dtype=np.float32)   # (9,)
        row_free = np.clip(
            (cam_row + np.outer(t, row_hit - cam_row)).astype(np.int32), 0, n - 1
        )   # (9, N)
        col_free = np.clip(
            (cam_col + np.outer(t, col_hit - cam_col)).astype(np.int32), 0, n - 1
        )   # (9, N)
        flat_free   = row_free.ravel() * n + col_free.ravel()
        self._rays += np.bincount(flat_free, minlength=n * n).reshape(n, n).astype(np.uint32)

    def reset(self) -> None:
        self._hits[:] = 0
        self._rays[:] = 0
        self._cam_t[:] = 0

    def render(self) -> np.ndarray:
        """Return n×n×3 uint8 RGB image of current map state."""
        n, mid = self._n, self._mid

        occ  = self._hits >= self.OCC_THRESH
        free = (self._rays >= self.FREE_THRESH) & ~occ

        rgb = np.full((n, n, 3), (200, 180, 0), dtype=np.uint8)  # unknown: yellow
        rgb[free] = (30, 180, 30)                                  # free:     green
        rgb[occ]  = (220, 50,  50)                                 # occupied: red

        # Camera marker — 5×5 white square (distinct from yellow background)
        cam_row = int(np.clip(mid - self._cam_t[0] / self.RES, 0, n - 1))
        cam_col = int(np.clip(mid - self._cam_t[1] / self.RES, 0, n - 1))
        r0 = max(0, cam_row - 2);  r1 = min(n, cam_row + 3)
        c0 = max(0, cam_col - 2);  c1 = min(n, cam_col + 3)
        rgb[r0:r1, c0:c1] = (255, 255, 255)

        return rgb


# ── Stage 9: Streamer ─────────────────────────────────────────────────────────

_JET_T = np.linspace(0, 1, 256, dtype=np.float32)
_JET_R = np.clip(1.5 - np.abs(4 * _JET_T - 3), 0, 1)
_JET_G = np.clip(1.5 - np.abs(4 * _JET_T - 2), 0, 1)
_JET_B = np.clip(1.5 - np.abs(4 * _JET_T - 1), 0, 1)
_JET   = (np.column_stack([_JET_R, _JET_G, _JET_B]) * 255).astype(np.uint8)

def _height_colors(z: np.ndarray, z_min: float = -0.1, z_max: float = 1.8) -> np.ndarray:
    t   = np.clip((z - z_min) / max(z_max - z_min, 1e-6), 0, 1)
    idx = (t * 255).astype(np.int32).clip(0, 255)
    return _JET[idx]


class DepthStreamer:
    """Assembles packets, drives all downstream stages, logs to Rerun."""

    MAX_CLOUD = 50_000   # per-frame cap to stay below Rerun h2 limit
    MAX_MAP   = 80_000   # map visualisation cap
    MAP_EVERY = 10       # frames between map + costmap updates

    def __init__(
        self,
        depth_filter:  DepthFilter,
        intrinsics:    IntrinsicsReader,
        pose:          PoseReader,
        backproj:      DepthBackprojector,
        voxels:        VoxelAccumulator,
        costgrid:      CostGrid,
        emit_confidence: bool = True,
    ) -> None:
        self._filt    = depth_filter
        self._intr    = intrinsics
        self._pose    = pose
        self._bp      = backproj
        self._vox     = voxels
        self._cg      = costgrid
        self._emit_c  = emit_confidence
        self._birdseye    = BirdsEyeOccupancy()
        self._world_occ   = WorldOccupancyGrid()
        self._was_locked  = False
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
            confidence = conf if self._emit_c else None,
        )

    def process(self, pkt: DepthFramePacket, frame: int) -> None:
        """Backproject → accumulate → (periodically) rebuild costmap.

        Before VIO lock: pose_R=I so xyz is in camera-link frame (X=fwd, Z=up).
        Voxels accumulate in that frame — costmap renders immediately.
        On first VIO lock: voxels are cleared and world-frame accumulation begins,
        giving a clean handoff without camera-frame contamination.
        """
        xyz = self._bp.project(pkt)
        if len(xyz) > 0:
            just_locked = self._pose.locked and not self._was_locked
            if just_locked:
                self._vox.clear()        # discard camera-frame voxels; restart in world frame
                self._world_occ.reset()  # same clean handoff for the occupancy grid
            self._vox.add(xyz)
            self._world_occ.update(xyz, pkt.pose_t)
        self._was_locked = self._pose.locked
        self._log_depth_frame(pkt, xyz)
        if frame % self.MAP_EVERY == 0:
            self._log_map_and_costmap()

    # ── Rerun logging ─────────────────────────────────────────────────────────

    def _log_depth_frame(self, pkt: DepthFramePacket, xyz: np.ndarray) -> None:
        if not self._pinhole_logged:
            rr.log("world/camera", rr.Pinhole(
                image_from_camera=pkt.K, width=pkt.width, height=pkt.height,
            ), static=True)
            self._pinhole_logged = True

        rr.log("world/camera/depth", rr.DepthImage(pkt.depth, meter=1.0))

        # Bird's-eye occupancy — no VIO needed, runs every frame
        fx, fy, cx, cy = pkt.intrinsics
        rr.log("occupancy/birdseye", rr.Image(
            self._birdseye.build(pkt.depth, fx, fy, cx, cy)
        ))

        if pkt.confidence is not None:
            conf_u8 = np.nan_to_num(pkt.confidence, nan=0.0).clip(0, 100)
            rr.log("world/camera/confidence", rr.Image((conf_u8 * 2.55).astype(np.uint8)))

        if len(xyz) > 0:
            n   = min(len(xyz), self.MAX_CLOUD)
            idx = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(n)
            rr.log("world/cloud", rr.Points3D(
                positions=xyz[idx],
                colors=_height_colors(xyz[idx, 2]),
                radii=0.003,
            ))

    def _log_map_and_costmap(self) -> None:
        xyz = self._vox.xyz
        if len(xyz) > 0:
            # Accumulated voxel map
            n   = min(len(xyz), self.MAX_MAP)
            idx = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(len(xyz))
            rr.log("world/map", rr.Points3D(
                positions=xyz[idx],
                colors=_height_colors(xyz[idx, 2]),
                radii=0.004,
            ))

            # 2-D costmap as RGB image
            rgb, ox, oy = self._cg.build(xyz)
            if rgb is not None:
                rr.log("world/costmap", rr.Image(rgb))
            else:
                z_min, z_max = float(xyz[:, 2].min()), float(xyz[:, 2].max())
                print(f"    costmap: {len(xyz)} voxels but Z range [{z_min:.2f}, {z_max:.2f}]m — "
                      "all filtered (expect Z in -0.3 to 1.8 m with floor-as-origin)")

        # World-frame occupancy — renders as soon as VIO locks
        if self._pose.locked:
            rr.log("occupancy/world", rr.Image(self._world_occ.render()))

    def log_stdout(self, pkt: DepthFramePacket, frame: int, fps: float) -> None:
        tx, ty, tz = pkt.pose_t
        lock = "LOCKED" if self._pose.locked else "no-lock"
        print(
            f"frame={frame:5d}  "
            f"valid={pkt.valid_fraction * 100:5.1f}%  "
            f"map={self._vox.count:6d}  "
            f"pose=({tx:.2f},{ty:.2f},{tz:.2f})  "
            f"vio={lock}  "
            f"fps={fps:.1f}"
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    # Fork Rerun BEFORE zed.open() — ZED starts internal capture threads on open();
    # forking after those threads exist is unsafe on macOS (fork-after-threads segfault).
    rr.init("zed_depth_costmap", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

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

    depth_filter = DepthFilter()
    intrinsics   = IntrinsicsReader()
    pose         = PoseReader()
    backproj     = DepthBackprojector()
    voxels       = VoxelAccumulator(voxel_size=0.02)
    costgrid     = CostGrid()
    streamer     = DepthStreamer(
        depth_filter, intrinsics, pose, backproj, voxels, costgrid,
        emit_confidence=True,
    )

    pose.enable(zed)  # safe: Rerun already forked before zed.open()

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
    print(f"done — {voxels.count} voxels in map")


if __name__ == "__main__":
    main()
