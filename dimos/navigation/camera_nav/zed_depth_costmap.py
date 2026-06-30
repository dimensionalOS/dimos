"""ZED Mini → persistent world-frame 3D occupancy map.

Pipeline stages:

  DepthFilter           sl.MEASURE.DEPTH + CONFIDENCE → depth H×W (NaN where bad)
  IntrinsicsReader      reads + caches ZED left-camera K matrix
  PoseReader            ZED VIO pose, camera→world (R, t)
  DepthFramePacket      one complete frame: depth + intrinsics + pose + confidence
  ConfidenceWeighter    confidence score → evidence weight (0.0–1.0) per pixel
  ConsistencyEstimator  image-space neighbourhood depth agreement → boost multiplier per pixel
  DepthBackprojector    depth image → world-frame XYZ + per-point evidence (N×3, N)
  VoxelAccumulator      unified persistent world-frame map: score-based evidence accumulation
  DepthStreamer         assembles packets, drives accumulation, logs to Rerun

Evidence pipeline (replaces binary fg/bg classification):
  1. NaN / out-of-range depth removed by DepthFilter
  2. conf → weight via ConfidenceWeighter  (below conf_min → weight = 0, excluded)
  3. weight × consistency_boost → final per-pixel evidence
  4. Voxels accumulate evidence_score across frames
  5. stable map = voxels with score >= min_score

Rerun entities:
  world/camera        Pinhole model (static)
  world/camera/depth  filtered depth image (float32, auto-colorised)
  world/cloud         current-frame scan coloured by evidence weight (blue=low, red=high)
  world/map           persistent accumulated map, height-colored:
                        blue  = ankle / low clutter  (0.05–0.5 m)
                        green = knee / waist          (0.5–1.0 m)
                        red   = shoulder / head       (1.0–2.0 m)

Usage:
  python -m dimos.navigation.camera_nav.zed_depth_costmap
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
import pyzed.sl as sl
import rerun as rr
import rerun.blueprint as rrb
from scipy.ndimage import median_filter, uniform_filter


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


# ── Optical-frame → camera_link rotation ─────────────────────────────────────
# ZED get_position() returns camera_link pose in world (X=fwd, Y=left, Z=up).
# Pinhole backproject gives camera-OPTICAL frame (X=right, Y=down, Z=depth).
# Rotate optical→link before applying VIO pose so Z=up, not Z=depth.
_R_OPT_TO_LINK = np.array(
    [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32
)


# ── Evidence weighting ────────────────────────────────────────────────────────

@dataclass
class ConfidenceConfig:
    conf_min: float = 25.0   # below this → zero weight, point excluded
    conf_full: float = 85.0  # above this → full weight (1.0)
    curve: float = 0.7       # exponent applied after linear rescale;
                             # < 1 lifts medium-confidence weights closer to 1.0


class ConfidenceWeighter:
    """Converts ZED stereo confidence (0–100) to an evidence weight (0.0–1.0).

    Weight = ((conf - conf_min) / (conf_full - conf_min)) ** curve, clamped to [0, 1].
    Pixels below conf_min are assigned weight 0 and excluded from backprojection.
    """

    def __init__(self, cfg: ConfidenceConfig | None = None) -> None:
        self.cfg = cfg or ConfidenceConfig()

    def compute(self, conf: np.ndarray) -> np.ndarray:
        """Return per-pixel weight H×W (float32, 0.0 where conf < conf_min)."""
        c = self.cfg
        ratio = (conf - c.conf_min) / (c.conf_full - c.conf_min)
        weight = np.clip(ratio, 0.0, 1.0) ** c.curve
        weight[conf < c.conf_min] = 0.0
        return weight.astype(np.float32)


# ── Spatial consistency ───────────────────────────────────────────────────────

@dataclass
class ConsistencyConfig:
    window_radius: int = 3         # pixels each side; 3 → 7×7 neighbourhood
    depth_tolerance: float = 0.15  # metres; neighbours within this → consistent
    min_valid_fraction: float = 0.25  # fraction of window that must be valid
    boost_max: float = 1.5         # multiplier at full consistency (isolated = 1.0)


class ConsistencyEstimator:
    """Image-space neighbourhood depth agreement → per-pixel boost multiplier.

    For each pixel, compares its depth to the local median across a square window.
    Pixels that agree with their neighbourhood (|depth - local_median| < depth_tolerance)
    AND whose neighbourhood has enough valid readings receive a boost > 1.0.
    Isolated mismatches stay at 1.0 (no boost, no penalty — evidence weight handles them).
    """

    def __init__(self, cfg: ConsistencyConfig | None = None) -> None:
        self.cfg = cfg or ConsistencyConfig()

    def compute(self, depth: np.ndarray, weight: np.ndarray) -> np.ndarray:
        """Return per-pixel boost multiplier H×W (float32, ≥ 1.0).

        Only pixels with weight > 0 can receive a boost; the rest return 1.0.
        """
        cfg = self.cfg
        w = 2 * cfg.window_radius + 1
        valid = np.isfinite(depth) & (weight > 0)

        depth_filled = np.where(valid, depth, 0.0).astype(np.float32)

        # Fraction of window that is valid
        valid_fraction = uniform_filter(valid.astype(np.float32), size=w)

        # Local median depth (approximate via median_filter on filled image)
        local_median = median_filter(depth_filled, size=w).astype(np.float32)

        # Depth agreement: pixel is consistent if it matches local median
        depth_diff = np.abs(depth_filled - local_median)
        is_consistent = (
            valid
            & (depth_diff < cfg.depth_tolerance)
            & (valid_fraction >= cfg.min_valid_fraction)
        )

        boost = np.where(is_consistent, cfg.boost_max, 1.0).astype(np.float32)
        boost[~valid] = 1.0
        return boost


# ── Unified voxel accumulator ─────────────────────────────────────────────────

@dataclass
class AccumulatorConfig:
    voxel_size: float = 0.08    # metres; coarser = faster, less precise
    capacity: int = 500_000     # pre-allocated voxel slots
    min_score: float = 1.2      # export threshold: voxels below this are hidden
    carve_penalty: float = 0.3  # score decrement per frame a ray passes through


class VoxelAccumulator:
    """Unified persistent world-frame voxel map with score-based evidence accumulation.

    Each voxel maintains:
        occupied_score    – cumulative evidence (weighted sum across frames)
        observation_count – number of frames that contributed any evidence
        last_seen         – monotonic timestamp of most recent observation
        mean_position     – running mean of observed 3D positions (sub-voxel precision)

    Replaces the previous _vox / _vox_bg split.  All observations go through
    the same path; walls earn their score via consistent medium-confidence returns
    rather than being assigned a separate accumulator with relaxed thresholds.
    """

    def __init__(self, cfg: AccumulatorConfig | None = None) -> None:
        self.cfg = cfg or AccumulatorConfig()
        cap = self.cfg.capacity
        self._idx: dict[int, int] = {}
        self._score = np.zeros(cap, dtype=np.float32)       # occupied_score
        self._obs   = np.zeros(cap, dtype=np.uint16)        # observation_count
        self._seen  = np.zeros(cap, dtype=np.float64)       # last_seen (monotonic)
        self._xyz   = np.empty((cap, 3), dtype=np.float32)  # mean_position
        self._n     = 0

    def add(self, xyz: np.ndarray, evidence: np.ndarray, ts: float) -> None:
        """Merge world-frame points with per-point evidence into the map.

        One entry per unique voxel per call — the point with the highest evidence
        in each voxel represents that frame's contribution for that voxel.
        """
        if len(xyz) == 0:
            return

        vk   = np.floor(xyz / self.cfg.voxel_size).astype(np.int32)
        keys = _pack(vk)

        # One representative point per voxel: highest evidence wins
        unique_keys, first_idx = np.unique(keys, return_index=True)
        xyz      = xyz[first_idx]
        evidence = evidence[first_idx]
        key_list = unique_keys.tolist()

        exists = np.array([k in self._idx for k in key_list], dtype=bool)

        if exists.any():
            rows = np.array(
                [self._idx[int(k)] for k in unique_keys[exists].tolist()],
                dtype=np.int32,
            )
            n = self._obs[rows].astype(np.float32)
            # Running mean position
            self._xyz[rows]   = (self._xyz[rows] * n[:, None] + xyz[exists]) / (n[:, None] + 1)
            self._score[rows] += evidence[exists]
            self._obs[rows]    = np.minimum(self._obs[rows] + 1, 65_535).astype(np.uint16)
            self._seen[rows]   = ts

        new_xyz  = xyz[~exists]
        new_keys = unique_keys[~exists]
        new_ev   = evidence[~exists]
        n_new    = len(new_xyz)
        if n_new == 0:
            return
        if self._n + n_new > self.cfg.capacity:
            self._grow()
        sl = slice(self._n, self._n + n_new)
        self._xyz[sl]   = new_xyz
        self._score[sl] = new_ev
        self._obs[sl]   = 1
        self._seen[sl]  = ts
        for i, k in enumerate(new_keys.tolist()):
            self._idx[int(k)] = self._n + i
        self._n += n_new

    def carve(self, xyz: np.ndarray, cam_pos: np.ndarray) -> None:
        """Decrement score for voxels along free-space rays toward each surface point.

        The segment (cam_pos → surface - ε) is known free space. Any voxel there
        is a stereo ghost; penalising it drives its score below min_score.
        Surface voxels observed this same frame are excluded from carving.
        """
        if self._n == 0 or len(xyz) == 0:
            return
        v = self.cfg.voxel_size

        vk = np.floor(xyz / v).astype(np.int32)
        raw_keys = _pack(vk)
        _, first = np.unique(raw_keys, return_index=True)
        xyz = xyz[first]
        surface_keys = set(raw_keys[first].tolist())

        rays  = xyz - cam_pos
        dists = np.linalg.norm(rays, axis=1)
        ok    = dists > v * 2
        if not ok.any():
            return
        dirs = rays[ok] / dists[ok, None]
        d    = dists[ok]

        max_t = int(d.max() / v)
        if max_t < 2:
            return
        t   = (np.arange(1, max_t) * v).astype(np.float32)
        pts = cam_pos + dirs[:, None, :] * t[None, :, None]
        valid = t[None, :] < (d[:, None] - v)
        pts = pts[valid]
        if len(pts) == 0:
            return

        free_keys = set(_pack(np.floor(pts / v).astype(np.int32)).tolist())
        free_keys -= surface_keys
        for k in free_keys:
            row = self._idx.get(int(k))
            if row is not None and self._score[row] > 0:
                self._score[row] = max(0.0, self._score[row] - self.cfg.carve_penalty)

    def stable_xyz(self) -> np.ndarray:
        """Points whose accumulated score meets the export threshold."""
        if self._n == 0:
            return np.zeros((0, 3), dtype=np.float32)
        mask = self._score[:self._n] >= self.cfg.min_score
        return self._xyz[:self._n][mask]

    @property
    def count(self) -> int:
        return self._n

    def _grow(self) -> None:
        cap2    = self.cfg.capacity * 2
        new_score = np.zeros(cap2, dtype=np.float32)
        new_obs   = np.zeros(cap2, dtype=np.uint16)
        new_seen  = np.zeros(cap2, dtype=np.float64)
        new_xyz   = np.empty((cap2, 3), dtype=np.float32)
        new_score[:self._n] = self._score[:self._n]
        new_obs[:self._n]   = self._obs[:self._n]
        new_seen[:self._n]  = self._seen[:self._n]
        new_xyz[:self._n]   = self._xyz[:self._n]
        self._score = new_score
        self._obs   = new_obs
        self._seen  = new_seen
        self._xyz   = new_xyz
        self.cfg = AccumulatorConfig(
            voxel_size=self.cfg.voxel_size,
            capacity=cap2,
            min_score=self.cfg.min_score,
            carve_penalty=self.cfg.carve_penalty,
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

    Only the hard floor on usability is applied here (conf < CONF_MIN → NaN).
    Further evidence weighting is handled by ConfidenceWeighter downstream.
    """

    CONF_MIN:  float = 25.0
    MIN_DEPTH: float = 0.3
    MAX_DEPTH: float = 8.0

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
    """DepthFramePacket + per-pixel evidence H×W → (xyz N×3, evidence N) in world frame.

    Three-step transform:
      1. Pinhole backproject (u,v,d) → camera-optical frame
      2. _R_OPT_TO_LINK            → camera_link frame (X=fwd, Y=left, Z=up)
      3. VIO pose_R / pose_t        → world frame  (identity before VIO locks)

    Only pixels with evidence > 0 are projected, so zero-weight pixels are
    dropped here rather than being filtered later.
    """

    def project(
        self, pkt: DepthFramePacket, evidence: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return (xyz (N,3) float32, evidence (N,) float32) for pixels with evidence > 0."""
        d = pkt.depth
        H, W = d.shape
        fx, fy, cx, cy = pkt.intrinsics

        uu, vv = np.meshgrid(
            np.arange(W, dtype=np.float32),
            np.arange(H, dtype=np.float32),
        )

        valid = np.isfinite(d) & (evidence > 0)
        dd    = d[valid]
        ev    = evidence[valid]

        x_opt = (uu[valid] - cx) * dd / fx
        y_opt = (vv[valid] - cy) * dd / fy
        z_opt = dd

        xyz_opt   = np.column_stack([x_opt, y_opt, z_opt]).astype(np.float32)
        xyz_link  = xyz_opt @ _R_OPT_TO_LINK.T
        xyz_world = xyz_link @ pkt.pose_R.T + pkt.pose_t

        return xyz_world, ev


# ── Stage 5: Streamer ─────────────────────────────────────────────────────────

class DepthStreamer:
    """Assembles packets, runs the evidence pipeline, builds the world map, logs to Rerun."""

    MAX_CLOUD = 50_000   # Rerun per-frame point cap
    MAX_MAP   = 100_000  # Rerun map point cap
    MAP_EVERY = 10       # frames between map updates

    def __init__(
        self,
        depth_filter:  DepthFilter,
        intrinsics:    IntrinsicsReader,
        pose:          PoseReader,
        backproj:      DepthBackprojector,
        conf_cfg:      ConfidenceConfig | None = None,
        consist_cfg:   ConsistencyConfig | None = None,
        accum_cfg:     AccumulatorConfig | None = None,
    ) -> None:
        self._filt    = depth_filter
        self._intr    = intrinsics
        self._pose    = pose
        self._bp      = backproj
        self._weighter   = ConfidenceWeighter(conf_cfg)
        self._consist    = ConsistencyEstimator(consist_cfg)
        self._vox        = VoxelAccumulator(accum_cfg)
        self._cam_z      = 0.0
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
        rr.set_time("frame", sequence=frame)
        if not self._pinhole_logged:
            rr.log("world/camera", rr.Pinhole(
                image_from_camera=pkt.K, width=pkt.width, height=pkt.height,
            ), static=True)
            self._pinhole_logged = True

        rr.log("world/camera/depth", rr.DepthImage(pkt.depth, meter=1.0))

        conf = pkt.confidence if pkt.confidence is not None else np.full(
            pkt.depth.shape, 100.0, dtype=np.float32
        )

        # Evidence pipeline: confidence → weight → consistency boost → final evidence
        weight    = self._weighter.compute(conf)           # H×W, 0 where unusable
        boost     = self._consist.compute(pkt.depth, weight)  # H×W, ≥ 1.0
        evidence  = (weight * boost).astype(np.float32)        # H×W, final per-pixel

        xyz, ev = self._bp.project(pkt, evidence)

        # ── Live cloud: colour by evidence weight (blue=low, red=high) ──────
        if len(xyz) > 0:
            n   = min(len(xyz), self.MAX_CLOUD)
            idx = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(n)
            t   = np.clip(ev[idx], 0.0, 1.5) / 1.5           # normalise to [0, 1]
            colors = np.column_stack([
                np.clip(2 * t - 1, 0, 1),       # red:   high evidence
                np.clip(1 - 2 * np.abs(t - 0.5), 0, 1),  # green: medium
                np.clip(1 - 2 * t, 0, 1),       # blue:  low evidence
            ])
            rr.log("world/cloud", rr.Points3D(
                positions=xyz[idx],
                colors=(colors * 255).astype(np.uint8),
                radii=0.003,
            ))

        # ── Persistent map ────────────────────────────────────────────────────
        self._cam_z = float(pkt.pose_t[2])
        if self._pose.locked and len(xyz) > 0:
            cam_pos = pkt.pose_t

            # Height band + floor-angle filter (camera-relative, VIO-drift-safe)
            h_rel    = xyz[:, 2] - self._cam_z
            rays     = xyz - cam_pos
            dist     = np.linalg.norm(rays, axis=1)
            d_z_norm = np.where(dist > 0, rays[:, 2] / dist, 0.0)
            keep = (
                (h_rel >= _Z_REL_LO) & (h_rel <= _Z_REL_HI)
                & (d_z_norm > _FLOOR_RAY_Z)
            )
            xyz_filt = xyz[keep]
            ev_filt  = ev[keep]

            if len(xyz_filt) > 0:
                self._vox.carve(xyz_filt, cam_pos)
                self._vox.add(xyz_filt, ev_filt, pkt.timestamp)

        if frame % self.MAP_EVERY == 0:
            self._log_map()

    def _log_map(self) -> None:
        pts = self._vox.stable_xyz()
        if len(pts) == 0:
            return
        n   = min(len(pts), self.MAX_MAP)
        idx = np.random.choice(len(pts), n, replace=False) if len(pts) > n else np.arange(n)
        rr.log("world/map", rr.Points3D(
            positions=pts[idx],
            colors=_height_color(pts[idx, 2] - self._cam_z),
            radii=0.005,
        ))

    def log_stdout(self, pkt: DepthFramePacket, frame: int, fps: float) -> None:
        lock   = "LOCKED" if self._pose.locked else "searching"
        stable = len(self._vox.stable_xyz())
        total  = self._vox.count
        print(
            f"frame={frame:5d}  "
            f"stable={stable:6d}  total_voxels={total:6d}  "
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
    rr.log("world/cloud", rr.Points3D([[0, 0, 0]]), static=True)

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

    zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 50)

    print("camera open — Ctrl-C to quit.")

    streamer = DepthStreamer(
        DepthFilter(),
        IntrinsicsReader(),
        PoseReader(),
        DepthBackprojector(),
        # Tune these configs independently:
        conf_cfg=ConfidenceConfig(conf_min=25.0, conf_full=85.0, curve=0.7),
        consist_cfg=ConsistencyConfig(window_radius=3, depth_tolerance=0.15, boost_max=1.5),
        accum_cfg=AccumulatorConfig(voxel_size=0.08, min_score=1.2, carve_penalty=0.3),
    )

    streamer._pose.enable(zed)

    rt = sl.RuntimeParameters()
    rt.texture_confidence_threshold = 80
    rt.remove_saturated_areas       = True

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
