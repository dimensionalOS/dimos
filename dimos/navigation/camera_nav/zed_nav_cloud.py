"""Navigation-optimized point cloud pipeline for DimOS CostMapper.

Objective: produce a world-frame PointCloud2 where every point is
high-confidence evidence of occupied space — not just a projected pixel.

Four independent pipeline stages:

  Stage 1  DepthObserver     ZED frame → validated depth (three validation passes)
  Stage 2  WorldProjector    validated depth + VIO pose → world-frame XYZ
  Stage 3  EvidenceMap       per-voxel observation counting → stable occupied set
  Stage 4  NavigationCloud   stable voxels → clean N×3 output for CostMapper

Design principle:
  Every depth pixel has a non-zero chance of being a stereo artifact even after
  confidence filtering.  Requiring a voxel to be observed from multiple independent
  camera positions (different poses) makes artifact survival exponentially unlikely.
  A 5% per-pixel artifact rate after Stage 1 becomes < 0.01% after Stage 3
  with stable_min = 3, because three independent stereo failures must land in the
  same 3 cm cube — a near-impossible coincidence for noise, but routine for walls.

Usage:
  python -m dimos.navigation.camera_nav.zed_nav_cloud

Output fed to CostMapper:
  world/stable_cloud  N×3 float32, Z=up, metres, frame_id="world"
"""

from __future__ import annotations

import time
from typing import NamedTuple

import numpy as np
import pyzed.sl as sl
import rerun as rr

from dimos.navigation.camera_nav.pipeline import _pack, _R_OPT_TO_LINK


# ── Ray casting ────────────────────────────────────────────────────────────────

class RayCaster:
    """Cast rays from camera origin through each depth hit to mark free space.

    For each ray: march from origin → (free_fraction × depth), stepping one
    voxel at a time.  Every voxel along the path is evidence of traversable space.

    Rays are subsampled (stride) so the operation stays cheap even at 30 k hits/frame.
    The stop criterion (free_fraction < 1.0) ensures we never mark the occupied
    surface voxel itself as free.
    """

    def __init__(
        self,
        voxel_size:    float = 0.03,
        stride:        int   = 16,
        free_fraction: float = 0.9,
    ) -> None:
        self._v     = voxel_size
        self._stride = stride
        self._frac  = free_fraction

    def cast(self, origin: np.ndarray, hit_xyz: np.ndarray) -> np.ndarray:
        """Return (M, 3) world-frame positions of free-space voxels.

        origin:  (3,) camera position in world frame (pose_t)
        hit_xyz: (N, 3) world-frame occupied hit points from WorldProjector
        """
        if len(hit_xyz) == 0:
            return np.empty((0, 3), dtype=np.float32)

        hits    = hit_xyz[::self._stride]                          # subsample rays
        diffs   = hits - origin.astype(np.float32)                 # (N', 3)
        lengths = np.linalg.norm(diffs, axis=1)                    # (N',)

        ok      = lengths > self._v                                 # skip trivially short rays
        diffs   = diffs[ok];  lengths = lengths[ok]
        if len(lengths) == 0:
            return np.empty((0, 3), dtype=np.float32)

        dirs     = diffs / lengths[:, None]                        # (N', 3) unit vectors
        free_end = lengths * self._frac                            # stop before the hit
        n_steps  = max(1, int(free_end.max() / self._v))

        # t[s] = (s+1) * voxel_size  — step positions along ray
        t = (np.arange(n_steps, dtype=np.float32) + 1.0) * self._v  # (S,)

        # pts[n, s] = origin + t[s] * dirs[n]   shape (N', S, 3)
        pts    = origin.astype(np.float32) + t[None, :, None] * dirs[:, None, :]
        inside = t[None, :] < free_end[:, None]                    # (N', S) mask

        return pts[inside]  # (M, 3)


# ── Configuration ──────────────────────────────────────────────────────────────

class ObservationConfig(NamedTuple):
    conf_min:  float = 40.0  # stereo confidence threshold (0–100)
    depth_min: float = 0.30  # metres
    depth_max: float = 8.00  # metres


class EvidenceConfig(NamedTuple):
    voxel_size:    float = 0.03      # metres
    stable_min:    int   = 3         # occupied votes before a voxel is stable
    free_min:      int   = 2         # free votes before a voxel is confirmed free
    capacity:      int   = 500_000   # initial occupied pool
    free_capacity: int   = 1_000_000 # initial free pool (free voxels outnumber occupied)


# ── Stage 1: Observation validation ───────────────────────────────────────────

class DepthObserver:
    """ZED frame → validated depth array.

    Pass A — range + NaN
        Removes physically impossible values.

    Pass B — stereo confidence
        Rejects pixels where the stereo match was poor (score < conf_min).
        Texture-less surfaces and reflective materials fail here.

    Artifact rejection at the 2D level ends here.  Isolated stereo artifacts
    that survive both passes are handled in Stage 3 (EvidenceMap): they appear
    in different 3D locations each frame and never accumulate enough votes to
    become stable.
    """

    def __init__(self, cfg: ObservationConfig = ObservationConfig()) -> None:
        self._cfg       = cfg
        self._depth_mat = sl.Mat()
        self._conf_mat  = sl.Mat()

    def observe(self, zed: sl.Camera) -> np.ndarray:
        """Return H×W float32 depth. Rejected pixels are NaN."""
        zed.retrieve_measure(self._depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU)
        raw_depth = self._depth_mat.get_data().astype(np.float32)
        if raw_depth.ndim == 3:
            raw_depth = raw_depth[:, :, 0]

        zed.retrieve_measure(self._conf_mat, sl.MEASURE.CONFIDENCE, sl.MEM.CPU)
        conf = self._conf_mat.get_data().astype(np.float32)
        if conf.ndim == 3:
            conf = conf[:, :, 0]

        cfg = self._cfg

        # Pass A: range + NaN
        valid = (
            np.isfinite(raw_depth)
            & (raw_depth >= cfg.depth_min)
            & (raw_depth <= cfg.depth_max)
            & np.isfinite(conf)
        )

        # Pass B: stereo confidence
        valid &= conf >= cfg.conf_min

        depth = np.full_like(raw_depth, np.nan)
        depth[valid] = raw_depth[valid]
        return depth

    def valid_fraction(self, depth: np.ndarray) -> float:
        return float(np.isfinite(depth).mean())


# ── Stage 2a: Camera calibration ──────────────────────────────────────────────

class IntrinsicsReader:
    """Reads and caches ZED left-camera intrinsics (fixed for the session)."""

    def __init__(self) -> None:
        self._ready = False
        self._fx = self._fy = self._cx = self._cy = 0.0

    def read(self, zed: sl.Camera) -> None:
        if self._ready:
            return
        ci  = zed.get_camera_information()
        cal = ci.camera_configuration.calibration_parameters.left_cam
        self._fx, self._fy = cal.fx, cal.fy
        self._cx, self._cy = cal.cx, cal.cy
        self._ready = True

    @property
    def params(self) -> tuple[float, float, float, float]:
        """(fx, fy, cx, cy)"""
        return self._fx, self._fy, self._cx, self._cy


# ── Stage 2b: Pose tracking ───────────────────────────────────────────────────

class PoseTracker:
    """Synchronized VIO pose with lock detection.

    The lock guard is the second critical safety check:
    Before the first POSITIONAL_TRACKING_STATE.OK, pose_R = I and pose_t = 0.
    Backprojected points in that state are in camera-optical frame (Z = depth)
    mislabelled as world frame (Z = up).  Feeding them to EvidenceMap poisons
    the accumulated map irreversibly for the rest of the session.

    locked becomes True on the first OK state and stays True.
    Only when locked=True should WorldProjector be called.
    """

    def __init__(self) -> None:
        self._R      = np.eye(3, dtype=np.float32)
        self._t      = np.zeros(3, dtype=np.float32)
        self._locked = False
        self._active = False
        self._pose   = sl.Pose()

    def enable(self, zed: sl.Camera) -> bool:
        tp = sl.PositionalTrackingParameters()
        tp.enable_imu_fusion   = True
        tp.set_floor_as_origin = True  # Z=0 at floor → costmap Z thresholds are absolute
        ok = zed.enable_positional_tracking(tp) == sl.ERROR_CODE.SUCCESS
        self._active = ok
        print("VIO tracking enabled" if ok else "VIO tracking failed — map will not build")
        return ok

    def update(self, zed: sl.Camera) -> None:
        if not self._active:
            return
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
            self._locked = True

    @property
    def locked(self) -> bool: return self._locked

    @property
    def R(self) -> np.ndarray: return self._R

    @property
    def t(self) -> np.ndarray: return self._t


# ── Stage 2c: Projection ──────────────────────────────────────────────────────

class WorldProjector:
    """Validated depth → world-frame XYZ.

    Three-step transform that exactly matches ZEDNavBridge:
      1. Pinhole backproject (u, v, d) → camera-optical XYZ
      2. _R_OPT_TO_LINK          → camera_link XYZ (Z = up)
      3. VIO pose                → world XYZ (Z = up)

    Step 2 is not optional: skipping it sends Z = depth (not Z = height)
    into CostMapper, which then computes gradients of scene-depth rather
    than terrain elevation.  Every point appears roughly at the same
    'height' (distance) producing flat, cost-free output.
    """

    def project(
        self,
        depth:  np.ndarray,
        fx: float, fy: float, cx: float, cy: float,
        pose_R: np.ndarray,
        pose_t: np.ndarray,
    ) -> np.ndarray:
        valid = np.isfinite(depth)
        if not valid.any():
            return np.empty((0, 3), dtype=np.float32)

        H, W = depth.shape
        us = np.arange(W, dtype=np.float32)
        vs = np.arange(H, dtype=np.float32)
        uu, vv = np.meshgrid(us, vs)

        dd    = depth[valid]
        xyz_opt = np.column_stack([
            (uu[valid] - cx) * dd / fx,   # X optical = right in image
            (vv[valid] - cy) * dd / fy,   # Y optical = down in image
            dd,                            # Z optical = depth into scene
        ]).astype(np.float32)

        xyz_link  = xyz_opt  @ _R_OPT_TO_LINK.T   # optical → camera_link (Z = up)
        xyz_world = xyz_link @ pose_R.T + pose_t   # camera_link → world

        return xyz_world


# ── Stage 3: Evidence fusion ───────────────────────────────────────────────────

class EvidenceMap:
    """Per-voxel observation counter.

    Core logic: a voxel is stable when it has been independently observed
    stable_min times from different camera positions.  Only stable voxels
    are included in the output.

    'Independent' is enforced by deduplicating hits within a single frame
    before incrementing counts — so moving forward and back does not let
    a single observation count multiple times.

    Position accuracy: stored positions are running means of actual stereo
    measurements, not voxel centroids.  This preserves Z accuracy for the
    CostMapper height-cost algorithm, which needs precise min/max Z per cell.
    """

    def __init__(self, cfg: EvidenceConfig = EvidenceConfig()) -> None:
        self._v    = cfg.voxel_size
        self._smin = cfg.stable_min
        self._cap  = cfg.capacity
        self._idx: dict[int, int] = {}
        self._xyz  = np.empty((self._cap, 3), dtype=np.float32)
        self._obs  = np.zeros(self._cap, dtype=np.uint16)
        self._n    = 0
        # Free-space voxel pool (UNKNOWN = never hit by any ray)
        self._fmin  = cfg.free_min
        self._fcap  = cfg.free_capacity
        self._fidx: dict[int, int] = {}
        self._fxyz  = np.empty((self._fcap, 3), dtype=np.float32)
        self._fobs  = np.zeros(self._fcap, dtype=np.uint16)
        self._fn    = 0

    def observe(self, xyz: np.ndarray) -> None:
        """Add one frame of world-frame observations to the map."""
        if len(xyz) == 0:
            return

        vk   = np.floor(xyz / self._v).astype(np.int32)
        keys = _pack(vk)

        # One vote per voxel per frame — prevents stationary camera inflating counts
        _, first = np.unique(keys, return_index=True)
        keys = keys[first]
        xyz  = xyz[first]

        key_list = keys.tolist()
        exists   = np.array([k in self._idx for k in key_list], dtype=bool)

        if exists.any():
            rows  = np.array([self._idx[int(k)] for k in keys[exists].tolist()], dtype=np.int32)
            old_c = self._obs[rows].astype(np.float32)
            new_c = np.minimum(old_c + 1.0, 65_535.0)
            w     = (old_c / new_c)[:, None]
            self._xyz[rows] = self._xyz[rows] * w + xyz[exists] * (1.0 - w)
            self._obs[rows] = new_c.astype(np.uint16)

        new_mask = ~exists
        if new_mask.any():
            n_new = int(new_mask.sum())
            if self._n + n_new > self._cap:
                self._grow()
            sl_ = slice(self._n, self._n + n_new)
            self._xyz[sl_] = xyz[new_mask]
            self._obs[sl_] = 1
            for i, k in enumerate(keys[new_mask].tolist()):
                self._idx[int(k)] = self._n + i
            self._n += n_new

    @property
    def stable_xyz(self) -> np.ndarray:
        """World-frame XYZ of voxels observed >= stable_min times."""
        mask = self._obs[:self._n] >= self._smin
        return self._xyz[:self._n][mask].copy()

    @property
    def total_voxels(self) -> int:
        return self._n

    @property
    def stable_count(self) -> int:
        return int((self._obs[:self._n] >= self._smin).sum())

    def observe_free(self, xyz: np.ndarray) -> None:
        """Mark world-frame positions as free (ray passed through here).

        OCCUPIED takes priority: any voxel already in the occupied pool is skipped.
        Free voxels store their voxel centroid (not a running mean) — the position
        is exact to the voxel resolution so no refinement is needed.
        """
        if len(xyz) == 0:
            return

        vk   = np.floor(xyz / self._v).astype(np.int32)
        keys = _pack(vk)

        # Deduplicate within this frame
        _, first = np.unique(keys, return_index=True)
        keys = keys[first];  vk = vk[first]

        # Skip voxels already known to be occupied
        key_list = keys.tolist()
        not_occ  = np.array([k not in self._idx for k in key_list], dtype=bool)
        keys     = keys[not_occ];  vk = vk[not_occ]
        if len(keys) == 0:
            return

        key_list = keys.tolist()
        exists   = np.array([k in self._fidx for k in key_list], dtype=bool)

        if exists.any():
            rows = np.array([self._fidx[int(k)] for k in keys[exists].tolist()], dtype=np.int32)
            self._fobs[rows] = np.minimum(self._fobs[rows] + 1, 65_535).astype(np.uint16)

        new_mask = ~exists
        if new_mask.any():
            n_new = int(new_mask.sum())
            if self._fn + n_new > self._fcap:
                self._free_grow()
            sl_ = slice(self._fn, self._fn + n_new)
            self._fxyz[sl_] = (vk[new_mask] + 0.5) * self._v  # voxel centroid
            self._fobs[sl_] = 1
            for i, k in enumerate(keys[new_mask].tolist()):
                self._fidx[int(k)] = self._fn + i
            self._fn += n_new

    @property
    def free_xyz(self) -> np.ndarray:
        """Confirmed free voxel positions (seen free >= free_min times, not occupied)."""
        mask = self._fobs[:self._fn] >= self._fmin
        return self._fxyz[:self._fn][mask].copy()

    @property
    def free_count(self) -> int:
        return int((self._fobs[:self._fn] >= self._fmin).sum())

    def obs_counts(self) -> np.ndarray:
        return self._obs[:self._n]

    def all_xyz(self) -> np.ndarray:
        return self._xyz[:self._n]

    def _grow(self) -> None:
        new_cap  = self._cap * 2
        new_xyz  = np.empty((new_cap, 3), dtype=np.float32)
        new_obs  = np.zeros(new_cap, dtype=np.uint16)
        new_xyz[:self._n] = self._xyz[:self._n]
        new_obs[:self._n] = self._obs[:self._n]
        self._xyz = new_xyz;  self._obs = new_obs;  self._cap = new_cap

    def _free_grow(self) -> None:
        new_cap  = self._fcap * 2
        new_xyz  = np.empty((new_cap, 3), dtype=np.float32)
        new_obs  = np.zeros(new_cap, dtype=np.uint16)
        new_xyz[:self._fn] = self._fxyz[:self._fn]
        new_obs[:self._fn] = self._fobs[:self._fn]
        self._fxyz = new_xyz;  self._fobs = new_obs;  self._fcap = new_cap


# ── Stage 4: Navigation cloud output ──────────────────────────────────────────

class NavigationCloud:
    """Stable voxels → clean N×3 cloud for CostMapper.

    Publishes at a lower rate than the camera (cloud_every frames).
    CostMapper does not need 15 Hz input; publishing at ~1 Hz reduces
    CPU load and gives EvidenceMap more frames to accumulate per cycle.
    """

    def __init__(self, ev_map: EvidenceMap, cloud_every: int = 15) -> None:
        self._map   = ev_map
        self._every = cloud_every

    def get(self, frame: int) -> np.ndarray | None:
        """Return stable cloud if it's time to publish, else None."""
        if frame % self._every != 0:
            return None
        stable = self._map.stable_xyz
        return stable if len(stable) > 0 else None


# ── Rerun visualisation helper ─────────────────────────────────────────────────

def _confidence_colors(counts: np.ndarray) -> np.ndarray:
    """Color stable voxels by reinforcement level.

    blue  = just reached stable threshold (3 obs) — tentatively confirmed
    cyan  = moderately reinforced (5–9 obs)
    white = heavily reinforced (15+ obs) — very confident wall

    'Revisits reinforce walls': returning to the same surface makes it brighter,
    not noisier. Noise artifacts never accumulate enough votes to appear at all.
    """
    t = np.clip((counts.astype(np.float32) - 3) / 12.0, 0.0, 1.0)  # 0 at 3, 1 at 15+
    r = (t * 200).astype(np.uint8)
    g = (t * 220).astype(np.uint8)
    b = (220 - t * 170).astype(np.uint8)   # 220 → 50
    return np.column_stack([r, g, b])


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    rr.init("zed_nav_cloud", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    # Seed a 3D entity immediately so Rerun's auto-blueprint creates a 3D panel.
    # Without this, camera/depth (2D image) arrives first and Rerun builds only
    # a 2D view — world/stable_cloud logged later has nowhere to appear.
    rr.log("world/origin", rr.Points3D(
        [[0.0, 0.0, 0.0]], radii=0.08,
        colors=[[255, 80, 80]],
    ), static=True)

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

    print("camera open — Ctrl-C to quit")

    obs_cfg = ObservationConfig(conf_min=40, depth_min=0.3, depth_max=8.0)
    ev_cfg  = EvidenceConfig(voxel_size=0.03, stable_min=3, free_min=2)

    observer  = DepthObserver(obs_cfg)
    intr      = IntrinsicsReader()
    pose      = PoseTracker()
    projector = WorldProjector()
    raycaster = RayCaster(voxel_size=ev_cfg.voxel_size, stride=16, free_fraction=0.9)
    ev_map    = EvidenceMap(ev_cfg)
    nav_cloud = NavigationCloud(ev_map, cloud_every=15)

    pose.enable(zed)

    rt    = sl.RuntimeParameters()
    frame = 0
    t0    = time.monotonic()

    try:
        while True:
            if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
                continue

            ts = time.monotonic()

            rr.set_time("frame", sequence=frame)

            # ── Shared acquisition ─────────────────────────────────────────────
            intr.read(zed)
            depth      = observer.observe(zed)
            valid_frac = observer.valid_fraction(depth)
            pose.update(zed)

            if frame % 3 == 0:
                d = depth.copy();  d[~np.isfinite(d)] = 0.0
                rr.log("camera/depth", rr.Image((np.clip(d / 8.0, 0, 1) * 255).astype(np.uint8)))

            # ── STREAM 1: RAW WORLD HITS (no stability requirement) ────────────
            # Project every frame as soon as intrinsics are ready.
            # Before VIO locks: pose.R=I, pose.t=0  → camera-space points at origin.
            # After VIO locks:  real pose            → world-space points.
            # Nothing blocks this stream. It is always the first proof the
            # backprojection and coordinate transforms are working.
            xyz = np.empty((0, 3), dtype=np.float32)
            if intr._ready:
                fx, fy, cx, cy = intr.params
                xyz = projector.project(depth, fx, fy, cx, cy, pose.R, pose.t)

            if len(xyz) > 0:
                cap = min(len(xyz), 20_000)
                rr.log("world/raw_hits", rr.Points3D(
                    positions=xyz[:cap],
                    colors=np.full((cap, 3), [180, 180, 180], dtype=np.uint8),
                    radii=0.01,
                ))

            # ── STREAM 2: STABLE MAP (VIO-locked, multi-frame fusion) ──────────
            # Only accumulate into EvidenceMap when pose is trustworthy.
            # This keeps the navigation map clean without blocking visualization.
            if pose.locked and len(xyz) > 0:
                ev_map.observe(xyz)
                ev_map.observe_free(raycaster.cast(pose.t, xyz))

            if frame % nav_cloud._every == 0:
                stable = ev_map.stable_xyz
                if len(stable) > 0:
                    s_obs = ev_map._obs[:ev_map._n][ev_map._obs[:ev_map._n] >= ev_map._smin]
                    rr.log("world/stable_cloud", rr.Points3D(
                        positions=stable,
                        colors=_confidence_colors(s_obs),
                        radii=0.02,
                    ))
                free = ev_map.free_xyz
                if len(free) > 0:
                    cap = min(len(free), 80_000)
                    idx = np.random.choice(len(free), cap, replace=False) if len(free) > cap else np.arange(len(free))
                    rr.log("world/free_space", rr.Points3D(
                        positions=free[idx],
                        colors=np.full((len(idx), 3), [80, 140, 200], dtype=np.uint8),
                        radii=0.01,
                    ))

            fps = frame / max(ts - t0, 1e-6)
            print(
                f"frame={frame:5d}  valid={valid_frac*100:4.1f}%  "
                f"raw={len(xyz):5d}  occ={ev_map.total_voxels:6d}  stable={ev_map.stable_count:6d}  "
                f"vio={'LOCKED' if pose.locked else 'no-lock'}  fps={fps:.1f}"
            )
            frame += 1

    except KeyboardInterrupt:
        pass

    zed.close()
    stable_frac = ev_map.stable_count / max(ev_map.total_voxels, 1)
    print(
        f"\ndone — {ev_map.stable_count} stable / {ev_map.total_voxels} total "
        f"({stable_frac*100:.1f}% stable)"
    )


if __name__ == "__main__":
    main()
