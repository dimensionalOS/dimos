"""ZED Mini → high-fidelity rolling obstacle map.

Dual-layer architecture
  Fine Cloud    actual stereo-geometry positions (≈2 mm resolution, unchanged)
  Fusion Layer  2 cm voxel grid used ONLY for identity / dedup — never for centroids

On revisit the stored position is refined via online weighted average of actual
observations. Geometry converges toward the true surface; it is never snapped to
a voxel centre.

Pipeline stages (one class each)
  PointCloudFilter   raw ZED frame → clean (xyz, rgb, dist)
  PoseTransform      camera frame → world frame  (passthrough in SDK 5.4.0)
  DualLayerFusion    identity-indexed fine-geometry map
  RerunPublisher     Rerun visualisation (frame + depth + accumulated map)

Usage
  python -m dimos.navigation.camera_nav.zed_global_map
"""

from __future__ import annotations
import time
import numpy as np
import pyzed.sl as sl
import rerun as rr


# ── Shared helpers ────────────────────────────────────────────────────────────

_OFF  = np.int64(100_000)
_MASK = np.int64(0x3FFFF)   # 18 bits → handles ±100 000 voxels per axis (±2 km at 2 cm)


def _pack(vkeys: np.ndarray) -> np.ndarray:
    """(N, 3) int32 voxel indices → unique int64 keys."""
    v = (vkeys.astype(np.int64) + _OFF) & _MASK
    return (v[:, 0] << 36) | (v[:, 1] << 18) | v[:, 2]


def _jet(t: np.ndarray) -> np.ndarray:
    """Jet colourmap. t ∈ [0, 1]: 0 = near = red, 1 = far = blue."""
    r = np.clip(1.5 - np.abs(4 * t - 3), 0.0, 1.0)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0.0, 1.0)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0.0, 1.0)
    return (np.column_stack([r, g, b]) * 255).astype(np.uint8)


# ── Stage 1: Filter ───────────────────────────────────────────────────────────

class PointCloudFilter:
    """Raw ZED frame → clean (xyz, rgb, dist) arrays.

    Uses sl.VIEW.LEFT for high-fidelity camera RGB instead of the packed
    RGBA baked into XYZRGBA (which loses colour precision).
    A regular stride-2 pixel grid gives ~40–60 k valid points at VGA —
    dense enough for stereo-like geometry, safe for Rerun.
    """

    def __init__(self, min_depth: float = 0.3, max_depth: float = 8.0, stride: int = 2):
        self.min_d  = min_depth
        self.max_d  = max_depth
        self.stride = stride
        self._img   = sl.Mat()
        self._pc    = sl.Mat()

    def grab(self, zed: sl.Camera) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
        """Return (xyz, rgb, dist) for valid pixels, or None if frame is empty."""
        # Actual camera colour (BGRA → RGB)
        zed.retrieve_image(self._img, sl.VIEW.LEFT, sl.MEM.CPU)
        bgra    = self._img.get_data()
        cam_rgb = bgra[:, :, 2::-1].copy()          # H×W×3 uint8 RGB

        # Stereo XYZ per pixel
        zed.retrieve_measure(self._pc, sl.MEASURE.XYZRGBA, sl.MEM.CPU)
        xyz_hw = self._pc.get_data()[:, :, :3]      # H×W×3 float32

        # Stride-2 regular grid subsample
        s       = self.stride
        xyz_sub = xyz_hw[::s, ::s, :].reshape(-1, 3).astype(np.float32)
        rgb_sub = cam_rgb[::s, ::s, :].reshape(-1, 3)

        dist = np.linalg.norm(xyz_sub, axis=1)
        ok   = np.isfinite(xyz_sub[:, 0]) & (dist > self.min_d) & (dist < self.max_d)

        if not ok.any():
            return None
        return xyz_sub[ok], rgb_sub[ok], dist[ok]


# ── Stage 2: Pose transform ───────────────────────────────────────────────────

class PoseTransform:
    """Camera-frame XYZ → world-frame XYZ via ZED VIO.

    ZED SDK 5.4.0 on macOS: enable_positional_tracking() segfaults.
    This class is currently a pass-through.  Call enable(zed) once that
    is resolved; the rest of the pipeline is unaffected.
    """

    def __init__(self) -> None:
        self._R      = np.eye(3, dtype=np.float32)
        self._t      = np.zeros(3, dtype=np.float32)
        self._active = False
        self._pose   = sl.Pose()

    def enable(self, zed: sl.Camera) -> bool:
        tp = sl.PositionalTrackingParameters()
        tp.enable_imu_fusion = True
        ok = zed.enable_positional_tracking(tp) == sl.ERROR_CODE.SUCCESS
        self._active = ok
        return ok

    def update(self, zed: sl.Camera) -> None:
        if not self._active:
            return
        if zed.get_position(self._pose, sl.REFERENCE_FRAME.WORLD) == sl.POSITIONAL_TRACKING_STATE.OK:
            T        = self._pose.pose_data.get_data().astype(np.float32)
            self._R  = T[:3, :3]
            self._t  = T[:3, 3]

    def apply(self, xyz: np.ndarray) -> np.ndarray:
        if not self._active:
            return xyz
        return xyz @ self._R.T + self._t


# ── Stage 3: Dual-layer fusion ────────────────────────────────────────────────

class DualLayerFusion:
    """High-fidelity point map with voxel identity layer.

    Identity (Fusion) Layer
      A 2 cm voxel hash (Python dict: int64 key → row index) decides whether
      an incoming point is new or a revisit.  The voxel size is used ONLY for
      this membership test — it is never used to compute a stored position.

    Fine Cloud
      Actual observed xyz positions are stored, not voxel centroids.
      On revisit: online weighted average of real observations refines the
      stored position toward the true surface.  Confidence accumulates but the
      geometry is never collapsed.

    Storage: pre-allocated numpy arrays grown in doubling chunks so no per-point
    allocation happens in the hot path.
    """

    def __init__(self, voxel_size: float = 0.02, initial_capacity: int = 500_000) -> None:
        self._v     = voxel_size
        self._cap   = initial_capacity
        # Identity layer: packed key → row index  (O(1) lookup/insert)
        self._index: dict[int, int] = {}
        # Fine cloud storage
        self._xyz   = np.empty((self._cap, 3), dtype=np.float32)
        self._rgb   = np.empty((self._cap, 3), dtype=np.uint8)
        self._conf  = np.zeros(self._cap,      dtype=np.uint16)
        self._n     = 0

    # ── public ───────────────────────────────────────────────────────────────

    def fuse(self, xyz: np.ndarray, rgb: np.ndarray) -> None:
        """Merge one frame into the map."""
        # Voxel key for every incoming point
        vk   = np.floor(xyz / self._v).astype(np.int32)
        keys = _pack(vk)

        # One representative point per voxel within this frame
        _, first = np.unique(keys, return_index=True)
        keys = keys[first]
        xyz  = xyz[first]
        rgb  = rgb[first]

        # Partition into existing vs new
        key_list = keys.tolist()
        exists   = np.array([k in self._index for k in key_list], dtype=bool)

        if exists.any():
            self._update(keys[exists], xyz[exists], rgb[exists])
        if (~exists).any():
            self._insert(keys[~exists], xyz[~exists], rgb[~exists])

    @property
    def xyz(self) -> np.ndarray:
        return self._xyz[:self._n]

    @property
    def rgb(self) -> np.ndarray:
        return self._rgb[:self._n]

    @property
    def count(self) -> int:
        return self._n

    # ── private ───────────────────────────────────────────────────────────────

    def _update(self, keys: np.ndarray, xyz: np.ndarray, rgb: np.ndarray) -> None:
        """Running-average update of actual geometry — no centroid collapse."""
        rows  = np.array([self._index[int(k)] for k in keys.tolist()], dtype=np.int32)
        old_c = self._conf[rows].astype(np.float32)
        new_c = np.minimum(old_c + 1.0, 65_535.0)
        w     = (old_c / new_c)[:, None]   # weight of the stored observation

        self._xyz[rows]  = self._xyz[rows] * w + xyz * (1.0 - w)
        self._rgb[rows]  = (self._rgb[rows].astype(np.float32) * w +
                            rgb.astype(np.float32) * (1.0 - w)).astype(np.uint8)
        self._conf[rows] = new_c.astype(np.uint16)

    def _insert(self, keys: np.ndarray, xyz: np.ndarray, rgb: np.ndarray) -> None:
        """Insert new voxels — store actual stereo position, not voxel centre."""
        n_new = len(keys)
        if self._n + n_new > self._cap:
            self._grow()
        rows = slice(self._n, self._n + n_new)
        self._xyz[rows]  = xyz
        self._rgb[rows]  = rgb
        self._conf[rows] = 1
        for i, k in enumerate(keys.tolist()):
            self._index[int(k)] = self._n + i
        self._n += n_new

    def _grow(self) -> None:
        new_cap          = self._cap * 2
        new_xyz          = np.empty((new_cap, 3), dtype=np.float32)
        new_rgb          = np.empty((new_cap, 3), dtype=np.uint8)
        new_conf         = np.zeros(new_cap,      dtype=np.uint16)
        new_xyz[:self._n]  = self._xyz[:self._n]
        new_rgb[:self._n]  = self._rgb[:self._n]
        new_conf[:self._n] = self._conf[:self._n]
        self._xyz  = new_xyz
        self._rgb  = new_rgb
        self._conf = new_conf
        self._cap  = new_cap


# ── Stage 4: Rerun publisher ──────────────────────────────────────────────────

class RerunPublisher:
    """Formats and logs point clouds to Rerun.

    world/frame   dense current frame, real RGB          (every frame)
    world/depth   same geometry, jet depth heatmap       (every frame)
    world/map     accumulated fine-geometry map, real RGB (every map_hz frames)
    """

    MAX_FRAME = 50_000   # per-frame cap (well under Rerun h2 limit)
    MAX_MAP   = 80_000   # visualisation cap for accumulated map

    def __init__(self, min_depth: float, max_depth: float, map_interval: int = 10) -> None:
        self._min_d       = min_depth
        self._max_d       = max_depth
        self._map_interval = map_interval

    def log_frame(self, xyz: np.ndarray, rgb: np.ndarray, dist: np.ndarray) -> None:
        n   = min(len(xyz), self.MAX_FRAME)
        idx = (np.random.choice(len(xyz), n, replace=False)
               if len(xyz) > n else np.arange(len(xyz)))

        rr.log("world/frame", rr.Points3D(
            positions=xyz[idx], colors=rgb[idx], radii=0.004,
        ))
        t   = np.clip((dist[idx] - self._min_d) / (self._max_d - self._min_d), 0.0, 1.0)
        rr.log("world/depth", rr.Points3D(
            positions=xyz[idx], colors=_jet(t), radii=0.004,
        ))

    def log_map(self, fusion: DualLayerFusion, frame: int) -> None:
        if frame % self._map_interval != 0 or fusion.count == 0:
            return
        xyz = fusion.xyz
        rgb = fusion.rgb
        if len(xyz) > self.MAX_MAP:
            idx = np.random.choice(len(xyz), self.MAX_MAP, replace=False)
            xyz = xyz[idx]; rgb = rgb[idx]
        rr.log("world/map", rr.Points3D(positions=xyz, colors=rgb, radii=0.004))


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
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

    rr.init("zed_hifi_map", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    print("camera open")

    filt   = PointCloudFilter(min_depth=0.3, max_depth=8.0, stride=2)
    pose   = PoseTransform()
    fusion = DualLayerFusion(voxel_size=0.02)
    pub    = RerunPublisher(min_depth=0.3, max_depth=8.0, map_interval=10)

    # Activate when SDK 5.4.0 tracking is fixed:
    # pose.enable(zed)

    rt    = sl.RuntimeParameters()
    frame = 0
    t0    = time.monotonic()

    try:
        while True:
            if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
                continue

            result = filt.grab(zed)
            if result is None:
                frame += 1
                continue

            xyz, rgb, dist = result

            pose.update(zed)
            xyz_world = pose.apply(xyz)

            fusion.fuse(xyz_world, rgb)
            pub.log_frame(xyz_world, rgb, dist)
            pub.log_map(fusion, frame)

            fps = frame / max(time.monotonic() - t0, 1e-6)
            print(f"frame={frame}  pts={len(xyz)}  map={fusion.count}  fps={fps:.1f}")
            frame += 1

    except KeyboardInterrupt:
        pass

    zed.close()
    print(f"done — {fusion.count} points in map")


if __name__ == "__main__":
    main()
