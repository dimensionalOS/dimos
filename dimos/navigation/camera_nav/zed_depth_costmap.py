"""ZED Mini → live cloud + persistent world map.

world/cloud  nearest-per-ray denoised live cloud
world/map    persistent obstacle map — accumulates across frames, one voxel per occupied cell

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
from scipy.ndimage import sobel


VOX_SIZE     = 0.020   # 2.0 cm final voxels
_Z_REL_HI   =  0.5    # camera-relative ceiling (0.5 m above camera)
_FLOOR_Z    =  0.03   # absolute world-Z floor cutoff (3 cm above gravity-aligned floor)
_GRAD_THRESH =  0.30  # Sobel gradient magnitude threshold — pixels above this are edge artifacts
MAP_EVERY   =  5      # log world/map every N frames (reduces Rerun overhead)

# ── Voxel key packing ────────────────────────────────────────────────────────
_VOFF  = np.int64(100_000)
_VMASK = np.int64(0x3FFFF)

def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + _VOFF) & _VMASK
    return (v[:, 0] << 36) | (v[:, 1] << 18) | v[:, 2]


def _filter_isolated(xyz: np.ndarray, voxel: float = 0.05, min_pts: int = 2) -> np.ndarray:
    if len(xyz) < min_pts:
        return np.zeros((0, 3), dtype=np.float32)
    vk = np.floor(xyz / voxel).astype(np.int32)
    _, inv, cnt = np.unique(_pack(vk), return_inverse=True, return_counts=True)
    return xyz[cnt[inv] >= min_pts]


# ── Persistent world map ─────────────────────────────────────────────────────
class FastVoxelMap:
    """Append-only world-frame obstacle map backed by numpy arrays.

    Each call to add() voxelizes the incoming points and appends only those
    whose voxel key is new.  A periodic compact() pass removes duplicates that
    sneak in between compactions, keeping memory bounded.
    """

    COMPACT_EVERY = 30

    def __init__(self, voxel_size: float = VOX_SIZE, capacity: int = 500_000) -> None:
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
        self._pts[self._n : self._n + len(new_pts)] = new_pts
        self._n      += len(new_pts)
        self._n_adds += 1
        if self._n_adds % self.COMPACT_EVERY == 0:
            self._compact()

    def _compact(self) -> None:
        if self._n == 0:
            return
        vk = np.floor(self._pts[: self._n] / self._v).astype(np.int32)
        _, idx = np.unique(_pack(vk), return_index=True)
        n_u = len(idx)
        self._pts[:n_u] = self._pts[: self._n][idx]
        self._n = n_u

    def _grow(self) -> None:
        cap2 = self._cap * 2
        buf  = np.empty((cap2, 3), dtype=np.float32)
        buf[: self._n] = self._pts[: self._n]
        self._pts = buf
        self._cap = cap2

    def points(self) -> np.ndarray:
        return self._pts[: self._n]

    @property
    def count(self) -> int:
        return self._n


# ── Nearest-per-ray filter (live cloud denoising only) ───────────────────────
def _nearest_per_ray_idx(xyz: np.ndarray, cam_pos: np.ndarray, deg: float = 3.0) -> np.ndarray:
    if len(xyz) == 0:
        return np.empty(0, dtype=np.intp)
    rays  = xyz - cam_pos
    dists = np.linalg.norm(rays, axis=1)
    ok    = dists > 0
    if not ok.any():
        return np.empty(0, dtype=np.intp)
    idx_ok = np.where(ok)[0]
    dirs   = rays[ok] / dists[ok, None]
    rad    = np.radians(deg)
    az     = np.floor(np.arctan2(dirs[:, 1], dirs[:, 0]) / rad).astype(np.int32)
    el     = np.floor(np.arcsin(np.clip(dirs[:, 2], -1.0, 1.0)) / rad).astype(np.int32)
    keys   = az.astype(np.int64) * 100_000 + el.astype(np.int64)
    order  = np.argsort(dists[ok])
    _, first = np.unique(keys[order], return_index=True)
    return idx_ok[order[first]]


# ── Height colormap ──────────────────────────────────────────────────────────
def _height_color(z_rel: np.ndarray) -> np.ndarray:
    t = np.clip((z_rel + 1.4) / 1.9, 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0.0, 1.0)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0.0, 1.0)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0.0, 1.0)
    return (np.column_stack([r, g, b]) * 255).astype(np.uint8)


# ── ZED data packet ──────────────────────────────────────────────────────────
@dataclass
class DepthFramePacket:
    timestamp:  float
    depth:      np.ndarray          # H×W float32, NaN=invalid
    intrinsics: np.ndarray          # [fx, fy, cx, cy]
    width:      int
    height:     int
    pose_R:     np.ndarray          # 3×3 float32 camera→world
    pose_t:     np.ndarray          # (3,) float32 camera→world
    xyz_cam_hw: np.ndarray | None = field(default=None)   # H×W×3 float32
    colors_hw:  np.ndarray | None = field(default=None)   # H×W×3 uint8 RGB

    @property
    def K(self) -> np.ndarray:
        fx, fy, cx, cy = self.intrinsics
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


# ── ZED source ───────────────────────────────────────────────────────────────
class ZEDDepthSource:
    MIN_DEPTH: float = 0.3
    MAX_DEPTH: float = 8.0

    def __init__(self, zed: sl.Camera) -> None:
        self._zed     = zed
        self._xyz_mat = sl.Mat()
        self._img_mat = sl.Mat()
        self._intr    = _IntrinsicsReader()
        self._pose    = _PoseReader()

    def enable_tracking(self) -> bool:
        return self._pose.enable(self._zed)

    @property
    def pose_locked(self) -> bool:
        return self._pose.locked

    def read(self, ts: float) -> DepthFramePacket:
        self._intr.read(self._zed)
        self._pose.update(self._zed)

        self._zed.retrieve_measure(self._xyz_mat, sl.MEASURE.XYZ, sl.MEM.CPU)
        xyz_raw = self._xyz_mat.get_data()[:, :, :3].astype(np.float32)

        depth = xyz_raw[:, :, 0].copy()
        invalid = ~np.isfinite(depth) | (depth <= self.MIN_DEPTH) | (depth > self.MAX_DEPTH)
        depth[invalid]   = np.nan
        xyz_raw[invalid] = np.nan

        self._zed.retrieve_image(self._img_mat, sl.VIEW.LEFT, sl.MEM.CPU)
        colors_hw = self._img_mat.get_data()[:, :, 2::-1].copy()  # BGRA→RGB

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


class _IntrinsicsReader:
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


class _PoseReader:
    def __init__(self) -> None:
        self._R      = np.eye(3, dtype=np.float32)
        self._t      = np.zeros(3, dtype=np.float32)
        self._active = False
        self._locked = False
        self._pose   = sl.Pose()
        self._t0     = time.monotonic()
        self._last_warn = 0.0

    def enable(self, zed: sl.Camera) -> bool:
        tp = sl.PositionalTrackingParameters()
        tp.enable_imu_fusion   = True
        tp.set_floor_as_origin = True
        ok = zed.enable_positional_tracking(tp) == sl.ERROR_CODE.SUCCESS
        self._active = ok
        print("VIO tracking enabled" if ok else "VIO tracking failed")
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


# ── Backprojector ─────────────────────────────────────────────────────────────
_R_OPT_TO_LINK = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)

def project(
    pkt: DepthFramePacket,
    stable: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray | None]:
    """DepthFramePacket → (xyz world-frame N×3, colors N×3 uint8 or None)."""
    valid = np.isfinite(pkt.depth)
    if stable is not None:
        valid = valid & stable
    if not valid.any():
        return np.empty((0, 3), dtype=np.float32), None

    if pkt.xyz_cam_hw is not None:
        xyz_cam = pkt.xyz_cam_hw[valid]
    else:
        d = pkt.depth
        H, W = d.shape
        fx, fy, cx, cy = pkt.intrinsics
        uu, vv = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
        dd = d[valid]
        xyz_opt = np.column_stack([(uu[valid]-cx)*dd/fx, (vv[valid]-cy)*dd/fy, dd]).astype(np.float32)
        xyz_cam = xyz_opt @ _R_OPT_TO_LINK.T

    xyz_world = (xyz_cam @ pkt.pose_R.T + pkt.pose_t).astype(np.float32)
    colors    = pkt.colors_hw[valid] if pkt.colors_hw is not None else None
    return xyz_world, colors


# ── Entry point ───────────────────────────────────────────────────────────────
def main() -> None:
    rr.init("zed_depth_costmap", spawn=True)
    rr.send_blueprint(rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(name="live cloud", origin="world",
                              contents=["world/cloud", "world/camera/**"]),
            rrb.Spatial3DView(name="world map", origin="world",
                              contents=["world/map"]),
        )
    ))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/cloud", rr.Points3D([[0, 0, 0]]), static=True)

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

    zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 50)
    print("camera open — Ctrl-C to quit.")

    src       = ZEDDepthSource(zed)
    src.enable_tracking()
    world_map = FastVoxelMap()

    rt = sl.RuntimeParameters()
    rt.remove_saturated_areas = True

    frame = 0
    t0    = time.monotonic()

    try:
        while True:
            if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
                continue
            ts  = time.monotonic()
            pkt = src.read(ts)

            rr.set_time("frame", sequence=frame)
            rr.log("world/camera", rr.Pinhole(image_from_camera=pkt.K,
                                               width=pkt.width, height=pkt.height), static=True)
            rr.log("world/camera/depth", rr.DepthImage(pkt.depth, meter=1.0))

            # ── Gradient stability filter (same as RealSense) ─────────────────
            # Pixels where depth changes abruptly are stereo edge artifacts.
            # Remove them before backprojection so they never enter the map.
            depth_f  = np.where(np.isfinite(pkt.depth), pkt.depth, 0.0).astype(np.float64)
            grad_mag = np.hypot(sobel(depth_f, axis=1), sobel(depth_f, axis=0))
            stable   = np.isfinite(pkt.depth) & (grad_mag < _GRAD_THRESH)

            xyz, colors = project(pkt, stable)
            if len(xyz) == 0:
                frame += 1
                continue

            cam_z   = float(pkt.pose_t[2])
            cam_pos = pkt.pose_t

            # ── Live cloud: nearest-per-ray denoised ──────────────────────────
            near    = _nearest_per_ray_idx(xyz, cam_pos)
            xyz_vis = xyz[near]
            col_vis = colors[near] if colors is not None else _height_color(xyz_vis[:, 2] - cam_z)
            rr.log("world/cloud", rr.Points3D(positions=xyz_vis, colors=col_vis, radii=0.003))

            # ── Voxel map: height filter → isolation → voxelise ─────────────
            # Floor removal strategy:
            #   VIO locked   → world Z is gravity-aligned (set_floor_as_origin puts floor
            #                  at 0). Absolute threshold world_Z > _FLOOR_Z keeps anything
            #                  3 cm above the floor — including chair legs and low boxes —
            #                  while removing the floor plane itself.
            #   VIO unlocked → world = camera-link frame, not gravity-aligned; fall back
            #                  to camera-relative band.
            # Ray-angle (d_z_norm) filter is intentionally absent: at 0.9 m camera height
            # it cuts everything below ~0.65 m at typical ranges, destroying low obstacles.
            # Gradient filter + isolation filter handle ghost pixels instead.
            h_rel = xyz[:, 2] - cam_z
            if src.pose_locked:
                keep = (xyz[:, 2] > _FLOOR_Z) & (h_rel <= _Z_REL_HI)
            else:
                keep = (h_rel >= -1.4) & (h_rel <= _Z_REL_HI)
            xyz_obs = _filter_isolated(xyz[keep])
            world_map.add(xyz_obs)

            if frame % MAP_EVERY == 0 and world_map.count > 0:
                pts = world_map.points()
                rr.log("world/map", rr.Points3D(
                    positions=pts,
                    colors=_height_color(pts[:, 2] - cam_z),
                    radii=0.010,
                ))

            fps = frame / max(ts - t0, 1e-6)
            print(
                f"frame={frame:5d}  cloud={len(xyz_vis):5d}  map={world_map.count:6d}"
                f"  vio={'LOCKED' if src.pose_locked else 'searching'}  fps={fps:.1f}",
                flush=True,
            )
            frame += 1

    except KeyboardInterrupt:
        pass

    zed.close()
    print("done")


if __name__ == "__main__":
    main()
