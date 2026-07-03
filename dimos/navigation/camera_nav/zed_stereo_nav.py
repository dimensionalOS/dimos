"""ZED Mini stereo depth pipeline — live cloud + per-frame map + persistent global map.

VIO pipeline (built into the ZED SDK):
  ZEDDepthSource   — ZED SDK frame → DepthFramePacket with pose from on-board VIO
                     Uses sl.MEASURE.XYZ (GPU-computed, camera_link frame) so
                     DepthBackprojector takes the fast path and skips pinhole math.

Three Rerun channels:
  world/cloud      nearest-per-ray denoised live cloud (every frame)
  world/map        per-frame voxel grid, refreshed each frame
  world/world_map  rolling deduplicated world map, refreshed every _MAP_EVERY frames
  world/global_map persistent panoramic map with ray-cast ghost clearing (VIO-locked)

Ghost clearing runs in camera-relative space (xyz_vox - pose_t) using fat-voxel
searchsorted (±1 voxel neighbour check) to absorb any residual VIO translation
jitter before applying deletions to world-frame acc_pts.

Usage:
  python -m dimos.navigation.camera_nav.zed_stereo_nav
"""

from __future__ import annotations

import time

import numpy as np
import pyzed.sl as sl
import rerun as rr
import rerun.blueprint as rrb
from scipy.ndimage import sobel

from dimos.navigation.camera_nav.pipeline import (
    _pack,
    _height_color,
    DepthBackprojector,
    DepthFramePacket,
)


VOX_SIZE     = 0.020   # 2 cm voxel grid — shared by per-frame map and world map
_FLOOR_Z     =  0.03   # world-Z floor cutoff when VIO locked (3 cm above gravity floor)
_GRAD_THRESH =  0.50   # Sobel gradient magnitude threshold
_MAP_EVERY   =  10     # log world/world_map every N frames (display rate, not update rate)
_MAP_MAX_PTS = 80_000  # cap Rerun message size

# ±1 voxel shift in packed-key space for each axis — used in fat-voxel ghost clearing.
_SHIFTS = np.array([0, 1<<36, -(1<<36), 1<<18, -(1<<18), 1, -1], dtype=np.int64)


# ── ZED camera helpers ────────────────────────────────────────────────────────

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


class _PoseReader:
    """ZED VIO camera→world pose. Identity passthrough until VIO locks."""

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
        tp.set_floor_as_origin = True  # Z=0 at floor → floor cutoffs are absolute
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


class ZEDDepthSource:
    """ZED SDK → DepthFramePacket. No confidence filtering — raw depth only.

    Uses sl.MEASURE.XYZ for GPU-computed point cloud in camera_link frame
    (RIGHT_HANDED_Z_UP_X_FWD), so DepthBackprojector takes the fast path.
    Quality filtering happens downstream (gradient filter, voxel map).
    """

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

        # H×W×4 float32 in camera_link frame: [X=fwd, Y=left, Z=up, pad]
        self._zed.retrieve_measure(self._xyz_mat, sl.MEASURE.XYZ, sl.MEM.CPU)
        xyz_raw = self._xyz_mat.get_data()[:, :, :3].astype(np.float32)

        depth = xyz_raw[:, :, 0].copy()   # X=fwd = forward distance
        invalid = ~np.isfinite(depth) | (depth <= self.MIN_DEPTH) | (depth > self.MAX_DEPTH)
        depth[invalid]   = np.nan
        xyz_raw[invalid] = np.nan

        self._zed.retrieve_image(self._img_mat, sl.VIEW.LEFT, sl.MEM.CPU)
        img       = self._img_mat.get_data()
        colors_hw = img[:, :, 2::-1].copy()   # BGRA → RGB uint8

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


# ── Ray-cast ghost clearing ───────────────────────────────────────────────────

def _raycast_free_keys(surface_pts: np.ndarray, vox_size: float, n_rays: int = 400) -> np.ndarray:
    """Return voxel keys that are free space between camera (origin) and surface.

    surface_pts must be in camera-relative frame (camera at [0,0,0]).
    """
    n = min(len(surface_pts), n_rays)
    if n == 0:
        return np.empty(0, dtype=np.int64)
    idx   = (np.random.choice(len(surface_pts), n, replace=False)
             if len(surface_pts) > n else np.arange(n))
    pts   = surface_pts[idx]
    dists = np.linalg.norm(pts, axis=1)
    ok    = dists > vox_size * 2
    pts, dists = pts[ok], dists[ok]
    if not len(pts):
        return np.empty(0, dtype=np.int64)
    dirs      = pts / dists[:, None]
    max_steps = int(np.ceil(dists.max() / vox_size))
    t_vals  = np.arange(max_steps, dtype=np.float32) * vox_size
    t_end   = (dists - vox_size * 1.5)[:, None]
    valid_m = (t_vals[None, :] >= 0) & (t_vals[None, :] < t_end)
    ray_pts  = dirs[:, None, :] * t_vals[None, :, None]
    vk_flat  = np.floor(ray_pts.reshape(-1, 3) / vox_size).astype(np.int32)
    keys_all = _pack(vk_flat)
    return np.unique(keys_all[valid_m.ravel()])


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


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    rr.init("zed_stereo_nav", spawn=True)
    rr.send_blueprint(rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(name="live cloud", origin="world",
                              contents=["world/cloud", "world/camera/**"]),
            rrb.Spatial3DView(name="voxel map", origin="world",
                              contents=["world/map"]),
            rrb.Spatial3DView(name="world map", origin="world",
                              contents=["world/world_map"]),
            rrb.Spatial3DView(name="global map", origin="world",
                              contents=["world/global_map"]),
        )
    ))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/cloud", rr.Points3D([[0, 0, 0]]), static=True)

    zed = sl.Camera()
    ip  = sl.InitParameters()
    ip.camera_resolution      = sl.RESOLUTION.HD720
    ip.camera_fps             = 15
    ip.depth_mode             = sl.DEPTH_MODE.ULTRA
    ip.coordinate_system      = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    ip.coordinate_units       = sl.UNIT.METER
    ip.depth_maximum_distance = 8.0

    if zed.open(ip) != sl.ERROR_CODE.SUCCESS:
        print("ZED failed to open"); return

    # Fixed exposure prevents bright lights from washing out stereo texture.
    zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 50)
    print("camera open — Ctrl-C to quit.")

    src = ZEDDepthSource(zed)
    src.enable_tracking()
    bp  = DepthBackprojector()

    _wm_chunks: list[np.ndarray] = []
    _wm_base:   np.ndarray = np.empty((0, 3), dtype=np.float32)
    acc_pts:    np.ndarray = np.empty((0, 3), dtype=np.float32)

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

            # ── Gradient stability filter ─────────────────────────────────────
            depth_f  = np.where(np.isfinite(pkt.depth), pkt.depth, 0.0).astype(np.float64)
            grad_mag = np.hypot(sobel(depth_f, axis=1), sobel(depth_f, axis=0))
            stable   = np.isfinite(pkt.depth) & (grad_mag < _GRAD_THRESH)

            xyz, colors = bp.project(pkt, stable)
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

            # ── Voxel map: floor removal → voxelise ──────────────────────────
            h_rel = xyz[:, 2] - cam_z
            if src.pose_locked and cam_z > 0.15:
                # Floor established: use absolute world-Z cutoff (floor is at ~0).
                keep = xyz[:, 2] > _FLOOR_Z
            else:
                # Pre-VIO or VIO just locked before floor calibrated: camera-relative band.
                keep = (h_rel >= -1.4) & (h_rel <= 1.5)

            xyz_vox  = np.empty((0, 3), dtype=np.float32)
            xyz_kept = xyz[keep]
            if len(xyz_kept):
                vk       = np.floor(xyz_kept / VOX_SIZE).astype(np.int32)
                _, first = np.unique(_pack(vk), return_index=True)
                xyz_vox  = xyz_kept[first]
                rr.log("world/map", rr.Points3D(
                    positions=xyz_vox,
                    colors=_height_color(xyz_vox[:, 2] - cam_z),
                    radii=0.010,
                ))

            # ── World map: chunk accumulation, dedup every _MAP_EVERY frames ──
            if len(xyz_vox):
                _wm_chunks.append(xyz_vox)

            if frame % _MAP_EVERY == 0:
                if _wm_chunks:
                    fresh    = np.concatenate(_wm_chunks[::-1])  # newest chunk rows first
                    combined = np.concatenate([fresh, _wm_base]) if len(_wm_base) else fresh
                    vk       = np.floor(combined / VOX_SIZE).astype(np.int32)
                    _, first = np.unique(_pack(vk), return_index=True)
                    _wm_base = combined[first]
                    _wm_chunks = []
                if len(_wm_base):
                    pts = _wm_base
                    if len(pts) > _MAP_MAX_PTS:
                        pts = pts[np.random.choice(len(pts), _MAP_MAX_PTS, replace=False)]
                    rr.log("world/world_map", rr.Points3D(
                        positions=pts,
                        colors=_height_color(pts[:, 2] - cam_z),
                        radii=0.012,
                    ))

            # ── Global map: ray-cast ghost clearing + full VIO world coords ────
            if src.pose_locked and len(xyz_vox):
                xyz_vox_rel = xyz_vox - pkt.pose_t   # camera-relative for ray cast

                if len(acc_pts) and frame % 3 == 0:
                    free_keys = _raycast_free_keys(xyz_vox_rel, VOX_SIZE, n_rays=200)
                    if len(free_keys):
                        acc_pts_rel = acc_pts - pkt.pose_t
                        keys_acc = _pack(np.floor(acc_pts_rel / VOX_SIZE).astype(np.int32))
                        shifted  = (keys_acc[:, None] + _SHIFTS[None, :]).ravel()
                        idx      = np.searchsorted(free_keys, shifted)
                        idx      = np.clip(idx, 0, len(free_keys) - 1)
                        in_free  = (free_keys[idx] == shifted).reshape(len(acc_pts), 7).any(axis=1)
                        acc_pts  = acc_pts[~in_free]

                acc_pts = np.vstack([acc_pts, xyz_vox]) if len(acc_pts) else xyz_vox.copy()
                vk_a    = np.floor(acc_pts / VOX_SIZE).astype(np.int32)
                _, ui   = np.unique(_pack(vk_a), return_index=True)
                acc_pts = acc_pts[ui]

                if len(acc_pts) > _MAP_MAX_PTS:
                    acc_pts = acc_pts[np.random.choice(len(acc_pts), _MAP_MAX_PTS, replace=False)]

                rr.log("world/global_map", rr.Points3D(
                    positions=acc_pts,
                    colors=_height_color(acc_pts[:, 2] - cam_z),
                    radii=0.012,
                ))

            fps = frame / max(ts - t0, 1e-6)
            print(
                f"frame={frame:5d}  raw={len(xyz):6d}  keep={keep.sum():6d}"
                f"  vox={len(xyz_vox):5d}  cloud={len(xyz_vis):5d}"
                f"  world={len(_wm_base):6d}  global={len(acc_pts):6d}  cam_z={cam_z:.2f}"
                f"  vio={'LOCKED' if src.pose_locked else 'search'}  fps={fps:.1f}",
                flush=True,
            )
            frame += 1

    except KeyboardInterrupt:
        pass

    zed.close()
    print("done")


if __name__ == "__main__":
    main()
