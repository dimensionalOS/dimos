"""ZED Mini → live cloud + per-frame voxel map.

world/cloud  nearest-per-ray denoised live cloud
world/map    full-density cloud voxelised at VOX_SIZE — one point per occupied cell

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


VOX_SIZE     = 0.020   # 2 cm voxel grid — shared by per-frame map and world map
_FLOOR_Z    =  0.03   # world-Z floor cutoff when VIO locked (3 cm above gravity floor)
_GRAD_THRESH =  0.50  # Sobel gradient magnitude threshold — 0.30 was too tight, killed too many pixels in corners/angled surfaces
_MAP_EVERY  =  10     # log world/world_map every N frames (display rate, not update rate)
_MAP_MAX_PTS = 80_000 # cap Rerun message size

# ── Voxel key packing ────────────────────────────────────────────────────────
_VOFF  = np.int64(100_000)
_VMASK = np.int64(0x3FFFF)

def _pack(vkeys: np.ndarray) -> np.ndarray:
    v = (vkeys.astype(np.int64) + _VOFF) & _VMASK
    return (v[:, 0] << 36) | (v[:, 1] << 18) | v[:, 2]




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
                print(f"*** VIO LOCKED after {now - self._t0:.1f}s  pose_t={self._t.tolist()} ***")
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
            rrb.Spatial3DView(name="voxel map", origin="world",
                              contents=["world/map"]),
            rrb.Spatial3DView(name="world map", origin="world",
                              contents=["world/world_map"]),
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

    src = ZEDDepthSource(zed)
    src.enable_tracking()
    _wm_chunks: list[np.ndarray] = []                        # per-frame voxels since last dedup
    _wm_base:   np.ndarray = np.empty((0, 3), dtype=np.float32)  # deduplicated world map

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
            # Newest observations win: when the same voxel key appears in
            # both fresh chunks and _wm_base, the chunk (recent) takes priority.
            if src.pose_locked and len(xyz_vox):
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

            fps = frame / max(ts - t0, 1e-6)
            print(
                f"frame={frame:5d}  raw={len(xyz):6d}  keep={keep.sum():6d}"
                f"  vox={len(xyz_vox):5d}  cloud={len(xyz_vis):5d}"
                f"  world={len(_wm_base):6d}  cam_z={cam_z:.2f}"
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
