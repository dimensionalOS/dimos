"""ZED Mini → live cloud + voxel maps.

world/cloud      nearest-per-ray denoised live cloud
world/map        per-frame voxel grid, refreshed each frame
world/world_map  rolling deduplicated world map, refreshed every _MAP_EVERY frames
world/global_map persistent panoramic map with ray-cast ghost clearing (VIO-locked)

Usage:
  python -m dimos.navigation.camera_nav.zed_depth_costmap
"""

from __future__ import annotations

import time

import numpy as np
import pyzed.sl as sl
import rerun as rr
import rerun.blueprint as rrb
from scipy.ndimage import sobel

from dimos.navigation.camera_nav.pipeline import _pack, _height_color, DepthBackprojector
from dimos.navigation.camera_nav.zed_depth_map import ZEDDepthSource


VOX_SIZE     = 0.020   # 2 cm voxel grid — shared by per-frame map and world map
_FLOOR_Z     =  0.03   # world-Z floor cutoff when VIO locked (3 cm above gravity floor)
_GRAD_THRESH =  0.50   # Sobel gradient magnitude threshold
_MAP_EVERY   =  10     # log world/world_map every N frames (display rate, not update rate)
_MAP_MAX_PTS = 80_000  # cap Rerun message size

# ±1 voxel shift in packed-key space for each axis — used in fat-voxel ghost clearing.
_SHIFTS = np.array([0, 1<<36, -(1<<36), 1<<18, -(1<<18), 1, -1], dtype=np.int64)


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
    rr.init("zed_depth_costmap", spawn=True)
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
            # Pixels where depth changes abruptly are stereo edge artifacts.
            # Remove them before backprojection so they never enter the map.
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
            # Newest observations win: when the same voxel key appears in
            # both fresh chunks and _wm_base, the chunk (recent) takes priority.
            # Accumulates immediately — does not wait for VIO lock.
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
            # ZED VIO translation is reliable so we keep full world coordinates
            # (unlike RealSense which strips translation to avoid ICP drift).
            # Ghost clearing is done in camera-relative space then applied to
            # world-frame acc_pts. Runs every 3 frames to keep CPU load low.
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
