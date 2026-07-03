"""RealSense accumulated global map.

Per-frame voxel map (world/map) + persistent global map (world/global_map).

Key design decisions
--------------------
Per-frame map  uses xyz_world = xyz_cam @ R.T + t  (ICP translation included)
               Refreshes every frame.  Moving objects update.  Never accumulates.

Global map     uses xyz_ronly = xyz_world - t = xyz_cam @ R.T  (rotation ONLY)
               ICP translation is deliberately excluded.  On the D435i there is
               no wheel odometry or VIO, so t from centroid-ICP oscillates ±1-2 cm
               even for a static camera.  At 2 cm voxel size that causes the same
               physical surface to land on different keys every frame → map explodes.
               Madgwick orientation (R) is reliable after convergence.  Using R only
               gives a stable panoramic map: pan the camera left/right and each new
               area of the scene lands at its correct world-frame position.

Usage
    python -m dimos.navigation.camera_nav.realsense_depth_map_acc
"""
from __future__ import annotations

import time

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

# ±1 voxel shift in packed-key space for each axis.
# Used in the fat-voxel clearing step to absorb ±2 cm ICP-t drift.
_SHIFTS = np.array([0, 1<<36, -(1<<36), 1<<18, -(1<<18), 1, -1], dtype=np.int64)


from dimos.navigation.camera_nav.realsense_depth_map import (
    DepthBackprojector,
    GradientStabilityFilter,
    RealSenseDepthSource,
    _FloorCalibrator,
    _VOX_SIZE,
    _height_color,
    _pack,
)


def _raycast_free_keys(
    surface_pts: np.ndarray,
    vox_size: float,
    n_rays: int = 400,
) -> np.ndarray:
    """Return int64 voxel keys that are free space between camera and surface.

    Camera is always at [0,0,0] in the rotation-only world frame.
    Fully vectorised: builds all ray-step grids at once, no Python loop.
    Stops 1.5 voxels short of each surface point so the surface itself is
    never cleared.

    Ghost clearing requirement: the camera must look toward the old object
    position and observe a surface BEHIND it (floor, wall, new object).
    If the old position is outside the current FOV, no rays reach it and
    the ghost persists until the camera sweeps back over that area.
    """
    n = min(len(surface_pts), n_rays)
    if n == 0:
        return np.empty(0, dtype=np.int64)

    idx   = (np.random.choice(len(surface_pts), n, replace=False)
             if len(surface_pts) > n else np.arange(n))
    pts   = surface_pts[idx]                   # (n, 3)
    dists = np.linalg.norm(pts, axis=1)        # camera at [0,0,0]
    ok    = dists > vox_size * 2               # skip points too close to origin
    pts, dists = pts[ok], dists[ok]
    if not len(pts):
        return np.empty(0, dtype=np.int64)

    dirs      = pts / dists[:, None]           # (n, 3) unit vectors
    max_steps = int(np.ceil(dists.max() / vox_size))

    # t-values for every step; stop 1.5 voxels short to avoid clipping surface
    t_vals  = np.arange(max_steps, dtype=np.float32) * vox_size   # (max_steps,)
    t_end   = (dists - vox_size * 1.5)[:, None]                   # (n, 1)
    valid_m = (t_vals[None, :] >= 0) & (t_vals[None, :] < t_end)  # (n, max_steps)

    # Positions along all rays: (n, max_steps, 3)
    ray_pts = dirs[:, None, :] * t_vals[None, :, None]

    vk_flat  = np.floor(ray_pts.reshape(-1, 3) / vox_size).astype(np.int32)
    keys_all = _pack(vk_flat)                  # (n * max_steps,)
    return np.unique(keys_all[valid_m.ravel()])


def main() -> None:
    rr.init("realsense_acc", spawn=True)
    rr.send_blueprint(rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(name="live cloud",  origin="world",
                              contents=["world/cloud", "world/camera/**"]),
            rrb.Spatial3DView(name="voxel map",   origin="world",
                              contents=["world/map"]),
            rrb.Spatial3DView(name="global map",  origin="world",
                              contents=["world/global_map"]),
        )
    ))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    src         = RealSenseDepthSource(width=848, height=480, fps=15)
    bp          = DepthBackprojector()
    grad        = GradientStabilityFilter()
    floor_calib = _FloorCalibrator()

    acc_pts:   np.ndarray = np.empty((0, 3), dtype=np.float32)
    map_ready: bool       = False
    frame          = 0
    t0             = time.monotonic()
    pinhole_logged = False

    print("RealSense open — hold still ~4 s for IMU + floor calibration, then explore freely.")

    try:
        while True:
            ts  = time.monotonic()
            pkt = src.read(ts)

            rr.set_time("frame", sequence=frame)

            if not pinhole_logged:
                rr.log("world/camera", rr.Pinhole(
                    image_from_camera=pkt.K, width=pkt.width, height=pkt.height,
                ), static=True)
                pinhole_logged = True

            rr.log("world/camera/depth", rr.DepthImage(pkt.depth, meter=1.0))

            # Project depth → world frame (xyz = xyz_cam @ R.T + t_icp)
            stable_mask = grad.compute(pkt.depth)
            xyz, colors = bp.project(pkt, stable_mask)
            if not len(xyz):
                frame += 1
                continue

            cam_z = float(pkt.pose_t[2])

            rr.log("world/cloud", rr.Points3D(
                positions=xyz,
                colors=colors if colors is not None else _height_color(xyz[:, 2] - cam_z),
                radii=0.003,
            ))

            # Floor calibration uses camera_link frame (Z always up, independent of IMU)
            xyz_cam = xyz @ pkt.pose_R
            floor_calib.update(xyz_cam)
            if floor_calib.ready:
                keep        = xyz_cam[:, 2] > (floor_calib.floor_z + 0.03)
                xyz_kept    = xyz[keep]
                xyz_cam_icp = xyz_cam[keep]
            else:
                xyz_kept    = xyz
                xyz_cam_icp = xyz_cam

            if len(xyz_kept):
                # ── Per-frame voxel map (includes ICP translation) ───────────
                vk       = np.floor(xyz_kept / _VOX_SIZE).astype(np.int32)
                _, first = np.unique(_pack(vk), return_index=True)
                xyz_vox  = xyz_kept[first]

                rr.log("world/map", rr.Points3D(
                    positions=xyz_vox,
                    colors=_height_color(xyz_vox[:, 2] - cam_z),
                    radii=0.010,
                ))

                # ── Global map (rotation ONLY — no ICP translation) ──────────
                # Wait for floor_calib.ready: guarantees ~60 frames of IMU data
                # so Madgwick orientation has converged before we start stacking.
                if floor_calib.ready:
                    if not map_ready:
                        acc_pts   = np.empty((0, 3), dtype=np.float32)
                        map_ready = True
                        print("*** global map started (IMU + floor converged) ***", flush=True)

                    # Strip ICP translation: xyz_ronly = xyz_cam @ R.T (pure rotation)
                    # Use a stricter floor clearance for the global map (8 cm vs 3 cm
                    # for the per-frame map) — accumulated maps collect marginal floor
                    # points across many frames even if each individual frame barely shows them.
                    gmap_keep      = xyz_cam_icp[:, 2] > (floor_calib.floor_z + 0.08)
                    xyz_ronly_kept = (xyz_kept - pkt.pose_t)[gmap_keep]

                    if not len(xyz_ronly_kept):
                        frame += 1
                        continue

                    vk_r       = np.floor(xyz_ronly_kept / _VOX_SIZE).astype(np.int32)
                    _, first_r  = np.unique(_pack(vk_r), return_index=True)
                    xyz_vox_r   = xyz_ronly_kept[first_r]

                    # Ray-cast: clear map voxels the current rays pass through.
                    # Do this BEFORE adding new observations so a moved object's
                    # old ghost gets erased and the new position is then inserted.
                    #
                    # Fat-voxel check: for each stored voxel we test it AND its
                    # 6 face-adjacent neighbours against the free-key set via
                    # searchsorted (free_keys is sorted by np.unique).  This
                    # absorbs the ±2 cm ICP-t oscillation that shifts a ghost's
                    # stored voxel key by exactly 1 step relative to the ray.
                    if len(acc_pts):
                        free_keys = _raycast_free_keys(xyz_vox_r, _VOX_SIZE, n_rays=400)
                        if len(free_keys):
                            keys_acc = _pack(np.floor(acc_pts / _VOX_SIZE).astype(np.int32))
                            shifted  = (keys_acc[:, None] + _SHIFTS[None, :]).ravel()
                            idx      = np.searchsorted(free_keys, shifted)
                            idx      = np.clip(idx, 0, len(free_keys) - 1)
                            in_free  = (free_keys[idx] == shifted).reshape(len(acc_pts), 7).any(axis=1)
                            acc_pts  = acc_pts[~in_free]

                    # Insert current frame's surface observations then dedup
                    acc_pts = np.vstack([acc_pts, xyz_vox_r]) if len(acc_pts) else xyz_vox_r.copy()
                    vk_a    = np.floor(acc_pts / _VOX_SIZE).astype(np.int32)
                    _, ui   = np.unique(_pack(vk_a), return_index=True)
                    acc_pts = acc_pts[ui]

                    rr.log("world/global_map", rr.Points3D(
                        positions=acc_pts,
                        colors=_height_color(acc_pts[:, 2] - cam_z),
                        radii=0.010,
                    ))

            # ICP: still update for per-frame map (world/map uses it)
            if len(xyz_cam_icp) >= 50:
                src.odom.update(xyz_cam_icp, pkt.pose_R)

            fps   = frame / max(time.monotonic() - t0, 1e-6)
            t_icp = src.odom.t
            # rotation angle from identity — stays 0 if IMU not tracking
            theta = np.degrees(np.arccos(np.clip((np.trace(pkt.pose_R) - 1.0) / 2.0, -1.0, 1.0)))
            print(
                f"frame={frame:5d}  rot={theta:.1f}deg  map={len(acc_pts):6d}vox  "
                f"t=[{t_icp[0]:+.3f},{t_icp[1]:+.3f},{t_icp[2]:+.3f}]  "
                f"floor={'ready' if floor_calib.ready else 'calibrating'}  fps={fps:.1f}",
                flush=True,
            )
            frame += 1

    except KeyboardInterrupt:
        pass

    src.stop()
    print(f"done — {len(acc_pts)} voxels in global map")


if __name__ == "__main__":
    main()
