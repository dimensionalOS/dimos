"""RealSense accumulated global map.

Per-frame voxel map (world/map) + persistent global map (world/global_map).

Key design decisions
--------------------
Both maps use the same coordinate system:
  xyz_world = xyz_cam @ R.T + t   (ICP translation included)

The per-frame map refreshes every frame — current view only.
The global map accumulates across frames so the same physical surface always
lands on the same world-frame voxel key regardless of robot position.

Using xyz_world (not rotation-only) is essential on a moving robot: in the
rotation-only frame, xyz_ronly = xyz_world - t shifts with t, so revisiting
a surface from a new robot position creates a duplicate voxel.  In world
frame, ICP t correctly places each observed surface at its stable world-space
address regardless of where the robot was when it observed it.

Usage
    python -m dimos.navigation.camera_nav.realsense_depth_map_acc
"""
from __future__ import annotations

import time

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

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
    cam_origin: np.ndarray | None = None,
) -> np.ndarray:
    """Return int64 voxel keys that are free space between camera and surface.

    surface_pts: (N, 3) surface points in the same frame as acc_pts.
    cam_origin:  (3,) camera position in that same frame. Defaults to [0,0,0].
    Fully vectorised: builds all ray-step grids at once, no Python loop.
    Stops one voxel short of each surface point so the surface itself is
    never cleared.
    """
    origin = np.zeros(3, dtype=np.float32) if cam_origin is None else np.asarray(cam_origin, dtype=np.float32)

    n = min(len(surface_pts), n_rays)
    if n == 0:
        return np.empty(0, dtype=np.int64)

    idx  = (np.random.choice(len(surface_pts), n, replace=False)
            if len(surface_pts) > n else np.arange(n))
    pts  = surface_pts[idx]                        # (n, 3)
    vecs = pts - origin                            # (n, 3) from camera to surface
    dists = np.linalg.norm(vecs, axis=1)           # (n,)
    ok    = dists > vox_size * 2                   # skip points too close to origin
    vecs, dists = vecs[ok], dists[ok]
    if not len(vecs):
        return np.empty(0, dtype=np.int64)

    dirs      = vecs / dists[:, None]              # (n, 3) unit vectors
    max_steps = int(np.ceil(dists.max() / vox_size))

    # t-values for every step; stop 1.5 voxels short to avoid clipping surface
    t_vals  = np.arange(max_steps, dtype=np.float32) * vox_size   # (max_steps,)
    t_end   = (dists - vox_size * 1.5)[:, None]                   # (n, 1)
    valid_m = (t_vals[None, :] >= 0) & (t_vals[None, :] < t_end)  # (n, max_steps)

    # Positions along all rays: (n, max_steps, 3)
    ray_pts = origin + dirs[:, None, :] * t_vals[None, :, None]

    vk_flat  = np.floor(ray_pts.reshape(-1, 3) / vox_size).astype(np.int32)
    keys_all = _pack(vk_flat)                      # (n * max_steps,)
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

    acc_pts:      np.ndarray = np.empty((0, 3), dtype=np.float32)
    map_ready:    bool       = False
    world_floor_z: float     = 0.0   # set once at map start; never updated
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
                # ── Per-frame voxel map ───────────────────────────────────────
                vk       = np.floor(xyz_kept / _VOX_SIZE).astype(np.int32)
                _, first = np.unique(_pack(vk), return_index=True)
                xyz_vox  = xyz_kept[first]

                rr.log("world/map", rr.Points3D(
                    positions=xyz_vox,
                    colors=_height_color(xyz_vox[:, 2] - cam_z),
                    radii=0.010,
                ))

                # ── Global map (world frame — same coordinate as per-frame map) ──
                # xyz_vox is already floor-filtered, gradient-stable, and voxelised.
                # Using world frame (not rotation-only) ensures the same physical
                # surface always maps to the same voxel key regardless of robot
                # position — required for correct accumulation on a moving robot.
                if floor_calib.ready:
                    if not map_ready:
                        acc_pts       = np.empty((0, 3), dtype=np.float32)
                        map_ready     = True
                        # Lock floor threshold once in world frame.  cam_z ≈ 0 here
                        # (robot stationary during calibration) so ICP Z drift can't
                        # corrupt the threshold later.  floor_calib.floor_z is negative
                        # (floor below camera); world_floor_z ≈ world Z of the floor.
                        world_floor_z = cam_z + floor_calib.floor_z
                        print(f"*** global map started — floor at world Z ≈ {world_floor_z:.3f} m ***",
                              flush=True)

                    # World-frame floor filter: more robust than camera-frame filter
                    # when camera tilts on uneven terrain.  Fixed threshold doesn't
                    # drift with ICP Z noise.
                    xyz_for_map = xyz_vox[xyz_vox[:, 2] > world_floor_z + 0.04]

                    # Ray-cast BEFORE inserting so moved-object ghosts are erased first.
                    # Camera origin in world frame = pkt.pose_t.
                    if len(acc_pts) and len(xyz_for_map):
                        free_keys = _raycast_free_keys(
                            xyz_for_map, _VOX_SIZE, n_rays=400, cam_origin=pkt.pose_t,
                        )
                        if len(free_keys):
                            keys_acc = _pack(np.floor(acc_pts / _VOX_SIZE).astype(np.int32))
                            acc_pts  = acc_pts[~np.isin(keys_acc, free_keys)]

                    if len(xyz_for_map):
                        acc_pts = np.vstack([acc_pts, xyz_for_map]) if len(acc_pts) else xyz_for_map.copy()
                        _, ui   = np.unique(_pack(np.floor(acc_pts / _VOX_SIZE).astype(np.int32)),
                                            return_index=True)
                        acc_pts = acc_pts[ui]

                    rr.log("world/global_map", rr.Points3D(
                        positions=acc_pts,
                        colors=_height_color(acc_pts[:, 2] - cam_z),
                        radii=0.010,
                    ))

            # ICP: update odometry for next frame's t
            if len(xyz_cam_icp) >= 50:
                src.odom.update(xyz_cam_icp, pkt.pose_R)

            fps   = frame / max(time.monotonic() - t0, 1e-6)
            t_icp = src.odom.t
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
