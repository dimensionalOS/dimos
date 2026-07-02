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

from dimos.navigation.camera_nav.realsense_depth_map import (
    DepthBackprojector,
    GradientStabilityFilter,
    RealSenseDepthSource,
    _FloorCalibrator,
    _VOX_SIZE,
    _height_color,
    _pack,
)


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
                    # floor_kept points in rotation-only world frame
                    xyz_ronly_kept = xyz_kept - pkt.pose_t

                    vk_r      = np.floor(xyz_ronly_kept / _VOX_SIZE).astype(np.int32)
                    _, first_r = np.unique(_pack(vk_r), return_index=True)
                    xyz_vox_r  = xyz_ronly_kept[first_r]

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
