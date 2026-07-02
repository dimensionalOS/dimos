"""RealSense accumulated global map.

Per-frame voxel map + persistent global map in one loop.
Run this instead of realsense_depth_map.py when you want global accumulation.

    python -m dimos.navigation.camera_nav.realsense_depth_map_acc

Rerun tabs:
  world/map        — per-frame voxel grid (refreshes every frame, moving objects update)
  world/global_map — persistent map that grows as camera explores new space
  world/cloud      — live dense point cloud
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

    # Global map: plain float32 array, grows as camera explores new space.
    # Only populated after floor_calib.ready so that Madgwick has converged
    # and xyz_world positions are stable.  Pre-convergence geometry is thrown away.
    acc_pts:           np.ndarray = np.empty((0, 3), dtype=np.float32)
    map_ready:         bool       = False   # True once we've started accumulating

    frame          = 0
    t0             = time.monotonic()
    pinhole_logged = False

    print("RealSense open — Ctrl-C to quit.")
    print("Hold still ~2 s for IMU to converge.")

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

            # Gradient-stable world-frame points
            stable_mask    = grad.compute(pkt.depth)
            xyz, colors    = bp.project(pkt, stable_mask)
            if not len(xyz):
                frame += 1
                continue

            cam_z = float(pkt.pose_t[2])

            rr.log("world/cloud", rr.Points3D(
                positions=xyz,
                colors=colors if colors is not None else _height_color(xyz[:, 2] - cam_z),
                radii=0.003,
            ))

            # Floor filter in camera_link frame (Z always up)
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
                # ── Per-frame voxel map ──────────────────────────────────────
                vk       = np.floor(xyz_kept / _VOX_SIZE).astype(np.int32)
                _, first = np.unique(_pack(vk), return_index=True)
                xyz_vox  = xyz_kept[first]

                rr.log("world/map", rr.Points3D(
                    positions=xyz_vox,
                    colors=_height_color(xyz_vox[:, 2] - cam_z),
                    radii=0.010,
                ))

                # ── Persistent global map ────────────────────────────────────
                # Don't start until floor_calib is ready: that guarantees
                # Madgwick has converged (~60 frames) so world-frame positions
                # are stable.  Pre-convergence geometry would corrupt the map.
                if floor_calib.ready:
                    if not map_ready:
                        # First frame after convergence: start from clean slate
                        acc_pts   = np.empty((0, 3), dtype=np.float32)
                        map_ready = True
                        print("*** global map started (IMU converged) ***", flush=True)

                    acc_pts = np.vstack([acc_pts, xyz_vox]) if len(acc_pts) else xyz_vox.copy()
                    vk_a    = np.floor(acc_pts / _VOX_SIZE).astype(np.int32)
                    _, ui   = np.unique(_pack(vk_a), return_index=True)
                    acc_pts = acc_pts[ui]

                    rr.log("world/global_map", rr.Points3D(
                        positions=acc_pts,
                        colors=_height_color(acc_pts[:, 2] - cam_z),
                        radii=0.010,
                    ))

            # ICP: refines camera translation so next frame lands at correct world pos
            if len(xyz_cam_icp) >= 50:
                src.odom.update(xyz_cam_icp, pkt.pose_R)

            fps   = frame / max(time.monotonic() - t0, 1e-6)
            t_icp = src.odom.t
            print(
                f"frame={frame:5d}  map={len(acc_pts):6d}vox  "
                f"t=[{t_icp[0]:+.2f},{t_icp[1]:+.2f},{t_icp[2]:+.2f}]  fps={fps:.1f}",
                flush=True,
            )
            frame += 1

    except KeyboardInterrupt:
        pass

    src.stop()
    print(f"done — {len(acc_pts)} voxels in global map")


if __name__ == "__main__":
    main()
