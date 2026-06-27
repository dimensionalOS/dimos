"""ZED Mini → live depth map + accumulating voxel map.

Uses ZED's native stereo depth (NEURAL mode) with positional tracking
(IMU-fused VIO, floor as Z=0 origin) so voxels from successive frames
land in the correct world position as the camera moves.

Outputs in Rerun:
  world/camera/color   — live colour image
  world/camera/depth   — stable depth map (fixed 0→max_depth range)
  world/voxels         — 3-D occupied voxels coloured by height:
                           green  = floor   (Z < 0.3 m)
                           red    = obstacle (robot height band)
                           blue   = overhead (Z > 1.8 m)

Usage:
    python -m dimos.navigation.camera_nav.run_zed_raw
    python -m dimos.navigation.camera_nav.run_zed_raw --voxel 0.05
    python -m dimos.navigation.camera_nav.run_zed_raw --max-depth 6
    python -m dimos.navigation.camera_nav.run_zed_raw --history 300
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import pyzed.sl as sl
import rerun as rr

# Height thresholds in ZED world frame (Z = up, floor = 0).
_Z_FLOOR    = 0.30   # m — below this is ground
_Z_OBSTACLE = 1.80   # m — above this is overhead / ceiling


def _colour_by_height(z: np.ndarray) -> np.ndarray:
    """Return (N,3) uint8 RGB: green=floor, red=obstacle, blue=overhead."""
    c = np.empty((len(z), 3), dtype=np.uint8)
    c[:] = [210, 50, 50]                      # default: obstacle (red)
    c[z < _Z_FLOOR]    = [60, 180, 60]        # floor   (green)
    c[z > _Z_OBSTACLE] = [50, 100, 220]        # overhead (blue)
    return c


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--resolution", default="VGA",
                        choices=["VGA", "HD720", "HD1080"])
    parser.add_argument("--max-depth", type=float, default=8.0)
    parser.add_argument("--voxel",    type=float, default=0.05,
                        help="Voxel size in metres (default 0.05)")
    parser.add_argument("--history",  type=int,   default=200,
                        help="Max frames to keep in voxel map (default 200)")
    args = parser.parse_args()

    rr.init("zed_voxel", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # ── Open ZED ────────────────────────────────────────────────────────────
    zed = sl.Camera()
    ip = sl.InitParameters()
    ip.camera_resolution      = getattr(sl.RESOLUTION, args.resolution)
    ip.camera_fps             = 15
    ip.depth_mode             = sl.DEPTH_MODE.NEURAL
    ip.coordinate_system      = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    ip.coordinate_units       = sl.UNIT.METER
    ip.depth_maximum_distance = args.max_depth

    print(f"Opening ZED ({args.resolution}, NEURAL) …")
    err = zed.open(ip)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED failed to open: {err}")

    # Positional tracking: IMU fusion + floor as Z=0 so height is meaningful.
    tp = sl.PositionalTrackingParameters()
    tp.enable_imu_fusion   = True
    tp.set_floor_as_origin = True
    zed.enable_positional_tracking(tp)
    print("ZED opened + tracking enabled  —  Ctrl-C to quit")

    # Intrinsics for depth image pinhole.
    ci  = zed.get_camera_information()
    cal = ci.camera_configuration.calibration_parameters.left_cam
    W   = ci.camera_configuration.resolution.width
    H   = ci.camera_configuration.resolution.height
    K   = np.array([[cal.fx, 0, cal.cx],
                    [0, cal.fy, cal.cy],
                    [0, 0, 1]], dtype=np.float64)
    rr.log("world/camera",
           rr.Pinhole(image_from_camera=K, width=W, height=H), static=True)

    # ── Voxel map (rolling window across frames) ─────────────────────────────
    # Each entry is an (N,3) int32 array of voxel grid indices.
    # When the deque overflows, the oldest frame's voxels are dropped.
    from collections import deque
    frame_vkeys: deque[np.ndarray] = deque(maxlen=args.history)
    v = args.voxel

    rt  = sl.RuntimeParameters()
    img_mat   = sl.Mat()
    depth_mat = sl.Mat()
    pc_mat    = sl.Mat()

    frame = 0
    t0 = time.monotonic()

    try:
        while True:
            if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.001)
                continue

            # ── Colour image ──────────────────────────────────────────────
            zed.retrieve_image(img_mat, sl.VIEW.LEFT)
            bgra = img_mat.get_data()
            rgb  = bgra[:, :, 2::-1].copy()
            rr.log("world/camera/color", rr.Image(rgb))

            # ── Depth image (stable colourmap) ───────────────────────────
            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
            depth = depth_mat.get_data()
            if depth.ndim == 3:
                depth = depth[:, :, 0]
            depth_u8 = (np.nan_to_num(depth, nan=0.0) / args.max_depth * 255
                        ).clip(0, 255).astype(np.uint8)
            rr.log("world/camera/depth", rr.Image(depth_u8))

            # ── Point cloud: ZED native XYZRGBA in world frame ───────────
            # RIGHT_HANDED_Z_UP_X_FWD: X=forward, Y=left, Z=up.
            # These coordinates are already correct for Rerun's Z-up world.
            zed.retrieve_measure(pc_mat, sl.MEASURE.XYZRGBA)
            pts4 = pc_mat.get_data().reshape(-1, 4)

            dist = np.linalg.norm(pts4[:, :3], axis=1)
            keep = np.isfinite(pts4[:, 0]) & (dist > 0.15) & (dist < args.max_depth)
            pts4 = pts4[keep]

            if len(pts4) == 0:
                frame += 1
                continue

            xyz  = pts4[:, :3].astype(np.float32)
            rgba = pts4[:, 3].view(np.uint32)
            r = ((rgba >> 16) & 0xFF).astype(np.uint8)
            g = ((rgba >>  8) & 0xFF).astype(np.uint8)
            b = ( rgba        & 0xFF).astype(np.uint8)
            rr.log("world/pointcloud",
                   rr.Points3D(positions=xyz,
                               colors=np.column_stack([r, g, b])))

            # ── Voxel grid: quantise + deduplicate + accumulate ───────────
            vkeys = np.floor(xyz / v).astype(np.int32)          # (N,3)
            vkeys = np.unique(vkeys, axis=0)                     # deduplicate
            frame_vkeys.append(vkeys)

            all_vkeys = np.unique(
                np.concatenate(list(frame_vkeys), axis=0), axis=0
            )
            centres = (all_vkeys.astype(np.float32) + 0.5) * v
            vcols   = _colour_by_height(centres[:, 2])           # Z = up

            rr.log("world/voxels",
                   rr.Points3D(positions=centres,
                               colors=vcols,
                               radii=v * 0.45))

            frame += 1
            if frame % 30 == 0:
                fps = frame / (time.monotonic() - t0)
                print(f"frame {frame:5d}  pts={len(xyz):6d}  "
                      f"voxels={len(all_vkeys):6d}  fps={fps:.1f}")

    except KeyboardInterrupt:
        pass
    finally:
        zed.disable_positional_tracking()
        zed.close()
        print("ZED closed")


if __name__ == "__main__":
    main()
