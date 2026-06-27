"""ZED Mini → depth map + point cloud viewer.

ZED already performs stereo triangulation internally (NEURAL mode refines it
with a learned stereo matcher).  This script retrieves:

  - sl.MEASURE.DEPTH      → per-pixel depth in metres (from triangulation)
  - sl.MEASURE.XYZRGBA    → native 3-D point cloud (derived from same depth)
  - sl.VIEW.LEFT          → colour image

All three are streamed to Rerun with the camera's factory calibration (pinhole)
so the colour image, depth image, and point cloud are co-registered in 3-D.

Usage:
    python -m dimos.navigation.camera_nav.run_zed_raw
    python -m dimos.navigation.camera_nav.run_zed_raw --resolution HD720
    python -m dimos.navigation.camera_nav.run_zed_raw --max-depth 8
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import pyzed.sl as sl
import rerun as rr


def main() -> None:
    parser = argparse.ArgumentParser(description="ZED Mini depth + point cloud viewer")
    parser.add_argument("--resolution", default="VGA",
                        choices=["VGA", "HD720", "HD1080"])
    parser.add_argument("--max-depth", type=float, default=8.0,
                        help="Clip depth beyond this distance in metres (default: 8)")
    args = parser.parse_args()

    # --- Rerun ---
    rr.init("zed_depth", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # --- Open ZED ---
    zed = sl.Camera()
    init_p = sl.InitParameters()
    init_p.camera_resolution = getattr(sl.RESOLUTION, args.resolution)
    init_p.camera_fps = 15
    init_p.depth_mode = sl.DEPTH_MODE.NEURAL
    init_p.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init_p.coordinate_units = sl.UNIT.METER
    init_p.depth_maximum_distance = args.max_depth

    print(f"Opening ZED ({args.resolution}, NEURAL depth) ...")
    err = zed.open(init_p)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED failed to open: {err}")
    print("ZED opened OK")

    # --- Camera intrinsics (factory calibration) ---
    cam_info = zed.get_camera_information()
    cal = cam_info.camera_configuration.calibration_parameters.left_cam
    W = cam_info.camera_configuration.resolution.width
    H = cam_info.camera_configuration.resolution.height
    K = np.array([[cal.fx, 0, cal.cx],
                  [0, cal.fy, cal.cy],
                  [0,      0,      1]], dtype=np.float64)

    # Log the camera pinhole once — Rerun uses this to back-project the
    # depth image into 3-D and to overlay the colour image on the point cloud.
    rr.log("world/camera", rr.Pinhole(image_from_camera=K, width=W, height=H), static=True)

    runtime = sl.RuntimeParameters()
    img_mat   = sl.Mat()
    depth_mat = sl.Mat()
    pc_mat    = sl.Mat()

    frame = 0
    t0 = time.monotonic()

    try:
        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.001)
                continue

            # --- Colour image ---
            zed.retrieve_image(img_mat, sl.VIEW.LEFT)
            img_bgra = img_mat.get_data()           # (H, W, 4) uint8 BGRA
            img_rgb  = img_bgra[:, :, 2::-1].copy() # → RGB
            rr.log("world/camera/color", rr.Image(img_rgb))

            # --- Depth map (metres, from stereo triangulation) ---
            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
            depth = depth_mat.get_data()            # (H, W) float32, metres
            if depth.ndim == 3:
                depth = depth[:, :, 0]
            # Replace NaN/Inf with 0 so Rerun renders cleanly
            depth_clean = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)
            rr.log("world/camera/depth",
                   rr.DepthImage(depth_clean, meter=1.0))

            # --- Native point cloud (co-registered with colour) ---
            zed.retrieve_measure(pc_mat, sl.MEASURE.XYZRGBA)
            pts4 = pc_mat.get_data().reshape(-1, 4)
            dist  = np.linalg.norm(pts4[:, :3], axis=1)
            keep  = np.isfinite(pts4[:, 0]) & (dist > 0.15) & (dist < args.max_depth)
            pts4  = pts4[keep]
            xyz   = pts4[:, :3].astype(np.float32)
            rgba  = pts4[:, 3].view(np.uint32)
            r = ((rgba >> 16) & 0xFF).astype(np.uint8)
            g = ((rgba >>  8) & 0xFF).astype(np.uint8)
            b = ( rgba        & 0xFF).astype(np.uint8)
            rr.log("world/pointcloud",
                   rr.Points3D(positions=xyz, colors=np.column_stack([r, g, b])))

            frame += 1
            if frame % 30 == 0:
                fps = frame / (time.monotonic() - t0)
                print(f"frame {frame:5d}  pts={len(xyz):6d}  fps={fps:.1f}")

    except KeyboardInterrupt:
        pass
    finally:
        zed.close()
        print("ZED closed")


if __name__ == "__main__":
    main()
