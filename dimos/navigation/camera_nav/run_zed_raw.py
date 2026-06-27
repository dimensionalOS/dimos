"""ZED Mini → depth map + dense point cloud viewer.

ZED performs stereo triangulation to produce a per-pixel depth map (NEURAL mode
refines it with a learned stereo matcher).  We back-project that depth map into
3-D using the factory-calibrated intrinsics:

    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth

This gives a dense, coloured point cloud that captures every object in view.
A --stride flag (default 2) skips pixels to stay lightweight on any device.

Usage:
    python -m dimos.navigation.camera_nav.run_zed_raw
    python -m dimos.navigation.camera_nav.run_zed_raw --resolution HD720
    python -m dimos.navigation.camera_nav.run_zed_raw --stride 1   # full density
    python -m dimos.navigation.camera_nav.run_zed_raw --stride 4   # very light
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import pyzed.sl as sl
import rerun as rr


def main() -> None:
    parser = argparse.ArgumentParser(description="ZED Mini depth + dense point cloud")
    parser.add_argument("--resolution", default="VGA",
                        choices=["VGA", "HD720", "HD1080"])
    parser.add_argument("--max-depth", type=float, default=8.0,
                        help="Clip depth beyond this distance in metres (default: 8)")
    parser.add_argument("--stride", type=int, default=2,
                        help="Sample every Nth pixel (default: 2). "
                             "1=full density, 4=very lightweight")
    args = parser.parse_args()

    # --- Rerun ---
    rr.init("zed_depth", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # --- Open ZED ---
    zed = sl.Camera()
    init_p = sl.InitParameters()
    init_p.camera_resolution   = getattr(sl.RESOLUTION, args.resolution)
    init_p.camera_fps          = 15
    init_p.depth_mode          = sl.DEPTH_MODE.NEURAL
    init_p.coordinate_system   = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init_p.coordinate_units    = sl.UNIT.METER
    init_p.depth_maximum_distance = args.max_depth

    print(f"Opening ZED ({args.resolution}, NEURAL depth, stride={args.stride}) …")
    err = zed.open(init_p)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED failed to open: {err}")
    print("ZED opened OK")

    # --- Factory calibration (used for back-projection) ---
    cam_info = zed.get_camera_information()
    cal = cam_info.camera_configuration.calibration_parameters.left_cam
    W   = cam_info.camera_configuration.resolution.width
    H   = cam_info.camera_configuration.resolution.height
    fx, fy, cx, cy = cal.fx, cal.fy, cal.cx, cal.cy

    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    # Log the pinhole once so Rerun can overlay the colour and depth images in 3-D.
    rr.log("world/camera", rr.Pinhole(image_from_camera=K, width=W, height=H), static=True)

    # Pre-build sub-sampled pixel grids (constant across frames).
    s = args.stride
    us = np.arange(0, W, s, dtype=np.float32)
    vs = np.arange(0, H, s, dtype=np.float32)
    uu, vv = np.meshgrid(us, vs)           # (H/s, W/s)
    # Normalised pixel directions — multiply by depth to get camera-frame XYZ.
    ray_x = (uu - cx) / fx                 # (H/s, W/s)
    ray_y = (vv - cy) / fy

    runtime = sl.RuntimeParameters()
    img_mat   = sl.Mat()
    depth_mat = sl.Mat()

    frame = 0
    t0 = time.monotonic()

    try:
        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.001)
                continue

            # --- Colour image ---
            zed.retrieve_image(img_mat, sl.VIEW.LEFT)
            img_bgra = img_mat.get_data()
            img_rgb  = img_bgra[:, :, 2::-1].copy()   # BGRA → RGB
            rr.log("world/camera/color", rr.Image(img_rgb))

            # --- Depth map (metres, stereo triangulation) ---
            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
            depth = depth_mat.get_data()
            if depth.ndim == 3:
                depth = depth[:, :, 0]
            depth_clean = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)
            rr.log("world/camera/depth", rr.DepthImage(depth_clean, meter=1.0))

            # --- Dense point cloud via back-projection ---
            # Sample depth and colour at stride intervals.
            d_sub = depth_clean[::s, ::s]              # (H/s, W/s)
            valid = (d_sub > 0.15) & (d_sub < args.max_depth)

            d   = d_sub[valid]
            X   = ray_x[valid] * d
            Y   = ray_y[valid] * d
            Z   = d
            xyz = np.column_stack([X, Y, Z]).astype(np.float32)

            # Sample RGB at the same strided pixels.
            col_sub = img_rgb[::s, ::s, :]             # (H/s, W/s, 3)
            colors  = col_sub.reshape(-1, 3)[valid.ravel()]

            rr.log("world/pointcloud", rr.Points3D(positions=xyz, colors=colors))

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
