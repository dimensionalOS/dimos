"""Minimal ZED Mini → Rerun viewer. No dimos pipeline, no accumulator, no ICP.

Opens the camera, grabs native XYZRGBA point clouds and colour images,
streams them straight to Rerun. Use this to confirm the ZED is working
before running the full pipeline.

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
    parser = argparse.ArgumentParser(description="ZED Mini raw viewer")
    parser.add_argument("--resolution", default="VGA",
                        choices=["VGA", "HD720", "HD1080"],
                        help="Camera resolution (default: VGA)")
    parser.add_argument("--max-depth", type=float, default=8.0,
                        help="Max depth in metres to display (default: 8)")
    args = parser.parse_args()

    # --- Rerun ---
    rr.init("zed_raw", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # --- ZED init ---
    zed = sl.Camera()
    p = sl.InitParameters()
    p.camera_resolution = getattr(sl.RESOLUTION, args.resolution)
    p.camera_fps = 15
    p.depth_mode = sl.DEPTH_MODE.NEURAL
    p.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    p.coordinate_units = sl.UNIT.METER

    print(f"Opening ZED ({args.resolution}, NEURAL depth) ...")
    err = zed.open(p)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED failed to open: {err}")
    print("ZED opened OK")

    runtime = sl.RuntimeParameters()
    pc_mat = sl.Mat()
    img_mat = sl.Mat()

    frame = 0
    t0 = time.monotonic()

    try:
        while True:
            if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.001)
                continue

            # --- Native point cloud (X Y Z RGBA packed as float32) ---
            zed.retrieve_measure(pc_mat, sl.MEASURE.XYZRGBA)
            raw = pc_mat.get_data()            # (H, W, 4) float32
            pts4 = raw.reshape(-1, 4)

            finite = np.isfinite(pts4[:, 0]) & np.isfinite(pts4[:, 1]) & np.isfinite(pts4[:, 2])
            near   = pts4[:, 2] < args.max_depth   # ZED Z_UP_X_FWD: Z = up, depth is X
            # For RIGHT_HANDED_Z_UP_X_FWD the forward axis is X, not Z.
            # Filter on distance from origin instead.
            dist = np.linalg.norm(pts4[:, :3], axis=1)
            keep = finite & (dist < args.max_depth) & (dist > 0.15)
            pts4 = pts4[keep]

            xyz = pts4[:, :3].astype(np.float32)

            # Unpack RGBA from float32 bit-pattern
            rgba_bits = pts4[:, 3].view(np.uint32)
            r = ((rgba_bits >> 16) & 0xFF).astype(np.uint8)
            g = ((rgba_bits >> 8)  & 0xFF).astype(np.uint8)
            b = ( rgba_bits        & 0xFF).astype(np.uint8)
            colors = np.column_stack([r, g, b])

            rr.log("world/pointcloud", rr.Points3D(positions=xyz, colors=colors))

            # --- Colour image ---
            zed.retrieve_image(img_mat, sl.VIEW.LEFT)
            img = img_mat.get_data()[:, :, :3]           # drop alpha, BGR
            img = img[:, :, ::-1].copy()                 # → RGB
            rr.log("world/color", rr.Image(img))

            frame += 1
            if frame % 30 == 0:
                elapsed = time.monotonic() - t0
                print(f"frame {frame}  pts={len(xyz)}  fps={frame/elapsed:.1f}")

    except KeyboardInterrupt:
        pass
    finally:
        zed.close()
        print("ZED closed")


if __name__ == "__main__":
    main()
