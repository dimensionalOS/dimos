"""ZED Mini → dense point cloud + voxel grid for navigation.

Pipeline per frame
------------------
1. Grab ZED depth (stereo triangulation, NEURAL mode) + colour image.
2. Back-project every valid depth pixel into 3-D:
       X = (u - cx) * depth / fx
       Y = (v - cy) * depth / fy
       Z = depth
3. Voxelise: quantise XYZ by voxel_size, deduplicate, accumulate.
4. Stream to Rerun: colour image, depth (normalised, stable range),
   dense RGB point cloud, accumulated voxel grid coloured by height.

The voxel grid is what a navigation stack consumes — each occupied voxel
represents a region of space the robot must avoid.

Usage:
    python -m dimos.navigation.camera_nav.run_zed_raw
    python -m dimos.navigation.camera_nav.run_zed_raw --stride 1   # full density
    python -m dimos.navigation.camera_nav.run_zed_raw --voxel 0.10 # 10 cm voxels
"""
from __future__ import annotations

import argparse
import time
from collections import deque

import numpy as np
import pyzed.sl as sl
import rerun as rr


# Height bands in camera frame (Y axis, positive = downward).
# Used to colour voxels for navigation: floor / obstacle / overhead.
_FLOOR_Y    =  0.5   # metres below camera — below this line is floor
_OBSTACLE_Y = -0.3   # metres above camera — above this is ceiling


def _height_colors(xyz: np.ndarray) -> np.ndarray:
    """Colour voxels by height (Y in camera frame, positive = down).

    green  = floor   (safe, robot rolls over it)
    red    = obstacle (robot height band — must avoid)
    blue   = overhead (ceiling, high shelves — ignore for ground nav)
    """
    y = xyz[:, 1]
    colors = np.empty((len(xyz), 3), dtype=np.uint8)
    colors[:] = [220, 50, 50]          # default: obstacle (red)
    colors[y >  _FLOOR_Y]    = [60, 180, 60]   # floor   (green)
    colors[y < _OBSTACLE_Y]  = [50, 100, 220]   # overhead (blue)
    return colors


def main() -> None:
    parser = argparse.ArgumentParser(description="ZED Mini voxel-grid navigation viewer")
    parser.add_argument("--resolution", default="VGA",
                        choices=["VGA", "HD720", "HD1080"])
    parser.add_argument("--max-depth", type=float, default=6.0,
                        help="Clip depth beyond this metres (default: 6)")
    parser.add_argument("--stride", type=int, default=2,
                        help="Sample every Nth pixel (default: 2). "
                             "1=full density, 4=lightweight")
    parser.add_argument("--voxel", type=float, default=0.05,
                        help="Voxel size in metres (default: 0.05 = 5 cm)")
    parser.add_argument("--history", type=int, default=60,
                        help="Number of frames to keep in the voxel map (default: 60)")
    args = parser.parse_args()

    # --- Rerun ---
    rr.init("zed_nav", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # --- Open ZED ---
    zed = sl.Camera()
    init_p = sl.InitParameters()
    init_p.camera_resolution      = getattr(sl.RESOLUTION, args.resolution)
    init_p.camera_fps             = 15
    init_p.depth_mode             = sl.DEPTH_MODE.NEURAL
    init_p.coordinate_system      = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init_p.coordinate_units       = sl.UNIT.METER
    init_p.depth_maximum_distance = args.max_depth

    print(f"Opening ZED ({args.resolution}, NEURAL, stride={args.stride}, "
          f"voxel={args.voxel}m) …")
    err = zed.open(init_p)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED failed to open: {err}")
    print("ZED opened  —  Ctrl-C to quit")

    # --- Intrinsics ---
    cam_info = zed.get_camera_information()
    cal = cam_info.camera_configuration.calibration_parameters.left_cam
    W, H = (cam_info.camera_configuration.resolution.width,
             cam_info.camera_configuration.resolution.height)
    fx, fy, cx, cy = cal.fx, cal.fy, cal.cx, cal.cy

    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    rr.log("world/camera", rr.Pinhole(image_from_camera=K, width=W, height=H), static=True)

    # Pre-build pixel ray directions (constant per session).
    s = args.stride
    us = np.arange(0, W, s, dtype=np.float32)
    vs = np.arange(0, H, s, dtype=np.float32)
    uu, vv = np.meshgrid(us, vs)
    ray_x = (uu - cx) / fx    # (H/s, W/s)
    ray_y = (vv - cy) / fy

    # Rolling voxel map: deque of per-frame voxel-coord arrays.
    # When the deque is full the oldest frame's voxels are dropped.
    v = args.voxel
    frame_voxels: deque[np.ndarray] = deque(maxlen=args.history)

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

            # ── Colour image ──────────────────────────────────────────────
            zed.retrieve_image(img_mat, sl.VIEW.LEFT)
            img_bgra = img_mat.get_data()
            img_rgb  = img_bgra[:, :, 2::-1].copy()      # BGRA → RGB
            rr.log("world/camera/color", rr.Image(img_rgb))

            # ── Depth map ────────────────────────────────────────────────
            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
            depth = depth_mat.get_data()
            if depth.ndim == 3:
                depth = depth[:, :, 0]
            depth = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)

            # Normalise to fixed 0→max_depth range so the colour scale
            # is stable across frames (no auto-scaling heatmap jumping).
            depth_u8 = (depth / args.max_depth * 255).clip(0, 255).astype(np.uint8)
            rr.log("world/camera/depth", rr.Image(depth_u8))

            # ── Dense point cloud (RGB, no heatmap) ──────────────────────
            d_sub  = depth[::s, ::s]
            valid  = (d_sub > 0.15) & (d_sub < args.max_depth)
            d      = d_sub[valid]

            X = ray_x[valid] * d
            Y = ray_y[valid] * d
            Z = d
            xyz = np.column_stack([X, Y, Z]).astype(np.float32)

            col_sub = img_rgb[::s, ::s, :]
            colors  = col_sub.reshape(-1, 3)[valid.ravel()]
            rr.log("world/pointcloud", rr.Points3D(positions=xyz, colors=colors))

            # ── Voxel grid ───────────────────────────────────────────────
            if len(xyz) > 0:
                # Quantise to integer voxel coordinates.
                vcoords = np.floor(xyz / v).astype(np.int32)     # (N, 3)

                # Deduplicate within this frame using numpy unique.
                _, idx = np.unique(vcoords, axis=0, return_index=True)
                frame_voxels.append(vcoords[idx])

                # Merge all frames in the rolling window.
                all_vcoords = np.concatenate(list(frame_voxels), axis=0)
                all_vcoords = np.unique(all_vcoords, axis=0)

                # Convert back to world-space centres and colour by height.
                centres = (all_vcoords.astype(np.float32) + 0.5) * v
                vcols   = _height_colors(centres)

                rr.log("world/voxels",
                       rr.Points3D(positions=centres,
                                   colors=vcols,
                                   radii=v * 0.45))   # near-cube appearance

            frame += 1
            if frame % 30 == 0:
                n_vox = len(all_vcoords) if len(xyz) > 0 else 0
                fps   = frame / (time.monotonic() - t0)
                print(f"frame {frame:5d}  pts={len(xyz):6d}  "
                      f"voxels={n_vox:6d}  fps={fps:.1f}")

    except KeyboardInterrupt:
        pass
    finally:
        zed.close()
        print("ZED closed")


if __name__ == "__main__":
    main()
