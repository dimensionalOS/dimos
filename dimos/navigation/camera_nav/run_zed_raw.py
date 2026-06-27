"""ZED Mini → live point cloud + growing world map.

Uses ZED's native NEURAL stereo depth + VIO positional tracking.
All coordinate math is done inside the ZED SDK — no TF, no dimos pipeline.

XYZRGBA gives points already in world frame (X=forward, Y=left, Z=up,
floor=Z=0 when set_floor_as_origin=True).  We voxelise them and accumulate
across frames so the map grows as the camera moves.

Rerun entities:
  world/cloud   current frame RGB point cloud (updates every frame)
  world/map     growing accumulated voxel map, height-coloured
  world/depth   stable depth image (fixed 0→max_depth range)

Usage:
    python -m dimos.navigation.camera_nav.run_zed_raw
    python -m dimos.navigation.camera_nav.run_zed_raw --voxel 0.07
    python -m dimos.navigation.camera_nav.run_zed_raw --max-depth 6
    python -m dimos.navigation.camera_nav.run_zed_raw --stride 3
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import pyzed.sl as sl
import rerun as rr

# ── Height thresholds in ZED world frame (Z=up, floor=0) ────────────────────
_Z_FLOOR    = 0.25   # below → floor (green)
_Z_OBSTACLE = 1.80   # above → overhead (blue); between → obstacle (red)
_MAX_VOXELS = 400_000


def _height_colors(z: np.ndarray) -> np.ndarray:
    c = np.empty((len(z), 3), dtype=np.uint8)
    c[:] = [210, 60, 60]
    c[z < _Z_FLOOR]    = [60, 200, 80]
    c[z > _Z_OBSTACLE] = [60, 120, 220]
    return c


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--resolution", default="VGA", choices=["VGA", "HD720"])
    ap.add_argument("--max-depth", type=float, default=8.0)
    ap.add_argument("--voxel",  type=float, default=0.07,
                    help="Voxel size in metres for the world map (default 0.07)")
    ap.add_argument("--stride", type=int,   default=2,
                    help="Pixel stride for point cloud (default 2, lower=denser)")
    args = ap.parse_args()

    rr.init("zed_map", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # ── Open ZED ────────────────────────────────────────────────────────────
    zed = sl.Camera()
    ip  = sl.InitParameters()
    ip.camera_resolution      = getattr(sl.RESOLUTION, args.resolution)
    ip.camera_fps             = 15
    ip.depth_mode             = sl.DEPTH_MODE.NEURAL
    ip.coordinate_system      = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    ip.coordinate_units       = sl.UNIT.METER
    ip.depth_maximum_distance = args.max_depth

    print(f"Opening ZED ({args.resolution}, NEURAL depth) …")
    if zed.open(ip) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("ZED failed to open")

    # VIO: IMU-fused, floor as Z=0 so height thresholds are meaningful.
    tp = sl.PositionalTrackingParameters()
    tp.enable_imu_fusion   = True
    tp.set_floor_as_origin = True
    zed.enable_positional_tracking(tp)
    print("Tracking enabled — walk around to build the map. Ctrl-C to quit.\n")

    # Intrinsics for the depth image pinhole overlay.
    ci  = zed.get_camera_information()
    cal = ci.camera_configuration.calibration_parameters.left_cam
    W   = ci.camera_configuration.resolution.width
    H   = ci.camera_configuration.resolution.height
    K   = np.array([[cal.fx, 0, cal.cx],
                    [0, cal.fy, cal.cy],
                    [0, 0, 1]], dtype=np.float64)
    rr.log("world/camera", rr.Pinhole(image_from_camera=K, width=W, height=H), static=True)

    # ── Accumulation state ───────────────────────────────────────────────────
    v = args.voxel
    # all_vkeys: (N, 3) int32 array of unique occupied voxel indices (world frame).
    # Starts empty, grows as new voxels are discovered. Bounded by _MAX_VOXELS.
    all_vkeys: np.ndarray = np.empty((0, 3), dtype=np.int32)

    rt        = sl.RuntimeParameters()
    img_mat   = sl.Mat()
    depth_mat = sl.Mat()
    pc_mat    = sl.Mat()

    frame = 0
    t0    = time.monotonic()

    try:
        while True:
            if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.001)
                continue

            # ── Depth image ──────────────────────────────────────────────────
            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
            depth = depth_mat.get_data()
            if depth.ndim == 3:
                depth = depth[:, :, 0]
            depth_u8 = (
                np.nan_to_num(depth, nan=0.0) / args.max_depth * 255
            ).clip(0, 255).astype(np.uint8)
            rr.log("world/depth", rr.Image(depth_u8))

            # ── Colour image ─────────────────────────────────────────────────
            zed.retrieve_image(img_mat, sl.VIEW.LEFT)
            bgra = img_mat.get_data()
            rgb  = bgra[:, :, 2::-1].copy()
            rr.log("world/camera/color", rr.Image(rgb))

            # ── Point cloud: XYZRGBA already in world frame ──────────────────
            # ZED applies VIO pose internally so every point is in the
            # floor-relative world frame without any extra transform.
            zed.retrieve_measure(pc_mat, sl.MEASURE.XYZRGBA)
            pts4 = pc_mat.get_data()

            # Subsample by stride before any heavy processing.
            pts4 = pts4[::args.stride, ::args.stride, :].reshape(-1, 4)

            dist = np.linalg.norm(pts4[:, :3], axis=1)
            keep = (
                np.isfinite(pts4[:, 0])
                & (dist > 0.2)
                & (dist < args.max_depth)
            )
            pts4 = pts4[keep]

            if len(pts4) == 0:
                frame += 1
                continue

            xyz  = pts4[:, :3].astype(np.float32)
            rgba = pts4[:, 3].view(np.uint32)
            r_ch = ((rgba >> 16) & 0xFF).astype(np.uint8)
            g_ch = ((rgba >>  8) & 0xFF).astype(np.uint8)
            b_ch = ( rgba        & 0xFF).astype(np.uint8)
            rgb_pts = np.column_stack([r_ch, g_ch, b_ch])

            # Current-frame cloud with real RGB from the camera.
            rr.log("world/cloud", rr.Points3D(positions=xyz, colors=rgb_pts))

            # ── Accumulate into world voxel map ──────────────────────────────
            new_vkeys = np.floor(xyz / v).astype(np.int32)
            new_vkeys = np.unique(new_vkeys, axis=0)

            if len(all_vkeys) == 0:
                all_vkeys = new_vkeys
            else:
                # Stack new voxels on top of existing and re-deduplicate.
                # np.unique on int32 arrays is fast (radix sort internally).
                all_vkeys = np.unique(
                    np.vstack([all_vkeys, new_vkeys]), axis=0
                )

            # Hard cap: if we exceed _MAX_VOXELS, coarsen to half resolution.
            if len(all_vkeys) > _MAX_VOXELS:
                all_vkeys = np.unique(all_vkeys >> 1, axis=0)
                v *= 2
                print(f"[map] coarsened voxels → {v:.3f}m  ({len(all_vkeys)} voxels)")

            # Render accumulated map every 5 frames (map doesn't need 15 Hz).
            if frame % 5 == 0:
                centres = (all_vkeys.astype(np.float32) + 0.5) * v
                rr.log(
                    "world/map",
                    rr.Points3D(
                        positions=centres,
                        colors=_height_colors(centres[:, 2]),
                        radii=v * 0.45,
                    ),
                )

            frame += 1
            if frame % 30 == 0:
                fps = frame / (time.monotonic() - t0)
                print(
                    f"frame {frame:5d}  pts={len(xyz):6d}  "
                    f"voxels={len(all_vkeys):6d}  fps={fps:.1f}"
                )

    except KeyboardInterrupt:
        pass
    finally:
        zed.disable_positional_tracking()
        zed.close()
        print("ZED closed.")


if __name__ == "__main__":
    main()
