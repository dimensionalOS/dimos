"""Step 1 of 8: Raw XYZRGBA input — receive, count, log. Nothing else.

Prints per-frame:
  - timestamp
  - total pixels in the XYZRGBA grid
  - how many are valid (finite, not NaN/Inf)
  - instantaneous FPS

Visualizes the raw cloud in Rerun with real RGB colours. No filtering,
no transformation, no modification of any kind.

Usage:
    python -m dimos.navigation.camera_nav.zed_step1_raw
"""
from __future__ import annotations

import time

import numpy as np
import pyzed.sl as sl
import rerun as rr


def main() -> None:
    rr.init("zed_step1_raw", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    # ── Open ZED ─────────────────────────────────────────────────────────────
    zed = sl.Camera()
    ip  = sl.InitParameters()
    ip.depth_mode        = sl.DEPTH_MODE.NEURAL
    ip.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    ip.coordinate_units  = sl.UNIT.METER
    ip.camera_fps        = 15
    ip.camera_resolution = sl.RESOLUTION.VGA

    print("Opening ZED …")
    if zed.open(ip) != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError("ZED failed to open")

    # Positional tracking — required for XYZRGBA to be in world frame.
    # Without this the points are in camera-optical frame (Z forward).
    tp = sl.PositionalTrackingParameters()
    tp.enable_imu_fusion   = True
    tp.set_floor_as_origin = True
    zed.enable_positional_tracking(tp)

    print("ZED open. Waiting for tracking to initialise …")
    print(f"{'frame':>6}  {'timestamp':>14}  {'total_px':>10}  {'valid_pts':>10}  {'fps':>6}")
    print("-" * 60)

    rt     = sl.RuntimeParameters()
    pc_mat = sl.Mat()

    frame  = 0
    t_prev = time.monotonic()

    try:
        while True:
            if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.001)
                continue

            ts    = time.time()
            t_now = time.monotonic()
            fps   = 1.0 / max(t_now - t_prev, 1e-6)
            t_prev = t_now

            # Raw XYZRGBA: shape (H, W, 4), dtype float32.
            # Channel layout: [X, Y, Z, packed_RGBA_as_float32]
            # When tracking is OK: X/Y/Z are in WORLD frame (VIO applied by SDK).
            # When tracking is SEARCHING: still camera frame — check state below.
            zed.retrieve_measure(pc_mat, sl.MEASURE.XYZRGBA)
            raw = pc_mat.get_data()          # (H, W, 4) float32

            total_pixels = raw.shape[0] * raw.shape[1]
            flat = raw.reshape(-1, 4)
            valid_mask = (
                np.isfinite(flat[:, 0]) &
                np.isfinite(flat[:, 1]) &
                np.isfinite(flat[:, 2])
            )
            valid = flat[valid_mask]

            # Tracking state — tells us which frame the points are in.
            pose  = sl.Pose()
            state = zed.get_position(pose, sl.REFERENCE_FRAME.WORLD)
            state_str = "WORLD" if state == sl.POSITIONAL_TRACKING_STATE.OK else f"CAM({state})"

            print(
                f"{frame:>6}  {ts:>14.3f}  {total_pixels:>10}  "
                f"{len(valid):>10}  {fps:>6.1f}  [{state_str}]"
            )

            # ── Rerun: raw cloud, real RGB, untouched ─────────────────────────
            if len(valid) > 0:
                xyz  = valid[:, :3]
                rgba = valid[:, 3].view(np.uint32)
                r    = ((rgba >> 16) & 0xFF).astype(np.uint8)
                g    = ((rgba >>  8) & 0xFF).astype(np.uint8)
                b    = ( rgba        & 0xFF).astype(np.uint8)
                rr.log(
                    "world/raw_cloud",
                    rr.Points3D(positions=xyz, colors=np.column_stack([r, g, b])),
                )

            frame += 1

    except KeyboardInterrupt:
        pass
    finally:
        zed.disable_positional_tracking()
        zed.close()
        print("\nZED closed.")


if __name__ == "__main__":
    main()
