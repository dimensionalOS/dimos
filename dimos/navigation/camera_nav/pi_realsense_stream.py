"""Lightweight RealSense D435i stream for Raspberry Pi.

No dimos dependency — only pyrealsense2, numpy, scipy, rerun.
Streams live point cloud + depth image to a remote Rerun viewer.

Usage (on Pi):
    python3 pi_realsense_stream.py --host 10.0.0.238
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import pyrealsense2 as rs
import rerun as rr
import rerun.blueprint as rrb
from scipy.ndimage import sobel


# Optical frame (X=right, Y=down, Z=depth) → camera_link (X=fwd, Y=left, Z=up)
_R_OPT_TO_LINK = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32)

_GRADIENT_THRESH = 0.30   # m/pixel — above this → depth edge → skip
_MAX_DEPTH       = 4.5    # metres


def _height_color(z: np.ndarray) -> np.ndarray:
    lo, hi = -1.4, 0.5
    t = np.clip((z - lo) / (hi - lo), 0.0, 1.0)
    r = np.clip(1.5 - np.abs(4 * t - 3), 0.0, 1.0)
    g = np.clip(1.5 - np.abs(4 * t - 2), 0.0, 1.0)
    b = np.clip(1.5 - np.abs(4 * t - 1), 0.0, 1.0)
    return (np.column_stack([r, g, b]) * 255).astype(np.uint8)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="10.0.0.238",
                        help="Laptop IP running the Rerun viewer")
    parser.add_argument("--port", type=int, default=9876)
    parser.add_argument("--width",  type=int, default=848)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps",    type=int, default=15)
    args = parser.parse_args()

    rr.init("pi_realsense")
    rr.connect_tcp(f"{args.host}:{args.port}")
    rr.send_blueprint(rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(name="live cloud", origin="world",
                              contents=["world/cloud", "world/camera/**"]),
            rrb.Spatial2DView(name="depth",      origin="world/camera/depth"),
        )
    ))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    pipeline = rs.pipeline()
    cfg      = rs.config()
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    profile  = pipeline.start(cfg)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale  = depth_sensor.get_depth_scale()

    align  = rs.align(rs.stream.color)
    intr   = (profile.get_stream(rs.stream.depth)
                     .as_video_stream_profile()
                     .get_intrinsics())
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

    rr.log("world/camera", rr.Pinhole(
        image_from_camera=K, width=args.width, height=args.height,
    ), static=True)

    uu, vv = np.meshgrid(np.arange(args.width,  dtype=np.float32),
                         np.arange(args.height, dtype=np.float32))

    print(f"Streaming to {args.host}:{args.port} — Ctrl-C to stop")
    frame = 0
    t0    = time.monotonic()

    try:
        while True:
            frames  = pipeline.wait_for_frames()
            aligned = align.process(frames)

            depth_raw = np.asarray(aligned.get_depth_frame().get_data(), dtype=np.float32) * depth_scale
            bgr       = np.asarray(aligned.get_color_frame().get_data(), dtype=np.uint8)
            rgb       = bgr[:, :, ::-1]

            depth = depth_raw.copy()
            depth[(depth <= 0) | (depth > _MAX_DEPTH)] = np.nan

            # Gradient stability filter — reject depth edges
            valid    = np.isfinite(depth)
            depth_f  = np.where(valid, depth, 0.0).astype(np.float64)
            grad_mag = np.hypot(sobel(depth_f, axis=1), sobel(depth_f, axis=0))
            stable   = valid & (grad_mag < _GRADIENT_THRESH)

            rr.set_time("frame", sequence=frame)
            rr.log("world/camera/depth", rr.DepthImage(depth, meter=1.0))

            # Backproject stable pixels → camera_link frame → log cloud
            dd      = depth[stable]
            xyz_opt = np.column_stack([
                (uu[stable] - cx) * dd / fx,
                (vv[stable] - cy) * dd / fy,
                dd,
            ]).astype(np.float32)
            xyz = xyz_opt @ _R_OPT_TO_LINK.T
            col = rgb[stable]

            n   = min(len(xyz), 50_000)
            idx = np.random.choice(len(xyz), n, replace=False) if len(xyz) > n else np.arange(n)
            rr.log("world/cloud", rr.Points3D(positions=xyz[idx], colors=col[idx], radii=0.003))

            fps = frame / max(time.monotonic() - t0, 1e-6)
            print(f"frame={frame:5d}  pts={len(xyz):6d}  fps={fps:.1f}", flush=True)
            frame += 1

    except KeyboardInterrupt:
        pass

    pipeline.stop()
    print("done")


if __name__ == "__main__":
    main()
