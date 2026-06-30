"""RealSense depth source + entry point for the depth pipeline.

Usage:
  python -m dimos.navigation.camera_nav.realsense_depth_map

Camera-specific parts (pyrealsense2 pipeline, depth scale, align) are here.
The pipeline (gradient filter → backproject → voxel map → Rerun) lives in
depth_map.py and is camera-agnostic.

No VIO: RealSense has no built-in positional tracking. pose_locked is always
True and the map accumulates in the camera's starting frame. Add an external
pose source (T265, ORB-SLAM3, etc.) to build a proper world-frame map.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import pyrealsense2 as rs

from dimos.navigation.camera_nav.depth_map import (
    DepthBackprojector,
    DepthFramePacket,
    DepthStreamer,
    GradientStabilityConfig,
    GradientStabilityFilter,
    init_rerun,
)


# ── RealSense depth source ───────────────────────────────────────────────────

class RealSenseDepthSource:
    """pyrealsense2 → DepthFramePacket.

    Depth is aligned to the color frame so both share the same intrinsics and
    DepthBackprojector can use the color K matrix for pinhole backprojection.
    No SDK-native XYZ: xyz_cam_hw is always None, so backprojection uses the
    standard pinhole fallback path (optical frame → camera_link via _R_OPT_TO_LINK).
    """

    MIN_DEPTH: float = 0.3
    MAX_DEPTH: float = 8.0

    def __init__(
        self,
        width: int = 848,
        height: int = 480,
        fps: int = 15,
    ) -> None:
        import pyrealsense2 as rs

        self._width  = width
        self._height = height
        self._fps    = fps

        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16,  fps)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        self._pipeline = rs.pipeline()
        profile        = self._pipeline.start(cfg)

        depth_sensor      = profile.get_device().first_depth_sensor()
        self._depth_scale = depth_sensor.get_depth_scale()

        self._align = rs.align(rs.stream.color)

        # Intrinsics from color stream (depth is aligned to it)
        color_profile  = profile.get_stream(rs.stream.color)
        intr           = color_profile.as_video_stream_profile().get_intrinsics()
        self._intrinsics = np.array(
            [intr.fx, intr.fy, intr.ppx, intr.ppy], dtype=np.float64
        )

        # Identity pose: map builds in starting-camera frame.
        self._pose_R = np.eye(3, dtype=np.float32)
        self._pose_t = np.zeros(3, dtype=np.float32)

    @property
    def pose_locked(self) -> bool:
        return True   # always "locked" — no VIO, camera frame is the world frame

    def read(self, ts: float) -> DepthFramePacket:
        frames        = self._pipeline.wait_for_frames()
        aligned       = self._align.process(frames)
        depth_frame   = aligned.get_depth_frame()
        color_frame   = aligned.get_color_frame()

        # z16 → float32 metres; 0 = no measurement → NaN
        depth = np.asarray(depth_frame.get_data(), dtype=np.float32) * self._depth_scale
        invalid = (depth <= 0) | (depth < self.MIN_DEPTH) | (depth > self.MAX_DEPTH)
        depth[invalid] = np.nan

        # BGR → RGB
        bgr       = np.asarray(color_frame.get_data(), dtype=np.uint8)
        colors_hw = bgr[:, :, ::-1].copy()

        return DepthFramePacket(
            timestamp  = ts,
            depth      = depth,
            intrinsics = self._intrinsics,
            width      = self._width,
            height     = self._height,
            pose_R     = self._pose_R,
            pose_t     = self._pose_t,
            xyz_cam_hw = None,     # backprojector uses pinhole fallback
            colors_hw  = colors_hw,
        )

    def stop(self) -> None:
        self._pipeline.stop()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    init_rerun("realsense_depth_map")

    src = RealSenseDepthSource(width=848, height=480, fps=15)
    print("RealSense open — Ctrl-C to quit.")

    streamer = DepthStreamer(
        src,
        DepthBackprojector(),
        GradientStabilityFilter(GradientStabilityConfig(gradient_threshold=0.30)),
    )

    frame = 0
    t0    = time.monotonic()

    try:
        while True:
            ts  = time.monotonic()
            pkt = streamer.assemble(ts)
            streamer.process(pkt, frame)
            streamer.log_stdout(pkt, frame, frame / max(ts - t0, 1e-6),
                                n_valid=streamer._last_n_valid,
                                n_stable=streamer._last_n_stable)
            frame += 1
    except KeyboardInterrupt:
        pass

    src.stop()
    print("done")


if __name__ == "__main__":
    main()
