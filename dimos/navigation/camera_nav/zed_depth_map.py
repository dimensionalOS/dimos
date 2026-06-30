"""ZED Mini depth source + entry point for the depth pipeline.

Usage:
  python -m dimos.navigation.camera_nav.zed_depth_map

Camera-specific parts (ZED SDK, VIO tracking, native XYZ retrieval) are here.
The pipeline (gradient filter → backproject → voxel map → Rerun) lives in
depth_map.py and is camera-agnostic.
"""

from __future__ import annotations

import time

import numpy as np
import pyzed.sl as sl

from dimos.navigation.camera_nav.depth_map import (
    DepthBackprojector,
    DepthFramePacket,
    DepthStreamer,
    GradientStabilityConfig,
    GradientStabilityFilter,
    init_rerun,
)


# ── ZED intrinsics reader ────────────────────────────────────────────────────

class _IntrinsicsReader:
    """Reads and caches ZED left-camera calibration (fixed for the session)."""

    def __init__(self) -> None:
        self._fx = self._fy = self._cx = self._cy = 0.0
        self._w = self._h = 0
        self._ready = False

    def read(self, zed: sl.Camera) -> None:
        if self._ready:
            return
        ci  = zed.get_camera_information()
        cal = ci.camera_configuration.calibration_parameters.left_cam
        res = ci.camera_configuration.resolution
        self._fx, self._fy = cal.fx, cal.fy
        self._cx, self._cy = cal.cx, cal.cy
        self._w, self._h   = res.width, res.height
        self._ready = True

    @property
    def intrinsics(self) -> np.ndarray:
        return np.array([self._fx, self._fy, self._cx, self._cy], dtype=np.float64)

    @property
    def width(self) -> int: return self._w

    @property
    def height(self) -> int: return self._h


# ── ZED VIO pose reader ──────────────────────────────────────────────────────

class _PoseReader:
    """ZED VIO camera→world pose. Identity passthrough until VIO locks."""

    def __init__(self) -> None:
        self._R      = np.eye(3, dtype=np.float32)
        self._t      = np.zeros(3, dtype=np.float32)
        self._active = False
        self._locked = False
        self._pose   = sl.Pose()
        self._t0     = time.monotonic()
        self._last_warn = 0.0

    def enable(self, zed: sl.Camera) -> bool:
        tp = sl.PositionalTrackingParameters()
        tp.enable_imu_fusion   = True
        tp.set_floor_as_origin = True
        ok = zed.enable_positional_tracking(tp) == sl.ERROR_CODE.SUCCESS
        self._active = ok
        print("VIO tracking enabled" if ok else "VIO tracking failed — running in camera frame")
        return ok

    def update(self, zed: sl.Camera) -> None:
        if not self._active:
            return
        now = time.monotonic()
        if zed.get_position(self._pose, sl.REFERENCE_FRAME.WORLD) == sl.POSITIONAL_TRACKING_STATE.OK:
            t = self._pose.get_translation()
            q = self._pose.get_orientation()
            self._t = np.array(t.get(), dtype=np.float32)
            x, y, z, w = np.array(q.get(), dtype=np.float32)
            self._R = np.array([
                [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)  ],
                [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)  ],
                [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
            ], dtype=np.float32)
            if not self._locked:
                print(f"*** VIO LOCKED after {now - self._t0:.1f}s ***")
            self._locked = True
        else:
            if now - max(self._last_warn, self._t0) >= 15.0:
                print(f"    VIO searching … {now - self._t0:.0f}s elapsed")
                self._last_warn = now

    @property
    def R(self) -> np.ndarray: return self._R

    @property
    def t(self) -> np.ndarray: return self._t

    @property
    def locked(self) -> bool: return self._locked


# ── ZED depth source ─────────────────────────────────────────────────────────

class ZEDDepthSource:
    """ZED SDK → DepthFramePacket. No confidence filtering — raw depth only.

    Uses sl.MEASURE.XYZ for GPU-computed point cloud in camera_link frame
    (RIGHT_HANDED_Z_UP_X_FWD), so DepthBackprojector takes the fast path.
    Quality filtering happens downstream (GradientStabilityFilter, voxel map).
    """

    MIN_DEPTH: float = 0.3
    MAX_DEPTH: float = 8.0

    def __init__(self, zed: sl.Camera) -> None:
        self._zed     = zed
        self._xyz_mat = sl.Mat()
        self._img_mat = sl.Mat()
        self._intr    = _IntrinsicsReader()
        self._pose    = _PoseReader()

    def enable_tracking(self) -> bool:
        return self._pose.enable(self._zed)

    @property
    def pose_locked(self) -> bool:
        return self._pose.locked

    def read(self, ts: float) -> DepthFramePacket:
        self._intr.read(self._zed)
        self._pose.update(self._zed)

        # H×W×4 float32 in camera_link frame: [X=fwd, Y=left, Z=up, pad]
        self._zed.retrieve_measure(self._xyz_mat, sl.MEASURE.XYZ, sl.MEM.CPU)
        xyz_raw = self._xyz_mat.get_data()[:, :, :3].astype(np.float32)

        depth = xyz_raw[:, :, 0].copy()   # X=fwd = forward distance
        invalid = ~np.isfinite(depth) | (depth <= self.MIN_DEPTH) | (depth > self.MAX_DEPTH)
        depth[invalid]   = np.nan
        xyz_raw[invalid] = np.nan

        self._zed.retrieve_image(self._img_mat, sl.VIEW.LEFT, sl.MEM.CPU)
        img       = self._img_mat.get_data()
        colors_hw = img[:, :, 2::-1].copy()   # BGRA → RGB uint8

        return DepthFramePacket(
            timestamp  = ts,
            depth      = depth,
            intrinsics = self._intr.intrinsics,
            width      = self._intr.width,
            height     = self._intr.height,
            pose_R     = self._pose.R.copy(),
            pose_t     = self._pose.t.copy(),
            xyz_cam_hw = xyz_raw,
            colors_hw  = colors_hw,
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    # Spawn Rerun BEFORE zed.open() — ZED capture threads make post-open fork unsafe.
    init_rerun("zed_depth_map")

    zed = sl.Camera()
    ip  = sl.InitParameters()
    ip.camera_resolution      = sl.RESOLUTION.HD720
    ip.camera_fps             = 15
    ip.depth_mode             = sl.DEPTH_MODE.NEURAL
    ip.coordinate_system      = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    ip.coordinate_units       = sl.UNIT.METER
    ip.depth_maximum_distance = 8.0

    if zed.open(ip) != sl.ERROR_CODE.SUCCESS:
        print("ZED failed to open"); return

    # Fixed exposure prevents bright lights from washing out stereo texture.
    # Range 0-100; lower = faster shutter. Tune up if the scene is too dark.
    zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 50)
    print("camera open — Ctrl-C to quit.")

    src = ZEDDepthSource(zed)
    src.enable_tracking()

    streamer = DepthStreamer(
        src,
        DepthBackprojector(),
        GradientStabilityFilter(GradientStabilityConfig(gradient_threshold=0.30)),
    )

    rt = sl.RuntimeParameters()
    rt.texture_confidence_threshold = 0     # no SDK-level texture gate; pipeline handles quality
    rt.remove_saturated_areas       = True  # overexposed pixels have no valid stereo signal

    frame = 0
    t0    = time.monotonic()

    try:
        while True:
            if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
                continue
            ts  = time.monotonic()
            pkt = streamer.assemble(ts)
            streamer.process(pkt, frame)
            streamer.log_stdout(pkt, frame, frame / max(ts - t0, 1e-6),
                                n_valid=streamer._last_n_valid,
                                n_stable=streamer._last_n_stable)
            frame += 1
    except KeyboardInterrupt:
        pass

    zed.close()
    print("done")


if __name__ == "__main__":
    main()
