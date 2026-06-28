"""ZED Mini → filtered depth frame packets for DimOS costmap generation.

Streams one DepthFramePacket per camera frame.  Each packet contains
everything DimOS needs to project depth into 3-D, fuse over time with
pose, and build a navigation costmap — without receiving a point cloud.

  DepthFramePacket
    timestamp   float       monotonic seconds
    depth       H×W f32    metres, NaN = invalid / uncertain / out-of-range
    intrinsics  (fx,fy,cx,cy)
    width, height
    pose_R      3×3 f32    camera→world rotation   (identity until tracking fixed)
    pose_t      (3,) f32   camera→world translation
    confidence  H×W f32    0-100 per-pixel score, or None if not requested

DimOS responsibilities (NOT done here):
  depth → 3-D projection using intrinsics + pose
  occupancy grid generation
  temporal fusion using pose
  costmap building

Pipeline classes
  DepthFramePacket   plain dataclass — the packet contract
  DepthFilter        filters raw ZED depth in-place (no downsampling)
  IntrinsicsReader   reads + caches ZED left-camera calibration
  PoseReader         ZED VIO pose (passthrough until SDK 5.4.0 is fixed)
  DepthStreamer      assembles packets, streams to Rerun + stdout

Usage
  python -m dimos.navigation.camera_nav.zed_depth_costmap
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
import pyzed.sl as sl
import rerun as rr


# ── Packet contract ────────────────────────────────────────────────────────────

@dataclass
class DepthFramePacket:
    """One complete depth frame ready for DimOS costmap fusion.

    depth is a float32 image where valid pixels carry depth in metres and
    invalid / low-confidence / out-of-range pixels are NaN.  DimOS uses
    np.isfinite(depth) to locate valid pixels before projection.
    """
    timestamp:  float                       # time.monotonic() at grab
    depth:      np.ndarray                  # H×W float32, metres, NaN=invalid
    intrinsics: np.ndarray                  # [fx, fy, cx, cy] float64
    width:      int
    height:     int
    pose_R:     np.ndarray                  # 3×3 float32  camera→world rotation
    pose_t:     np.ndarray                  # (3,) float32 camera→world translation
    confidence: np.ndarray | None = field(default=None)  # H×W float32 0-100

    @property
    def valid_fraction(self) -> float:
        return float(np.isfinite(self.depth).mean())

    @property
    def K(self) -> np.ndarray:
        fx, fy, cx, cy = self.intrinsics
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)


# ── Stage 1: Depth filter ──────────────────────────────────────────────────────

class DepthFilter:
    """Filters raw ZED MEASURE.DEPTH into a clean float32 depth image.

    Invalid pixels (NaN, Inf, ≤ 0), low-confidence pixels, and out-of-range
    pixels are set to NaN.  The pixel grid is kept at full camera resolution —
    no downsampling — so DimOS receives the maximum scene content per frame.

    Confidence threshold is intentionally moderate (40) to remove clear stereo
    failures without stripping valid geometry at edges or low-texture surfaces.
    """

    CONF_THRESHOLD: float = 40.0   # pixels below this score → NaN
    MIN_DEPTH:      float = 0.3    # metres — ignore near-field noise
    MAX_DEPTH:      float = 8.0    # metres — ignore unstable long-range depth

    def __init__(self) -> None:
        self._depth_mat = sl.Mat()
        self._conf_mat  = sl.Mat()

    def filter(self, zed: sl.Camera) -> tuple[np.ndarray, np.ndarray]:
        """Return (depth_f32 H×W, confidence_f32 H×W).

        depth pixels outside valid range or below confidence are NaN.
        confidence is always returned so callers can inspect it.
        """
        # Raw depth (metres, float32, NaN/Inf for invalid stereo matches)
        zed.retrieve_measure(self._depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU)
        depth = self._depth_mat.get_data().astype(np.float32)
        if depth.ndim == 3:
            depth = depth[:, :, 0]

        # Per-pixel stereo confidence (0 = worst, 100 = best)
        zed.retrieve_measure(self._conf_mat, sl.MEASURE.CONFIDENCE, sl.MEM.CPU)
        conf = self._conf_mat.get_data().astype(np.float32)
        if conf.ndim == 3:
            conf = conf[:, :, 0]

        # Build combined invalid mask — these pixels become NaN
        invalid = (
            ~np.isfinite(depth)
            | (depth <= self.MIN_DEPTH)
            | (depth >  self.MAX_DEPTH)
            | ~np.isfinite(conf)
            | (conf < self.CONF_THRESHOLD)
        )
        depth[invalid] = np.nan

        return depth, conf


# ── Stage 2: Intrinsics reader ────────────────────────────────────────────────

class IntrinsicsReader:
    """Reads and caches ZED left-camera calibration parameters.

    Calibration is fixed per session; we read once after open() and reuse.
    """

    def __init__(self) -> None:
        self._fx: float = 0.0
        self._fy: float = 0.0
        self._cx: float = 0.0
        self._cy: float = 0.0
        self._w:  int   = 0
        self._h:  int   = 0
        self._ready = False

    def read(self, zed: sl.Camera) -> None:
        if self._ready:
            return
        ci  = zed.get_camera_information()
        cal = ci.camera_configuration.calibration_parameters.left_cam
        res = ci.camera_configuration.resolution
        self._fx = cal.fx
        self._fy = cal.fy
        self._cx = cal.cx
        self._cy = cal.cy
        self._w  = res.width
        self._h  = res.height
        self._ready = True

    @property
    def intrinsics(self) -> np.ndarray:
        return np.array([self._fx, self._fy, self._cx, self._cy], dtype=np.float64)

    @property
    def width(self) -> int:
        return self._w

    @property
    def height(self) -> int:
        return self._h

    @property
    def K(self) -> np.ndarray:
        return np.array(
            [[self._fx, 0, self._cx],
             [0, self._fy, self._cy],
             [0, 0, 1]], dtype=np.float64
        )


# ── Stage 3: Pose reader ──────────────────────────────────────────────────────

class PoseReader:
    """Returns ZED VIO camera→world pose.

    ZED SDK 5.4.0 on macOS: enable_positional_tracking() segfaults.
    Returns identity (R=I, t=0) until tracking is fixed.
    Call enable(zed) after that SDK issue is resolved — nothing else changes.
    """

    def __init__(self) -> None:
        self._R      = np.eye(3, dtype=np.float32)
        self._t      = np.zeros(3, dtype=np.float32)
        self._active = False
        self._pose   = sl.Pose()

    def enable(self, zed: sl.Camera) -> bool:
        tp = sl.PositionalTrackingParameters()
        tp.enable_imu_fusion = True
        ok = zed.enable_positional_tracking(tp) == sl.ERROR_CODE.SUCCESS
        self._active = ok
        return ok

    def update(self, zed: sl.Camera) -> None:
        if not self._active:
            return
        if zed.get_position(self._pose, sl.REFERENCE_FRAME.WORLD) == sl.POSITIONAL_TRACKING_STATE.OK:
            T       = self._pose.pose_data.get_data().astype(np.float32)
            self._R = T[:3, :3]
            self._t = T[:3, 3]

    @property
    def R(self) -> np.ndarray:
        return self._R

    @property
    def t(self) -> np.ndarray:
        return self._t


# ── Stage 4: Streamer ─────────────────────────────────────────────────────────

class DepthStreamer:
    """Assembles and streams DepthFramePackets.

    Rerun entities:
      world/camera            Pinhole camera model (static)
      world/camera/depth      filtered depth image (DepthImage, colorised)
      world/camera/confidence confidence map as greyscale image
    """

    def __init__(
        self,
        depth_filter:  DepthFilter,
        intrinsics:    IntrinsicsReader,
        pose:          PoseReader,
        emit_confidence: bool = True,
    ) -> None:
        self._filt    = depth_filter
        self._intr    = intrinsics
        self._pose    = pose
        self._emit_c  = emit_confidence
        self._pinhole_logged = False

    def assemble(self, zed: sl.Camera, timestamp: float) -> DepthFramePacket:
        """Build one DepthFramePacket from the current grabbed frame."""
        self._intr.read(zed)
        self._pose.update(zed)
        depth, conf = self._filt.filter(zed)

        return DepthFramePacket(
            timestamp  = timestamp,
            depth      = depth,
            intrinsics = self._intr.intrinsics,
            width      = self._intr.width,
            height     = self._intr.height,
            pose_R     = self._pose.R.copy(),
            pose_t     = self._pose.t.copy(),
            confidence = conf if self._emit_c else None,
        )

    def log_rerun(self, pkt: DepthFramePacket) -> None:
        """Visualise the packet in Rerun."""
        # Log pinhole once (calibration is static)
        if not self._pinhole_logged:
            rr.log(
                "world/camera",
                rr.Pinhole(
                    image_from_camera=pkt.K,
                    width=pkt.width,
                    height=pkt.height,
                ),
                static=True,
            )
            self._pinhole_logged = True

        # Depth image — Rerun colorises float32 depth automatically
        rr.log("world/camera/depth", rr.DepthImage(pkt.depth, meter=1.0))

        # Confidence map as greyscale (0-100 → 0-255)
        if pkt.confidence is not None:
            conf_u8 = np.nan_to_num(pkt.confidence, nan=0.0).clip(0, 100)
            conf_u8 = (conf_u8 * 2.55).astype(np.uint8)
            rr.log("world/camera/confidence", rr.Image(conf_u8))

    def log_stdout(self, pkt: DepthFramePacket, frame: int, fps: float) -> None:
        valid = pkt.valid_fraction
        tx, ty, tz = pkt.pose_t
        print(
            f"frame={frame:5d}  "
            f"valid={valid*100:5.1f}%  "
            f"pose=({tx:.2f},{ty:.2f},{tz:.2f})  "
            f"fps={fps:.1f}"
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    # Fork Rerun BEFORE zed.open() — ZED starts internal capture threads on open();
    # forking after those threads exist is unsafe on macOS and causes segfaults.
    rr.init("zed_depth_costmap", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    zed = sl.Camera()
    ip  = sl.InitParameters()
    ip.camera_resolution      = sl.RESOLUTION.VGA
    ip.camera_fps             = 15
    ip.depth_mode             = sl.DEPTH_MODE.NEURAL
    ip.coordinate_system      = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    ip.coordinate_units       = sl.UNIT.METER
    ip.depth_maximum_distance = 8.0

    if zed.open(ip) != sl.ERROR_CODE.SUCCESS:
        print("ZED failed to open"); return

    print("camera open — streaming depth packets. Ctrl-C to quit.")

    depth_filter = DepthFilter()
    intrinsics   = IntrinsicsReader()
    pose         = PoseReader()
    streamer     = DepthStreamer(depth_filter, intrinsics, pose, emit_confidence=True)

    pose.enable(zed)  # safe: Rerun forked before zed.open(), no fork-after-threads

    rt    = sl.RuntimeParameters()
    frame = 0
    t0    = time.monotonic()

    try:
        while True:
            if zed.grab(rt) != sl.ERROR_CODE.SUCCESS:
                continue

            ts  = time.monotonic()
            pkt = streamer.assemble(zed, ts)
            streamer.log_rerun(pkt)
            streamer.log_stdout(pkt, frame, frame / max(ts - t0, 1e-6))

            # pkt is ready for DimOS here — hand it to the costmap pipeline:
            # costmap_module.receive(pkt)

            frame += 1

    except KeyboardInterrupt:
        pass

    zed.close()
    print("done")


if __name__ == "__main__":
    main()
