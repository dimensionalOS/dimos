# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""A RealSense camera's colour and depth read straight off V4L2, no vendor SDK.

A D400-series camera exposes colour (YUYV) and depth (Z16) as separate UVC
nodes, but they are not independent. Measured on the R1 Pro's D405s:

* once one of a camera's streams is running, the camera refuses to configure
  another (the kernel logs a stalled UVC probe, -32), so both nodes are set up
  before either starts;
* started depth first, depth never delivers; colour first, both reach full
  rate within a second or two.

So one module owns both nodes of a camera and opens, starts and restarts them
together. Depth values are in the camera's depth unit, a setting stored on the
device (librealsense's ``depth_units``): at its 0.001 m default they are
millimetres, the unit dimos expects of ``DEPTH16``. What librealsense adds on
top -- alignment to colour, filtering, intrinsics -- is not here; depth comes
in the depth imager's own pixel grid.
"""

from __future__ import annotations

import select
import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import Out
from dimos.hardware.sensors.camera.v4l2_capture import V4L2Capture
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class V4L2ColorDepthConfig(ModuleConfig):
    # /dev/v4l/by-path links; on a D405 the colour node is video-index4 and
    # the depth node video-index0 of the same camera.
    color_device: str = "/dev/video4"
    depth_device: str = "/dev/video0"
    color_width: int = 640
    color_height: int = 480
    depth_width: int = 848
    depth_height: int = 480
    fps: float = 30.0
    frame_id: str = "camera_optical"
    # How long to wait before trying again after the camera could not be
    # opened or stopped delivering.
    retry_s: float = 3.0
    # Consecutive seconds with no frame on either stream before both are
    # closed and reopened.
    max_silent_s: float = 3.0
    # Log per-stream rates this often; 0 disables.
    stats_period_s: float = 10.0


class V4L2ColorDepthModule(Module):
    """Publish a depth camera's colour as BGR and its depth as ``DEPTH16``."""

    config: V4L2ColorDepthConfig

    color_out: Out[Image]
    depth_out: Out[Image]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._open_failed = False

    @rpc
    def start(self) -> None:
        super().start()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name=f"V4L2ColorDepth:{self.config.frame_id}"
        )
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        super().stop()

    # ─── capture ────────────────────────────────────────────────────────

    def _run(self) -> None:
        while not self._stop.is_set():
            pair = self._open()
            if pair is None:
                self._stop.wait(self.config.retry_s)
                continue
            try:
                self._pump(*pair)
            finally:
                for cap in pair:
                    cap.release()
            self._stop.wait(self.config.retry_s)

    def _open(self) -> tuple[V4L2Capture, V4L2Capture] | None:
        cfg = self.config
        color = V4L2Capture(
            cfg.color_device, cfg.color_width, cfg.color_height, cfg.fps, "YUYV", start=False
        )
        depth = V4L2Capture(
            cfg.depth_device, cfg.depth_width, cfg.depth_height, cfg.fps, "Z16 ", start=False
        )
        try:
            if color.error or depth.error:
                raise OSError(color.error or depth.error)
            color.start()
            depth.start()
        except OSError as e:
            color.release()
            depth.release()
            if not self._open_failed:
                logger.warning(
                    "cannot open %s (%s): absent, or held by another process such as the "
                    "vendor camera node; retrying every %.0fs",
                    cfg.frame_id,
                    e,
                    cfg.retry_s,
                )
            self._open_failed = True
            return None
        for name, cap, want in (
            ("colour", color, (cfg.color_width, cfg.color_height)),
            ("depth", depth, (cfg.depth_width, cfg.depth_height)),
        ):
            logger.info(
                "opened %s %s at %dx%d @ %.0f fps",
                cfg.frame_id,
                name,
                cap.width,
                cap.height,
                cap.fps,
            )
            if (cap.width, cap.height) != want:
                logger.warning("%s %s: the driver picked the nearest mode", cfg.frame_id, name)
        self._open_failed = False
        return color, depth

    def _color_image(self, frame: Any) -> Image:
        import cv2

        return Image.from_numpy(
            cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUYV),
            format=ImageFormat.BGR,
            frame_id=self.config.frame_id,
            ts=time.time(),
        )

    def _depth_image(self, frame: Any) -> Image:
        return Image.from_numpy(
            frame, format=ImageFormat.DEPTH16, frame_id=self.config.frame_id, ts=time.time()
        )

    def _pump(self, color: V4L2Capture, depth: V4L2Capture) -> None:
        """Read and publish both streams until stopped or either goes quiet."""
        cfg = self.config
        streams = {
            color: (self.color_out, self._color_image, "colour"),
            depth: (self.depth_out, self._depth_image, "depth"),
        }
        now = time.monotonic()
        last_frame = dict.fromkeys(streams, now)
        counts = dict.fromkeys(streams, 0)
        window_t = now
        while not self._stop.is_set():
            ready, _, _ = select.select(list(streams), [], [], 0.5)
            for cap in ready:
                ok, frame = cap.read(timeout_s=0)
                if ok:
                    out, convert, _ = streams[cap]
                    out.publish(convert(frame))
                    last_frame[cap] = time.monotonic()
                    counts[cap] += 1
            now = time.monotonic()
            for cap, (_, _, name) in streams.items():
                if now - last_frame[cap] > cfg.max_silent_s:
                    logger.warning(
                        "%s %s stopped delivering frames; reopening both streams",
                        cfg.frame_id,
                        name,
                    )
                    return
            elapsed = now - window_t
            if elapsed >= cfg.stats_period_s > 0:
                logger.info(
                    "%s: colour %.1f fps, depth %.1f fps",
                    cfg.frame_id,
                    counts[color] / elapsed,
                    counts[depth] / elapsed,
                )
                counts = dict.fromkeys(streams, 0)
                window_t = now
