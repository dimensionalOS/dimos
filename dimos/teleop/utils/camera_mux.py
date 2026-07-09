# Copyright 2026 Dimensional Inc.
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

"""Operator-facing camera mux for hosted teleop modules.

Mixin over a ``Module`` that owns a ``mux_image: Out[Image]`` port. Collects
the latest frame per named camera, composites the operator-selected subset
(single frame passthrough, or hstack scaled to the shortest tile), applies
publish-side width/fps caps, and optionally appends the latency-stamp strip.

Extracted from ``Go2HostedConnection`` so arm stations can reuse it;
the stamp cell constants MUST stay in sync with webrtc.js readLatencyStamp.
"""

from __future__ import annotations

import threading
import time

import numpy as np

from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Frame-embedded capture time for glass-to-glass latency (see module docstring).
# MSB-first: SYNC then time. Constants MUST match webrtc.js readLatencyStamp.
_STAMP_CELL_PX = 16  # cell width — big enough to survive H.264 compression
_STAMP_STRIP_PX = 16  # height of the appended timestamp band, in rows
_STAMP_SYNC = (1, 0, 1, 0)  # both sides must agree
_STAMP_TIME_BITS = 44  # ms since epoch (~41 bits) + headroom
_STAMP_CELLS = len(_STAMP_SYNC) + _STAMP_TIME_BITS


class CameraMuxMixin:
    """Camera mux for hosted teleop: latest-frame cache → composite → mux_image.

    Host module contract:
      - ``mux_image: Out[Image]`` port declared on the host.
      - config provides ``video_max_width: int``, ``video_max_fps: float``,
        ``latency_stamp: bool`` (0/0.0/False = off).
      - host calls ``_mux_init(cameras)`` in ``__init__`` with the ordered
        camera names (e.g. ``["cam1", "cam2"]``; first entry is the boot
        default selection), and subscribes each camera stream to
        ``lambda img: self._on_cam("cam1", img)``.
    """

    def _mux_init(self, cameras: list[str]) -> None:
        """Set up mux state: known camera order, default selection = first cam."""
        self._cam_order: list[str] = list(cameras)
        self._cam_lock = threading.Lock()
        self._cam_frames: dict[str, Image] = {}
        self._cam_selected: list[str] = self._cam_order[:1]
        self._last_mux_pub = 0.0  # monotonic stamp for the video_max_fps cap

    def _on_cam(self, cam: str, img: Image) -> None:
        """Cache the latest frame; if `cam` is selected, fps-cap then publish."""
        with self._cam_lock:
            self._cam_frames[cam] = img
            shown = cam in self._cam_selected
        if not shown:
            return
        # FPS cap before any mux/encode work — skipping here is nearly free.
        max_fps = self.config.video_max_fps  # type: ignore[attr-defined]
        if max_fps > 0:
            now = time.monotonic()
            if now - self._last_mux_pub < 1.0 / max_fps:
                return
            self._last_mux_pub = now
        out = self._composite()
        if out is not None:
            self.mux_image.publish(out)  # type: ignore[attr-defined]

    def _composite(self) -> Image | None:
        """Selected frames → one Image: single passthrough, else hstack scaled to
        min height. None if nothing cached yet. Always even-sized (see _even_dims)."""
        with self._cam_lock:
            order = [c for c in self._cam_order if c in self._cam_selected]
            imgs = [self._cam_frames[c] for c in order if c in self._cam_frames]
        if not imgs:
            return None
        if len(imgs) == 1:
            return self._even_dims(self._stamp(self._downscale(imgs[0])))
        import cv2

        target_h = min(im.data.shape[0] for im in imgs)
        tiles = []
        for im in imgs:
            h, w = im.data.shape[:2]
            tiles.append(
                cv2.resize(im.data, (max(1, int(w * target_h / h)), target_h))
                if h != target_h
                else im.data
            )
        return self._even_dims(
            self._stamp(
                self._downscale(
                    Image(data=np.hstack(tiles), format=imgs[0].format, frame_id="camera_mux")
                )
            )
        )

    @staticmethod
    def _even_dims(img: Image) -> Image:
        """Crop to even width/height — an odd composite crashes libx264's
        avcodec_open2 when a camera switch reopens the encoder."""
        data = img.data
        if data.ndim < 2:
            return img
        h, w = data.shape[:2]
        if h % 2 == 0 and w % 2 == 0:
            return img
        data = data[: h - (h % 2), : w - (w % 2)]
        return Image(data=np.ascontiguousarray(data), format=img.format, frame_id=img.frame_id)

    def _downscale(self, img: Image) -> Image:
        """Cap publish width at config.video_max_width (0 = off). Runs before
        _stamp so the strip's 16px cells stay decodable at the sent size."""
        max_w = self.config.video_max_width  # type: ignore[attr-defined]
        if max_w <= 0 or img.data.ndim < 2:
            return img
        h, w = img.data.shape[:2]
        if w <= max_w:
            return img
        import cv2

        out = cv2.resize(img.data, (max_w, max(1, int(h * max_w / w))))
        return Image(data=out, format=img.format, frame_id=img.frame_id)

    def _stamp(self, img: Image) -> Image:
        """Append (not overwrite) a bottom strip encoding capture time as B/W
        cells; the operator reads then crops it. No-op unless latency_stamp."""
        if not self.config.latency_stamp:  # type: ignore[attr-defined]
            return img

        ms = int(time.time() * 1000)
        bits = list(_STAMP_SYNC) + [
            (ms >> (_STAMP_TIME_BITS - 1 - i)) & 1 for i in range(_STAMP_TIME_BITS)
        ]

        s = _STAMP_CELL_PX
        data = img.data
        if data.ndim < 2 or data.shape[1] < _STAMP_CELLS * s:
            return img

        # Build the strip (black), paint cells across it, then stack below.
        strip_shape = (_STAMP_STRIP_PX, data.shape[1], *data.shape[2:])
        strip = np.zeros(strip_shape, dtype=data.dtype)
        for i, bit in enumerate(bits):
            if bit:
                strip[:, i * s : (i + 1) * s] = 255
        out = np.vstack([data, strip])
        return Image(data=out, format=img.format, frame_id=img.frame_id)

    def _set_cam_selection(self, cams: list[str]) -> None:
        """camera_select: filter to known names (fallback first cam), then
        republish immediately so the view flips without waiting for a frame."""
        sel = [c for c in cams if c in self._cam_order] or self._cam_order[:1]
        with self._cam_lock:
            self._cam_selected = sel
        logger.info("camera selection → %s", sel)
        out = self._composite()
        if out is not None:
            self.mux_image.publish(out)  # type: ignore[attr-defined]

    def _mux_state(self) -> list[str]:
        """Current selection, for the telemetry payload."""
        with self._cam_lock:
            return list(self._cam_selected)
