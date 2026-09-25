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

"""Synthetic H.265 clip for tests and gates that have no camera.

A white square walks across a grey frame so decoded pixels can be checked per frame.
Stands in for a real capture; the RtspCamera replay path treats a file like the RTSP
stream apart from the transport and the mp4 framing.
"""

from __future__ import annotations

from pathlib import Path

import av
import numpy as np

SQUARE = 24


def square_origin(index: int, width: int, height: int, centered: bool = False) -> tuple[int, int]:
    """Top-left corner of the white square in frame ``index`` (walking, or parked at centre)."""
    x = width // 2 - SQUARE // 2 if centered else (index * 8) % max(1, width - SQUARE)
    y = height // 2 - SQUARE // 2
    return x, y


def write_synthetic_h265(
    path: Path,
    *,
    width: int = 320,
    height: int = 180,
    fps: int = 25,
    seconds: float = 2.0,
    centered: bool = False,
) -> int:
    """Encode the clip with libx265 to ``path`` (.mp4). Returns the frame count."""
    n = round(fps * seconds)
    with av.open(str(path), mode="w") as container:
        stream = container.add_stream("libx265", rate=fps)
        assert isinstance(stream, av.VideoStream)
        stream.width = width
        stream.height = height
        stream.pix_fmt = "yuv420p"
        # No B-frames, like a live camera: decode order is display order.
        stream.options = {
            "preset": "ultrafast",
            "x265-params": "log-level=none:bframes=0",
            "crf": "20",
        }
        for i in range(n):
            img = np.full((height, width, 3), 96, dtype=np.uint8)
            x, y = square_origin(i, width, height, centered)
            img[y : y + SQUARE, x : x + SQUARE] = 255
            frame = av.VideoFrame.from_ndarray(img, format="rgb24")
            frame.pts = i
            for packet in stream.encode(frame):
                container.mux(packet)
        for packet in stream.encode():
            container.mux(packet)
    return n
