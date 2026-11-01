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

from collections.abc import Callable
from pathlib import Path

import av
import numpy as np
import pytest


@pytest.fixture(scope="session")
def write_synthetic_h265() -> Callable[..., int]:
    """Writes an H.265 clip to a path and returns its frame count. A white 24 px square
    walks 8 px a frame along the middle row of a grey frame, so decoded pixels can be
    checked per frame. No B-frames, like a live camera."""

    def write(
        path: Path, *, width: int = 320, height: int = 180, fps: int = 25, seconds: float = 2.0
    ) -> int:
        n = round(fps * seconds)
        with av.open(str(path), mode="w") as container:
            stream = container.add_stream("libx265", rate=fps)
            assert isinstance(stream, av.VideoStream)
            stream.width, stream.height, stream.pix_fmt = width, height, "yuv420p"
            stream.options = {
                "preset": "ultrafast",
                "x265-params": "log-level=none:bframes=0",
                "crf": "20",
            }
            for i in range(n):
                img = np.full((height, width, 3), 96, dtype=np.uint8)
                x, y = (i * 8) % (width - 24), height // 2 - 12
                img[y : y + 24, x : x + 24] = 255
                frame = av.VideoFrame.from_ndarray(img, format="rgb24")
                frame.pts = i
                container.mux(stream.encode(frame))
            container.mux(stream.encode())
        return n

    return write
