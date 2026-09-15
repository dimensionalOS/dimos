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

"""``Depth2Depth``: fill the holes in a stereo depth stream from the colour one.

Pairs each colour frame with the depth frame closest in time, runs the fusion
(see ``fusion.py``) and publishes a dense float32 depth image in meters on
``fused_depth``, plus the decision mask on ``kept_raw_mask`` so the fill can be
watched. Anything reading depth can read this stream instead.
"""

from __future__ import annotations

import asyncio
from collections import deque
from typing import TYPE_CHECKING

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.depth2depth.fusion import DEPTH_MODEL_NAME, Depth2Depth as Fuser, FuseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


class Depth2DepthConfig(ModuleConfig):
    """Which checkpoint, and the fusion thresholds (see FuseConfig)."""

    model_name: str = DEPTH_MODEL_NAME
    # "auto" = cuda if available, else cpu. Metal asserts inside the forkserver
    # workers on macOS, so "auto" never picks it; pass "mps" to try anyway.
    device: str = "auto"
    model_height: int = 280
    model_width: int = 504
    near_m: float = 0.3
    far_m: float = 6.0
    ema_new_weight: float = 0.3
    abs_tol: float = 0.3
    rel_tol: float = 0.1
    # A colour frame is fused with a depth frame no further than this away.
    max_pair_dt: float = 0.05
    # Depth frames kept while waiting for their colour frame.
    depth_history: int = 30
    # At most this many frames a second; 0 = every frame that pairs.
    max_hz: float = 0.0


class Depth2Depth(Module):
    """Dense metric depth from the sparse sensor depth and the colour frame."""

    config: Depth2DepthConfig

    color_image: In[Image]
    depth_image: In[Image]

    fused_depth: Out[Image]
    kept_raw_mask: Out[Image]

    fuser: Fuser | None = None

    @rpc
    def start(self) -> None:
        device = self.config.device
        if device == "auto":
            import torch

            device = "cuda" if torch.cuda.is_available() else "cpu"
        self.fuser = Fuser(
            config=FuseConfig(
                model_height=self.config.model_height,
                model_width=self.config.model_width,
                near_m=self.config.near_m,
                far_m=self.config.far_m,
                ema_new_weight=self.config.ema_new_weight,
                abs_tol=self.config.abs_tol,
                rel_tol=self.config.rel_tol,
            ),
            model_name=self.config.model_name,
            device=device,
        )
        logger.info(f"depth2depth: loading {self.config.model_name} on {device}")
        self.fuser.start()
        self._depths: deque[tuple[float, Image]] = deque(maxlen=self.config.depth_history)
        self._last_fused = 0.0
        self.stats = {"color": 0, "depth": 0, "fused": 0, "unpaired": 0}
        super().start()

    @rpc
    def fusion_stats(self) -> dict[str, int]:
        return dict(self.stats)

    async def handle_depth_image(self, image: Image) -> None:
        self.stats["depth"] += 1
        self._depths.append((float(image.ts), image))

    async def handle_color_image(self, image: Image) -> None:
        self.stats["color"] += 1
        if self.fuser is None:
            return
        if self.config.max_hz > 0 and float(image.ts) - self._last_fused < 1.0 / self.config.max_hz:
            return
        depth = self._paired_depth(float(image.ts))
        if depth is None:
            self.stats["unpaired"] += 1
            return
        self._last_fused = float(image.ts)
        # The forward pass blocks for ~70 ms on a GPU and seconds on a CPU; the
        # handler's latest-only dispatch drops the frames that arrive meanwhile.
        await asyncio.get_running_loop().run_in_executor(None, self._fuse, image, depth)

    def _paired_depth(self, ts: float) -> Image | None:
        """The depth frame closest to this colour frame, if one is close enough.
        Depth and colour must be the same moment: a stale colour frame fills the
        holes with geometry from wherever the camera used to point."""
        best, best_dt = None, self.config.max_pair_dt
        for depth_ts, depth in self._depths:
            dt = abs(depth_ts - ts)
            if dt <= best_dt:
                best, best_dt = depth, dt
        return best

    def _fuse(self, color: Image, depth: Image) -> None:
        if self.fuser is None:
            return
        raw = depth_in_meters(depth)
        rgb = np.asarray(color.to_rgb().as_numpy(), dtype=np.uint8)
        if rgb.shape[:2] != raw.shape[:2]:
            self.stats["unpaired"] += 1
            logger.warning(
                f"depth2depth: colour {rgb.shape[:2]} and depth {raw.shape[:2]} differ; "
                "register the depth to the colour camera first"
            )
            return
        fusion = self.fuser.fuse(rgb, raw)
        self.stats["fused"] += 1
        if self.stats["fused"] in (1, 20) or self.stats["fused"] % 100 == 0:
            logger.info(
                f"depth2depth: {self.stats['fused']} frames, "
                f"{fusion.kept_fraction:.0%} sensor, scale {fusion.a:.2f} offset {fusion.b:+.2f} m"
            )
        self.fused_depth.publish(
            Image.from_numpy(
                fusion.fused, format=ImageFormat.DEPTH, frame_id=depth.frame_id, ts=depth.ts
            )
        )
        self.kept_raw_mask.publish(
            Image.from_numpy(
                (fusion.kept_raw * 255).astype(np.uint8),
                format=ImageFormat.GRAY,
                frame_id=depth.frame_id,
                ts=depth.ts,
            )
        )


def depth_in_meters(image: Image) -> NDArray[np.float32]:
    """Depth frames arrive as uint16 millimeters or float32 meters; 0 is a hole."""
    depth = np.asarray(image.as_numpy())
    metres = depth.astype(np.float32) * (0.001 if depth.dtype == np.uint16 else 1.0)
    metres[~np.isfinite(metres)] = 0.0
    return metres
