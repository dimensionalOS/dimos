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

"""Periodically upload the 2D occupancy map (global_costmap) to the robomoo app.

Subscribes to `global_costmap`, renders it to a PNG with the existing
`turbo_image` colormap (same look as the command-center), throttles, and POSTs
it plus grid metadata (resolution, origin, width, height) to robomoo's
`/api/robot/map`. The web app overlays capture pins on it via
`col = (x - originX) / resolution`, `row = (y - originY) / resolution`.

Env: ROBOMOO_URL, ROBOT_INGEST_TOKEN.
"""

import os
import time

import cv2
import httpx

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.mapping.occupancy.visualizations import turbo_image
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MapUploaderConfig(ModuleConfig):
    robomoo_url: str = os.getenv("ROBOMOO_URL", "")
    ingest_token: str = os.getenv("ROBOT_INGEST_TOKEN", "")
    min_period_s: float = 5.0  # throttle: at most one upload every N seconds


class MapUploader(Module):
    config: MapUploaderConfig
    global_costmap: In[OccupancyGrid]

    @rpc
    def start(self) -> None:
        super().start()
        self._last = 0.0
        self.global_costmap.subscribe(self._on_costmap)

    def _on_costmap(self, grid: OccupancyGrid) -> None:
        now = time.monotonic()
        if now - self._last < self.config.min_period_s:
            return
        url = self.config.robomoo_url
        token = self.config.ingest_token
        if not url or not token:
            return
        self._last = now

        try:
            bgr = turbo_image(grid.grid)  # (H, W, 3) uint8
            ok, buf = cv2.imencode(".png", bgr)
            if not ok:
                return
            httpx.post(
                f"{url.rstrip('/')}/api/robot/map",
                headers={"Authorization": f"Bearer {token}"},
                files={"file": ("map.png", buf.tobytes(), "image/png")},
                data={
                    "resolution": str(grid.resolution),
                    "originX": str(grid.origin.position.x),
                    "originY": str(grid.origin.position.y),
                    "width": str(grid.width),
                    "height": str(grid.height),
                },
                timeout=30.0,
            ).raise_for_status()
        except Exception as e:  # noqa: BLE001 — best-effort; never break the stream
            logger.warning("map upload failed: %s", e)
