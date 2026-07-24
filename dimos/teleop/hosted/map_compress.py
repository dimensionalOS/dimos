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

"""Map compressor module: OccupancyGrid → compact PNG payload for the operator.

Throttles, coarsens (block-max), colorizes and PNG-encodes the costmap into a
JSON payload under the 32 KB datachannel budget, plus a compact odom pose so
the operator marker moves between map frames. ``map_out`` binds to a
``CloudflareTransport("map_unreliable")``.
"""

from __future__ import annotations

import base64
import json
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class MapCompressConfig(ModuleConfig):
    map_hz: float = 2.0
    map_min_resolution: float = 0.1
    odom_hz: float = 15.0


class MapCompressModule(Module):
    """Costmap (+ odom) → PNG map payloads on ``map_out`` for the operator."""

    config: MapCompressConfig

    _MAX_MAP_BYTES = 32 * 1024  # ~50% of CF's observed ~64 KB drop threshold

    global_costmap: In[OccupancyGrid]
    odom: In[PoseStamped]
    map_out: Out[bytes]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._last_map_pub = 0.0
        self._last_odom_pub = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        if self.config.map_hz > 0:
            self.register_disposable(Disposable(self.global_costmap.subscribe(self._on_costmap)))
        if self.config.odom_hz > 0:
            self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_costmap(self, grid: OccupancyGrid) -> None:
        """Throttle → coarsen → colorize → PNG → map_out (under the 32 KB cap)."""
        now = time.monotonic()
        if now - self._last_map_pub < 1.0 / self.config.map_hz:
            return

        cells = grid.grid
        if cells is None or cells.size == 0:
            return

        # A bad frame must drop, not kill the RxPY costmap subscription.
        try:
            import cv2

            res = grid.resolution
            img_cells = cells
            if 0 < res < self.config.map_min_resolution:
                factor = max(1, round(self.config.map_min_resolution / res))
                if factor > 1:
                    img_cells = self._block_max(cells, factor)
                    res = res * factor

            ok, buf = cv2.imencode(".png", self._occupancy_to_bgra(img_cells))
            if not ok:
                return
            png_b64 = base64.b64encode(buf.tobytes()).decode("ascii")

            # origin lets the browser place map + robot: cell = (world_xy - origin)/res.
            h, w = img_cells.shape[:2]
            origin = grid.origin.position
            payload = {
                "type": "map",
                "fmt": "png",
                "w": int(w),
                "h": int(h),
                "res": float(res),
                "origin": [float(origin.x), float(origin.y)],
                "stamp": float(grid.ts),
                "png_b64": png_b64,
            }
            data = json.dumps(payload, separators=(",", ":")).encode()
            if len(data) > self._MAX_MAP_BYTES:
                logger.warning("map payload too large (%d bytes), dropping frame", len(data))
                self._last_map_pub = now  # don't retry the same oversized frame immediately
                return
            self.map_out.publish(data)
        except Exception:
            logger.warning("map encode/publish failed, dropping frame", exc_info=True)
            return
        self._last_map_pub = now

    def _on_odom(self, pose: PoseStamped) -> None:
        """Compact 2D pose at odom rate so the marker moves between map frames."""
        now = time.monotonic()
        if now - self._last_odom_pub < 1.0 / self.config.odom_hz:
            return
        try:
            payload = {
                "type": "odom",
                "x": float(pose.position.x),
                "y": float(pose.position.y),
                "yaw": float(pose.orientation.to_euler().yaw),
                "ts": float(pose.ts),
            }
            self.map_out.publish(json.dumps(payload).encode())
        except Exception:
            logger.warning("odom encode/publish failed, dropping frame", exc_info=True)
            return
        self._last_odom_pub = now

    @staticmethod
    def _block_max(cells: Any, factor: int) -> Any:
        """Block maximum (not mean) so coarsening never erases an obstacle."""
        import numpy as np

        h, w = cells.shape[:2]
        new_h, new_w = h // factor, w // factor
        if new_h == 0 or new_w == 0:
            return cells
        trimmed = cells[: new_h * factor, : new_w * factor]
        blocks = trimmed.reshape(new_h, factor, new_w, factor)
        # Sink unknown below every known value for the max, then map it back.
        as_int = blocks.astype(np.int16)
        known = np.where(as_int < 0, -1000, as_int)
        reduced = known.max(axis=(1, 3))
        reduced[reduced == -1000] = -1
        return reduced.astype(np.int8)

    @staticmethod
    def _occupancy_to_bgra(cells: Any) -> Any:
        """Occupancy int8 {-1,0,1..100} → BGRA for PNG; unknown transparent."""
        import numpy as np

        # (B, G, R, A) — RGB reversed for OpenCV.
        c_unknown = (0, 0, 0, 0)
        c_free = (68, 58, 30, 255)  # #1e3a44 dark cyan
        c_occupied = (239, 220, 143, 255)  # #8fdcef bright cyan
        c_lethal = (255, 255, 255, 255)

        out = np.empty((*cells.shape, 4), dtype=np.uint8)
        out[...] = c_unknown
        out[cells == 0] = c_free
        out[cells >= 1] = c_occupied
        out[cells >= 100] = c_lethal
        return out
