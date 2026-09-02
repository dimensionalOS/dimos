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

"""Built-in web encoders (jpeg.v1, pose.json.v1, costmap.zlib.v1).

Registered into dimos.web.codecs at import time; relay_bridge_module imports
this module so every bridge process (parent and worker) has the built-ins.
Wire bytes are pinned by web/shared/fixtures/costmap_frames.json and the
relay e2e tests; the matching JS decoders live in web/sdk/src/decoders/.
"""

from collections.abc import Mapping
import json
from typing import Any
import zlib

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid, block_max_reduce
from dimos.msgs.sensor_msgs.Image import Image
from dimos.web.codecs import EncodedPayload, web_encoder

# Custom jpeg channels authored without a quality param; the built-in
# color_image channel never reaches this (main() merges config.jpeg_quality
# into its params).
_DEFAULT_JPEG_QUALITY = 75


def _check_jpeg_params(params: Mapping[str, Any]) -> None:
    quality = params.get("quality")
    if quality is None:
        return
    if isinstance(quality, bool) or not isinstance(quality, int) or not 0 <= quality <= 100:
        raise ValueError(f"quality must be an int in 0..100, got {quality!r}")


@web_encoder("jpeg.v1", check_params=_check_jpeg_params)
def encode_jpeg(msg: Image, params: Mapping[str, Any]) -> EncodedPayload:
    # TurboJPEG via the message's own encoder (handles BGR/RGB/gray inputs).
    return EncodedPayload(
        msg.to_jpeg_bytes(quality=params.get("quality", _DEFAULT_JPEG_QUALITY)),
        {"w": msg.width, "h": msg.height},
    )


@web_encoder("pose.json.v1")
def encode_pose(msg: PoseStamped) -> bytes:
    pose = {
        "x": msg.position.x,
        "y": msg.position.y,
        "z": msg.position.z,
        "yaw": msg.yaw,
        "ts": msg.ts,
    }
    return json.dumps(pose, separators=(",", ":")).encode()


# The historical costmap encoder's choice (websocket_vis/optimized_costmap.py);
# full grids compress to ~10-30 KB at <= 5 Hz, so speed over ratio is fine.
_COSTMAP_ZLIB_LEVEL = 6
# Render budget shared with the cockpit decoder (MAX_COSTMAP_DIM in
# costmap.ts): larger grids are block-max downsampled before compression so
# every frame stays within what consumers accept and render. 2048^2 raw is
# 4 MiB, and zlib worst case adds ~0.01%, so the 8 MiB payload caps
# (_wt_session._MAX_PAYLOAD_BYTES and the cockpit's) are unreachable.
_COSTMAP_MAX_SIDE = 2048


@web_encoder("costmap.zlib.v1")
def encode_costmap(msg: OccupancyGrid) -> EncodedPayload | None:
    grid = msg.grid
    if grid.size == 0:
        return None  # mapper still warming up; nothing to draw
    res = msg.resolution
    side = max(grid.shape)
    if side > _COSTMAP_MAX_SIDE:
        factor = -(-side // _COSTMAP_MAX_SIDE)
        grid = block_max_reduce(grid, factor)
        res *= factor
    h, w = grid.shape
    # Wire contract (costmap.zlib.v1): uint8 cells, ROS -1 unknown -> 255.
    # int8 -1 is byte 0xff and 0..100 are byte-identical, so the raw buffer
    # already is the wire payload - no mask/astype/tobytes copies.
    cells = np.ascontiguousarray(grid)
    origin = msg.origin
    meta = {
        "w": w,
        "h": h,
        "res": res,
        "origin": [origin.position.x, origin.position.y, origin.yaw],
    }
    return EncodedPayload(zlib.compress(cells, _COSTMAP_ZLIB_LEVEL), meta)
