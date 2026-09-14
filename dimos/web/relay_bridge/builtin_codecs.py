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

"""Built-in web codecs (jpeg.v1, pose.json.v1, costmap.zlib.v1, text.json.v1,
stats.json.v1).

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
from dimos.web.codecs import EncodedPayload, web_decoder, web_encoder

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


@web_decoder("text.json.v1")
def decode_text(msg: str) -> str:
    # The browser value arrives as parsed JSON, so the annotation cannot be
    # trusted at runtime; the explicit check gives a clear nack message.
    if not isinstance(msg, str):
        raise ValueError(f"text.json.v1 wants a string, got {type(msg).__name__}")
    return msg


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


# stats.json.v1: the resource monitor's /resource_stats dict (asdict of
# ProcessStats/WorkerStats/ChildProcessStats, dimos/core/resource_monitor/)
# as JSON with exactly the keys the Stats page reads and dtop renders. Picked
# by name on purpose: a renamed producer field raises KeyError here (an
# encode error the bridge logs) instead of silently vanishing from the page,
# and test_stats_encoding.py pins the subset against the dataclasses.
_STATS_PROCESS_KEYS = (
    "pid",
    "alive",
    "cpu_percent",
    "cpu_time_user",
    "cpu_time_system",
    "cpu_time_iowait",
    "pss",
    "num_threads",
    "num_children",
    "num_fds",
    "io_read_bytes",
    "io_write_bytes",
)
_STATS_WORKER_KEYS = (*_STATS_PROCESS_KEYS, "worker_id", "modules", "dedicated")
_STATS_CHILD_KEYS = ("pid", "name", "cpu_percent")


def _pick(stats: Mapping[str, Any], keys: tuple[str, ...]) -> dict[str, Any]:
    return {key: stats[key] for key in keys}


# The registry keys encoders by the bare message class (dict[str, Any] is not
# a class), hence the unparameterized annotation.
@web_encoder("stats.json.v1")
def encode_stats(msg: dict) -> bytes:  # type: ignore[type-arg]
    workers = [
        {
            **_pick(worker, _STATS_WORKER_KEYS),
            "children": [_pick(child, _STATS_CHILD_KEYS) for child in worker["children"]],
        }
        for worker in msg["workers"]
    ]
    stats = {"coordinator": _pick(msg["coordinator"], _STATS_PROCESS_KEYS), "workers": workers}
    return json.dumps(stats, separators=(",", ":")).encode()
