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

"""Built-in web codecs (jpeg.v1, pose.json.v1, costmap.zlib.v1, voxels.zlib.v1,
text.json.v1, stats.json.v1, path.json.v1, point.json.v1, bool.json.v1).

Registered into dimos.web.codecs at import time; relay_bridge_module imports
this module so every bridge process (parent and worker) has the built-ins.
Wire bytes are pinned by web/shared/fixtures/costmap_frames.json,
voxel_frames.json and the relay e2e tests; the matching JS decoders live in
web/sdk/src/decoders/.
"""

from collections.abc import Mapping
import json
from typing import Any
import zlib

from dimos_lcm.std_msgs import Bool
import numpy as np

from dimos.mapping.voxels.keys import FIELD_BITS, KEY_OFFSET
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid, block_max_reduce
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.generic import finite_number
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
    meta = {"w": msg.width, "h": msg.height}
    # An image that arrived as JPEG (a JpegLcmTransport stream) is forwarded
    # as it came, at the producer's quality: the channel's `quality` applies
    # to images the bridge encodes itself.
    if msg.jpeg is not None:
        return EncodedPayload(msg.jpeg, meta)
    # TurboJPEG via the message's own encoder (handles BGR/RGB/gray inputs).
    return EncodedPayload(
        msg.to_jpeg_bytes(quality=params.get("quality", _DEFAULT_JPEG_QUALITY)), meta
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


@web_encoder("path.json.v1")
def encode_path(msg: Path) -> bytes:
    # Empty paths must reach the viewer to clear the overlay.
    points = [[round(p.x, 3), round(p.y, 3)] for p in msg.poses]
    return json.dumps(points, separators=(",", ":"), allow_nan=False).encode()


@web_decoder("point.json.v1")
def decode_point(msg: dict[str, Any]) -> PointStamped:
    if not isinstance(msg, dict):
        raise ValueError(f"point.json.v1 wants an object, got {type(msg).__name__}")
    return PointStamped(
        finite_number(msg.get("x"), "x"), finite_number(msg.get("y"), "y"), frame_id="world"
    )


@web_decoder("bool.json.v1")
def decode_bool(msg: bool) -> Bool:
    if not isinstance(msg, bool):
        raise ValueError(f"bool.json.v1 wants a boolean, got {type(msg).__name__}")
    return Bool(data=msg)


# Costmap cells and voxel bit masks are long runs of identical bytes, so the
# run-length strategy at level 1 compresses them in a fraction of the time of
# the default level 6 (a 2048^2 costmap 18 -> 4 ms, a scattered 1M-voxel map
# 230 -> 25 ms) for a payload that is about twice the size on a costmap and
# smaller on a scattered cloud; both stay under 100 KB at the shipped rates.
# Still RFC 1950, so the SDK's inflate needs no change.
_ZLIB_LEVEL = 1


def _zlib_rle(data: bytes) -> bytes:
    compressor = zlib.compressobj(
        _ZLIB_LEVEL, zlib.DEFLATED, zlib.MAX_WBITS, zlib.DEF_MEM_LEVEL, zlib.Z_RLE
    )
    return compressor.compress(data) + compressor.flush()


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
    return EncodedPayload(_zlib_rle(cells.tobytes()), meta)


# voxels.zlib.v1: a PointCloud2 as voxel occupancy bits. Points are quantized
# to `res` (the mapper's own voxel size by default, so voxel centres land back
# in their cell), grouped into 16^3 chunks, and every non-empty chunk is one
# 524-byte record: int32 LE chunk coords, then 512 bytes of bits indexed
# (lz * 16 + ly) * 16 + lx in little-endian bit order. zlib over the records
# costs ~0.3 bytes per voxel on a real map (103 KB for the 408k-voxel
# big_office cloud) against 16 bytes per point for the LCM encoding. The
# cockpit decoder (voxels.ts) mirrors the caps; a cloud over budget is
# coarsened (res doubled) until it fits, the costmap's block-max idea in 3D.
# The packing sorts once (a scattered 1M-voxel cloud 570 -> 400 ms with the
# zlib, a 500k floor 80 -> 65); still ~10% of a core per hertz at 1M, so keep
# such maps at 1-2 Hz or coarser.
# An empty cloud is data, not a warm-up gap: it encodes to zero records so a
# filtered-out or cleared map clears the panel (the costmap's None is for a
# grid that does not exist yet).
_VOXEL_CHUNK_SHIFT = 4
_VOXEL_CHUNK = 1 << _VOXEL_CHUNK_SHIFT
_VOXEL_CHUNK_BITS = _VOXEL_CHUNK**3
_VOXEL_RECORD = np.dtype(
    [("cx", "<i4"), ("cy", "<i4"), ("cz", "<i4"), ("bits", "u1", (_VOXEL_CHUNK_BITS // 8,))]
)
_VOXEL_MAX_VOXELS = 1_000_000
_VOXEL_MAX_CHUNKS = 32_768
_DEFAULT_VOXEL_RES = 0.05


def _check_voxel_params(params: Mapping[str, Any]) -> None:
    res = params.get("res")
    if res is not None and finite_number(res, "res") <= 0:
        raise ValueError(f"res must be positive, got {res!r}")


# One sortable int64 per voxel: its chunk's (x, y, z) in three 17-bit fields
# (the mapping key layout's 21 bits less the 4 the chunk absorbs, so a map
# spans the same +-52 km), then the voxel's bit position in the chunk. One
# np.unique over them orders and deduplicates the voxels by chunk and bit at
# once, and every boundary after that is an np.diff: the chunk ids, the mask
# bytes. Chunks come out in the key layout's (x, y, z) order.
_CHUNK_FIELD_BITS = FIELD_BITS - _VOXEL_CHUNK_SHIFT
_CHUNK_BIAS = KEY_OFFSET >> _VOXEL_CHUNK_SHIFT
_CHUNK_MASK = (1 << _CHUNK_FIELD_BITS) - 1
_BIT_BITS = 3 * _VOXEL_CHUNK_SHIFT


def _voxel_order_keys(idx: np.ndarray) -> np.ndarray:
    """Sorted unique keys of (N, 3) voxel indices (|idx| < KEY_OFFSET)."""
    c = (idx >> _VOXEL_CHUNK_SHIFT) + _CHUNK_BIAS
    local = idx & (_VOXEL_CHUNK - 1)
    bit = (local[:, 2] * _VOXEL_CHUNK + local[:, 1]) * _VOXEL_CHUNK + local[:, 0]
    chunk = (c[:, 0] << (2 * _CHUNK_FIELD_BITS)) | (c[:, 1] << _CHUNK_FIELD_BITS) | c[:, 2]
    return np.unique((chunk << _BIT_BITS) | bit)


def _voxel_index(keys: np.ndarray) -> np.ndarray:
    """(N, 3) voxel indices, the inverse of _voxel_order_keys."""
    chunk = keys >> _BIT_BITS
    bit = keys & ((1 << _BIT_BITS) - 1)
    cx = (chunk >> (2 * _CHUNK_FIELD_BITS)) - _CHUNK_BIAS
    cy = ((chunk >> _CHUNK_FIELD_BITS) & _CHUNK_MASK) - _CHUNK_BIAS
    cz = (chunk & _CHUNK_MASK) - _CHUNK_BIAS
    return np.stack(
        [
            (cx << _VOXEL_CHUNK_SHIFT) + (bit & (_VOXEL_CHUNK - 1)),
            (cy << _VOXEL_CHUNK_SHIFT) + ((bit >> _VOXEL_CHUNK_SHIFT) & (_VOXEL_CHUNK - 1)),
            (cz << _VOXEL_CHUNK_SHIFT) + (bit >> (2 * _VOXEL_CHUNK_SHIFT)),
        ],
        axis=1,
    )


def _boundaries(values: np.ndarray) -> np.ndarray:
    """Indices where a run of equal values starts (values sorted)."""
    if len(values) == 0:
        return np.zeros(0, dtype=np.int64)
    return np.flatnonzero(np.r_[True, values[1:] != values[:-1]])


@web_encoder("voxels.zlib.v1", check_params=_check_voxel_params)
def encode_voxels(msg: PointCloud2, params: Mapping[str, Any]) -> EncodedPayload:
    res = float(params.get("res", _DEFAULT_VOXEL_RES))
    pts = msg.points_f32()
    pts = pts[np.isfinite(pts).all(axis=1)]
    idx = np.floor(pts / res).astype(np.int64)
    # Indices live in the key's 21-bit fields (+-52 km at 5 cm); beyond that
    # a point is dropped.
    idx = idx[(np.abs(idx) < KEY_OFFSET).all(axis=1)]
    keys = _voxel_order_keys(idx)
    while True:
        chunk = keys >> _BIT_BITS
        starts = _boundaries(chunk)
        if len(keys) <= _VOXEL_MAX_VOXELS and len(starts) <= _VOXEL_MAX_CHUNKS:
            break
        keys = _voxel_order_keys(_voxel_index(keys) >> 1)
        res *= 2
    n_chunks = len(starts)
    # Bytes of the bit masks: the keys are ordered by (chunk, bit), so each
    # mask byte's voxels are one run, OR'd in one reduceat.
    chunk_of = np.repeat(np.arange(n_chunks, dtype=np.int64), np.diff(np.r_[starts, len(keys)]))
    bit = keys & ((1 << _BIT_BITS) - 1)
    byte = chunk_of * (_VOXEL_CHUNK_BITS // 8) + (bit >> 3)
    first = _boundaries(byte)
    masks = np.zeros(n_chunks * (_VOXEL_CHUNK_BITS // 8), dtype=np.uint8)
    if len(keys):
        masks[byte[first]] = np.bitwise_or.reduceat((1 << (bit & 7)).astype(np.uint8), first)
    chunk_id = chunk[starts]
    records = np.empty(n_chunks, dtype=_VOXEL_RECORD)
    records["cx"] = (chunk_id >> (2 * _CHUNK_FIELD_BITS)) - _CHUNK_BIAS
    records["cy"] = ((chunk_id >> _CHUNK_FIELD_BITS) & _CHUNK_MASK) - _CHUNK_BIAS
    records["cz"] = (chunk_id & _CHUNK_MASK) - _CHUNK_BIAS
    records["bits"] = masks.reshape(-1, _VOXEL_CHUNK_BITS // 8)
    meta = {"res": res, "n": len(keys), "chunks": n_chunks}
    return EncodedPayload(_zlib_rle(records.tobytes()), meta)


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
