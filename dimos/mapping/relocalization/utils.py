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

"""Relocalization helpers: premap and named-location loading, map naming, and the live clouds it matches."""

from __future__ import annotations

from collections import deque
import math
from pathlib import Path
import re
import threading

import numpy as np
from reactivex import Observable, operators as ops
from scipy.spatial.transform import Rotation, Slerp

from dimos.mapping.relocalization.cloud_relocalizer import RelocalizeConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.PointCloud2 import FLAG_NAMES, PointCloud2
from dimos.msgs.visualization_msgs.InteractiveMarkerUpdate import InteractiveMarkerUpdate, pose_of
from dimos.utils.data import resolve_named_path
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

RECORDING_SUFFIXES = (".db", ".mcap")


def window(scans: Observable[PointCloud2], cfg: RelocalizeConfig) -> Observable[PointCloud2]:
    """The last ``max_frames`` scans merged, on a worker that takes only the latest window whenever it is free."""
    held: deque[PointCloud2] = deque(maxlen=cfg.max_frames)
    return backpressure(
        scans.pipe(
            ops.do_action(held.append),
            ops.filter(lambda _: len(held) >= cfg.min_frames),
            # A snapshot, so the merge on the worker never sees the deque change under it
            ops.map(lambda _: tuple(held)),
        )
    ).pipe(ops.map(_merged))


def _merged(frames: tuple[PointCloud2, ...]) -> PointCloud2:
    points = np.concatenate([frame.points_f32().reshape(-1, 3) for frame in frames])
    return PointCloud2.from_numpy(
        points, frame_id=frames[0].frame_id, timestamp=max(frame.ts for frame in frames)
    )


_PREMAP_BIT = 1 << FLAG_NAMES.index("premap")

# A voxel key's three axes, packed into one int64 at this many bits each.
_KEY_BITS = 21
_KEY_OFFSET = 1 << (_KEY_BITS - 1)


def _pack(points: np.ndarray, voxel: float) -> np.ndarray:
    keys = np.floor(points / voxel).astype(np.int64) + _KEY_OFFSET
    packed: np.ndarray = (keys[:, 0] << (2 * _KEY_BITS)) | (keys[:, 1] << _KEY_BITS) | keys[:, 2]
    return packed


def moved(points: np.ndarray, tf: Transform) -> np.ndarray:
    """*points* in tf's child frame, expressed in its parent frame."""
    matrix = tf.to_matrix()
    placed: np.ndarray = (points @ matrix[:3, :3].T + matrix[:3, 3]).astype(np.float32)
    return placed


def blend(start: Transform, end: Transform, weight: float) -> Transform:
    """*start* moved *weight* (0-1) of the way to *end*."""
    a, b = start.to_matrix(), end.to_matrix()
    matrix = np.eye(4)
    matrix[:3, :3] = Slerp([0.0, 1.0], Rotation.from_matrix([a[:3, :3], b[:3, :3]]))(
        weight
    ).as_matrix()
    matrix[:3, 3] = (1.0 - weight) * a[:3, 3] + weight * b[:3, 3]
    return Transform.from_matrix(matrix, frame_id=end.frame_id, child_frame_id=end.child_frame_id)


class LiveMap:
    """Live points (lidar scans, or the non-premap part of the ray tracer's local maps), kept one per voxel."""

    def __init__(self, voxel: float) -> None:
        self.voxel = voxel
        self._keys = np.empty(0, dtype=np.int64)
        self._points = np.empty((0, 3), dtype=np.float32)
        self._lock = threading.Lock()

    def __len__(self) -> int:
        return len(self._keys)

    def fold(self, local_map: PointCloud2, to_frame: Transform | None = None) -> None:
        """Keep *local_map*'s non-premap voxels, moved by *to_frame* (frame -> local_map's frame) first."""
        flags = local_map.flags_u32()
        if flags is None:
            raise ValueError(
                "local_map has no flags, so premap voxels cannot be told apart: run the ray tracer with a premap"
            )
        points = local_map.points_f32().reshape(-1, 3)[(flags & _PREMAP_BIT) == 0]
        self.add(points if to_frame is None else moved(points, to_frame))

    def add(self, points: np.ndarray) -> None:
        """Keep *points* (N x 3, already in the kept frame)."""
        keys = _pack(points, self.voxel)
        with self._lock:
            all_keys = np.concatenate([keys, self._keys])
            all_points = np.concatenate([points, self._points])
            # First occurrence wins, so a voxel's newest point is kept.
            unique_keys, first = np.unique(all_keys, return_index=True)
            self._keys, self._points = unique_keys, all_points[first]

    def cloud(
        self, frame_id: str, center: tuple[float, float] | None, radius: float
    ) -> PointCloud2:
        """The kept points, only those within ``radius`` (xy) of ``center`` when both are given."""
        with self._lock:
            points = self._points
        if center is not None and radius > 0:
            dx = points[:, 0] - center[0]
            dy = points[:, 1] - center[1]
            points = points[dx * dx + dy * dy <= radius * radius]
        return PointCloud2.from_numpy(points, frame_id=frame_id)


def stream_key(name: str) -> str:
    """*name* with ``/`` folded to ``_``, since a ``.db`` stream name is a SQL identifier."""
    return name.lstrip("/_").replace("/", "_")


def find_stream(streams: list[str], wanted: str) -> str | None:
    return next((name for name in streams if stream_key(name) == stream_key(wanted)), None)


DEFAULT_MAP_FRAME = "map"

VALID_MAP_NAMES = re.compile(rf"^{re.escape(DEFAULT_MAP_FRAME)}_(?P<name>.+)_cloud$")

# A cloud_stream that is not a named map has no locations of its own
OVERRIDE_LOCATIONS = "named_locations"


def named_map_frame(name: str) -> str:
    return f"{DEFAULT_MAP_FRAME}/{name}"


def map_cloud_stream(name: str, slashed: bool = True) -> str:
    stream = f"{named_map_frame(name)}/cloud"
    return stream if slashed else stream_key(stream)


def map_names(streams: list[str]) -> dict[str, str]:
    """Map name -> its cloud stream."""
    found: dict[str, str] = {}
    for name in streams:
        match = VALID_MAP_NAMES.match(stream_key(name))
        if match:
            found[match["name"]] = name
    return found


def resolve_map(streams: list[str], wanted: str | None) -> tuple[str, str, str]:
    """(cloud stream, map frame, locations stream); *wanted* overrides the lone ``map/<name>/cloud``."""
    named = map_names(streams)
    if wanted is not None:
        name = next((n for n, s in named.items() if stream_key(s) == stream_key(wanted)), None)
        if name is None:
            if find_stream(streams, wanted) is None:
                raise ValueError(
                    f"the recording has no {wanted!r} stream, it holds {sorted(streams)}"
                )
            return wanted, wanted.lstrip("/"), OVERRIDE_LOCATIONS
    elif len(named) == 1:
        name = next(iter(named))
    elif not named:
        raise ValueError(
            f"no map in the recording: nothing in {sorted(streams)} is a {map_cloud_stream('<name>')}"
        )
    else:
        raise ValueError(
            f"{len(named)} maps in the recording ({', '.join(sorted(named))}): "
            "pass cloud_stream to say which"
        )
    frame = named_map_frame(name)
    return named[name], frame, f"{frame}/named_locations"


def load_premap(map_file: str, stream: str | None = None) -> PointCloud2:
    """The last cloud of the recording's map, or of *stream*."""
    if Path(map_file).suffix not in RECORDING_SUFFIXES:
        raise ValueError(
            f"map_file {map_file!r} must be a {' or '.join(RECORDING_SUFFIXES)} recording"
        )
    from dimos.memory.cli.dataset import open_store

    path = resolve_named_path(map_file, Path(map_file).suffix)
    with open_store(path, decode_metadata=True) as store:
        streams = store.list_streams()
        if stream is None:
            stream, _, _ = resolve_map(streams, None)
        name = find_stream(streams, stream)
        if name is None:
            raise ValueError(f"{path} has no {stream!r} stream, have {streams}")
        cloud: PointCloud2 = store.stream(name).last().data
    return cloud


def yaw_deg(tf: Transform) -> float:
    return math.degrees(tf.rotation.euler.z)


def fix_error(fix: Transform, recorded: Transform) -> tuple[float, float]:
    """Yaw in degrees and translation in meters between two fixes of the same frame pair."""
    dyaw = (yaw_deg(fix) - yaw_deg(recorded) + 180.0) % 360.0 - 180.0
    return dyaw, (fix.translation - recorded.translation).length()


def load_locations(path: str, stream: str) -> dict[str, PoseStamped]:
    """The last locations snapshot in a .db or .mcap, or none if the stream is absent."""
    from dimos.memory.cli.dataset import open_store

    if not Path(path).exists():
        logger.warning(f"named locations file {path} does not exist yet, starting with none")
        return {}
    with open_store(path, decode_metadata=True) as store:
        name = find_stream(store.list_streams(), stream)
        if name is None:
            return {}
        last: InteractiveMarkerUpdate = store.stream(name).last().data
    return {named.name: pose_of(named) for named in last.poses}
