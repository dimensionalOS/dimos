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

"""Transform lookups over the transforms recorded in a store.

A store's ``tf`` member lazily reads every transform recorded under the ``tf``
(and ``tf_static``) streams into a :class:`MultiTBuffer`, then answers
``store.tf.get(target_frame, source_frame, time)`` — composing multi-hop chains
(e.g. ``world -> map -> odom -> base_link -> mid360_link``) and interpolating to
the nearest recorded sample. This makes world-registration a real transform
lookup instead of assuming a single baked-in pose.

``write_tf_tree`` populates those streams for a recording that lacks them.
"""

from __future__ import annotations

import re
import sqlite3
import threading
from typing import TYPE_CHECKING

import numpy as np

from dimos.memory2.store.sqlite import SqliteStoreConfig
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.tf import MultiTBuffer

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store

TF_STREAMS = ("tf", "tf_static")
# Larger than any single recording's span so the buffer never prunes loaded
# transforms (MultiTBuffer drops samples older than ts - buffer_size).
_NO_PRUNE = 1.0e15
# SQLite can't parameterize table names, so caller-supplied stream names are
# interpolated; allow only safe identifiers to keep that injection-free.
_SAFE_TABLE = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def _safe_table(name: str) -> str:
    if not _SAFE_TABLE.match(name):
        raise ValueError(f"unsafe stream/table name: {name!r}")
    return name


class DbTf:
    """Transform lookups backed by the ``tf``/``tf_static`` streams of a store."""

    def __init__(self, store: Store, stream_names: tuple[str, ...] = TF_STREAMS) -> None:
        self._store = store
        self._stream_names = stream_names
        self._buffer: MultiTBuffer | None = None
        self._load_lock = threading.Lock()

    def _ensure_loaded(self) -> MultiTBuffer:
        if self._buffer is not None:
            return self._buffer
        with self._load_lock:
            if self._buffer is not None:  # another thread loaded while we waited
                return self._buffer
            buffer = MultiTBuffer(buffer_size=_NO_PRUNE)
            available = set(self._store.list_streams())
            for name in self._stream_names:
                if name not in available:
                    continue
                for observation in self._store.stream(name, TFMessage):
                    message = observation.data
                    transforms = getattr(message, "transforms", None)
                    if transforms is None:
                        transforms = [message]
                    buffer.receive_transform(*transforms)
            self._buffer = buffer
            return buffer

    def has_transforms(self) -> bool:
        return bool(self._ensure_loaded().buffers)

    def get(
        self,
        target_frame: str,
        source_frame: str,
        time_point: float | None = None,
        time_tolerance: float | None = None,
    ) -> Transform | None:
        """Transform that maps a point in ``source_frame`` into ``target_frame``.

        Returns ``None`` if no chain connects the two frames. Uses the buffer's
        non-warning lookup so per-scan misses don't spam the log.
        """
        buffer = self._ensure_loaded()
        return buffer.lookup(target_frame, source_frame, time_point, time_tolerance)


def transform_matrix(transform: Transform) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(R, t)`` (3x3, 3) for ``transform`` so ``p_target = p_source @ R.T + t``."""
    rotation = transform.rotation
    rotation_matrix = np.asarray(rotation.to_rotation_matrix(), float).reshape(3, 3)
    translation = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z], float
    )
    return rotation_matrix, translation


def write_tf_tree(
    store: Store,
    *,
    odom_stream: str,
    odom_parent: str = "odom",
    odom_child: str = "base_link",
    root_links: tuple[tuple[str, str], ...] = (("world", "map"), ("map", "odom")),
    sensor_child: str = "mid360_link",
    sensor_translation: tuple[float, float, float] = (0.0, 0.0, 0.0),
    sensor_rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    static_period: float = 0.45,
    stream_name: str = "tf",
) -> int:
    """Populate ``store``'s tf stream from an odometry stream.

    - ``root_links`` and ``odom_child -> sensor_child`` are emitted as identity /
      fixed transforms every ``static_period`` seconds across the recording span.
    - ``odom_parent -> odom_child`` is emitted once per odometry sample, taken
      from each observation's pose.

    Returns the number of tf observations written.
    """
    config = store.config
    if not isinstance(config, SqliteStoreConfig):
        raise TypeError("write_tf_tree reads the db directly and needs a SqliteStore")
    db_path = config.path
    connection = sqlite3.connect(f"file:{db_path}?mode=ro&immutable=1", uri=True)
    odom = np.array(
        list(
            connection.execute(
                "select ts,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw "
                f"from {_safe_table(odom_stream)} order by ts"
            )
        ),
        float,
    )
    connection.close()
    if not len(odom):
        raise ValueError(f"odom stream {odom_stream!r} is empty; cannot build tf tree")

    tf_stream = store.stream(stream_name, TFMessage)
    written = 0

    # dynamic: odom_parent -> odom_child, one per odometry sample
    for row in odom:
        ts = float(row[0])
        transform = Transform(
            translation=Vector3(row[1], row[2], row[3]),
            rotation=Quaternion(row[4], row[5], row[6], row[7]),
            frame_id=odom_parent,
            child_frame_id=odom_child,
            ts=ts,
        )
        tf_stream.append(TFMessage(transform), ts=ts)
        written += 1

    # static: root links + sensor mount, resampled every static_period
    t0 = float(odom[0, 0])
    t1 = float(odom[-1, 0])

    def statics_at(ts: float) -> list[Transform]:
        links = [
            Transform(frame_id=parent, child_frame_id=child, ts=ts) for parent, child in root_links
        ]
        links.append(
            Transform(
                translation=Vector3(*sensor_translation),
                rotation=Quaternion(*sensor_rotation),
                frame_id=odom_child,
                child_frame_id=sensor_child,
                ts=ts,
            )
        )
        return links

    for static_ts in np.arange(t0, t1 + static_period, static_period):
        tf_stream.append(TFMessage(*statics_at(float(static_ts))), ts=float(static_ts))
        written += 1

    return written
