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

"""Seed a replay from the loaded_map stream a relocalized recording carries."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from dimos.memory.store.sqlite import SqliteStore
from dimos.memory.tf import StreamTF
from dimos.memory.type.observation import Observation
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def first_loaded_map(store: SqliteStore, stream: str) -> Observation[PointCloud2] | None:
    """The earliest loaded map in the recording, or None without the stream."""
    if stream not in store.list_streams():
        return None
    return store.stream(stream, PointCloud2).order_by("ts").first()


def place_loaded_map(
    loaded_map: Observation[PointCloud2], tf: StreamTF, world_frame: str, ts: float
) -> NDArray[np.float32]:
    """World-frame points of the loaded map, placed by the transform at ts."""
    placement = tf.get(world_frame, loaded_map.data.frame_id, time_point=ts)
    if placement is None:
        raise RuntimeError(
            f"no {world_frame}->{loaded_map.data.frame_id} transform at ts={ts:.3f} "
            "to place the loaded map"
        )
    return loaded_map.data.transform(placement).points_f32()


def log_loaded_map(points: NDArray[np.float32]) -> None:
    import rerun as rr

    rr.log("world/loaded_map", rr.Points3D(points, colors=[[130, 130, 130]], radii=0.008))
