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

"""Open a Go2 DDS mcap directly as a read-only memory2 store.

    from dimos.robot.unitree.go2.dds.store import Go2McapStore

    store = Go2McapStore(path="go2_china_office_indoor.mcap")
    print(store.list_streams())
    for obs in store.streams.lidar.limit(5):
        print(obs.ts, obs.data)         # obs.data is a dimos PointCloud2

Thin Go2 wiring over the generic :class:`dimos.memory2.store.mcap.McapStore`:
supplies the Go2 codec set and stream-name map, and resolves the path through
the repo data dir / LFS.
"""

from __future__ import annotations

from typing import Any

from dimos.memory2.store.mcap import McapStore
from dimos.robot.unitree.go2.dds.codec import GO2_CODECS
from dimos.utils.data import resolve_named_path

# memory2 stream name -> Go2 DDS topic.
STREAMS: dict[str, str] = {
    "lidar": "rt/utlidar/cloud",
    "imu": "rt/utlidar/imu",
    "odom": "rt/utlidar/robot_odom",
    "color_image": "rt/frontvideo",
    "lowstate": "rt/lowstate",
    "sportmodestate": "rt/sportmodestate",
}


class Go2McapStore(McapStore):
    """``McapStore`` preset with the Go2 codecs, stream names, and path resolution."""

    def __init__(self, *, path: str, **kwargs: Any) -> None:
        super().__init__(
            path=str(resolve_named_path(path, ".mcap")),
            codecs=GO2_CODECS,
            streams=STREAMS,
            **kwargs,
        )
