#!/usr/bin/env python3
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

"""Go2 + Livox Mid-360 data collection blueprint.

Records color_image (Go2 camera), FastLio2 lidar (Mid360), and FastLio2
odometry to a SQLite database for offline map validation and multi-level
path planning development (issue #2202).

GO2Connection's native ``lidar`` is remapped to ``go2_lidar`` to avoid
colliding with FastLio2's registered point cloud on the ``lidar`` topic.

Usage::

    dimos --dtop --robot-ip 192.168.1.73 run unitree-go2-mid360-memory
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.memory2.stream import Stream
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection import GO2Connection

_voxel_size = 0.05

# FastLIO ports stamped by the C++ binary with hardware clock; must be
# overridden to time.time() so they align with color_image (also time.time()).
_FASTLIO_PORTS = frozenset({"lidar", "odometry"})


class Go2Mid360MemoryConfig(RecorderConfig):
    db_path: str | Path = "recording_go2_mid360.db"
    default_frame_id: str = "base_link"


class Go2Mid360Memory(Recorder):
    """Records Go2 camera, native Go2 lidar, Mid-360 lidar, and FastLio2 odometry."""

    color_image: In[Image]
    go2_lidar: In[PointCloud2]
    lidar: In[PointCloud2]
    odometry: In[Odometry]
    config: Go2Mid360MemoryConfig

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        if name not in _FASTLIO_PORTS:
            super()._port_to_stream(name, input_topic, stream)
            return

        # Force time.time() so FastLIO hardware timestamps match image timestamps.
        default_frame_id = self.config.default_frame_id
        tf_tolerance = self.config.tf_tolerance

        def on_msg(msg: Any) -> None:
            ts = time.time()
            frame_id = (
                getattr(msg, "child_frame_id", None)
                or getattr(msg, "frame_id", None)
                or default_frame_id
            )
            if frame_id == "world":
                frame_id = default_frame_id
            transform = self.tf.get("world", frame_id, time_point=ts, time_tolerance=tf_tolerance)
            pose = transform.to_pose() if transform is not None else None
            stream.append(msg, ts=ts, pose=pose)

        self.register_disposable(Disposable(input_topic.subscribe(on_msg)))


unitree_go2_mid360_memory = (
    autoconnect(
        unitree_go2_basic,
        FastLio2.blueprint(
            voxel_size=_voxel_size,
            map_voxel_size=_voxel_size,
            map_freq=-1,
        ),
        Go2Mid360Memory.blueprint(),
    )
    .remappings([
        (GO2Connection, "lidar", "go2_lidar"),
    ])
    .global_config(n_workers=6, robot_model="unitree_go2")
)

__all__ = ["unitree_go2_mid360_memory"]
