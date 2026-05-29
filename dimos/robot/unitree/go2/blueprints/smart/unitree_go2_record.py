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

import os
from pathlib import Path
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.memory2.stream import Stream
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Go2Mid360MemoryConfig(RecorderConfig):
    db_path: str | Path = "recording_go2_mid360.db"
    default_frame_id: str = "base_link"


class Go2Mid360Memory(Recorder):
    """Records Go2 camera, native Go2 lidar, Mid-360 lidar, FastLio2 odometry, and Go2 leg odometry."""

    config: Go2Mid360MemoryConfig

    color_image: In[Image]
    lidar: In[PointCloud2]
    odom: In[PoseStamped]
    fastlio_lidar: In[PointCloud2]
    fastlio_odometry: In[Odometry]

    def _port_to_stream(self, name: str, input_topic: In[Any], stream: Stream[Any]) -> None:
        """Append each message from *input_topic* to *stream*, attaching world pose via tf.

        Stamped messages use their own ``.frame_id`` and ``.ts``; unstamped
        messages (or ones whose frame isn't in the tf graph, e.g. a payload
        already in world coords) fall back to ``config.default_frame_id`` —
        so every observation gets a robot-pose anchor when tf is publishing.

        Registers the subscription as a disposable on this module.
        """

        default_frame_id = self.config.default_frame_id
        tf_tolerance = self.config.tf_tolerance

        def on_msg(msg: Any) -> None:
            # Force system time for all messages
            ts = time.time()
            frame_id = (
                getattr(msg, "child_frame_id", None)
                or getattr(msg, "frame_id", None)
                or default_frame_id
            )
            transform = self.tf.get("world", frame_id, time_point=ts, time_tolerance=tf_tolerance)
            pose = transform.to_pose() if transform is not None else None

            stream.append(msg, ts=ts, pose=pose)

        self.register_disposable(Disposable(input_topic.subscribe(on_msg)))


unitree_go2_record = autoconnect(
    GO2Connection.blueprint(),
    KeyboardTeleop.blueprint(),
    MovementManager.blueprint(),
    FastLio2.blueprint(
        frame_id="world",
        map_freq=-1,
        lidar_ip=os.getenv("LIDAR_IP", "192.168.1.155"),
        record_pcap=True,
        deterministic_clock=True,
    ).remappings(
        [
            (FastLio2, "lidar", "fastlio_lidar"),
            (FastLio2, "odometry", "fastlio_odometry"),
        ]
    ),
    Go2Mid360Memory.blueprint(),
).global_config(n_workers=10, robot_model="unitree_go2")
