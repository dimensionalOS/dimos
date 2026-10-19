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

"""Deterministic generated-message replay for browser integration tests."""

import math
from pathlib import Path

from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.sensor_msgs.msg import Image, PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.memory.codecs.cdr import CdrCodec
from dimos.memory.codecs.lz4 import Lz4Codec
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.image import image_from_array
from dimos.msgs.pointcloud import pointcloud_from_xyz
from dimos.msgs.time import time_from_nanoseconds


def write_go2_cdr_replay(path: Path, *, duration_s: int = 180) -> None:
    """Record varying camera pixels, world poses and lidar as typed CDR streams."""
    base_ns = 1_700_000_000_123_456_789
    points = np.array([[1, 0, 0], [1, 1, 0], [2, 0, 0]], dtype=np.float32)
    with SqliteStore(path=str(path)) as store:
        video = store.stream("color_image", Image, codec=Lz4Codec(CdrCodec(Image)))
        odom = store.stream("odom", PoseStamped)
        lidar = store.stream("lidar", PointCloud2)
        for index in range(duration_s * 4):
            stamp_ns = base_ns + index * 250_000_000
            stamp = time_from_nanoseconds(stamp_ns)
            angle = index / 40
            header = Header(stamp=stamp, frame_id="world")
            pose = PoseStamped(
                header=header,
                pose=Pose(
                    position=Point(x=math.cos(angle), y=math.sin(angle), z=0.0),
                    orientation=Quaternion(
                        z=math.sin(angle / 2), w=math.cos(angle / 2), x=0.0, y=0.0
                    ),
                ),
            )
            pixels = np.full((240, 320, 3), index % 256, dtype=np.uint8)
            pixels[:, (index * 7) % 320] = (255, 0, 127)
            image = image_from_array(
                pixels, encoding="rgb8", header=Header(stamp=stamp, frame_id="camera_optical")
            )
            cloud = pointcloud_from_xyz(points, header=Header(stamp=stamp, frame_id="base_link"))
            receipt = stamp_ns / 1e9
            video.append(image, ts=receipt)
            odom.append(pose, ts=receipt)
            lidar.append(cloud, ts=receipt)
