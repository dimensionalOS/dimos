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

"""Small deterministic CDR recordings for offline documentation examples."""

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


def write_demo_recording(path: Path, *, samples: int = 720) -> SqliteStore:
    """Create a new image/pose/cloud recording at 4 Hz without downloads or models.

    Refuse existing files so examples cannot replace a user's recording.
    Images have both exact source stamps and observation poses. Storage uses
    LZ4 around generated CDR, retaining complete type/schema metadata.
    """
    if path.exists():
        raise FileExistsError(path)
    if samples <= 0:
        raise ValueError("samples must be positive")
    store = SqliteStore(path=path)
    images = store.stream("color_image", Image, codec=Lz4Codec(CdrCodec(Image)))
    poses = store.stream("odom", PoseStamped, codec=Lz4Codec(CdrCodec(PoseStamped)))
    clouds = store.stream("lidar", PointCloud2, codec=Lz4Codec(CdrCodec(PointCloud2)))
    x, y = np.meshgrid(np.linspace(-2.0, 2.0, 16), np.linspace(-2.0, 2.0, 16))
    points = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    for index in range(samples):
        stamp = time_from_nanoseconds(1_700_000_000_123_456_789 + index * 250_000_000)
        header = Header(stamp=stamp, frame_id="world")
        angle = index / 80.0
        pose = Pose(
            position=Point(x=math.cos(angle), y=math.sin(angle), z=0.0),
            orientation=Quaternion(w=1.0, x=0.0, y=0.0, z=0.0),
        )
        pixels = np.full((48, 64, 3), 40 + index % 180, dtype=np.uint8)
        pixels[:, 16:32, 1] = 230
        image = image_from_array(pixels, encoding="rgb8", header=header)
        ts = 1_700_000_000.1234567 + index / 4.0
        images.append(image, ts=ts, pose=pose)
        poses.append(PoseStamped(header=header, pose=pose), ts=ts, pose=pose)
        clouds.append(pointcloud_from_xyz(points, header=header), ts=ts, pose=pose)
    return store
