# Copyright 2025 Dimensional Inc.
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

import shutil
import time

from dimos_lcm.sensor_msgs import PointCloud2

from dimos import core
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection2d.testing import get_moment, publish_moment
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils.data import _get_data_dir
from dimos.utils.testing import TimedSensorReplay, TimedSensorStorage


def test_publish():
    def start_recorder():
        recording_name = "test_play_recording"
        record_data_dir = _get_data_dir(recording_name)

        # Delete old data directory if it exists
        if record_data_dir.exists():
            shutil.rmtree(record_data_dir)

        lidar_store: TimedSensorStorage = TimedSensorStorage(f"{recording_name}/lidar")
        lidar_sub = core.LCMTransport("/lidar", PointCloud2)
        lidar_store.consume_stream(lidar_sub.observable())

    start_recorder()

    dir_name = "unitree_go2_lidar_corrected"
    lidar_store = TimedSensorReplay(f"{dir_name}/lidar")
    odom_store = TimedSensorReplay(
        f"{dir_name}/odom", autocast=Odometry.from_msg
    )  # don't worry about autocast, this particular recording requires it
    video_store = TimedSensorReplay(f"{dir_name}/video")

    lidar_pub = core.LCMTransport("/lidar", PointCloud2)
    odom_pub = core.LCMTransport("/odom", Odometry)
    image_pub = core.LCMTransport("/image", Image)

    lidar_store.stream(duration=2.0).subscribe(lidar_pub.publish)
    odom_store.stream(duration=2.0).subscribe(odom_pub.publish)
    video_store.stream(duration=2.0).subscribe(image_pub.publish)
