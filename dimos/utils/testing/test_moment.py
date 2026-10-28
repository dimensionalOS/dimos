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
import time

import pytest

from dimos.core.transport import LCMTransport
from dimos.core.transport_factory import make_transport
from dimos.message_utils import time as message_time
from dimos.messages.geometry_msgs.msg import PoseStamped, TransformStamped
from dimos.messages.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.protocol.tf.tf import TF
from dimos.robot.unitree.go2 import connection
from dimos.utils.data import get_data
from dimos.utils.testing.moment import Moment, SensorMoment

pytestmark = pytest.mark.self_hosted

_DATA_DIR_NAME = "unitree_go2_office_walk2"


class Go2Moment(Moment):
    lidar: SensorMoment[PointCloud2]
    video: SensorMoment[Image]
    odom: SensorMoment[PoseStamped]

    def __init__(self) -> None:
        data_dir = get_data(_DATA_DIR_NAME)
        self.lidar = SensorMoment(f"{data_dir}/lidar", LCMTransport("/lidar", PointCloud2))
        self.video = SensorMoment(f"{data_dir}/video", LCMTransport("/color_image", Image))
        self.odom = SensorMoment(f"{data_dir}/odom", LCMTransport("/odom", PoseStamped))

    @property
    def transforms(self) -> list[TransformStamped]:
        if self.odom.value is None:
            return []

        # we just make sure to change timestamps so that we can jump
        # back and forth through time and the viewer doesn't get confused
        odom = self.odom.value
        odom.header.stamp = message_time.time_from_seconds(time.time())
        return connection.GO2Connection._odom_to_tf(odom)

    def publish(self) -> None:
        tf_transport = make_transport("/tf", TFMessage)
        t = TF(tf_transport)
        t.publish(*self.transforms)
        t.dispose()
        tf_transport.stop()

        camera_info = connection._camera_info_static()
        camera_info.header.stamp = message_time.time_from_seconds(time.time())
        camera_info_transport: LCMTransport[CameraInfo] = LCMTransport("/camera_info", CameraInfo)
        camera_info_transport.publish(camera_info)
        camera_info_transport.stop()

        super().publish()


def test_moment_seek_and_publish() -> None:
    moment = Go2Moment()

    # Seek to 5 seconds
    moment.seek(5.0)

    # Check that frames were loaded
    assert moment.lidar.value is not None
    assert moment.video.value is not None
    assert moment.odom.value is not None

    # Publish all frames
    moment.publish()
    moment.stop()
