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
from pathlib import Path
import time

from dimos_generated.geometry_msgs.msg import PoseStamped, TransformStamped
from dimos_generated.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos_generated.tf2_msgs.msg import TFMessage
import pytest

from dimos.core.transport import LCMTransport
from dimos.core.transport_factory import make_transport
from dimos.e2e_tests.cdr_replay_fixture import write_go2_cdr_replay
from dimos.msgs.time import time_from_seconds
from dimos.protocol.tf.tf import TF
from dimos.robot.unitree.go2 import connection
from dimos.utils.testing.moment import Moment, SensorMoment

pytestmark = pytest.mark.self_hosted


class Go2Moment(Moment):
    lidar: SensorMoment[PointCloud2]
    video: SensorMoment[Image]
    odom: SensorMoment[PoseStamped]

    def __init__(self, recording: str | Path) -> None:
        self.lidar = SensorMoment(f"{recording}/lidar", LCMTransport("/lidar", PointCloud2))
        self.video = SensorMoment(f"{recording}/color_image", LCMTransport("/color_image", Image))
        self.odom = SensorMoment(f"{recording}/odom", LCMTransport("/odom", PoseStamped))

    @property
    def transforms(self) -> list[TransformStamped]:
        if self.odom.value is None:
            return []

        # we just make sure to change timestamps so that we can jump
        # back and forth through time and the viewer doesn't get confused
        odom = PoseStamped.decode(self.odom.value.encode())
        odom.header.stamp = time_from_seconds(time.time())
        return connection.GO2Connection._odom_to_tf(odom)

    def publish(self) -> None:
        tf_transport = make_transport("/tf", TFMessage)
        t = TF(tf_transport)
        t.publish(*self.transforms)
        t.dispose()
        tf_transport.stop()

        camera_info = CameraInfo.decode(connection.GO2Connection.camera_info_static.encode())
        camera_info.header.stamp = time_from_seconds(time.time())
        camera_info_transport: LCMTransport[CameraInfo] = LCMTransport("/camera_info", CameraInfo)
        camera_info_transport.publish(camera_info)
        camera_info_transport.stop()

        super().publish()


def test_moment_seek_and_publish(tmp_path: Path) -> None:
    recording = tmp_path / "go2-cdr.db"
    write_go2_cdr_replay(recording, duration_s=8)
    moment = Go2Moment(recording)
    try:
        moment.seek(5.0)
        assert moment.lidar.value is not None
        assert moment.video.value is not None
        assert moment.odom.value is not None
        moment.publish()
    finally:
        moment.stop()
