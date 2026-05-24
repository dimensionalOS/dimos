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
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.tf import TF
from dimos.robot.unitree.go2.config import Go2Config, camera_info_static
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
    def transforms(self) -> list[Transform]:
        if self.odom.value is None:
            return []

        # we just make sure to change timestamps so that we can jump
        # back and forth through time and the viewer doesn't get confused
        odom = self.odom.value
        odom.ts = time.time()
        statics = [
            Transform(
                translation=transform.translation,
                rotation=transform.rotation,
                frame_id=transform.frame_id,
                child_frame_id=transform.child_frame_id,
                ts=odom.ts,
            )
            for transform in Go2Config.static_transforms.values()
        ]
        return [Transform.from_pose(Go2Config.body_frame, odom), *statics]

    def publish(self) -> None:
        t = TF()
        t.publish(*self.transforms)
        t.stop()

        camera_info = camera_info_static()
        camera_info.ts = time.time()
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
