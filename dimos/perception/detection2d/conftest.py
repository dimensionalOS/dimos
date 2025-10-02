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

from typing import Callable, Optional, TypedDict

import pytest
from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
from dimos_lcm.foxglove_msgs.SceneUpdate import SceneUpdate
from dimos_lcm.sensor_msgs import CameraInfo, PointCloud2
from dimos_lcm.visualization_msgs.MarkerArray import MarkerArray

from dimos.core import start
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.testing import Moment, Moment2D, Moment3D
from dimos.protocol.tf import TF
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay


@pytest.fixture
def dimos_cluster():
    dimos = start(5)
    yield dimos
    dimos.stop()


@pytest.fixture
def tf():
    return TF(autostart=False)


@pytest.fixture
def get_moment(tf):
    def moment_provider(**kwargs) -> Moment:
        seek = kwargs.get("seek", 10.0)

        data_dir = "unitree_go2_lidar_corrected"
        get_data(data_dir)

        lidar_frame = TimedSensorReplay(f"{data_dir}/lidar").find_closest_seek(seek)

        image_frame = TimedSensorReplay(
            f"{data_dir}/video",
        ).find_closest(lidar_frame.ts)

        image_frame.frame_id = "camera_optical"

        odom_frame = TimedSensorReplay(f"{data_dir}/odom", autocast=Odometry.from_msg).find_closest(
            lidar_frame.ts
        )

        transforms = ConnectionModule._odom_to_tf(odom_frame)

        tf.receive_transform(*transforms)
        return {
            "odom_frame": odom_frame,
            "lidar_frame": lidar_frame,
            "image_frame": image_frame,
            "camera_info": ConnectionModule._camera_info(),
            "transforms": transforms,
            "tf": tf,
        }

    return moment_provider


@pytest.fixture
def get_moment_2d(get_moment) -> Callable[[], Moment2D]:
    module = Detection2DModule()

    def moment_provider(**kwargs) -> Moment2D:
        moment = get_moment(**kwargs)
        detections = module.process_image_frame(moment.get("image_frame"))

        return {
            **moment,
            "detections2d": detections,
        }

    yield moment_provider
    module._close_module()
