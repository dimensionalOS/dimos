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

from typing import Callable

import pytest

from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.testing import Moment, Moment2D
from dimos.perception.detection2d.type import Detection2D, Detection3D
from dimos.protocol.tf import TF
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay


@pytest.fixture
def tf():
    t = TF(autostart=False)
    yield t
    t.stop()


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
def detection2d(get_moment_2d) -> Detection2D:
    moment = get_moment_2d(seek=10.0)
    assert len(moment["detections2d"]) > 0, "No detections found in the moment"
    return moment["detections2d"][0]


@pytest.fixture
def detection3d(get_moment_3d) -> Detection3D:
    moment = get_moment_3d(seek=10.0)
    assert len(moment["detections3d"]) > 0, "No detections found in the moment"
    print(moment["detections3d"])
    return moment["detections3d"][0]


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


@pytest.fixture
def get_moment_3d(get_moment_2d) -> Callable[[], Moment2D]:
    module = None

    def moment_provider(**kwargs) -> Moment2D:
        nonlocal module
        moment = get_moment_2d(**kwargs)

        module = Detection3DModule(camera_info=moment["camera_info"])

        camera_transform = moment["tf"].get("camera_optical", moment.get("lidar_frame").frame_id)
        if camera_transform is None:
            raise ValueError("No camera_optical transform in tf")

        return {
            **moment,
            "detections3d": module.process_frame(
                moment["detections2d"], moment["lidar_frame"], camera_transform
            ),
        }

    yield moment_provider

    print("Closing 3D detection module")
    module._close_module()
    module.stop()
    import time

    time.sleep(1)
