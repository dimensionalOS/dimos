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

from collections.abc import Callable, Generator
import functools
from typing import TypedDict
from unittest import mock

from dimos_lcm.visualization_msgs.MarkerArray import MarkerArray
import pytest

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.module2D import Detection2DModule
from dimos.perception.detection.module3D import Detection3DModule
from dimos.perception.detection.type.detection2d.base import Detection2D
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.protocol.tf.tf import TF
from dimos.robot.unitree.go2 import connection
from dimos.robot.unitree.type.odometry import Odometry
from dimos.utils.data import get_data
from dimos.utils.testing.legacy_pickle import LegacyPickleStore


class Moment(TypedDict, total=False):
    odom_frame: Odometry
    lidar_frame: PointCloud2
    image_frame: Image
    camera_info: CameraInfo
    transforms: list[Transform]
    tf: TF
    detections: ImageDetections3DPC | None
    markers: MarkerArray | None


class Moment2D(Moment):
    detections2d: ImageDetections2D


class Moment3D(Moment):
    detections3dpc: ImageDetections3DPC


@pytest.fixture(scope="session")
def tf():
    t = TF()
    yield t
    t.stop()


@pytest.fixture(scope="session")
def get_moment(tf):
    @functools.lru_cache(maxsize=1)
    def moment_provider(**kwargs) -> Moment:
        print("MOMENT PROVIDER ARGS:", kwargs)
        seek = kwargs.get("seek", 10.0)

        data_dir = "unitree_go2_lidar_corrected"
        get_data(data_dir)

        lidar_frame_result = LegacyPickleStore(f"{data_dir}/lidar").find_closest_seek(seek)
        if lidar_frame_result is None:
            raise ValueError("No lidar frame found")
        lidar_frame: PointCloud2 = lidar_frame_result

        image_frame = LegacyPickleStore(
            f"{data_dir}/video",
        ).find_closest(lidar_frame.ts)

        if image_frame is None:
            raise ValueError("No image frame found")

        image_frame.frame_id = "camera_optical"

        odom_frame = LegacyPickleStore(f"{data_dir}/odom", autocast=Odometry.from_msg).find_closest(
            lidar_frame.ts
        )

        if odom_frame is None:
            raise ValueError("No odom frame found")

        transforms = connection.GO2Connection._odom_to_tf(odom_frame)

        tf.receive_transform(*transforms)

        return {
            "odom_frame": odom_frame,
            "lidar_frame": lidar_frame,
            "image_frame": image_frame,
            "camera_info": connection._camera_info_static(),
            "transforms": transforms,
            "tf": tf,
        }

    yield moment_provider
    moment_provider.cache_clear()


@pytest.fixture(scope="session")
def publish_moment():
    def publisher(moment: Moment | Moment2D | Moment3D) -> None:
        detections2d_val = moment.get("detections2d")
        if detections2d_val:
            detections: LCMTransport[Detection2DArray] = LCMTransport(
                "/detections", Detection2DArray
            )
            assert isinstance(detections2d_val, ImageDetections2D)
            detections.publish(detections2d_val.to_ros_detection2d_array())
            detections.lcm.stop()

        lidar_frame = moment.get("lidar_frame")
        if lidar_frame:
            lidar: LCMTransport[PointCloud2] = LCMTransport("/lidar", PointCloud2)
            lidar.publish(lidar_frame)
            lidar.lcm.stop()

        image_frame = moment.get("image_frame")
        if image_frame:
            image: LCMTransport[Image] = LCMTransport("/image", Image)
            image.publish(image_frame)
            image.lcm.stop()

        camera_info_val = moment.get("camera_info")
        if camera_info_val:
            camera_info: LCMTransport[CameraInfo] = LCMTransport("/camera_info", CameraInfo)
            camera_info.publish(camera_info_val)
            camera_info.lcm.stop()

        tf = moment.get("tf")
        transforms = moment.get("transforms")
        if tf is not None and transforms is not None:
            tf.publish(*transforms)

    return publisher


@pytest.fixture(scope="session")
def imageDetections2d(get_moment_2d) -> ImageDetections2D:
    moment = get_moment_2d()
    assert len(moment["detections2d"]) > 0, "No detections found in the moment"
    return moment["detections2d"]


@pytest.fixture(scope="session")
def detection2d(get_moment_2d) -> Detection2D:
    moment = get_moment_2d()
    assert len(moment["detections2d"]) > 0, "No detections found in the moment"
    return moment["detections2d"][0]


@pytest.fixture(scope="session")
def detections3dpc(get_moment_3dpc) -> Detection3DPC:
    moment = get_moment_3dpc(seek=10.0)
    assert len(moment["detections3dpc"]) > 0, "No detections found in the moment"
    return moment["detections3dpc"]


@pytest.fixture(scope="session")
def detection3dpc(detections3dpc) -> Detection3DPC:
    return detections3dpc[0]


@pytest.fixture(scope="session")
def get_moment_2d(get_moment) -> Generator[Callable[[], Moment2D], None, None]:
    from dimos.perception.detection.detectors.yolo import Yolo2DDetector

    c = mock.create_autospec(CameraInfo, spec_set=True, instance=True)
    module = Detection2DModule(detector=lambda: Yolo2DDetector(device="cpu"), camera_info=c)

    @functools.lru_cache(maxsize=1)
    def moment_provider(**kwargs) -> Moment2D:
        moment = get_moment(**kwargs)
        detections = module.process_image_frame(moment.get("image_frame"))

        return {
            **moment,
            "detections2d": detections,
        }

    yield moment_provider

    moment_provider.cache_clear()
    module._close_module()


@pytest.fixture(scope="session")
def get_moment_3dpc(get_moment_2d) -> Generator[Callable[[], Moment3D], None, None]:
    module: Detection3DModule | None = None

    @functools.lru_cache(maxsize=1)
    def moment_provider(**kwargs) -> Moment3D:
        nonlocal module
        moment = get_moment_2d(**kwargs)

        if not module:
            module = Detection3DModule(camera_info=moment["camera_info"])

        lidar_frame = moment.get("lidar_frame")
        if lidar_frame is None:
            raise ValueError("No lidar frame found")

        camera_transform = moment["tf"].get("camera_optical", lidar_frame.frame_id)
        if camera_transform is None:
            raise ValueError("No camera_optical transform in tf")

        detections3dpc = module.process_frame(
            moment["detections2d"], moment["lidar_frame"], camera_transform
        )

        return {
            **moment,
            "detections3dpc": detections3dpc,
        }

    yield moment_provider
    moment_provider.cache_clear()
    if module is not None:
        module._close_module()
