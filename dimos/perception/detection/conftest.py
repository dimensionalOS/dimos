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

from collections.abc import Callable, Generator
import functools
from typing import TypedDict

from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
from dimos_lcm.foxglove_msgs.SceneUpdate import SceneUpdate
from dimos_lcm.visualization_msgs.MarkerArray import MarkerArray
import pytest

from dimos.core import LCMTransport
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import CameraInfo, Image, PointCloud2
from dimos.msgs.vision_msgs import Detection2DArray
from dimos.perception.detection.module2D import Detection2DModule
from dimos.perception.detection.module3D import Detection3DModule
from dimos.perception.detection.moduleDB import ObjectDBModule
from dimos.perception.detection.type import (
    Detection2D,
    Detection3DPC,
    ImageDetections2D,
    ImageDetections3DPC,
)
from dimos.protocol.tf import TF
from dimos.robot.unitree.connection import go2
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils.data import get_data
from dimos.utils.testing import TimedSensorReplay


class Moment(TypedDict, total=False):
    odom_frame: Odometry
    lidar_frame: LidarMessage
    image_frame: Image
    camera_info: CameraInfo
    transforms: list[Transform]
    tf: TF
    annotations: ImageAnnotations | None
    detections: ImageDetections3DPC | None
    markers: MarkerArray | None
    scene_update: SceneUpdate | None


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

        lidar_frame_result = TimedSensorReplay(f"{data_dir}/lidar").find_closest_seek(seek)
        if lidar_frame_result is None:
            raise ValueError("No lidar frame found")
        lidar_frame: LidarMessage = lidar_frame_result

        image_frame = TimedSensorReplay(
            f"{data_dir}/video",
        ).find_closest(lidar_frame.ts)

        if image_frame is None:
            raise ValueError("No image frame found")

        image_frame.frame_id = "camera_optical"

        odom_frame = TimedSensorReplay(f"{data_dir}/odom", autocast=Odometry.from_msg).find_closest(
            lidar_frame.ts
        )

        if odom_frame is None:
            raise ValueError("No odom frame found")

        transforms = go2.GO2Connection._odom_to_tf(odom_frame)

        tf.receive_transform(*transforms)

        return {
            "odom_frame": odom_frame,
            "lidar_frame": lidar_frame,
            "image_frame": image_frame,
            "camera_info": go2._camera_info_static(),
            "transforms": transforms,
            "tf": tf,
        }

    return moment_provider


@pytest.fixture(scope="session")
def publish_moment():
    def publisher(moment: Moment | Moment2D | Moment3D) -> None:
        detections2d_val = moment.get("detections2d")
        if detections2d_val:
            # 2d annotations
            annotations: LCMTransport[ImageAnnotations] = LCMTransport(
                "/annotations", ImageAnnotations
            )
            assert isinstance(detections2d_val, ImageDetections2D)
            annotations.publish(detections2d_val.to_foxglove_annotations())

            detections: LCMTransport[Detection2DArray] = LCMTransport(
                "/detections", Detection2DArray
            )
            detections.publish(detections2d_val.to_ros_detection2d_array())

            annotations.lcm.stop()
            detections.lcm.stop()

        detections3dpc_val = moment.get("detections3dpc")
        if detections3dpc_val:
            scene_update: LCMTransport[SceneUpdate] = LCMTransport("/scene_update", SceneUpdate)
            # 3d scene update
            assert isinstance(detections3dpc_val, ImageDetections3DPC)
            scene_update.publish(detections3dpc_val.to_foxglove_scene_update())
            scene_update.lcm.stop()

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

    # moduleDB.scene_update.transport = LCMTransport("/scene_update", SceneUpdate)
    # moduleDB.target.transport = LCMTransport("/target", PoseStamped)

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
    from dimos.perception.detection.detectors import Yolo2DDetector

    module = Detection2DModule(detector=lambda: Yolo2DDetector(device="cpu"))

    @functools.lru_cache(maxsize=1)
    def moment_provider(**kwargs) -> Moment2D:
        moment = get_moment(**kwargs)
        detections = module.process_image_frame(moment.get("image_frame"))

        return {
            **moment,
            "detections2d": detections,
        }

    yield moment_provider

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
    if module is not None:
        module._close_module()


@pytest.fixture(scope="session")
def object_db_module(get_moment):
    """Create and populate an ObjectDBModule with detections from multiple frames."""
    from dimos.perception.detection.detectors import Yolo2DDetector

    module2d = Detection2DModule(detector=lambda: Yolo2DDetector(device="cpu"))
    module3d = Detection3DModule(camera_info=go2._camera_info_static())
    moduleDB = ObjectDBModule(camera_info=go2._camera_info_static())

    # Process 5 frames to build up object history
    for i in range(5):
        seek_value = 10.0 + (i * 2)
        moment = get_moment(seek=seek_value)

        # Process 2D detections
        imageDetections2d = module2d.process_image_frame(moment["image_frame"])

        # Get camera transform
        camera_transform = moment["tf"].get("camera_optical", moment.get("lidar_frame").frame_id)

        # Process 3D detections
        imageDetections3d = module3d.process_frame(
            imageDetections2d, moment["lidar_frame"], camera_transform
        )

        # Add to database
        moduleDB.add_detections(imageDetections3d)

    yield moduleDB

    module2d._close_module()
    module3d._close_module()
    moduleDB._close_module()


@pytest.fixture(scope="session")
def first_object(object_db_module):
    """Get the first object from the database."""
    objects = list(object_db_module.objects.values())
    assert len(objects) > 0, "No objects found in database"
    return objects[0]


@pytest.fixture(scope="session")
def all_objects(object_db_module):
    """Get all objects from the database."""
    return list(object_db_module.objects.values())
