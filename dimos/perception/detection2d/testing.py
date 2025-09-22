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

import functools
from typing import Optional, TypedDict, Union

from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
from dimos_lcm.foxglove_msgs.SceneUpdate import SceneUpdate
from dimos_lcm.sensor_msgs import CameraInfo, PointCloud2
from dimos_lcm.visualization_msgs.MarkerArray import MarkerArray

from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.type import ImageDetections2D, ImageDetections3D
from dimos.protocol.service import lcmservice as lcm
from dimos.protocol.tf import TF
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule
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
    annotations: Optional[ImageAnnotations]
    detections: Optional[ImageDetections3D]
    markers: Optional[MarkerArray]
    scene_update: Optional[SceneUpdate]


class Moment2D(Moment):
    detections2d: ImageDetections2D


class Moment3D(Moment):
    detections3d: ImageDetections3D


@functools.lru_cache
def get_moment(seek: float = 10):
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

    tf = TF()
    tf.publish(*transforms)

    return {
        "odom_frame": odom_frame,
        "lidar_frame": lidar_frame,
        "image_frame": image_frame,
        "camera_info": ConnectionModule._camera_info(),
        "transforms": transforms,
        "tf": tf,
    }


@functools.lru_cache
def detections2d(get_moment: Moment, seek: float = 10.0) -> Moment2D:
    moment = get_moment(seek=seek)
    return {
        **moment,
        "detections2d": Detection2DModule().process_image_frame(moment["image_frame"]),
    }


@functools.lru_cache
def detections3d(seek: float = 10.0) -> Moment3D:
    moment = detections2d(get_moment, seek=seek)
    camera_transform = moment["tf"].get("camera_optical", "world")
    if camera_transform is None:
        raise ValueError("No camera_optical transform in tf")

    return {
        **moment,
        "detections3d": Detection3DModule(camera_info=moment["camera_info"]).process_frame(
            moment["detections2d"], moment["lidar_frame"], camera_transform
        ),
    }


def publish_moment(moment: Union[Moment, Moment2D, Moment3D]):
    lcm.autoconf()

    lidar_frame_transport: LCMTransport = LCMTransport("/lidar", LidarMessage)
    lidar_frame_transport.publish(moment.get("lidar_frame"))

    image_frame_transport: LCMTransport = LCMTransport("/image", Image)
    image_frame_transport.publish(moment.get("image_frame"))

    odom_frame_transport: LCMTransport = LCMTransport("/odom", Odometry)
    odom_frame_transport.publish(moment.get("odom_frame"))

    camera_info_transport: LCMTransport = LCMTransport("/camera_info", CameraInfo)
    camera_info_transport.publish(moment.get("camera_info"))

    detections2d: ImageDetections2D = moment.get("detections2d")
    if detections2d:
        annotations_transport: LCMTransport = LCMTransport("/annotations", ImageAnnotations)
        annotations_transport.publish(detections2d.to_image_annotations())

    detections3d: ImageDetections3D = moment.get("detections3d")
    if detections3d:
        for index, detection in enumerate(detections3d[:3]):
            pointcloud_topic = LCMTransport("/detected/pointcloud/" + str(index), PointCloud2)
            image_topic = LCMTransport("/detected/image/" + str(index), Image)
            pointcloud_topic.publish(detection.pointcloud)
            image_topic.publish(detection.cropped_image())

        scene_entity_transport: LCMTransport = LCMTransport("/scene_update", SceneUpdate)
        scene_entity_transport.publish(detections3d.to_foxglove_scene_update())
