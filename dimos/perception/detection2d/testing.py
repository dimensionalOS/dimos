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
import hashlib
import os
import time
from pathlib import Path
from typing import Optional, TypedDict, Union

from dimos_lcm.foxglove_msgs.ImageAnnotations import ImageAnnotations
from dimos_lcm.foxglove_msgs.SceneUpdate import SceneUpdate
from dimos_lcm.visualization_msgs.MarkerArray import MarkerArray

from dimos.core.transport import LCMTransport
from dimos.hardware.camera import zed
from dimos.msgs.geometry_msgs import Transform
from dimos.msgs.sensor_msgs import CameraInfo, PointCloud2
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.tf2_msgs import TFMessage
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.moduleDB import ObjectDBModule
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


tf = TF()


def get_g1_moment(**kwargs):
    seek = kwargs.get("seek", 10.0)

    data_dir = "replay_g1"
    get_data(data_dir)

    lidar_frame = PointCloud2.lcm_decode(
        TimedSensorReplay(f"{data_dir}/map#sensor_msgs.PointCloud2").find_closest_seek(seek)
    )

    tf_replay = TimedSensorReplay(f"{data_dir}/tf#tf2_msgs.TFMessage")
    tf = TF()
    tf.start()

    tf_window = 1.5
    for timestamp, tf_frame in tf_replay.iterate_ts(seek=seek - tf_window, duration=tf_window):
        tf.publish(*TFMessage.lcm_decode(tf_frame).transforms)

    print(tf)
    image_frame = Image.lcm_decode(
        TimedSensorReplay(f"{data_dir}/image#sensor_msgs.Image").find_closest_seek(seek)
    )

    return {
        "lidar_frame": lidar_frame,
        "image_frame": image_frame,
        "camera_info": zed.CameraInfo.SingleWebcam,
        "tf": tf,
    }


def get_moment(**kwargs) -> Moment:
    seek = kwargs.get("seek", 10.0)
    g1 = kwargs.get("g1", False)
    if g1:
        return get_g1_moment(**kwargs)

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

    tf.publish(*transforms)

    return {
        "odom_frame": odom_frame,
        "lidar_frame": lidar_frame,
        "image_frame": image_frame,
        "camera_info": ConnectionModule._camera_info(),
        "transforms": transforms,
        "tf": tf,
    }


# Create a single instance of Detection2DModule
_detection2d_module = None
_objectdb_module = None


def detections2d(**kwargs) -> Moment2D:
    global _detection2d_module
    moment = get_moment(**kwargs)
    if _detection2d_module is None:
        _detection2d_module = Detection2DModule()

    return {
        **moment,
        "detections2d": _detection2d_module.process_image_frame(moment["image_frame"]),
    }


# Create a single instance of Detection3DModule
_detection3d_module = None


def detections3d(**kwargs) -> Moment3D:
    global _detection3d_module

    moment = detections2d(**kwargs)

    camera_transform = moment["tf"].get("camera_optical", moment.get("lidar_frame").frame_id)
    if camera_transform is None:
        raise ValueError("No camera_optical transform in tf")

    if _detection3d_module is None:
        _detection3d_module = Detection3DModule(camera_info=moment["camera_info"])

    return {
        **moment,
        "detections3d": _detection3d_module.process_frame(
            moment["detections2d"], moment["lidar_frame"], camera_transform
        ),
    }


def objectdb(**kwargs) -> Moment3D:
    global _objectdb_module

    moment = detections3d(**kwargs)
    camera_transform = moment["tf"].get("camera_optical", moment.get("lidar_frame").frame_id)
    if camera_transform is None:
        raise ValueError("No camera_optical transform in tf")

    if _objectdb_module is None:
        _objectdb_module = ObjectDBModule(
            camera_info=moment["camera_info"], goto=lambda x: print("GOTO", x)
        )

    for detection in moment["detections3d"]:
        _objectdb_module.add_detection(detection)

    return {**moment, "objectdb": _objectdb_module}


# Create transport instances once and reuse them
_transports = {}


def _get_transport(topic: str, msg_type):
    """Get or create a transport for the given topic."""
    key = (topic, msg_type)
    if key not in _transports:
        _transports[key] = LCMTransport(topic, msg_type)
    return _transports[key]


def publish_moment(moment: Union[Moment, Moment2D, Moment3D]):
    lidar_frame_transport = _get_transport("/lidar", LidarMessage)
    lidar_frame_transport.publish(moment.get("lidar_frame"))

    image_frame_transport = _get_transport("/image", Image)
    image_frame_transport.publish(moment.get("image_frame"))

    camera_info_transport = _get_transport("/camera_info", CameraInfo)
    camera_info_transport.publish(moment.get("camera_info"))

    odom_frame = moment.get("odom_frame")
    if odom_frame:
        odom_frame_transport = _get_transport("/odom", Odometry)
        odom_frame_transport.publish(moment.get("odom_frame"))

    moment.get("tf").publish_all()
    time.sleep(0.1)
    moment.get("tf").publish_all()

    detections2d: ImageDetections2D = moment.get("detections2d")
    if detections2d:
        annotations_transport = _get_transport("/annotations", ImageAnnotations)
        annotations_transport.publish(detections2d.to_foxglove_annotations())

    detections3d: ImageDetections3D = moment.get("detections3d")
    if detections3d:
        for index, detection in enumerate(detections3d[:3]):
            pointcloud_topic = _get_transport(f"/detected/pointcloud/{index}", PointCloud2)
            image_topic = _get_transport(f"/detected/image/{index}", Image)
            pointcloud_topic.publish(detection.pointcloud)
            image_topic.publish(detection.cropped_image())

        # scene_entity_transport = _get_transport("/scene_update", SceneUpdate)
        # scene_entity_transport.publish(detections3d.to_foxglove_scene_update())

    objectdb: ObjectDBModule = moment.get("objectdb")
    if objectdb:
        print("PUB OBJECT DB", list(objectdb.objects.keys()))
        scene_entity_transport = _get_transport("/scene_update", SceneUpdate)
        scene_entity_transport.publish(objectdb.to_foxglove_scene_update())
