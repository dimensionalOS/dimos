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

import pickle

from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]

from dimos.msgs.sensor_msgs import Image
from dimos.msgs.std_msgs import Header
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry

image_resize_factor = 1
originalwidth, originalheight = (1280, 720)


def camera_info() -> CameraInfo:
    fx, fy, cx, cy = list(
        map(
            lambda x: int(x / image_resize_factor),
            [819.553492, 820.646595, 625.284099, 336.808987],
        )
    )
    width, height = tuple(
        map(
            lambda x: int(x / image_resize_factor),
            [originalwidth, originalheight],
        )
    )

    # Camera matrix K (3x3)
    K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]

    # No distortion coefficients for now
    D = [0.0, 0.0, 0.0, 0.0, 0.0]

    # Identity rotation matrix
    R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    # Projection matrix P (3x4)
    P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]

    base_msg = {
        "D_length": len(D),
        "height": height,
        "width": width,
        "distortion_model": "plumb_bob",
        "D": D,
        "K": K,
        "R": R,
        "P": P,
        "binning_x": 0,
        "binning_y": 0,
    }

    return CameraInfo(
        **base_msg,
        header=Header("camera_optical"),
    )


def transform_chain(odom_frame: Odometry) -> list:  # type: ignore[type-arg]
    from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
    from dimos.protocol.tf import TF

    camera_link = Transform(
        translation=Vector3(0.3, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="base_link",
        child_frame_id="camera_link",
        ts=odom_frame.ts,
    )

    camera_optical = Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
        frame_id="camera_link",
        child_frame_id="camera_optical",
        ts=camera_link.ts,
    )

    tf = TF()
    tf.publish(
        Transform.from_pose("base_link", odom_frame),
        camera_link,
        camera_optical,
    )

    return tf  # type: ignore[return-value]


def broadcast(  # type: ignore[no-untyped-def]
    timestamp: float,
    lidar_frame: LidarMessage,
    video_frame: Image,
    odom_frame: Odometry,
    detections,
    annotations,
) -> None:
    from dimos_lcm.foxglove_msgs.ImageAnnotations import (  # type: ignore[import-untyped]
        ImageAnnotations,
    )

    from dimos.core import LCMTransport
    from dimos.msgs.geometry_msgs import PoseStamped

    lidar_transport = LCMTransport("/lidar", LidarMessage)  # type: ignore[var-annotated]
    odom_transport = LCMTransport("/odom", PoseStamped)  # type: ignore[var-annotated]
    video_transport = LCMTransport("/image", Image)  # type: ignore[var-annotated]
    camera_info_transport = LCMTransport("/camera_info", CameraInfo)  # type: ignore[var-annotated]

    lidar_transport.broadcast(None, lidar_frame)
    video_transport.broadcast(None, video_frame)
    odom_transport.broadcast(None, odom_frame)
    camera_info_transport.broadcast(None, camera_info())

    transform_chain(odom_frame)

    print(lidar_frame)
    print(video_frame)
    print(odom_frame)
    video_transport = LCMTransport("/image", Image)
    annotations_transport = LCMTransport("/annotations", ImageAnnotations)  # type: ignore[var-annotated]
    annotations_transport.broadcast(None, annotations)


def process_data():  # type: ignore[no-untyped-def]
    from dimos.msgs.sensor_msgs import Image
    from dimos.perception.detection.module2D import (  # type: ignore[attr-defined]
        Detection2DModule,
        build_imageannotations,
    )
    from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
    from dimos.robot.unitree_webrtc.type.odometry import Odometry
    from dimos.utils.data import get_data
    from dimos.utils.testing import TimedSensorReplay

    get_data("unitree_office_walk")
    target = 1751591272.9654856
    lidar_store = TimedSensorReplay("unitree_office_walk/lidar", autocast=LidarMessage.from_msg)
    video_store = TimedSensorReplay("unitree_office_walk/video", autocast=Image.from_numpy)
    odom_store = TimedSensorReplay("unitree_office_walk/odom", autocast=Odometry.from_msg)

    def attach_frame_id(image: Image) -> Image:
        image.frame_id = "camera_optical"
        return image

    lidar_frame = lidar_store.find_closest(target, tolerance=1)
    video_frame = attach_frame_id(video_store.find_closest(target, tolerance=1))  # type: ignore[arg-type]
    odom_frame = odom_store.find_closest(target, tolerance=1)

    detector = Detection2DModule()
    detections = detector.detect(video_frame)  # type: ignore[attr-defined]
    annotations = build_imageannotations(detections)

    data = (target, lidar_frame, video_frame, odom_frame, detections, annotations)

    with open("filename.pkl", "wb") as file:
        pickle.dump(data, file)

    return data


def main() -> None:
    try:
        with open("filename.pkl", "rb") as file:
            data = pickle.load(file)
    except FileNotFoundError:
        print("Processing data and creating pickle file...")
        data = process_data()  # type: ignore[no-untyped-call]
    broadcast(*data)


main()
