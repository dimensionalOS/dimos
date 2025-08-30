#!/usr/bin/env python3

#!/usr/bin/env python3

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
import logging
import pickle
import time
import warnings

import reactivex as rx
from dimos_lcm.sensor_msgs import CameraInfo
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat, sharpness_window
from dimos.msgs.std_msgs import Header
from dimos.robot.unitree_webrtc.connection import SerializableVideoFrame, UnitreeWebRTCConnection
from dimos.robot.unitree_webrtc.modular.frame_alignment import FrameAlignment
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing import TimedSensorReplay

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_go2", level=logging.INFO)

# Suppress verbose loggers
logging.getLogger("aiortc.codecs.h264").setLevel(logging.ERROR)
logging.getLogger("lcm_foxglove_bridge").setLevel(logging.ERROR)
logging.getLogger("websockets.server").setLevel(logging.ERROR)
logging.getLogger("FoxgloveServer").setLevel(logging.ERROR)
logging.getLogger("asyncio").setLevel(logging.ERROR)
logging.getLogger("root").setLevel(logging.WARNING)

# Suppress warnings
warnings.filterwarnings("ignore", message="coroutine.*was never awaited")
warnings.filterwarnings("ignore", message="H264Decoder.*failed to decode")

image_resize_factor = 4
originalwidth, originalheight = (1280, 720)
get_data("unitree_raw_webrtc_replay")


class FakeRTC(UnitreeWebRTCConnection):
    # we don't want UnitreeWebRTCConnection to init
    def __init__(self, *args, **kwargs):
        pass

    def connect(self):
        pass

    def start(self):
        pass

    def standup(self):
        print("standup suppressed")

    def liedown(self):
        print("liedown suppressed")

    @functools.cache
    def raw_lidar_stream(self):
        print("lidar stream start")
        lidar_store = TimedSensorReplay("unitree_raw_webrtc_replay/lidar")
        return lidar_store.stream()

    @functools.cache
    def raw_odom_stream(self):
        print("odom stream start")
        odom_store = TimedSensorReplay("unitree_raw_webrtc_replay/odom")
        return odom_store.stream()

    # we don't have raw video stream in the data set
    @functools.cache
    def raw_video_stream(self):
        def maybe_cast_avframe(frame) -> Image:
            if isinstance(frame, SerializableVideoFrame):
                image = Image.from_numpy(frame.to_ndarray(), ImageFormat.BGR, ts=frame.time)
                return image

            return Image.from_numpy(frame)

        print("video stream start")
        video_store = TimedSensorReplay(
            "unitree_raw_webrtc_replay/video", autocast=maybe_cast_avframe
        )
        return video_store.stream()

    @functools.cache
    def video_stream(self):
        return self.raw_video_stream()

    def move(self, vector: Vector3, duration: float = 0.0):
        pass

    def publish_request(self, topic: str, data: dict):
        """Fake publish request for testing."""
        return {"status": "ok", "message": "Fake publish"}


class ConnectionModule(Module):
    ip: str
    connection_type: str = "webrtc"
    camera_info: Out[CameraInfo] = None
    odom: Out[PoseStamped] = None
    lidar: Out[LidarMessage] = None
    change: Out[Vector3] = None
    video: Out[Image] = None
    movecmd: In[Vector3] = None

    def __init__(self, ip: str = None, connection_type: str = "webrtc", *args, **kwargs):
        self.ip = ip
        self.connection_type = connection_type
        self.connection = None
        Module.__init__(self, *args, **kwargs)

    @rpc
    def record(self, recording_name: str):
        from dimos.utils.testing import TimedSensorStorage

        lidar_store = TimedSensorStorage(f"{recording_name}/lidar")
        lidar_store.save_stream(self.connection.raw_lidar_stream()).subscribe(lambda x: x)

        odom_store = TimedSensorStorage(f"{recording_name}/odom")
        odom_store.save_stream(self.connection.raw_odom_stream()).subscribe(lambda x: x)

        video_store = TimedSensorStorage(f"{recording_name}/video")
        video_store.save_stream(self.connection.raw_video_stream()).subscribe(lambda x: x)

    @rpc
    def start(self):
        """Start the connection and subscribe to sensor streams."""
        match self.connection_type:
            case "webrtc":
                self.connection = UnitreeWebRTCConnection(self.ip)
            case "fake":
                self.connection = FakeRTC()
            case "mujoco":
                from dimos.robot.unitree_webrtc.mujoco_connection import MujocoConnection

                self.connection = MujocoConnection()
                self.connection.start()
            case _:
                raise ValueError(f"Unknown connection type: {self.connection_type}")

        def odom_pub(odom):
            self._publish_tf(odom)
            self.odom.publish(odom)

        # Connect sensor streams to outputs
        self.connection.lidar_stream().subscribe(self.lidar.publish)
        self.connection.odom_stream().subscribe(odom_pub)

        def attach_frame_id(image: Image) -> Image:
            image.frame_id = "camera_optical"
            return image.resize(
                int(originalwidth / image_resize_factor), int(originalheight / image_resize_factor)
            )

        frame_alignment = FrameAlignment()
        frame_alignment.add_stream("video", self.connection.video_stream())
        frame_alignment.add_stream("odom", self.connection.odom_stream())
        frame_alignment.start()
        frame_alignment.get_magnitude_stream(
            "video",
        ).pipe(ops.map(lambda x: Vector3([x, 0.0, 0.0]))).subscribe(self.change.publish)

        # emit change vector @ 10hz
        # rx.interval(0.03).pipe(ops.map(lambda _: change)).subscribe(self.change.publish)

        # sharpness_window(
        #    1, self.connection.video_stream().pipe(ops.map(attach_frame_id))
        # ).subscribe(image_pub)

        self.connection.video_stream().pipe(ops.map(attach_frame_id)).subscribe(self.video.publish)
        self.camera_info_stream().subscribe(self.camera_info.publish)
        self.movecmd.subscribe(self.connection.move)

    def _publish_tf(self, msg):
        self.odom.publish(msg)

        camera_link = Transform(
            translation=Vector3(0.3, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=time.time(),
        )

        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id="camera_link",
            child_frame_id="camera_optical",
            ts=camera_link.ts,
        )

        self.tf.publish(
            Transform.from_pose("base_link", msg),
            camera_link,
            camera_optical,
        )

    @rpc
    def publish_request(self, topic: str, data: dict):
        """Publish a request to the WebRTC connection.
        Args:
            topic: The RTC topic to publish to
            data: The data dictionary to publish
        Returns:
            The result of the publish request
        """
        return self.connection.publish_request(topic, data)

    @functools.cache
    def camera_info_stream(self) -> Observable[CameraInfo]:
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

        return rx.interval(1).pipe(
            ops.map(
                lambda x: CameraInfo(
                    **base_msg,
                    header=Header("camera_optical"),
                )
            )
        )
