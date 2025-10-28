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

import logging
from threading import Thread
import time
from typing import Protocol

from dimos_lcm.sensor_msgs import CameraInfo
from reactivex.observable import Observable

from dimos import spec
from dimos.core import DimosCluster, In, LCMTransport, Module, Out, pSHMTransport, rpc
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    TwistStamped,
    Vector3,
)
from dimos.msgs.sensor_msgs import Image, PointCloud2
from dimos.msgs.std_msgs import Header
from dimos.robot.unitree.connection.connection import UnitreeWebRTCConnection
from dimos.utils.data import get_data
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing import TimedSensorReplay

logger = setup_logger(__file__, level=logging.INFO)


class Go2ConnectionProtocol(Protocol):
    """Protocol defining the interface for Go2 robot connections."""

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def lidar_stream(self) -> Observable: ...
    def odom_stream(self) -> Observable: ...
    def video_stream(self) -> Observable: ...
    def move(self, twist: TwistStamped, duration: float = 0.0) -> bool: ...
    def standup(self) -> None: ...
    def liedown(self) -> None: ...
    def publish_request(self, topic: str, data: dict) -> dict: ...


def _camera_info() -> CameraInfo:
    fx, fy, cx, cy = (819.553492, 820.646595, 625.284099, 336.808987)
    width, height = (1280, 720)

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

    return CameraInfo(**base_msg, header=Header("camera_optical"))


camera_info = _camera_info()


class ReplayConnection(UnitreeWebRTCConnection):
    dir_name = "unitree_go2_office_walk2"

    # we don't want UnitreeWebRTCConnection to init
    def __init__(
        self,
        **kwargs,
    ) -> None:
        get_data(self.dir_name)
        self.replay_config = {
            "loop": kwargs.get("loop"),
            "seek": kwargs.get("seek"),
            "duration": kwargs.get("duration"),
        }

    def connect(self) -> None:
        pass

    def start(self) -> None:
        pass

    def standup(self) -> None:
        print("standup suppressed")

    def liedown(self) -> None:
        print("liedown suppressed")

    @simple_mcache
    def lidar_stream(self):
        print("lidar stream start")
        lidar_store = TimedSensorReplay(f"{self.dir_name}/lidar")
        return lidar_store.stream(**self.replay_config)

    @simple_mcache
    def odom_stream(self):
        print("odom stream start")
        odom_store = TimedSensorReplay(f"{self.dir_name}/odom")
        return odom_store.stream(**self.replay_config)

    # we don't have raw video stream in the data set
    @simple_mcache
    def video_stream(self):
        print("video stream start")
        video_store = TimedSensorReplay(f"{self.dir_name}/video")

        return video_store.stream(**self.replay_config)

    def move(self, twist: TwistStamped, duration: float = 0.0) -> None:
        pass

    def publish_request(self, topic: str, data: dict):
        """Fake publish request for testing."""
        return {"status": "ok", "message": "Fake publish"}


class GO2Connection(Module, spec.Camera, spec.Pointcloud):
    cmd_vel: In[TwistStamped] = None  # type: ignore
    pointcloud: Out[PointCloud2] = None  # type: ignore
    image: Out[Image] = None  # type: ignore
    camera_info_stream: Out[CameraInfo] = None  # type: ignore
    connection_type: str = "webrtc"

    connection: Go2ConnectionProtocol

    ip: str | None

    camera_info: CameraInfo = camera_info

    def __init__(
        self,
        ip: str | None = None,
        *args,
        **kwargs,
    ) -> None:
        match ip:
            case None | "fake" | "mock" | "replay":
                self.connection = ReplayConnection()
            case "mujoco":
                from dimos.robot.unitree_webrtc.mujoco_connection import MujocoConnection

                self.connection = MujocoConnection()
            case _:
                self.connection = UnitreeWebRTCConnection(ip)

        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self) -> None:
        """Start the connection and subscribe to sensor streams."""
        super().start()

        self.connection.start()

        self._disposables.add(
            self.connection.lidar_stream().subscribe(self.pointcloud.publish),
        )

        self._disposables.add(
            self.connection.odom_stream().subscribe(self._publish_tf),
        )

        self._disposables.add(
            self.connection.video_stream().subscribe(self.image.publish),
        )

        self.cmd_vel.subscribe(self.move)

        self._camera_info_thread = Thread(
            target=self.publish_camera_info,
            daemon=True,
        )
        self._camera_info_thread.start()

        self.standup()

    @rpc
    def stop(self) -> None:
        self.liedown()
        if self.connection:
            self.connection.stop()
        if hasattr(self, "_camera_info_thread"):
            self._camera_info_thread.join(timeout=1.0)
        super().stop()

    @classmethod
    def _odom_to_tf(cls, odom: PoseStamped) -> list[Transform]:
        camera_link = Transform(
            translation=Vector3(0.3, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=odom.ts,
        )

        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id="camera_link",
            child_frame_id="camera_optical",
            ts=odom.ts,
        )

        sensor = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="world",
            child_frame_id="sensor",
            ts=odom.ts,
        )

        return [
            Transform.from_pose("base_link", odom),
            camera_link,
            camera_optical,
            sensor,
        ]

    def _publish_tf(self, msg) -> None:
        self.tf.publish(*self._odom_to_tf(msg))

    def publish_camera_info(self) -> None:
        while True:
            self.camera_info_stream.publish(camera_info)
            time.sleep(1.0)

    @rpc
    def move(self, twist: TwistStamped, duration: float = 0.0) -> None:
        """Send movement command to robot."""
        self.connection.move(twist, duration)

    @rpc
    def standup(self):
        """Make the robot stand up."""
        return self.connection.standup()

    @rpc
    def liedown(self):
        """Make the robot lie down."""
        return self.connection.liedown()

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


def deploy(dimos: DimosCluster, ip: str, prefix: str = "") -> GO2Connection:
    from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE

    connection = dimos.deploy(GO2Connection, ip)

    connection.pointcloud.transport = pSHMTransport(
        f"{prefix}/lidar", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    )
    connection.image.transport = pSHMTransport(
        f"{prefix}/image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    )

    connection.cmd_vel.transport = LCMTransport(f"{prefix}/cmd_vel", TwistStamped)

    connection.camera_info_stream.transport = LCMTransport(f"{prefix}/camera_info", CameraInfo)
    connection.start()

    return connection
