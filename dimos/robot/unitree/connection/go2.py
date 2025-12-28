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
from typing import Any, Protocol

from reactivex.disposable import Disposable
from reactivex.observable import Observable

from dimos import spec
from dimos.core import DimosCluster, In, LCMTransport, Module, Out, pSHMTransport, rpc
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    Vector3,
)
from dimos.msgs.sensor_msgs import CameraInfo, Image, PointCloud2
from dimos.robot.unitree.connection.connection import UnitreeWebRTCConnection
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.data import get_data
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing import TimedSensorReplay, TimedSensorStorage

logger = setup_logger(level=logging.INFO)


class Go2ConnectionProtocol(Protocol):
    """Protocol defining the interface for Go2 robot connections."""

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def lidar_stream(self) -> Observable: ...  # type: ignore[type-arg]
    def odom_stream(self) -> Observable: ...  # type: ignore[type-arg]
    def video_stream(self) -> Observable: ...  # type: ignore[type-arg]
    def move(self, twist: Twist, duration: float = 0.0) -> bool: ...
    def standup(self) -> bool: ...
    def liedown(self) -> bool: ...
    def publish_request(self, topic: str, data: dict) -> dict: ...  # type: ignore[type-arg]


def _camera_info_static() -> CameraInfo:
    fx, fy, cx, cy = (819.553492, 820.646595, 625.284099, 336.808987)
    width, height = (1280, 720)

    return CameraInfo(
        frame_id="camera_optical",
        height=height,
        width=width,
        distortion_model="plumb_bob",
        D=[0.0, 0.0, 0.0, 0.0, 0.0],
        K=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        binning_x=0,
        binning_y=0,
    )


class ReplayConnection(UnitreeWebRTCConnection):
    dir_name = "unitree_go2_bigoffice"

    # we don't want UnitreeWebRTCConnection to init
    def __init__(  # type: ignore[no-untyped-def]
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

    def standup(self) -> bool:
        return True

    def liedown(self) -> bool:
        return True

    @simple_mcache
    def lidar_stream(self):  # type: ignore[no-untyped-def]
        lidar_store = TimedSensorReplay(f"{self.dir_name}/lidar")  # type: ignore[var-annotated]
        return lidar_store.stream(**self.replay_config)  # type: ignore[arg-type]

    @simple_mcache
    def odom_stream(self):  # type: ignore[no-untyped-def]
        odom_store = TimedSensorReplay(f"{self.dir_name}/odom")  # type: ignore[var-annotated]
        return odom_store.stream(**self.replay_config)  # type: ignore[arg-type]

    # we don't have raw video stream in the data set
    @simple_mcache
    def video_stream(self):  # type: ignore[no-untyped-def]
        video_store = TimedSensorReplay(f"{self.dir_name}/video")  # type: ignore[var-annotated]
        return video_store.stream(**self.replay_config)  # type: ignore[arg-type]

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        return True

    def publish_request(self, topic: str, data: dict):  # type: ignore[no-untyped-def, type-arg]
        """Fake publish request for testing."""
        return {"status": "ok", "message": "Fake publish"}


class GO2Connection(Module, spec.Camera, spec.Pointcloud):
    cmd_vel: In[Twist]
    pointcloud: Out[PointCloud2]
    odom: Out[PoseStamped]
    lidar: Out[LidarMessage]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    connection: Go2ConnectionProtocol
    camera_info_static: CameraInfo = _camera_info_static()
    _global_config: GlobalConfig
    _camera_info_thread: Thread | None = None

    def __init__(  # type: ignore[no-untyped-def]
        self,
        ip: str | None = None,
        global_config: GlobalConfig | None = None,
        *args,
        **kwargs,
    ) -> None:
        self._global_config = global_config or GlobalConfig()

        ip = ip if ip is not None else self._global_config.robot_ip

        connection_type = self._global_config.unitree_connection_type

        if ip in ["fake", "mock", "replay"] or connection_type == "replay":
            self.connection = ReplayConnection()
        elif ip == "mujoco" or connection_type == "mujoco":
            from dimos.robot.unitree_webrtc.mujoco_connection import MujocoConnection

            self.connection = MujocoConnection(self._global_config)
        else:
            assert ip is not None, "IP address must be provided"
            self.connection = UnitreeWebRTCConnection(ip)

        Module.__init__(self, *args, **kwargs)

    @rpc
    def record(self, recording_name: str) -> None:
        lidar_store: TimedSensorStorage = TimedSensorStorage(f"{recording_name}/lidar")  # type: ignore[type-arg]
        lidar_store.save_stream(self.connection.lidar_stream()).subscribe(lambda x: x)  # type: ignore[arg-type, attr-defined]

        odom_store: TimedSensorStorage = TimedSensorStorage(f"{recording_name}/odom")  # type: ignore[type-arg]
        odom_store.save_stream(self.connection.odom_stream()).subscribe(lambda x: x)  # type: ignore[arg-type, attr-defined]

        video_store: TimedSensorStorage = TimedSensorStorage(f"{recording_name}/video")  # type: ignore[type-arg]
        video_store.save_stream(self.connection.video_stream()).subscribe(lambda x: x)  # type: ignore[arg-type, attr-defined]

    @rpc
    def start(self) -> None:
        super().start()

        self.connection.start()

        self._disposables.add(self.connection.lidar_stream().subscribe(self.lidar.publish))
        self._disposables.add(self.connection.odom_stream().subscribe(self._publish_tf))
        self._disposables.add(self.connection.video_stream().subscribe(self.color_image.publish))
        self._disposables.add(Disposable(self.cmd_vel.subscribe(self.move)))

        self._camera_info_thread = Thread(
            target=self.publish_camera_info,
            daemon=True,
        )
        self._camera_info_thread.start()

        self.standup()
        # self.record("go2_bigoffice")

    @rpc
    def stop(self) -> None:
        self.liedown()

        if self.connection:
            self.connection.stop()

        if self._camera_info_thread and self._camera_info_thread.is_alive():
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

        map_to_world = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="map",
            child_frame_id="world",
            ts=time.time(),
        )

        return [
            Transform.from_pose("base_link", odom),
            camera_link,
            camera_optical,
            sensor,
            map_to_world,
        ]

    def _publish_tf(self, msg: PoseStamped) -> None:
        self.tf.publish(*self._odom_to_tf(msg))
        if self.odom.transport:
            self.odom.publish(msg)

    def publish_camera_info(self) -> None:
        while True:
            self.camera_info.publish(_camera_info_static())
            time.sleep(1.0)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to robot."""
        return self.connection.move(twist, duration)

    @rpc
    def standup(self) -> bool:
        """Make the robot stand up."""
        return self.connection.standup()

    @rpc
    def liedown(self) -> bool:
        """Make the robot lie down."""
        return self.connection.liedown()

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        """Publish a request to the WebRTC connection.
        Args:
            topic: The RTC topic to publish to
            data: The data dictionary to publish
        Returns:
            The result of the publish request
        """
        return self.connection.publish_request(topic, data)


go2_connection = GO2Connection.blueprint


def deploy(dimos: DimosCluster, ip: str, prefix: str = "") -> GO2Connection:
    from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE

    connection = dimos.deploy(GO2Connection, ip)  # type: ignore[attr-defined]

    connection.pointcloud.transport = pSHMTransport(
        f"{prefix}/lidar", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    )
    connection.color_image.transport = pSHMTransport(
        f"{prefix}/image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    )

    connection.cmd_vel.transport = LCMTransport(f"{prefix}/cmd_vel", Twist)

    connection.camera_info.transport = LCMTransport(f"{prefix}/camera_info", CameraInfo)
    connection.start()

    return connection  # type: ignore[no-any-return]


__all__ = ["GO2Connection", "deploy", "go2_connection"]
