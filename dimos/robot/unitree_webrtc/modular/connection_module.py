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
from dataclasses import dataclass
import functools
import logging
import os
import queue
import warnings

from dimos_lcm.sensor_msgs import CameraInfo  # type: ignore[import-untyped]
import reactivex as rx
from reactivex import operators as ops
from reactivex.observable import Observable

from dimos.agents import Output, Reducer, Stream, skill  # type: ignore[attr-defined]
from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core import DimosCluster, In, LCMTransport, Module, ModuleConfig, Out, pSHMTransport, rpc
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Transform, Twist, Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.std_msgs import Header
from dimos.robot.unitree.connection.connection import UnitreeWebRTCConnection
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.utils.data import get_data
from dimos.utils.decorators import simple_mcache
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing import TimedSensorReplay, TimedSensorStorage

logger = setup_logger(level=logging.INFO)

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

image_resize_factor = 1
originalwidth, originalheight = (1280, 720)


class FakeRTC(UnitreeWebRTCConnection):
    dir_name = "unitree_go2_office_walk2"

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

    def standup(self) -> None:
        print("standup suppressed")

    def liedown(self) -> None:
        print("liedown suppressed")

    @simple_mcache
    def lidar_stream(self):  # type: ignore[no-untyped-def]
        print("lidar stream start")
        lidar_store = TimedSensorReplay(f"{self.dir_name}/lidar")  # type: ignore[var-annotated]
        return lidar_store.stream(**self.replay_config)  # type: ignore[arg-type]

    @simple_mcache
    def odom_stream(self):  # type: ignore[no-untyped-def]
        print("odom stream start")
        odom_store = TimedSensorReplay(f"{self.dir_name}/odom")  # type: ignore[var-annotated]
        return odom_store.stream(**self.replay_config)  # type: ignore[arg-type]

    # we don't have raw video stream in the data set
    @simple_mcache
    def video_stream(self):  # type: ignore[no-untyped-def]
        print("video stream start")
        video_store = TimedSensorReplay(f"{self.dir_name}/video")  # type: ignore[var-annotated]

        return video_store.stream(**self.replay_config)  # type: ignore[arg-type]

    def move(self, vector: Twist, duration: float = 0.0) -> None:  # type: ignore[override]
        pass

    def publish_request(self, topic: str, data: dict):  # type: ignore[no-untyped-def, type-arg]
        """Fake publish request for testing."""
        return {"status": "ok", "message": "Fake publish"}


@dataclass
class ConnectionModuleConfig(ModuleConfig):
    ip: str | None = None
    connection_type: str = "fake"  # or "fake" or "mujoco"
    loop: bool = False  # For fake connection
    speed: float = 1.0  # For fake connection


class ConnectionModule(Module):
    camera_info: Out[CameraInfo]
    odom: Out[PoseStamped]
    lidar: Out[LidarMessage]
    video: Out[Image]
    movecmd: In[Twist]

    connection = None

    default_config = ConnectionModuleConfig

    # mega temporary, skill should have a limit decorator for number of
    # parallel calls
    video_running: bool = False

    def __init__(self, connection_type: str = "webrtc", *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        self.connection_config = kwargs
        self.connection_type = connection_type
        Module.__init__(self, *args, **kwargs)

    @skill(stream=Stream.passive, output=Output.image, reducer=Reducer.latest)  # type: ignore[arg-type]
    def video_stream_tool(self) -> Image:  # type: ignore[misc]
        """implicit video stream skill, don't call this directly"""
        if self.video_running:
            return "video stream already running"
        self.video_running = True
        _queue = queue.Queue(maxsize=1)  # type: ignore[var-annotated]
        self.connection.video_stream().subscribe(_queue.put)  # type: ignore[attr-defined]

        yield from iter(_queue.get, None)

    @rpc
    def record(self, recording_name: str) -> None:
        lidar_store: TimedSensorStorage = TimedSensorStorage(f"{recording_name}/lidar")  # type: ignore[type-arg]
        lidar_store.save_stream(self.connection.lidar_stream()).subscribe(lambda x: x)  # type: ignore[arg-type, attr-defined]

        odom_store: TimedSensorStorage = TimedSensorStorage(f"{recording_name}/odom")  # type: ignore[type-arg]
        odom_store.save_stream(self.connection.odom_stream()).subscribe(lambda x: x)  # type: ignore[arg-type, attr-defined]

        video_store: TimedSensorStorage = TimedSensorStorage(f"{recording_name}/video")  # type: ignore[type-arg]
        video_store.save_stream(self.connection.video_stream()).subscribe(lambda x: x)  # type: ignore[arg-type, attr-defined]

    @rpc
    def start(self):  # type: ignore[no-untyped-def]
        """Start the connection and subscribe to sensor streams."""

        super().start()

        match self.connection_type:
            case "webrtc":
                self.connection = UnitreeWebRTCConnection(**self.connection_config)
            case "fake":
                self.connection = FakeRTC(**self.connection_config, seek=12.0)
            case "mujoco":
                from dimos.robot.unitree_webrtc.mujoco_connection import MujocoConnection

                self.connection = MujocoConnection(GlobalConfig())  # type: ignore[assignment]
                self.connection.start()  # type: ignore[union-attr]
            case _:
                raise ValueError(f"Unknown connection type: {self.connection_type}")

        unsub = self.connection.odom_stream().subscribe(  # type: ignore[union-attr]
            lambda odom: self._publish_tf(odom) and self.odom.publish(odom)  # type: ignore[func-returns-value]
        )
        self._disposables.add(unsub)

        # Connect sensor streams to outputs
        unsub = self.connection.lidar_stream().subscribe(self.lidar.publish)  # type: ignore[union-attr]
        self._disposables.add(unsub)

        # self.connection.lidar_stream().subscribe(lambda lidar: print("LIDAR", lidar.ts))
        # self.connection.video_stream().subscribe(lambda video: print("IMAGE", video.ts))
        # self.connection.odom_stream().subscribe(lambda odom: print("ODOM", odom.ts))

        def resize(image: Image) -> Image:
            return image.resize(
                int(originalwidth / image_resize_factor), int(originalheight / image_resize_factor)
            )

        unsub = self.connection.video_stream().subscribe(self.video.publish)  # type: ignore[union-attr]
        self._disposables.add(unsub)
        unsub = self.camera_info_stream().subscribe(self.camera_info.publish)
        self._disposables.add(unsub)
        unsub = self.movecmd.subscribe(self.connection.move)  # type: ignore[union-attr]
        self._disposables.add(unsub)  # type: ignore[arg-type]

    @rpc
    def stop(self) -> None:
        if self.connection:
            self.connection.stop()

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

    def _publish_tf(self, msg) -> None:  # type: ignore[no-untyped-def]
        self.odom.publish(msg)
        self.tf.publish(*self._odom_to_tf(msg))

    @rpc
    def publish_request(self, topic: str, data: dict):  # type: ignore[no-untyped-def, type-arg]
        """Publish a request to the WebRTC connection.
        Args:
            topic: The RTC topic to publish to
            data: The data dictionary to publish
        Returns:
            The result of the publish request
        """
        return self.connection.publish_request(topic, data)  # type: ignore[union-attr]

    @classmethod
    def _camera_info(cls) -> Out[CameraInfo]:
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

        return CameraInfo(**base_msg, header=Header("camera_optical"))  # type: ignore[no-any-return]

    @functools.cache
    def camera_info_stream(self) -> Observable[CameraInfo]:
        return rx.interval(1).pipe(ops.map(lambda _: self._camera_info()))


def deploy_connection(dimos: DimosCluster, **kwargs):  # type: ignore[no-untyped-def]
    foxglove_bridge = dimos.deploy(FoxgloveBridge)  # type: ignore[attr-defined, name-defined]
    foxglove_bridge.start()

    connection = dimos.deploy(  # type: ignore[attr-defined]
        ConnectionModule,
        ip=os.getenv("ROBOT_IP"),
        connection_type=os.getenv("CONNECTION_TYPE", "fake"),
        **kwargs,
    )

    connection.odom.transport = LCMTransport("/odom", PoseStamped)

    connection.video.transport = pSHMTransport(
        "/image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    )

    connection.lidar.transport = pSHMTransport(
        "/lidar", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    )

    connection.video.transport = LCMTransport("/image", Image)
    connection.lidar.transport = LCMTransport("/lidar", LidarMessage)
    connection.movecmd.transport = LCMTransport("/cmd_vel", Twist)
    connection.camera_info.transport = LCMTransport("/camera_info", CameraInfo)

    return connection
