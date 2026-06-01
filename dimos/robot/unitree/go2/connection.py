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

from enum import Enum
import sys
import time
from typing import TYPE_CHECKING, Any, Protocol

from pydantic import Field
from reactivex.disposable import Disposable
from reactivex.observable import Observable
import rerun.blueprint as rrb

from dimos.agents.annotation import skill
from dimos.constants import (
    DEFAULT_CAPACITY_COLOR_IMAGE,
    DEFAULT_WORLD_FRAME,
)
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.core.resource import CompositeResource
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.spec.perception import Camera, Pointcloud
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.rpc_client import ModuleProxy
from dimos.memory2.replay import Replay, resolve_db_path
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.robot.unitree.go2.config import Go2Config, camera_info_static
from dimos.utils.decorators.decorators import cached_property, simple_mcache

if sys.version_info < (3, 13):
    from typing_extensions import TypeVar
else:
    from typing import TypeVar

logger = setup_logger()


class Go2Mode(str, Enum):
    DEFAULT = "default"
    RAGE = "rage"


class ConnectionConfig(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)
    mode: Go2Mode = Go2Mode.DEFAULT
    frame_mapping: dict[str, str] = Field(
        default_factory=lambda: dict(
            body=Go2Config.body_frame,
            parent=DEFAULT_WORLD_FRAME,
            camera_link="camera_link",
            camera_optical="camera_optical",
        )
    )
    static_transforms: dict[str, Transform] = Field(
        default_factory=lambda: dict(Go2Config.static_transforms)
    )


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
    def balance_stand(self) -> bool: ...
    def set_obstacle_avoidance(self, enabled: bool = True) -> None: ...
    def enable_rage_mode(self) -> bool: ...
    def publish_request(self, topic: str, data: dict) -> dict: ...  # type: ignore[type-arg]


def make_connection(ip: str | None, cfg: GlobalConfig) -> Go2ConnectionProtocol:
    connection_type = cfg.unitree_connection_type.lower()

    if ip in ("fake", "mock", "replay") or connection_type == "replay":
        dataset = cfg.replay_db
        return ReplayConnection(dataset=dataset)
    elif ip == "mujoco" or connection_type in ("mujoco", "true"):
        from dimos.robot.unitree.mujoco_connection import MujocoConnection

        return MujocoConnection(cfg)
    elif connection_type == "dimsim":
        from dimos.robot.unitree.dimsim_connection import DimSimConnection

        return DimSimConnection(cfg)
    elif connection_type == "webrtc":
        assert ip is not None, "IP address must be provided"
        return UnitreeWebRTCConnection(ip)
    else:
        raise ValueError(f"Unknown simulator {cfg.simulation!r}. Choose from: mujoco, dimsim")


class ReplayConnection(UnitreeWebRTCConnection, CompositeResource):
    def __init__(  # type: ignore[no-untyped-def]
        self,
        dataset: str = "go2_china_office",
        **kwargs,
    ) -> None:
        self.dataset = dataset
        self._loop = kwargs.get("loop", False)
        self._seek = kwargs.get("seek")
        self._duration = kwargs.get("duration")

    @cached_property
    def replay(self) -> Replay:
        # One shared store + Replay so lidar/odom/video advance against the
        # same wall-clock anchor on subscribe.
        store = self.register_disposable(
            SqliteStore(path=str(resolve_db_path(self.dataset)), must_exist=True)
        )
        store.start()
        return store.replay(loop=self._loop, seek=self._seek, duration=self._duration)

    def connect(self) -> None:
        pass

    def start(self) -> None:
        pass

    def standup(self) -> bool:
        return True

    def liedown(self) -> bool:
        return True

    def balance_stand(self) -> bool:
        return True

    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        pass

    def enable_rage_mode(self) -> bool:
        return True

    @simple_mcache
    def lidar_stream(self) -> Observable[PointCloud2]:
        return self.replay.streams.lidar.observable()

    @simple_mcache
    def odom_stream(self) -> Observable[PoseStamped]:
        return self.replay.streams.odom.observable()

    @simple_mcache
    def video_stream(self) -> Observable[Image]:
        return self.replay.streams.color_image.observable()

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        return True

    def publish_request(self, topic: str, data: dict):  # type: ignore[no-untyped-def, type-arg]
        """Fake publish request for testing."""
        return {"status": "ok", "message": "Fake publish"}


_Config = TypeVar("_Config", bound=ConnectionConfig, default=ConnectionConfig)


class GO2Connection(Module, Camera, Pointcloud):
    dedicated_worker = True

    config: ConnectionConfig
    cmd_vel: In[Twist]
    pointcloud: Out[PointCloud2]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    connection: Go2ConnectionProtocol
    camera_info_static: CameraInfo = camera_info_static()
    _latest_video_frame: Image | None = None

    @classmethod
    def rerun_views(cls):  # type: ignore[no-untyped-def]
        """Return Rerun view blueprints for GO2 camera visualization."""
        return [
            rrb.Spatial2DView(
                name="Camera",
                origin="world/robot/camera/rgb",
            ),
        ]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.connection = make_connection(self.config.ip, self.config.g)
        self._camera_info_static = camera_info_static()

        if hasattr(self.connection, "camera_info_static"):
            self._camera_info_static = self.connection.camera_info_static

    @rpc
    def start(self) -> None:
        super().start()
        if not hasattr(self, "connection"):
            return
        self.connection.start()

        def on_image(image: Image) -> None:
            image.frame_id = self.frame_mapping["camera_optical"]
            self.color_image.publish(image)
            self._latest_video_frame = image

        def on_lidar(pointcloud: PointCloud2) -> None:
            pointcloud.frame_id = self.frame_mapping["body"]
            self.lidar.publish(pointcloud)

        self.register_disposable(self.connection.lidar_stream().subscribe(on_lidar))
        self.register_disposable(self.connection.odom_stream().subscribe(self._publish_tf))
        self.register_disposable(self.connection.video_stream().subscribe(on_image))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

        self.standup()
        time.sleep(3)
        self.connection.balance_stand()

        if self.config.mode == Go2Mode.RAGE:
            self.connection.enable_rage_mode()

        self.connection.set_obstacle_avoidance(self.config.g.obstacle_avoidance)

    @rpc
    def stop(self) -> None:
        self.liedown()

        if self.connection:
            self.connection.stop()

        super().stop()

    def _on_static_publish(self) -> None:
        self._camera_info_static.frame_id = self.frame_mapping["camera_optical"]
        self.camera_info.publish(self._camera_info_static)

    def _publish_tf(self, msg: PoseStamped) -> None:
        self.tf.publish(
            Transform(
                translation=msg.position,
                rotation=msg.orientation,
                frame_id=self.frame_mapping["parent"],
                child_frame_id=self.frame_mapping["body"],
                ts=msg.ts,
            )
        )
        if self.odom.transport:
            msg.frame_id = self.frame_mapping["parent"]
            self.odom.publish(msg)

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
    def balance_stand(self) -> bool:
        """Enter BalanceStand: neutral state for switching locomotion modes"""
        return self.connection.balance_stand()

    @rpc
    def enable_rage_mode(self) -> bool:
        """Enable Rage Mode (~2.5 m/s forward velocity envelope).
        Ensures BalanceStand precondition regardless of current FSM state.
        """
        self.connection.balance_stand()
        time.sleep(0.3)
        result = self.connection.enable_rage_mode()
        logger.info("Rage Mode enabled")
        return result

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

    @skill
    def observe(self) -> Image | None:
        """Returns the latest video frame from the robot camera. Use this skill for any visual world queries.

        This skill provides the current camera view for perception tasks.
        Returns None if no frame has been captured yet.
        """
        return self._latest_video_frame


def deploy(dimos: ModuleCoordinator, ip: str, prefix: str = "") -> "ModuleProxy":
    connection = dimos.deploy(GO2Connection, ip=ip)

    connection.pointcloud.transport = pSHMTransport(
        f"{prefix}/lidar", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    )
    connection.color_image.transport = pSHMTransport(
        f"{prefix}/image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
    )

    connection.cmd_vel.transport = LCMTransport(f"{prefix}/cmd_vel", Twist)

    connection.camera_info.transport = LCMTransport(f"{prefix}/camera_info", CameraInfo)
    connection.start()

    return connection
