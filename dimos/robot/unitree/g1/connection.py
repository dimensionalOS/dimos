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


from abc import ABC, abstractmethod
from pathlib import Path
from threading import Event, Thread
import time
from typing import TYPE_CHECKING, Any

from reactivex.disposable import Disposable

from dimos import spec
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs import Quaternion, Transform, Twist, Vector3
from dimos.msgs.sensor_msgs import CameraInfo, Image
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.rpc_client import ModuleProxy

logger = setup_logger()


def _camera_info_static() -> CameraInfo:
    params_path = Path(__file__).resolve().parent.parent / "params" / "front_camera_720.yaml"
    return CameraInfo.from_yaml(str(params_path))


class G1ConnectionBase(Module, ABC):
    """Abstract base for G1 connections (real hardware and simulation).

    Modules that depend on G1 connection RPC methods should reference this
    base class so the blueprint wiring works regardless of which concrete
    connection is deployed.
    """

    @rpc
    @abstractmethod
    def start(self) -> None:
        super().start()

    @rpc
    @abstractmethod
    def stop(self) -> None:
        super().stop()

    @rpc
    @abstractmethod
    def move(self, twist: Twist, duration: float = 0.0) -> None: ...

    @rpc
    @abstractmethod
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]: ...


class G1Connection(G1ConnectionBase):
    cmd_vel: In[Twist]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    ip: str | None
    connection_type: str | None = None
    _global_config: GlobalConfig

    connection: UnitreeWebRTCConnection | None
    camera_info_static: CameraInfo = _camera_info_static()
    _camera_info_thread: Thread | None
    _stop_event: Event

    def __init__(
        self,
        ip: str | None = None,
        connection_type: str | None = None,
        cfg: GlobalConfig = global_config,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        self._global_config = cfg
        self.ip = ip if ip is not None else self._global_config.robot_ip
        self.connection_type = connection_type or self._global_config.unitree_connection_type
        self.connection = None
        self._camera_info_thread = None
        self._stop_event = Event()
        super().__init__(*args, **kwargs)

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()

        match self.connection_type:
            case "webrtc":
                assert self.ip is not None, "IP address must be provided"
                self.connection = UnitreeWebRTCConnection(self.ip)
            case "replay":
                raise ValueError("Replay connection not implemented for G1 robot")
            case "mujoco":
                raise ValueError(
                    "This module does not support simulation, use G1SimConnection instead"
                )
            case _:
                raise ValueError(f"Unknown connection type: {self.connection_type}")

        assert self.connection is not None
        self.connection.start()

        self._disposables.add(self.connection.video_stream().subscribe(self.color_image.publish))
        self._disposables.add(Disposable(self.cmd_vel.subscribe(self.move)))

        self._camera_info_thread = Thread(target=self._publish_camera_info_loop, daemon=True)
        self._camera_info_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        assert self.connection is not None
        self.connection.stop()
        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=1.0)
        super().stop()

    def _publish_camera_info_loop(self) -> None:
        while not self._stop_event.is_set():
            ts = time.time()
            self.camera_info.publish(self.camera_info_static.with_ts(ts))
            self.tf.publish(
                Transform(
                    translation=Vector3(0.05, 0.0, 0.6),
                    rotation=Quaternion.from_euler(Vector3(0.0, 0.2, 0.0)),
                    frame_id="base_link",
                    child_frame_id="camera_link",
                    ts=ts,
                ),
                Transform(
                    translation=Vector3(0.0, 0.0, 0.0),
                    rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
                    frame_id="camera_link",
                    child_frame_id="camera_optical",
                    ts=ts,
                ),
            )
            self._stop_event.wait(1.0)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> None:
        assert self.connection is not None
        self.connection.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        logger.info(f"Publishing request to topic: {topic} with data: {data}")
        assert self.connection is not None
        return self.connection.publish_request(topic, data)  # type: ignore[no-any-return]


g1_connection = G1Connection.blueprint


def deploy(dimos: ModuleCoordinator, ip: str, local_planner: spec.LocalPlanner) -> "ModuleProxy":
    connection = dimos.deploy(G1Connection, ip)  # type: ignore[attr-defined]
    connection.cmd_vel.connect(local_planner.cmd_vel)
    connection.start()
    return connection


__all__ = ["G1Connection", "G1ConnectionBase", "deploy", "g1_connection"]
