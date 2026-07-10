# Copyright 2026 Dimensional Inc.
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

from __future__ import annotations

import threading
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.robot.limx.tron1.high_level_client import TRON1HighLevelClient
from dimos.robot.limx.tron1 import protocol
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TRON1Config(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)
    ws_url: str | None = Field(default_factory=lambda m: m["g"].tron1_ws_url)
    accid: str | None = Field(default_factory=lambda m: m["g"].tron1_accid)
    max_vx: float = Field(default_factory=lambda m: m["g"].tron1_max_vx)
    max_vy: float = Field(default_factory=lambda m: m["g"].tron1_max_vy)
    max_yaw: float = Field(default_factory=lambda m: m["g"].tron1_max_yaw)
    enable_odom: bool = Field(default_factory=lambda m: m["g"].tron1_enable_odom)
    enable_imu: bool = Field(default_factory=lambda m: m["g"].tron1_enable_imu)
    auto_stand: bool = Field(default_factory=lambda m: m["g"].tron1_auto_stand)
    auto_walk_mode: bool = Field(default_factory=lambda m: m["g"].tron1_auto_walk_mode)
    odom_frame_id: str = "odom"
    base_frame_id: str = "base_link"


class TRON1Connection(Module):
    config: TRON1Config

    cmd_vel: In[Twist]
    state_estimation: Out[Odometry]
    odom: Out[PoseStamped]
    imu: Out[Imu]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._client: TRON1HighLevelClient | None = None
        self._walk_mode_enabled = False
        self._stop_timer: threading.Timer | None = None

    @rpc
    def start(self) -> None:
        super().start()

        ws_url = self.config.ws_url or (f"{self.config.ip}:5000" if self.config.ip else None)
        if not ws_url:
            raise ValueError("TRON1 ws_url is required. Set DIMOS_TRON1_WS_URL or pass --tron1-ws-url.")

        self._client = TRON1HighLevelClient(ws_url=ws_url, accid=self.config.accid)
        self._client.register_notify_handler("notify_odom", self._on_notify_odom)
        self._client.register_notify_handler("notify_imu", self._on_notify_imu)
        self._client.start()

        if self.config.enable_odom:
            self._client.send("request_enable_odom", protocol.build_request_enable_odom(True))
        if self.config.enable_imu:
            self._client.send("request_enable_imu", protocol.build_request_enable_imu(True))
        if self.config.auto_stand:
            self._client.send("request_stand_mode", protocol.build_request_stand_mode())

        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

    @rpc
    def stop(self) -> None:
        if self._stop_timer:
            try:
                self._stop_timer.cancel()
            except Exception:
                pass
            self._stop_timer = None
        if self._client:
            self._client.stop()
        self._client = None
        super().stop()

    @rpc
    def publish_request(self, name: str, data: dict[str, Any]) -> dict[str, Any]:
        if not self._client:
            raise RuntimeError("TRON1Connection is not started")
        try:
            return self._client.call(name, data)
        except TimeoutError:
            logger.warning("TRON1 request timed out", name=name)
            return {"timeout": True}

    @rpc
    def stand(self) -> dict[str, Any]:
        return self.publish_request("request_stand_mode", protocol.build_request_stand_mode())

    @rpc
    def sitdown(self) -> dict[str, Any]:
        return self.publish_request("request_sitdown", protocol.build_request_sitdown())

    @rpc
    def recover(self) -> dict[str, Any]:
        return self.publish_request("request_recover", protocol.build_request_recover())

    @rpc
    def emergency_stop(self) -> dict[str, Any]:
        return self.publish_request("request_emgy_stop", protocol.build_request_emgy_stop())

    @rpc
    def set_base_height(self, height: float) -> dict[str, Any]:
        return self.publish_request("request_base_height", protocol.build_request_base_height(height))

    @rpc
    def set_light_effect(self, effect: str) -> dict[str, Any]:
        return self.publish_request("request_light_effect", protocol.build_request_light_effect(effect))

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> None:
        if not self._client:
            raise RuntimeError("TRON1Connection is not started")

        if self._stop_timer:
            try:
                self._stop_timer.cancel()
            except Exception:
                pass
            self._stop_timer = None

        if self.config.auto_walk_mode and not self._walk_mode_enabled:
            self._client.send("request_walk_mode", protocol.build_request_walk_mode())
            self._walk_mode_enabled = True

        payload = protocol.build_request_twist(
            twist,
            max_vx=self.config.max_vx,
            max_vy=self.config.max_vy,
            max_yaw=self.config.max_yaw,
        )
        self._client.send("request_twist", payload)

        if duration > 0.0:
            self._stop_timer = threading.Timer(duration, self._send_zero_twist)
            self._stop_timer.daemon = True
            self._stop_timer.start()

    def _send_zero_twist(self) -> None:
        if not self._client:
            return
        zero = Twist()
        payload = protocol.build_request_twist(
            zero,
            max_vx=self.config.max_vx,
            max_vy=self.config.max_vy,
            max_yaw=self.config.max_yaw,
        )
        try:
            self._client.send("request_twist", payload)
        except Exception:
            logger.exception("Failed to send zero twist")

    def _on_notify_odom(self, payload: dict[str, Any]) -> None:
        odom_msg, pose_msg = protocol.parse_notify_odom(
            payload,
            frame_id=self.config.odom_frame_id,
            child_frame_id=self.config.base_frame_id,
        )
        self.state_estimation.publish(odom_msg)
        self.odom.publish(pose_msg)

    def _on_notify_imu(self, payload: dict[str, Any]) -> None:
        imu_msg = protocol.parse_notify_imu(payload)
        self.imu.publish(imu_msg)
