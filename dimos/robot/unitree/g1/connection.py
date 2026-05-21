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

"""Real-hardware G1 connection over Unitree's WebRTC stack.

Composes `UnitreeWebRtcSession` for the asyncio loop / handshake / move /
publish_request.
"""

from __future__ import annotations

from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.connection_registry import connection
from dimos.robot.unitree.webrtc_session import UnitreeWebRtcSession
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1Config(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)


@connection(robot="g1", backend="webrtc")
class G1WebRtcConnection(Module):
    """Real-hardware G1 connection over Unitree's WebRTC stack."""

    config: G1Config
    cmd_vel: In[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.session: UnitreeWebRtcSession | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.session = UnitreeWebRtcSession(self.config.ip)
        self.session.start()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

    @rpc
    def stop(self) -> None:
        if self.session is not None:
            self.session.stop()
            self.session = None
        super().stop()

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> None:
        assert self.session is not None
        self.session.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        logger.info(f"Publishing request to topic: {topic} with data: {data}")
        assert self.session is not None
        return self.session.publish_request(topic, data)  # type: ignore[no-any-return]
