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


from typing import Any

from reactivex.disposable import Disposable

from dimos import spec
from dimos.core import DimosCluster, In, Module, rpc
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import Twist
from dimos.robot.unitree.connection.connection import UnitreeWebRTCConnection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1Connection(Module):
    cmd_vel: In[Twist]
    ip: str | None
    connection_type: str | None = None
    _global_config: GlobalConfig

    connection: UnitreeWebRTCConnection | None

    def __init__(
        self,
        ip: str | None = None,
        connection_type: str | None = None,
        global_config: GlobalConfig | None = None,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        self._global_config = global_config or GlobalConfig()
        self.ip = ip if ip is not None else self._global_config.robot_ip
        self.connection_type = connection_type or self._global_config.unitree_connection_type
        self.connection = None
        super().__init__(*args, **kwargs)

    @rpc
    def start(self) -> None:
        super().start()

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

        self._disposables.add(Disposable(self.cmd_vel.subscribe(self.move)))

    @rpc
    def stop(self) -> None:
        assert self.connection is not None
        self.connection.stop()
        super().stop()

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


def deploy(dimos: DimosCluster, ip: str, local_planner: spec.LocalPlanner) -> G1Connection:
    connection = dimos.deploy(G1Connection, ip)  # type: ignore[attr-defined]
    connection.cmd_vel.connect(local_planner.cmd_vel)
    connection.start()
    return connection  # type: ignore[no-any-return]


__all__ = ["G1Connection", "deploy", "g1_connection"]
