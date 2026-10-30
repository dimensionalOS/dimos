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
from typing import Any, ClassVar

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.unitree.config import connection_type
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.robot.unitree.g1.sim_device import G1SimDevice
from dimos.sim2.types import Frame, JointTargets, Layout
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1Config(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)
    connection_type: str = Field(
        default_factory=lambda m: connection_type(
            replay=m["g"].replay, simulation=m["g"].simulation
        )
    )
    # Per-device AES-128 key (G1 fw >=1.5.1).
    aes_128_key: str | None = Field(default=None, repr=False)
    sim_prefix: str = "g1"


class G1ConnectionBase(Module, ABC):
    """Abstract base for G1 connections (real hardware and simulation).

    Modules that depend on G1 connection RPC methods should reference this
    base class so the blueprint wiring works regardless of which concrete
    connection is deployed.
    """

    config: ModuleConfig

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
    config: G1Config
    cmd_vel: In[Twist]
    dedicated_worker: ClassVar[bool] = True
    world_layout: In[Layout]
    world_frame: In[Frame]
    targets: Out[JointTargets]
    odom: Out[Odometry]
    connection: UnitreeWebRTCConnection | G1SimDevice | None = None
    _sim_root: int = 0

    @rpc
    def start(self) -> None:
        super().start()

        match self.config.connection_type:
            case "webrtc":
                self.connection = UnitreeWebRTCConnection(
                    self.config.ip, aes_128_key=self.config.aes_128_key
                )
            case "sim2":
                self.connection = G1SimDevice(self.config.sim_prefix, self.targets.publish)
            case "replay":
                raise ValueError("Replay connection not implemented for G1 robot")
            case "mujoco":
                raise ValueError(
                    "This module does not support simulation, use G1SimConnection instead"
                )
            case _:
                raise ValueError(f"Unknown connection type: {self.config.connection_type}")

        assert self.connection is not None
        self.connection.start()

        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

    @rpc
    def stop(self) -> None:
        if self.connection is not None:
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

    async def handle_world_layout(self, layout: Layout) -> None:
        if isinstance(self.connection, G1SimDevice):
            self.connection.bind(layout)
            self._sim_root = layout.bodies.index(self.config.sim_prefix + "/pelvis")

    async def handle_world_frame(self, frame: Frame) -> None:
        if isinstance(self.connection, G1SimDevice) and self.connection.observe(frame):
            state = frame.state
            pose = state.poses[self._sim_root]
            inverse = Quaternion(*pose[3:]).conjugate()
            velocity = state.twists[self._sim_root]
            self.odom.publish(
                Odometry(
                    ts=state.wall_time,
                    frame_id="world",
                    child_frame_id=self.config.sim_prefix + "/pelvis",
                    pose=Pose(position=Vector3(*pose[:3]), orientation=Quaternion(*pose[3:])),
                    twist=Twist(
                        linear=inverse.rotate_vector(Vector3(*velocity[:3])),
                        angular=inverse.rotate_vector(Vector3(*velocity[3:])),
                    ),
                )
            )

    @rpc
    def control_status(self) -> dict[str, Any]:
        if isinstance(self.connection, G1SimDevice):
            return {
                "controller": "G1 ONNX walking policy",
                "inferences": self.connection.inferences,
                "max_inference_ms": self.connection.max_inference_ms,
            }
        return {"controller": "Unitree firmware"}
