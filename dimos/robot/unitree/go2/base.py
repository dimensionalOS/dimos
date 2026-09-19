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

"""What every Go2 connection profile shares.

The ports are the wire contract (``dimos/<port>/<msg.NAME>`` on zenoh), so remap in the
blueprint rather than renaming. The verbs are the ``command`` vocabulary the robot side
resolves (``topics::sport_id`` in go2web and in ``go2/dds/rust``). The mount tree and the
camera intrinsics are the static data no robot-side process sends.
"""

from __future__ import annotations

import asyncio
import math
import threading
import time
from typing import Any

from pydantic import Field, field_validator
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.foxglove_msgs.CompressedVideo import CompressedVideo
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisher, StaticTfPublisherConfig
from dimos.robot.unitree.go2.connection import _camera_info_static
from dimos.robot.unitree.go2.go2_mid360_static_transforms import (
    CAMERA_XYZ,
    MID360_MOUNT_PRESETS,
    MID360_XYZ,
    OPTICAL_RPY,
)


class Go2BaseConfig(StaticTfPublisherConfig):
    # front_camera -> mid360_link, fixed-axis rpy in degrees. Either a raw (roll, pitch,
    # yaw) tuple or a name from MID360_MOUNT_PRESETS.
    mid360_mount: tuple[float, float, float] | str = MID360_MOUNT_PRESETS["SF"]
    camera_info_hz: float = Field(default=1.0, gt=0.0)

    @field_validator("mid360_mount", mode="before")
    @classmethod
    def _resolve_mid360_mount(cls, value: Any) -> Any:
        if isinstance(value, str):
            try:
                return MID360_MOUNT_PRESETS[value]
            except KeyError:
                raise ValueError(
                    f"unknown mid360_mount preset {value!r}; "
                    f"expected one of {sorted(MID360_MOUNT_PRESETS)}"
                ) from None
        return value


class Go2Base(StaticTfPublisher):
    """The Go2's port profile, its action verbs, and the static data it does not send."""

    config: Go2BaseConfig

    # Consumed on the robot side, never published here.
    cmd_vel: In[Twist]
    # One verb per message: "sit", "1016", "lidar on", "led red"; the table is the rust
    # `module::parse_verb`. Only the rpcs below publish onto it.
    command: In[String]
    odometry: Out[Odometry]
    lidar: Out[PointCloud2]
    video: Out[CompressedVideo]  # front camera, H.264 annex-B

    # Ours: nothing on the robot emits intrinsics.
    camera_info: Out[CameraInfo]

    _camera_info: CameraInfo = _camera_info_static()

    @rpc
    def start(self) -> None:
        super().start()
        self.spawn(self._publish_camera_info())
        # Deferred: a verb sent before the transport matches the robot side is dropped.
        timer = threading.Timer(5.0, self._startup_pose)
        timer.daemon = True
        timer.start()
        self.register_disposable(Disposable(timer.cancel))

    def _startup_pose(self) -> None:
        self.standup()

    @rpc
    def send_command(self, verb: str) -> None:
        """Fire an action verb at the robot side."""
        self.command.transport.publish(String(verb))

    @rpc
    def sport_command(self, api_id: int) -> None:
        """Same, by raw sport api id; the robot side parses a numeric verb."""
        self.send_command(str(api_id))

    @rpc
    def standup(self) -> None:
        self.send_command("stand-up")

    @rpc
    def liedown(self) -> None:
        self.send_command("stand-down")

    @rpc
    def balance_stand(self) -> None:
        self.send_command("balance")

    @rpc
    def sit(self) -> None:
        self.send_command("sit")

    @rpc
    def hello(self) -> None:
        self.send_command("hello")

    @rpc
    def jump(self) -> None:
        self.send_command("jump")

    @rpc
    def stop_movement(self) -> None:
        self.sport_command(1003)

    @rpc
    def set_lidar(self, enabled: bool) -> None:
        """The head L1 on or off."""
        self.send_command("lidar on" if enabled else "lidar off")

    @rpc
    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        """Toggle the onboard obstacle avoidance."""
        self.send_command(f"obstacle-avoidance {'on' if enabled else 'off'}")

    @rpc
    def set_rage_mode(self, enable: bool) -> None:
        self.send_command(f"rage {'on' if enable else 'off'}")

    @rpc
    def switch_joystick(self, enable: bool = True) -> None:
        """Firmware joystick listening on/off."""
        self.send_command(f"joystick {'on' if enable else 'off'}")

    @rpc
    def set_light(self, level: int) -> None:
        """Head LED panel brightness, 0..10."""
        self.send_command(f"brightness {level}")

    @rpc
    def set_led(self, color: str) -> None:
        """Head LED colour name, "off" to darken."""
        self.send_command(f"led {color}")

    @rpc
    def set_volume(self, level: int) -> None:
        """Speaker volume, 0..10."""
        self.send_command(f"volume {level}")

    def mount_edges(self) -> dict[str, Transform]:
        """The mount tree by child frame, measured outward from base_link."""
        base_to_camera = Transform(
            translation=Vector3(*CAMERA_XYZ),
            frame_id="base_link",
            child_frame_id="front_camera",
        )
        camera_to_mid360 = Transform(
            translation=Vector3(*MID360_XYZ),
            rotation=Quaternion.from_euler(
                Vector3(*(math.radians(float(d)) for d in self.config.mid360_mount))
            ),
            frame_id="front_camera",
            child_frame_id="mid360_link",
        )
        camera_to_optical = Transform(
            rotation=Quaternion.from_euler(Vector3(*OPTICAL_RPY)),
            frame_id="front_camera",
            child_frame_id="camera_optical",
        )
        return {t.child_frame_id: t for t in (base_to_camera, camera_to_mid360, camera_to_optical)}

    def transforms(self) -> list[Transform]:
        """Rooted at base_link, the frame the robot's own odometry moves."""
        return list(self.mount_edges().values())

    async def _publish_camera_info(self) -> None:
        period = 1.0 / self.config.camera_info_hz
        while self._running:
            self._camera_info.ts = time.time()
            self.camera_info.publish(self._camera_info)
            await asyncio.sleep(period)
