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
resolves (``topics::sport_id`` in go2web and in ``go2/dds/rust``). The mount and camera
calibration are config, published natively by GO2DDS.
"""

from __future__ import annotations

import threading
from typing import Any, Literal

from dimos_generated.foxglove_msgs.msg import CompressedVideo
from dimos_generated.geometry_msgs.msg import Twist
from dimos_generated.nav_msgs.msg import Odometry
from dimos_generated.sensor_msgs.msg import CameraInfo, PointCloud2
from dimos_generated.std_msgs.msg import String
from pydantic import Field, field_validator
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.tf.static_tf_publisher import StaticTfPublisherConfig
from dimos.robot.unitree.go2.camera_calibration import front_camera_calibration
from dimos.robot.unitree.go2.go2_mid360_static_transforms import (
    CAMERA_XYZ,
    MID360_MOUNT_PRESETS,
    MID360_XYZ,
)


class Go2BaseConfig(StaticTfPublisherConfig):
    # base_link -> front_camera, and front_camera -> mid360_link, in metres.
    camera_xyz: tuple[float, float, float] = CAMERA_XYZ
    mid360_xyz: tuple[float, float, float] = MID360_XYZ
    # front_camera -> mid360_link, fixed-axis rpy in degrees. Either a raw (roll, pitch,
    # yaw) tuple or a name from MID360_MOUNT_PRESETS.
    mid360_mount: tuple[float, float, float] | str = MID360_MOUNT_PRESETS["SF"]
    # The front camera's calibration, published at camera_info_hz.
    camera_info: CameraInfo = Field(default_factory=front_camera_calibration)
    camera_info_hz: float = Field(default=1.0, gt=0.0)
    # The frame the live odometry moves; the mount edges above it are inverted so it
    # never gets two parents. GO2DDS publishes its own odom edge only for base_link.
    tf_root: Literal["base_link", "mid360_link"] = "base_link"

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


class Go2Base(Module):
    """The Go2's port profile and its action verbs."""

    config: Go2BaseConfig

    # Consumed on the robot side, never published here.
    cmd_vel: In[Twist]
    # One verb per message: "sit", "1016", "lidar on", "led red"; the table is the rust
    # `module::parse_verb`. Only the rpcs below publish onto it.
    command: In[String]
    odometry: Out[Odometry]
    lidar: Out[PointCloud2]
    video: Out[CompressedVideo]  # front camera, H.264 annex-B

    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]

    @rpc
    def start(self) -> None:
        super().start()
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
        self.command.transport.publish(String(data=verb))

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
