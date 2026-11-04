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

"""The real Go2 over its WebRTC link: a Rust process (rust/) holds the link and publishes the ports itself."""

from __future__ import annotations

from enum import Enum
import json
from threading import Event
from typing import Any, Protocol

from pydantic import Field, model_validator
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.unitree.go2.config import (
    CAMERA_OFFSET,
    CMD_VEL_TIMEOUT,
    JOINT_NAMES,
    L1_FRAME,
    WORLD_FRAME,
    front_camera_info,
    l1_mount,
)
from dimos.spec.perception import Camera, Pointcloud
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Go2Mode(str, Enum):
    DEFAULT = "default"
    RAGE = "rage"


class WebRtcTiming(BaseConfig):
    """Every wait, timeout and buffer size of the Rust WebRTC module (seconds unless named otherwise)."""

    answer_timeout: float = 5.0
    cmd_vel_timeout: float = CMD_VEL_TIMEOUT
    move_period: float = 0.01
    # the physical robot takes about 3 s to stand before it accepts BalanceStand
    stand_up_settle: float = 3.0
    rage_balance_settle: float = 0.3
    rage_switch_settle: float = 2.0
    motion_mode_settle: float = 5.0
    reconnect_pause: float = 2.0
    signaling_timeout: float = 4.0
    ice_gather_timeout: float = 4.0
    validation_timeout: float = 8.0
    heartbeat_period: float = 2.0
    ice_keepalive_period: float = 2.0
    # silence of link_quiet_timeout marks the link disconnected (it can recover); link_lost_timeout ends it
    link_quiet_timeout: float = 3.0
    link_lost_timeout: float = 5.0
    keyframe_retry_period: float = 1.0
    clock_window: float = 30.0
    clock_reset: float = 1.0
    video_queue_packets: int = 2048
    capture_max_waiting: int = 16
    # how long RTP packets wait for a late earlier one (a NACK's retransmit, an RTT behind) before it counts as lost
    reorder_wait: float = 0.15
    max_frame_bytes: int = 4 << 20
    camera_info_period: float = 1.0
    stats_period: float = 10.0

    def connect_deadline(self) -> float:
        """The longest a connect can take: ICE gathering, signaling (con_notify, con_ing, or the legacy offer)
        and the data channel's validation."""
        return self.ice_gather_timeout + 3 * self.signaling_timeout + self.validation_timeout


class Go2WebRtcConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/robot/unitree/go2/connections/webrtc/rust"
    executable: str = "result/bin/go2_webrtc"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    # None leaves the module idle (no robot to dial), so a blueprint can carry it while replaying.
    ip: str | None = Field(default_factory=lambda m: m["g"].robot_ip)
    mode: Go2Mode = Go2Mode.DEFAULT
    lidar: bool = True
    camera: bool = True
    velocity_api: bool = False
    # "mcf" for stair traversal, "normal" for basic, None to leave it as is
    motion_mode: str | None = None
    # Per-device AES-128 key (Go2 fw >=1.1.15).
    aes_128_key: str | None = Field(default=None, repr=False)
    obstacle_avoidance: bool = True
    # TF parent frame of the internal odometry (odom_frame_id -> base_link).
    odom_frame_id: str = WORLD_FRAME
    # Turn off where another module owns the base_link edge. The odom port keeps publishing either way.
    publish_tf: bool = True
    # Reconnect when the Go2 drops the WebRTC link (a Wi-Fi drop mustn't end a recording); off, the module exits.
    auto_reconnect: bool = True
    timing: WebRtcTiming = Field(default_factory=WebRtcTiming)

    @model_validator(mode="after")
    def _shutdown_outlasts_the_lie_down(self) -> Go2WebRtcConfig:
        # teardown waits for the verb running (one answer) and the lie-down (one more) before it lets go of the robot
        if "shutdown_timeout" not in self.model_fields_set:
            self.shutdown_timeout = 2 * self.timing.answer_timeout + DEFAULT_THREAD_JOIN_TIMEOUT
        return self

    def to_cli_args(self) -> list[str]:
        """Nothing: the Rust module reads its config from stdin, and the command line is logged (the AES key)."""
        return []

    def to_config_dict(self) -> dict[str, Any]:
        """Exactly the Rust module's Config: it rejects unknown fields and has no defaults."""
        camera_info = front_camera_info()
        camera_info.frame_id = _prefixed(self.frame_id_prefix, camera_info.frame_id)
        return {
            "ip": self.ip or "",
            "aes_128_key": self.aes_128_key or "",
            "mode": Go2Mode(self.mode).value,
            "lidar": self.lidar,
            "camera": self.camera,
            "velocity_api": self.velocity_api,
            "motion_mode": self.motion_mode or "",
            "obstacle_avoidance": self.obstacle_avoidance,
            "odom_frame_id": self.odom_frame_id,
            "publish_tf": self.publish_tf,
            "frame_id_prefix": self.frame_id_prefix or "",
            "auto_reconnect": self.auto_reconnect,
            "camera_info": camera_info.to_dict(),
            "camera_offset": list(CAMERA_OFFSET),
            "lidar_frame": L1_FRAME,
            "lidar_mount": list(l1_mount()),
            "joint_names": list(JOINT_NAMES),
            "timing": self.timing.model_dump(),
        }


class GO2ConnectionSpec(Spec, Protocol):
    """What a skill or bridge needs of a Go2 connection, real or simulated."""

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any] | None: ...


def _prefixed(prefix: str | None, name: str) -> str:
    """Apply a TF namespace prefix (ModuleConfig.frame_id_prefix) to a frame name."""
    if not prefix or not name:
        return name
    return f"{prefix}/{name}"


class Go2WebRtc(NativeModule, Camera, Pointcloud):
    """The Go2's streams and commands over WebRTC. The RPCs reach the Rust process as verbs on `command`
    (the Rust `parse_verb`): "stand-up", "rage on", "move 0.5 0 0 2"; fire-and-forget."""

    dedicated_worker = True

    config: Go2WebRtcConfig
    cmd_vel: In[Twist]
    command: In[String]
    pointcloud: Out[PointCloud2]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]
    battery: Out[BatteryState]
    joint_state: Out[JointState]

    camera_info_static: CameraInfo = front_camera_info()
    _latest_battery: BatteryState | None = None

    @rpc
    def build(self) -> None:
        if not self.config.ip:
            Module.build(self)
            return
        super().build()

    @rpc
    def start(self) -> None:
        if not self.config.ip:
            logger.info("Go2WebRtc idle: no robot IP (set ROBOT_IP or --go2webrtc.ip)")
            Module.start(self)
            return
        # the Rust process publishes straight to the transports, so listen there
        linked = Event()
        for unsubscribe in (
            self.battery.transport.subscribe(self._on_battery, self.battery),
            self.odom.transport.subscribe(lambda _: linked.set(), self.odom),
        ):
            self.register_disposable(Disposable(unsubscribe))
        super().start()
        # a bad key or a busy robot fails the run here
        deadline = self.config.timing.connect_deadline()
        if not linked.wait(deadline):
            raise RuntimeError(
                f"the Go2 at {self.config.ip} didn't connect within {deadline} s (see the go2_webrtc log)"
            )

    def _send(self, verb: str) -> bool:
        """A verb for the Rust module: it runs in order, and a failure is logged there."""
        # TODO: True only means "sent", not "ran"; return the outcome once Rust modules have RPCs.
        self.command.transport.publish(String(verb))
        return True

    def _on_battery(self, msg: BatteryState) -> None:
        self._latest_battery = msg

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to robot."""
        return self._send(f"move {twist.linear.x} {twist.linear.y} {twist.angular.z} {duration}")

    @rpc
    def standup(self) -> bool:
        """Make the robot stand up."""
        return self._send("stand-up")

    @rpc
    def liedown(self) -> bool:
        """Make the robot lie down."""
        return self._send("stand-down")

    @rpc
    def balance_stand(self) -> bool:
        """Enter BalanceStand: neutral state for switching locomotion modes"""
        return self._send("balance")

    @rpc
    def set_rage_mode(self, enable: bool) -> bool:
        """Toggle Rage Mode on/off (~2.5 m/s envelope when on)."""
        logger.info("Rage Mode", enabled=enable)
        return self._send(f"rage {'on' if enable else 'off'}")

    @rpc
    def sport_command(self, api_id: int) -> bool:
        """Send a parameterless SPORT_MOD command by api_id (Hello, Damp, ...)."""
        return self._send(str(api_id))

    @rpc
    def set_light(self, level: int) -> bool:
        """Head-LED brightness level 0-10 (0 = off)."""
        return self._send(f"brightness {level}")

    @rpc
    def set_obstacle_avoidance(self, enabled: bool = True) -> bool:
        """Toggle the onboard obstacle avoidance."""
        return self._send(f"obstacle-avoidance {'on' if enabled else 'off'}")

    @rpc
    def switch_joystick(self, enable: bool = True) -> bool:
        """Firmware joystick listening on/off (WASD stick emulation needs it on)."""
        return self._send(f"joystick {'on' if enable else 'off'}")

    @rpc
    def stop_movement(self) -> None:
        """Zero the base immediately (webrtc deadman stop)."""
        self._send("stop")

    @skill
    def get_battery_soc(self) -> int | None:
        """Returns the robot's battery state-of-charge as a percentage (0-100).

        Use this skill to answer battery / power / charge questions. Returns
        None if no low-level state has been received yet.
        """
        return self.battery_soc()

    @rpc
    def battery_soc(self) -> int | None:
        """Battery SOC 0-100 (or None until lowstate arrives)."""
        battery = self._latest_battery
        return None if battery is None else round(battery.percentage * 100)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> None:
        """Send a request on an RTC topic (fire-and-forget: the robot's answer is not returned)."""
        parameter = data.get("parameter")
        verb = f"request {topic} {data['api_id']}"
        if parameter is not None:
            verb += f" {json.dumps(parameter)}"
        self._send(verb)
