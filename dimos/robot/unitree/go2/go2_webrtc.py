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

"""Go2 WebRTC connection — Go2's move/mode commands and sensor decoding (G1: g1/g1_webrtc.py)."""

import asyncio
from dataclasses import dataclass
from enum import Enum
import functools
import json
import threading
import time
from typing import TypeVar

import numpy as np
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.subject import Subject
from unitree_webrtc_connect.constants import (
    DATA_CHANNEL_TYPE,
    RTC_TOPIC,
    SPORT_CMD,
    VUI_COLOR,
)

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.type.lidar import RawLidarMsg, pointcloud2_from_webrtc_lidar
from dimos.robot.unitree.type.lowstate import LowStateMsg
from dimos.robot.unitree.type.odometry import Odometry
from dimos.robot.unitree.unitree_webrtc import UnitreeWebRTCConnection
from dimos.types.timestamped import Timestamped
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

_T = TypeVar("_T", bound=Timestamped)


def time_is_now(x: _T) -> _T:
    x.ts = time.time()
    return x


@dataclass
class SerializableVideoFrame:
    """Pickleable wrapper for av.VideoFrame with all metadata"""

    data: np.ndarray
    pts: int | None = None
    time: float | None = None
    dts: int | None = None
    width: int | None = None
    height: int | None = None
    format: str | None = None

    @classmethod
    def from_av_frame(cls, frame):  # type: ignore[no-untyped-def]
        return cls(
            data=frame.to_ndarray(format="rgb24"),
            pts=frame.pts,
            time=frame.time,
            dts=frame.dts,
            width=frame.width,
            height=frame.height,
            format=frame.format.name if hasattr(frame, "format") and frame.format else None,
        )

    def to_ndarray(self, format: str | None = None) -> np.ndarray:
        return self.data


def _frame_to_image(frame: SerializableVideoFrame) -> Image:
    return Image.from_numpy(
        frame.to_ndarray(format="rgb24"),
        format=ImageFormat.RGB,  # frame is RGB24, not BGR
        frame_id="camera_optical",
    )


class TwistMode(str, Enum):
    """VELOCITY = SPORT Move (true m/s & rad/s); JOYSTICK = wireless axes (also forced in rage)."""

    VELOCITY = "velocity"
    JOYSTICK = "joystick"


class Go2WebRTCConnection(UnitreeWebRTCConnection):
    """Go2 WebRTC connection — move (SPORT velocity / joystick / rage), postures/modes, streams."""

    _SPORT_API_ID_RAGEMODE: int = 2059

    def __init__(
        self,
        ip: str,
        mode: str = "ai",
        aes_128_key: str | None = None,
        twist_mode: TwistMode = TwistMode.VELOCITY,
    ) -> None:
        self.twist_mode = TwistMode(twist_mode)
        self._move_seq = 0  # monotonic request id for SPORT Move commands
        self._rage_active = False
        self._stop_handle: asyncio.TimerHandle | None = None
        self.cmd_vel_timeout = 0.2
        super().__init__(ip, mode=mode, aes_128_key=aes_128_key)

    def _publish_velocity(self, vx: float, vy: float, vyaw: float) -> None:
        """SPORT Move (api 1008) velocity command — true m/s & rad/s, body frame."""
        self._move_seq += 1
        # parameter is a JSON STRING (firmware contract).
        self.publish(
            RTC_TOPIC["SPORT_MOD"],
            {
                "header": {"identity": {"id": self._move_seq, "api_id": SPORT_CMD["Move"]}},
                "parameter": json.dumps({"x": vx, "y": vy, "z": vyaw}),
            },
            msg_type=DATA_CHANNEL_TYPE["REQUEST"],
        )

    def _publish_joystick(self, x: float, y: float, yaw: float) -> None:
        """Wireless-controller joystick send (yaw is a joystick axis, not rad/s)."""
        self.publish(RTC_TOPIC["WIRELESS_CONTROLLER"], {"lx": -y, "ly": x, "rx": -yaw, "ry": 0})

    @property
    def _effective_twist_mode(self) -> TwistMode:
        """Rage Mode's FSM only accepts joystick axes, so it overrides twist_mode."""
        return TwistMode.JOYSTICK if self._rage_active else self.twist_mode

    def _send_twist(self, x: float, y: float, yaw: float) -> None:
        if self._effective_twist_mode is TwistMode.VELOCITY:
            self._publish_velocity(x, y, yaw)
        else:
            self._publish_joystick(x, y, yaw)

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send a twist. duration=0 sends once; >0 resends every 10ms until elapsed."""
        x, y, yaw = twist.linear.x, twist.linear.y, twist.angular.z
        self._arm_auto_stop()
        try:
            if duration > 0:
                start_time = time.time()
                while time.time() - start_time < duration:
                    self._send_twist(x, y, yaw)
                    time.sleep(0.01)
                self._send_twist(0.0, 0.0, 0.0)
            else:
                self._send_twist(x, y, yaw)
            return True
        except Exception as e:
            logger.warning("Failed to send movement command: %s", e)
            return False

    def _arm_auto_stop(self) -> None:
        """(Re)arm the command-timeout auto-stop on the event loop (no per-call threads)."""

        def _rearm() -> None:  # runs on the loop thread
            if self._stop_handle is not None:
                self._stop_handle.cancel()
            self._stop_handle = self.loop.call_later(self.cmd_vel_timeout, self._auto_stop)

        self.loop.call_soon_threadsafe(_rearm)

    def _auto_stop(self) -> None:
        """Zero the robot when commands stop arriving (runs on the loop thread)."""
        self._stop_handle = None
        try:
            self._send_twist(0.0, 0.0, 0.0)
        except Exception as e:
            logger.warning("Auto-stop send failed: %s", e)

    def stop(self) -> None:
        # Safety: zero the robot before the base tears down the loop. Loop teardown
        # also drops any pending auto-stop, so nothing can fire afterwards.
        try:
            self._send_twist(0.0, 0.0, 0.0)
        except Exception as e:
            logger.warning("Failed to send stop on disconnect: %s", e)
        super().stop()

    def standup(self) -> bool:
        return bool(self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandUp"]}))

    def liedown(self) -> bool:
        return bool(
            self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandDown"]})
        )

    def balance_stand(self) -> bool:
        """Activate BalanceStand mode — enables WIRELESS_CONTROLLER joystick commands."""
        return bool(
            self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["BalanceStand"]})
        )

    def free_walk(self) -> bool:
        """Activate FreeWalk locomotion mode — enables walking and velocity commands."""
        return bool(self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["FreeWalk"]}))

    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        self.publish_request(
            RTC_TOPIC["OBSTACLES_AVOID"],
            {"api_id": 1001, "parameter": {"enable": int(enabled)}},
        )

    def set_motion_mode(self, name: str) -> None:
        """Select the motion controller (mcf = stair-capable sport, normal = basic)."""
        # api_id 1001 = CheckMode, 1002 = SelectMode, param {"name": <mode>}.
        current = None
        try:
            resp = self.publish_request(RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1001})
            current = json.loads(resp["data"]["data"]).get("name")
        except (KeyError, TypeError, ValueError) as e:
            logger.warning("Motion mode check failed: %s", e)
        if current == name:
            return
        self.publish_request(
            RTC_TOPIC["MOTION_SWITCHER"],
            {"api_id": 1002, "parameter": {"name": name}},
        )
        time.sleep(5)

    def set_rage_mode(self, enable: bool) -> bool:
        """Toggle Rage Mode (api 2059). When on, its FSM ignores SPORT Move → move() uses joystick."""
        # Re-establish BalanceStand before toggling (notes: always BalanceStand
        # before flipping Rage).
        if not self.balance_stand():
            logger.warning("balance_stand() failed before rage toggle — proceeding")
        time.sleep(0.3)

        rage_ok = bool(
            self.publish_request(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": self._SPORT_API_ID_RAGEMODE, "parameter": {"data": enable}},
            )
        )
        if not rage_ok:
            return False

        # FsmRageMode is (de)active now; route move() to the joystick path accordingly.
        self._rage_active = enable

        if enable:
            time.sleep(2.0)  # let FsmRageMode transition settle
        joystick_ok = bool(
            self.publish_request(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": SPORT_CMD["SwitchJoystick"], "parameter": {"data": enable}},
            )
        )
        if not joystick_ok:
            logger.warning("SwitchJoystick failed after rage toggle", enabled=enable)
        return joystick_ok

    async def handstand(self):  # type: ignore[no-untyped-def]
        return self.publish_request(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["Standup"], "parameter": {"data": True}},
        )

    def color(self, color: VUI_COLOR = VUI_COLOR.RED, colortime: int = 60) -> bool:
        return self.publish_request(  # type: ignore[no-any-return]
            RTC_TOPIC["VUI"],
            {"api_id": 1001, "parameter": {"color": color, "time": colortime}},
        )

    @simple_mcache
    def raw_lidar_stream(self) -> Observable[RawLidarMsg]:
        return backpressure(self.subscribe(RTC_TOPIC["ULIDAR_ARRAY"]))

    @simple_mcache
    def raw_odom_stream(self) -> Observable[Pose]:
        return backpressure(self.subscribe(RTC_TOPIC["ROBOTODOM"]))

    @simple_mcache
    def lidar_stream(self) -> Observable[PointCloud2]:
        return backpressure(
            self.raw_lidar_stream().pipe(
                ops.map(pointcloud2_from_webrtc_lidar),
                ops.map(time_is_now),
            )
        )

    @simple_mcache
    def tf_stream(self) -> Observable[Transform]:
        base_link = functools.partial(Transform.from_pose, "base_link")
        return backpressure(self.odom_stream().pipe(ops.map(base_link)))

    @simple_mcache
    def odom_stream(self) -> Observable[Pose]:
        return backpressure(
            self.raw_odom_stream().pipe(
                ops.map(Odometry.from_msg),
                ops.map(time_is_now),
            )
        )

    @simple_mcache
    def video_stream(self) -> Observable[Image]:
        return backpressure(
            self.raw_video_stream().pipe(
                ops.filter(lambda frame: frame is not None),
                ops.map(_frame_to_image),
                ops.map(time_is_now),
            )
        )

    @simple_mcache
    def lowstate_stream(self) -> Observable[LowStateMsg]:
        return backpressure(self.subscribe(RTC_TOPIC["LOW_STATE"]))

    @simple_mcache
    def raw_video_stream(self) -> Observable[SerializableVideoFrame]:
        subject: Subject[SerializableVideoFrame] = Subject()
        stop_event = threading.Event()

        from aiortc import MediaStreamTrack

        async def accept_track(track: MediaStreamTrack) -> None:
            while True:
                if stop_event.is_set():
                    return
                frame = await track.recv()
                serializable_frame = SerializableVideoFrame.from_av_frame(frame)  # type: ignore[no-untyped-call]
                subject.on_next(serializable_frame)

        self.conn.video.add_track_callback(accept_track)

        def switch_video_channel() -> None:
            self.conn.video.switchVideoChannel(True)

        self.loop.call_soon_threadsafe(switch_video_channel)

        def stop() -> None:
            stop_event.set()
            self.conn.video.track_callbacks.remove(accept_track)

            def switch_video_channel_off() -> None:
                self.conn.video.switchVideoChannel(False)

            self.loop.call_soon_threadsafe(switch_video_channel_off)

        return subject.pipe(ops.finally_action(stop))

    def get_video_stream(self, fps: int = 30) -> Observable[Image]:
        """Video stream from the robot camera (fps kept for API compatibility, ignored)."""
        return self.video_stream()
