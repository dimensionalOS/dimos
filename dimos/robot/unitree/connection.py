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

import asyncio
from dataclasses import dataclass
import functools
import threading
import time
from typing import Any, TypeAlias, TypeVar

import numpy as np
from numpy.typing import NDArray
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.subject import Subject
from unitree_webrtc_connect.constants import (
    RTC_TOPIC,
    SPORT_CMD,
    VUI_COLOR,
)
from unitree_webrtc_connect.webrtc_driver import (
    UnitreeWebRTCConnection as LegionConnection,
    WebRTCConnectionMethod,
)

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.resource import Resource
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.type.lidar import (
    RawLidarMsg,
    pointcloud2_from_webrtc_lidar,
)
from dimos.robot.unitree.type.lowstate import LowStateMsg
from dimos.robot.unitree.type.odometry import Odometry
from dimos.types.timestamped import Timestamped
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.reactive import backpressure, callback_to_observable

VideoMessage: TypeAlias = NDArray[np.uint8]  # Shape: (height, width, 3)


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

    def to_ndarray(self, format=None):  # type: ignore[no-untyped-def]
        return self.data


class UnitreeWebRTCConnection(Resource):
    _SPORT_API_ID_RAGEMODE: int = 2059

    # How far to raise the head (body pitch, radians) when the robot stops.
    # The Go2 Euler command accepts roughly [-0.75, 0.75] rad per axis.
    _LOOK_UP_PITCH_RAD: float = 0.6

    def __init__(
        self,
        ip: str,
        mode: str = "ai",
        connection_method: WebRTCConnectionMethod = WebRTCConnectionMethod.LocalSTA,
    ) -> None:
        self.ip = ip
        self.mode = mode
        self.stop_timer: threading.Timer | None = None
        self.cmd_vel_timeout = 0.2
        # When True, raise the head once whenever the robot transitions from
        # moving to stopped, and level it again when it starts moving.
        self.look_up_on_stop = True
        self.look_up_pitch = self._LOOK_UP_PITCH_RAD
        self._was_moving = False
        # LocalSTA: robot joined your router (sends id="STA_localNetwork").
        # LocalAP: you're on the robot's own hotspot at 192.168.12.1 (sends id="").
        self.conn = LegionConnection(connection_method, ip=self.ip)
        self.connect()

    def connect(self) -> None:
        self.loop = asyncio.new_event_loop()
        self.task = None
        self.connected_event = asyncio.Event()
        self.connection_ready = threading.Event()

        async def async_connect() -> None:
            await self.conn.connect()
            await self.conn.datachannel.disableTrafficSaving(True)

            self.conn.datachannel.set_decoder(decoder_type="native")

            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1002, "parameter": {"name": self.mode}}
            )

            self.connected_event.set()
            self.connection_ready.set()

            while True:
                await asyncio.sleep(1)

        def start_background_loop() -> None:
            asyncio.set_event_loop(self.loop)
            self.task = self.loop.create_task(async_connect())
            self.loop.run_forever()

        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=start_background_loop, daemon=True)
        self.thread.start()
        self.connection_ready.wait()

    def start(self) -> None:
        pass

    def stop(self) -> None:
        # Cancel timer
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None

        if self.task:
            self.task.cancel()

        async def async_disconnect() -> None:
            try:
                # Send stop command directly since we're already in the event loop.
                self.conn.datachannel.pub_sub.publish_without_callback(
                    RTC_TOPIC["WIRELESS_CONTROLLER"],
                    data={"lx": 0, "ly": 0, "rx": 0, "ry": 0},
                )
                await self.conn.disconnect()
            except Exception:
                pass

        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(async_disconnect(), self.loop)

            self.loop.call_soon_threadsafe(self.loop.stop)

        if self.thread.is_alive():
            self.thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to the robot using Twist commands.

        Args:
            twist: Twist message with linear and angular velocities
            duration: How long to move (seconds). If 0, command is continuous

        Returns:
            bool: True if command was sent successfully
        """
        x, y, yaw = twist.linear.x, twist.linear.y, twist.angular.z
        moving = bool(x or y or yaw)

        # WebRTC coordinate mapping:
        # x - Positive right, negative left
        # y - positive forward, negative backwards
        # yaw - Positive rotate right, negative rotate left
        async def async_move() -> None:
            self.conn.datachannel.pub_sub.publish_without_callback(
                RTC_TOPIC["WIRELESS_CONTROLLER"],
                data={"lx": -y, "ly": x, "rx": -yaw, "ry": 0},
            )

        async def async_move_duration() -> None:
            """Send movement commands continuously for the specified duration."""
            start_time = time.time()
            sleep_time = 0.01

            while time.time() - start_time < duration:
                await async_move()
                await asyncio.sleep(sleep_time)

        # Cancel existing timer and start a new one
        if self.stop_timer:
            self.stop_timer.cancel()

        # Auto-stop after 0.5 seconds if no new commands
        self.stop_timer = threading.Timer(self.cmd_vel_timeout, self.stop_movement)
        self.stop_timer.daemon = True
        self.stop_timer.start()

        try:
            if duration > 0:
                # Send continuous move commands for the duration
                future = asyncio.run_coroutine_threadsafe(async_move_duration(), self.loop)
                future.result()
                # The robot stops at the end of a timed move; mark it moving so
                # stop_movement() raises the head on the resulting stop.
                if moving:
                    self._was_moving = True
                # Stop after duration
                self.stop_movement()
            else:
                # Single command for continuous movement
                future = asyncio.run_coroutine_threadsafe(async_move(), self.loop)
                future.result()
                self._update_head_posture(moving)
            return True
        except Exception as e:
            print(f"Failed to send movement command: {e}")
            return False

    # Generic conversion of unitree subscription to Subject (used for all subs)
    def unitree_sub_stream(self, topic_name: str):  # type: ignore[no-untyped-def]
        def subscribe_in_thread(cb) -> None:  # type: ignore[no-untyped-def]
            # Run the subscription in the background thread that has the event loop
            def run_subscription() -> None:
                self.conn.datachannel.pub_sub.subscribe(topic_name, cb)

            # Use call_soon_threadsafe to run in the background thread
            self.loop.call_soon_threadsafe(run_subscription)

        def unsubscribe_in_thread(cb) -> None:  # type: ignore[no-untyped-def]
            # Run the unsubscription in the background thread that has the event loop
            def run_unsubscription() -> None:
                self.conn.datachannel.pub_sub.unsubscribe(topic_name)

            # Use call_soon_threadsafe to run in the background thread
            self.loop.call_soon_threadsafe(run_unsubscription)

        return callback_to_observable(
            start=subscribe_in_thread,
            stop=unsubscribe_in_thread,
        )

    # Generic sync API call (we jump into the client thread)
    def publish_request(self, topic: str, data: dict[Any, Any]) -> Any:
        future = asyncio.run_coroutine_threadsafe(
            self.conn.datachannel.pub_sub.publish_request_new(topic, data), self.loop
        )
        return future.result()

    @simple_mcache
    def raw_lidar_stream(self) -> Observable[RawLidarMsg]:
        return backpressure(self.unitree_sub_stream(RTC_TOPIC["ULIDAR_ARRAY"]))

    @simple_mcache
    def raw_odom_stream(self) -> Observable[Pose]:
        return backpressure(self.unitree_sub_stream(RTC_TOPIC["ROBOTODOM"]))

    @simple_mcache
    def lidar_stream(self) -> Observable[PointCloud2]:
        return backpressure(
            self.raw_lidar_stream().pipe(
                ops.map(pointcloud2_from_webrtc_lidar),
                ops.map(time_is_now),
                # repair_stale_ts(),
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
                ops.map(
                    Odometry.from_msg,
                ),
                ops.map(time_is_now),
            )
        )

    @simple_mcache
    def video_stream(self) -> Observable[Image]:
        return backpressure(
            self.raw_video_stream().pipe(
                ops.filter(lambda frame: frame is not None),
                ops.map(
                    lambda frame: Image.from_numpy(
                        # np.ascontiguousarray(frame.to_ndarray("rgb24")),
                        frame.to_ndarray(format="rgb24"),  # type: ignore[attr-defined]
                        format=ImageFormat.RGB,  # Frame is RGB24, not BGR
                        frame_id="camera_optical",
                    ),
                ),
                ops.map(time_is_now),
            )
        )

    @simple_mcache
    def lowstate_stream(self) -> Observable[LowStateMsg]:
        return backpressure(self.unitree_sub_stream(RTC_TOPIC["LOW_STATE"]))

    def standup(self) -> bool:
        return bool(self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandUp"]}))

    def balance_stand(self) -> bool:
        """Activate BalanceStand mode — enables WIRELESS_CONTROLLER joystick commands."""
        return bool(
            self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["BalanceStand"]})
        )

    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        self.publish_request(
            RTC_TOPIC["OBSTACLES_AVOID"],
            {"api_id": 1001, "parameter": {"enable": int(enabled)}},
        )

    def free_walk(self) -> bool:
        """Activate FreeWalk locomotion AND enable joystick velocity input.

        Two-step protocol (mirrors enable_rage_mode, which is the only path
        in this codebase that demonstrably gets the dog walking from
        joystick input):

        1. ``FreeWalk`` (1045) — switches the locomotion gait.
        2. ``SwitchJoystick`` (1027, data=True) — explicitly enables joystick
           velocity interpretation. WITHOUT this, lx/ly are interpreted as
           body-pose lean (BalanceStand semantics) and the dog only sways
           in place even though the gait is technically FreeWalk.

        Returns True if both publishes succeed.
        """
        gait_ok = bool(
            self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["FreeWalk"]})
        )
        # Small settle so the gait switch lands before we flip the joystick
        # channel — matches the 2 s delay rage_mode uses for the same reason.
        time.sleep(0.3)
        joystick_ok = bool(
            self.publish_request(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": SPORT_CMD["SwitchJoystick"], "parameter": {"data": True}},
            )
        )
        return gait_ok and joystick_ok

    def enable_rage_mode(self) -> bool:
        """Enable Rage Mode on the Go2 via WebRTC.
        Assumes the robot is already in BalanceStand.
        """
        rage_ok = bool(
            self.publish_request(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": self._SPORT_API_ID_RAGEMODE, "parameter": {"data": True}},
            )
        )
        time.sleep(2.0)

        joystick_ok = bool(
            self.publish_request(
                RTC_TOPIC["SPORT_MOD"],
                {
                    "api_id": SPORT_CMD["SwitchJoystick"],
                    "parameter": {"data": True},
                },
            )
        )
        return rage_ok and joystick_ok

    def set_body_euler(self, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> bool:
        """Set the body orientation (radians) via the Euler sport command.

        Requires the robot to be standing (BalanceStand). Each axis is clamped
        to the Go2's safe range of roughly [-0.75, 0.75] rad.
        """

        def _clamp(v: float) -> float:
            return max(-0.75, min(0.75, v))

        return bool(
            self.publish_request(
                RTC_TOPIC["SPORT_MOD"],
                {
                    "api_id": SPORT_CMD["Euler"],
                    "parameter": {"x": _clamp(roll), "y": _clamp(pitch), "z": _clamp(yaw)},
                },
            )
        )

    def look_up(self, pitch: float | None = None) -> bool:
        """Raise the robot's head by pitching the body nose-up.

        `pitch` is the look-up angle in radians (positive = look up); defaults
        to ``look_up_pitch``. On the Go2 a positive body pitch raises the front;
        flip the sign here if your unit tilts the other way.
        """
        angle = self.look_up_pitch if pitch is None else pitch
        return self.set_body_euler(pitch=angle)

    def reset_posture(self) -> bool:
        """Level the body again (head neutral)."""
        return self.set_body_euler()

    def _update_head_posture(self, moving: bool) -> None:
        """Raise the head on the moving->stopped edge, level it on stopped->moving."""
        if not self.look_up_on_stop:
            return
        if moving and not self._was_moving:
            self.reset_posture()
        elif not moving and self._was_moving:
            self.look_up()
        self._was_moving = moving

    def liedown(self) -> bool:
        return bool(
            self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandDown"]})
        )

    async def handstand(self):  # type: ignore[no-untyped-def]
        return self.publish_request(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["Standup"], "parameter": {"data": True}},
        )

    def color(self, color: VUI_COLOR = VUI_COLOR.RED, colortime: int = 60) -> bool:
        return self.publish_request(  # type: ignore[no-any-return]
            RTC_TOPIC["VUI"],
            {
                "api_id": 1001,
                "parameter": {
                    "color": color,
                    "time": colortime,
                },
            },
        )

    @simple_mcache
    def raw_video_stream(self) -> Observable[VideoMessage]:
        subject: Subject[VideoMessage] = Subject()
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

        # Run the video channel switching in the background thread
        def switch_video_channel() -> None:
            self.conn.video.switchVideoChannel(True)

        self.loop.call_soon_threadsafe(switch_video_channel)

        def stop() -> None:
            stop_event.set()  # Signal the loop to stop
            self.conn.video.track_callbacks.remove(accept_track)

            # Run the video channel switching off in the background thread
            def switch_video_channel_off() -> None:
                self.conn.video.switchVideoChannel(False)

            self.loop.call_soon_threadsafe(switch_video_channel_off)

        return subject.pipe(ops.finally_action(stop))

    def get_video_stream(self, fps: int = 30) -> Observable[Image]:
        """Get the video stream from the robot's camera.

        Implements the AbstractRobot interface method.

        Args:
            fps: Frames per second. This parameter is included for API compatibility,
                 but doesn't affect the actual frame rate which is determined by the camera.

        Returns:
            Observable: An observable stream of video frames or None if video is not available.
        """
        return self.video_stream()

    def stop_movement(self) -> None:
        """Cancel the auto-stop timer (used by move() for continuous commands)."""
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None
        # Reaching the timeout means commands stopped arriving while moving:
        # treat that as a stop and raise the head once.
        if self.look_up_on_stop and self._was_moving:
            self._was_moving = False
            self.look_up()

    def disconnect(self) -> None:
        """Disconnect from the robot and clean up resources."""
        # Cancel timer
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None

        if hasattr(self, "task") and self.task:
            self.task.cancel()
        if hasattr(self, "conn"):

            async def async_disconnect() -> None:
                try:
                    await self.conn.disconnect()
                except:
                    pass

            if hasattr(self, "loop") and self.loop.is_running():
                asyncio.run_coroutine_threadsafe(async_disconnect(), self.loop)

        if hasattr(self, "loop") and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)

        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
