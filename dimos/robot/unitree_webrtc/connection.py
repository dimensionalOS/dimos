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

import asyncio
import functools
import threading
import time
from dataclasses import dataclass
from typing import Literal, Optional, TypeAlias

import numpy as np
from aiortc import MediaStreamTrack
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD, VUI_COLOR
from go2_webrtc_driver.webrtc_driver import (  # type: ignore[import-not-found]
    Go2WebRTCConnection,
    WebRTCConnectionMethod,
)
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.subject import Subject

from dimos.core import In, Module, Out, rpc
from dimos.core.resource import Resource
from dimos.msgs.geometry_msgs import Pose, Transform, Twist, Vector3
from dimos.msgs.sensor_msgs import Image
from dimos.robot.connection_interface import ConnectionInterface
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.lowstate import LowStateMsg
from dimos.robot.unitree_webrtc.type.odometry import Odometry
from dimos.utils.reactive import backpressure, callback_to_observable

VideoMessage: TypeAlias = np.ndarray[tuple[int, int, Literal[3]], np.uint8]


@dataclass
class SerializableVideoFrame:
    """Pickleable wrapper for av.VideoFrame with all metadata"""

    data: np.ndarray
    pts: Optional[int] = None
    time: Optional[float] = None
    dts: Optional[int] = None
    width: Optional[int] = None
    height: Optional[int] = None
    format: Optional[str] = None

    @classmethod
    def from_av_frame(cls, frame):
        return cls(
            data=frame.to_ndarray(format="rgb24"),
            pts=frame.pts,
            time=frame.time,
            dts=frame.dts,
            width=frame.width,
            height=frame.height,
            format=frame.format.name if hasattr(frame, "format") and frame.format else None,
        )

    def to_ndarray(self, format=None):
        return self.data


class UnitreeWebRTCConnection(Resource):
    def __init__(self, ip: str, mode: str = "ai"):
        self.ip = ip
        self.mode = mode
        self.stop_timer = None
        self.cmd_vel_timeout = 0.2
        self.conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=self.ip)
        self.connect()

    def connect(self):
        self.loop = asyncio.new_event_loop()
        self.task = None
        self.connected_event = asyncio.Event()
        self.connection_ready = threading.Event()

        async def async_connect():
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

        def start_background_loop():
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
                await self.conn.disconnect()
            except Exception:
                pass

        if self.loop.is_running():
            asyncio.run_coroutine_threadsafe(async_disconnect(), self.loop)

            self.loop.call_soon_threadsafe(self.loop.stop)

        if self.thread.is_alive():
            self.thread.join(timeout=2.0)

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to the robot using Twist commands.

        Args:
            twist: Twist message with linear and angular velocities
            duration: How long to move (seconds). If 0, command is continuous

        Returns:
            bool: True if command was sent successfully
        """
        x, y, yaw = twist.linear.x, twist.linear.y, twist.angular.z

        # WebRTC coordinate mapping:
        # x - Positive right, negative left
        # y - positive forward, negative backwards
        # yaw - Positive rotate right, negative rotate left
        async def async_move():
            self.conn.datachannel.pub_sub.publish_without_callback(
                RTC_TOPIC["WIRELESS_CONTROLLER"],
                data={"lx": -y, "ly": x, "rx": -yaw, "ry": 0},
            )

        async def async_move_duration():
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
        self.stop_timer = threading.Timer(self.cmd_vel_timeout, self.stop)
        self.stop_timer.daemon = True
        self.stop_timer.start()

        try:
            if duration > 0:
                # Send continuous move commands for the duration
                future = asyncio.run_coroutine_threadsafe(async_move_duration(), self.loop)
                future.result()
                # Stop after duration
                self.stop()
            else:
                # Single command for continuous movement
                future = asyncio.run_coroutine_threadsafe(async_move(), self.loop)
                future.result()
            return True
        except Exception as e:
            print(f"Failed to send movement command: {e}")
            return False

    # Generic conversion of unitree subscription to Subject (used for all subs)
    def unitree_sub_stream(self, topic_name: str):
        def subscribe_in_thread(cb):
            # Run the subscription in the background thread that has the event loop
            def run_subscription():
                self.conn.datachannel.pub_sub.subscribe(topic_name, cb)

            # Use call_soon_threadsafe to run in the background thread
            self.loop.call_soon_threadsafe(run_subscription)

        def unsubscribe_in_thread(cb):
            # Run the unsubscription in the background thread that has the event loop
            def run_unsubscription():
                self.conn.datachannel.pub_sub.unsubscribe(topic_name)

            # Use call_soon_threadsafe to run in the background thread
            self.loop.call_soon_threadsafe(run_unsubscription)

        return callback_to_observable(
            start=subscribe_in_thread,
            stop=unsubscribe_in_thread,
        )

    # Generic sync API call (we jump into the client thread)
    def publish_request(self, topic: str, data: dict):
        future = asyncio.run_coroutine_threadsafe(
            self.conn.datachannel.pub_sub.publish_request_new(topic, data), self.loop
        )
        return future.result()

    @functools.cache
    def raw_lidar_stream(self) -> Subject[LidarMessage]:
        return backpressure(self.unitree_sub_stream(RTC_TOPIC["ULIDAR_ARRAY"]))

    @functools.cache
    def raw_odom_stream(self) -> Subject[Pose]:
        return backpressure(self.unitree_sub_stream(RTC_TOPIC["ROBOTODOM"]))

    @functools.cache
    def lidar_stream(self) -> Subject[LidarMessage]:
        return backpressure(
            self.raw_lidar_stream().pipe(
                ops.map(lambda raw_frame: LidarMessage.from_msg(raw_frame, ts=time.time()))
            )
        )

    @functools.cache
    def tf_stream(self) -> Subject[Transform]:
        base_link = functools.partial(Transform.from_pose, "base_link")
        return backpressure(self.odom_stream().pipe(ops.map(base_link)))

    @functools.cache
    def odom_stream(self) -> Subject[Pose]:
        return backpressure(self.raw_odom_stream().pipe(ops.map(Odometry.from_msg)))

    @functools.cache
    def video_stream(self) -> Observable[Image]:
        return backpressure(
            self.raw_video_stream().pipe(
                ops.filter(lambda frame: frame is not None),
                ops.map(
                    lambda frame: Image.from_numpy(
                        frame.to_ndarray(format="rgb24"),
                        frame_id="camera_optical",
                    )
                ),
            )
        )

    @functools.cache
    def lowstate_stream(self) -> Subject[LowStateMsg]:
        return backpressure(self.unitree_sub_stream(RTC_TOPIC["LOW_STATE"]))

    def standup_ai(self):
        return self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["BalanceStand"]})

    def standup_normal(self):
        self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandUp"]})
        time.sleep(0.5)
        self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["RecoveryStand"]})
        return True

    @rpc
    def standup(self):
        if self.mode == "ai":
            return self.standup_ai()
        else:
            return self.standup_normal()

    @rpc
    def liedown(self):
        return self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandDown"]})

    async def handstand(self):
        return self.publish_request(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["Standup"], "parameter": {"data": True}},
        )

    @rpc
    def color(self, color: VUI_COLOR = VUI_COLOR.RED, colortime: int = 60) -> bool:
        return self.publish_request(
            RTC_TOPIC["VUI"],
            {
                "api_id": 1001,
                "parameter": {
                    "color": color,
                    "time": colortime,
                },
            },
        )

    @functools.lru_cache(maxsize=None)
    def raw_video_stream(self) -> Observable[VideoMessage]:
        subject: Subject[VideoMessage] = Subject()
        stop_event = threading.Event()

        async def accept_track(track: MediaStreamTrack) -> VideoMessage:
            while True:
                if stop_event.is_set():
                    return
                frame = await track.recv()
                serializable_frame = SerializableVideoFrame.from_av_frame(frame)
                subject.on_next(serializable_frame)

        self.conn.video.add_track_callback(accept_track)

        # Run the video channel switching in the background thread
        def switch_video_channel():
            self.conn.video.switchVideoChannel(True)

        self.loop.call_soon_threadsafe(switch_video_channel)

        def stop():
            stop_event.set()  # Signal the loop to stop
            self.conn.video.track_callbacks.remove(accept_track)

            # Run the video channel switching off in the background thread
            def switch_video_channel_off():
                self.conn.video.switchVideoChannel(False)

            self.loop.call_soon_threadsafe(switch_video_channel_off)

        return subject.pipe(ops.finally_action(stop))

    def get_video_stream(self, fps: int = 30) -> Observable[VideoMessage]:
        """Get the video stream from the robot's camera.

        Implements the AbstractRobot interface method.

        Args:
            fps: Frames per second. This parameter is included for API compatibility,
                 but doesn't affect the actual frame rate which is determined by the camera.

        Returns:
            Observable: An observable stream of video frames or None if video is not available.
        """
        try:
            print("Starting WebRTC video stream...")
            stream = self.video_stream()
            if stream is None:
                print("Warning: Video stream is not available")
            return stream

        except Exception as e:
            print(f"Error getting video stream: {e}")
            return None

    def stop(self) -> bool:
        """Stop the robot's movement.

        Returns:
            bool: True if stop command was sent successfully
        """
        # Cancel timer since we're explicitly stopping
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None

        return self.move(Twist())

    def disconnect(self) -> None:
        """Disconnect from the robot and clean up resources."""
        # Cancel timer
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None

        if hasattr(self, "task") and self.task:
            self.task.cancel()
        if hasattr(self, "conn"):

            async def async_disconnect():
                try:
                    await self.conn.disconnect()
                except:
                    pass

            if hasattr(self, "loop") and self.loop.is_running():
                asyncio.run_coroutine_threadsafe(async_disconnect(), self.loop)

        if hasattr(self, "loop") and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)

        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=2.0)
