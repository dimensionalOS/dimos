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

import functools
import asyncio
import threading
from typing import TypeAlias, Literal, Optional
from dimos.utils.reactive import backpressure, callback_to_observable
from dimos.types.vector import Vector
from dimos.types.position import Position
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry as OdometryType
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod  # type: ignore[import-not-found]
from go2_webrtc_driver.constants import RTC_TOPIC, VUI_COLOR, SPORT_CMD
from reactivex.subject import Subject
from reactivex.observable import Observable
import numpy as np
from reactivex import operators as ops
from aiortc import MediaStreamTrack
from dimos.robot.unitree_webrtc.type.lowstate import LowStateMsg
from dimos.utils.reactive import getter_streaming
from dimos.robot.connection import BaseConnection
from dimos.robot.capabilities import (
    Move,
    Stop,
    Video,
    Lidar,
    Odometry,
    implements,
    WebRTCRequest,
)
import time
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree_webrtc.connection")

VideoMessage: TypeAlias = np.ndarray[tuple[int, int, Literal[3]], np.uint8]

# Public export for other modules
__all__ = ["UnitreeWebRTCConnection", "WebRTCRobot", "VideoMessage"]


@implements(Move, Stop, Video, Lidar, Odometry)
class UnitreeWebRTCConnection(BaseConnection):
    def __init__(self, ip: str, mode: str = "normal"):
        super().__init__(ip)
        self.mode = mode
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

    def disconnect(self):
        """Close connection and clean up resources."""
        if hasattr(self, "loop") and self.loop and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        if hasattr(self, "thread") and self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        logger.info("UnitreeWebRTCConnection disconnected")

    def move(self, velocity: Vector, duration: float = 0.0) -> bool:
        """Send movement command to the robot using velocity commands.

        Args:
            velocity: Velocity vector [x, y, yaw] where:
                     x: Forward/backward velocity (m/s)
                     y: Left/right velocity (m/s)
                     yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds). If 0, command is continuous

        Returns:
            bool: True if command was sent successfully
        """
        x, y, yaw = velocity.x, velocity.y, velocity.z

        # WebRTC coordinate mapping:
        # x - Positive right, negative left
        # y - positive forward, negative backwards
        # yaw - Positive rotate right, negative rotate left
        async def async_move():
            self.conn.datachannel.pub_sub.publish_without_callback(
                RTC_TOPIC["WIRELESS_CONTROLLER"],
                data={"lx": y, "ly": x, "rx": -yaw, "ry": 0},
            )

        async def async_move_duration():
            """Send movement commands continuously for the specified duration."""
            start_time = time.time()
            sleep_time = 0.01

            while time.time() - start_time < duration:
                await async_move()
                await asyncio.sleep(sleep_time)

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

    def get_pose(self) -> dict:
        """
        Get the current pose (position and rotation) of the robot in the map frame.

        Returns:
            Dictionary containing:
                - position: Vector (x, y, z)
                - rotation: Vector (roll, pitch, yaw) in radians
        """
        odom = getter_streaming(self.odom_stream())
        position = Vector(odom().pos.x, odom().pos.y, odom().pos.z)
        orientation = Vector(odom().rot.x, odom().rot.y, odom().rot.z)
        return {"position": position, "rotation": orientation}

    # Generic conversion of unitree subscription to Subject (used for all subs)
    def unitree_sub_stream(self, topic_name: str):
        return callback_to_observable(
            start=lambda cb: self.conn.datachannel.pub_sub.subscribe(topic_name, cb),
            stop=lambda: self.conn.datachannel.pub_sub.unsubscribe(topic_name),
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
    def raw_odom_stream(self) -> Subject[Position]:
        return backpressure(self.unitree_sub_stream(RTC_TOPIC["ROBOTODOM"]))

    @functools.cache
    def lidar_stream(self) -> Subject[LidarMessage]:
        return backpressure(
            self.raw_lidar_stream().pipe(
                ops.map(lambda raw_frame: LidarMessage.from_msg(raw_frame))
            )
        )

    @functools.cache
    def odom_stream(self) -> Subject[Position]:
        return backpressure(self.raw_odom_stream().pipe(ops.map(OdometryType.from_msg)))

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

    def standup(self):
        if self.mode == "ai":
            return self.standup_ai()
        else:
            return self.standup_normal()

    def liedown(self):
        return self.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandDown"]})

    async def handstand(self):
        return self.publish_request(
            RTC_TOPIC["SPORT_MOD"],
            {"api_id": SPORT_CMD["Standup"], "parameter": {"data": True}},
        )

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

    # TODO: implement queue_webrtc_req
    def webrtc_req(
        self,
        api_id: int,
        topic: str = None,
        parameter: str = "",
        priority: int = 0,
        request_id: str = None,
        data=None,
        timeout: float = 1000.0,
    ):
        """Send a WebRTC request command to the robot.

        Args:
            api_id: The API ID for the command.
            topic: The API topic to publish to. Defaults to ROSControl.webrtc_api_topic.
            parameter: Additional parameter data. Defaults to "".
            priority: Priority of the request. Defaults to 0.
            request_id: Unique identifier for the request. If None, one will be generated.
            data: Additional data to include with the request. Defaults to None.
            timeout: Timeout for the request in milliseconds. Defaults to 1000.0.

        Returns:
            The result of the WebRTC request.

        Raises:
            RuntimeError: If no connection interface with WebRTC capability is available.
        """
        if self.conn is None:
            raise RuntimeError("No connection interface available for WebRTC commands")

        # WebRTC requests are only available on ROS control interfaces
        if hasattr(self.conn, "queue_webrtc_req"):
            return self.conn.queue_webrtc_req(
                api_id=api_id,
                topic=topic,
                parameter=parameter,
                priority=priority,
                request_id=request_id,
                data=data,
                timeout=timeout,
            )
        else:
            raise RuntimeError("WebRTC requests not supported by this connection interface")

    @functools.lru_cache(maxsize=None)
    def video_stream(self) -> Observable[VideoMessage]:
        subject: Subject[VideoMessage] = Subject()
        stop_event = threading.Event()

        async def accept_track(track: MediaStreamTrack) -> VideoMessage:
            while True:
                if stop_event.is_set():
                    return
                frame = await track.recv()
                subject.on_next(frame.to_ndarray(format="bgr24"))

        self.conn.video.add_track_callback(accept_track)
        self.conn.video.switchVideoChannel(True)

        def stop(cb):
            stop_event.set()  # Signal the loop to stop
            self.conn.video.track_callbacks.remove(accept_track)
            self.conn.video.switchVideoChannel(False)

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
        return self.move(Vector(0.0, 0.0, 0.0))


# Backwards compatibility for tests
WebRTCRobot = UnitreeWebRTCConnection
