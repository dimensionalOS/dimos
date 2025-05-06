import asyncio
import threading
from av import VideoFrame
from typing import TypeVar, Callable, Any
from dataclasses import dataclass
from dimos.utils.reactive import backpressure, callback_to_observable
from dimos.types.vector import Vector
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod  # type: ignore[import-not-found]
from go2_webrtc_driver.constants import RTC_TOPIC


from reactivex.operators import ops
from reactivex.subject import Subject
from reactivex.disposable import Disposable, CompositeDisposable

MSG = TypeVar("MSG")


class RawOdometryMessage: ...


@dataclass
class OdometryMessage:
    pos: Vector
    rot: Vector

    @classmethod
    def from_msg(cls, raw_message: RawOdometryMessage):
        return cls(pos=Vector(0, 0, 0), rot=Vector(0, 0, 0))


@dataclass
class Go2WebRTConnection:
    ip: str
    mode: str = "ai"

    conn: Go2WebRTCConnection

    # mode = "ai" or "normal"
    def __init__(self):
        super().__init__()
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

        # Wait for connection to be established before returning
        self.connection_ready.wait()

    # generic conversion from unitree
    def unitree_sub_stream(self, topic_name: str, callback: Callable[[MSG], Any]):
        return callback_to_observable(
            start=lambda cb: self.conn.datachannel.pub_sub.subscribe(topic_name, cb),
            stop=lambda: self.conn.datachannel.pub_sub.unsubscribe(topic_name),
        )

    def lidar_stream(self) -> Subject[LidarMessage]:
        return backpressure(
            self.unitree_sub_stream(RTC_TOPIC["ULIDAR_ARRAY"]).pipe(
                ops.map(lambda raw_frame: LidarMessage.from_msg(raw_frame))
            )
        )

    def odom_stream(self) -> Subject[OdometryMessage]:
        return backpressure(
            self.unitree_sub_stream(RTC_TOPIC["LOW_STATE"]).pipe(
                ops.map(lambda raw_frame: OdometryMessage.from_msg(raw_frame))
            )
        )

    def video_stream(self) -> Subject[Any]:
        def start(cb):
            self.conn.video.add_track_callback(cb)
            self.conn.video.switchVideoChannel(True)

        def stop(cb):
            self.conn.video.track_callbacks.remove(cb)
            self.conn.video.switchVideoChannel(False)

        return backpressure(callback_to_observable(start, stop))

    def stop(self):
        if hasattr(self, "task") and self.task:
            self.task.cancel()
        if hasattr(self, "conn"):

            async def disconnect():
                try:
                    await self.conn.disconnect()
                except:
                    pass

            if hasattr(self, "loop") and self.loop.is_running():
                asyncio.run_coroutine_threadsafe(disconnect(), self.loop)

        if hasattr(self, "loop") and self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)

        if hasattr(self, "thread") and self.thread.is_alive():
            self.thread.join(timeout=2.0)
