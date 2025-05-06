import asyncio
import threading

from dataclasses import dataclass
from dimos.robot.unitree_standalone.type.lidar import LidarMessage
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod  # type: ignore[import-not-found]
from go2_webrtc_driver.constants import RTC_TOPIC

from dimos.robot.unitree_standalone.type.map import Map
from dimos.robot.global_planner.planner import AstarPlanner

from reactivex.subject import Subject
from reactivex.observable import Observable
from reactivex.disposable import Disposable, CompositeDisposable


@dataclass
class UnitreeGo2:
    ip: str
    conn: Go2WebRTCConnection
    mode: str = "ai"

    # mode = "ai" or "normal"
    def __init__(self, ip=None, mode="ai"):
        super().__init__()
        self.conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ip)
        self.connect()

        self.global_planner = AstarPlanner(
            set_local_nav=self.navigate_path_local,  # needs implementation
            get_costmap=self.ros_control.topic_latest("map", self.map.costmap),
            get_robot_pos=lambda: [0, 0, 0],  # self.ros_control.transform_euler_pos("base_link"),
        )

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

    def lidar_stream(self) -> Subject[LidarMessage]:
        subject: Subject[LidarMessage] = Subject()
        dispose = CompositeDisposable()

        def on_lidar_data(frame):
            if not subject.is_disposed:
                subject.on_next(LidarMessage.from_msg(frame))

        def cleanup():
            self.conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "off")

        dispose.add(Disposable(cleanup))

        self.conn.datachannel.pub_sub.subscribe("rt/utlidar/voxel_map_compressed", on_lidar_data)
        return subject

    def map_stream(self) -> Observable[Map]:
        self.map = Map()
        return self.map.consume(self.lidar_stream())

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
