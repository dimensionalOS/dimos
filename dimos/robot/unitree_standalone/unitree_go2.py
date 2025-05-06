import asyncio
from dataclasses import dataclass
from dimos.robot.unitree_standalone.type.lidar import LidarMessage
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod  # type: ignore[import-not-found]
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD

from reactivex.subject import Subject
from reactivex.disposable import Disposable, CompositeDisposable

import logging

logging.basicConfig(level=logging.DEBUG)


@dataclass
class UnitreeGo2:
    ip: str
    conn: Go2WebRTCConnection

    def __init__(self, ip=None):
        super().__init__()
        self.ip = ip
        self.conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ip)
        self.connect()

    def connect(self):
        async def async_connect():
            await self.conn.connect()
            self.conn.datachannel.set_decoder(decoder_type="native")
            await self.conn.datachannel.disableTrafficSaving(True)

            await self.conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1002, "parameter": {"name": "ai"}}
            )

            # await self.conn.datachannel.pub_sub.publish_request_new(
            #     RTC_TOPIC["SPORT_MOD"],
            #     {"api_id": SPORT_CMD["Standup"], "parameter": {"data": True}},
            # )

        self.loop = asyncio.new_event_loop()
        self.loop.run_until_complete(async_connect())

    def lidar_stream(self) -> Subject[LidarMessage]:
        subject: Subject[LidarMessage] = Subject()
        dispose = CompositeDisposable()

        def on_lidar_data(frame):
            print("GOT FRAME", frame)
            if not subject.is_disposed:
                subject.on_next(LidarMessage.from_msg(frame))

        def cleanup():
            self.conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "off")

        dispose.add(Disposable(cleanup))

        self.conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")
        self.conn.datachannel.pub_sub.subscribe("rt/utlidar/voxel_map_compressed", lambda x: print(x))

        return subject
