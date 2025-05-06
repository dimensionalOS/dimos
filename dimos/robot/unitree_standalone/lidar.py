import logging
import threading
import asyncio
from reactivex.subject import Subject
from reactivex.disposable import Disposable, CompositeDisposable
from dimos.robot.unitree_standalone.type.lidar import LidarMessage
from dimos.robot.unitree.external.go2_webrtc_connect.go2_webrtc_driver.webrtc_driver import (
    Go2WebRTCConnection,
    WebRTCConnectionMethod,
)
from typing import cast

logging.basicConfig(level=logging.INFO)


def lidar(ip: str = "192.168.9.140", autocast: bool = True) -> Subject[LidarMessage]:
    # Create a Subject that will bridge between callback and Observable
    subject: Subject[LidarMessage] = Subject()

    # Create a CompositeDisposable to handle cleanup
    dispose = CompositeDisposable()

    async def async_setup():
        print("CONNECTING TO", ip)

        # Callback function that pushes to the subject
        def on_lidar_data(frame):
            if not subject.is_disposed:
                if autocast:
                    subject.on_next(LidarMessage.from_msg(frame))
                else:
                    subject.on_next(frame)

        conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip=ip)

        await conn.connect()
        await conn.datachannel.disableTrafficSaving(True)

        #        await conn.datachannel.pub_sub.publish_request_new(
        #            RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1002, "parameter": {"name": "ai"}}
        #        )

        # Switch to Handstand Mode
        # await conn.datachannel.pub_sub.publish_request_new(
        #    RTC_TOPIC["SPORT_MOD"],
        #    {"api_id": SPORT_CMD["Standup"], "parameter": {"data": True}},
        # )

        # // robot.connection.send({
        # //     "type": "msg",
        # //     "topic": Topic.WIRELESS_CONTROLLER,
        # //     "data": { "lx": 0, "ly": 0, "rx": 0, "ry": 0 },
        # // })

        conn.datachannel.set_decoder(decoder_type="native")
        conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")
        conn.datachannel.pub_sub.subscribe("rt/utlidar/voxel_map_compressed", on_lidar_data)

        return conn

    # Function to be run in a separate thread
    def run_async_setup():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        try:
            conn = loop.run_until_complete(async_setup())

            # Add dispose action to the composite disposable
            def cleanup():
                async def async_cleanup():
                    # Turn off lidar
                    conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "off")
                    # Close connection
                    await conn.close()

                cleanup_loop = asyncio.new_event_loop()
                asyncio.set_event_loop(cleanup_loop)
                cleanup_loop.run_until_complete(async_cleanup())
                cleanup_loop.close()

            dispose.add(Disposable(cleanup))

            # Keep the thread alive until subject is disposed
            while not subject.is_disposed:
                loop.run_until_complete(asyncio.sleep(0.1))

        finally:
            loop.close()

    # Start the setup in a background thread
    thread = threading.Thread(target=run_async_setup, daemon=True)
    thread.start()

    # Add thread stopping logic to the dispose action
    original_dispose = subject.dispose

    def enhanced_dispose():
        original_dispose()
        dispose.dispose()

    subject.dispose = enhanced_dispose  # type: ignore[method-assign]

    # Return the subject directly
    return cast(Subject[LidarMessage], subject)
