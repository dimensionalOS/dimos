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

from dimos.stream.video_provider import AbstractVideoProvider

from queue import Queue
from dimos.robot.unitree.external.go2_webrtc_connect.go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD, WebRTCConnectionMethod
from dimos.robot.unitree.external.go2_webrtc_connect.go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection
from aiortc import MediaStreamTrack
import asyncio
from reactivex import Observable, create, operators as ops
import logging
import threading
import time

class UnitreeVideoProvider(AbstractVideoProvider):
    def __init__(self, dev_name: str = "UnitreeGo2", connection_method: WebRTCConnectionMethod = WebRTCConnectionMethod.LocalSTA, serial_number: str = None, ip: str = None):
        """Initialize the Unitree video stream with WebRTC connection.
        
        Args:
            dev_name: Name of the device
            connection_method: WebRTC connection method (LocalSTA, LocalAP, Remote)
            serial_number: Serial number of the robot (required for LocalSTA with serial)
            ip: IP address of the robot (required for LocalSTA with IP)
        """
        super().__init__(dev_name)
        self.frame_queue = Queue()
        self.loop = None
        self.asyncio_thread = None
        
        # Initialize WebRTC connection based on method
        if connection_method == WebRTCConnectionMethod.LocalSTA:
            if serial_number:
                self.conn = Go2WebRTCConnection(connection_method, serialNumber=serial_number)
            elif ip:
                self.conn = Go2WebRTCConnection(connection_method, ip=ip)
            else:
                raise ValueError("Either serial_number or ip must be provided for LocalSTA connection")
        elif connection_method == WebRTCConnectionMethod.LocalAP:
            self.conn = Go2WebRTCConnection(connection_method)
        else:
            raise ValueError("Unsupported connection method")

    async def _recv_camera_stream(self, track: MediaStreamTrack):
        """Receive video frames from WebRTC and put them in the queue."""
        while True:
            frame = await track.recv()
            # Convert the frame to a NumPy array in BGR format
            img = frame.to_ndarray(format="bgr24")
            self.frame_queue.put(img)

    def _run_asyncio_loop(self, loop):
        """Run the asyncio event loop in a separate thread."""
        asyncio.set_event_loop(loop)
        
        async def setup():
            try:
                await self.conn.connect()
                self.conn.video.switchVideoChannel(True)
                self.conn.video.add_track_callback(self._recv_camera_stream)

                await self.conn.datachannel.switchToNormalMode()
                # await self.conn.datachannel.sendDamp()

                # await asyncio.sleep(5)

                # await self.conn.datachannel.sendDamp()
                # await asyncio.sleep(5)
                # await self.conn.datachannel.sendStandUp()
                # await asyncio.sleep(5)  

                # Wiggle the robot
                # await self.conn.datachannel.switchToNormalMode()
                # await self.conn.datachannel.sendWiggle()
                #await asyncio.sleep(3)

                # Stretch the robot
                # await self.conn.datachannel.sendStretch()
                # await asyncio.sleep(3)
                
            except Exception as e:
                logging.error(f"Error in WebRTC connection: {e}")
                raise

        loop.run_until_complete(setup())
        loop.run_forever()

    def capture_video_as_observable(self, fps: int = 30) -> Observable:
        """Create an observable that emits video frames at the specified FPS.
        
        Args:
            fps: Frames per second to emit (default: 30)
            
        Returns:
            Observable emitting video frames
        """
        frame_interval = 1.0 / fps

        def emit_frames(observer, scheduler):
            try:
                # Start asyncio loop if not already running
                if not self.loop:
                    self.loop = asyncio.new_event_loop()
                    self.asyncio_thread = threading.Thread(
                        target=self._run_asyncio_loop,
                        args=(self.loop,)
                    )
                    self.asyncio_thread.start()

                frame_time = time.monotonic()
                
                while True:
                    if not self.frame_queue.empty():
                        frame = self.frame_queue.get()
                        
                        # Control frame rate
                        now = time.monotonic()
                        next_frame_time = frame_time + frame_interval
                        sleep_time = next_frame_time - now
                        
                        if sleep_time > 0:
                            time.sleep(sleep_time)
                            
                        observer.on_next(frame)
                        frame_time = next_frame_time
                    else:
                        time.sleep(0.001)  # Small sleep to prevent CPU overuse

            except Exception as e:
                logging.error(f"Error during frame emission: {e}")
                observer.on_error(e)
            finally:
                if self.loop:
                    self.loop.call_soon_threadsafe(self.loop.stop)
                if self.asyncio_thread:
                    self.asyncio_thread.join()
                observer.on_completed()

        return create(emit_frames).pipe(
            ops.share()  # Share the stream among multiple subscribers
        )

    def dispose_all(self):
        """Clean up resources."""
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.asyncio_thread:
            self.asyncio_thread.join()
        super().dispose_all()
