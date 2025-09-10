#!/usr/bin/env python3

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

import logging
import socket
import struct
import sys
import threading

import numpy as np

from dimos.core import Module, Out, rpc
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

# Add system path for gi module if needed
if "/usr/lib/python3/dist-packages" not in sys.path:
    sys.path.insert(0, "/usr/lib/python3/dist-packages")

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GLib, GstApp

logger = setup_logger("dimos.hardware.tcp_camera", level=logging.INFO)

Gst.init(None)

# Frame header format: timestamp (double), frame_size (uint32)
HEADER_FORMAT = "!dI"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


class TCPCameraModule(Module):
    """Module that receives video frames with absolute timestamps over TCP.

    This module connects to a TCP server that sends H264 frames with
    absolute Unix timestamps embedded in the frame headers.

    To start the TCP server:
    ```bash
    python3 tcp_sender.py --device /dev/video0 --port 5555
    ```
    """

    video: Out[Image] = None

    def __init__(
        self,
        host: str = "localhost",
        port: int = 5555,
        frame_id: str = "camera",
        reconnect_delay: float = 2.0,
        *args,
        **kwargs,
    ):
        """Initialize the TCP camera module.

        Args:
            host: TCP server host
            port: TCP server port
            frame_id: Frame ID for the published images
            reconnect_delay: Delay between reconnection attempts in seconds
        """
        self.host = host
        self.port = port
        self.frame_id = frame_id
        self.reconnect_delay = reconnect_delay

        self.pipeline = None
        self.appsrc = None
        self.appsink = None
        self.socket = None
        self.receive_thread = None
        self.running = False
        self.frame_count = 0

        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        if self.running:
            logger.warning("TCP camera module is already running")
            return

        self.running = True
        self.frame_count = 0

        # Create GStreamer pipeline
        self._create_pipeline()
        self._start_pipeline()

        # Start TCP receive thread
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        logger.info(f"TCP camera module started - connecting to {self.host}:{self.port}")

    @rpc
    def stop(self):
        if not self.running:
            return

        self.running = False

        # Close socket
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None

        # Stop pipeline
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)

        # Wait for thread to stop
        if self.receive_thread:
            self.receive_thread.join(timeout=2.0)

        logger.info(f"TCP camera module stopped (received {self.frame_count} frames)")

    def _create_pipeline(self):
        """Create GStreamer pipeline for H264 decoding."""
        pipeline_str = """
            appsrc name=source is-live=true format=3 !
            video/x-h264,stream-format=byte-stream,alignment=au !
            h264parse !
            avdec_h264 !
            videoconvert !
            video/x-raw,format=BGR !
            appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true
        """

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsrc = self.pipeline.get_by_name("source")
            self.appsink = self.pipeline.get_by_name("sink")
            self.appsink.connect("new-sample", self._on_new_sample)

            # Set caps on appsrc
            caps = Gst.Caps.from_string("video/x-h264,stream-format=byte-stream,alignment=au")
            self.appsrc.set_property("caps", caps)

            # Add bus watch for debugging
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self._on_bus_message)
        except Exception as e:
            logger.error(f"Failed to create GStreamer pipeline: {e}")
            raise

    def _start_pipeline(self):
        """Start the GStreamer pipeline."""
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            logger.error("Unable to set the pipeline to playing state")
            raise RuntimeError("Failed to start GStreamer pipeline")

    def _receive_loop(self):
        """Main loop for receiving TCP frames."""
        while self.running:
            try:
                # Connect to server
                if not self.socket:
                    self._connect()

                # Receive frame header
                header_data = self._receive_exact(HEADER_SIZE)
                if not header_data:
                    raise socket.error("Connection closed")

                # Parse header
                timestamp, frame_size = struct.unpack(HEADER_FORMAT, header_data)
                print(f"Received header: ts={timestamp:.3f}, size={frame_size}")

                # Receive frame data
                frame_data = self._receive_exact(frame_size)
                if not frame_data:
                    raise socket.error("Connection closed")

                # Push frame to GStreamer with timestamp
                buffer = Gst.Buffer.new_wrapped(frame_data)
                buffer.pts = int(timestamp * 1e9)  # Convert to nanoseconds
                buffer.dts = buffer.pts

                # Push to pipeline
                ret = self.appsrc.push_buffer(buffer)
                print(f"Push buffer result: {ret}")
                if ret != Gst.FlowReturn.OK:
                    logger.warning(f"Failed to push buffer: {ret}")

                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    logger.debug(f"Received frame {self.frame_count}, ts={timestamp:.3f}")

                # Store timestamp for decoded frame
                self.last_timestamp = timestamp

            except (socket.error, ConnectionError) as e:
                if self.running:
                    logger.warning(f"Connection error: {e}")
                    self._disconnect()
                    logger.info(f"Reconnecting in {self.reconnect_delay} seconds...")
                    import time

                    time.sleep(self.reconnect_delay)
            except Exception as e:
                if self.running:
                    logger.error(f"Error in receive loop: {e}")

    def _connect(self):
        """Connect to TCP server."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.socket.connect((self.host, self.port))
            logger.info(f"Connected to TCP server at {self.host}:{self.port}")
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            self._disconnect()
            raise

    def _disconnect(self):
        """Disconnect from TCP server."""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None

    def _receive_exact(self, size):
        """Receive exactly size bytes from socket."""
        data = b""
        while len(data) < size:
            chunk = self.socket.recv(size - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def _on_bus_message(self, bus, message):
        """Handle bus messages for debugging."""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            logger.error(f"Pipeline error: {err}, {debug}")
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            logger.warning(f"Pipeline warning: {err}, {debug}")
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending = message.parse_state_changed()
                print(
                    f"Pipeline state changed from {old_state.value_nick} to {new_state.value_nick}"
                )
        return True

    def _on_new_sample(self, appsink):
        """Handle decoded video frames."""
        print("New sample received")
        sample = appsink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        buffer = sample.get_buffer()
        caps = sample.get_caps()

        # Extract video format
        struct = caps.get_structure(0)
        width = struct.get_value("width")
        height = struct.get_value("height")

        # Use the absolute timestamp from TCP header
        timestamp = self.last_timestamp if hasattr(self, "last_timestamp") else 0

        # Map buffer to access data
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            logger.error("Failed to map buffer")
            return Gst.FlowReturn.ERROR

        try:
            # Convert to numpy array
            data = np.frombuffer(map_info.data, dtype=np.uint8)
            image_array = data.reshape((height, width, 3))

            # Create Image message with absolute timestamp
            image_msg = Image(
                data=image_array.copy(),
                format=ImageFormat.BGR,
                frame_id=self.frame_id,
                ts=timestamp,
            )

            # Publish the image
            if self.video and self.running:
                self.video.publish(image_msg)

        except Exception as e:
            logger.error(f"Error processing frame: {e}")

        finally:
            buffer.unmap(map_info)

        return Gst.FlowReturn.OK

    def __del__(self):
        self.stop()
