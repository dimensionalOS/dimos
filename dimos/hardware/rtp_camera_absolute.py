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
import sys
import threading
import time

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
from gi.repository import Gst, GLib

logger = setup_logger("dimos.hardware.rtp_camera_absolute", level=logging.INFO)

Gst.init(None)


class RTPCameraAbsoluteModule(Module):
    """Module that captures frames from RTP stream with absolute timestamps.

    This module receives RTP video with absolute timestamps preserved from the sender.

    To start the RTP sender:
    ```bash
    python3 rtp_sender_absolute.py --device /dev/video0 --multicast-iface wlo1
    ```
    """

    video: Out[Image] = None

    def __init__(
        self,
        host: str = "224.0.0.1",
        port: int = 5000,
        multicast_iface: str = "enp109s0",
        frame_id: str = "camera",
        use_multicast: bool = True,
        *args,
        **kwargs,
    ):
        """Initialize the RTP camera module.

        Args:
            host: Source host (multicast group or sender address)
            port: UDP port for receiving video
            multicast_iface: Network interface for multicast
            frame_id: Frame ID for the published images
            use_multicast: Whether to use multicast
        """
        self.host = host
        self.port = port
        self.multicast_iface = multicast_iface
        self.frame_id = frame_id
        self.use_multicast = use_multicast

        self.pipeline = None
        self.appsink = None
        self.main_loop = None
        self.main_loop_thread = None
        self.running = False
        self.frame_count = 0
        self.last_timestamp = None

        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        if self.running:
            logger.warning("RTP camera module is already running")
            return

        self._create_pipeline()
        self._start_pipeline()
        self.running = True
        self.frame_count = 0

        if self.use_multicast:
            logger.info(
                f"RTP camera started - receiving from multicast {self.host}:{self.port} on {self.multicast_iface}"
            )
        else:
            logger.info(f"RTP camera started - receiving from {self.host}:{self.port}")

    @rpc
    def stop(self):
        if not self.running:
            return

        self.running = False

        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)

        if self.main_loop:
            self.main_loop.quit()

        if self.main_loop_thread:
            self.main_loop_thread.join(timeout=2.0)

        logger.info(f"RTP camera stopped (received {self.frame_count} frames)")

    def _create_pipeline(self):
        """Create the GStreamer pipeline for RTP reception with absolute timestamps."""

        # Build pipeline string based on multicast/unicast mode
        if self.use_multicast:
            # Multicast reception
            pipeline_str = f"""
                udpsrc multicast-group={self.host} port={self.port}
                    multicast-iface={self.multicast_iface} 
                    do-timestamp=false !
                application/x-rtp,payload=96,clock-rate=90000,encoding-name=H264 !
                rtpjitterbuffer mode=none !
                rtph264depay !
                avdec_h264 !
                videoconvert !
                video/x-raw,format=BGR !
                appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true
            """
        else:
            # Unicast reception
            pipeline_str = f"""
                udpsrc port={self.port} do-timestamp=false !
                application/x-rtp,payload=96,clock-rate=90000,encoding-name=H264 !
                rtpjitterbuffer mode=none !
                rtph264depay !
                avdec_h264 !
                videoconvert !
                video/x-raw,format=BGR !
                appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true
            """

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsink = self.pipeline.get_by_name("sink")
            self.appsink.connect("new-sample", self._on_new_sample)
        except Exception as e:
            logger.error(f"Failed to create GStreamer pipeline: {e}")
            raise

    def _start_pipeline(self):
        """Start the GStreamer pipeline and main loop."""
        self.main_loop = GLib.MainLoop()

        # Use system clock for absolute timestamps
        clock = Gst.SystemClock.obtain()
        self.pipeline.use_clock(clock)

        # Set base time to 0 to preserve absolute timestamps
        self.pipeline.set_base_time(0)

        # Start the pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            logger.error("Unable to set the pipeline to playing state")
            raise RuntimeError("Failed to start GStreamer pipeline")

        # Run the main loop in a separate thread
        self.main_loop_thread = threading.Thread(target=self._run_main_loop)
        self.main_loop_thread.daemon = True
        self.main_loop_thread.start()

        # Set up bus message handling
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    def _run_main_loop(self):
        try:
            self.main_loop.run()
        except Exception as e:
            logger.error(f"Main loop error: {e}")

    def _on_bus_message(self, bus, message):
        t = message.type

        if t == Gst.MessageType.EOS:
            logger.info("End of stream")
            self.stop()
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            logger.error(f"GStreamer error: {err}, {debug}")
            self.stop()
        elif t == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            logger.warning(f"GStreamer warning: {warn}, {debug}")

    def _on_new_sample(self, appsink):
        """Handle new video samples from the appsink."""
        sample = appsink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        buffer = sample.get_buffer()
        caps = sample.get_caps()

        # Extract video format information
        struct = caps.get_structure(0)
        width = struct.get_value("width")
        height = struct.get_value("height")

        # Extract timestamp from buffer
        # The sender uses absolute timestamps which are preserved
        if buffer.pts != Gst.CLOCK_TIME_NONE:
            # Convert nanoseconds to seconds
            timestamp = buffer.pts / 1e9

            # Log timestamp info periodically for debugging
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # Log every 30 frames
                current_time = time.time()
                latency = current_time - timestamp

                if self.last_timestamp:
                    time_diff = timestamp - self.last_timestamp
                    fps = 30.0 / time_diff if time_diff > 0 else 0
                    logger.info(
                        f"Frame {self.frame_count}: ts={timestamp:.3f}, "
                        f"latency={latency:.3f}s, fps={fps:.1f}"
                    )
                else:
                    logger.info(
                        f"Frame {self.frame_count}: ts={timestamp:.3f}, latency={latency:.3f}s"
                    )

                    # Check if timestamps are absolute
                    if timestamp > 1577836800:  # > Jan 1, 2020
                        logger.info("✓ Using absolute timestamps from sender")
                    else:
                        logger.warning("⚠ Timestamps appear to be relative, not absolute")

                self.last_timestamp = timestamp
        else:
            # Fallback to current time if no timestamp
            timestamp = time.time()
            if self.frame_count % 30 == 0:
                logger.warning("No timestamp in buffer, using current time")

        # Map the buffer to access the data
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            logger.error("Failed to map buffer")
            return Gst.FlowReturn.ERROR

        try:
            # Convert buffer data to numpy array
            data = np.frombuffer(map_info.data, dtype=np.uint8)

            # Reshape to image dimensions (BGR format, 3 channels)
            image_array = data.reshape((height, width, 3))

            # Create an Image message with the absolute timestamp
            image_msg = Image(
                data=image_array.copy(),  # Make a copy to ensure data persistence
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
