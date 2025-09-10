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

logger = setup_logger("dimos.hardware.rtsp_camera", level=logging.INFO)

Gst.init(None)


class RTSPCameraModule(Module):
    """Module that captures frames from an RTSP stream and publishes them as Image messages.

    This module connects to an RTSP server and properly handles absolute timestamps
    from the stream, ensuring frame timestamps match the capture time on the sender.

    To start the RTSP server:
    ```bash
    python3 rtsp_sender.py --device /dev/video0 --port 8554
    ```

    The module will connect to rtsp://localhost:8554/video by default.
    """

    video: Out[Image] = None

    def __init__(
        self,
        rtsp_url: str = "rtsp://localhost:8554/video",
        frame_id: str = "camera",
        buffer_size: int = 0,
        latency: int = 0,
        tcp_mode: bool = False,
        *args,
        **kwargs,
    ):
        """Initialize the RTSP camera module.

        Args:
            rtsp_url: RTSP stream URL
            frame_id: Frame ID for the published images
            buffer_size: Buffer size in bytes (0 for default)
            latency: Latency in milliseconds (0 for default)
            tcp_mode: Use TCP instead of UDP for RTSP transport
        """
        self.rtsp_url = rtsp_url
        self.frame_id = frame_id
        self.buffer_size = buffer_size
        self.latency = latency
        self.tcp_mode = tcp_mode

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
            logger.warning("RTSP camera module is already running")
            return

        self._create_pipeline()
        self._start_pipeline()
        self.running = True
        self.frame_count = 0
        logger.info(f"RTSP camera module started - connecting to {self.rtsp_url}")

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

        logger.info(f"RTSP camera module stopped (received {self.frame_count} frames)")

    def _create_pipeline(self):
        """Create the GStreamer pipeline for RTSP reception."""
        # Build pipeline string
        # rtspsrc handles RTSP protocol and preserves timestamps
        # We use do-timestamp=false to preserve sender timestamps
        # protocols: 4 = TCP, 7 = UDP+TCP (fallback)
        protocols = 4 if self.tcp_mode else 7

        pipeline_str = f"""
            rtspsrc location={self.rtsp_url} 
                protocols={protocols}
                buffer-mode=0
                latency={self.latency}
                do-timestamp=false
                ntp-sync=false
                drop-on-latency=true !
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

            # Get rtspsrc element to configure it
            rtspsrc = self.pipeline.get_by_name("rtspsrc0")
            if rtspsrc:
                # Set buffer size if specified
                if self.buffer_size > 0:
                    rtspsrc.set_property("udp-buffer-size", self.buffer_size)

                # Use system clock to preserve absolute timestamps
                rtspsrc.set_property("use-pipeline-clock", False)

        except Exception as e:
            logger.error(f"Failed to create GStreamer pipeline: {e}")
            raise

    def _start_pipeline(self):
        """Start the GStreamer pipeline and main loop."""
        self.main_loop = GLib.MainLoop()

        # Use system clock for absolute timestamps
        clock = Gst.SystemClock.obtain()
        self.pipeline.use_clock(clock)

        # Set base time to 0 to preserve absolute timestamps from sender
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
        elif t == Gst.MessageType.ELEMENT:
            struct = message.get_structure()
            if struct and struct.get_name() == "rtsp-message":
                logger.debug(f"RTSP message: {struct.to_string()}")

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
        # The RTSP sender uses absolute timestamps, which are preserved through the pipeline
        if buffer.pts != Gst.CLOCK_TIME_NONE:
            # Convert nanoseconds to seconds
            timestamp = buffer.pts / 1e9

            # Log timestamp info periodically for debugging
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # Log every 30 frames
                if self.last_timestamp:
                    time_diff = timestamp - self.last_timestamp
                    fps = 30.0 / time_diff if time_diff > 0 else 0
                    logger.info(
                        f"Frame {self.frame_count}: timestamp={timestamp:.6f}, "
                        f"delta={time_diff:.6f}s, fps={fps:.1f}"
                    )
                else:
                    logger.info(f"Frame {self.frame_count}: timestamp={timestamp:.6f}")
                self.last_timestamp = timestamp
        else:
            # Fallback to current time if no timestamp available
            timestamp = time.time()
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
