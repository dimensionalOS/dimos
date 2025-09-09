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

logger = setup_logger("dimos.hardware.gstreamer_camera", level=logging.INFO)

Gst.init(None)


class GstreamerCameraModule(Module):
    """Module that captures frames from a remote camera using GStreamer and publishes them as Image messages.

    To send the video from the server:

    ```bash
    python3 gstreamer_sender_with_metadata.py --device /dev/video0 --multicast-iface wlo1
    ```

    Note: change wlo1 with your actual network interface.
    """

    video: Out[Image] = None

    def __init__(
        self,
        multicast_group: str = "224.0.0.1",
        port: int = 5000,
        multicast_iface: str = "enp109s0",
        frame_id: str = "camera",
        timestamp_offset: float = 0.0,
        *args,
        **kwargs,
    ):
        """Initialize the GStreamer camera module.

        Args:
            multicast_group: Multicast group address for receiving video
            port: UDP port for receiving video
            multicast_iface: Network interface for multicast
            frame_id: Frame ID for the published images
            timestamp_offset: Offset to add to timestamps (useful for clock synchronization)
        """
        self.multicast_group = multicast_group
        self.port = port
        self.multicast_iface = multicast_iface
        self.frame_id = frame_id
        self.timestamp_offset = timestamp_offset

        self.pipeline = None
        self.appsink = None
        self.main_loop = None
        self.main_loop_thread = None
        self.running = False

        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        if self.running:
            logger.warning("GStreamer camera module is already running")
            return

        self._create_pipeline()
        self._start_pipeline()
        self.running = True
        logger.info(
            f"GStreamer camera module started - receiving from {self.multicast_group}:{self.port} on {self.multicast_iface}"
        )

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

        logger.info("GStreamer camera module stopped")

    def _create_pipeline(self):
        # Note: We add do-timestamp=false to udpsrc to preserve original timestamps
        # and use-pipeline-clock=false on rtpjitterbuffer to avoid overwriting them
        pipeline_str = f"""
            udpsrc multicast-group={self.multicast_group} port={self.port} 
                multicast-iface={self.multicast_iface} do-timestamp=false !
            application/x-rtp,payload=96,clock-rate=90000 !
            rtpjitterbuffer mode=slave !
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
        pts_seconds = buffer.pts / 1e9
        timestamp = pts_seconds + self.timestamp_offset

        # Map the buffer to access the data
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            logger.error("Failed to map buffer")
            return Gst.FlowReturn.ERROR

        try:
            # Convert buffer data to numpy array
            # The videoconvert element outputs BGR format
            data = np.frombuffer(map_info.data, dtype=np.uint8)

            # Reshape to image dimensions
            # For BGR format, we have 3 channels
            image_array = data.reshape((height, width, 3))

            # Create an Image message with the extracted timestamp
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
