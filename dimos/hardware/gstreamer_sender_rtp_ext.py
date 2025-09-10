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

"""
GStreamer sender that uses RTP header extensions to transmit absolute timestamps.
This approach uses the standard RTP timestamp extensions.
"""

import argparse
import logging
import signal
import struct
import sys
import time

# Add system path for gi module if needed
if "/usr/lib/python3/dist-packages" not in sys.path:
    sys.path.insert(0, "/usr/lib/python3/dist-packages")

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtp", "1.0")
from gi.repository import GLib, Gst, GstRtp

# Initialize GStreamer
Gst.init(None)

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("gstreamer_sender_rtp_ext")


class GStreamerVideoSenderWithRTPExt:
    """Send video with absolute timestamps in RTP header extensions."""

    def __init__(
        self,
        device: str = "/dev/video0",
        width: int = 2560,
        height: int = 720,
        framerate: int = 60,
        format_str: str = "YUY2",
        bitrate: int = 5000,
        multicast_group: str = "224.0.0.1",
        port: int = 5000,
        multicast_iface: str = "wlo1",
    ):
        """Initialize the GStreamer video sender with RTP extension support.

        Args:
            device: Video device path
            width: Video width in pixels
            height: Video height in pixels
            framerate: Frame rate in fps
            format_str: Video format
            bitrate: H264 encoding bitrate in kbps
            multicast_group: Multicast group address
            port: UDP port for sending
            multicast_iface: Network interface for multicast
        """
        self.device = device
        self.width = width
        self.height = height
        self.framerate = framerate
        self.format = format_str
        self.bitrate = bitrate
        self.multicast_group = multicast_group
        self.port = port
        self.multicast_iface = multicast_iface

        self.pipeline = None
        self.main_loop = None
        self.running = False
        self.frame_count = 0
        self.start_time = None

    def add_rtp_timestamp_extension(self, pad, info, user_data):
        """Add absolute timestamp to RTP header extension."""
        buffer = info.get_buffer()
        if not buffer:
            return Gst.PadProbeReturn.OK

        # Try to map the buffer for writing first
        # If it fails, make a copy
        rtp_buffer = GstRtp.RTPBuffer()
        success = rtp_buffer.map(buffer, Gst.MapFlags.WRITE | Gst.MapFlags.READ)

        if not success:
            # Buffer is not writable, make a copy
            buffer = buffer.copy_deep()
            if not buffer:
                logger.error("Failed to create writable buffer copy")
                return Gst.PadProbeReturn.OK
            # Replace the buffer in the probe info
            info.set_buffer(buffer)
            # Try mapping again
            success = rtp_buffer.map(buffer, Gst.MapFlags.WRITE | Gst.MapFlags.READ)

        if success:
            try:
                current_time = time.time()

                # Convert timestamp to NTP format (seconds since 1900)
                # NTP epoch is January 1, 1900
                # Unix epoch is January 1, 1970
                # Difference is 2208988800 seconds
                ntp_timestamp = current_time + 2208988800

                # NTP timestamp is 64 bits: 32 bits seconds, 32 bits fraction
                ntp_seconds = int(ntp_timestamp)
                ntp_fraction = int((ntp_timestamp - ntp_seconds) * (2**32))

                # Pack as 8 bytes
                timestamp_bytes = struct.pack(">II", ntp_seconds, ntp_fraction)

                # Add as RTP header extension
                # Extension ID 1 is commonly used for timestamps
                rtp_buffer.add_extension_onebyte_header(1, timestamp_bytes)

                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    logger.debug(f"Added RTP extension with timestamp {current_time}")

            finally:
                rtp_buffer.unmap()
        else:
            logger.error("Failed to map RTP buffer even after copy")

        return Gst.PadProbeReturn.OK

    def create_pipeline(self):
        """Create the GStreamer pipeline with RTP extensions."""

        # Build pipeline string
        pipeline_str = f"""
            v4l2src device={self.device} do-timestamp=true !
            video/x-raw,width={self.width},height={self.height},format={self.format},framerate={self.framerate}/1 !
            videoconvert !
            x264enc tune=zerolatency bitrate={self.bitrate} key-int-max=30 !
            rtph264pay config-interval=1 pt=96 !
            udpsink host={self.multicast_group} port={self.port} auto-multicast=true 
                multicast-iface={self.multicast_iface} sync=false
        """

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)

            # Get the payloader element to add probe
            # Find rtph264pay element
            it = self.pipeline.iterate_elements()
            while True:
                result, element = it.next()
                if result != Gst.IteratorResult.OK:
                    break
                if element and element.get_factory().get_name() == "rtph264pay":
                    # Add probe after payloader to modify RTP headers
                    src_pad = element.get_static_pad("src")
                    src_pad.add_probe(
                        Gst.PadProbeType.BUFFER, self.add_rtp_timestamp_extension, None
                    )
                    logger.info("Added RTP timestamp extension probe")
                    break

        except Exception as e:
            logger.error(f"Failed to create pipeline: {e}")
            raise

        # Set up bus message handling
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    def _on_bus_message(self, bus, message):
        """Handle GStreamer bus messages."""
        t = message.type

        if t == Gst.MessageType.EOS:
            logger.info("End of stream")
            self.stop()
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            logger.error(f"Pipeline error: {err}, {debug}")
            self.stop()
        elif t == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            logger.warning(f"Pipeline warning: {warn}, {debug}")

    def start(self):
        if self.running:
            logger.warning("Sender is already running")
            return

        logger.info("Creating pipeline with RTP header extension support...")
        self.create_pipeline()

        logger.info("Starting pipeline...")
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            logger.error("Failed to start pipeline")
            raise RuntimeError("Failed to start GStreamer pipeline")

        self.running = True
        self.start_time = time.time()
        self.frame_count = 0

        logger.info("Video sender with RTP extensions started:")
        logger.info(f"  Source: {self.device}")
        logger.info(f"  Resolution: {self.width}x{self.height} @ {self.framerate}fps")
        logger.info(f"  Bitrate: {self.bitrate} kbps")
        logger.info(f"  Multicast: {self.multicast_group}:{self.port} on {self.multicast_iface}")

        self.main_loop = GLib.MainLoop()
        try:
            self.main_loop.run()
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.stop()

    def stop(self):
        if not self.running:
            return

        self.running = False

        if self.pipeline:
            logger.info("Stopping pipeline...")
            self.pipeline.set_state(Gst.State.NULL)

        if self.main_loop and self.main_loop.is_running():
            self.main_loop.quit()

        if self.frame_count > 0 and self.start_time:
            elapsed = time.time() - self.start_time
            avg_fps = self.frame_count / elapsed
            logger.info(f"Total frames sent: {self.frame_count}, Average FPS: {avg_fps:.1f}")

        logger.info("Video sender stopped")


def main():
    parser = argparse.ArgumentParser(
        description="GStreamer video sender with RTP header extension timestamps"
    )

    # Video source options
    parser.add_argument(
        "--device", default="/dev/video0", help="Video device path (default: /dev/video0)"
    )

    # Video format options
    parser.add_argument("--width", type=int, default=2560, help="Video width (default: 2560)")
    parser.add_argument("--height", type=int, default=720, help="Video height (default: 720)")
    parser.add_argument("--framerate", type=int, default=15, help="Frame rate in fps (default: 15)")
    parser.add_argument("--format", default="YUY2", help="Video format (default: YUY2)")

    # Encoding options
    parser.add_argument(
        "--bitrate", type=int, default=5000, help="H264 bitrate in kbps (default: 5000)"
    )

    # Network options
    parser.add_argument(
        "--multicast-group",
        default="224.0.0.1",
        help="Multicast group address (default: 224.0.0.1)",
    )
    parser.add_argument("--port", type=int, default=5000, help="UDP port (default: 5000)")
    parser.add_argument(
        "--multicast-iface", default="wlo1", help="Network interface for multicast (default: wlo1)"
    )

    args = parser.parse_args()

    # Create and start sender
    sender = GStreamerVideoSenderWithRTPExt(
        device=args.device,
        width=args.width,
        height=args.height,
        framerate=args.framerate,
        format_str=args.format,
        bitrate=args.bitrate,
        multicast_group=args.multicast_group,
        port=args.port,
        multicast_iface=args.multicast_iface,
    )

    # Handle signals gracefully
    def signal_handler(sig, frame):
        logger.info(f"Received signal {sig}, shutting down...")
        sender.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        sender.start()
    except Exception as e:
        logger.error(f"Failed to start sender: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
