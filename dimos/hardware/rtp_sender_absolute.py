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
RTP sender that embeds absolute timestamps in the stream.
This is a simpler alternative to RTSP that doesn't require the RTSP server library.
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
from gi.repository import GLib, Gst

# Initialize GStreamer
Gst.init(None)

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("rtp_sender_absolute")


class RTPVideoSenderAbsolute:
    """Send video with absolute timestamps via RTP."""

    def __init__(
        self,
        device: str = "/dev/video0",
        width: int = 2560,
        height: int = 720,
        framerate: int = 60,
        format_str: str = "YUY2",
        bitrate: int = 5000,
        host: str = "224.0.0.1",
        port: int = 5000,
        multicast_iface: str = "wlo1",
        use_multicast: bool = True,
    ):
        """Initialize the RTP video sender.

        Args:
            device: Video device path
            width: Video width in pixels
            height: Video height in pixels
            framerate: Frame rate in fps
            format_str: Video format
            bitrate: H264 encoding bitrate in kbps
            host: Destination host (multicast group or unicast address)
            port: UDP port for sending
            multicast_iface: Network interface for multicast
            use_multicast: Whether to use multicast
        """
        self.device = device
        self.width = width
        self.height = height
        self.framerate = framerate
        self.format = format_str
        self.bitrate = bitrate
        self.host = host
        self.port = port
        self.multicast_iface = multicast_iface
        self.use_multicast = use_multicast

        self.pipeline = None
        self.main_loop = None
        self.running = False
        self.start_time = None
        self.frame_count = 0

    def create_pipeline(self):
        """Create the GStreamer pipeline with absolute timestamps."""

        # Create pipeline
        self.pipeline = Gst.Pipeline.new("sender-pipeline")

        # Create elements
        videosrc = Gst.ElementFactory.make("v4l2src", "source")
        videosrc.set_property("device", self.device)
        videosrc.set_property("do-timestamp", True)  # Use absolute timestamps

        # Create caps filter for video format
        capsfilter = Gst.ElementFactory.make("capsfilter", "capsfilter")
        caps = Gst.Caps.from_string(
            f"video/x-raw,width={self.width},height={self.height},"
            f"format={self.format},framerate={self.framerate}/1"
        )
        capsfilter.set_property("caps", caps)

        # Video converter
        videoconvert = Gst.ElementFactory.make("videoconvert", "convert")

        # H264 encoder
        encoder = Gst.ElementFactory.make("x264enc", "encoder")
        encoder.set_property("tune", "zerolatency")
        encoder.set_property("bitrate", self.bitrate)
        encoder.set_property("key-int-max", 30)

        # RTP payloader with timestamp preservation
        payloader = Gst.ElementFactory.make("rtph264pay", "payloader")
        payloader.set_property("config-interval", 1)
        payloader.set_property("pt", 96)
        # Important: preserve timestamps
        payloader.set_property("perfect-rtptime", False)
        payloader.set_property("mtu", 1400)

        # UDP sink
        udpsink = Gst.ElementFactory.make("udpsink", "sink")
        udpsink.set_property("host", self.host)
        udpsink.set_property("port", self.port)
        udpsink.set_property("sync", False)

        if self.use_multicast:
            udpsink.set_property("auto-multicast", True)
            udpsink.set_property("multicast-iface", self.multicast_iface)

        # Add elements to pipeline
        self.pipeline.add(videosrc)
        self.pipeline.add(capsfilter)
        self.pipeline.add(videoconvert)
        self.pipeline.add(encoder)
        self.pipeline.add(payloader)
        self.pipeline.add(udpsink)

        # Link elements
        if not videosrc.link(capsfilter):
            raise RuntimeError("Failed to link source to capsfilter")
        if not capsfilter.link(videoconvert):
            raise RuntimeError("Failed to link capsfilter to videoconvert")
        if not videoconvert.link(encoder):
            raise RuntimeError("Failed to link videoconvert to encoder")
        if not encoder.link(payloader):
            raise RuntimeError("Failed to link encoder to payloader")
        if not payloader.link(udpsink):
            raise RuntimeError("Failed to link payloader to udpsink")

        # Use system clock for absolute timestamps
        clock = Gst.SystemClock.obtain()
        self.pipeline.use_clock(clock)

        # Set base time to 0 to use absolute timestamps
        self.pipeline.set_base_time(0)

        # Add probe to monitor timestamps
        payloader_src_pad = payloader.get_static_pad("src")
        payloader_src_pad.add_probe(Gst.PadProbeType.BUFFER, self._monitor_timestamp, None)

        # Set up bus message handling
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    def _monitor_timestamp(self, pad, info, user_data):
        """Monitor timestamps in the RTP stream."""
        buffer = info.get_buffer()
        if buffer and buffer.pts != Gst.CLOCK_TIME_NONE:
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # Log every 30 frames
                pts_seconds = buffer.pts / 1e9
                logger.info(f"Frame {self.frame_count}: timestamp={pts_seconds:.6f}")
        return Gst.PadProbeReturn.OK

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

        logger.info("Creating pipeline with absolute timestamps...")
        self.create_pipeline()

        logger.info("Starting pipeline...")
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            logger.error("Failed to start pipeline")
            raise RuntimeError("Failed to start GStreamer pipeline")

        self.running = True
        self.start_time = time.time()
        self.frame_count = 0

        logger.info("RTP sender with absolute timestamps started:")
        logger.info(f"  Source: {self.device}")
        logger.info(f"  Resolution: {self.width}x{self.height} @ {self.framerate}fps")
        logger.info(f"  Bitrate: {self.bitrate} kbps")
        if self.use_multicast:
            logger.info(f"  Multicast: {self.host}:{self.port} on {self.multicast_iface}")
        else:
            logger.info(f"  Unicast: {self.host}:{self.port}")
        logger.info("  Timestamps: Absolute (Unix epoch)")

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

        logger.info("RTP sender stopped")


def main():
    parser = argparse.ArgumentParser(description="RTP video sender with absolute timestamps")

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
        "--host", default="224.0.0.1", help="Destination host (default: 224.0.0.1 for multicast)"
    )
    parser.add_argument("--port", type=int, default=5000, help="UDP port (default: 5000)")
    parser.add_argument(
        "--multicast-iface", default="wlo1", help="Network interface for multicast (default: wlo1)"
    )
    parser.add_argument("--unicast", action="store_true", help="Use unicast instead of multicast")

    # Logging options
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Create and start sender
    sender = RTPVideoSenderAbsolute(
        device=args.device,
        width=args.width,
        height=args.height,
        framerate=args.framerate,
        format_str=args.format,
        bitrate=args.bitrate,
        host=args.host,
        port=args.port,
        multicast_iface=args.multicast_iface,
        use_multicast=not args.unicast,
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
