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
GStreamer TCP sender that preserves absolute timestamps.

This sender uses TCP transport with GDPPAY serialization to maintain
exact capture timestamps through the network transmission.

Ubuntu dependencies:
    sudo apt install python3-gi python3-gi-cairo gir1.2-gstreamer-1.0 \
        gir1.2-gst-plugins-base-1.0 gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x \
        gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 \
        gstreamer1.0-qt5 gstreamer1.0-pulseaudio libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev
"""

import argparse
import logging
import signal
import sys
import time

# Add system path for gi module if needed
if "/usr/lib/python3/dist-packages" not in sys.path:
    sys.path.insert(0, "/usr/lib/python3/dist-packages")

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import GLib, Gst

# Initialize GStreamer
Gst.init(None)

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("gstreamer_tcp_sender")


class GStreamerTCPSender:
    """Send video over TCP with preserved absolute timestamps."""

    def __init__(
        self,
        device: str = "/dev/video0",
        width: int = 2560,
        height: int = 720,
        framerate: int = 60,
        format_str: str = "YUY2",
        bitrate: int = 5000,
        host: str = "0.0.0.0",
        port: int = 5000,
    ):
        """Initialize the GStreamer TCP sender.

        Args:
            device: Video device path
            width: Video width in pixels
            height: Video height in pixels
            framerate: Frame rate in fps
            format_str: Video format
            bitrate: H264 encoding bitrate in kbps
            host: Host to listen on (0.0.0.0 for all interfaces)
            port: TCP port for listening
        """
        self.device = device
        self.width = width
        self.height = height
        self.framerate = framerate
        self.format = format_str
        self.bitrate = bitrate
        self.host = host
        self.port = port

        self.pipeline = None
        self.videosrc = None
        self.encoder = None
        self.main_loop = None
        self.running = False
        self.start_time = None
        self.frame_count = 0
        self.system_start_time = time.time()

    def create_pipeline(self):
        """Create the GStreamer pipeline with TCP server sink."""

        # Create pipeline
        self.pipeline = Gst.Pipeline.new("tcp-sender-pipeline")

        # Create elements
        self.videosrc = Gst.ElementFactory.make("v4l2src", "source")
        self.videosrc.set_property("device", self.device)
        self.videosrc.set_property("do-timestamp", True)
        logger.info(f"Using camera device: {self.device}")

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
        self.encoder = Gst.ElementFactory.make("x264enc", "encoder")
        self.encoder.set_property("tune", "zerolatency")
        self.encoder.set_property("bitrate", self.bitrate)
        self.encoder.set_property("key-int-max", 30)  # Insert keyframe every 30 frames

        # H264 parser to ensure proper stream formatting
        h264parse = Gst.ElementFactory.make("h264parse", "parser")

        # GDP payloader - serializes GStreamer buffers with timestamps
        gdppay = Gst.ElementFactory.make("gdppay", "gdppay")

        # TCP server sink
        tcpserversink = Gst.ElementFactory.make("tcpserversink", "sink")
        tcpserversink.set_property("host", self.host)
        tcpserversink.set_property("port", self.port)
        tcpserversink.set_property("sync", False)

        # Add elements to pipeline
        self.pipeline.add(self.videosrc)
        self.pipeline.add(capsfilter)
        self.pipeline.add(videoconvert)
        self.pipeline.add(self.encoder)
        self.pipeline.add(h264parse)
        self.pipeline.add(gdppay)
        self.pipeline.add(tcpserversink)

        # Link elements
        if not self.videosrc.link(capsfilter):
            raise RuntimeError("Failed to link source to capsfilter")
        if not capsfilter.link(videoconvert):
            raise RuntimeError("Failed to link capsfilter to videoconvert")
        if not videoconvert.link(self.encoder):
            raise RuntimeError("Failed to link videoconvert to encoder")
        if not self.encoder.link(h264parse):
            raise RuntimeError("Failed to link encoder to h264parse")
        if not h264parse.link(gdppay):
            raise RuntimeError("Failed to link h264parse to gdppay")
        if not gdppay.link(tcpserversink):
            raise RuntimeError("Failed to link gdppay to tcpserversink")

        # Add probe to track frames
        encoder_src_pad = self.encoder.get_static_pad("src")
        encoder_src_pad.add_probe(Gst.PadProbeType.BUFFER, self._on_buffer_probe, None)

        # Set up bus message handling
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    def _on_buffer_probe(self, pad, info, user_data):
        """Probe to track frame timestamps."""
        buffer = info.get_buffer()
        if buffer:
            # The buffer already has PTS set by do-timestamp=True
            # This is the absolute timestamp we want to preserve
            if buffer.pts != Gst.CLOCK_TIME_NONE:
                self.frame_count += 1
                if self.frame_count % 30 == 0:  # Log every 30 frames
                    pts_seconds = buffer.pts / 1e9
                    elapsed = time.time() - self.system_start_time
                    logger.debug(
                        f"Frame {self.frame_count}: PTS={pts_seconds:.3f}s, System elapsed={elapsed:.3f}s"
                    )
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
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                logger.debug(
                    f"Pipeline state changed: {old_state.value_nick} -> {new_state.value_nick}"
                )

    def start(self):
        if self.running:
            logger.warning("Sender is already running")
            return

        logger.info("Creating TCP pipeline...")
        self.create_pipeline()

        logger.info("Starting pipeline...")
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            logger.error("Failed to start pipeline")
            raise RuntimeError("Failed to start GStreamer pipeline")

        self.running = True
        self.start_time = time.time()
        self.frame_count = 0

        logger.info(f"TCP video sender started:")
        logger.info(f"  Source: {self.device}")
        logger.info(f"  Resolution: {self.width}x{self.height} @ {self.framerate}fps")
        logger.info(f"  Bitrate: {self.bitrate} kbps")
        logger.info(f"  TCP Server: {self.host}:{self.port}")
        logger.info(f"  Waiting for client connections...")

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

        logger.info("TCP video sender stopped")


def main():
    parser = argparse.ArgumentParser(
        description="GStreamer TCP video sender with preserved absolute timestamps"
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
        "--host",
        default="0.0.0.0",
        help="Host to listen on (default: 0.0.0.0 for all interfaces)",
    )
    parser.add_argument("--port", type=int, default=5000, help="TCP port (default: 5000)")

    # Logging options
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Create and start sender
    sender = GStreamerTCPSender(
        device=args.device,
        width=args.width,
        height=args.height,
        framerate=args.framerate,
        format_str=args.format,
        bitrate=args.bitrate,
        host=args.host,
        port=args.port,
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
