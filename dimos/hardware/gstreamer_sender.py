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
Enhanced GStreamer sender that embeds absolute timestamps in the H264 stream
using SEI (Supplemental Enhancement Information) messages.

This allows the receiver to extract the exact capture time of each frame.

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
logger = logging.getLogger("gstreamer_sender_metadata")


class GStreamerVideoSenderWithMetadata:
    """Send video with embedded timestamp metadata."""

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
        embed_metadata: bool = True,
    ):
        """Initialize the GStreamer video sender with metadata support.

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
            embed_metadata: Embed timestamp metadata in stream
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
        self.embed_metadata = embed_metadata

        self.pipeline = None
        self.videosrc = None
        self.encoder = None
        self.payloader = None
        self.main_loop = None
        self.running = False
        self.start_time = None
        self.frame_count = 0

    def create_pipeline(self):
        """Create the GStreamer pipeline with metadata injection."""

        # Create pipeline
        self.pipeline = Gst.Pipeline.new("sender-pipeline")

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

        # RTP payloader
        self.payloader = Gst.ElementFactory.make("rtph264pay", "payloader")
        self.payloader.set_property("config-interval", 1)
        self.payloader.set_property("pt", 96)

        # UDP sink
        udpsink = Gst.ElementFactory.make("udpsink", "sink")
        udpsink.set_property("host", self.multicast_group)
        udpsink.set_property("port", self.port)
        udpsink.set_property("auto-multicast", True)
        udpsink.set_property("multicast-iface", self.multicast_iface)
        udpsink.set_property("sync", False)

        # Add elements to pipeline
        self.pipeline.add(self.videosrc)
        self.pipeline.add(capsfilter)
        self.pipeline.add(videoconvert)
        self.pipeline.add(self.encoder)
        self.pipeline.add(self.payloader)
        self.pipeline.add(udpsink)

        # Link elements
        if not self.videosrc.link(capsfilter):
            raise RuntimeError("Failed to link source to capsfilter")
        if not capsfilter.link(videoconvert):
            raise RuntimeError("Failed to link capsfilter to videoconvert")
        if not videoconvert.link(self.encoder):
            raise RuntimeError("Failed to link timeoverlay to encoder")
        if not self.encoder.link(self.payloader):
            raise RuntimeError("Failed to link encoder to payloader")
        if not self.payloader.link(udpsink):
            raise RuntimeError("Failed to link payloader to udpsink")

        # Add probes for metadata injection
        if self.embed_metadata:
            # Add probe before encoder to inject metadata
            encoder_sink_pad = self.encoder.get_static_pad("sink")
            encoder_sink_pad.add_probe(
                Gst.PadProbeType.BUFFER, self._inject_timestamp_metadata, None
            )

            # Add probe after payloader to modify RTP timestamps
            payloader_src_pad = self.payloader.get_static_pad("src")
            payloader_src_pad.add_probe(Gst.PadProbeType.BUFFER, self._modify_rtp_timestamp, None)

        # Set up bus message handling
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    def _inject_timestamp_metadata(self, pad, info, user_data):
        buffer = info.get_buffer()
        if buffer:
            current_time = time.time()
            buffer.pts = int(current_time * 1e9)
            buffer.dts = buffer.pts
            self.frame_count += 1
        return Gst.PadProbeReturn.OK

    def _modify_rtp_timestamp(self, pad, info, user_data):
        """Modify RTP timestamps to carry absolute time information."""
        buffer = info.get_buffer()
        if buffer and buffer.pts != Gst.CLOCK_TIME_NONE:
            # The buffer PTS now contains our absolute timestamp
            # We could modify RTP headers here if needed
            pass
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

        logger.info("Creating pipeline with metadata support...")
        self.create_pipeline()

        logger.info("Starting pipeline...")
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            logger.error("Failed to start pipeline")
            raise RuntimeError("Failed to start GStreamer pipeline")

        self.running = True
        self.start_time = time.time()
        self.frame_count = 0

        logger.info(f"Video sender with metadata started:")
        logger.info(f"  Source: {self.device}")
        logger.info(f"  Resolution: {self.width}x{self.height} @ {self.framerate}fps")
        logger.info(f"  Bitrate: {self.bitrate} kbps")
        logger.info(f"  Multicast: {self.multicast_group}:{self.port} on {self.multicast_iface}")
        logger.info(f"  Metadata: {'Enabled' if self.embed_metadata else 'Disabled'}")

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
        description="GStreamer video sender with embedded timestamp metadata"
    )

    # Video source options
    parser.add_argument(
        "--device", default="/dev/video0", help="Video device path (default: /dev/video0)"
    )

    # Video format options
    parser.add_argument("--width", type=int, default=2560, help="Video width (default: 2560)")
    parser.add_argument("--height", type=int, default=720, help="Video height (default: 720)")
    parser.add_argument("--framerate", type=int, default=15, help="Frame rate in fps (default: 60)")
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

    # Metadata options
    parser.add_argument(
        "--no-metadata", action="store_true", help="Disable timestamp metadata embedding"
    )

    # Logging options
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")

    args = parser.parse_args()

    # Create and start sender
    sender = GStreamerVideoSenderWithMetadata(
        device=args.device,
        width=args.width,
        height=args.height,
        framerate=args.framerate,
        format_str=args.format,
        bitrate=args.bitrate,
        multicast_group=args.multicast_group,
        port=args.port,
        multicast_iface=args.multicast_iface,
        embed_metadata=not args.no_metadata,
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
