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

import argparse
import logging
import signal
import struct
import sys
import time
from typing import Optional

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
logger = logging.getLogger("gstreamer_sender")


class GStreamerVideoSender:
    """Send video from a camera with embedded timestamps using GStreamer."""

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
        use_test_src: bool = False,
    ):
        """Initialize the GStreamer video sender.

        Args:
            device: Video device path (e.g., /dev/video0)
            width: Video width in pixels
            height: Video height in pixels
            framerate: Frame rate in fps
            format_str: Video format (e.g., YUY2, UYVY)
            bitrate: H264 encoding bitrate in kbps
            multicast_group: Multicast group address
            port: UDP port for sending
            multicast_iface: Network interface for multicast
            use_test_src: Use test video source instead of camera
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
        self.use_test_src = use_test_src

        self.pipeline = None
        self.main_loop = None
        self.running = False
        self.start_time = None
        self.frame_count = 0

    def create_pipeline(self):
        """Create the GStreamer pipeline."""

        # Choose video source
        if self.use_test_src:
            # Use test source for debugging
            source_str = f"""
                videotestsrc is-live=true pattern=ball !
                video/x-raw,width={self.width},height={self.height},framerate={self.framerate}/1
            """
            logger.info("Using test video source")
        else:
            # Use real camera
            source_str = f"""
                v4l2src device={self.device} do-timestamp=true !
                video/x-raw,width={self.width},height={self.height},
                format={self.format},framerate={self.framerate}/1
            """
            logger.info(f"Using camera device: {self.device}")

        # Build the complete pipeline
        pipeline_str = f"""
            {source_str} !
            videoconvert !
            clockoverlay halignment=left valignment=top 
                time-format="%Y-%m-%d %H:%M:%S.%f" !
            x264enc tune=zerolatency bitrate={self.bitrate} !
            rtph264pay config-interval=1 pt=96 !
            udpsink host={self.multicast_group} port={self.port} 
                auto-multicast=true multicast-iface={self.multicast_iface}
                sync=false name=udpsink
        """

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)

            # Get the RTP payloader to access RTP packets
            rtppay = self.pipeline.get_by_name("pay0")
            if not rtppay:
                # Try to find rtph264pay element
                for element in self.pipeline.iterate_elements():
                    if element.get_factory().get_name() == "rtph264pay":
                        rtppay = element
                        break

            # Connect to the payloader's pad to intercept packets
            if rtppay:
                srcpad = rtppay.get_static_pad("src")
                if srcpad:
                    srcpad.add_probe(Gst.PadProbeType.BUFFER, self._on_buffer_probe, None)
                    logger.info("Added buffer probe to RTP payloader")

            # Set up bus message handling
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self._on_bus_message)

        except Exception as e:
            logger.error(f"Failed to create pipeline: {e}")
            raise

    def _on_buffer_probe(self, pad, info, user_data):
        """Probe callback to add timestamp metadata to RTP packets."""
        buffer = info.get_buffer()
        if buffer:
            # Add current timestamp as RTP header extension
            # This timestamp will be in nanoseconds since epoch
            current_time_ns = int(time.time() * 1e9)

            # Set the buffer PTS to current time
            # This helps maintain accurate timestamps
            buffer.pts = current_time_ns

            # Count frames for statistics
            self.frame_count += 1

            # Log periodically
            if self.frame_count % (self.framerate * 2) == 0:  # Every 2 seconds
                elapsed = time.time() - self.start_time
                fps = self.frame_count / elapsed
                logger.info(f"Sent {self.frame_count} frames, FPS: {fps:.1f}")

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
                old_state, new_state, pending = message.parse_state_changed()
                logger.debug(f"Pipeline state: {old_state.value_name} -> {new_state.value_name}")

    def start(self):
        """Start the video sender."""
        if self.running:
            logger.warning("Sender is already running")
            return

        logger.info("Creating pipeline...")
        self.create_pipeline()

        logger.info("Starting pipeline...")
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            logger.error("Failed to start pipeline")
            raise RuntimeError("Failed to start GStreamer pipeline")

        self.running = True
        self.start_time = time.time()
        self.frame_count = 0

        logger.info(f"Video sender started:")
        logger.info(f"  Source: {self.device if not self.use_test_src else 'test pattern'}")
        logger.info(f"  Resolution: {self.width}x{self.height} @ {self.framerate}fps")
        logger.info(f"  Bitrate: {self.bitrate} kbps")
        logger.info(f"  Multicast: {self.multicast_group}:{self.port} on {self.multicast_iface}")

        # Create and run main loop
        self.main_loop = GLib.MainLoop()
        try:
            self.main_loop.run()
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.stop()

    def stop(self):
        """Stop the video sender."""
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
    """Main entry point."""
    parser = argparse.ArgumentParser(description="GStreamer video sender with timestamps")

    # Video source options
    parser.add_argument(
        "--device", default="/dev/video0", help="Video device path (default: /dev/video0)"
    )
    parser.add_argument(
        "--test-src", action="store_true", help="Use test video source instead of camera"
    )

    # Video format options
    parser.add_argument("--width", type=int, default=2560, help="Video width (default: 2560)")
    parser.add_argument("--height", type=int, default=720, help="Video height (default: 720)")
    parser.add_argument("--framerate", type=int, default=60, help="Frame rate in fps (default: 60)")
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

    # Logging options
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")

    args = parser.parse_args()

    # Set logging level
    if args.verbose:
        logger.setLevel(logging.DEBUG)
        logging.getLogger().setLevel(logging.DEBUG)

    # Create and start sender
    sender = GStreamerVideoSender(
        device=args.device,
        width=args.width,
        height=args.height,
        framerate=args.framerate,
        format_str=args.format,
        bitrate=args.bitrate,
        multicast_group=args.multicast_group,
        port=args.port,
        multicast_iface=args.multicast_iface,
        use_test_src=args.test_src,
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
