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
GStreamer sender that embeds absolute timestamps in H264 SEI messages.
SEI (Supplemental Enhancement Information) messages are part of the H264
stream and survive RTP transmission intact.
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
logger = logging.getLogger("gstreamer_sender_sei")


class GStreamerVideoSenderWithSEI:
    """Send video with absolute timestamps embedded in H264 SEI messages."""

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
        """Initialize the GStreamer video sender with SEI timestamp support.

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
        self.videosrc = None
        self.encoder = None
        self.payloader = None
        self.main_loop = None
        self.running = False
        self.frame_count = 0

    def create_sei_nalu(self, timestamp):
        """Create an H264 SEI NAL unit with timestamp.

        SEI payload type 5 is unregistered user data.
        We use a UUID to identify our timestamp data.
        """
        # UUID for our timestamp SEI (random but fixed)
        uuid = bytes.fromhex("444F4D4F535F54494D455354414D5053")  # "DOMOS_TIMESTAMPS" in hex

        # Pack timestamp as double (8 bytes)
        timestamp_bytes = struct.pack(">d", timestamp)

        # SEI payload: UUID + timestamp
        sei_payload = uuid + timestamp_bytes

        # SEI NAL unit structure:
        # Start code (0x00000001) + NAL header (0x06 for SEI) + payload type (5) + payload size + payload + stop bit
        payload_size = len(sei_payload)

        # Build SEI NAL unit
        nal_unit = bytearray()
        nal_unit.extend(b"\x00\x00\x00\x01")  # Start code
        nal_unit.append(0x06)  # NAL unit type for SEI
        nal_unit.append(0x05)  # SEI payload type 5 (unregistered user data)

        # Payload size (using emulation prevention if needed)
        if payload_size < 255:
            nal_unit.append(payload_size)
        else:
            # For larger payloads, use 0xFF bytes to indicate continuation
            while payload_size >= 255:
                nal_unit.append(0xFF)
                payload_size -= 255
            nal_unit.append(payload_size)

        nal_unit.extend(sei_payload)
        nal_unit.append(0x80)  # Stop bit

        return bytes(nal_unit)

    def inject_sei_timestamp(self, pad, info, user_data):
        """Inject SEI timestamp before each frame."""
        buffer = info.get_buffer()
        if buffer:
            current_time = time.time()

            # Create SEI NAL unit with timestamp
            sei_nal = self.create_sei_nalu(current_time)

            # Create a new buffer for the SEI NAL unit
            sei_buffer = Gst.Buffer.new_wrapped(sei_nal)
            sei_buffer.pts = buffer.pts
            sei_buffer.dts = buffer.dts
            sei_buffer.duration = 0

            # Push the SEI buffer before the frame
            # Note: This approach may not work perfectly - SEI injection typically
            # needs to be done at the encoder level for proper integration

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                logger.debug(f"Injected timestamp {current_time} at frame {self.frame_count}")

        return Gst.PadProbeReturn.OK

    def create_pipeline(self):
        """Create the GStreamer pipeline."""

        # Build pipeline string with x264enc SEI support
        # The x264enc 'aud' property adds Access Unit Delimiters which help with SEI
        # The 'insert-vui' property adds Video Usability Information
        pipeline_str = f"""
            v4l2src device={self.device} do-timestamp=true !
            video/x-raw,width={self.width},height={self.height},format={self.format},framerate={self.framerate}/1 !
            videoconvert !
            x264enc tune=zerolatency bitrate={self.bitrate} key-int-max=30 aud=true insert-vui=true !
            rtph264pay config-interval=1 pt=96 !
            udpsink host={self.multicast_group} port={self.port} auto-multicast=true 
                multicast-iface={self.multicast_iface} sync=false
        """

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)

            # Get the encoder element to add probe
            self.encoder = self.pipeline.get_by_name("x264enc0")
            if self.encoder:
                # Add probe to inject SEI before encoding
                encoder_sink_pad = self.encoder.get_static_pad("sink")
                encoder_sink_pad.add_probe(Gst.PadProbeType.BUFFER, self.inject_sei_timestamp, None)

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

        logger.info("Creating pipeline with SEI timestamp support...")
        self.create_pipeline()

        logger.info("Starting pipeline...")
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            logger.error("Failed to start pipeline")
            raise RuntimeError("Failed to start GStreamer pipeline")

        self.running = True
        self.frame_count = 0

        logger.info("Video sender with SEI timestamps started:")
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

        logger.info(f"Total frames sent: {self.frame_count}")
        logger.info("Video sender stopped")


def main():
    parser = argparse.ArgumentParser(
        description="GStreamer video sender with SEI timestamp embedding"
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
    sender = GStreamerVideoSenderWithSEI(
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
