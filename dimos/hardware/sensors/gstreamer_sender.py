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
import sys
import time

# Add system path for gi module if needed
if "/usr/lib/python3/dist-packages" not in sys.path:
    sys.path.insert(0, "/usr/lib/python3/dist-packages")

import gi  # type: ignore[import-untyped,import-not-found]

gi.require_version("Gst", "1.0")
gi.require_version("GstVideo", "1.0")
from gi.repository import GLib, Gst  # type: ignore[import-untyped,import-not-found]

# Initialize GStreamer
Gst.init(None)

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("gstreamer_tcp_sender")


class GStreamerTCPSender:
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
        single_camera: bool = False,
    ) -> None:
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
            single_camera: If True, crop to left half (for stereo cameras)
        """
        self.device = device
        self.width = width
        self.height = height
        self.framerate = framerate
        self.format = format_str
        self.bitrate = bitrate
        self.host = host
        self.port = port
        self.single_camera = single_camera

        self.pipeline = None
        self.videosrc = None
        self.encoder = None
        self.mux = None
        self.main_loop = None
        self.running = False
        self.start_time = None
        self.frame_count = 0

    def create_pipeline(self):  # type: ignore[no-untyped-def]
        """Create the GStreamer pipeline with TCP server sink."""

        # Create pipeline
        self.pipeline = Gst.Pipeline.new("tcp-sender-pipeline")

        # Create elements
        self.videosrc = Gst.ElementFactory.make("v4l2src", "source")
        self.videosrc.set_property("device", self.device)  # type: ignore[attr-defined]
        self.videosrc.set_property("do-timestamp", True)  # type: ignore[attr-defined]
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

        # Crop element for single camera mode
        videocrop = None
        if self.single_camera:
            videocrop = Gst.ElementFactory.make("videocrop", "crop")
            # Crop to left half: for 2560x720 stereo, get left 1280x720
            videocrop.set_property("left", 0)
            videocrop.set_property("right", self.width // 2)  # Remove right half
            videocrop.set_property("top", 0)
            videocrop.set_property("bottom", 0)

        # H264 encoder
        self.encoder = Gst.ElementFactory.make("x264enc", "encoder")
        self.encoder.set_property("tune", "zerolatency")  # type: ignore[attr-defined]
        self.encoder.set_property("bitrate", self.bitrate)  # type: ignore[attr-defined]
        self.encoder.set_property("key-int-max", 30)  # type: ignore[attr-defined]

        # H264 parser
        h264parse = Gst.ElementFactory.make("h264parse", "parser")

        # Use matroskamux which preserves timestamps better
        self.mux = Gst.ElementFactory.make("matroskamux", "mux")
        self.mux.set_property("streamable", True)  # type: ignore[attr-defined]
        self.mux.set_property("writing-app", "gstreamer-tcp-sender")  # type: ignore[attr-defined]

        # TCP server sink
        tcpserversink = Gst.ElementFactory.make("tcpserversink", "sink")
        tcpserversink.set_property("host", self.host)
        tcpserversink.set_property("port", self.port)
        tcpserversink.set_property("sync", False)

        # Add elements to pipeline
        self.pipeline.add(self.videosrc)  # type: ignore[attr-defined]
        self.pipeline.add(capsfilter)  # type: ignore[attr-defined]
        self.pipeline.add(videoconvert)  # type: ignore[attr-defined]
        if videocrop:
            self.pipeline.add(videocrop)  # type: ignore[attr-defined]
        self.pipeline.add(self.encoder)  # type: ignore[attr-defined]
        self.pipeline.add(h264parse)  # type: ignore[attr-defined]
        self.pipeline.add(self.mux)  # type: ignore[attr-defined]
        self.pipeline.add(tcpserversink)  # type: ignore[attr-defined]

        # Link elements
        if not self.videosrc.link(capsfilter):  # type: ignore[attr-defined]
            raise RuntimeError("Failed to link source to capsfilter")
        if not capsfilter.link(videoconvert):
            raise RuntimeError("Failed to link capsfilter to videoconvert")

        # Link through crop if in single camera mode
        if videocrop:
            if not videoconvert.link(videocrop):
                raise RuntimeError("Failed to link videoconvert to videocrop")
            if not videocrop.link(self.encoder):
                raise RuntimeError("Failed to link videocrop to encoder")
        else:
            if not videoconvert.link(self.encoder):
                raise RuntimeError("Failed to link videoconvert to encoder")

        if not self.encoder.link(h264parse):  # type: ignore[attr-defined]
            raise RuntimeError("Failed to link encoder to h264parse")
        if not h264parse.link(self.mux):
            raise RuntimeError("Failed to link h264parse to mux")
        if not self.mux.link(tcpserversink):  # type: ignore[attr-defined]
            raise RuntimeError("Failed to link mux to tcpserversink")

        # Add probe to inject absolute timestamps
        # Place probe after crop (if present) or after videoconvert
        if videocrop:
            probe_element = videocrop
        else:
            probe_element = videoconvert
        probe_pad = probe_element.get_static_pad("src")
        probe_pad.add_probe(Gst.PadProbeType.BUFFER, self._inject_absolute_timestamp, None)

        # Set up bus message handling
        bus = self.pipeline.get_bus()  # type: ignore[attr-defined]
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    def _inject_absolute_timestamp(self, pad, info, user_data):  # type: ignore[no-untyped-def]
        buffer = info.get_buffer()
        if buffer:
            absolute_time = time.time()
            absolute_time_ns = int(absolute_time * 1e9)

            # Set both PTS and DTS to the absolute time
            # This will be preserved by matroskamux
            buffer.pts = absolute_time_ns
            buffer.dts = absolute_time_ns

            self.frame_count += 1
        return Gst.PadProbeReturn.OK

    def _on_bus_message(self, bus, message) -> None:  # type: ignore[no-untyped-def]
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
                old_state, new_state, _pending_state = message.parse_state_changed()
                logger.debug(
                    f"Pipeline state changed: {old_state.value_nick} -> {new_state.value_nick}"
                )

    def start(self):  # type: ignore[no-untyped-def]
        if self.running:
            logger.warning("Sender is already running")
            return

        logger.info("Creating TCP pipeline with absolute timestamps...")
        self.create_pipeline()  # type: ignore[no-untyped-call]

        logger.info("Starting pipeline...")
        ret = self.pipeline.set_state(Gst.State.PLAYING)  # type: ignore[attr-defined]
        if ret == Gst.StateChangeReturn.FAILURE:
            logger.error("Failed to start pipeline")
            raise RuntimeError("Failed to start GStreamer pipeline")

        self.running = True
        self.start_time = time.time()  # type: ignore[assignment]
        self.frame_count = 0

        logger.info("TCP video sender started:")
        logger.info(f"  Source: {self.device}")
        if self.single_camera:
            output_width = self.width // 2
            logger.info(f"  Input Resolution: {self.width}x{self.height} @ {self.framerate}fps")
            logger.info(
                f"  Output Resolution: {output_width}x{self.height} @ {self.framerate}fps (left camera only)"
            )
        else:
            logger.info(f"  Resolution: {self.width}x{self.height} @ {self.framerate}fps")
        logger.info(f"  Bitrate: {self.bitrate} kbps")
        logger.info(f"  TCP Server: {self.host}:{self.port}")
        logger.info("  Container: Matroska (preserves absolute timestamps)")
        logger.info("  Waiting for client connections...")

        self.main_loop = GLib.MainLoop()
        try:
            self.main_loop.run()  # type: ignore[attr-defined]
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.stop()

    def stop(self) -> None:
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


def main() -> None:
    parser = argparse.ArgumentParser(
        description="GStreamer TCP video sender with absolute timestamps"
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

    # Camera options
    parser.add_argument(
        "--single-camera",
        action="store_true",
        help="Extract left camera only from stereo feed (crops 2560x720 to 1280x720)",
    )

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
        single_camera=args.single_camera,
    )

    # Handle signals gracefully
    def signal_handler(sig, frame) -> None:  # type: ignore[no-untyped-def]
        logger.info(f"Received signal {sig}, shutting down...")
        sender.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        sender.start()  # type: ignore[no-untyped-call]
    except Exception as e:
        logger.error(f"Failed to start sender: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
