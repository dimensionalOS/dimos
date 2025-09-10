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
RTSP server that sends video with absolute timestamps using ONVIF metadata.
This properly preserves Unix epoch timestamps through the RTSP stream.
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
gi.require_version("GstRtspServer", "1.0")
from gi.repository import GLib, Gst, GstRtspServer

# Initialize GStreamer
Gst.init(None)

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("rtsp_absolute_sender")


class RTSPAbsoluteTimestampServer:
    """RTSP server that preserves absolute Unix timestamps."""

    def __init__(
        self,
        device: str = "/dev/video0",
        width: int = 2560,
        height: int = 720,
        framerate: int = 60,
        format_str: str = "YUY2",
        bitrate: int = 5000,
        port: int = 8554,
        mount_point: str = "/video",
    ):
        """Initialize the RTSP server with absolute timestamp support.

        Args:
            device: Video device path
            width: Video width in pixels
            height: Video height in pixels
            framerate: Frame rate in fps
            format_str: Video format
            bitrate: H264 encoding bitrate in kbps
            port: RTSP server port
            mount_point: RTSP mount point (URL path)
        """
        self.device = device
        self.width = width
        self.height = height
        self.framerate = framerate
        self.format = format_str
        self.bitrate = bitrate
        self.port = port
        self.mount_point = mount_point

        self.server = None
        self.factory = None
        self.main_loop = None
        self.running = False
        self.start_time = None

    def create_pipeline_string(self):
        """Create pipeline with ONVIF timestamp metadata for absolute timestamps."""
        # Use rtponviftimestamp to add absolute timestamp metadata
        # and rtponvifparse on receiver to extract them
        pipeline = (
            f"v4l2src device={self.device} ! "
            f"video/x-raw,width={self.width},height={self.height},format={self.format},framerate={self.framerate}/1 ! "
            f"videoconvert ! "
            f'clockoverlay time-format="%Y-%m-%d %H:%M:%S" ! '  # Visual timestamp for debugging
            f"x264enc tune=zerolatency bitrate={self.bitrate} key-int-max=30 ! "
            f"rtph264pay name=pay0 pt=96 config-interval=1 ! "
            f"rtponviftimestamp name=onvifts ntp-offset=0 set-e-bit=true set-t-bit=true"
        )
        return f"( {pipeline} )"

    def create_server(self):
        """Create and configure the RTSP server."""
        # Create RTSP server
        self.server = GstRtspServer.RTSPServer()
        self.server.set_service(str(self.port))

        # Create media factory
        self.factory = GstRtspServer.RTSPMediaFactory()

        # Set the pipeline launch string
        pipeline_str = self.create_pipeline_string()
        self.factory.set_launch(pipeline_str)

        # Make the stream shareable
        self.factory.set_shared(True)

        # Use NTP timestamps (absolute)
        self.factory.set_clock(None)  # Use default system clock

        # Get mount points and add factory
        mounts = self.server.get_mount_points()
        mounts.add_factory(self.mount_point, self.factory)

        # Attach server to main context
        self.server.attach(None)

        logger.info(f"RTSP server created on port {self.port}")
        logger.info(f"Stream URL: rtsp://localhost:{self.port}{self.mount_point}")

    def start(self):
        """Start the RTSP server."""
        if self.running:
            logger.warning("Server is already running")
            return

        logger.info("Creating RTSP server with absolute timestamps...")
        self.create_server()

        self.running = True
        self.start_time = time.time()

        logger.info("RTSP server started:")
        logger.info(f"  Source: {self.device}")
        logger.info(f"  Resolution: {self.width}x{self.height} @ {self.framerate}fps")
        logger.info(f"  Bitrate: {self.bitrate} kbps")
        logger.info(f"  RTSP URL: rtsp://localhost:{self.port}{self.mount_point}")
        logger.info("  Timestamps: Absolute with ONVIF metadata")

        # Create and run main loop
        self.main_loop = GLib.MainLoop()
        try:
            self.main_loop.run()
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.stop()

    def stop(self):
        """Stop the RTSP server."""
        if not self.running:
            return

        self.running = False

        if self.main_loop and self.main_loop.is_running():
            self.main_loop.quit()

        if self.start_time:
            elapsed = time.time() - self.start_time
            logger.info(f"Server ran for {elapsed:.1f} seconds")

        logger.info("RTSP server stopped")


def main():
    parser = argparse.ArgumentParser(
        description="RTSP server with absolute timestamps using ONVIF metadata"
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

    # RTSP server options
    parser.add_argument("--port", type=int, default=8554, help="RTSP server port (default: 8554)")
    parser.add_argument(
        "--mount-point", default="/video", help="RTSP mount point (default: /video)"
    )

    args = parser.parse_args()

    # Create and start server
    server = RTSPAbsoluteTimestampServer(
        device=args.device,
        width=args.width,
        height=args.height,
        framerate=args.framerate,
        format_str=args.format,
        bitrate=args.bitrate,
        port=args.port,
        mount_point=args.mount_point,
    )

    # Handle signals gracefully
    def signal_handler(sig, frame):
        logger.info(f"Received signal {sig}, shutting down...")
        server.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        server.start()
    except Exception as e:
        logger.error(f"Failed to start server: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
