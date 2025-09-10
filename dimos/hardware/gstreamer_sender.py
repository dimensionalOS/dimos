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
TCP server that sends ZED camera frames with absolute timestamps.
Each frame is sent with a header containing the timestamp and frame size.
"""

import argparse
import logging
import signal
import socket
import struct
import sys
import threading
import time

# Add system path for gi module if needed
if "/usr/lib/python3/dist-packages" not in sys.path:
    sys.path.insert(0, "/usr/lib/python3/dist-packages")

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import GLib, Gst, GstApp

# Initialize GStreamer
Gst.init(None)

# Setup logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("tcp_sender")

# Frame header format: timestamp (double), frame_size (uint32)
HEADER_FORMAT = "!dI"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


class TCPVideoServer:
    """TCP server for video streaming with absolute timestamps."""

    def __init__(
        self,
        device: str = "/dev/video0",
        width: int = 2560,
        height: int = 720,
        framerate: int = 15,
        format_str: str = "YUY2",
        bitrate: int = 5000,
        port: int = 5555,
        host: str = "0.0.0.0",
    ):
        """Initialize the TCP video server.

        Args:
            device: Video device path
            width: Video width in pixels
            height: Video height in pixels
            framerate: Frame rate in fps
            format_str: Video format
            bitrate: H264 encoding bitrate in kbps
            port: TCP server port
            host: Server bind address
        """
        self.device = device
        self.width = width
        self.height = height
        self.framerate = framerate
        self.format = format_str
        self.bitrate = bitrate
        self.port = port
        self.host = host

        self.pipeline = None
        self.appsink = None
        self.main_loop = None
        self.server_socket = None
        self.client_sockets = []
        self.client_lock = threading.Lock()
        self.running = False
        self.frame_count = 0
        self.server_thread = None

    def create_pipeline(self):
        """Create the GStreamer pipeline."""
        pipeline_str = f"""
            v4l2src device={self.device} do-timestamp=true !
            video/x-raw,width={self.width},height={self.height},
                format={self.format},framerate={self.framerate}/1 !
            videoconvert !
            x264enc tune=zerolatency bitrate={self.bitrate} key-int-max=30 !
            h264parse !
            appsink name=sink emit-signals=true sync=false max-buffers=1 drop=false
        """

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsink = self.pipeline.get_by_name("sink")
            self.appsink.connect("new-sample", self._on_new_sample)

            # Use system clock for absolute timestamps
            clock = Gst.SystemClock.obtain()
            self.pipeline.use_clock(clock)
            self.pipeline.set_base_time(0)

        except Exception as e:
            logger.error(f"Failed to create GStreamer pipeline: {e}")
            raise

    def _on_new_sample(self, appsink):
        """Handle new video samples from the appsink."""
        sample = appsink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        buffer = sample.get_buffer()

        # Always use current Unix time for absolute timestamps
        # GStreamer's buffer.pts is relative to pipeline start, not Unix epoch
        timestamp = time.time()

        # Extract H264 data
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            logger.error("Failed to map buffer")
            return Gst.FlowReturn.ERROR

        try:
            frame_data = bytes(map_info.data)
            frame_size = len(frame_data)

            # Create header with timestamp and frame size
            header = struct.pack(HEADER_FORMAT, timestamp, frame_size)

            # Send to all connected clients
            with self.client_lock:
                disconnected_clients = []
                for client_socket in self.client_sockets:
                    try:
                        # Send header + frame data
                        client_socket.sendall(header + frame_data)
                        self.frame_count += 1

                        if self.frame_count % 30 == 0:
                            logger.debug(
                                f"Sent frame {self.frame_count}, ts={timestamp:.3f}, size={frame_size}"
                            )
                    except (socket.error, BrokenPipeError):
                        disconnected_clients.append(client_socket)

                # Remove disconnected clients
                for client in disconnected_clients:
                    self.client_sockets.remove(client)
                    try:
                        client.close()
                    except:
                        pass
                    logger.info(f"Client disconnected. Active clients: {len(self.client_sockets)}")

        finally:
            buffer.unmap(map_info)

        return Gst.FlowReturn.OK

    def start_tcp_server(self):
        """Start the TCP server to accept client connections."""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)

        logger.info(f"TCP server listening on {self.host}:{self.port}")

        self.server_thread = threading.Thread(target=self._accept_clients)
        self.server_thread.daemon = True
        self.server_thread.start()

    def _accept_clients(self):
        """Accept client connections in a separate thread."""
        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                client_socket, addr = self.server_socket.accept()

                # Disable Nagle's algorithm for lower latency
                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

                with self.client_lock:
                    self.client_sockets.append(client_socket)

                logger.info(
                    f"Client connected from {addr}. Active clients: {len(self.client_sockets)}"
                )
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    logger.error(f"Error accepting client: {e}")

    def start(self):
        """Start the TCP video server."""
        if self.running:
            logger.warning("Server is already running")
            return

        self.running = True
        self.frame_count = 0

        # Start TCP server
        self.start_tcp_server()

        # Create and start GStreamer pipeline
        logger.info("Creating GStreamer pipeline...")
        self.create_pipeline()

        logger.info("Starting pipeline...")
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            logger.error("Failed to start pipeline")
            raise RuntimeError("Failed to start GStreamer pipeline")

        logger.info("TCP video server started:")
        logger.info(f"  Source: {self.device}")
        logger.info(f"  Resolution: {self.width}x{self.height} @ {self.framerate}fps")
        logger.info(f"  Bitrate: {self.bitrate} kbps")
        logger.info(f"  TCP Port: {self.port}")
        logger.info("  Timestamps: Absolute (Unix epoch)")

        # Set up bus message handling
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        # Run main loop
        self.main_loop = GLib.MainLoop()
        try:
            self.main_loop.run()
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.stop()

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

    def stop(self):
        """Stop the TCP video server."""
        if not self.running:
            return

        self.running = False

        # Stop pipeline
        if self.pipeline:
            logger.info("Stopping pipeline...")
            self.pipeline.set_state(Gst.State.NULL)

        # Close all client connections
        with self.client_lock:
            for client in self.client_sockets:
                try:
                    client.close()
                except:
                    pass
            self.client_sockets.clear()

        # Close server socket
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass

        # Stop main loop
        if self.main_loop and self.main_loop.is_running():
            self.main_loop.quit()

        logger.info("TCP video server stopped")


def main():
    parser = argparse.ArgumentParser(
        description="TCP server for video streaming with absolute timestamps"
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

    # TCP server options
    parser.add_argument("--port", type=int, default=5555, help="TCP server port (default: 5555)")
    parser.add_argument("--host", default="0.0.0.0", help="Server bind address (default: 0.0.0.0)")

    args = parser.parse_args()

    # Create and start server
    server = TCPVideoServer(
        device=args.device,
        width=args.width,
        height=args.height,
        framerate=args.framerate,
        format_str=args.format,
        bitrate=args.bitrate,
        port=args.port,
        host=args.host,
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
