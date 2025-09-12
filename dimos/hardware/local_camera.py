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
import time

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

logger = setup_logger("dimos.hardware.local_camera", level=logging.INFO)

Gst.init(None)


class LocalCamera(Module):
    """Module that captures frames directly from a local camera device using GStreamer."""

    video: Out[Image] = None

    def __init__(
        self,
        device: str = "/dev/video0",
        width: int = 2560,
        height: int = 720,
        framerate: int = 60,
        format_str: str = "YUY2",
        frame_id: str = "camera",
        single_camera: bool = False,
        *args,
        **kwargs,
    ):
        """Initialize the local camera module.

        Args:
            device: Video device path
            width: Video width in pixels
            height: Video height in pixels
            framerate: Frame rate in fps
            format_str: Video format
            frame_id: Frame ID for the published images
            single_camera: If True, crop to left half (for stereo cameras)
        """
        self.device = device
        self.width = width
        self.height = height
        self.framerate = framerate
        self.format = format_str
        self.frame_id = frame_id
        self.single_camera = single_camera

        self.pipeline = None
        self.appsink = None
        self.main_loop = None
        self.main_loop_thread = None
        self.running = False
        self.frame_count = 0
        self.last_log_time = time.time()
        self.start_time = None

        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self):
        if self.running:
            logger.warning("Local camera module is already running")
            return

        try:
            self._create_pipeline()
            self._start_pipeline()
            self.running = True
            self.start_time = time.time()
            logger.info(f"Local camera module started using device: {self.device}")
        except Exception as e:
            logger.error(f"Failed to start local camera: {e}")
            raise

    @rpc
    def stop(self):
        if not self.running:
            return

        self.running = False

        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)

        if self.main_loop:
            self.main_loop.quit()

        # Only join the thread if we're not calling from within it
        if self.main_loop_thread and self.main_loop_thread != threading.current_thread():
            self.main_loop_thread.join(timeout=2.0)

        if self.frame_count > 0 and self.start_time:
            elapsed = time.time() - self.start_time
            avg_fps = self.frame_count / elapsed
            logger.info(f"Total frames captured: {self.frame_count}, Average FPS: {avg_fps:.1f}")

        logger.info("Local camera module stopped")

    def _create_pipeline(self):
        """Create the GStreamer pipeline for local camera capture."""

        # Create pipeline
        self.pipeline = Gst.Pipeline.new("local-camera-pipeline")

        # Create elements
        videosrc = Gst.ElementFactory.make("v4l2src", "source")
        videosrc.set_property("device", self.device)
        videosrc.set_property("do-timestamp", True)

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

        # Create caps filter to ensure BGR output
        bgr_capsfilter = Gst.ElementFactory.make("capsfilter", "bgr_capsfilter")
        bgr_caps = Gst.Caps.from_string("video/x-raw,format=BGR")
        bgr_capsfilter.set_property("caps", bgr_caps)

        # Create appsink
        self.appsink = Gst.ElementFactory.make("appsink", "sink")
        self.appsink.set_property("emit-signals", True)
        self.appsink.set_property("sync", False)
        self.appsink.set_property("max-buffers", 1)
        self.appsink.set_property("drop", True)

        # Add elements to pipeline
        self.pipeline.add(videosrc)
        self.pipeline.add(capsfilter)
        self.pipeline.add(videoconvert)
        if videocrop:
            self.pipeline.add(videocrop)
        self.pipeline.add(bgr_capsfilter)
        self.pipeline.add(self.appsink)

        # Link elements
        if not videosrc.link(capsfilter):
            raise RuntimeError("Failed to link source to capsfilter")
        if not capsfilter.link(videoconvert):
            raise RuntimeError("Failed to link capsfilter to videoconvert")

        # Link through crop if in single camera mode
        if videocrop:
            if not videoconvert.link(videocrop):
                raise RuntimeError("Failed to link videoconvert to videocrop")
            if not videocrop.link(bgr_capsfilter):
                raise RuntimeError("Failed to link videocrop to bgr_capsfilter")
        else:
            if not videoconvert.link(bgr_capsfilter):
                raise RuntimeError("Failed to link videoconvert to bgr_capsfilter")

        if not bgr_capsfilter.link(self.appsink):
            raise RuntimeError("Failed to link bgr_capsfilter to appsink")

        # Connect new-sample signal
        self.appsink.connect("new-sample", self._on_new_sample)

        # Set up bus message handling
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        logger.info("Pipeline created:")
        logger.info(f"  Source: {self.device}")
        if self.single_camera:
            output_width = self.width // 2
            logger.info(f"  Input Resolution: {self.width}x{self.height} @ {self.framerate}fps")
            logger.info(
                f"  Output Resolution: {output_width}x{self.height} @ {self.framerate}fps (left camera only)"
            )
        else:
            logger.info(f"  Resolution: {self.width}x{self.height} @ {self.framerate}fps")

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
            logger.error(f"Pipeline error: {err}, {debug}")
            self.stop()
        elif t == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            logger.warning(f"Pipeline warning: {warn}, {debug}")
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                if new_state == Gst.State.PLAYING:
                    logger.info("Pipeline is now playing - capturing from local camera")

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

        # Get the current timestamp
        timestamp = time.time()

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

            # Create an Image message with the current timestamp
            image_msg = Image(
                data=image_array.copy(),  # Make a copy to ensure data persistence
                format=ImageFormat.BGR,
                frame_id=self.frame_id,
                ts=timestamp,
            )

            # Publish the image
            if self.video and self.running:
                self.video.publish(image_msg)

            # Log statistics periodically
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.last_log_time >= 5.0:
                fps = self.frame_count / (current_time - self.last_log_time)
                logger.debug(
                    f"Capturing frames - FPS: {fps:.1f}, Resolution: {width}x{height}, "
                    f"Timestamp: {timestamp:.6f}"
                )
                self.frame_count = 0
                self.last_log_time = current_time

        except Exception as e:
            logger.error(f"Error processing frame: {e}")

        finally:
            buffer.unmap(map_info)

        return Gst.FlowReturn.OK

    def __del__(self):
        self.stop()


def main():
    import argparse
    from dimos import core
    from dimos.protocol import pubsub

    parser = argparse.ArgumentParser(description="Local camera capture module using GStreamer")

    # Video device options
    parser.add_argument(
        "--device", default="/dev/video0", help="Video device path (default: /dev/video0)"
    )

    # Video format options
    parser.add_argument("--width", type=int, default=2560, help="Video width (default: 2560)")
    parser.add_argument("--height", type=int, default=720, help="Video height (default: 720)")
    parser.add_argument("--framerate", type=int, default=60, help="Frame rate in fps (default: 60)")
    parser.add_argument("--format", default="YUY2", help="Video format (default: YUY2)")

    # Camera options
    parser.add_argument(
        "--frame-id",
        default="camera",
        help="Frame ID for published images (default: camera)",
    )
    parser.add_argument(
        "--single-camera",
        action="store_true",
        help="Extract left camera only from stereo feed (crops to left half)",
    )

    # LCM topic
    parser.add_argument(
        "--topic",
        default="/camera/video",
        help="LCM topic to publish images to (default: /camera/video)",
    )

    # Logging options
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Initialize LCM
    pubsub.lcm.autoconf()

    # Start dimos
    logger.info("Starting dimos...")
    dimos = core.start(8)

    # Deploy the LocalCamera module
    logger.info(f"Deploying LocalCamera module (device: {args.device})...")
    camera = dimos.deploy(
        LocalCamera,
        device=args.device,
        width=args.width,
        height=args.height,
        framerate=args.framerate,
        format_str=args.format,
        frame_id=args.frame_id,
        single_camera=args.single_camera,
    )

    # Set up LCM transport for the video output
    camera.video.transport = core.LCMTransport(args.topic, Image)

    # Counter for received frames
    frame_count = [0]
    last_log_time = [time.time()]

    def on_frame(msg):
        frame_count[0] += 1
        current_time = time.time()

        # Log stats every 2 seconds
        if current_time - last_log_time[0] >= 2.0:
            fps = frame_count[0] / (current_time - last_log_time[0])
            logger.info(
                f"Publishing {frame_count[0]} frames - FPS: {fps:.1f} - "
                f"Resolution: {msg.width}x{msg.height} - "
                f"Timestamp: {msg.ts:.3f}"
            )
            frame_count[0] = 0
            last_log_time[0] = current_time

    # Subscribe to video output for monitoring
    camera.video.subscribe(on_frame)

    # Start the camera
    logger.info("Starting local camera...")
    camera.start()

    logger.info("LocalCamera module is running. Press Ctrl+C to stop.")
    logger.info(f"Capturing from device: {args.device}")
    logger.info(f"Publishing frames to LCM topic: {args.topic}")
    if args.single_camera:
        logger.info(
            f"Single camera mode: cropping {args.width}x{args.height} to {args.width // 2}x{args.height}"
        )

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        camera.stop()
        logger.info("Stopped.")


if __name__ == "__main__":
    main()
