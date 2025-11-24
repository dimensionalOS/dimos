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

from dimos.stream.rtsp_video_provider import RtspVideoProvider
from dimos.web.robot_web_interface import RobotWebInterface
import tests.test_header

import logging
import time

import numpy as np
import reactivex as rx
from reactivex import operators as ops

from dimos.stream.frame_processor import FrameProcessor
from dimos.stream.video_operators import VideoOperators as vops
from dimos.stream.video_provider import get_scheduler
from dimos.utils.logging_config import setup_logger


logger = setup_logger("tests.test_rtsp_video_provider")

import sys
import os

# Load environment variables from .env file
from dotenv import load_dotenv
load_dotenv()

# RTSP URL must be provided as a command-line argument or environment variable
RTSP_URL = os.environ.get("TEST_RTSP_URL", "")
if len(sys.argv) > 1:
    RTSP_URL = sys.argv[1] # Allow overriding with command-line argument
elif RTSP_URL == "":
        print("Please provide an RTSP URL for testing.")
        print("You can set the TEST_RTSP_URL environment variable or pass it as a command-line argument.")
        print("Example: python -m dimos.stream.rtsp_video_provider rtsp://...")
        sys.exit(1)

logger.info(f"Attempting to connect to provided RTSP URL.")
provider = RtspVideoProvider(dev_name="TestRtspCam", rtsp_url=RTSP_URL)

logger.info("Creating observable...")
video_stream_observable = provider.capture_video_as_observable()

logger.info("Subscribing to observable...")
frame_counter = 0
start_time = time.monotonic() # Re-initialize start_time
last_log_time = start_time # Keep this for interval timing

# Create a subject for ffmpeg responses
ffmpeg_response_subject = rx.subject.Subject()
ffmpeg_response_stream = ffmpeg_response_subject.pipe(ops.observe_on(get_scheduler()), ops.share())

def process_frame(frame: np.ndarray):
    """Callback function executed for each received frame."""
    global frame_counter, last_log_time, start_time # Add start_time to global
    frame_counter += 1
    current_time = time.monotonic()
    # Log stats periodically (e.g., every 5 seconds)
    if current_time - last_log_time >= 5.0:
        total_elapsed_time = current_time - start_time # Calculate total elapsed time
        avg_fps = frame_counter / total_elapsed_time if total_elapsed_time > 0 else 0
        logger.info(f"Received frame {frame_counter}. Shape: {frame.shape}. Avg FPS: {avg_fps:.2f}")
        ffmpeg_response_subject.on_next(f"Received frame {frame_counter}. Shape: {frame.shape}. Avg FPS: {avg_fps:.2f}")
        last_log_time = current_time # Update log time for the next interval

def handle_error(error: Exception):
    """Callback function executed if the observable stream errors."""
    logger.error(f"Stream error: {error}", exc_info=True) # Log with traceback

def handle_completion():
    """Callback function executed when the observable stream completes."""
    logger.info("Stream completed.")

# Subscribe to the observable stream
processor = FrameProcessor()
subscription = video_stream_observable.pipe(
    # ops.subscribe_on(get_scheduler()),
    ops.observe_on(get_scheduler()),
    ops.share(),
    vops.with_jpeg_export(processor, suffix="reolink_", save_limit=30, loop=True),
).subscribe(
    on_next=process_frame,
    on_error=handle_error,
    on_completed=handle_completion
)

streams = {
    "reolink_video": video_stream_observable
}
text_streams = {
    "ffmpeg_responses": ffmpeg_response_stream,
}

web_interface = RobotWebInterface(port=5555, text_streams=text_streams, **streams)

web_interface.run() # This may block the main thread

# TODO: Redo disposal / keep-alive loop

# Keep the main thread alive to receive frames (e.g., for 60 seconds)
print("Stream running. Press Ctrl+C to stop...")
try:
    # Keep running indefinitely until interrupted
    while True:
        time.sleep(1)
        # Optional: Check if subscription is still active
        # if not subscription.is_disposed:
        #     # logger.debug("Subscription active...")
        #     pass
        # else:
        #     logger.warning("Subscription was disposed externally.")
        #     break

except KeyboardInterrupt:
    print("KeyboardInterrupt received. Shutting down...")
finally:
    # Ensure resources are cleaned up regardless of how the loop exits
    print("Disposing subscription...")
    # subscription.dispose()
    print("Disposing provider resources...")
    provider.dispose_all()
    print("Cleanup finished.")

# Final check (optional, for debugging)
time.sleep(1) # Give background threads a moment
final_process = provider._ffmpeg_process
if final_process and final_process.poll() is None:
        print(f"WARNING: ffmpeg process (PID: {final_process.pid}) may still be running after cleanup!")
else:
        print("ffmpeg process appears terminated.") 