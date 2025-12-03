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

"""RTSP video provider using ffmpeg for robust stream handling."""

import subprocess
import threading
import time

import ffmpeg  # type: ignore[import-untyped]  # ffmpeg-python wrapper
import numpy as np
import reactivex as rx
from reactivex import operators as ops
from reactivex.disposable import Disposable
from reactivex.observable import Observable
from reactivex.scheduler import ThreadPoolScheduler

from dimos.utils.logging_config import setup_logger

# Assuming AbstractVideoProvider and exceptions are in the sibling file
from .video_provider import AbstractVideoProvider, VideoFrameError, VideoSourceError

logger = setup_logger()


class RtspVideoProvider(AbstractVideoProvider):
    """Video provider implementation for capturing RTSP streams using ffmpeg.

    This provider uses the ffmpeg-python library to interact with ffmpeg,
    providing more robust handling of various RTSP streams compared to OpenCV's
    built-in VideoCapture for RTSP.
    """

    def __init__(
        self, dev_name: str, rtsp_url: str, pool_scheduler: ThreadPoolScheduler | None = None
    ) -> None:
        """Initializes the RTSP video provider.

        Args:
            dev_name: The name of the device or stream (for identification).
            rtsp_url: The URL of the RTSP stream (e.g., "rtsp://user:pass@ip:port/path").
            pool_scheduler: The scheduler for thread pool operations. Defaults to global scheduler.
        """
        super().__init__(dev_name, pool_scheduler)
        self.rtsp_url = rtsp_url
        # Holds the currently active ffmpeg process Popen object
        self._ffmpeg_process: subprocess.Popen | None = None  # type: ignore[type-arg]
        # Lock to protect access to the ffmpeg process object
        self._lock = threading.Lock()

    def _get_stream_info(self) -> dict:  # type: ignore[type-arg]
        """Probes the RTSP stream to get video dimensions and FPS using ffprobe."""
        logger.info(f"({self.dev_name}) Probing RTSP stream.")
        try:
            # Probe the stream without the problematic timeout argument
            probe = ffmpeg.probe(self.rtsp_url)
        except ffmpeg.Error as e:
            stderr = e.stderr.decode("utf8", errors="ignore") if e.stderr else "No stderr"
            msg = f"({self.dev_name}) Failed to probe RTSP stream {self.rtsp_url}: {stderr}"
            logger.error(msg)
            raise VideoSourceError(msg) from e
        except Exception as e:
            msg = f"({self.dev_name}) Unexpected error during probing {self.rtsp_url}: {e}"
            logger.error(msg)
            raise VideoSourceError(msg) from e

        video_stream = next(
            (stream for stream in probe.get("streams", []) if stream.get("codec_type") == "video"),
            None,
        )

        if video_stream is None:
            msg = f"({self.dev_name}) No video stream found in {self.rtsp_url}"
            logger.error(msg)
            raise VideoSourceError(msg)

        width = video_stream.get("width")
        height = video_stream.get("height")
        fps_str = video_stream.get("avg_frame_rate", "0/1")

        if not width or not height:
            msg = f"({self.dev_name}) Could not determine resolution for {self.rtsp_url}. Stream info: {video_stream}"
            logger.error(msg)
            raise VideoSourceError(msg)

        try:
            if "/" in fps_str:
                num, den = map(int, fps_str.split("/"))
                fps = float(num) / den if den != 0 else 30.0
            else:
                fps = float(fps_str)
            if fps <= 0:
                logger.warning(
                    f"({self.dev_name}) Invalid avg_frame_rate '{fps_str}', defaulting FPS to 30."
                )
                fps = 30.0
        except ValueError:
            logger.warning(
                f"({self.dev_name}) Could not parse FPS '{fps_str}', defaulting FPS to 30."
            )
            fps = 30.0

        logger.info(f"({self.dev_name}) Stream info: {width}x{height} @ {fps:.2f} FPS")
        return {"width": width, "height": height, "fps": fps}

    def _start_ffmpeg_process(self, width: int, height: int) -> subprocess.Popen:  # type: ignore[type-arg]
        """Starts the ffmpeg process to capture and decode the stream."""
        logger.info(f"({self.dev_name}) Starting ffmpeg process for rtsp stream.")
        try:
            # Configure ffmpeg input: prefer TCP, set timeout, reduce buffering/delay
            input_options = {
                "rtsp_transport": "tcp",
                "stimeout": "5000000",  # 5 seconds timeout for RTSP server responses
                "fflags": "nobuffer",  # Reduce input buffering
                "flags": "low_delay",  # Reduce decoding delay
                # 'timeout': '10000000' # Removed: This was misinterpreted as listen timeout
            }
            process = (
                ffmpeg.input(self.rtsp_url, **input_options)
                .output("pipe:", format="rawvideo", pix_fmt="bgr24")  # Output raw BGR frames
                .global_args("-loglevel", "warning")  # Reduce ffmpeg log spam, use 'error' for less
                .run_async(pipe_stdout=True, pipe_stderr=True)  # Capture stdout and stderr
            )
            logger.info(f"({self.dev_name}) ffmpeg process started (PID: {process.pid})")
            return process  # type: ignore[no-any-return]
        except ffmpeg.Error as e:
            stderr = e.stderr.decode("utf8", errors="ignore") if e.stderr else "No stderr"
            msg = f"({self.dev_name}) Failed to start ffmpeg for {self.rtsp_url}: {stderr}"
            logger.error(msg)
            raise VideoSourceError(msg) from e
        except Exception as e:  # Catch other errors like ffmpeg executable not found
            msg = f"({self.dev_name}) An unexpected error occurred starting ffmpeg: {e}"
            logger.error(msg)
            raise VideoSourceError(msg) from e

    def capture_video_as_observable(self, fps: int = 0) -> Observable:  # type: ignore[type-arg]
        """Creates an observable from the RTSP stream using ffmpeg.

        The observable attempts to reconnect if the stream drops.

        Args:
            fps: This argument is currently ignored. The provider attempts
                 to use the stream's native frame rate. Future versions might
                 allow specifying an output FPS via ffmpeg filters.

        Returns:
            Observable: An observable emitting video frames as NumPy arrays (BGR format).

        Raises:
            VideoSourceError: If the stream cannot be initially probed or the
                              ffmpeg process fails to start.
            VideoFrameError: If there's an error reading or processing frames.
        """
        if fps != 0:
            logger.warning(
                f"({self.dev_name}) The 'fps' argument ({fps}) is currently ignored. Using stream native FPS."
            )

        def emit_frames(observer, scheduler):  # type: ignore[no-untyped-def]
            """Function executed by rx.create to emit frames."""
            process: subprocess.Popen | None = None  # type: ignore[type-arg]
            # Event to signal the processing loop should stop (e.g., on dispose)
            should_stop = threading.Event()

            def cleanup_process() -> None:
                """Safely terminate the ffmpeg process if it's running."""
                nonlocal process
                logger.debug(f"({self.dev_name}) Cleanup requested.")
                # Use lock to ensure thread safety when accessing/modifying process
                with self._lock:
                    # Check if the process exists and is still running
                    if process and process.poll() is None:
                        logger.info(
                            f"({self.dev_name}) Terminating ffmpeg process (PID: {process.pid})."
                        )
                        try:
                            process.terminate()  # Ask ffmpeg to exit gracefully
                            process.wait(timeout=1.0)  # Wait up to 1 second
                        except subprocess.TimeoutExpired:
                            logger.warning(
                                f"({self.dev_name}) ffmpeg (PID: {process.pid}) did not terminate gracefully, killing."
                            )
                            process.kill()  # Force kill if it didn't exit
                        except Exception as e:
                            logger.error(f"({self.dev_name}) Error during ffmpeg termination: {e}")
                        finally:
                            # Ensure we clear the process variable even if wait/kill fails
                            process = None
                            # Also clear the shared class attribute if this was the active process
                            if self._ffmpeg_process and self._ffmpeg_process.pid == process.pid:  # type: ignore[attr-defined]
                                self._ffmpeg_process = None
                    elif process and process.poll() is not None:
                        # Process exists but already terminated
                        logger.debug(
                            f"({self.dev_name}) ffmpeg process (PID: {process.pid}) already terminated (exit code: {process.poll()})."
                        )
                        process = None  # Clear the variable
                        # Clear shared attribute if it matches
                        if self._ffmpeg_process and self._ffmpeg_process.pid == process.pid:  # type: ignore[attr-defined]
                            self._ffmpeg_process = None
                    else:
                        # Process variable is already None or doesn't match _ffmpeg_process
                        logger.debug(
                            f"({self.dev_name}) No active ffmpeg process found needing termination in cleanup."
                        )

            try:
                # 1. Probe the stream to get essential info (width, height)
                stream_info = self._get_stream_info()
                width = stream_info["width"]
                height = stream_info["height"]
                # Calculate expected bytes per frame (BGR format = 3 bytes per pixel)
                frame_size = width * height * 3

                # 2. Main loop: Start ffmpeg and read frames. Retry on failure.
                while not should_stop.is_set():
                    try:
                        # Start or reuse the ffmpeg process
                        with self._lock:
                            # Check if another thread/subscription already started the process
                            if self._ffmpeg_process and self._ffmpeg_process.poll() is None:
                                logger.warning(
                                    f"({self.dev_name}) ffmpeg process (PID: {self._ffmpeg_process.pid}) seems to be already running. Reusing."
                                )
                                process = self._ffmpeg_process
                            else:
                                # Start a new ffmpeg process
                                process = self._start_ffmpeg_process(width, height)
                                # Store the new process handle in the shared class attribute
                                self._ffmpeg_process = process

                        # 3. Frame reading loop
                        while not should_stop.is_set():
                            # Read exactly one frame's worth of bytes
                            in_bytes = process.stdout.read(frame_size)  # type: ignore[union-attr]

                            if len(in_bytes) == 0:
                                # End of stream or process terminated unexpectedly
                                logger.warning(
                                    f"({self.dev_name}) ffmpeg stdout returned 0 bytes. EOF or process terminated."
                                )
                                process.wait(timeout=0.5)  # Allow stderr to flush
                                stderr_data = process.stderr.read().decode("utf8", errors="ignore")  # type: ignore[union-attr]
                                exit_code = process.poll()
                                logger.warning(
                                    f"({self.dev_name}) ffmpeg process (PID: {process.pid}) exited with code {exit_code}. Stderr: {stderr_data}"
                                )
                                # Break inner loop to trigger cleanup and potential restart
                                with self._lock:
                                    # Clear the shared process handle if it matches the one that just exited
                                    if (
                                        self._ffmpeg_process
                                        and self._ffmpeg_process.pid == process.pid
                                    ):
                                        self._ffmpeg_process = None
                                process = None  # Clear local process variable
                                break  # Exit frame reading loop

                            elif len(in_bytes) != frame_size:
                                # Received incomplete frame data - indicates a problem
                                msg = f"({self.dev_name}) Incomplete frame read. Expected {frame_size}, got {len(in_bytes)}. Stopping."
                                logger.error(msg)
                                observer.on_error(VideoFrameError(msg))
                                should_stop.set()  # Signal outer loop to stop
                                break  # Exit frame reading loop

                            # Convert the raw bytes to a NumPy array (height, width, channels)
                            frame = np.frombuffer(in_bytes, np.uint8).reshape((height, width, 3))
                            # Emit the frame to subscribers
                            observer.on_next(frame)

                        # 4. Handle ffmpeg process exit/crash (if not stopping deliberately)
                        if not should_stop.is_set() and process is None:
                            logger.info(
                                f"({self.dev_name}) ffmpeg process ended. Attempting reconnection in 5 seconds..."
                            )
                            # Wait for a few seconds before trying to restart
                            time.sleep(5)
                            # Continue to the next iteration of the outer loop to restart

                    except (VideoSourceError, ffmpeg.Error) as e:
                        # Errors during ffmpeg process start or severe runtime errors
                        logger.error(
                            f"({self.dev_name}) Unrecoverable ffmpeg error: {e}. Stopping emission."
                        )
                        observer.on_error(e)
                        should_stop.set()  # Stop retrying
                    except Exception as e:
                        # Catch other unexpected errors during frame reading/processing
                        logger.error(
                            f"({self.dev_name}) Unexpected error processing stream: {e}",
                            exc_info=True,
                        )
                        observer.on_error(VideoFrameError(f"Frame processing failed: {e}"))
                        should_stop.set()  # Stop retrying

                # 5. Loop finished (likely due to should_stop being set)
                logger.info(f"({self.dev_name}) Frame emission loop stopped.")
                observer.on_completed()

            except VideoSourceError as e:
                # Handle errors during the initial probing phase
                logger.error(f"({self.dev_name}) Failed initial setup: {e}")
                observer.on_error(e)
            except Exception as e:
                # Catch-all for unexpected errors before the main loop starts
                logger.error(f"({self.dev_name}) Unexpected setup error: {e}", exc_info=True)
                observer.on_error(VideoSourceError(f"Setup failed: {e}"))
            finally:
                # Crucial: Ensure the ffmpeg process is terminated when the observable
                # is completed, errored, or disposed.
                logger.debug(f"({self.dev_name}) Entering finally block in emit_frames.")
                cleanup_process()

            # Return a Disposable that, when called (by unsubscribe/dispose),
            # signals the loop to stop. The finally block handles the actual cleanup.
            return Disposable(should_stop.set)

        # Create the observable using rx.create, applying scheduling and sharing
        return rx.create(emit_frames).pipe(
            ops.subscribe_on(self.pool_scheduler),  # Run the emit_frames logic on the pool
            # ops.observe_on(self.pool_scheduler), # Optional: Switch thread for downstream operators
            ops.share(),  # Ensure multiple subscribers share the same ffmpeg process
        )

    def dispose_all(self) -> None:
        """Disposes of all managed resources, including terminating the ffmpeg process."""
        logger.info(f"({self.dev_name}) dispose_all called.")
        # Terminate the ffmpeg process using the same locked logic as cleanup
        with self._lock:
            process = self._ffmpeg_process  # Get the current process handle
            if process and process.poll() is None:
                logger.info(
                    f"({self.dev_name}) Terminating ffmpeg process (PID: {process.pid}) via dispose_all."
                )
                try:
                    process.terminate()
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    logger.warning(
                        f"({self.dev_name}) ffmpeg process (PID: {process.pid}) kill required in dispose_all."
                    )
                    process.kill()
                except Exception as e:
                    logger.error(
                        f"({self.dev_name}) Error during ffmpeg termination in dispose_all: {e}"
                    )
                finally:
                    self._ffmpeg_process = None  # Clear handle after attempting termination
            elif process:  # Process exists but already terminated
                logger.debug(
                    f"({self.dev_name}) ffmpeg process (PID: {process.pid}) already terminated in dispose_all."
                )
                self._ffmpeg_process = None
            else:
                logger.debug(
                    f"({self.dev_name}) No active ffmpeg process found during dispose_all."
                )

        # Call the parent class's dispose_all to handle Rx Disposables
        super().dispose_all()

    def __del__(self) -> None:
        """Destructor attempts to clean up resources if not explicitly disposed."""
        # Logging in __del__ is generally discouraged due to interpreter state issues,
        # but can be helpful for debugging resource leaks. Use print for robustness here if needed.
        # print(f"DEBUG: ({self.dev_name}) __del__ called.")
        self.dispose_all()
