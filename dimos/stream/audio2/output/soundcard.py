#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Soundcard output node for playing audio through system speakers."""

import threading
from typing import Optional, Union

import gi
from pydantic import Field

gi.require_version("Gst", "1.0")

from dimos.stream.audio2.base import GStreamerSinkBase
from dimos.stream.audio2.gstreamer import GStreamerNodeConfig
from dimos.stream.audio2.types import AudioSink
from dimos.stream.audio2.utils import apply_gstreamer_properties
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.output.soundcard")


class SoundcardOutputConfig(GStreamerNodeConfig):
    """Configuration for soundcard output."""

    device: Optional[str] = Field(
        default=None, description="Audio device name (None = default device)"
    )


class SoundcardOutputNode(GStreamerSinkBase):
    """Soundcard output that plays AudioEvents through system speakers."""

    def __init__(self, config: SoundcardOutputConfig):
        super().__init__(config)
        self.config: SoundcardOutputConfig = config  # Type hint for better IDE support

    def _get_pipeline_string(self) -> str:
        """Build the soundcard output pipeline."""
        # Pipeline with decoder support for both raw and compressed audio
        # decodebin auto-detects and decodes compressed formats
        # queue for buffering
        # audioconvert and audioresample for format flexibility
        # autoaudiosink automatically selects the best available sink
        parts = [
            "queue",
            "!",
            "decodebin",
            "!",
            "audioconvert",
            "!",
            "audioresample",
            "!",
            "autoaudiosink name=sink",
        ]

        return " ".join(parts)

    def _get_sink_name(self) -> str:
        """Get descriptive name for this sink."""
        device_str = f" [{self.config.device}]" if self.config.device else ""
        return f"SoundcardOutput{device_str}"

    def _configure_sink(self):
        """Configure the actual audio sink after pipeline creation."""
        # Try to get the actual sink from autoaudiosink
        sink = self._pipeline.get_by_name("sink")
        if sink:
            # autoaudiosink is a bin, get the actual sink inside it
            actual_sink = None

            # For autoaudiosink, we need to wait for it to be realized
            # Just use the autoaudiosink directly - it will forward properties
            sink_to_configure = sink

            # Set sync property - False so sink plays audio as it receives it
            try:
                # Set sync to False - the sink should play whatever it receives immediately
                # The source controls the timing by emitting events at the right rate
                sink_to_configure.set_property("sync", False)
                logger.info(f"Set sync=False on {sink_to_configure.get_name()} (plays as received)")
            except Exception as e:
                logger.warning(f"Failed to set sync property: {e}")

            # Set buffer-time and latency-time if supported
            if self.config.buffer_time is not None:
                try:
                    sink_to_configure.set_property("buffer-time", self.config.buffer_time)
                    logger.debug(f"Set buffer-time={self.config.buffer_time}")
                except Exception:
                    pass  # Not all sinks support this

            if self.config.latency_time is not None:
                try:
                    sink_to_configure.set_property("latency-time", self.config.latency_time)
                    logger.debug(f"Set latency-time={self.config.latency_time}")
                except Exception:
                    pass  # Not all sinks support this

            # Set any custom properties
            if self.config.properties:
                apply_gstreamer_properties(
                    sink_to_configure,
                    self.config.properties,
                    skip_properties=set(),
                    element_name="sink",
                )


class BlockingSpeaker:
    """A speaker sink that blocks until playback completes.

    This wrapper provides a cleaner API for simple playback scenarios
    where you want the subscription to block until audio finishes playing.
    """

    def __init__(self, sink_node):
        self._sink = sink_node
        self._completion_event = threading.Event()
        self._started = False

    def on_next(self, value):
        self._started = True
        self._sink.on_next(value)

    def on_error(self, error):
        self._sink.on_error(error)
        self._completion_event.set()

    def on_completed(self):
        # Call sink's on_completed
        self._sink.on_completed()
        # Signal completion
        self._completion_event.set()

    def wait_for_completion(self):
        """Block until playback completes."""
        # Wait for the completion event
        self._completion_event.wait(timeout=30.0)  # Max 30 seconds

        # Wait for the sink to finish processing
        import time

        max_wait = 10.0
        start = time.time()
        while self._sink._is_playing and (time.time() - start) < max_wait:
            time.sleep(0.1)

        # Wait for FileInput threads and cleanup threads to finish
        max_cleanup_wait = 5.0
        cleanup_start = time.time()
        while (time.time() - cleanup_start) < max_cleanup_wait:
            file_input_threads = [
                t for t in threading.enumerate() if "FileInput" in t.name and t.is_alive()
            ]
            if not file_input_threads:
                break
            time.sleep(0.1)

        # Give a bit more time for GStreamer cleanup
        time.sleep(0.5)

    def __del__(self):
        """When the speaker goes out of scope, wait for completion and cleanup."""
        # Block until playback completes
        if hasattr(self, "_started") and self._started:
            self.wait_for_completion()

        # Stop the sink if it's still running
        if hasattr(self, "_sink"):
            self._sink.stop()


def speaker(
    device: Optional[str] = None,
    buffer_time: Optional[int] = None,
    latency_time: Optional[int] = None,
    blocking: bool = True,
    **kwargs,
):
    """Create a soundcard output operator for playing audio.

    Can be used either as a sink (Observer) for subscribe() or as an
    operator for pipe().

    Args:
        device: Audio device name (None = default device)
        buffer_time: Buffer time in microseconds (None = auto)
        latency_time: Latency time in microseconds (None = auto)
        blocking: If True, blocks until playback completes (default: True)
        **kwargs: Additional arguments passed to SoundcardOutputConfig:
            - output: Output audio specification (usually not needed)
            - properties: Additional GStreamer element properties

    Returns:
        Either an operator function for use with pipe() or an AudioSink
        for use with subscribe(), depending on context.

    Examples:
        # Use as operator with pipe() and run()
        file_input("audio.mp3").pipe(speaker()).run()

        # Use as sink with subscribe()
        file_input("audio.mp3").subscribe(speaker())

        # Low latency output
        file_input("audio.mp3").pipe(speaker(buffer_time=10000)).run()

        # Specific device
        file_input("audio.mp3").pipe(speaker(device="hw:1,0")).run()
    """
    from reactivex import operators as ops

    config = SoundcardOutputConfig(
        device=device, buffer_time=buffer_time, latency_time=latency_time, **kwargs
    )

    node = SoundcardOutputNode(config)

    # Create a sink (wrapped if blocking)
    sink: Union[BlockingSpeaker, SoundcardOutputNode]
    if blocking:
        sink = BlockingSpeaker(node)
    else:
        sink = node

    # Return a dual-purpose object that can be used as operator or sink
    class SpeakerOperator:
        """Can be used as both an operator and a sink."""

        def __init__(self, sink_node):
            self._sink = sink_node

        def __call__(self, source):
            """Act as an operator for pipe() that properly waits for completion."""
            from reactivex import create
            import threading

            def subscribe(observer, scheduler=None):
                # Event to track when sink actually finishes
                sink_completed = threading.Event()

                # Wrap sink callbacks to track completion
                def on_next_wrapper(value):
                    self._sink.on_next(value)
                    # Pass through to observer
                    observer.on_next(value)

                def on_error_wrapper(error):
                    self._sink.on_error(error)
                    sink_completed.set()
                    observer.on_error(error)

                def on_completed_wrapper():
                    from dimos.utils.logging_config import setup_logger

                    logger = setup_logger("dimos.stream.audio2.output.soundcard")
                    logger.info("SpeakerOperator: on_completed received from source")

                    self._sink.on_completed()

                    # Wait for sink to actually finish processing
                    # This is where the sink plays remaining audio and cleans up
                    import time

                    max_wait = 15.0
                    start = time.time()
                    # Get the actual sink node (unwrap BlockingSpeaker if present)
                    actual_sink = self._sink._sink if hasattr(self._sink, "_sink") else self._sink
                    logger.info(
                        f"SpeakerOperator: Waiting for sink to finish (is_playing={getattr(actual_sink, '_is_playing', 'N/A')})"
                    )
                    while hasattr(actual_sink, "_is_playing") and actual_sink._is_playing:
                        if time.time() - start > max_wait:
                            logger.warning("SpeakerOperator: Timeout waiting for sink to finish")
                            break
                        time.sleep(0.1)
                    logger.info(
                        f"SpeakerOperator: Sink finished playing after {time.time() - start:.2f}s"
                    )

                    # Wait for cleanup threads to finish
                    max_cleanup_wait = 1.0
                    cleanup_start = time.time()
                    logger.info("SpeakerOperator: Waiting for cleanup threads")
                    while (time.time() - cleanup_start) < max_cleanup_wait:
                        # Check for any audio-related threads still running
                        # Exclude -notify threads (they're waiting for US to complete)
                        audio_threads = [
                            t
                            for t in threading.enumerate()
                            if any(
                                name in t.name
                                for name in ["FileInput", "TestSignal", "GStreamerMainLoop"]
                            )
                            and "-notify" not in t.name  # Don't wait for notification threads
                            and t.is_alive()
                        ]
                        if not audio_threads:
                            logger.info("SpeakerOperator: All audio threads cleaned up")
                            break
                        if time.time() - cleanup_start > 0.1:  # Log after 0.1s
                            logger.info(
                                f"SpeakerOperator: Still waiting for threads: {[t.name for t in audio_threads]}"
                            )
                        time.sleep(0.1)
                    logger.info(
                        f"SpeakerOperator: Cleanup wait finished after {time.time() - cleanup_start:.2f}s"
                    )

                    sink_completed.set()
                    logger.info("SpeakerOperator: Calling downstream observer.on_completed()")
                    observer.on_completed()

                # Subscribe to source with wrapped callbacks
                return source.subscribe(
                    on_next_wrapper, on_error_wrapper, on_completed_wrapper, scheduler=scheduler
                )

            return create(subscribe)

        # Forward Observer methods for subscribe() compatibility
        def on_next(self, value):
            return self._sink.on_next(value)

        def on_error(self, error):
            return self._sink.on_error(error)

        def on_completed(self):
            return self._sink.on_completed()

    return SpeakerOperator(sink)
