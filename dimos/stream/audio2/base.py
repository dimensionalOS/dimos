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

"""Base classes for GStreamer-based audio sources and sinks."""

import threading
import time
from abc import ABC, abstractmethod
from typing import Optional, Union

import gi
import numpy as np
from reactivex import create, disposable
from reactivex.abc import ObserverBase, DisposableBase
from reactivex.observable import Observable

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GstApp

from dimos.stream.audio2.gstreamer import GStreamerNodeConfig, GStreamerPipelineBase
from dimos.stream.audio2.types import (
    AudioEvent,
    AudioSpec,
    RawAudioEvent,
    CompressedAudioEvent,
    AudioSink,
)
from dimos.stream.audio2.utils import (
    buffer_to_audio_event,
    parse_caps_to_spec,
    validate_pipeline_element,
    get_numpy_dtype_for_format,
    apply_gstreamer_properties,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.base")


class GStreamerSourceBase(GStreamerPipelineBase, ABC):
    """Base class for GStreamer sources that use appsink.

    This handles common functionality for all pull-based audio sources:
    - Appsink configuration and management
    - Pull thread lifecycle
    - Buffer to AudioEvent conversion
    - Observable creation

    Subclasses need to implement:
    - _get_pipeline_string(): Return the GStreamer pipeline before appsink
    """

    def __init__(self, config: GStreamerNodeConfig):
        super().__init__()
        self.config = config

        # Pipeline elements
        self._appsink: Optional[GstApp.AppSink] = None

        # Pull thread management
        self._observer: Optional[ObserverBase] = None
        self._pull_thread: Optional[threading.Thread] = None
        self._pull_thread_name: Optional[str] = None

        # Detected audio parameters
        self._detected_rate: Optional[int] = None
        self._detected_channels: Optional[int] = None
        self._detected_format: Optional[str] = None

    @abstractmethod
    def _get_pipeline_string(self) -> str:
        """Get the GStreamer pipeline string before appsink.

        This should return everything before the appsink element.
        The base class will add appropriate conversion and the appsink.

        Returns:
            Pipeline string (e.g., "filesrc location=foo.mp3 ! decodebin")
        """
        pass

    def _get_source_name(self) -> str:
        """Get a descriptive name for this source. Override for better names."""
        return self.__class__.__name__

    def _create_pipeline(self):
        """Create the complete GStreamer pipeline with appsink."""
        # Get source-specific pipeline
        source_pipeline = self._get_pipeline_string()

        # Add conversion and format specification
        if self.config.output.format.is_raw:
            output_section = self.config.output.to_gst_caps_string()
        else:
            output_section = self.config.get_encoder_string()

        # Build complete pipeline
        pipeline_str = (
            f"{source_pipeline} ! "
            f"audioconvert ! "
            f"audioresample ! "
            f"{output_section} ! "
            f"appsink name=sink"
        )

        logger.debug(f"{self._get_source_name()}: Creating pipeline: {pipeline_str}")
        self._pipeline = Gst.parse_launch(pipeline_str)

        # Get and configure appsink
        self._appsink = validate_pipeline_element(self._pipeline, "sink")
        self._configure_appsink()

        # Set up bus for error/EOS handling
        self._setup_bus(self._pipeline)

    def _configure_appsink(self, sync: Optional[bool] = None):
        """Configure appsink properties.

        Args:
            sync: Whether to sync to clock. If None, uses False (default).
                  Subclasses can override by passing realtime config value.
        """
        if not self._appsink:
            return

        self._appsink.set_property("emit-signals", True)

        # Use sync parameter if provided, otherwise default to False
        sync_value = sync if sync is not None else False
        self._appsink.set_property("sync", sync_value)
        logger.info(f"{self._get_source_name()}: Set appsink sync={sync_value}")

        # Apply any custom properties
        if hasattr(self.config, "properties") and self.config.properties:
            apply_gstreamer_properties(
                self._appsink,
                self.config.properties,
                skip_properties={"emit-signals", "sync"},
                element_name="appsink",
            )

    def _handle_eos(self):
        """Handle end of stream."""
        import threading

        logger.info(f"{self._get_source_name()}: End of stream")
        self._running = False

        # Check if cleanup thread already exists (avoid duplicate cleanup threads)
        if (
            hasattr(self, "_cleanup_thread")
            and self._cleanup_thread
            and self._cleanup_thread.is_alive()
        ):
            logger.debug(f"{self._get_source_name()}: Cleanup thread already running, skipping")
            return

        # Call observer.on_completed() in a separate thread to avoid blocking
        # (downstream operators may block waiting for cleanup)
        notify_thread = None
        if self._observer:

            def notify_completion():
                logger.info(f"{self._get_source_name()}: Calling observer.on_completed()")
                self._observer.on_completed()

            notify_thread = threading.Thread(
                target=notify_completion, daemon=True, name=f"{self._get_source_name()}-notify"
            )
            notify_thread.start()

        # Schedule cleanup in a separate thread, but wait for notify to complete first
        def cleanup_after_notify():
            # Wait for notify thread to complete so downstream operators can finish
            # Notify typically completes in <100ms, but use generous timeout for safety
            if notify_thread:
                notify_thread.join(timeout=0.5)
                if notify_thread.is_alive():
                    logger.warning(
                        f"{self._get_source_name()}: Notify thread still alive after 0.5s, "
                        f"proceeding with cleanup anyway"
                    )
            self._cleanup_on_completion()

        try:
            self._cleanup_thread = threading.Thread(
                target=cleanup_after_notify,
                daemon=False,  # Non-daemon so it always completes
                name=f"{self._get_source_name()}-cleanup",
            )
            self._cleanup_thread.start()
            logger.info(f"{self._get_source_name()}: Cleanup thread started")
        except Exception as e:
            logger.error(f"{self._get_source_name()}: Failed to start cleanup thread: {e}")

    def _handle_error(self, error):
        """Handle pipeline errors."""
        logger.error(f"{self._get_source_name()}: Pipeline error: {error}")
        self._running = False
        if self._observer:
            self._observer.on_error(Exception(str(error)))

    def _pull_samples(self):
        """Pull samples from appsink and emit as AudioEvents."""
        logger.debug(f"Pull thread '{self._pull_thread_name}' starting")

        buffer_count = 0
        try:
            while self._running:
                # Pull sample from appsink
                sample = self._appsink.emit("pull-sample")
                if sample is None:
                    if not self._running:
                        break
                    time.sleep(0.001)
                    continue

                # Get buffer
                buffer = sample.get_buffer()
                if buffer is None:
                    continue

                # Parse caps on first sample to get actual format info
                if self._detected_rate is None:
                    caps = sample.get_caps()
                    if caps:
                        rate, channels, format_str = parse_caps_to_spec(caps)
                        if rate:
                            self._detected_rate = rate
                            self._detected_channels = channels
                            self._detected_format = format_str
                            logger.info(
                                f"{self._get_source_name()}: Detected format: "
                                f"{rate}Hz, {channels}ch, {format_str}"
                            )

                # Convert buffer to AudioEvent
                try:
                    # Extract timestamp from buffer PTS (in nanoseconds, convert to seconds)
                    buffer_timestamp = None
                    if buffer.pts != Gst.CLOCK_TIME_NONE:
                        buffer_timestamp = buffer.pts / Gst.SECOND

                    event = buffer_to_audio_event(
                        buffer=buffer,
                        spec=self.config.output,
                        detected_rate=self._detected_rate,
                        detected_channels=self._detected_channels,
                        timestamp=buffer_timestamp,
                    )

                    buffer_count += 1
                    if buffer_count == 1:
                        logger.info(f"{self._get_source_name()}: First buffer pushed")
                    elif buffer_count % 10 == 0:
                        logger.debug(f"{self._get_source_name()}: {buffer_count} buffers pushed")

                    if self._observer:
                        self._observer.on_next(event)

                except Exception as e:
                    logger.error(f"Failed to process buffer: {e}")
                    if self._observer:
                        self._observer.on_error(e)

        except Exception as e:
            logger.error(f"Pull thread error: {e}")
            if self._observer:
                self._observer.on_error(e)

        finally:
            logger.info(f"{self._get_source_name()}: Total buffers pushed: {buffer_count}")
            logger.info(f"Pull thread '{self._pull_thread_name}' exiting")
            # Don't cleanup pipeline here - it's done in _cleanup_on_completion

    def _cleanup_on_completion(self):
        """Clean up resources when stream completes naturally."""
        logger.info(f"{self._get_source_name()}: Cleaning up after completion")

        # Mark as not running first
        self._running = False

        # Stop pipeline (check again inside to handle race conditions)
        if self._pipeline:
            logger.debug(f"{self._get_source_name()}: Setting pipeline to NULL")
            try:
                self._pipeline.set_state(Gst.State.NULL)
            except (AttributeError, RuntimeError):
                # Pipeline may have been cleaned up by another thread
                pass

        # Wait for pull thread to finish
        if self._pull_thread and self._pull_thread.is_alive():
            logger.debug(f"{self._get_source_name()}: Waiting for pull thread to finish")
            self._pull_thread.join(timeout=2.0)
            if self._pull_thread.is_alive():
                logger.warning(f"Pull thread '{self._pull_thread_name}' did not stop in time")

        # Clean up pipeline resources (this also releases MainLoop reference)
        # Note: This might be called redundantly if disposal also happens, but _cleanup_pipeline() is idempotent
        self._cleanup_pipeline()

        logger.info(f"{self._get_source_name()}: Cleanup complete")

    def _generate_thread_name(self) -> str:
        """Generate a unique thread name for debugging."""
        import random

        # Use random number to avoid collisions
        unique_id = random.randint(1000, 9999)
        return f"{self._get_source_name()}-{unique_id}"

    def create_observable(self) -> Observable[AudioEvent]:
        """Create an observable that emits AudioEvents from this source."""

        def on_subscribe(observer, scheduler):
            self._observer = observer

            try:
                # Ensure pipeline is ready
                self._ensure_pipeline_ready()
                self._create_pipeline()

                # Start pipeline
                ret = self._pipeline.set_state(Gst.State.PLAYING)
                if ret == Gst.StateChangeReturn.FAILURE:
                    raise RuntimeError("Failed to start pipeline")

                # Start pull thread
                self._pull_thread_name = self._generate_thread_name()
                self._pull_thread = threading.Thread(
                    target=self._pull_samples,
                    daemon=True,
                    name=self._pull_thread_name,
                )
                self._pull_thread.start()
                logger.info(f"{self._get_source_name()}: Started")

            except Exception as e:
                observer.on_error(e)
                self._cleanup_pipeline()
                return disposable.Disposable.empty()

            # Return disposable for cleanup
            def dispose():
                logger.info(f"{self._get_source_name()}: Stopping")
                self._running = False

                # Stop pipeline
                if self._pipeline:
                    self._pipeline.set_state(Gst.State.NULL)

                # Wait for pull thread
                if self._pull_thread and self._pull_thread.is_alive():
                    self._pull_thread.join(timeout=2.0)
                    if self._pull_thread.is_alive():
                        logger.warning(
                            f"Pull thread '{self._pull_thread_name}' did not stop in time"
                        )

                # Clean up
                self._cleanup_pipeline()
                self._observer = None
                logger.debug(f"{self._get_source_name()}: Disposal complete")

            return disposable.Disposable(dispose)

        return create(on_subscribe)


class GStreamerSinkBase(GStreamerPipelineBase, ABC):
    """Base class for GStreamer sinks that use appsrc.

    This handles common functionality for all push-based audio sinks:
    - Appsrc configuration and management
    - AudioEvent to buffer conversion
    - Observer implementation

    Subclasses need to implement:
    - _get_pipeline_string(): Return the GStreamer pipeline after appsrc
    """

    def __init__(self, config: GStreamerNodeConfig):
        super().__init__()
        self.config = config

        # Pipeline elements
        self._appsrc: Optional[GstApp.AppSrc] = None

        # Track if we're actively playing
        self._is_playing = False
        self._subscription: Optional[DisposableBase] = None

        # Input format detection
        self._input_format: Optional[AudioSpec] = None

    @abstractmethod
    def _get_pipeline_string(self) -> str:
        """Get the GStreamer pipeline string after appsrc.

        This should return everything after the appsrc element.
        The base class will add appsrc and appropriate conversion.

        Returns:
            Pipeline string (e.g., "audioconvert ! autoaudiosink")
        """
        pass

    def _get_sink_name(self) -> str:
        """Get a descriptive name for this sink. Override for better names."""
        return self.__class__.__name__

    def _create_pipeline(self):
        """Create the complete GStreamer pipeline with appsrc."""
        # Get sink-specific pipeline
        sink_pipeline = self._get_pipeline_string()

        # Build complete pipeline with appsrc
        # We'll accept any format and let audioconvert handle it
        pipeline_str = f"appsrc name=src ! {sink_pipeline}"

        logger.debug(f"{self._get_sink_name()}: Creating pipeline: {pipeline_str}")
        self._pipeline = Gst.parse_launch(pipeline_str)

        # Get and configure appsrc
        self._appsrc = validate_pipeline_element(self._pipeline, "src")
        self._configure_appsrc()

        # Configure sink if it exists and has properties
        if hasattr(self, "_configure_sink"):
            self._configure_sink()

        # Set up bus for error handling
        self._setup_bus(self._pipeline)

    def _configure_appsrc(self):
        """Configure appsrc properties."""
        # Set properties for non-live streaming (play as fast as possible)
        self._appsrc.set_property("is-live", False)
        self._appsrc.set_property("format", Gst.Format.TIME)

        # Don't block when queue is full - drop old buffers
        self._appsrc.set_property("block", False)
        self._appsrc.set_property("max-bytes", 0)  # No limit

        # Set stream-type to seekable for proper EOS handling
        self._appsrc.set_property("stream-type", 0)  # GST_APP_STREAM_TYPE_STREAM

    def _handle_error(self, error):
        """Handle pipeline errors."""
        logger.error(f"{self._get_sink_name()}: Pipeline error: {error}")
        # Don't stop on errors - just log them

    def _handle_eos(self):
        """Handle end of stream - automatically stop the sink."""
        import threading
        import traceback

        logger.info(
            f"{self._get_sink_name()}: EOS received in thread {threading.current_thread().name}"
        )
        logger.debug(
            f"{self._get_sink_name()}: EOS stack trace:\n{''.join(traceback.format_stack())}"
        )

        # Schedule stop in a separate thread to avoid deadlock
        # (EOS handler runs in GStreamer thread, can't join itself)
        def stop_and_log():
            logger.info(f"{self._get_sink_name()}: Calling stop() from EOS handler")
            self.stop()
            logger.info(
                f"{self._get_sink_name()}: stop() completed, _is_playing={self._is_playing}"
            )

        threading.Thread(
            target=stop_and_log, daemon=True, name=f"{self._get_sink_name()}-stop"
        ).start()

    def _audio_event_to_buffer(self, event: AudioEvent) -> Gst.Buffer:
        """Convert an AudioEvent to a GStreamer buffer."""
        # Set caps based on first event
        if self._input_format is None and self._appsrc:
            self._input_format = AudioSpec(
                format=event.format, sample_rate=event.sample_rate, channels=event.channels
            )
            # Ensure caps include layout=interleaved for proper negotiation
            caps_str = self._input_format.to_gst_caps_string()
            if "layout=" not in caps_str:
                caps_str += ",layout=interleaved"
            caps = Gst.Caps.from_string(caps_str)
            self._appsrc.set_property("caps", caps)
            logger.info(f"{self._get_sink_name()}: Set input caps: {caps_str}")

            # Notify subclass about input format if it has a handler
            if hasattr(self, "_on_input_format_detected"):
                self._on_input_format_detected(self._input_format)

        # Create buffer
        buffer = Gst.Buffer.new_allocate(None, 0, None)

        if isinstance(event, RawAudioEvent):
            # Convert numpy array to bytes
            if event.channels > 1:
                # Ensure data is C-contiguous for GStreamer
                data = np.ascontiguousarray(event.data)
            else:
                data = event.data

            buffer = Gst.Buffer.new_wrapped(data.tobytes())

        elif isinstance(event, CompressedAudioEvent):
            # Use compressed data directly
            buffer = Gst.Buffer.new_wrapped(event.data)

        # Set timestamp
        buffer.pts = int(event.timestamp * Gst.SECOND)

        # Set duration if available
        if hasattr(event, "duration") and event.duration:
            buffer.duration = int(event.duration * Gst.SECOND)
        elif isinstance(event, RawAudioEvent):
            # Calculate duration from sample count
            num_samples = len(event.data)
            duration_sec = num_samples / event.sample_rate
            buffer.duration = int(duration_sec * Gst.SECOND)

        return buffer

    def on_next(self, event: AudioEvent):
        """Handle incoming audio event."""
        if not self._is_playing:
            # Auto-start on first event
            self.start()

        if not self._appsrc:
            return

        try:
            buffer = self._audio_event_to_buffer(event)

            # Push buffer to appsrc
            ret = self._appsrc.emit("push-buffer", buffer)
            if ret != Gst.FlowReturn.OK:
                logger.warning(f"{self._get_sink_name()}: Failed to push buffer: {ret}")
            else:
                if not hasattr(self, "_buffer_count"):
                    self._buffer_count = 0
                self._buffer_count += 1
                if self._buffer_count == 1 or self._buffer_count % 10 == 0:
                    logger.info(
                        f"{self._get_sink_name()}: Pushed {self._buffer_count} buffers to sink (last size={buffer.get_size()})"
                    )

        except Exception as e:
            logger.error(f"{self._get_sink_name()}: Error processing event: {e}")

    def on_error(self, error: Exception):
        """Handle stream error."""
        logger.error(f"{self._get_sink_name()}: Stream error: {error}")
        self.stop()

    def on_completed(self):
        """Handle stream completion."""
        logger.info(f"{self._get_sink_name()}: Stream completed, sending EOS to pipeline")
        # Send EOS to pipeline - this will trigger _handle_eos() which stops the sink
        if self._appsrc:
            self._appsrc.emit("end-of-stream")
            logger.info(f"{self._get_sink_name()}: EOS sent to pipeline")
        else:
            logger.warning(f"{self._get_sink_name()}: No appsrc to send EOS to")

    def start(self):
        """Start the audio sink pipeline."""
        if self._is_playing:
            return

        try:
            # Ensure pipeline is ready
            self._ensure_pipeline_ready()
            self._create_pipeline()

            # Start pipeline
            ret = self._pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Failed to start pipeline")

            self._is_playing = True
            logger.info(f"{self._get_sink_name()}: Started")

        except Exception as e:
            logger.error(f"{self._get_sink_name()}: Failed to start: {e}")
            self._cleanup_pipeline()
            raise

    def stop(self):
        """Stop the audio sink pipeline."""
        if not self._is_playing:
            return

        logger.info(f"{self._get_sink_name()}: Stopping")
        self._is_playing = False

        # Stop pipeline
        if self._pipeline:
            self._pipeline.set_state(Gst.State.NULL)
            # Wait for state change to complete
            self._pipeline.get_state(Gst.CLOCK_TIME_NONE)

        # Clean up
        self._cleanup_pipeline()
        self._input_format = None

        logger.debug(f"{self._get_sink_name()}: Stopped")

    def subscribe_to(self, observable: Observable[AudioEvent]) -> DisposableBase:
        """Subscribe this sink to an audio observable.

        Args:
            observable: Observable that emits AudioEvents

        Returns:
            Disposable subscription
        """
        # Ensure we start before subscribing
        self.start()

        # Subscribe
        self._subscription = observable.subscribe(
            on_next=self.on_next, on_error=self.on_error, on_completed=self.on_completed
        )

        return self._subscription

    def __enter__(self):
        """Context manager support."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager cleanup."""
        if self._subscription:
            self._subscription.dispose()
        self.stop()
