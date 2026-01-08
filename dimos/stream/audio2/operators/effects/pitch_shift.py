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

"""Pitch shift effect using GStreamer's pitch element."""

import threading
import time

import gi
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst
from reactivex import create
from reactivex.abc import ObservableBase

from dimos.stream.audio2.gstreamer import GStreamerPipelineBase
from dimos.stream.audio2.types import AudioEvent, AudioSpec, RawAudioEvent
from dimos.stream.audio2.utils import buffer_to_audio_event, validate_pipeline_element
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.operators.effects.pitch_shift")


class PitchShiftNode(GStreamerPipelineBase):
    """GStreamer-based pitch shift effect."""

    def __init__(self, pitch: float = 1.0):
        super().__init__()

        # Pitch shift parameter
        self.pitch = pitch

        # Pipeline elements
        self._appsrc = None
        self._appsink = None
        self._pitch = None

        # Input/output format tracking
        self._input_format = None
        self._output_rate = None
        self._output_channels = None

        # Threading
        self._pull_thread = None
        self._observer = None

    def _create_pipeline(self):
        """Create the pitch shift pipeline.

        Expects raw PCM F32LE input (standard internal format).
        """
        # Pipeline: appsrc → pitch → appsink
        pipeline_str = (
            f"appsrc name=src format=time ! "
            f"queue ! "
            f"audioconvert ! "
            f"audio/x-raw,format=F32LE ! "
            f"pitch name=pitch pitch={self.pitch} ! "
            f"audioconvert ! "
            f"audio/x-raw,format=F32LE ! "
            f"appsink name=sink emit-signals=true sync=false"
        )

        logger.debug(f"Creating pitch shift pipeline: {pipeline_str}")
        self._pipeline = Gst.parse_launch(pipeline_str)

        # Get elements
        self._appsrc = validate_pipeline_element(self._pipeline, "src")
        self._appsink = validate_pipeline_element(self._pipeline, "sink")
        self._pitch = validate_pipeline_element(self._pipeline, "pitch")

        # Configure appsrc
        self._appsrc.set_property("is-live", False)
        self._appsrc.set_property("format", Gst.Format.TIME)
        self._appsrc.set_property("block", False)

        # Configure appsink
        self._appsink.set_property("emit-signals", True)
        self._appsink.set_property("sync", False)

        # Set up bus
        self._setup_bus(self._pipeline)

    def _push_event_to_pipeline(self, event: AudioEvent):
        """Push a raw AudioEvent to the pipeline via appsrc.

        Only accepts raw PCM audio (standard internal format).
        """
        # Set caps on first event
        if self._input_format is None:
            if not event.format.is_raw:
                raise ValueError(
                    f"PitchShift expects raw audio input, got {event.format.value}. "
                    "Ensure upstream source outputs raw audio (default behavior)."
                )

            self._input_format = AudioSpec(
                format=event.format, sample_rate=event.sample_rate, channels=event.channels
            )
            caps_str = self._input_format.to_gst_caps_string()
            caps = Gst.Caps.from_string(caps_str)
            self._appsrc.set_property("caps", caps)
            logger.info(f"Set input caps: {caps_str}")

        # Create buffer from raw audio
        if not isinstance(event, RawAudioEvent):
            logger.warning(f"Unexpected event type: {type(event)}, skipping")
            return

        if event.channels > 1:
            data = np.ascontiguousarray(event.data)
        else:
            data = event.data
        buffer = Gst.Buffer.new_wrapped(data.tobytes())

        # Set timestamp
        buffer.pts = int(event.timestamp * Gst.SECOND)

        # Push to pipeline
        ret = self._appsrc.emit("push-buffer", buffer)
        if ret != Gst.FlowReturn.OK:
            logger.warning(f"Failed to push buffer: {ret}")

    def _pull_from_pipeline(self):
        """Pull processed samples from appsink and emit as AudioEvents."""
        logger.debug("Pull thread starting")

        buffer_count = 0
        try:
            while self._running:
                sample = self._appsink.emit("pull-sample")
                if sample is None:
                    if not self._running:
                        break
                    time.sleep(0.001)
                    continue

                buffer = sample.get_buffer()
                if buffer is None:
                    continue

                # Parse caps on first sample
                if self._output_rate is None:
                    caps = sample.get_caps()
                    if caps:
                        structure = caps.get_structure(0)
                        self._output_rate = structure.get_int("rate")[1]
                        self._output_channels = structure.get_int("channels")[1]
                        logger.info(
                            f"Output format: {self._output_rate}Hz, {self._output_channels}ch"
                        )

                # Convert to AudioEvent
                from dimos.stream.audio2.types import AudioFormat

                output_spec = AudioSpec(
                    format=AudioFormat.PCM_F32LE,
                    sample_rate=self._output_rate,
                    channels=self._output_channels,
                )

                # Extract timestamp from buffer PTS (preserve timing from input)
                buffer_timestamp = None
                if buffer.pts != Gst.CLOCK_TIME_NONE:
                    buffer_timestamp = buffer.pts / Gst.SECOND

                event = buffer_to_audio_event(
                    buffer=buffer,
                    spec=output_spec,
                    detected_rate=self._output_rate,
                    detected_channels=self._output_channels,
                    timestamp=buffer_timestamp,
                )

                buffer_count += 1

                if self._observer:
                    self._observer.on_next(event)

        except Exception as e:
            logger.error(f"Pull thread error: {e}")
            if self._observer:
                self._observer.on_error(e)

        finally:
            logger.info(f"Pull thread exiting (processed {buffer_count} buffers)")

    def process_observable(self, source: ObservableBase) -> ObservableBase:
        """Process an observable of AudioEvents through the pitch shift effect."""

        def subscribe(observer, scheduler=None):
            self._observer = observer

            try:
                # Ensure mainloop is running
                self._ensure_pipeline_ready()
                self._create_pipeline()

                # Start pipeline
                ret = self._pipeline.set_state(Gst.State.PLAYING)
                if ret == Gst.StateChangeReturn.FAILURE:
                    raise RuntimeError("Failed to start pipeline")

                # Start pull thread
                self._pull_thread = threading.Thread(
                    target=self._pull_from_pipeline, daemon=True, name="PitchShift-pull"
                )
                self._pull_thread.start()

                logger.info(f"GStreamer pitch shift started (pitch={self.pitch})")

                # Subscribe to source and push events to pipeline
                def on_next(event):
                    self._push_event_to_pipeline(event)

                def on_error(error):
                    logger.error(f"Source error: {error}")
                    self._cleanup_pipeline()
                    observer.on_error(error)

                def on_completed():
                    logger.info("Source completed, sending EOS")
                    if self._appsrc:
                        self._appsrc.emit("end-of-stream")
                    # Wait a bit for pipeline to finish processing
                    time.sleep(0.1)
                    self._running = False
                    if self._pull_thread:
                        self._pull_thread.join(timeout=2.0)

                    # Cleanup pipeline BEFORE calling observer.on_completed()
                    self._cleanup_pipeline()

                    # Now notify observer
                    observer.on_completed()

                source.subscribe(on_next, on_error, on_completed, scheduler=scheduler)

            except Exception as e:
                observer.on_error(e)
                self._cleanup_pipeline()

            def dispose():
                logger.info("Disposing GStreamer pitch shift effect")
                self._running = False
                if self._pipeline:
                    self._pipeline.set_state(Gst.State.NULL)
                if self._pull_thread:
                    self._pull_thread.join(timeout=2.0)
                self._cleanup_pipeline()

            from reactivex import disposable

            return disposable.Disposable(dispose)

        return create(subscribe)


def pitch_shift(pitch: float = 1.0):
    """Create a GStreamer-based pitch shift effect operator.

    This operator shifts the pitch of audio by a given multiplier without changing tempo.
    Works with raw PCM audio (standard internal format).

    Args:
        pitch: Pitch shift multiplier. Values > 1.0 shift pitch up, < 1.0 shift down.
               For example: 1.5 = up 7 semitones, 0.5 = down 12 semitones (default: 1.0)

    Returns:
        An operator function that can be used with pipe()

    Examples:
        # Shift pitch up by a fifth (1.5x)
        file_input("voice.wav").pipe(
            pitch_shift(1.5),
            speaker()
        ).run()

        # Shift pitch down an octave
        file_input("music.mp3").pipe(
            pitch_shift(0.5),
            network_output()
        ).run()
    """

    node = PitchShiftNode(pitch=pitch)

    def _pitch_shift(source: ObservableBase) -> ObservableBase:
        return node.process_observable(source)

    return _pitch_shift
