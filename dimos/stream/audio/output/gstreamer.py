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

from typing import Optional

import numpy as np
from reactivex import Observable

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
gi.require_version("GstAudio", "1.0")
from gi.repository import Gst, GstApp, GstAudio, GLib

from dimos.stream.audio.base import AbstractAudioTransform
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio.output.gstreamer")


class GstreamerOutput(AbstractAudioTransform):
    """
    Audio output implementation using GStreamer.

    This class implements AbstractAudioTransform to play audio through GStreamer
    and optionally pass audio events through to other components.
    """

    def __init__(
        self,
        sample_rate: int = 16000,
        channels: int = 1,
        dtype: np.dtype = np.float32,
        device: Optional[str] = None,
    ):
        """
        Initialize GstreamerOutput.

        Args:
            sample_rate: Audio sample rate in Hz
            channels: Number of audio channels (1=mono, 2=stereo)
            dtype: Data type for audio samples (np.float32 or np.int16)
            device: Audio device name (None for default autoaudiosink)
        """
        Gst.init(None)

        self.sample_rate = sample_rate
        self.channels = channels
        self.dtype = dtype
        self.device = device

        self._pipeline = None
        self._appsrc = None
        self._running = False
        self._subscription = None
        self.audio_observable = None

        # Determine audio format based on dtype
        if dtype == np.float32:
            self.audio_format = "F32LE"
            self.gst_format = GstAudio.AudioFormat.F32LE
        elif dtype == np.int16:
            self.audio_format = "S16LE"
            self.gst_format = GstAudio.AudioFormat.S16LE
        else:
            raise ValueError(f"Unsupported dtype: {dtype}")

        self._create_pipeline()

    def _create_pipeline(self):
        """Create the GStreamer pipeline for audio output."""
        # Create pipeline
        self._pipeline = Gst.Pipeline.new("audio-output-pipeline")

        # Create elements
        self._appsrc = Gst.ElementFactory.make("appsrc", "audio-source")
        audioconvert = Gst.ElementFactory.make("audioconvert", "convert")
        audioresample = Gst.ElementFactory.make("audioresample", "resample")

        # Use specified device or autoaudiosink
        if self.device:
            audiosink = Gst.ElementFactory.make(self.device, "output")
            if not audiosink:
                logger.warning(f"Failed to create {self.device}, falling back to autoaudiosink")
                audiosink = Gst.ElementFactory.make("autoaudiosink", "output")
        else:
            audiosink = Gst.ElementFactory.make("autoaudiosink", "output")

        if not all([self._appsrc, audioconvert, audioresample, audiosink]):
            raise RuntimeError("Failed to create GStreamer elements")

        # Configure appsrc
        caps_str = f"audio/x-raw,format={self.audio_format},rate={self.sample_rate},channels={self.channels},layout=interleaved"
        caps = Gst.Caps.from_string(caps_str)
        self._appsrc.set_property("caps", caps)
        self._appsrc.set_property("format", Gst.Format.TIME)
        self._appsrc.set_property("is-live", True)
        self._appsrc.set_property("block", False)

        # Add elements to pipeline
        self._pipeline.add(self._appsrc)
        self._pipeline.add(audioconvert)
        self._pipeline.add(audioresample)
        self._pipeline.add(audiosink)

        # Link elements
        if not self._appsrc.link(audioconvert):
            raise RuntimeError("Failed to link appsrc to audioconvert")
        if not audioconvert.link(audioresample):
            raise RuntimeError("Failed to link audioconvert to audioresample")
        if not audioresample.link(audiosink):
            raise RuntimeError("Failed to link audioresample to audiosink")

        # Set up bus for error handling
        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    def _on_bus_message(self, bus, message):
        """Handle messages from the GStreamer bus."""
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            logger.error(f"GStreamer error: {err}, {debug}")
            self.stop()
        elif message.type == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            logger.warning(f"GStreamer warning: {err}, {debug}")
        elif message.type == Gst.MessageType.EOS:
            logger.info("GStreamer: End of stream")
            self.stop()

    def consume_audio(self, audio_observable: Observable) -> "GstreamerOutput":
        """
        Subscribe to an audio observable and play the audio through GStreamer.

        Args:
            audio_observable: Observable emitting AudioEvent objects

        Returns:
            Self for method chaining
        """
        self.audio_observable = audio_observable

        # Start the pipeline
        ret = self._pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to start GStreamer pipeline")

        self._running = True

        logger.info(
            f"Started GStreamer audio output: {self.sample_rate}Hz, "
            f"{self.channels} channels, format={self.audio_format}"
        )

        # Subscribe to the observable
        self._subscription = audio_observable.subscribe(
            on_next=self._play_audio_event,
            on_error=self._handle_error,
            on_completed=self._handle_completion,
        )

        return self

    def emit_audio(self) -> Observable:
        """
        Pass through the audio observable to allow chaining with other components.

        Returns:
            The same Observable that was provided to consume_audio
        """
        if self.audio_observable is None:
            raise ValueError("No audio source provided. Call consume_audio() first.")

        return self.audio_observable

    def _play_audio_event(self, audio_event):
        """Push audio data to GStreamer pipeline."""
        if not self._running or not self._appsrc:
            return

        try:
            # Ensure data type matches our stream
            if audio_event.dtype != self.dtype:
                if self.dtype == np.float32:
                    audio_event = audio_event.to_float32()
                elif self.dtype == np.int16:
                    audio_event = audio_event.to_int16()

            # Create GStreamer buffer from audio data
            data = audio_event.data.tobytes()
            buf = Gst.Buffer.new_wrapped(data)

            # Set timestamp if available
            if hasattr(audio_event, "timestamp") and audio_event.timestamp is not None:
                buf.pts = int(audio_event.timestamp * Gst.SECOND)
                buf.duration = int(len(audio_event.data) * Gst.SECOND / self.sample_rate)

            # Push buffer to pipeline
            ret = self._appsrc.emit("push-buffer", buf)
            if ret != Gst.FlowReturn.OK:
                logger.warning(f"Failed to push buffer: {ret}")

        except Exception as e:
            logger.error(f"Error playing audio: {e}")

    def _handle_error(self, error):
        """Handle errors from the observable."""
        logger.error(f"Error in audio observable: {error}")
        self.stop()

    def _handle_completion(self):
        """Handle completion of the observable."""
        logger.info("Audio observable completed")
        if self._appsrc:
            self._appsrc.emit("end-of-stream")

    def stop(self):
        """Stop audio output and clean up resources."""
        logger.info("Stopping GStreamer audio output")
        self._running = False

        if self._subscription:
            self._subscription.dispose()
            self._subscription = None

        if self._pipeline:
            self._pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
            self._appsrc = None


if __name__ == "__main__":
    from dimos.stream.audio.input.microphone import SounddeviceAudioSource
    from dimos.stream.audio.node_normalizer import AudioNormalizer
    from dimos.stream.audio.utils import keepalive

    # Create microphone source, normalizer and audio output
    mic = SounddeviceAudioSource()
    normalizer = AudioNormalizer()
    speaker = GstreamerOutput()

    # Connect the components in a pipeline
    normalizer.consume_audio(mic.emit_audio())
    speaker.consume_audio(normalizer.emit_audio())

    keepalive()
