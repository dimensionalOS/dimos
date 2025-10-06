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

import gi
import numpy as np
from reactivex import Observable

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
gi.require_version("GstAudio", "1.0")
from gi.repository import GLib, Gst, GstApp, GstAudio

from dimos.stream.audio.base import AbstractAudioTransform
from dimos.stream.audio.input.player import FilePlayerInput
from dimos.stream.audio.node_normalizer import AudioNormalizer
from dimos.utils.gstreamer_manager import ensure_mainloop_running
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
        input_sample_rate: int = 44100,
        channels: int = 1,
        dtype: np.dtype = np.float32,
        device: Optional[str] = None,
    ):
        """
        Initialize GstreamerOutput.

        Args:
            input_sample_rate: Expected sample rate of input audio in Hz
            channels: Number of audio channels (1=mono, 2=stereo)
            dtype: Data type for audio samples (np.float32 or np.int16)
            device: Audio device name (None for default autoaudiosink)
        """
        Gst.init(None)

        # Ensure GLib MainLoop is running for GStreamer
        ensure_mainloop_running()

        self.input_sample_rate = input_sample_rate
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
        queue = Gst.ElementFactory.make("queue", "buffer")
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

        if not all([self._appsrc, queue, audioconvert, audioresample, audiosink]):
            raise RuntimeError("Failed to create GStreamer elements")

        # Configure appsrc
        caps_str = f"audio/x-raw,format={self.audio_format},rate={self.input_sample_rate},channels={self.channels},layout=interleaved"
        caps = Gst.Caps.from_string(caps_str)
        self._appsrc.set_property("caps", caps)
        self._appsrc.set_property("format", Gst.Format.TIME)
        self._appsrc.set_property("is-live", False)  # Always non-live for source-agnostic behavior
        self._appsrc.set_property("block", False)

        # Configure queue for buffering
        queue.set_property("max-size-time", 200000000)  # 200ms buffer
        queue.set_property("max-size-buffers", 0)
        queue.set_property("max-size-bytes", 0)

        # Add elements to pipeline
        self._pipeline.add(self._appsrc)
        self._pipeline.add(queue)
        self._pipeline.add(audioconvert)
        self._pipeline.add(audioresample)
        self._pipeline.add(audiosink)

        # Link elements
        if not self._appsrc.link(queue):
            raise RuntimeError("Failed to link appsrc to queue")
        if not queue.link(audioconvert):
            raise RuntimeError("Failed to link queue to audioconvert")
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
            f"Started GStreamer audio output: {self.input_sample_rate}Hz input, "
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

            # Set buffer duration based on sample count
            buf.duration = int(len(audio_event.data) * Gst.SECOND / self.input_sample_rate)

            # For non-live sources, we don't need timestamps as GStreamer will handle timing

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
    import sys

    from dimos.stream.audio.input.player import FilePlayerInput
    from dimos.stream.audio.utils import keepalive
    from dimos.utils.data import get_data

    # Test with petty_concerns.wav or command line argument
    if len(sys.argv) > 1:
        audio_file = sys.argv[1]
    else:
        try:
            audio_file = get_data("petty_concerns.wav")
        except Exception:
            print("Usage: python gstreamer.py [audio_file]")
            print("No audio file specified and petty_concerns.wav not found")
            sys.exit(1)

    loop = "--loop" in sys.argv

    print(f"Testing GstreamerOutput with: {audio_file}")
    print(f"Loop mode: {loop}")
    print("Press Ctrl+C to stop")
    print("-" * 40)

    try:
        # Create file player
        file_player = FilePlayerInput(str(audio_file), loop=loop, sample_rate=44100)

        # Create audio output
        speaker = GstreamerOutput(input_sample_rate=44100)

        normalizer = AudioNormalizer()

        # Connect pipeline
        normalizer.consume_audio(file_player.emit_audio())
        speaker.consume_audio(normalizer.emit_audio())
        # Keep running
        keepalive()

    except KeyboardInterrupt:
        print("\nStopped by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
