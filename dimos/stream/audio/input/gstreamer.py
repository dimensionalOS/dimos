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
import threading
import time

import numpy as np
from reactivex import Observable, create, disposable

try:
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    gi.require_version("GstAudio", "1.0")
    from gi.repository import Gst, GstApp, GstAudio, GLib
except (ImportError, ValueError) as exc:  # pragma: no cover - exercised in CI without GStreamer
    raise ImportError("GStreamer dependencies are not available") from exc

from dimos.stream.audio.base import AbstractAudioEmitter, AudioEvent
from dimos.utils.gstreamer_manager import ensure_mainloop_running
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio.input.gstreamer")


class GstreamerInput(AbstractAudioEmitter):
    """
    Audio input implementation using GStreamer.

    This class can receive audio from various GStreamer sources including:
    - audiotestsrc for testing
    - autoaudiosrc for microphone input
    - udpsrc for network audio
    - filesrc for audio files
    """

    def __init__(
        self,
        pipeline_str: Optional[str] = None,
        sample_rate: int = 16000,
        channels: int = 1,
        dtype: np.dtype = np.float32,
    ):
        """
        Initialize GstreamerInput.

        Args:
            pipeline_str: Custom GStreamer pipeline string (before appsink)
                         If None, uses audiotestsrc for testing
            sample_rate: Audio sample rate in Hz
            channels: Number of audio channels (1=mono, 2=stereo)
            dtype: Data type for audio samples (np.float32 or np.int16)
        """
        Gst.init(None)

        # Ensure GLib MainLoop is running for GStreamer
        ensure_mainloop_running()

        self.sample_rate = sample_rate
        self.channels = channels
        self.dtype = dtype
        self.pipeline_str = pipeline_str

        self._pipeline = None
        self._appsink = None
        self._running = False
        self._thread = None
        self._observer = None

        # Determine audio format based on dtype
        if dtype == np.float32:
            self.audio_format = "F32LE"
            self.bytes_per_sample = 4
        elif dtype == np.int16:
            self.audio_format = "S16LE"
            self.bytes_per_sample = 2
        else:
            raise ValueError(f"Unsupported dtype: {dtype}")

    def _create_pipeline(self):
        """Create the GStreamer pipeline for audio input."""
        # Default pipeline uses audiotestsrc for testing
        if self.pipeline_str is None:
            self.pipeline_str = "audiotestsrc wave=sine freq=440 ! audioconvert ! audioresample"

        # Build complete pipeline with appsink
        caps_str = f"audio/x-raw,format={self.audio_format},rate={self.sample_rate},channels={self.channels},layout=interleaved"
        full_pipeline_str = f"{self.pipeline_str} ! {caps_str} ! appsink name=sink"

        # Parse pipeline
        self._pipeline = Gst.parse_launch(full_pipeline_str)

        # Get appsink element
        self._appsink = self._pipeline.get_by_name("sink")
        if not self._appsink:
            raise RuntimeError("Failed to get appsink from pipeline")

        # Configure appsink
        self._appsink.set_property("emit-signals", True)
        self._appsink.set_property("sync", False)
        self._appsink.set_property("max-buffers", 10)
        self._appsink.set_property("drop", True)

        # Set up bus for error handling
        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

    def _on_bus_message(self, bus, message):
        """Handle messages from the GStreamer bus."""
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            logger.error(f"GStreamer error: {err}, {debug}")
            self._running = False
        elif message.type == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            logger.warning(f"GStreamer warning: {err}, {debug}")
        elif message.type == Gst.MessageType.EOS:
            logger.info("GStreamer: End of stream")
            self._running = False

    def _pull_samples_thread(self):
        """Thread to pull samples from appsink and emit AudioEvents."""
        try:
            while self._running:
                # Pull sample from appsink
                sample = self._appsink.emit("pull-sample")
                if sample is None:
                    time.sleep(0.001)
                    continue

                # Get buffer from sample
                buf = sample.get_buffer()
                if buf is None:
                    continue

                # Extract audio data
                success, map_info = buf.map(Gst.MapFlags.READ)
                if not success:
                    continue

                try:
                    # Convert buffer data to numpy array
                    data = np.frombuffer(map_info.data, dtype=self.dtype)

                    # Reshape for multiple channels
                    if self.channels > 1:
                        data = data.reshape(-1, self.channels)

                    # Create AudioEvent
                    audio_event = AudioEvent(
                        data=data,
                        sample_rate=self.sample_rate,
                        timestamp=time.time(),
                        channels=self.channels,
                    )

                    # Emit event
                    if self._observer:
                        self._observer.on_next(audio_event)

                finally:
                    buf.unmap(map_info)

        except Exception as e:
            logger.error(f"Error in GStreamer pull thread: {e}")
            if self._observer:
                self._observer.on_error(e)
        finally:
            if self._observer:
                self._observer.on_completed()

    def emit_audio(self) -> Observable:
        """
        Create an observable that emits audio from GStreamer.

        Returns:
            Observable emitting AudioEvent objects
        """

        def on_subscribe(observer, scheduler):
            self._observer = observer

            # Create pipeline
            self._create_pipeline()

            # Start pipeline
            ret = self._pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("Failed to start GStreamer pipeline")

            self._running = True

            # Start pull thread
            self._thread = threading.Thread(target=self._pull_samples_thread, daemon=True)
            self._thread.start()

            logger.info(
                f"Started GStreamer audio input: {self.sample_rate}Hz, "
                f"{self.channels} channels, format={self.audio_format}"
            )

            # Return disposable
            def dispose():
                logger.info("Stopping GStreamer audio input")
                self._running = False

                if self._pipeline:
                    self._pipeline.set_state(Gst.State.NULL)
                    self._pipeline = None

                # Only join if we're not being called from the pull thread itself
                if (
                    self._thread
                    and self._thread.is_alive()
                    and threading.current_thread() != self._thread
                ):
                    self._thread.join(timeout=1.0)

                self._observer = None

            return disposable.Disposable(dispose)

        return create(on_subscribe)


if __name__ == "__main__":
    from dimos.stream.audio.node_normalizer import AudioNormalizer
    from dimos.stream.audio.output.soundcard import SounddeviceAudioOutput
    from dimos.stream.audio.utils import keepalive

    # Create GStreamer input with test source
    gst_input = GstreamerInput()

    # Create normalizer and output
    normalizer = AudioNormalizer()
    speaker = SounddeviceAudioOutput()

    # Connect pipeline
    normalizer.consume_audio(gst_input.emit_audio())
    speaker.consume_audio(normalizer.emit_audio())

    keepalive()
