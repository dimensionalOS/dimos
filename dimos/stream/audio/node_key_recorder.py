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

import select
import sys
import threading
import time

import numpy as np
from reactivex import Observable
from reactivex.subject import ReplaySubject, Subject

from dimos.stream.audio.base import AbstractAudioTransform, AudioEvent
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class KeyRecorder(AbstractAudioTransform):
    """
    Audio recorder that captures audio events and combines them.
    Press a key to toggle recording on/off.
    """

    def __init__(
        self,
        max_recording_time: float = 120.0,
        always_subscribe: bool = False,
    ) -> None:
        """
        Initialize KeyRecorder.

        Args:
            max_recording_time: Maximum recording time in seconds
            always_subscribe: If True, subscribe to audio source continuously,
                              If False, only subscribe when recording (more efficient
                              but some audio devices may need time to initialize)
        """
        self.max_recording_time = max_recording_time
        self.always_subscribe = always_subscribe

        self._audio_buffer = []  # type: ignore[var-annotated]
        self._is_recording = False
        self._recording_start_time = 0
        self._sample_rate = None  # Will be updated from incoming audio
        self._channels = None  # Will be set from first event

        self._audio_observable = None
        self._subscription = None
        self._output_subject = Subject()  # type: ignore[var-annotated]  # For record-time passthrough
        self._recording_subject = ReplaySubject(1)  # type: ignore[var-annotated]  # For full completed recordings

        # Start a thread to monitor for input
        self._running = True
        self._input_thread = threading.Thread(target=self._input_monitor, daemon=True)
        self._input_thread.start()

        logger.info("Started audio recorder (press any key to start/stop recording)")

    def consume_audio(self, audio_observable: Observable) -> "KeyRecorder":  # type: ignore[type-arg]
        """
        Set the audio observable to use when recording.
        If always_subscribe is True, subscribes immediately.
        Otherwise, subscribes only when recording starts.

        Args:
            audio_observable: Observable emitting AudioEvent objects

        Returns:
            Self for method chaining
        """
        self._audio_observable = audio_observable  # type: ignore[assignment]

        # If configured to always subscribe, do it now
        if self.always_subscribe and not self._subscription:
            self._subscription = audio_observable.subscribe(  # type: ignore[assignment]
                on_next=self._process_audio_event,
                on_error=self._handle_error,
                on_completed=self._handle_completion,
            )
            logger.debug("Subscribed to audio source (always_subscribe=True)")

        return self

    def emit_audio(self) -> Observable:  # type: ignore[type-arg]
        """
        Create an observable that emits audio events in real-time (pass-through).

        Returns:
            Observable emitting AudioEvent objects in real-time
        """
        return self._output_subject

    def emit_recording(self) -> Observable:  # type: ignore[type-arg]
        """
        Create an observable that emits combined audio recordings when recording stops.

        Returns:
            Observable emitting AudioEvent objects with complete recordings
        """
        return self._recording_subject

    def stop(self) -> None:
        """Stop recording and clean up resources."""
        logger.info("Stopping audio recorder")

        # If recording is in progress, stop it first
        if self._is_recording:
            self._stop_recording()

        # Always clean up subscription on full stop
        if self._subscription:
            self._subscription.dispose()
            self._subscription = None

        # Stop input monitoring thread
        self._running = False
        if self._input_thread.is_alive():
            self._input_thread.join(1.0)

    def _input_monitor(self) -> None:
        """Monitor for key presses to toggle recording."""
        logger.info("Press Enter to start/stop recording...")

        while self._running:
            # Check if there's input available
            if select.select([sys.stdin], [], [], 0.1)[0]:
                sys.stdin.readline()

                if self._is_recording:
                    self._stop_recording()
                else:
                    self._start_recording()

            # Sleep a bit to reduce CPU usage
            time.sleep(0.1)

    def _start_recording(self) -> None:
        """Start recording audio and subscribe to the audio source if not always subscribed."""
        if not self._audio_observable:
            logger.error("Cannot start recording: No audio source has been set")
            return

        # Subscribe to the observable if not using always_subscribe
        if not self._subscription:
            self._subscription = self._audio_observable.subscribe(
                on_next=self._process_audio_event,
                on_error=self._handle_error,
                on_completed=self._handle_completion,
            )
            logger.debug("Subscribed to audio source for recording")

        self._is_recording = True
        self._recording_start_time = time.time()
        self._audio_buffer = []
        logger.info("Recording... (press Enter to stop)")

    def _stop_recording(self) -> None:
        """Stop recording, unsubscribe from audio source if not always subscribed, and emit the combined audio event."""
        self._is_recording = False
        recording_duration = time.time() - self._recording_start_time

        # Unsubscribe from the audio source if not using always_subscribe
        if not self.always_subscribe and self._subscription:
            self._subscription.dispose()
            self._subscription = None
            logger.debug("Unsubscribed from audio source after recording")

        logger.info(f"Recording stopped after {recording_duration:.2f} seconds")

        # Combine all audio events into one
        if len(self._audio_buffer) > 0:
            combined_audio = self._combine_audio_events(self._audio_buffer)
            self._recording_subject.on_next(combined_audio)
        else:
            logger.warning("No audio was recorded")

    def _process_audio_event(self, audio_event) -> None:  # type: ignore[no-untyped-def]
        """Process incoming audio events."""

        # Only buffer if recording
        if not self._is_recording:
            return

        # Pass through audio events in real-time
        self._output_subject.on_next(audio_event)

        # First audio event - determine channel count/sample rate
        if self._channels is None:
            self._channels = audio_event.channels
            self._sample_rate = audio_event.sample_rate
            logger.info(f"Setting channel count to {self._channels}")

        # Add to buffer
        self._audio_buffer.append(audio_event)

        # Check if we've exceeded max recording time
        if time.time() - self._recording_start_time > self.max_recording_time:
            logger.warning(f"Max recording time ({self.max_recording_time}s) reached")
            self._stop_recording()

    def _combine_audio_events(self, audio_events: list[AudioEvent]) -> AudioEvent:
        """Combine multiple audio events into a single event."""
        if not audio_events:
            logger.warning("Attempted to combine empty audio events list")
            return None  # type: ignore[return-value]

        # Filter out any empty events that might cause broadcasting errors
        valid_events = [
            event
            for event in audio_events
            if event is not None
            and (hasattr(event, "data") and event.data is not None and event.data.size > 0)
        ]

        if not valid_events:
            logger.warning("No valid audio events to combine")
            return None  # type: ignore[return-value]

        first_event = valid_events[0]
        channels = first_event.channels
        dtype = first_event.data.dtype

        # Calculate total samples only from valid events
        total_samples = sum(event.data.shape[0] for event in valid_events)

        # Safety check - if somehow we got no samples
        if total_samples <= 0:
            logger.warning(f"Combined audio would have {total_samples} samples - aborting")
            return None  # type: ignore[return-value]

        # For multichannel audio, data shape could be (samples,) or (samples, channels)
        if len(first_event.data.shape) == 1:
            # 1D audio data (mono)
            combined_data = np.zeros(total_samples, dtype=dtype)

            # Copy data
            offset = 0
            for event in valid_events:
                samples = event.data.shape[0]
                if samples > 0:  # Extra safety check
                    combined_data[offset : offset + samples] = event.data
                    offset += samples
        else:
            # Multichannel audio data (stereo or more)
            combined_data = np.zeros((total_samples, channels), dtype=dtype)

            # Copy data
            offset = 0
            for event in valid_events:
                samples = event.data.shape[0]
                if samples > 0 and offset + samples <= total_samples:  # Safety check
                    try:
                        combined_data[offset : offset + samples] = event.data
                        offset += samples
                    except ValueError as e:
                        logger.error(
                            f"Error combining audio events: {e}. "
                            f"Event shape: {event.data.shape}, "
                            f"Combined shape: {combined_data.shape}, "
                            f"Offset: {offset}, Samples: {samples}"
                        )
                        # Continue with next event instead of failing completely

        # Create new audio event with the combined data
        if combined_data.size > 0:
            return AudioEvent(
                data=combined_data,
                sample_rate=self._sample_rate,  # type: ignore[arg-type]
                timestamp=valid_events[0].timestamp,
                channels=channels,
            )
        else:
            logger.warning("Failed to create valid combined audio event")
            return None  # type: ignore[return-value]

    def _handle_error(self, error) -> None:  # type: ignore[no-untyped-def]
        """Handle errors from the observable."""
        logger.error(f"Error in audio observable: {error}")

    def _handle_completion(self) -> None:
        """Handle completion of the observable."""
        logger.info("Audio observable completed")
        self.stop()


if __name__ == "__main__":
    from dimos.stream.audio.node_microphone import (
        SounddeviceAudioSource,
    )
    from dimos.stream.audio.node_normalizer import AudioNormalizer
    from dimos.stream.audio.node_output import SounddeviceAudioOutput
    from dimos.stream.audio.node_volume_monitor import monitor
    from dimos.stream.audio.utils import keepalive

    # Create microphone source, recorder, and audio output
    mic = SounddeviceAudioSource()

    # my audio device needs time to init, so for smoother ux we constantly listen
    recorder = KeyRecorder(always_subscribe=True)

    normalizer = AudioNormalizer()
    speaker = SounddeviceAudioOutput()

    # Connect the components
    normalizer.consume_audio(mic.emit_audio())
    recorder.consume_audio(normalizer.emit_audio())
    # recorder.consume_audio(mic.emit_audio())

    # Monitor microphone input levels (real-time pass-through)
    monitor(recorder.emit_audio())

    # Connect the recorder output to the speakers to hear recordings when completed
    playback_speaker = SounddeviceAudioOutput()
    playback_speaker.consume_audio(recorder.emit_recording())

    # TODO: we should be able to run normalizer post hoc on the recording as well,
    # it's not working, this needs a review
    #
    # normalizer.consume_audio(recorder.emit_recording())
    # playback_speaker.consume_audio(normalizer.emit_audio())

    keepalive()
