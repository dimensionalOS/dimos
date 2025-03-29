#!/usr/bin/env python3
from typing import List
import numpy as np
import time
import threading
import sys
import select
from reactivex import Observable
from reactivex.subject import Subject, ReplaySubject

from dimos.stream.audio.base import AbstractAudioTransform, AudioEvent

from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.audio.key_recorder")


class KeyRecorder(AbstractAudioTransform):
    """
    Audio recorder that captures audio events and combines them.
    Press a key to toggle recording on/off.
    """

    def __init__(
        self,
        max_recording_time: float = 120.0,
    ):
        """
        Initialize KeyRecorder.

        Args:
            max_recording_time: Maximum recording time in seconds
        """
        self.max_recording_time = max_recording_time

        self._audio_buffer = []
        self._is_recording = False
        self._recording_start_time = 0
        self._sample_rate = None  # Will be updated from incoming audio
        self._channels = None  # Will be set from first event

        self._audio_observable = None
        self._subscription = None
        self._output_subject = Subject()  # For record-time passthrough
        self._recording_subject = ReplaySubject(1)  # For full completed recordings

        # Start a thread to monitor for input
        self._running = True
        self._input_thread = threading.Thread(target=self._input_monitor, daemon=True)
        self._input_thread.start()

        logger.info("Started audio recorder (press any key to start/stop recording)")

    def consume_audio(self, audio_observable: Observable) -> "KeyRecorder":
        """
        Subscribe to an audio observable and record on key press.

        Args:
            audio_observable: Observable emitting AudioEvent objects

        Returns:
            Self for method chaining
        """
        self._audio_observable = audio_observable

        # Subscribe to the observable
        self._subscription = audio_observable.subscribe(
            on_next=self._process_audio_event,
            on_error=self._handle_error,
            on_completed=self._handle_completion,
        )

        return self

    def emit_audio(self) -> Observable:
        """
        Create an observable that emits audio events in real-time (pass-through).

        Returns:
            Observable emitting AudioEvent objects in real-time
        """
        return self._output_subject

    def emit_recording(self) -> Observable:
        """
        Create an observable that emits combined audio recordings when recording stops.

        Returns:
            Observable emitting AudioEvent objects with complete recordings
        """
        return self._recording_subject

    def stop(self):
        """Stop recording and clean up resources."""
        logger.info("Stopping audio recorder")

        if self._subscription:
            self._subscription.dispose()
            self._subscription = None

        # Stop input monitoring thread
        self._running = False
        if self._input_thread.is_alive():
            self._input_thread.join(1.0)

    def _input_monitor(self):
        """Monitor for key presses to toggle recording."""
        print("Press Enter to start/stop recording...")

        while self._running:
            # Check if there's input available
            if select.select([sys.stdin], [], [], 0.1)[0]:
                if self._is_recording:
                    self._stop_recording()
                else:
                    self._start_recording()

            # Sleep a bit to reduce CPU usage
            time.sleep(0.1)

    def _start_recording(self):
        """Start recording audio."""
        self._is_recording = True
        self._recording_start_time = time.time()
        self._audio_buffer = []
        logger.info("Recording started")
        print("Recording... (press Enter to stop)")

    def _stop_recording(self):
        """Stop recording and emit the combined audio event."""
        self._is_recording = False
        recording_duration = time.time() - self._recording_start_time

        logger.info(f"Recording stopped after {recording_duration:.2f} seconds")
        print(f"Recording complete: {recording_duration:.2f} seconds")

        # Combine all audio events into one
        if len(self._audio_buffer) > 0:
            combined_audio = self._combine_audio_events(self._audio_buffer)
            self._recording_subject.on_next(combined_audio)
        else:
            logger.warning("No audio was recorded")

    def _process_audio_event(self, audio_event):
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

    def _combine_audio_events(self, audio_events: List[AudioEvent]) -> AudioEvent:
        """Combine multiple audio events into a single event."""
        if not audio_events:
            return None

        first_event = audio_events[0]
        channels = first_event.channels
        dtype = first_event.data.dtype

        # For multichannel audio, data shape could be (samples,) or (samples, channels)
        if len(first_event.data.shape) == 1:
            # 1D audio data (mono)
            total_samples = sum(event.data.shape[0] for event in audio_events)
            combined_data = np.zeros(total_samples, dtype=dtype)

            # Copy data
            offset = 0
            for event in audio_events:
                samples = event.data.shape[0]
                combined_data[offset : offset + samples] = event.data
                offset += samples
        else:
            # Multichannel audio data (stereo or more)
            total_samples = sum(event.data.shape[0] for event in audio_events)
            combined_data = np.zeros((total_samples, channels), dtype=dtype)

            # Copy data
            offset = 0
            for event in audio_events:
                samples = event.data.shape[0]
                combined_data[offset : offset + samples] = event.data
                offset += samples

        # Create new audio event with the combined data
        return AudioEvent(
            data=combined_data,
            sample_rate=self._sample_rate,
            timestamp=audio_events[0].timestamp,
            channels=channels,
        )

    def _handle_error(self, error):
        """Handle errors from the observable."""
        logger.error(f"Error in audio observable: {error}")

    def _handle_completion(self):
        """Handle completion of the observable."""
        logger.info("Audio observable completed")
        self.stop()


if __name__ == "__main__":
    from dimos.stream.audio.node_microphone import (
        SounddeviceAudioSource,
    )
    from dimos.stream.audio.node_output import SounddeviceAudioOutput
    from dimos.stream.audio.node_volume_monitor import monitor
    from dimos.stream.audio.node_normalizer import AudioNormalizer
    from dimos.stream.audio.utils import keepalive

    import whisper

    model = whisper.load_model("small")

    # Create microphone source, recorder, and audio output
    mic = SounddeviceAudioSource()
    recorder = KeyRecorder()
    normalizer = AudioNormalizer()
    speaker = SounddeviceAudioOutput()

    # Connect the components
    normalizer.consume_audio(mic.emit_audio())
    recorder.consume_audio(normalizer.emit_audio())

    # Monitor microphone input levels (real-time pass-through)
    print("Real-time audio monitoring:")
    monitor(recorder.emit_audio())

    # Connect the recorder output to the speakers to hear recordings when completed
    playback_speaker = SounddeviceAudioOutput()
    playback_speaker.consume_audio(recorder.emit_recording())

    # Setup transcription for completed recordings if whisper is available
    def process_recording(recording):
        print("Processing recording for transcription...")
        result = model.transcribe(recording.data.flatten(), language="en")
        print("\nTranscription: " + result["text"].strip())

    # Subscribe to the recording observable
    recorder.emit_recording().subscribe(on_next=process_recording)

    # Monitor the volume of completed recordings
    print("Recording playback monitoring:")

    print("\nPress Enter to start recording, press Enter again to stop and play back.")

    keepalive()
