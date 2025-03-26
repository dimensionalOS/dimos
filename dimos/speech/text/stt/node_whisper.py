#!/usr/bin/env python3

import logging
import time
import io
import threading
import numpy as np
from typing import Optional, List, Dict, Any, Union
from reactivex import Observable, Subject, disposable
from reactivex.subject import BehaviorSubject
import soundfile as sf
import whisper

from audio.abstract import AbstractAudioConsumer, AudioEvent
from text.abstract import AbstractTextEmitter

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class WhisperSTTNode(AbstractAudioConsumer, AbstractTextEmitter):
    """
    A speech-to-text node that consumes audio and emits transcribed text using OpenAI's Whisper.

    This node will buffer audio data until it reaches a certain duration or silence is detected,
    then batch process the audio using Whisper to convert it to text.
    """

    def __init__(
        self,
        model_name: str = "tiny",
        language: Optional[str] = None,
        buffer_duration: float = 3.0,
        silence_threshold: float = 0.01,
        silence_duration: float = 0.7,
        device: Optional[str] = None,
    ):
        """
        Initialize WhisperSTTNode.

        Args:
            model_name: Whisper model name ('tiny', 'base', 'small', 'medium', 'large')
            language: Language code to use for transcription (e.g., 'en', 'fr') or None for auto-detect
            buffer_duration: Maximum duration in seconds to buffer before processing
            silence_threshold: Threshold below which audio is considered silence (0.0 to 1.0)
            silence_duration: Duration of silence in seconds to trigger processing
            device: Device to use for inference ('cpu', 'cuda', etc.)
        """
        self.model_name = model_name
        self.language = language
        self.buffer_duration = buffer_duration
        self.silence_threshold = silence_threshold
        self.silence_duration = silence_duration
        self.device = device

        print(whisper)
        # Initialize model
        logger.info(f"Loading Whisper model: {model_name}")
        self.model = whisper.load_model(model_name, device=device)

        # Initialize buffers and state
        self.reset_buffer()
        self.text_subject = Subject()
        self.subscription = None
        self.processing_lock = threading.Lock()
        self.is_processing = False
        self.sample_rate = 16000  # Whisper uses 16kHz audio

        # Status reporting
        self.status = BehaviorSubject("Ready")

    def reset_buffer(self):
        """Reset the audio buffer."""
        self.buffer = []
        self.buffer_samples = 0
        self.last_audio_time = time.time()
        self.silent_start_time = None

    def emit_text(self) -> Observable:
        """
        Returns an observable that emits transcribed text strings.

        Returns:
            Observable emitting text strings
        """
        return self.text_subject

    def consume_audio(self, audio_observable: Observable) -> "AbstractAudioConsumer":
        """
        Start consuming audio from the observable source.

        Args:
            audio_observable: Observable source of AudioEvent objects

        Returns:
            Self for method chaining
        """
        logger.info("Starting WhisperSTTNode")
        self.status.on_next("Listening")

        # Subscribe to the audio observable
        self.subscription = audio_observable.subscribe(
            on_next=self.process_audio_chunk,
            on_error=lambda e: logger.error(f"Error in WhisperSTTNode: {e}"),
            on_completed=lambda: self.on_audio_completed(),
        )

        return self

    def process_audio_chunk(self, audio_event: AudioEvent) -> None:
        """
        Process an incoming audio chunk: add to buffer and check if we should process.

        Args:
            audio_event: The audio event to process
        """
        # Convert to float32 and correct sample rate if needed
        audio_data = audio_event.to_float32().data

        # Check if this is silence
        rms = np.sqrt(np.mean(np.square(audio_data)))
        is_silent = rms < self.silence_threshold

        # Track silence for triggering processing
        current_time = time.time()
        if is_silent:
            if self.silent_start_time is None:
                self.silent_start_time = current_time
            elif (current_time - self.silent_start_time) >= self.silence_duration:
                if self.buffer_samples > 0:
                    self.process_buffer()
                return
        else:
            self.silent_start_time = None

        # Add to buffer
        self.buffer.append(audio_data)
        self.buffer_samples += len(audio_data)
        self.last_audio_time = current_time

        # Check if buffer is long enough to process
        buffer_duration = self.buffer_samples / self.sample_rate
        if buffer_duration >= self.buffer_duration:
            self.process_buffer()

    def process_buffer(self) -> None:
        """Process the current audio buffer using Whisper."""
        if not self.buffer or self.is_processing:
            return

        with self.processing_lock:
            if self.is_processing:
                return

            self.is_processing = True
            self.status.on_next("Processing")

            # Combine buffer into one array
            audio_data = np.concatenate(self.buffer)
            self.reset_buffer()

            # Process in a separate thread to not block the audio processing
            threading.Thread(
                target=self._transcribe_audio, args=(audio_data.copy(),), daemon=True
            ).start()

    def _transcribe_audio(self, audio_data: np.ndarray) -> None:
        """
        Transcribe audio data using Whisper (runs in separate thread).

        Args:
            audio_data: Audio data as numpy array
        """
        try:
            # Ensure audio is the right format for Whisper
            if audio_data.ndim > 1 and audio_data.shape[1] > 1:
                # Convert stereo to mono
                audio_data = np.mean(audio_data, axis=1)

            # Normalize audio
            if np.abs(audio_data).max() > 0:
                audio_data = audio_data / np.abs(audio_data).max()

            # Transcribe with Whisper
            transcribe_options = {}
            if self.language:
                transcribe_options["language"] = self.language

            result = self.model.transcribe(audio_data, **transcribe_options)

            # Extract transcription
            text = result["text"].strip()

            if text:
                logger.debug(f"Transcribed: {text}")
                self.text_subject.on_next(text)

        except Exception as e:
            logger.error(f"Error transcribing audio: {e}")
        finally:
            with self.processing_lock:
                self.is_processing = False
                self.status.on_next("Listening")

    def on_audio_completed(self) -> None:
        """Handle completion of the input observable."""
        logger.info("Input audio stream completed")

        # Process any remaining audio in the buffer
        if self.buffer_samples > 0:
            self.process_buffer()

        # Allow time for processing to complete
        time.sleep(1)

        # Signal completion to subscribers
        self.text_subject.on_completed()
        self.status.on_next("Completed")

    def dispose(self) -> None:
        """Clean up resources."""
        logger.info("Disposing WhisperSTTNode")
        if self.subscription:
            self.subscription.dispose()
            self.subscription = None

        # Process any remaining buffered audio
        if self.buffer_samples > 0:
            self.process_buffer()


if __name__ == "__main__":
    import time
    from audio.node_microphone import SounddeviceAudioSource
    from text.node_stdout import TextPrinterNode

    # Create a simulated audio source
    audio_source = SounddeviceAudioSource()

    # Create and connect the STT node
    stt_node = WhisperSTTNode(model_name="tiny")
    stt_node.consume_audio(audio_source.emit_audio())

    # Connect a text printer to display the results
    text_printer = TextPrinterNode(prefix="[STT] ")
    text_printer.consume_text(stt_node.emit_text())

    # Setup status reporting
    status_printer = TextPrinterNode(prefix="[Status] ")
    status_printer.consume_text(stt_node.status)

    print("Starting Whisper STT test...")
    print("Processing simulated audio for transcription")
    print("-" * 60)

    try:
        # Keep the program running
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopping Whisper STT node")
    finally:
        # Clean up
        stt_node.dispose()
