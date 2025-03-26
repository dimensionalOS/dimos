#!/usr/bin/env python3

import logging
import threading
import time
import numpy as np
from enum import Enum
from typing import Optional
from reactivex import Observable, Subject, disposable
import io
import soundfile as sf
from openai import OpenAI

from text.abstract import AbstractTextConsumer, AbstractTextEmitter
from audio.abstract import AbstractAudioEmitter, AudioEvent

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class Voice(str, Enum):
    """Available voices in OpenAI TTS API."""

    ALLOY = "alloy"
    ECHO = "echo"
    FABLE = "fable"
    ONYX = "onyx"
    NOVA = "nova"
    SHIMMER = "shimmer"


class OpenAITTSNode(AbstractTextConsumer, AbstractAudioEmitter, AbstractTextEmitter):
    """
    A text-to-speech node that consumes text, emits audio using OpenAI's TTS API, and passes through text.

    This node implements AbstractTextConsumer to receive text input, AbstractAudioEmitter
    to provide audio output, and AbstractTextEmitter to pass through the text being spoken,
    allowing it to be inserted into a text-to-audio pipeline with text passthrough capabilities.
    """

    def __init__(
        self,
        api_key: Optional[str] = None,
        voice: Voice = Voice.ONYX,
        model: str = "tts-1",
        buffer_size: int = 1024,
    ):
        """
        Initialize OpenAITTSNode.

        Args:
            api_key: OpenAI API key (if None, will try to use environment variable)
            voice: TTS voice to use
            model: TTS model to use
            buffer_size: Audio buffer size in samples
        """
        self.voice = voice
        self.model = model
        self.buffer_size = buffer_size

        # Initialize OpenAI client
        self.client = OpenAI(api_key=api_key)

        # Initialize state
        self.audio_subject = Subject()
        self.text_subject = Subject()
        self.subscription = None
        self.processing_thread = None
        self.is_running = True
        self.text_queue = []
        self.queue_lock = threading.Lock()

    def emit_audio(self) -> Observable:
        """
        Returns an observable that emits audio frames.

        Returns:
            Observable emitting AudioEvent objects
        """
        return self.audio_subject
        
    def emit_text(self) -> Observable:
        """
        Returns an observable that emits the text being spoken.

        Returns:
            Observable emitting text strings
        """
        return self.text_subject

    def consume_text(self, text_observable: Observable) -> "AbstractTextConsumer":
        """
        Start consuming text from the observable source.

        Args:
            text_observable: Observable source of text strings

        Returns:
            Self for method chaining
        """
        logger.info("Starting OpenAITTSNode")

        # Start the processing thread
        self.processing_thread = threading.Thread(
            target=self._process_queue, daemon=True
        )
        self.processing_thread.start()

        # Subscribe to the text observable
        self.subscription = text_observable.subscribe(
            on_next=self._queue_text,
            on_error=lambda e: logger.error(f"Error in OpenAITTSNode: {e}")
        )

        return self

    def _queue_text(self, text: str) -> None:
        """
        Add text to the processing queue and pass it through to text_subject.

        Args:
            text: The text to synthesize
        """
        if not text.strip():
            return

        with self.queue_lock:
            self.text_queue.append(text)

    def _process_queue(self) -> None:
        """Background thread to process the text queue."""
        while self.is_running:
            # Check if there's text to process
            text_to_process = None
            with self.queue_lock:
                if self.text_queue:
                    text_to_process = self.text_queue.pop(0)

            if text_to_process:
                self._synthesize_speech(text_to_process)
            else:
                # Sleep a bit to avoid busy-waiting
                time.sleep(0.1)

    def _synthesize_speech(self, text: str) -> None:
        """
        Convert text to speech using OpenAI API.

        Args:
            text: The text to synthesize
        """
        try:
            # Call OpenAI TTS API
            response = self.client.audio.speech.create(
                model=self.model, voice=self.voice.value, input=text
            )
            self.text_subject.on_next(text)

            # Convert the response to audio data
            audio_data = io.BytesIO(response.content)

            # Read with soundfile
            with sf.SoundFile(audio_data, "r") as sound_file:
                # Get the sample rate from the file
                actual_sample_rate = sound_file.samplerate
                # Read the entire file
                audio_array = sound_file.read()

            # Debug log the sample rate from the OpenAI file
            logger.debug(f"OpenAI audio sample rate: {actual_sample_rate}Hz")

            timestamp = time.time()

            # Create AudioEvent and emit it
            audio_event = AudioEvent(
                data=audio_array,
                sample_rate=24000,
                timestamp=timestamp,
                channels=1 if audio_array.ndim == 1 else audio_array.shape[1],
            )

            self.audio_subject.on_next(audio_event)

        except Exception as e:
            logger.error(f"Error synthesizing speech: {e}")

    def dispose(self) -> None:
        """Clean up resources."""
        logger.info("Disposing OpenAITTSNode")

        self.is_running = False

        if self.processing_thread and self.processing_thread.is_alive():
            self.processing_thread.join(timeout=5.0)

        if self.subscription:
            self.subscription.dispose()
            self.subscription = None
            
        # Complete the subjects
        self.audio_subject.on_completed()
        self.text_subject.on_completed()


if __name__ == "__main__":
    import time
    from utils import keepalive
    from reactivex import Subject
    from audio.node_output import SounddeviceAudioOutput
    from text.node_stdout import TextPrinterNode

    # Create a simple text subject that we can push values to
    text_subject = Subject()

    tts_node = OpenAITTSNode(voice=Voice.ALLOY)
    tts_node.consume_text(text_subject)

    # Create and connect an audio output node - explicitly set sample rate
    audio_output = SounddeviceAudioOutput(sample_rate=24000)
    audio_output.consume_audio(tts_node.emit_audio())

    stdout = TextPrinterNode(prefix="[Spoken Text] ")

    stdout.consume_text(tts_node.emit_text())

    # Emit some test messages
    test_messages = [
        "Hello!",
        "This is a test of the OpenAI text to speech system.",
    ]

    print("Starting OpenAI TTS test...")
    print("-" * 60)

    for i, message in enumerate(test_messages):
        text_subject.on_next(message)

    keepalive()
