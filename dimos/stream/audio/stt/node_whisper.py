from dimos.stream.audio.sound_processing.abstract import AbstractAudioTransform, AudioEvent, AbstractAudioConsumer
from dimos.stream.audio.text.abstract import AbstractTextEmitter
from typing import Dict, Any
from reactivex import Observable, create, disposable
import whisper
import logging

# Set up logger
logger = logging.getLogger(__name__)

class WhisperNode(AbstractAudioConsumer, AbstractTextEmitter):
    """
    A node that transcribes audio using OpenAI's Whisper model and emits the transcribed text.
    """

    def __init__(
        self,
        model: str = "base",
        modelopts: Dict[str, Any] = { "language": "en" , "fp16": False },
    ):
        self.audio_observable = None
        self.modelopts = modelopts
        self.model = whisper.load_model(model)

    def consume_audio(self, audio_observable: Observable) -> 'WhisperNode':
        """
        Set the audio source observable to consume.

        Args:
            audio_observable: Observable emitting AudioEvent objects

        Returns:
            Self for method chaining
        """
        self.audio_observable = audio_observable
        return self

    def emit_text(self) -> Observable:
        """
        Create an observable that emits transcribed text from audio.

        Returns:
            Observable emitting transcribed text from audio recordings
        """
        if self.audio_observable is None:
            raise ValueError("No audio source provided. Call consume_audio() first.")

        def on_subscribe(observer, scheduler):
            logger.info("Starting Whisper transcription service")

            # Subscribe to the audio source
            def on_audio_event(event: AudioEvent):
                try:
                    result = self.model.transcribe(event.data.flatten(), **self.modelopts)
                    observer.on_next(result["text"].strip())
                except Exception as e:
                    logger.error(f"Error processing audio event: {e}")
                    observer.on_error(e)
            
            # Set up subscription to audio source
            subscription = self.audio_observable.subscribe(
                on_next=on_audio_event,
                on_error=lambda e: observer.on_error(e),
                on_completed=lambda: observer.on_completed()
            )

            # Return a disposable to clean up resources
            def dispose():
                subscription.dispose()

            return disposable.Disposable(dispose)

        return create(on_subscribe)


if __name__ == "__main__":
    from dimos.stream.audio.sound_processing.node_microphone import SounddeviceAudioSource
    from dimos.stream.audio.sound_processing.node_output import SounddeviceAudioOutput
    from dimos.stream.audio.sound_processing.node_volume_monitor import monitor
    from dimos.stream.audio.sound_processing.node_normalizer import AudioNormalizer
    from dimos.stream.audio.sound_processing.node_key_recorder import KeyTriggeredAudioRecorder
    from dimos.stream.audio.text.node_stdout import TextPrinterNode
    from dimos.stream.audio.tts.node_openai import OpenAITTSNode
    from dimos.stream.audio.utils import keepalive

    # Create microphone source, recorder, and audio output
    mic = SounddeviceAudioSource()
    normalizer = AudioNormalizer()
    recorder = KeyTriggeredAudioRecorder()
    whisper_node = WhisperNode()
    output = SounddeviceAudioOutput(sample_rate=24000)

    normalizer.consume_audio(mic.emit_audio())
    recorder.consume_audio(normalizer.emit_audio())
    monitor(recorder.emit_audio())
    whisper_node.consume_audio(recorder.emit_recording())

    # Create and connect the text printer node
    text_printer = TextPrinterNode(prefix="USER: ")
    text_printer.consume_text(whisper_node.emit_text())

    tts_node = OpenAITTSNode()
    tts_node.consume_text(whisper_node.emit_text())

    output.consume_audio(tts_node.emit_audio())

    keepalive()
