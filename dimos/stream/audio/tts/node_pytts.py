#!/usr/bin/env python3
from reactivex import Observable, Subject
import pyttsx3

from dimos.stream.audio.text.abstract import AbstractTextTransform

from dimos.utils.logging_config import setup_logger

logger = setup_logger(__name__)


class PyTTSNode(AbstractTextTransform):
    """
    A transform node that passes through text but also speaks it using pyttsx3.

    This node implements AbstractTextTransform, so it both consumes and emits
    text observables, allowing it to be inserted into a text processing pipeline.
    """

    def __init__(self, rate: int = 200, volume: float = 1.0):
        """
        Initialize PyTTSNode.

        Args:
            rate: Speech rate (words per minute)
            volume: Volume level (0.0 to 1.0)
        """
        self.engine = pyttsx3.init()
        self.engine.setProperty("rate", rate)
        self.engine.setProperty("volume", volume)

        self.text_subject = Subject()
        self.subscription = None

    def emit_text(self) -> Observable:
        """
        Returns an observable that emits text strings passed through this node.

        Returns:
            Observable emitting text strings
        """
        return self.text_subject

    def consume_text(self, text_observable: Observable) -> "AbstractTextTransform":
        """
        Start processing text from the observable source.

        Args:
            text_observable: Observable source of text strings

        Returns:
            Self for method chaining
        """
        logger.info("Starting PyTTSNode")

        # Subscribe to the text observable
        self.subscription = text_observable.subscribe(
            on_next=self.process_text,
            on_error=lambda e: logger.error(f"Error in PyTTSNode: {e}"),
            on_completed=lambda: self.on_text_completed(),
        )

        return self

    def process_text(self, text: str) -> None:
        """
        Process the input text: speak it and pass it through.

        Args:
            text: The text to process
        """
        # Speak the text
        logger.debug(f"Speaking: {text}")
        self.engine.say(text)
        self.engine.runAndWait()

        # Pass the text through to any subscribers
        self.text_subject.on_next(text)

    def on_text_completed(self) -> None:
        """Handle completion of the input observable."""
        logger.info("Input text stream completed")
        # Signal completion to subscribers
        self.text_subject.on_completed()

    def dispose(self) -> None:
        """Clean up resources."""
        logger.info("Disposing PyTTSNode")
        if self.subscription:
            self.subscription.dispose()
            self.subscription = None


if __name__ == "__main__":
    import time

    # Create a simple text subject that we can push values to
    text_subject = Subject()

    # Create and connect the TTS node
    tts_node = PyTTSNode(rate=150)
    tts_node.consume_text(text_subject)

    # Optional: Connect to the output to demonstrate it's a transform
    from dimos.stream.audio.text.node_stdout import TextPrinterNode

    printer = TextPrinterNode(prefix="[Spoken Text] ")
    printer.consume_text(tts_node.emit_text())

    # Emit some test messages
    test_messages = [
        "Hello, world!",
        "This is a test of the text-to-speech node",
        "Using the AbstractTextTransform interface",
        "It passes text through while also speaking it",
    ]

    print("Starting test...")
    print("-" * 60)

    try:
        # Emit each message with a delay
        for message in test_messages:
            text_subject.on_next(message)
            time.sleep(2)  # Longer delay to let speech finish

    except KeyboardInterrupt:
        print("\nStopping TTS node")
    finally:
        # Clean up
        tts_node.dispose()
        text_subject.on_completed()
