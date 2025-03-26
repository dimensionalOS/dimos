#!/usr/bin/env python3
import logging
from typing import Optional
from reactivex import Observable, disposable

from text.abstract import AbstractTextConsumer

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TextPrinterNode(AbstractTextConsumer):
    """
    A node that subscribes to a text observable and prints the text.
    """

    def __init__(self, prefix: str = "", suffix: str = "", end: str = "\n"):
        """
        Initialize TextPrinterNode.

        Args:
            prefix: Text to print before each line
            suffix: Text to print after each line
            end: String to append at the end of each line
        """
        self.prefix = prefix
        self.suffix = suffix
        self.end = end
        self.subscription = None

    def print_text(self, text: str) -> None:
        """
        Print the text with prefix and suffix.

        Args:
            text: The text to print
        """
        print(f"{self.prefix}{text}{self.suffix}", end=self.end, flush=True)

    def consume_text(self, text_observable: Observable) -> 'AbstractTextConsumer':
        """
        Start processing text from the observable source.

        Args:
            text_observable: Observable source of text strings

        Returns:
            Self for method chaining
        """
        logger.info("Starting text printer")
        
        # Subscribe to the text observable
        self.subscription = text_observable.subscribe(
            on_next=self.print_text,
            on_error=lambda e: logger.error(f"Error: {e}"),
            on_completed=lambda: logger.info("Text printer completed")
        )
        
        return self


if __name__ == "__main__":
    import time
    from reactivex import Subject
    
    # Create a simple text subject that we can push values to
    text_subject = Subject()
    
    # Create and connect the text printer
    text_printer = TextPrinterNode(prefix="Text: ")
    text_printer.consume_text(text_subject)
    
    # Emit some test messages
    test_messages = [
        "Hello, world!",
        "This is a test of the text printer",
        "Using the new AbstractTextConsumer interface",
        "Press Ctrl+C to exit"
    ]
    
    print("Starting test...")
    print("-" * 60)
    
    # Emit each message with a delay
    try:
        for message in test_messages:
            text_subject.on_next(message)
            time.sleep(0.1)
            
        # Keep the program running
        while True:
            text_subject.on_next(f"Current time: {time.strftime('%H:%M:%S')}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nStopping text printer")
    finally:
        # Clean up
        if text_printer.subscription:
            text_printer.subscription.dispose()
        text_subject.on_completed()
