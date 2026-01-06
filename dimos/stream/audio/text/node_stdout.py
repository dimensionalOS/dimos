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

from reactivex import Observable

from dimos.stream.audio.text.base import AbstractTextConsumer
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TextPrinterNode(AbstractTextConsumer):
    """
    A node that subscribes to a text observable and prints the text.
    """

    def __init__(self, prefix: str = "", suffix: str = "", end: str = "\n") -> None:
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

    def consume_text(self, text_observable: Observable) -> "AbstractTextConsumer":  # type: ignore[type-arg]
        """
        Start processing text from the observable source.

        Args:
            text_observable: Observable source of text strings

        Returns:
            Self for method chaining
        """
        logger.info("Starting text printer")

        # Subscribe to the text observable
        self.subscription = text_observable.subscribe(  # type: ignore[assignment]
            on_next=self.print_text,
            on_error=lambda e: logger.error(f"Error: {e}"),
            on_completed=lambda: logger.info("Text printer completed"),
        )

        return self


if __name__ == "__main__":
    import time

    from reactivex import Subject

    # Create a simple text subject that we can push values to
    text_subject = Subject()  # type: ignore[var-annotated]

    # Create and connect the text printer
    text_printer = TextPrinterNode(prefix="Text: ")
    text_printer.consume_text(text_subject)

    # Emit some test messages
    test_messages = [
        "Hello, world!",
        "This is a test of the text printer",
        "Using the new AbstractTextConsumer interface",
        "Press Ctrl+C to exit",
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
