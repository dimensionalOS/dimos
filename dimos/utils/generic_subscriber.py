#!/usr/bin/env python3

import threading
import logging
from typing import Optional, Any
from reactivex import Observable
from reactivex.disposable import Disposable

logger = logging.getLogger(__name__)


class GenericSubscriber:
    """Subscribes to an RxPy Observable stream and stores the latest message."""

    def __init__(self, stream: Observable):
        """Initialize the subscriber and subscribe to the stream.

        Args:
            stream: The RxPy Observable stream to subscribe to.
        """
        self.latest_message: Optional[Any] = None
        self._lock = threading.Lock()
        self._subscription: Optional[Disposable] = None
        self._stream_completed = threading.Event()
        self._stream_error: Optional[Exception] = None

        if stream is not None:
            try:
                self._subscription = stream.subscribe(
                    on_next=self._on_next,
                    on_error=self._on_error,
                    on_completed=self._on_completed
                )
                logger.debug(f"Subscribed to stream {stream}")
            except Exception as e:
                logger.error(f"Error subscribing to stream {stream}: {e}")
                self._stream_error = e # Store error if subscription fails immediately
        else:
            logger.warning("Initialized GenericSubscriber with a None stream.")

    def _on_next(self, message: Any):
        """Callback for receiving a new message."""
        with self._lock:
            self.latest_message = message
            # logger.debug("Received new message") # Can be noisy

    def _on_error(self, error: Exception):
        """Callback for stream error."""
        logger.error(f"Stream error: {error}")
        with self._lock:
            self._stream_error = error
        self._stream_completed.set() # Signal completion/error

    def _on_completed(self):
        """Callback for stream completion."""
        logger.info("Stream completed.")
        self._stream_completed.set()

    def get_data(self) -> Optional[Any]:
        """Get the latest message received from the stream.

        Returns:
            The latest message, or None if no message has been received yet.
        """
        with self._lock:
            # Optionally check for errors if needed by the caller
            # if self._stream_error:
            #    logger.warning("Attempting to get message after stream error.")
            return self.latest_message

    def has_error(self) -> bool:
        """Check if the stream encountered an error."""
        with self._lock:
            return self._stream_error is not None

    def is_completed(self) -> bool:
        """Check if the stream has completed or encountered an error."""
        return self._stream_completed.is_set()

    def dispose(self):
        """Dispose of the subscription to stop receiving messages."""
        if self._subscription is not None:
            try:
                self._subscription.dispose()
                logger.debug("Subscription disposed.")
                self._subscription = None
            except Exception as e:
                logger.error(f"Error disposing subscription: {e}")
        self._stream_completed.set() # Ensure completed flag is set on manual dispose

    def __del__(self):
        """Ensure cleanup on object deletion."""
        self.dispose()
