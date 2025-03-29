from abc import ABC, abstractmethod
from reactivex import Observable


class AbstractTextEmitter(ABC):
    """Base class for components that emit audio."""

    @abstractmethod
    def emit_text(self) -> Observable:
        """Create an observable that emits audio frames.

        Returns:
            Observable emitting audio frames
        """
        pass


class AbstractTextConsumer(ABC):
    """Base class for components that consume audio."""

    @abstractmethod
    def consume_text(self, text_observable: Observable) -> "AbstractTextConsumer":
        """Set the audio observable to consume.

        Args:
            audio_observable: Observable emitting audio frames

        Returns:
            Self for method chaining
        """
        pass


class AbstractTextTransform(AbstractTextConsumer, AbstractTextEmitter):
    """Base class for components that both consume and emit audio.

    This represents a transform in an audio processing pipeline.
    """

    pass
