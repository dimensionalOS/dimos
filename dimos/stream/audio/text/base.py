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

from abc import ABC, abstractmethod

from reactivex import Observable


class AbstractTextEmitter(ABC):
    """Base class for components that emit audio."""

    @abstractmethod
    def emit_text(self) -> Observable:  # type: ignore[type-arg]
        """Create an observable that emits audio frames.

        Returns:
            Observable emitting audio frames
        """
        pass


class AbstractTextConsumer(ABC):
    """Base class for components that consume audio."""

    @abstractmethod
    def consume_text(self, text_observable: Observable) -> "AbstractTextConsumer":  # type: ignore[type-arg]
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
