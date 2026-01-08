# Copyright 2025-2026 Dimensional Inc.
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

"""Stream processor module providing modular, pluggable video stream processing.

This module defines abstract interfaces and concrete implementations for processing
video streams without hard-coding model dependencies. Processors can be dynamically
loaded and configured at runtime.
"""

from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Union, Callable
from reactivex import Observable
from reactivex import operators as ops
import logging

logger = logging.getLogger(__name__)


class StreamProcessor(ABC):
    """Abstract base class for stream processors.

    Stream processors transform video frames or other data streams by applying
    various processing operations (detection, tracking, segmentation, etc.).
    They are designed to be modular and pluggable.
    """

    def __init__(self, name: str, config: Optional[Dict[str, Any]] = None):
        """Initialize the stream processor.

        Args:
            name: Unique name for this processor
            config: Optional configuration dictionary
        """
        self.name = name
        self.config = config or {}
        self._is_initialized = False

    @abstractmethod
    def initialize(self) -> bool:
        """Initialize the processor with its configuration.

        Returns:
            bool: True if initialization succeeded, False otherwise
        """
        pass

    @abstractmethod
    def process_frame(
        self, frame: Any, metadata: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Process a single frame.

        Args:
            frame: Input frame (typically numpy array)
            metadata: Optional metadata associated with the frame

        Returns:
            Dict containing processed results and visualization
        """
        pass

    @abstractmethod
    def cleanup(self) -> None:
        """Clean up resources used by the processor."""
        pass

    def create_stream(self, input_stream: Observable) -> Observable:
        """Create a processed stream from an input stream.

        Args:
            input_stream: Input observable stream

        Returns:
            Observable: Processed stream
        """
        if not self._is_initialized:
            if not self.initialize():
                raise RuntimeError(f"Failed to initialize processor {self.name}")
            self._is_initialized = True

        def process_with_error_handling(frame):
            try:
                return self.process_frame(frame)
            except Exception as e:
                logger.error(f"Error in processor {self.name}: {e}")
                return {"frame": frame, "viz_frame": frame, "error": str(e), "processor": self.name}

        return input_stream.pipe(ops.map(process_with_error_handling))


class PassthroughProcessor(StreamProcessor):
    """A passthrough processor that doesn't modify frames.

    Useful for testing or as a placeholder when no processing is needed.
    """

    def initialize(self) -> bool:
        """Initialize the passthrough processor."""
        return True

    def process_frame(
        self, frame: Any, metadata: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """Pass through the frame without modification."""
        return {"frame": frame, "viz_frame": frame, "targets": [], "metadata": metadata or {}}

    def cleanup(self) -> None:
        """No cleanup needed for passthrough processor."""
        pass


class StreamProcessorRegistry:
    """Registry for managing stream processors.

    Allows dynamic registration and creation of stream processors without
    hard-coding dependencies.
    """

    def __init__(self):
        self._processors: Dict[str, Callable[..., StreamProcessor]] = {}
        self._register_builtin_processors()

    def _register_builtin_processors(self):
        """Register built-in processors."""
        self.register("passthrough", PassthroughProcessor)

    def register(self, name: str, processor_class: Callable[..., StreamProcessor]) -> None:
        """Register a processor class.

        Args:
            name: Unique name for the processor type
            processor_class: Class or factory function that creates processor instances
        """
        self._processors[name] = processor_class
        logger.info(f"Registered processor: {name}")

    def create_processor(
        self, processor_type: str, name: str, config: Optional[Dict[str, Any]] = None
    ) -> StreamProcessor:
        """Create a processor instance.

        Args:
            processor_type: Type of processor to create
            name: Unique name for this processor instance
            config: Optional configuration dictionary

        Returns:
            StreamProcessor: Created processor instance

        Raises:
            ValueError: If processor type is not registered
        """
        if processor_type not in self._processors:
            raise ValueError(
                f"Unknown processor type: {processor_type}. Available: {list(self._processors.keys())}"
            )

        processor_class = self._processors[processor_type]
        return processor_class(name=name, config=config)

    def list_processors(self) -> List[str]:
        """List all registered processor types.

        Returns:
            List of registered processor type names
        """
        return list(self._processors.keys())


class StreamProcessorPipeline:
    """Pipeline for chaining multiple stream processors.

    Allows sequential processing of streams through multiple processors.
    """

    def __init__(self, name: str):
        """Initialize the pipeline.

        Args:
            name: Name for this pipeline
        """
        self.name = name
        self.processors: List[StreamProcessor] = []

    def add_processor(self, processor: StreamProcessor) -> None:
        """Add a processor to the pipeline.

        Args:
            processor: Processor to add
        """
        self.processors.append(processor)
        logger.info(f"Added processor {processor.name} to pipeline {self.name}")

    def create_stream(self, input_stream: Observable) -> Observable:
        """Create a processed stream by chaining all processors.

        Args:
            input_stream: Input observable stream

        Returns:
            Observable: Processed stream
        """
        current_stream = input_stream

        for processor in self.processors:
            current_stream = processor.create_stream(current_stream)

        return current_stream

    def cleanup(self) -> None:
        """Clean up all processors in the pipeline."""
        for processor in self.processors:
            try:
                processor.cleanup()
            except Exception as e:
                logger.error(f"Error cleaning up processor {processor.name}: {e}")


# Global registry instance
_global_registry = StreamProcessorRegistry()


def get_processor_registry() -> StreamProcessorRegistry:
    """Get the global processor registry.

    Returns:
        StreamProcessorRegistry: Global registry instance
    """
    return _global_registry


def register_processor(name: str, processor_class: Callable[..., StreamProcessor]) -> None:
    """Register a processor class globally.

    Args:
        name: Unique name for the processor type
        processor_class: Class or factory function that creates processor instances
    """
    _global_registry.register(name, processor_class)


def create_processor(
    processor_type: str, name: str, config: Optional[Dict[str, Any]] = None
) -> StreamProcessor:
    """Create a processor instance using the global registry.

    Args:
        processor_type: Type of processor to create
        name: Unique name for this processor instance
        config: Optional configuration dictionary

    Returns:
        StreamProcessor: Created processor instance
    """
    return _global_registry.create_processor(processor_type, name, config)
