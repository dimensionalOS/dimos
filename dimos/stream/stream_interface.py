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

"""Base interface for all perception and processing streams."""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, field
from reactivex import Observable
import logging

logger = logging.getLogger(__name__)


@dataclass
class StreamConfig:
    """Configuration for a stream plugin."""

    name: str
    enabled: bool = True
    device: str = "cpu"  # "cpu", "cuda", "cuda:0", etc.
    model_path: Optional[str] = None
    parameters: Dict[str, Any] = field(default_factory=dict)
    dependencies: List[str] = field(default_factory=list)  # List of required stream names
    priority: int = 0  # Higher priority streams are initialized first

    def __post_init__(self):
        """Validate configuration after initialization."""
        if not self.name:
            raise ValueError("Stream name cannot be empty")

        # Validate device string
        valid_devices = ["cpu", "cuda"]
        if not any(self.device.startswith(d) for d in valid_devices):
            raise ValueError(f"Invalid device: {self.device}. Must be 'cpu' or 'cuda[:N]'")


class StreamInterface(ABC):
    """Abstract base class for all perception and processing streams.

    This interface defines the contract that all stream plugins must implement.
    Streams can depend on other streams and share data through the Observable pattern.
    """

    def __init__(self, config: StreamConfig):
        """Initialize the stream with configuration.

        Args:
            config: StreamConfig object containing stream configuration
        """
        self.config = config
        self.name = config.name
        self._initialized = False
        self._dependencies_met = False
        self._stream = None

    @abstractmethod
    def initialize(self, dependencies: Dict[str, "StreamInterface"] = None) -> bool:
        """Initialize the stream and any required models or resources.

        This method is called after all dependencies are available.

        Args:
            dependencies: Dictionary mapping dependency names to initialized stream instances

        Returns:
            bool: True if initialization successful, False otherwise
        """
        pass

    @abstractmethod
    def create_stream(self, input_stream: Observable) -> Observable:
        """Create the processing stream from an input video stream.

        Args:
            input_stream: Observable that emits video frames

        Returns:
            Observable: Stream that emits processed results
        """
        pass

    @abstractmethod
    def cleanup(self):
        """Clean up any resources used by the stream.

        This method is called when the stream is being shut down.
        """
        pass

    @property
    def is_initialized(self) -> bool:
        """Check if the stream is initialized and ready to use."""
        return self._initialized

    @property
    def requires_gpu(self) -> bool:
        """Check if this stream requires GPU resources."""
        return self.config.device.startswith("cuda")

    def get_dependencies(self) -> List[str]:
        """Get list of stream names this stream depends on."""
        return self.config.dependencies

    def get_info(self) -> Dict[str, Any]:
        """Get information about the stream.

        Returns:
            Dictionary containing stream information
        """
        return {
            "name": self.name,
            "enabled": self.config.enabled,
            "device": self.config.device,
            "initialized": self._initialized,
            "requires_gpu": self.requires_gpu,
            "dependencies": self.config.dependencies,
            "priority": self.config.priority,
        }

    def __repr__(self):
        return f"{self.__class__.__name__}(name='{self.name}', initialized={self._initialized})"
