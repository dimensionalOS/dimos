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

"""Registry for managing stream plugins."""

from typing import Dict, Type, List, Optional, Set
from .stream_interface import StreamInterface, StreamConfig
import logging
from collections import defaultdict

logger = logging.getLogger(__name__)


class StreamRegistry:
    """Registry for managing and initializing stream plugins.

    This class handles:
    - Registration of stream plugin classes
    - Configuration management
    - Dependency resolution
    - Initialization ordering
    """

    def __init__(self):
        """Initialize the stream registry."""
        self._stream_classes: Dict[str, Type[StreamInterface]] = {}
        self._stream_instances: Dict[str, StreamInterface] = {}
        self._configurations: Dict[str, StreamConfig] = {}

    def register_stream_class(self, name: str, stream_class: Type[StreamInterface]):
        """Register a stream plugin class.

        Args:
            name: Name identifier for the stream
            stream_class: Class that implements StreamInterface
        """
        if not issubclass(stream_class, StreamInterface):
            raise TypeError(f"{stream_class} must inherit from StreamInterface")

        if name in self._stream_classes:
            logger.warning(f"Overwriting existing stream class: {name}")

        self._stream_classes[name] = stream_class
        logger.info(f"Registered stream class: {name}")

    def configure_stream(self, config: StreamConfig):
        """Configure a stream for initialization.

        Args:
            config: StreamConfig object with stream configuration
        """
        if config.name not in self._stream_classes:
            raise ValueError(f"Stream class not registered: {config.name}")

        self._configurations[config.name] = config
        logger.info(f"Configured stream: {config.name}")

    def initialize_streams(self, force_cpu: bool = False) -> Dict[str, StreamInterface]:
        """Initialize all configured streams respecting dependencies.

        Args:
            force_cpu: If True, force all streams to use CPU even if configured for GPU

        Returns:
            Dictionary mapping stream names to initialized instances
        """
        # Filter enabled streams
        enabled_configs = {
            name: config for name, config in self._configurations.items() if config.enabled
        }

        if not enabled_configs:
            logger.info("No streams enabled for initialization")
            return {}

        # Override device if force_cpu is set
        if force_cpu:
            for config in enabled_configs.values():
                config.device = "cpu"
                logger.info(f"Forcing {config.name} to use CPU")

        # Perform topological sort based on dependencies
        sorted_names = self._topological_sort(enabled_configs)

        # Initialize streams in order
        for name in sorted_names:
            if name in self._stream_instances:
                continue  # Already initialized

            config = enabled_configs[name]
            stream_class = self._stream_classes[name]

            # Gather dependencies
            dependencies = {}
            for dep_name in config.dependencies:
                if dep_name not in self._stream_instances:
                    logger.error(f"Dependency {dep_name} not initialized for {name}")
                    continue
                dependencies[dep_name] = self._stream_instances[dep_name]

            # Create and initialize stream
            try:
                stream = stream_class(config)
                if stream.initialize(dependencies):
                    self._stream_instances[name] = stream
                    logger.info(f"Successfully initialized stream: {name}")
                else:
                    logger.error(f"Failed to initialize stream: {name}")
            except Exception as e:
                logger.error(f"Error initializing stream {name}: {e}")

        return self._stream_instances

    def _topological_sort(self, configs: Dict[str, StreamConfig]) -> List[str]:
        """Perform topological sort on streams based on dependencies.

        Args:
            configs: Dictionary of stream configurations

        Returns:
            List of stream names in initialization order
        """
        # Build adjacency list
        graph = defaultdict(list)
        in_degree = defaultdict(int)

        for name, config in configs.items():
            in_degree[name]  # Ensure all nodes are in in_degree
            for dep in config.dependencies:
                if dep in configs:  # Only consider enabled dependencies
                    graph[dep].append(name)
                    in_degree[name] += 1

        # Sort by priority first, then by name for stable ordering
        queue = sorted(
            [name for name, degree in in_degree.items() if degree == 0],
            key=lambda x: (-configs[x].priority, x),
        )

        result = []
        while queue:
            current = queue.pop(0)
            result.append(current)

            # Reduce in-degree for neighbors
            for neighbor in graph[current]:
                in_degree[neighbor] -= 1
                if in_degree[neighbor] == 0:
                    queue.append(neighbor)

            # Re-sort queue by priority
            queue.sort(key=lambda x: (-configs[x].priority, x))

        # Check for cycles
        if len(result) != len(configs):
            unprocessed = set(configs.keys()) - set(result)
            raise ValueError(f"Circular dependency detected involving: {unprocessed}")

        return result

    def get_stream(self, name: str) -> Optional[StreamInterface]:
        """Get an initialized stream instance by name.

        Args:
            name: Name of the stream

        Returns:
            StreamInterface instance or None if not found/initialized
        """
        return self._stream_instances.get(name)

    def get_all_streams(self) -> Dict[str, StreamInterface]:
        """Get all initialized stream instances.

        Returns:
            Dictionary mapping names to stream instances
        """
        return self._stream_instances.copy()

    def cleanup_all(self):
        """Clean up all initialized streams."""
        for name, stream in self._stream_instances.items():
            try:
                stream.cleanup()
                logger.info(f"Cleaned up stream: {name}")
            except Exception as e:
                logger.error(f"Error cleaning up stream {name}: {e}")

        self._stream_instances.clear()

    def list_available_streams(self) -> List[str]:
        """List all registered stream classes.

        Returns:
            List of registered stream names
        """
        return list(self._stream_classes.keys())

    def list_initialized_streams(self) -> List[str]:
        """List all initialized streams.

        Returns:
            List of initialized stream names
        """
        return list(self._stream_instances.keys())


# Global registry instance
stream_registry = StreamRegistry()
