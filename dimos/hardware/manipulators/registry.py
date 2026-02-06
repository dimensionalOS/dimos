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

"""Adapter registry with auto-discovery.

Automatically discovers and registers manipulator adapters from subpackages.
Each adapter provides a `register()` function in its adapter.py module.

Usage:
    from dimos.hardware.manipulators.registry import adapter_registry

    # Create an adapter by name
    adapter = adapter_registry.create("xarm", ip="192.168.1.185", dof=6)
    adapter = adapter_registry.create("piper", can_port="can0", dof=6)
    adapter = adapter_registry.create("mock", dof=7)

    # List available adapters
    print(adapter_registry.available())  # ["mock", "piper", "xarm"]
"""

from __future__ import annotations

import importlib
import logging
import pkgutil
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.hardware.manipulators.spec import ManipulatorAdapter

logger = logging.getLogger(__name__)


class AdapterRegistry:
    """Registry for manipulator adapters with auto-discovery."""

    def __init__(self) -> None:
        self._adapters: dict[str, type[ManipulatorAdapter]] = {}

    def register(self, name: str, cls: type[ManipulatorAdapter]) -> None:
        """Register an adapter class."""
        self._adapters[name.lower()] = cls

    def create(self, name: str, **kwargs: Any) -> ManipulatorAdapter:
        """Create an adapter instance by name.

        Args:
            name: Adapter name (e.g., "xarm", "piper", "mock")
            **kwargs: Arguments passed to adapter constructor

        Returns:
            Configured adapter instance

        Raises:
            KeyError: If adapter name is not found
        """
        key = name.lower()
        if key not in self._adapters:
            raise KeyError(f"Unknown adapter: {name}. Available: {self.available()}")

        return self._adapters[key](**kwargs)

    def available(self) -> list[str]:
        """List available adapter names."""
        return sorted(self._adapters.keys())

    def discover(self) -> None:
        """Discover and register adapters from subpackages.

        Can be called multiple times to pick up newly added adapters.
        """
        import dimos.hardware.manipulators as pkg

        for _, name, ispkg in pkgutil.iter_modules(pkg.__path__):
            if not ispkg:
                continue
            try:
                module = importlib.import_module(f"dimos.hardware.manipulators.{name}.adapter")
                if hasattr(module, "register"):
                    module.register(self)
            except ImportError as e:
                logger.debug(f"Skipping adapter {name}: {e}")


adapter_registry = AdapterRegistry()
adapter_registry.discover()

__all__ = ["AdapterRegistry", "adapter_registry"]
