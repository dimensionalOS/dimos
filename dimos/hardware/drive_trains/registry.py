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

"""TwistBase adapter registry with auto-discovery.

Automatically discovers and registers twist base adapters from subpackages.
Each adapter provides a `register()` function in its adapter.py module.

Usage:
    from dimos.hardware.drive_trains.registry import twist_base_adapter_registry

    # Create an adapter by name
    adapter = twist_base_adapter_registry.create("mock_twist_base", dof=3)
    adapter = twist_base_adapter_registry.create("flowbase", dof=3, address="172.6.2.20:11323")

    # List available adapters
    print(twist_base_adapter_registry.available())  # ["flowbase", "mock_twist_base"]
"""

from __future__ import annotations

import importlib
import logging
import pkgutil
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.hardware.drive_trains.spec import TwistBaseAdapter

logger = logging.getLogger(__name__)


class TwistBaseAdapterRegistry:
    """Registry for twist base adapters with auto-discovery."""

    def __init__(self) -> None:
        self._adapters: dict[str, type[TwistBaseAdapter]] = {}

    def register(self, name: str, cls: type[TwistBaseAdapter]) -> None:
        """Register an adapter class."""
        self._adapters[name.lower()] = cls

    def create(self, name: str, **kwargs: Any) -> TwistBaseAdapter:
        """Create an adapter instance by name.

        Args:
            name: Adapter name (e.g., "mock_twist_base", "flowbase")
            **kwargs: Arguments passed to adapter constructor

        Returns:
            Configured adapter instance

        Raises:
            KeyError: If adapter name is not found
        """
        key = name.lower()
        if key not in self._adapters:
            raise KeyError(f"Unknown twist base adapter: {name}. Available: {self.available()}")

        return self._adapters[key](**kwargs)

    def available(self) -> list[str]:
        """List available adapter names."""
        return sorted(self._adapters.keys())

    def discover(self) -> None:
        """Discover and register adapters from subpackages.

        Can be called multiple times to pick up newly added adapters.
        """
        import dimos.hardware.drive_trains as pkg

        for _, name, ispkg in pkgutil.iter_modules(pkg.__path__):
            if not ispkg:
                continue
            try:
                module = importlib.import_module(f"dimos.hardware.drive_trains.{name}.adapter")
                if hasattr(module, "register"):
                    module.register(self)
            except ImportError as e:
                logger.debug(f"Skipping twist base adapter {name}: {e}")


twist_base_adapter_registry = TwistBaseAdapterRegistry()
twist_base_adapter_registry.discover()

__all__ = ["TwistBaseAdapterRegistry", "twist_base_adapter_registry"]
