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

"""Backend registry with auto-discovery.

Automatically discovers and registers manipulator backends from subpackages.
Each backend provides a `register()` function in its backend.py module.

Usage:
    from dimos.hardware.manipulators.registry import backend_registry

    # Create a backend by name
    backend = backend_registry.create("xarm", ip="192.168.1.185", dof=6)
    backend = backend_registry.create("piper", can_port="can0", dof=6)
    backend = backend_registry.create("mock", dof=7)

    # List available backends
    print(backend_registry.available())  # ["mock", "piper", "xarm"]
"""

from __future__ import annotations

import importlib
import logging
import pkgutil
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.hardware.manipulators.spec import ManipulatorBackend

logger = logging.getLogger(__name__)


class BackendRegistry:
    """Registry for manipulator backends with auto-discovery."""

    def __init__(self) -> None:
        self._backends: dict[str, type[ManipulatorBackend]] = {}
        self._discovered = False

    def register(self, name: str, cls: type[ManipulatorBackend]) -> None:
        """Register a backend class."""
        self._backends[name.lower()] = cls

    def create(self, name: str, **kwargs: Any) -> ManipulatorBackend:
        """Create a backend instance by name.

        Args:
            name: Backend name (e.g., "xarm", "piper", "mock")
            **kwargs: Arguments passed to backend constructor

        Returns:
            Configured backend instance

        Raises:
            KeyError: If backend name is not found
        """
        if not self._discovered:
            self._discover()

        key = name.lower()
        if key not in self._backends:
            raise KeyError(f"Unknown backend: {name}. Available: {self.available()}")

        return self._backends[key](**kwargs)

    def available(self) -> list[str]:
        """List available backend names."""
        if not self._discovered:
            self._discover()
        return sorted(self._backends.keys())

    def _discover(self) -> None:
        """Auto-discover backends in subpackages."""
        import dimos.hardware.manipulators as pkg

        for _, name, ispkg in pkgutil.iter_modules(pkg.__path__):
            if not ispkg:
                continue
            try:
                module = importlib.import_module(f"dimos.hardware.manipulators.{name}.backend")
                if hasattr(module, "register"):
                    module.register(self)
            except ImportError as e:
                logger.debug(f"Skipping backend {name}: {e}")

        self._discovered = True


backend_registry = BackendRegistry()

__all__ = ["BackendRegistry", "backend_registry"]
