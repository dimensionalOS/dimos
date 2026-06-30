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

"""Adapter registry with lazy auto-discovery.

Automatically discovers manipulator adapters from lightweight subpackage
``__registry__.py`` manifests. Adapter implementation modules are imported
only when their adapter key is selected via :meth:`AdapterRegistry.create`.

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

from collections.abc import Callable, Mapping
import importlib
from pathlib import Path
from typing import TYPE_CHECKING, cast

if TYPE_CHECKING:
    from dimos.hardware.manipulators.spec import ManipulatorAdapter

AdapterFactory = Callable[..., "ManipulatorAdapter"]


class AdapterRegistry:
    """Registry for manipulator adapters with lazy auto-discovery."""

    def __init__(self) -> None:
        self._adapter_paths: dict[str, str] = {}
        self._adapters: dict[str, AdapterFactory] = {}

    def register(self, name: str, cls: AdapterFactory) -> None:
        """Register an already-imported adapter factory."""
        self._adapters[name.lower()] = cls

    def register_path(self, name: str, factory_path: str) -> None:
        """Register a lazy adapter factory import path."""
        if ":" not in factory_path:
            raise ValueError(f"Invalid adapter factory path: {factory_path!r}")
        module_name, attr = factory_path.split(":", maxsplit=1)
        if not module_name or not attr:
            raise ValueError(f"Invalid adapter factory path: {factory_path!r}")

        key = name.lower()
        existing = self._adapter_paths.get(key)
        if existing is not None and existing != factory_path:
            raise ValueError(f"Duplicate adapter {key!r}: {existing!r} vs {factory_path!r}")
        self._adapter_paths[key] = factory_path

    def create(self, name: str, **kwargs: object) -> ManipulatorAdapter:
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
        if key not in self._adapters and key not in self._adapter_paths:
            raise KeyError(f"Unknown adapter: {name}. Available: {self.available()}")

        return self._resolve_adapter(key)(**kwargs)

    def available(self) -> list[str]:
        """List available adapter names."""
        return sorted(set(self._adapter_paths) | set(self._adapters))

    def discover(self) -> None:
        """Discover and register adapter manifests from subpackages.

        Scans for subdirectories containing a ``__registry__.py`` manifest.
        Can be called multiple times to pick up newly added adapters.
        """
        import dimos.hardware.manipulators as pkg

        for root in pkg.__path__:
            for child in sorted(Path(root).iterdir()):
                if not child.is_dir() or child.name.startswith(("_", ".")):
                    continue
                if not (child / "__registry__.py").exists():
                    continue

                module_name = f"dimos.hardware.manipulators.{child.name}.__registry__"
                module = importlib.import_module(module_name)
                adapter_factories_obj = getattr(module, "ADAPTER_FACTORIES", None)
                if not isinstance(adapter_factories_obj, Mapping):
                    raise TypeError(f"{module_name} must define ADAPTER_FACTORIES")
                adapter_factories = cast("Mapping[object, object]", adapter_factories_obj)
                for name, factory_path in adapter_factories.items():
                    if not isinstance(name, str) or not isinstance(factory_path, str):
                        raise TypeError(
                            f"{module_name}.ADAPTER_FACTORIES must map strings to strings"
                        )
                    self.register_path(name, factory_path)

    def _resolve_adapter(self, key: str) -> AdapterFactory:
        if key in self._adapters:
            return self._adapters[key]
        factory_path = self._adapter_paths[key]
        module_name, attr = factory_path.split(":", maxsplit=1)
        try:
            module = importlib.import_module(module_name)
        except ModuleNotFoundError as exc:
            if exc.name is not None and module_name.startswith(exc.name):
                raise ImportError(
                    f"Adapter {key!r} is registered to missing module {module_name!r}"
                ) from exc
            raise
        try:
            factory = cast("AdapterFactory", getattr(module, attr))
        except AttributeError as exc:
            raise ImportError(
                f"Adapter {key!r} is registered to missing factory {factory_path!r}"
            ) from exc
        if not callable(factory):
            raise TypeError(f"Adapter factory {factory_path!r} is not callable")
        self._adapters[key] = factory
        return factory


adapter_registry = AdapterRegistry()
adapter_registry.discover()

__all__ = ["AdapterFactory", "AdapterRegistry", "adapter_registry"]
