# Copyright 2026 Dimensional Inc.
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

"""Lazy adapter registry shared by the hardware adapter families.

Adapter packages declare factories in dependency-free ``_registry.py``
manifests mapping name to a ``"module:attr"`` import path::

    ADAPTER_FACTORIES = {
        "xarm": "dimos.hardware.manipulators.xarm.adapter:XArmAdapter",
    }

Adapter modules are imported only when ``create()`` is first called for a
name, so a missing vendor SDK fails loudly at creation instead of silently
dropping the adapter from the registry at import time.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping
import importlib
import os
from typing import Any, ClassVar, Generic, TypeVar, cast

AdapterT = TypeVar("AdapterT")


class LazyAdapterRegistry(Generic[AdapterT]):
    """Name-to-factory registry resolving ``"module:attr"`` paths on first use."""

    kind: ClassVar[str]
    """Human-readable adapter kind used in error messages."""

    manifest_roots: ClassVar[tuple[tuple[str, int], ...]]
    """``(package, max_depth)`` roots scanned for ``_registry.py`` manifests."""

    def __init__(self) -> None:
        self._factory_paths: dict[str, str] = {}
        self._factories: dict[str, Callable[..., AdapterT]] = {}
        self.discover()

    def discover(self) -> None:
        """Discover adapter manifests without importing adapter implementations."""
        for pkg_name, max_depth in self.manifest_roots:
            pkg = importlib.import_module(pkg_name)
            self._load_manifest(pkg_name)
            for root in pkg.__path__:
                self._scan(pkg_name, root, max_depth)

    def register_path(self, name: str, factory_path: str) -> None:
        """Register a lazy factory import path; conflicting duplicates raise."""
        if ":" not in factory_path:
            raise ValueError(f"Invalid adapter factory path: {factory_path!r}")
        key = name.lower()
        existing = self._factory_paths.get(key)
        if existing is not None and existing != factory_path:
            raise ValueError(f"Duplicate {self.kind} {key!r}: {existing!r} vs {factory_path!r}")
        self._factory_paths[key] = factory_path

    def register(self, name: str, cls: Callable[..., AdapterT]) -> None:
        """Register a factory (class or callable) directly, last-wins."""
        self._factories[name.lower()] = cls

    def create(self, name: str, **kwargs: Any) -> AdapterT:
        """Create an adapter instance by registered name.

        Raises:
            KeyError: If the name is not registered.
            ImportError: If the factory's module cannot be imported (e.g.
                the vendor SDK is not installed).
        """
        return self._resolve_factory(name)(**kwargs)

    def available(self) -> list[str]:
        """List registered adapter names; import is not attempted."""
        return sorted(self._factory_paths.keys() | self._factories.keys())

    def _resolve_factory(self, name: str) -> Callable[..., AdapterT]:
        key = name.lower()
        if key in self._factories:
            return self._factories[key]
        if key not in self._factory_paths:
            raise KeyError(f"Unknown {self.kind}: {name}. Available: {self.available()}")
        factory_path = self._factory_paths[key]
        module_name, attr = factory_path.split(":", maxsplit=1)
        try:
            module = importlib.import_module(module_name)
        except ImportError as exc:
            raise ImportError(
                f"Cannot create {self.kind} {name!r}: "
                f"importing {module_name!r} (from {factory_path!r}) failed: {exc}"
            ) from exc
        factory = cast("Callable[..., AdapterT]", getattr(module, attr))
        if not callable(factory):
            raise TypeError(f"Adapter factory {factory_path!r} is not callable")
        self._factories[key] = factory
        return factory

    def _scan(self, pkg_name: str, dir_path: str, depth: int) -> None:
        if depth <= 0:
            return
        for entry in sorted(os.listdir(dir_path)):
            entry_path = os.path.join(dir_path, entry)
            if entry.startswith(("_", ".")) or not os.path.isdir(entry_path):
                continue
            sub_pkg = f"{pkg_name}.{entry}"
            self._load_manifest(sub_pkg)
            self._scan(sub_pkg, entry_path, depth - 1)

    def _load_manifest(self, pkg_name: str) -> None:
        module_name = f"{pkg_name}._registry"
        try:
            module = importlib.import_module(module_name)
        except ModuleNotFoundError as exc:
            if exc.name == module_name:
                return
            raise
        factories = getattr(module, "ADAPTER_FACTORIES", None)
        if not isinstance(factories, Mapping):
            raise TypeError(f"{module_name} must define ADAPTER_FACTORIES")
        for name, factory_path in factories.items():
            if not isinstance(name, str) or not isinstance(factory_path, str):
                raise TypeError(f"{module_name}.ADAPTER_FACTORIES must map strings to strings")
            self.register_path(name, factory_path)
