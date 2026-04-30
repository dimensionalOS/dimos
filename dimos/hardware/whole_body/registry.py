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

"""WholeBodyAdapter registry with auto-discovery.

Mirrors the TwistBaseAdapterRegistry pattern: each subpackage provides a
``register(registry)`` function in its ``adapter.py`` module.

Usage:
    from dimos.hardware.whole_body.registry import whole_body_adapter_registry

    adapter = whole_body_adapter_registry.create("unitree_go2")
    print(whole_body_adapter_registry.available())  # ["unitree_go2"]
"""

from __future__ import annotations

import importlib
import logging
import os
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.hardware.whole_body.spec import WholeBodyAdapter

logger = logging.getLogger(__name__)


class WholeBodyAdapterRegistry:
    """Registry for whole-body motor adapters with auto-discovery."""

    def __init__(self) -> None:
        self._adapters: dict[str, type[WholeBodyAdapter]] = {}

    def register(self, name: str, cls: type[WholeBodyAdapter]) -> None:
        """Register an adapter class."""
        self._adapters[name.lower()] = cls

    def create(self, name: str, **kwargs: Any) -> WholeBodyAdapter:
        """Create an adapter instance by name."""
        key = name.lower()
        if key not in self._adapters:
            raise KeyError(f"Unknown whole-body adapter: {name}. Available: {self.available()}")
        return self._adapters[key](**kwargs)

    def available(self) -> list[str]:
        """List available adapter names."""
        return sorted(self._adapters.keys())

    def discover(self) -> None:
        """Discover and register adapters from subpackages.

        Mirrors ``dimos.hardware.drive_trains.registry``'s walk pattern: raw
        filesystem listing + ``os.path.isdir`` check. We do NOT use
        ``pkgutil.iter_modules`` because dimos uses PEP 420 namespace packages
        (no ``__init__.py``) and ``iter_modules`` doesn't reliably enumerate
        subpackage directories under namespace packages.

        Each direct subpackage of ``dimos.hardware.whole_body`` is expected
        to expose ``adapter.py`` with a ``register(registry)`` function.
        """
        import dimos.hardware.whole_body as pkg

        pkg_dir = pkg.__path__[0]
        for entry in sorted(os.listdir(pkg_dir)):
            entry_path = os.path.join(pkg_dir, entry)
            if not os.path.isdir(entry_path) or entry.startswith(("_", ".")):
                continue
            try:
                mod = importlib.import_module(f"dimos.hardware.whole_body.{entry}.adapter")
                if hasattr(mod, "register"):
                    mod.register(self)
            except ImportError as e:
                logger.warning(f"Skipping whole-body adapter {entry}: {e}")


whole_body_adapter_registry = WholeBodyAdapterRegistry()
whole_body_adapter_registry.discover()

__all__ = ["WholeBodyAdapterRegistry", "whole_body_adapter_registry"]
