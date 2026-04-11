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

Usage::

    from dimos.hardware.whole_body.registry import whole_body_adapter_registry

    adapter = whole_body_adapter_registry.create("r1pro_whole_body")
    print(whole_body_adapter_registry.available())  # ["r1pro_whole_body"]
"""

from __future__ import annotations

import importlib
import logging
import pkgutil
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.hardware.whole_body.spec import WholeBodyAdapter

logger = logging.getLogger(__name__)


class WholeBodyAdapterRegistry:
    """Registry for whole-body motor adapters with auto-discovery."""

    def __init__(self) -> None:
        self._adapters: dict[str, type] = {}

    def register(self, name: str, cls: type) -> None:
        """Register an adapter class."""
        self._adapters[name.lower()] = cls

    def create(self, name: str, **kwargs: Any) -> Any:
        """Create an adapter instance by name."""
        key = name.lower()
        if key not in self._adapters:
            raise KeyError(
                f"Unknown whole-body adapter: {name!r}. Available: {self.available()}"
            )
        return self._adapters[key](**kwargs)

    def available(self) -> list[str]:
        """List available adapter names."""
        return sorted(self._adapters.keys())

    def discover(self) -> None:
        """Discover and register adapters from subpackages.

        Scans ``dimos.hardware.whole_body.*.*`` for ``adapter.py`` modules
        with a ``register(registry)`` function.
        """
        import dimos.hardware.whole_body as pkg

        for _, name, ispkg in pkgutil.iter_modules(pkg.__path__):
            if not ispkg:
                continue
            # Walk one level deeper: dimos.hardware.whole_body.r1pro.adapter
            try:
                sub = importlib.import_module(f"dimos.hardware.whole_body.{name}")
                for _, sub_name, sub_ispkg in pkgutil.iter_modules(sub.__path__):
                    if not sub_ispkg:
                        continue
                    try:
                        mod = importlib.import_module(
                            f"dimos.hardware.whole_body.{name}.{sub_name}.adapter"
                        )
                        if hasattr(mod, "register"):
                            mod.register(self)
                    except ImportError as e:
                        logger.debug("Skipping whole-body adapter %s.%s: %s", name, sub_name, e)
            except ImportError as e:
                logger.debug("Skipping whole-body package %s: %s", name, e)
            # Also check direct adapter.py at dimos.hardware.whole_body.{name}.adapter
            try:
                mod = importlib.import_module(f"dimos.hardware.whole_body.{name}.adapter")
                if hasattr(mod, "register"):
                    mod.register(self)
            except ImportError:
                pass


whole_body_adapter_registry = WholeBodyAdapterRegistry()
whole_body_adapter_registry.discover()

__all__ = ["WholeBodyAdapterRegistry", "whole_body_adapter_registry"]
