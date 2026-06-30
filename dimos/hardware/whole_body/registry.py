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

Each adapter module exposes a ``register(registry)`` function that the
registry calls during discovery. Two roots are scanned:

* ``dimos/hardware/whole_body/`` — real-hardware adapters (Unitree DDS,
  transport-LCM bridge). Subpackages are either flat ``<kind>/adapter.py``
  or nested ``<vendor>/<robot>/adapter.py``.
* ``dimos/simulation/adapters/whole_body/`` — sim adapters
  (``g1.py``, etc.). Sim engines live under ``dimos/simulation/`` so
  their adapter glue lives there too instead of under ``hardware/``.

Usage:
    from dimos.hardware.whole_body.registry import whole_body_adapter_registry

    adapter = whole_body_adapter_registry.create("sim_mujoco_g1")
    print(whole_body_adapter_registry.available())  # ["sim_mujoco_g1", ...]
"""

from __future__ import annotations

from collections.abc import Callable
import importlib
import os
from typing import TYPE_CHECKING, Any

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.hardware.whole_body.spec import WholeBodyAdapter

logger = setup_logger()


class WholeBodyAdapterRegistry:
    """Registry for whole-body motor adapters with auto-discovery."""

    def __init__(self) -> None:
        # Factory may be a class or any other callable (e.g. functools.partial
        # binding transport_cls). Store as Callable so `register("transport_lcm",
        # partial(TransportWholeBodyAdapter, ...))` typechecks.
        self._adapters: dict[str, Callable[..., WholeBodyAdapter]] = {}

    def register(self, name: str, cls: Callable[..., WholeBodyAdapter]) -> None:
        """Register an adapter factory (class or callable)."""
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
        """Discover and register whole-body adapters from the hardware and sim roots."""
        import dimos.hardware.whole_body as hw_pkg

        self._discover_in("dimos.hardware.whole_body", hw_pkg.__path__[0], max_depth=2)

        try:
            import dimos.simulation.adapters.whole_body as sim_pkg
        except ImportError as e:
            logger.warning(f"Skipping sim whole-body adapters: {e}")
            return
        for sim_root in sim_pkg.__path__:
            self._discover_flat("dimos.simulation.adapters.whole_body", sim_root)

    def _discover_in(self, pkg_path: str, dir_path: str, *, max_depth: int) -> None:
        for entry in sorted(os.listdir(dir_path)):
            entry_path = os.path.join(dir_path, entry)
            if not os.path.isdir(entry_path) or entry.startswith(("_", ".")):
                continue
            sub_pkg_path = f"{pkg_path}.{entry}"
            adapter_module = f"{sub_pkg_path}.adapter"
            try:
                mod = importlib.import_module(adapter_module)
            except ImportError as e:
                # No adapter.py at this level — recurse one deeper if budget left.
                if max_depth > 1:
                    self._discover_in(sub_pkg_path, entry_path, max_depth=max_depth - 1)
                else:
                    logger.warning(f"Skipping whole-body adapter {entry}: {e}")
                continue
            if hasattr(mod, "register"):
                mod.register(self)

    def _discover_flat(self, pkg_path: str, dir_path: str) -> None:
        for entry in sorted(os.listdir(dir_path)):
            if not entry.endswith(".py") or entry.startswith(("_", ".")):
                continue
            try:
                mod = importlib.import_module(f"{pkg_path}.{entry[:-3]}")
            except ImportError as e:
                logger.warning(f"Skipping whole-body adapter {entry}: {e}")
                continue
            if hasattr(mod, "register"):
                mod.register(self)


whole_body_adapter_registry = WholeBodyAdapterRegistry()
whole_body_adapter_registry.discover()
