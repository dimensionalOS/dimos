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

"""Manipulator adapter registry with lazy manifest discovery.

Adapter subpackages declare factories in ``_registry.py`` manifests
(see ``dimos.hardware.adapter_registry``).

Usage:
    from dimos.hardware.manipulators.registry import adapter_registry

    adapter = adapter_registry.create("xarm", ip="192.168.1.185", dof=6)
    print(adapter_registry.available())  # ["a750", "mock", ..., "xarm"]
"""

from __future__ import annotations

from dimos.hardware.adapter_registry import LazyAdapterRegistry
from dimos.hardware.manipulators.spec import ManipulatorAdapter


class AdapterRegistry(LazyAdapterRegistry[ManipulatorAdapter]):
    """Registry for manipulator adapters."""

    kind = "adapter"
    manifest_roots = (("dimos.hardware.manipulators", 1),)


adapter_registry = AdapterRegistry()
