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

"""TwistBase adapter registry with lazy manifest discovery.

Adapter subpackages declare factories in ``_registry.py`` manifests
(see ``dimos.hardware.adapter_registry``).

Usage:
    from dimos.hardware.drive_trains.registry import twist_base_adapter_registry

    adapter = twist_base_adapter_registry.create("mock_twist_base", dof=3)
    print(twist_base_adapter_registry.available())  # ["flowbase", "mock_twist_base", ...]
"""

from __future__ import annotations

from dimos.hardware.adapter_registry import LazyAdapterRegistry
from dimos.hardware.drive_trains.spec import TwistBaseAdapter


class TwistBaseAdapterRegistry(LazyAdapterRegistry[TwistBaseAdapter]):
    """Registry for twist base adapters."""

    kind = "twist base adapter"
    manifest_roots = (("dimos.hardware.drive_trains", 1),)


twist_base_adapter_registry = TwistBaseAdapterRegistry()
