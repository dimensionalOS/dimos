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

"""WholeBodyAdapter registry with lazy manifest discovery.

Adapter packages declare factories in ``_registry.py`` manifests
(see ``dimos.hardware.adapter_registry``). The hardware root is scanned:

* ``dimos/hardware/whole_body/`` — direct hardware and protocol adapters.
  Subpackages are either flat ``<kind>/`` or
  nested ``<vendor>/<robot>/``.

Usage:
    from dimos.hardware.whole_body.registry import whole_body_adapter_registry

    adapter = whole_body_adapter_registry.create("openarm_damiao")
    print(whole_body_adapter_registry.available())
"""

from __future__ import annotations

from dimos.hardware.adapter_registry import LazyAdapterRegistry
from dimos.hardware.whole_body.spec import WholeBodyAdapter


class WholeBodyAdapterRegistry(LazyAdapterRegistry[WholeBodyAdapter]):
    """Registry for whole-body motor adapters."""

    kind = "whole-body adapter"
    manifest_roots = (("dimos.hardware.whole_body", 2),)


whole_body_adapter_registry = WholeBodyAdapterRegistry()
