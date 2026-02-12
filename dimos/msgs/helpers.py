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

from __future__ import annotations

from functools import lru_cache
import importlib
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.msgs import DimosMsg


@lru_cache(maxsize=256)
def resolve_msg_type(type_name: str) -> type[DimosMsg] | None:
    """Resolve a message type name to its class.

    Args:
        type_name: Type name in format "module.ClassName" (e.g., "geometry_msgs.Vector3")

    Returns:
        The message class or None if not found.
    """
    try:
        module_name, class_name = type_name.rsplit(".", 1)
    except ValueError:
        return None

    # Try different import paths
    import_paths = [
        f"dimos.msgs.{module_name}",
        f"dimos_lcm.{module_name}",
    ]

    for path in import_paths:
        try:
            module = importlib.import_module(path)
            return getattr(module, class_name)  # type: ignore[no-any-return]
        except (ImportError, AttributeError):
            continue

    return None
