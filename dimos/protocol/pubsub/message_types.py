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

"""Resolve registered generated message types without importing legacy packages."""

from dimos_cdrgen.registry import message_types


def resolve_message_type(name: str) -> type | None:
    """Resolve a canonical ROS name or a package.Type shorthand."""
    registered = message_types()
    if name in registered:
        return registered[name]
    if "/" not in name and name.count(".") == 1:
        package, message = name.split(".")
        return registered.get(f"{package}/msg/{message}")
    return None
