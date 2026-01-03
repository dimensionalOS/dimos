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

"""Convenience rendering functions for module introspection."""

from collections.abc import Callable
from typing import Any

from dimos.core.introspection.module import ansi
from dimos.core.introspection.module.info import extract_module_info


def render_module_io(
    name: str,
    inputs: dict[str, Any],
    outputs: dict[str, Any],
    rpcs: dict[str, Callable],  # type: ignore[type-arg]
    color: bool = True,
) -> str:
    """Render module IO diagram using the default (ANSI) renderer.

    Args:
        name: Module class name.
        inputs: Dict of input stream name -> stream object or formatted string.
        outputs: Dict of output stream name -> stream object or formatted string.
        rpcs: Dict of RPC method name -> callable.
        color: Whether to include ANSI color codes.

    Returns:
        ASCII diagram showing module inputs, outputs, RPCs, and skills.
    """
    info = extract_module_info(name, inputs, outputs, rpcs)
    return ansi.render(info, color=color)
