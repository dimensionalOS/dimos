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

"""Unified SVG rendering for modules and blueprints."""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.core.blueprints import Blueprint
    from dimos.core.introspection.blueprint.dot import LayoutAlgo
    from dimos.core.introspection.module.info import ModuleInfo


def to_svg(
    target: ModuleInfo | Blueprint,
    output_path: str,
    *,
    layout: set[LayoutAlgo] | None = None,
) -> None:
    """Render a module or blueprint to SVG.

    Dispatches to the appropriate renderer based on input type:
    - ModuleInfo -> module/dot.render_svg
    - Blueprint -> blueprint/dot.render_svg

    Args:
        target: Either a ModuleInfo (single module) or Blueprint (blueprint graph).
        output_path: Path to write the SVG file.
        layout: Layout algorithms (only used for blueprints).
    """
    # Avoid circular imports by importing here
    from dimos.core.blueprints import Blueprint
    from dimos.core.introspection.module.info import ModuleInfo

    if isinstance(target, ModuleInfo):
        from dimos.core.introspection.module import dot as module_dot

        module_dot.render_svg(target, output_path)
    elif isinstance(target, Blueprint):
        from dimos.core.introspection.blueprint import dot as blueprint_dot

        blueprint_dot.render_svg(target, output_path, layout=layout)
    else:
        raise TypeError(f"Expected ModuleInfo or Blueprint, got {type(target).__name__}")
