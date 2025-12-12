# Copyright 2025 Dimensional Inc.
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

"""Blueprint introspection and rendering.

Renderers:
    - dot: Graphviz DOT format (direct edges between modules)
    - dot2: Hub-style DOT format (type nodes as intermediate hubs)
"""

from dimos.core.introspection.blueprint import dot, dot2
from dimos.core.introspection.blueprint.dot2 import LayoutAlgo

__all__ = ["LayoutAlgo", "dot", "dot2", "to_dot", "to_svg"]


def to_dot(blueprint_set: "ModuleBlueprintSet") -> str:
    """Generate DOT graph from a blueprint set.

    Convenience function that uses the dot renderer.
    """
    return dot.render(blueprint_set)


def to_svg(blueprint_set: "ModuleBlueprintSet", output_path: str) -> None:
    """Generate SVG file from a blueprint set.

    Convenience function that uses the dot renderer.
    """
    dot.render_svg(blueprint_set, output_path)
