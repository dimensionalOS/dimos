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

"""Blueprint visualization using Graphviz DOT format."""

from collections import defaultdict
import hashlib

from dimos.core.blueprints import ModuleBlueprintSet
from dimos.core.module import Module
from dimos.utils.cli import theme


def color_for_string(colors: list[str], s: str) -> str:
    """Get a consistent color for a string based on its hash."""
    h = int(hashlib.md5(s.encode()).hexdigest(), 16)
    return colors[h % len(colors)]


def to_dot(blueprint_set: ModuleBlueprintSet) -> str:
    """Generate a DOT graph from a ModuleBlueprintSet.

    Returns a string in DOT format showing modules as nodes and
    connections between them as edges labeled with name:type.
    odules are grouped into subgraphs by their package directory.
    """
    # Collect all outputs: (name, type) -> module
    outputs: dict[tuple[str, type], list[type[Module]]] = defaultdict(list)
    # Collect all inputs: (name, type) -> module
    inputs: dict[tuple[str, type], list[type[Module]]] = defaultdict(list)
    # Module name -> module class (for getting package info)
    module_classes: dict[str, type[Module]] = {}

    for bp in blueprint_set.blueprints:
        module_classes[bp.module.__name__] = bp.module
        for conn in bp.connections:
            # Apply remapping
            remapped_name = blueprint_set.remapping_map.get((bp.module, conn.name), conn.name)
            key = (remapped_name, conn.type)
            if conn.direction == "out":
                outputs[key].append(bp.module)
            else:
                inputs[key].append(bp.module)

    # Generate edges: for each (name, type), connect outputs to inputs
    edges: set[tuple[str, str, str]] = set()

    # Connections to ignore (too noisy/common)
    ignored_connections = {("odom", "PoseStamped")}

    # Modules to ignore entirely
    ignored_modules = {"WebsocketVisModule", "UtilizationModule", "FoxgloveBridge"}

    for key, out_modules in outputs.items():
        name, type_ = key
        if key not in inputs:
            continue
        type_name = type_.__name__
        if (name, type_name) in ignored_connections:
            continue
        for out_mod in out_modules:
            if out_mod.__name__ in ignored_modules:
                continue
            for in_mod in inputs[key]:
                if in_mod.__name__ in ignored_modules:
                    continue
                label = f"{name}:{type_name}"
                edges.add((out_mod.__name__, in_mod.__name__, label))

    # Group modules by package
    def get_group(mod_class: type[Module]) -> str:
        module_path = mod_class.__module__
        # Extract meaningful group from path like "dimos.navigation.planner"
        parts = module_path.split(".")
        if len(parts) >= 2 and parts[0] == "dimos":
            return parts[1]  # e.g., "navigation", "perception", "robot"
        return "other"

    by_group: dict[str, list[str]] = defaultdict(list)
    for mod_name, mod_class in module_classes.items():
        if mod_name in ignored_modules:
            continue
        group = get_group(mod_class)
        by_group[group].append(mod_name)

    # Colors for group borders (bright, distinct, good on dark backgrounds)
    group_colors = [
        "#5C9FF0",  # blue
        "#FFB74D",  # orange
        "#81C784",  # green
        "#BA68C8",  # purple
        "#4ECDC4",  # teal
        "#FF6B6B",  # coral
        "#FFE66D",  # yellow
        "#7986CB",  # indigo
        "#F06292",  # pink
        "#4DB6AC",  # teal green
        "#9575CD",  # deep purple
        "#AED581",  # lime
        "#64B5F6",  # light blue
        "#FF8A65",  # deep orange
        "#AA96DA",  # lavender
    ]

    # Colors for edges (bright, distinct, good on dark backgrounds)
    edge_colors = [
        "#FF6B6B",  # coral red
        "#4ECDC4",  # teal
        "#FFE66D",  # yellow
        "#95E1D3",  # mint
        "#F38181",  # salmon
        "#AA96DA",  # lavender
        "#81C784",  # green
        "#64B5F6",  # light blue
        "#FFB74D",  # orange
        "#BA68C8",  # purple
        "#4DD0E1",  # cyan
        "#AED581",  # lime
        "#FF8A65",  # deep orange
        "#7986CB",  # indigo
        "#F06292",  # pink
        "#A1887F",  # brown
        "#90A4AE",  # blue grey
        "#DCE775",  # lime yellow
        "#4DB6AC",  # teal green
        "#9575CD",  # deep purple
        "#E57373",  # light red
        "#81D4FA",  # sky blue
        "#C5E1A5",  # light green
        "#FFCC80",  # light orange
        "#B39DDB",  # light purple
        "#80DEEA",  # light cyan
        "#FFAB91",  # peach
        "#CE93D8",  # light violet
        "#80CBC4",  # light teal
        "#FFF59D",  # light yellow
    ]

    # Build label -> color mapping (consistent based on name:type)
    all_labels = sorted(set(label for _, _, label in edges))
    label_color_map = {label: color_for_string(edge_colors, label) for label in all_labels}

    # Build DOT output
    lines = [
        "digraph modules {",
        "    bgcolor=transparent;",
        "    rankdir=LR;",
        "    splines=true;",
        "    remincross=true;",
        "    nodesep=1.5;",  # horizontal spacing between nodes
        "    ranksep=1.5;",  # vertical spacing between ranks
        f'    node [shape=box, style=filled, fillcolor="{theme.BACKGROUND}", fontcolor="{theme.FOREGROUND}", color="{theme.BLUE}", fontname=fixed, fontsize=12, width=2, height=0.8, margin="0.2,0.1"];',
        "    edge [fontname=fixed, fontsize=10];",
        "",
    ]

    # Add subgraphs for each group with different colors
    sorted_groups = sorted(by_group.keys())
    for group in sorted_groups:
        mods = sorted(by_group[group])
        color = color_for_string(group_colors, group)
        lines.append(f"    subgraph cluster_{group} {{")
        lines.append(f'        label="{group}";')
        lines.append("         labeljust=r;")  # right-justify label
        lines.append("         fontname=fixed;")
        lines.append("         fontsize=14;")
        lines.append(f'        fontcolor="{theme.FOREGROUND}";')
        lines.append('         style="filled,dashed";')
        lines.append(f'        color="{color}";')
        lines.append("         penwidth=1;")
        lines.append(f'        fillcolor="{color}10";')  # 10 = ~6% opacity
        for mod in mods:
            lines.append(f"        {mod};")
        lines.append("    }")
        lines.append("")

    # Add edges with label-based coloring (name:type determines color)
    # Use sametail/samehead to group edges of the same label at the same port
    for src, dst, label in sorted(edges):
        color = label_color_map.get(label, theme.WHITE)
        # Create a port name from the label (sanitize for graphviz)
        port = label.replace(":", "_").replace(" ", "_")
        lines.append(
            f'    {src} -> {dst} [xlabel="{label}", color="{color}", fontcolor="{color}", forcelabels=false, sametail="{port}", samehead="{port}"];'
        )

    lines.append("}")
    return "\n".join(lines)


def to_svg(blueprint_set: ModuleBlueprintSet, output_path: str) -> None:
    """Generate an SVG file from a ModuleBlueprintSet using graphviz.

    Args:
        blueprint_set: The blueprint set to visualize.
        output_path: Path to write the SVG file.
    """
    import subprocess

    dot_code = to_dot(blueprint_set)
    result = subprocess.run(
        ["dot", "-Tsvg", "-o", output_path],
        input=dot_code,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"graphviz failed: {result.stderr}")
