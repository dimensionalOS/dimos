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

"""Graphviz DOT renderer for module IO diagrams."""

import hashlib
import re

from dimos.core.introspection.module.info import ModuleInfo  # type: ignore[import-untyped]
from dimos.utils.cli import theme


def _color_for_string(colors: list[str], s: str) -> str:
    """Get a consistent color for a string based on its hash."""
    h = int(hashlib.md5(s.encode()).hexdigest(), 16)
    return colors[h % len(colors)]


def _sanitize_id(s: str) -> str:
    """Sanitize a string to be a valid graphviz node ID."""
    return re.sub(r"[^a-zA-Z0-9_]", "_", s)


# Colors for type nodes (same as dot2 for consistency)
TYPE_COLORS = [
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
]

# Colors for RPCs/Skills
RPC_COLOR = "#7986CB"  # indigo
SKILL_COLOR = "#4ECDC4"  # teal


def render(info: ModuleInfo) -> str:
    """Render module info as a DOT graph.

    Shows the module as a central node with input streams as nodes
    pointing in and output streams as nodes pointing out.

    Args:
        info: ModuleInfo structure to render.

    Returns:
        DOT format string.
    """
    lines = [
        "digraph module {",
        "    bgcolor=transparent;",
        "    rankdir=LR;",
        "    compound=true;",
        "    splines=true;",
        f'    node [shape=box, style=filled, fillcolor="{theme.BACKGROUND}", fontcolor="{theme.FOREGROUND}", color="{theme.BLUE}", fontname=fixed, fontsize=12, margin="0.1,0.1"];',
        "    edge [fontname=fixed, fontsize=10, penwidth=1];",
        "",
    ]

    # Module node (central, larger)
    module_id = _sanitize_id(info.name)
    lines.append(f'    {module_id} [label="{info.name}", width=2, height=0.8];')
    lines.append("")

    # Input stream nodes (on the left)
    if info.inputs:
        lines.append("    // Input streams")
        lines.append("    subgraph cluster_inputs {")
        lines.append('        label="";')
        lines.append("        style=invis;")
        lines.append('        rank="same";')
        for stream in info.inputs:
            label = f"{stream.name}:{stream.type_name}"
            color = _color_for_string(TYPE_COLORS, label)
            node_id = _sanitize_id(f"in_{stream.name}")
            lines.append(
                f'        {node_id} [label="{label}", shape=note, style=filled, '
                f'fillcolor="{color}35", color="{color}", '
                f'width=0, height=0, margin="0.1,0.05", fontsize=10];'
            )
        lines.append("    }")
        lines.append("")

    # Output stream nodes (on the right)
    if info.outputs:
        lines.append("    // Output streams")
        lines.append("    subgraph cluster_outputs {")
        lines.append('        label="";')
        lines.append("        style=invis;")
        lines.append('        rank="same";')
        for stream in info.outputs:
            label = f"{stream.name}:{stream.type_name}"
            color = _color_for_string(TYPE_COLORS, label)
            node_id = _sanitize_id(f"out_{stream.name}")
            lines.append(
                f'        {node_id} [label="{label}", shape=note, style=filled, '
                f'fillcolor="{color}35", color="{color}", '
                f'width=0, height=0, margin="0.1,0.05", fontsize=10];'
            )
        lines.append("    }")
        lines.append("")

    # RPC nodes (in subgraph)
    if info.rpcs:
        lines.append("    // RPCs")
        lines.append("    subgraph cluster_rpcs {")
        lines.append('        label="RPCs";')
        lines.append("        labeljust=l;")
        lines.append("        fontname=fixed;")
        lines.append("        fontsize=14;")
        lines.append(f'        fontcolor="{theme.FOREGROUND}";')
        lines.append('        style="filled,dashed";')
        lines.append(f'        color="{RPC_COLOR}";')
        lines.append("        penwidth=1;")
        lines.append(f'        fillcolor="{RPC_COLOR}10";')
        for rpc in info.rpcs:
            params = ", ".join(
                f"{p.name}: {p.type_name}" if p.type_name else p.name for p in rpc.params
            )
            ret = f" -> {rpc.return_type}" if rpc.return_type else ""
            label = f"{rpc.name}({params}){ret}"
            node_id = _sanitize_id(f"rpc_{rpc.name}")
            lines.append(
                f'        {node_id} [label="{label}", shape=cds, style=filled, '
                f'fillcolor="{RPC_COLOR}35", color="{RPC_COLOR}", '
                f'width=0, height=0, margin="0.1,0.05", fontsize=9];'
            )
        lines.append("    }")
        lines.append("")

    # Skill nodes (in subgraph)
    if info.skills:
        lines.append("    // Skills")
        lines.append("    subgraph cluster_skills {")
        lines.append('        label="Skills";')
        lines.append("        labeljust=l;")
        lines.append("        fontname=fixed;")
        lines.append("        fontsize=14;")
        lines.append(f'        fontcolor="{theme.FOREGROUND}";')
        lines.append('        style="filled,dashed";')
        lines.append(f'        color="{SKILL_COLOR}";')
        lines.append("        penwidth=1;")
        lines.append(f'        fillcolor="{SKILL_COLOR}20";')
        for skill in info.skills:
            parts = [skill.name]
            if skill.stream:
                parts.append(f"stream={skill.stream}")
            if skill.reducer:
                parts.append(f"reducer={skill.reducer}")
            label = " ".join(parts)
            node_id = _sanitize_id(f"skill_{skill.name}")
            lines.append(
                f'        {node_id} [label="{label}", shape=cds, style=filled, '
                f'fillcolor="{SKILL_COLOR}35", color="{SKILL_COLOR}", '
                f'width=0, height=0, margin="0.1,0.05", fontsize=9];'
            )
        lines.append("    }")
        lines.append("")

    # Edges: inputs -> module
    lines.append("    // Edges")
    for stream in info.inputs:
        label = f"{stream.name}:{stream.type_name}"
        color = _color_for_string(TYPE_COLORS, label)
        node_id = _sanitize_id(f"in_{stream.name}")
        lines.append(f'    {node_id} -> {module_id} [color="{color}"];')

    # Edges: module -> outputs
    for stream in info.outputs:
        label = f"{stream.name}:{stream.type_name}"
        color = _color_for_string(TYPE_COLORS, label)
        node_id = _sanitize_id(f"out_{stream.name}")
        lines.append(f'    {module_id} -> {node_id} [color="{color}"];')

    # Edge: module -> RPCs cluster (dashed, no arrow)
    if info.rpcs:
        first_rpc_id = _sanitize_id(f"rpc_{info.rpcs[0].name}")
        lines.append(
            f"    {module_id} -> {first_rpc_id} [lhead=cluster_rpcs, style=filled, weight=3"
            f'color="{RPC_COLOR}", arrowhead=none];'
        )

    # Edge: module -> Skills cluster (dashed, no arrow)
    if info.skills:
        first_skill_id = _sanitize_id(f"skill_{info.skills[0].name}")
        lines.append(
            f"    {module_id} -> {first_skill_id} [lhead=cluster_skills, style=filled, weight=3"
            f'color="{SKILL_COLOR}", arrowhead=none];'
        )

    lines.append("}")
    return "\n".join(lines)


def render_svg(info: ModuleInfo, output_path: str) -> None:
    """Generate an SVG file from ModuleInfo using graphviz.

    Args:
        info: ModuleInfo structure to render.
        output_path: Path to write the SVG file.
    """
    import subprocess

    dot_code = render(info)
    result = subprocess.run(
        ["dot", "-Tsvg", "-o", output_path],
        input=dot_code,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"graphviz failed: {result.stderr}")
