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

"""Graphviz DOT renderer for Blueprint stream and RPC relationships.

This renderer creates intermediate "type nodes" for data flow, making it clearer
when one output fans out to multiple consumers:

    ModuleA --> [name:Type] --> ModuleB
                            --> ModuleC

Resolved ModuleRef dependencies use RPC contract nodes with the declared Spec
methods between the caller and provider modules.
"""

from collections import defaultdict
from dataclasses import dataclass
from enum import Enum, auto
import json
from urllib.parse import quote

from dimos.cli import theme
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import resolve_module_refs
from dimos.core.introspection.utils import (
    GROUP_COLORS,
    RPC_COLOR,
    TYPE_COLORS,
    color_for_string,
    render_dot_svg,
)
from dimos.core.module import ModuleBase, is_module_type
from dimos.spec.utils import get_protocol_method_signatures, is_spec


class LayoutAlgo(Enum):
    """Layout algorithms for controlling graph structure."""

    STACK_CLUSTERS = auto()  # Stack clusters vertically (invisible edges between clusters)
    STACK_NODES = auto()  # Stack nodes within clusters vertically
    FDP = auto()  # Use fdp (force-directed) layout engine instead of dot


# Connections to ignore (too noisy/common)
DEFAULT_IGNORED_CONNECTIONS = {("odom", "PoseStamped")}

DEFAULT_IGNORED_MODULES = {
    "WebsocketVisModule",
}


@dataclass(frozen=True)
class RpcConnection:
    """A resolved ModuleRef from an RPC caller to its provider."""

    caller: str
    provider: str
    ref_name: str
    contract_name: str
    methods: tuple[str, ...]


def module_node_id(name: str) -> str:
    """Keep module IDs distinct through both DOT and Graphviz plain-text escaping."""
    return f"module:{quote(name)}"


def _dot_label(text: str) -> str:
    """DOT preserves literal tabs, whereas Graphviz displays the JSON tab escape as 't'."""
    parts = [json.dumps(part, ensure_ascii=False)[1:-1] for part in text.split("\t")]
    return '"' + "\t".join(parts) + '"'


def _rpc_methods(contract: type) -> tuple[str, ...]:
    if is_spec(contract):
        return tuple(sorted(get_protocol_method_signatures(contract)))
    if is_module_type(contract):
        return tuple(sorted(contract.rpcs))
    return ()


def _resolve_rpc_connections(blueprint: Blueprint) -> list[RpcConnection]:
    resolved = resolve_module_refs(blueprint)
    connections: list[RpcConnection] = []

    for atom in blueprint.active_blueprints:
        for module_ref in atom.module_refs:
            provider_name = resolved.get((atom.name, module_ref.name))
            if provider_name is None:
                continue

            connections.append(
                RpcConnection(
                    caller=atom.name,
                    provider=provider_name,
                    ref_name=module_ref.name,
                    contract_name=module_ref.spec.__name__,
                    methods=_rpc_methods(module_ref.spec),
                )
            )

    return connections


def render(
    blueprint_set: Blueprint,
    *,
    layout: set[LayoutAlgo] | None = None,
    ignored_streams: set[tuple[str, str]] | None = None,
    ignored_modules: set[str] | None = None,
    include_rpc: bool = True,
) -> str:
    """Generate a stream and RPC relationship graph from a Blueprint.

    This creates intermediate "type nodes" that represent data channels,
    connecting producers to consumers through a central hub node. Resolved
    ModuleRefs become RPC contract nodes between callers and providers.

    Args:
        blueprint_set: The blueprint set to visualize.
        layout: Set of layout algorithms to apply. Default is none (let graphviz decide).
        ignored_streams: Set of (name, type_name) tuples to ignore.
        ignored_modules: Set of module names to ignore.
        include_rpc: Whether to include RPC contracts and relationships.

    Returns:
        A DOT graph containing modules, stream channels, and RPC contracts.
    """
    if layout is None:
        layout = set()
    if ignored_streams is None:
        ignored_streams = DEFAULT_IGNORED_CONNECTIONS
    if ignored_modules is None:
        ignored_modules = DEFAULT_IGNORED_MODULES

    rpc_connections = _resolve_rpc_connections(blueprint_set) if include_rpc else []

    atoms = {
        atom.name: atom
        for atom in blueprint_set.active_blueprints
        if atom.module.__name__ not in ignored_modules
    }
    node_ids = {name: json.dumps(module_node_id(name), ensure_ascii=False) for name in atoms}
    producers: dict[tuple[str, type], list[str]] = defaultdict(list)
    consumers: dict[tuple[str, type], list[str]] = defaultdict(list)
    for bp in atoms.values():
        for conn in bp.streams:
            name = blueprint_set.remapping_map.get((bp.name, conn.name), conn.name)
            if not isinstance(name, str):
                continue
            key = (name, conn.type)
            if conn.can_publish:
                producers[key].append(bp.name)
            if conn.can_subscribe:
                consumers[key].append(bp.name)

    channels = sorted(
        producers.keys() & consumers.keys(),
        key=lambda key: (key[0], key[1].__module__, key[1].__qualname__),
    )

    # Group modules by package
    def get_group(mod_class: type[ModuleBase]) -> str:
        module_path = mod_class.__module__
        parts = module_path.split(".")
        if len(parts) >= 2 and parts[0] == "dimos":
            return parts[1]
        return "other"

    by_group: dict[str, list[str]] = defaultdict(list)
    for name, atom in atoms.items():
        by_group[get_group(atom.module)].append(name)

    # Build DOT output
    lines = [
        "digraph modules {",
        f'    bgcolor="{theme.BACKGROUND}";',
        "    rankdir=LR;",
        # "    nodesep=1;",  # horizontal spacing between nodes
        # "    ranksep=1.5;",  # vertical spacing between ranks
        "    splines=true;",
        f'    node [shape=box, style=filled, fillcolor="{theme.BACKGROUND}", fontcolor="{theme.FOREGROUND}", color="{theme.BLUE}", fontname=fixed, fontsize=12, margin="0.1,0.1"];',
        "    edge [fontname=fixed, fontsize=10];",
        "",
    ]

    # Add subgraphs for each module group
    sorted_groups = sorted(by_group.keys())
    for group in sorted_groups:
        mods = sorted(by_group[group])
        color = color_for_string(GROUP_COLORS, group)
        lines.append(f"    subgraph cluster_{group} {{")
        lines.append(f'        label="{group}";')
        lines.append("         labeljust=r;")
        lines.append("         fontname=fixed;")
        lines.append("         fontsize=14;")
        lines.append(f'        fontcolor="{theme.FOREGROUND}";')
        lines.append('         style="filled,dashed";')
        lines.append(f'        color="{color}";')
        lines.append("         penwidth=1;")
        lines.append(f'        fillcolor="{color}10";')
        for mod in mods:
            atom = atoms[mod]
            label = atom.module.__name__
            if atom.instance_name is not None:
                label = f"{mod}\n{label}"
            lines.append(f"        {node_ids[mod]} [label={_dot_label(label)}];")
        # Stack nodes vertically within cluster
        if LayoutAlgo.STACK_NODES in layout and len(mods) > 1:
            for i in range(len(mods) - 1):
                lines.append(
                    f"        {node_ids[mods[i]]} -> {node_ids[mods[i + 1]]} [style=invis];"
                )
        lines.append("    }")
        lines.append("")

    # Add invisible edges between clusters to force vertical stacking
    if LayoutAlgo.STACK_CLUSTERS in layout and len(sorted_groups) > 1:
        lines.append("    // Force vertical cluster layout")
        for i in range(len(sorted_groups) - 1):
            group_a = sorted_groups[i]
            group_b = sorted_groups[i + 1]
            # Pick first node from each cluster
            node_a = sorted(by_group[group_a])[0]
            node_b = sorted(by_group[group_b])[0]
            lines.append(f"    {node_ids[node_a]} -> {node_ids[node_b]} [style=invis, weight=10];")
        lines.append("")

    # Each wired stream type has its own node, including namespaced streams.
    for index, (name, type_) in enumerate(channels):
        if (name, type_.__name__) in ignored_streams:
            continue
        node_id = f"channel_{index}"
        label = f"{name}:{type_.__name__}"
        color = color_for_string(TYPE_COLORS, label)
        lines.append(
            f"    {node_id} [label={_dot_label(label)}, shape=note, style=filled, "
            f'fillcolor="{color}35", color="{color}", fontcolor="{theme.FOREGROUND}", '
            f'width=0, height=0, margin="0.1,0.05", fontsize=10];'
        )
        for producer in producers[(name, type_)]:
            lines.append(
                f'    {node_ids[producer]} -> {node_id} [color="{color}", arrowhead=none];'
            )
        for consumer in consumers[(name, type_)]:
            lines.append(f'    {node_id} -> {node_ids[consumer]} [color="{color}"];')

    for index, connection in enumerate(
        sorted(rpc_connections, key=lambda c: (c.caller, c.ref_name))
    ):
        if connection.caller not in atoms or connection.provider not in atoms:
            continue
        node_id = f"rpc_{index}"
        method_lines = "".join(f"\\n{method}()" for method in connection.methods)
        label = f"{connection.ref_name}:{connection.contract_name}{method_lines}"
        lines.append(
            f'    {node_id} [label="{label}", shape=component, style=filled, '
            f'fillcolor="{RPC_COLOR}35", color="{RPC_COLOR}", '
            f'fontcolor="{theme.FOREGROUND}", width=0, height=0, '
            'margin="0.1,0.05", fontsize=9];'
        )
        lines.append(
            f"    {node_ids[connection.caller]} -> {node_id} "
            f'[color="{RPC_COLOR}", style=dashed, arrowhead=none];'
        )
        lines.append(
            f'    {node_id} -> {node_ids[connection.provider]} [color="{RPC_COLOR}", style=dashed];'
        )

    lines.append("}")
    return "\n".join(lines)


def render_svg(
    blueprint_set: Blueprint,
    output_path: str,
    *,
    layout: set[LayoutAlgo] | None = None,
    include_rpc: bool = True,
) -> None:
    """Generate an SVG file from a Blueprint using graphviz.

    Args:
        blueprint_set: The blueprint set to visualize.
        output_path: Path to write the SVG file.
        layout: Set of layout algorithms to apply.
        include_rpc: Whether to include RPC contracts and relationships.
    """
    if layout is None:
        layout = set()

    dot_code = render(blueprint_set, layout=layout, include_rpc=include_rpc)
    engine = "fdp" if LayoutAlgo.FDP in layout else "dot"
    render_dot_svg(dot_code, output_path, engine=engine)
