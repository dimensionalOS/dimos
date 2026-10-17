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

from dimos.cli import theme
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.coordination.module_coordinator import _resolve_single_ref
from dimos.core.introspection.utils import (
    GROUP_COLORS,
    RPC_COLOR,
    TYPE_COLORS,
    color_for_string,
    sanitize_id,
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


def _rpc_methods(contract: type) -> tuple[str, ...]:
    if is_spec(contract):
        return tuple(sorted(get_protocol_method_signatures(contract)))
    if is_module_type(contract):
        return tuple(sorted(contract.rpcs))
    return ()


def _resolve_rpc_connections(blueprint: Blueprint) -> list[RpcConnection]:
    atoms_by_name = {atom.name: atom for atom in blueprint.active_blueprints}
    disabled_set = set(blueprint.disabled_modules_tuple)
    connections: list[RpcConnection] = []

    for atom in blueprint.active_blueprints:
        for module_ref in atom.module_refs:
            replacement = blueprint.remapping_map.get((atom.name, module_ref.name), module_ref.spec)
            if not (is_spec(replacement) or is_module_type(replacement)):
                continue

            provider_name = _resolve_single_ref(
                atom,
                module_ref,
                replacement,
                blueprint,
                disabled_set,
            )
            if not isinstance(provider_name, str):
                continue
            provider = atoms_by_name.get(provider_name)
            if provider is None:
                continue

            connections.append(
                RpcConnection(
                    caller=atom.module.__name__,
                    provider=provider.module.__name__,
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

    # Collect all outputs: (name, type) -> list of producer modules
    producers: dict[tuple[str, type], list[type[ModuleBase]]] = defaultdict(list)
    # Collect all inputs: (name, type) -> list of consumer modules
    consumers: dict[tuple[str, type], list[type[ModuleBase]]] = defaultdict(list)
    # Module name -> module class (for getting package info)
    module_classes: dict[str, type[ModuleBase]] = {}

    for bp in blueprint_set.blueprints:
        module_classes[bp.module.__name__] = bp.module
        for conn in bp.streams:
            # Apply remapping
            remapped_name = blueprint_set.remapping_map.get((bp.name, conn.name), conn.name)
            key = (remapped_name, conn.type)
            if conn.direction == "out":
                producers[key].append(bp.module)  # type: ignore[index]
            else:
                consumers[key].append(bp.module)  # type: ignore[index]

    # Find all active channels (have both producers AND consumers)
    active_channels: dict[tuple[str, type], str] = {}  # key -> color
    for key in producers:
        name, type_ = key
        type_name = type_.__name__
        if key not in consumers:
            continue
        if (name, type_name) in ignored_streams:
            continue
        # Check if all modules are ignored
        valid_producers = [m for m in producers[key] if m.__name__ not in ignored_modules]
        valid_consumers = [m for m in consumers[key] if m.__name__ not in ignored_modules]
        if not valid_producers or not valid_consumers:
            continue
        label = f"{name}:{type_name}"
        active_channels[key] = color_for_string(TYPE_COLORS, label)

    # Group modules by package
    def get_group(mod_class: type[ModuleBase]) -> str:
        module_path = mod_class.__module__
        parts = module_path.split(".")
        if len(parts) >= 2 and parts[0] == "dimos":
            return parts[1]
        return "other"

    by_group: dict[str, list[str]] = defaultdict(list)
    for mod_name, mod_class in module_classes.items():
        if mod_name in ignored_modules:
            continue
        group = get_group(mod_class)
        by_group[group].append(mod_name)

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
            lines.append(f"        {mod};")
        # Stack nodes vertically within cluster
        if LayoutAlgo.STACK_NODES in layout and len(mods) > 1:
            for i in range(len(mods) - 1):
                lines.append(f"        {mods[i]} -> {mods[i + 1]} [style=invis];")
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
            lines.append(f"    {node_a} -> {node_b} [style=invis, weight=10];")
        lines.append("")

    # Add type nodes (outside all clusters)
    lines.append("    // Type nodes (data channels)")
    for key, color in sorted(
        active_channels.items(), key=lambda x: f"{x[0][0]}:{x[0][1].__name__}"
    ):
        name, type_ = key
        type_name = type_.__name__
        node_id = sanitize_id(f"chan_{name}_{type_name}")
        label = f"{name}:{type_name}"
        lines.append(
            f'    {node_id} [label="{label}", shape=note, style=filled, '
            f'fillcolor="{color}35", color="{color}", fontcolor="{theme.FOREGROUND}", '
            f'width=0, height=0, margin="0.1,0.05", fontsize=10];'
        )

    lines.append("")

    # Add RPC contract nodes (one per caller ModuleRef)
    lines.append("    // RPC contracts")
    for connection in sorted(rpc_connections, key=lambda c: (c.caller, c.ref_name)):
        if connection.caller in ignored_modules or connection.provider in ignored_modules:
            continue
        node_id = sanitize_id(f"rpc_{connection.caller}_{connection.ref_name}")
        method_lines = "".join(f"\\n{method}()" for method in connection.methods)
        label = f"{connection.ref_name}:{connection.contract_name}{method_lines}"
        lines.append(
            f'    {node_id} [label="{label}", shape=component, style=filled, '
            f'fillcolor="{RPC_COLOR}35", color="{RPC_COLOR}", '
            f'fontcolor="{theme.FOREGROUND}", width=0, height=0, '
            'margin="0.1,0.05", fontsize=9];'
        )

    lines.append("")

    # Add stream edges: producer -> type_node -> consumer
    lines.append("    // Stream edges")
    for key, color in sorted(
        active_channels.items(), key=lambda x: f"{x[0][0]}:{x[0][1].__name__}"
    ):
        name, type_ = key
        type_name = type_.__name__
        node_id = sanitize_id(f"chan_{name}_{type_name}")

        # Edges from producers to type node (no arrow, kept close)
        for producer in producers[key]:
            if producer.__name__ in ignored_modules:
                continue
            lines.append(f'    {producer.__name__} -> {node_id} [color="{color}", arrowhead=none];')

        # Edges from type node to consumers (with arrow)
        for consumer in consumers[key]:
            if consumer.__name__ in ignored_modules:
                continue
            lines.append(f'    {node_id} -> {consumer.__name__} [color="{color}"];')

    lines.append("")

    # Add RPC edges: caller -> contract -> provider
    lines.append("    // RPC edges")
    for connection in sorted(rpc_connections, key=lambda c: (c.caller, c.ref_name)):
        if connection.caller in ignored_modules or connection.provider in ignored_modules:
            continue
        node_id = sanitize_id(f"rpc_{connection.caller}_{connection.ref_name}")
        lines.append(
            f"    {connection.caller} -> {node_id} "
            f'[color="{RPC_COLOR}", style=dashed, arrowhead=none];'
        )
        lines.append(f'    {node_id} -> {connection.provider} [color="{RPC_COLOR}", style=dashed];')

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
    import subprocess

    if layout is None:
        layout = set()

    dot_code = render(blueprint_set, layout=layout, include_rpc=include_rpc)
    engine = "fdp" if LayoutAlgo.FDP in layout else "dot"
    result = subprocess.run(
        [engine, "-Tsvg", "-o", output_path],
        input=dot_code,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        raise RuntimeError(f"graphviz failed: {result.stderr}")
