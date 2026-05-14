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

"""Mermaid diagram renderer for blueprint visualization.

Generates a Mermaid flowchart with direct labelled edges between modules:

    ModuleA -- "name:Type" --> ModuleB
"""

from __future__ import annotations

from collections import defaultdict
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dimos.core.blueprints import Blueprint
    from dimos.core.module import Module

# ---------------------------------------------------------------------------
# Colour themes
# ---------------------------------------------------------------------------
# Each theme is a dict with "nodes" and "edges" colour lists.

THEMES: dict[str, dict[str, list[str]]] = {
    # Vivid — bold, high-contrast, maximally distinct
    "vivid": {
        "nodes": [
            "#1565c0",  # blue
            "#c62828",  # red
            "#2e7d32",  # green
            "#6a1b9a",  # purple
            "#d84315",  # burnt orange
            "#00838f",  # teal
            "#ad1457",  # pink
            "#4527a0",  # deep purple
            "#ef6c00",  # orange
            "#00695c",  # dark teal
            "#283593",  # indigo
            "#9e9d24",  # olive
            "#1565a0",  # steel blue
            "#b71c1c",  # dark red
            "#558b2f",  # lime green
            "#6d4c41",  # brown
            "#00796b",  # sea green
            "#7b1fa2",  # violet
            "#e65100",  # deep orange
            "#0277bd",  # light blue
        ],
        "edges": [
            "#4cc9f0",  # sky blue
            "#f77f00",  # orange
            "#80ed99",  # mint green
            "#c77dff",  # lavender
            "#ffd166",  # gold
            "#ef476f",  # coral red
            "#06d6a0",  # teal
            "#3a86ff",  # bright blue
            "#ff9e00",  # amber
            "#e5383b",  # red
            "#2ec4b6",  # cyan-teal
            "#9b5de5",  # purple
            "#00f5d4",  # aquamarine
            "#fee440",  # yellow
            "#f15bb5",  # magenta
            "#00bbf9",  # cerulean
            "#8ac926",  # lime green
            "#ff595e",  # salmon
            "#1982c4",  # steel blue
            "#ffca3a",  # sunflower
        ],
    },
    # Tailwind — coordinated palette based on Tailwind CSS colour system.
    # Nodes use the 700 shade (rich, readable with white text).
    # Edges use the 400 shade (bright, high-visibility on dark backgrounds).
    "tailwind": {
        "nodes": [
            "#3b82f6",  # blue-500
            "#ef4444",  # red-500
            "#22c55e",  # green-500
            "#8b5cf6",  # violet-500
            "#f97316",  # orange-500
            "#06b6d4",  # cyan-500
            "#ec4899",  # pink-500
            "#6366f1",  # indigo-500
            "#eab308",  # yellow-500
            "#14b8a6",  # teal-500
            "#f43f5e",  # rose-500
            "#84cc16",  # lime-500
            "#0ea5e9",  # sky-500
            "#d946ef",  # fuchsia-500
            "#10b981",  # emerald-500
            "#a855f7",  # purple-500
            "#f59e0b",  # amber-500
            "#38bdf8",  # sky-400
            "#fb7185",  # rose-400
            "#a3e635",  # lime-400
        ],
        "edges": [
            "#60a5fa",  # blue-400
            "#f87171",  # red-400
            "#4ade80",  # green-400
            "#a78bfa",  # violet-400
            "#fb923c",  # orange-400
            "#22d3ee",  # cyan-400
            "#f472b6",  # pink-400
            "#818cf8",  # indigo-400
            "#facc15",  # yellow-400
            "#2dd4bf",  # teal-400
            "#fb7185",  # rose-400
            "#a3e635",  # lime-400
            "#38bdf8",  # sky-400
            "#e879f9",  # fuchsia-400
            "#34d399",  # emerald-400
            "#c084fc",  # purple-400
            "#fbbf24",  # amber-400
            "#67e8f9",  # cyan-300
            "#fda4af",  # rose-300
            "#bef264",  # lime-300
        ],
    },
}

DEFAULT_THEME = "tailwind"


class _ColorAssigner:
    """Assigns colours from a palette sequentially, cycling when exhausted."""

    def __init__(self, palette: list[str]) -> None:
        self._palette = palette
        self._assigned: dict[str, str] = {}
        self._next = 0

    def __call__(self, key: str) -> str:
        if key not in self._assigned:
            self._assigned[key] = self._palette[self._next % len(self._palette)]
            self._next += 1
        return self._assigned[key]


# Connections to ignore (too noisy/common)
DEFAULT_IGNORED_CONNECTIONS = {("odom", "PoseStamped")}

DEFAULT_IGNORED_MODULES = {
    "WebsocketVisModule",
}

_COMPACT_ONLY_IGNORED_MODULES = {
    "WebsocketVisModule",
}


def _mermaid_id(name: str) -> str:
    """Sanitize a string into a valid Mermaid node id."""
    return name.replace(" ", "_").replace("-", "_")


def render(
    blueprint_set: Blueprint,
    *,
    ignored_streams: set[tuple[str, str]] | None = None,
    ignored_modules: set[str] | None = None,
    show_disconnected: bool = False,
    theme: str = DEFAULT_THEME,
) -> tuple[str, dict[str, str], set[str]]:
    """Generate a Mermaid flowchart from a Blueprint.

    Returns ``(mermaid_code, label_color_map, disconnected_labels)`` where
    *label_color_map* maps each edge label string to its hex colour and
    *disconnected_labels* is the set of labels for dangling streams.

    Args:
        theme: Colour theme name (one of ``THEMES`` keys).
    """
    if ignored_streams is None:
        ignored_streams = DEFAULT_IGNORED_CONNECTIONS
    if ignored_modules is None:
        if show_disconnected:
            ignored_modules = DEFAULT_IGNORED_MODULES - _COMPACT_ONLY_IGNORED_MODULES
        else:
            ignored_modules = DEFAULT_IGNORED_MODULES

    # Collect producers/consumers
    producers: dict[tuple[str, type], list[type[Module]]] = defaultdict(list)
    consumers: dict[tuple[str, type], list[type[Module]]] = defaultdict(list)
    module_names: set[str] = set()

    for bp in blueprint_set.blueprints:
        if bp.module.__name__ in ignored_modules:
            continue
        module_names.add(bp.module.__name__)
        for conn in bp.streams:
            remapped_name = blueprint_set.remapping_map.get((bp.module, conn.name), conn.name)
            key = (remapped_name, conn.type)
            if conn.direction == "out":
                producers[key].append(bp.module)
            else:
                consumers[key].append(bp.module)

    # Active channels: both producer and consumer exist
    active_keys: list[tuple[str, type]] = []
    for key in producers:
        name, type_ = key
        if key not in consumers:
            continue
        if (name, type_.__name__) in ignored_streams:
            continue
        valid_p = [m for m in producers[key] if m.__name__ not in ignored_modules]
        valid_c = [m for m in consumers[key] if m.__name__ not in ignored_modules]
        if valid_p and valid_c:
            active_keys.append(key)

    # Disconnected channels
    disconnected_keys: list[tuple[str, type]] = []
    if show_disconnected:
        all_keys = set(producers.keys()) | set(consumers.keys())
        for key in all_keys:
            if key in active_keys:
                continue
            name, type_ = key
            if (name, type_.__name__) in ignored_streams:
                continue
            relevant = producers.get(key, []) + consumers.get(key, [])
            if all(m.__name__ in ignored_modules for m in relevant):
                continue
            disconnected_keys.append(key)

    palette = THEMES.get(theme, THEMES[DEFAULT_THEME])
    node_color = _ColorAssigner(palette["nodes"])
    edge_color = _ColorAssigner(palette["edges"])

    lines = ["graph LR"]

    # Declare module nodes with rounded boxes
    sorted_modules = sorted(module_names)
    for mod_name in sorted_modules:
        mid = _mermaid_id(mod_name)
        lines.append(f"    {mid}([{mod_name}]):::moduleNode")

    lines.append("")

    edge_idx = 0
    edge_colors: list[str] = []
    label_color_map: dict[str, str] = {}
    stream_node_ids: dict[str, str] = {}  # stream_node_id -> color
    disconnected_labels: set[str] = set()

    # Active streams: producer -> stream-node -> consumers
    lines.append("    %% Stream nodes and edges")
    for key in sorted(active_keys, key=lambda k: f"{k[0]}:{k[1].__name__}"):
        name, type_ = key
        label = f"{name}:{type_.__name__}"
        color = edge_color(label)
        label_color_map[label] = color

        valid_producers = [m for m in producers[key] if m.__name__ not in ignored_modules]
        valid_consumers = [m for m in consumers[key] if m.__name__ not in ignored_modules]

        for prod in valid_producers:
            # Create a stream node per producer+stream pair
            sn_id = _mermaid_id(f"{prod.__name__}_{name}_{type_.__name__}")
            if sn_id not in stream_node_ids:
                lines.append(f"    {sn_id}[{label}]:::streamNode")
                stream_node_ids[sn_id] = color

            # Edge: producer --- stream-node (no arrow, module color)
            pid = _mermaid_id(prod.__name__)
            lines.append(f"    {pid} --- {sn_id}")
            edge_colors.append(node_color(prod.__name__))
            edge_idx += 1

            # Edges: stream-node -> each consumer
            for cons in valid_consumers:
                cid = _mermaid_id(cons.__name__)
                lines.append(f"    {sn_id} --> {cid}")
                edge_colors.append(color)
                edge_idx += 1

    # Disconnected streams
    if disconnected_keys:
        lines.append("")
        lines.append("    %% Disconnected streams")
        for key in sorted(disconnected_keys, key=lambda k: f"{k[0]}:{k[1].__name__}"):
            name, type_ = key
            label = f"{name}:{type_.__name__}"
            color = edge_color(label)
            label_color_map[label] = color
            disconnected_labels.add(label)

            for prod in producers.get(key, []):
                if prod.__name__ in ignored_modules:
                    continue
                sn_id = _mermaid_id(f"{prod.__name__}_{name}_{type_.__name__}")
                if sn_id not in stream_node_ids:
                    lines.append(f"    {sn_id}[{label}]:::streamNode")
                    stream_node_ids[sn_id] = color
                pid = _mermaid_id(prod.__name__)
                lines.append(f"    {pid} -.- {sn_id}")
                edge_colors.append(node_color(prod.__name__))
                edge_idx += 1

            for cons in consumers.get(key, []):
                if cons.__name__ in ignored_modules:
                    continue
                # Consumer-only: create a standalone stream node
                sn_id = _mermaid_id(f"dangling_{name}_{type_.__name__}")
                if sn_id not in stream_node_ids:
                    lines.append(f"    {sn_id}[{label}]:::streamNode")
                    stream_node_ids[sn_id] = color
                cid = _mermaid_id(cons.__name__)
                lines.append(f"    {sn_id} -.-> {cid}")
                edge_colors.append(color)
                edge_idx += 1

    # Module node styles (colored fill)
    lines.append("")
    for mod_name in sorted_modules:
        mid = _mermaid_id(mod_name)
        c = node_color(mod_name)
        lines.append(f"    style {mid} fill:{c}bf,stroke:{c},color:#eee,stroke-width:2px")

    # Stream node styles (no fill, colored text and border)
    for sn_id, color in stream_node_ids.items():
        lines.append(
            f"    style {sn_id} fill:transparent,stroke:{color},color:{color},stroke-width:1px"
        )

    # Edge styles
    if edge_colors:
        lines.append("")
        for i, c in enumerate(edge_colors):
            lines.append(f"    linkStyle {i} stroke:{c},stroke-width:2px")

    return "\n".join(lines), label_color_map, disconnected_labels
