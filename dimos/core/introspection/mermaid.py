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

"""Mermaid flowchart renderer for Blueprint visualization."""

from __future__ import annotations

from collections import defaultdict
from typing import Any

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.introspection.utils import sanitize_id
from dimos.core.module import ModuleBase

Theme = dict[str, Any]

THEMES: dict[str, Theme] = {
    "tailwind": {
        "background": "#1e1e1e",
        "mermaid_theme": "dark",
        "nodes": [
            "#3b82f6",
            "#ef4444",
            "#22c55e",
            "#8b5cf6",
            "#f97316",
            "#06b6d4",
            "#ec4899",
            "#6366f1",
            "#eab308",
            "#14b8a6",
            "#f43f5e",
            "#84cc16",
            "#0ea5e9",
            "#d946ef",
            "#10b981",
            "#a855f7",
            "#f59e0b",
            "#38bdf8",
            "#fb7185",
            "#a3e635",
        ],
        "edges": [
            "#60a5fa",
            "#f87171",
            "#4ade80",
            "#a78bfa",
            "#fb923c",
            "#22d3ee",
            "#f472b6",
            "#818cf8",
            "#facc15",
            "#2dd4bf",
            "#fb7185",
            "#a3e635",
            "#38bdf8",
            "#e879f9",
            "#34d399",
            "#c084fc",
            "#fbbf24",
            "#67e8f9",
            "#fda4af",
            "#bef264",
        ],
    },
    "ocean": {
        "background": "#0f172a",
        "mermaid_theme": "dark",
        "nodes": [
            "#38bdf8",
            "#818cf8",
            "#2dd4bf",
            "#a78bfa",
            "#67e8f9",
            "#c084fc",
            "#5eead4",
            "#93c5fd",
            "#7dd3fc",
            "#6366f1",
        ],
        "edges": [
            "#7dd3fc",
            "#a5b4fc",
            "#99f6e4",
            "#c4b5fd",
            "#a5f3fc",
            "#ddd6fe",
            "#6ee7b7",
            "#bfdbfe",
            "#bae6fd",
            "#a5b4fc",
        ],
    },
    "ember": {
        "background": "#1c1210",
        "mermaid_theme": "dark",
        "nodes": [
            "#ef4444",
            "#f97316",
            "#eab308",
            "#f59e0b",
            "#fb923c",
            "#fbbf24",
            "#f87171",
            "#facc15",
            "#fb7185",
            "#fca5a5",
        ],
        "edges": [
            "#fca5a5",
            "#fdba74",
            "#fde047",
            "#fcd34d",
            "#fed7aa",
            "#fef08a",
            "#fecaca",
            "#fef9c3",
            "#fda4af",
            "#fecdd3",
        ],
    },
    "forest": {
        "background": "#0f1a14",
        "mermaid_theme": "dark",
        "nodes": [
            "#22c55e",
            "#14b8a6",
            "#84cc16",
            "#10b981",
            "#a3e635",
            "#34d399",
            "#4ade80",
            "#2dd4bf",
            "#86efac",
            "#6ee7b7",
        ],
        "edges": [
            "#86efac",
            "#5eead4",
            "#bef264",
            "#6ee7b7",
            "#d9f99d",
            "#99f6e4",
            "#bbf7d0",
            "#a7f3d0",
            "#ecfccb",
            "#ccfbf1",
        ],
    },
    "light": {
        "background": "#f8fafc",
        "mermaid_theme": "default",
        "nodes": [
            "#2563eb",
            "#dc2626",
            "#16a34a",
            "#7c3aed",
            "#ea580c",
            "#0891b2",
            "#db2777",
            "#4f46e5",
            "#ca8a04",
            "#0d9488",
        ],
        "edges": [
            "#3b82f6",
            "#ef4444",
            "#22c55e",
            "#8b5cf6",
            "#f97316",
            "#06b6d4",
            "#ec4899",
            "#6366f1",
            "#eab308",
            "#14b8a6",
        ],
    },
}

DEFAULT_THEME = "tailwind"

DEFAULT_IGNORED_CONNECTIONS: set[tuple[str, str]] = set()
DEFAULT_IGNORED_MODULES: set[str] = set()


class _ColorAssigner:
    def __init__(self, palette: list[str]) -> None:
        self._palette = palette
        self._assigned: dict[str, str] = {}
        self._next = 0

    def __call__(self, key: str) -> str:
        if key not in self._assigned:
            self._assigned[key] = self._palette[self._next % len(self._palette)]
            self._next += 1
        return self._assigned[key]


def render_mermaid(
    blueprint_set: Blueprint,
    *,
    ignored_streams: set[tuple[str, str]] | None = None,
    ignored_modules: set[str] | None = None,
    show_disconnected: bool = False,
    theme: str = DEFAULT_THEME,
) -> tuple[str, dict[str, str], set[str], dict[str, str]]:
    """Generate a Mermaid flowchart from a Blueprint.

    Returns (mermaid_code, label_color_map, disconnected_labels, node_color_map).
    """
    if ignored_streams is None:
        ignored_streams = DEFAULT_IGNORED_CONNECTIONS
    if ignored_modules is None:
        ignored_modules = DEFAULT_IGNORED_MODULES

    producers: dict[tuple[str, type], list[type[ModuleBase]]] = defaultdict(list)
    consumers: dict[tuple[str, type], list[type[ModuleBase]]] = defaultdict(list)
    module_names: set[str] = set()

    for bp in blueprint_set.blueprints:
        if bp.module.__name__ in ignored_modules:
            continue
        module_names.add(bp.module.__name__)
        for conn in bp.streams:
            remapped_name = blueprint_set.remapping_map.get((bp.module, conn.name), conn.name)
            if not isinstance(remapped_name, str):
                continue
            key = (remapped_name, conn.type)
            if conn.direction == "out":
                producers[key].append(bp.module)
            else:
                consumers[key].append(bp.module)

    active_keys: list[tuple[str, type]] = []
    for key in producers:
        name, type_ = key
        if key not in consumers:
            continue
        if (name, type_.__name__) in ignored_streams:
            continue
        valid_producers = [m for m in producers[key] if m.__name__ not in ignored_modules]
        valid_consumers = [m for m in consumers[key] if m.__name__ not in ignored_modules]
        if valid_producers and valid_consumers:
            active_keys.append(key)

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

    sorted_modules = sorted(module_names)
    for module_name in sorted_modules:
        mermaid_id = sanitize_id(module_name)
        lines.append(f"    {mermaid_id}([{module_name}]):::moduleNode")

    lines.append("")

    edge_idx = 0
    edge_colors: list[str] = []
    label_color_map: dict[str, str] = {}
    stream_node_ids: dict[str, str] = {}
    disconnected_labels: set[str] = set()

    lines.append("    %% Stream nodes and edges")
    for key in sorted(active_keys, key=lambda k: f"{k[0]}:{k[1].__name__}"):
        name, type_ = key
        label = f"{name}:{type_.__name__}"
        color = edge_color(label)
        label_color_map[label] = color

        valid_producers = [m for m in producers[key] if m.__name__ not in ignored_modules]
        valid_consumers = [m for m in consumers[key] if m.__name__ not in ignored_modules]

        for prod in valid_producers:
            stream_node_id = sanitize_id(f"{prod.__name__}_{name}_{type_.__name__}")
            if stream_node_id not in stream_node_ids:
                lines.append(f"    {stream_node_id}[{label}]:::streamNode")
                stream_node_ids[stream_node_id] = color

            producer_id = sanitize_id(prod.__name__)
            lines.append(f"    {producer_id} --- {stream_node_id}")
            edge_colors.append(node_color(prod.__name__))
            edge_idx += 1

            for cons in valid_consumers:
                consumer_id = sanitize_id(cons.__name__)
                lines.append(f"    {stream_node_id} --> {consumer_id}")
                edge_colors.append(color)
                edge_idx += 1

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
                stream_node_id = sanitize_id(f"{prod.__name__}_{name}_{type_.__name__}")
                if stream_node_id not in stream_node_ids:
                    lines.append(f"    {stream_node_id}[{label}]:::streamNode")
                    stream_node_ids[stream_node_id] = color
                producer_id = sanitize_id(prod.__name__)
                lines.append(f"    {producer_id} -.- {stream_node_id}")
                edge_colors.append(node_color(prod.__name__))
                edge_idx += 1

            for cons in consumers.get(key, []):
                if cons.__name__ in ignored_modules:
                    continue
                stream_node_id = sanitize_id(f"dangling_{name}_{type_.__name__}")
                if stream_node_id not in stream_node_ids:
                    lines.append(f"    {stream_node_id}[{label}]:::streamNode")
                    stream_node_ids[stream_node_id] = color
                consumer_id = sanitize_id(cons.__name__)
                lines.append(f"    {stream_node_id} -.-> {consumer_id}")
                edge_colors.append(color)
                edge_idx += 1

    lines.append("")
    for module_name in sorted_modules:
        mermaid_id = sanitize_id(module_name)
        color = node_color(module_name)
        lines.append(
            f"    style {mermaid_id} fill:{color}bf,stroke:{color},color:#eee,stroke-width:2px"
        )

    for stream_node_id, color in stream_node_ids.items():
        lines.append(
            f"    style {stream_node_id} fill:transparent,stroke:{color},color:{color},stroke-width:1px"
        )

    if edge_colors:
        lines.append("")
        for i, color in enumerate(edge_colors):
            lines.append(f"    linkStyle {i} stroke:{color},stroke-width:2px")

    node_color_map = dict(node_color._assigned)
    return "\n".join(lines), label_color_map, disconnected_labels, node_color_map
