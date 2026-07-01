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

"""Render DimOS Blueprint graphs as Mermaid diagrams in the browser.

Loads Blueprint instances defined as module-level variables in a Python file
and serves an interactive Mermaid flowchart per blueprint.
"""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, HTTPServer
import importlib.util
import json
import os
from pathlib import Path
import sys
from typing import Any
import webbrowser

import jinja2

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.introspection.mermaid import DEFAULT_THEME, THEMES, render_mermaid
from dimos.utils.data import get_data

_CLI_DIR = Path(__file__).parent
_MERMAID_JS = get_data("mermaid.min.js").read_text(encoding="utf-8")
_TEMPLATE = jinja2.Template(
    (_CLI_DIR / "graph.html.jinja").read_text(encoding="utf-8"),
    autoescape=False,
)


def _levenshtein(a: str, b: str) -> int:
    if len(a) < len(b):
        return _levenshtein(b, a)
    if not b:
        return len(a)
    prev = list(range(len(b) + 1))
    for i, char_a in enumerate(a):
        curr = [i + 1]
        for j, char_b in enumerate(b):
            curr.append(min(prev[j + 1] + 1, curr[j] + 1, prev[j] + (char_a != char_b)))
        prev = curr
    return prev[-1]


def _find_package_root(filepath: str) -> str | None:
    directory = os.path.dirname(filepath)
    root = None
    while os.path.isfile(os.path.join(directory, "__init__.py")):
        root = directory
        parent = os.path.dirname(directory)
        if parent == directory:
            break
        directory = parent
    if root is not None:
        return os.path.dirname(root)
    return None


def _load_blueprints(python_file: str) -> list[tuple[str, Blueprint]]:
    filepath = os.path.abspath(python_file)
    if not os.path.isfile(filepath):
        raise FileNotFoundError(filepath)

    pkg_root = _find_package_root(filepath)
    if pkg_root and pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)

    spec = importlib.util.spec_from_file_location("_render_target", filepath)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load {filepath}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    blueprints: list[tuple[str, Blueprint]] = []
    for name, obj in vars(module).items():
        if isinstance(obj, Blueprint):
            blueprints.append((name, obj))

    if not blueprints:
        raise RuntimeError("No Blueprint instances found in module globals.")

    blueprints.reverse()
    print(
        f"Found {len(blueprints)} blueprint(s): {', '.join(n for n, _ in blueprints)}",
        file=sys.stderr,
    )
    return blueprints


def _build_html(
    python_file: str, *, show_disconnected: bool = True, theme: str = DEFAULT_THEME
) -> str:
    blueprints = _load_blueprints(python_file)
    palette = THEMES.get(theme, THEMES[DEFAULT_THEME])
    background = palette.get("background", "#1e1e1e")
    mermaid_theme = palette.get("mermaid_theme", "dark")
    is_light = mermaid_theme != "dark"
    text_color = "#334155" if is_light else "#ccc"
    text_muted = "#64748b" if is_light else "#888"
    text_bright = "#1e293b" if is_light else "#eee"
    surface = "#e2e8f0" if is_light else "#252525"
    surface_hover = "#cbd5e1" if is_light else "#2a2a2a"
    controls_bg = "#e2e8f0" if is_light else "#2a2a2a"
    controls_btn = "#cbd5e1" if is_light else "#333"
    controls_border = "#94a3b8" if is_light else "#555"
    border_color = "#cbd5e1" if is_light else "#444"
    label_bg = "rgba(248,250,252,0.85)" if is_light else "rgba(30,30,30,0.7)"

    per_bp_label_colors: list[dict[str, str]] = []
    per_bp_disconnected: list[set[str]] = []
    per_bp_node_colors: list[dict[str, str]] = []
    per_bp_conflicts: list[list[dict[str, Any]]] = []
    per_bp_typos: list[list[dict[str, object]]] = []

    tab_buttons: list[dict[str, str]] = []
    tab_panels: list[dict[str, str]] = []
    for name, bp in blueprints:
        mermaid_code, label_colors, disconnected, node_colors = render_mermaid(
            bp, show_disconnected=show_disconnected, theme=theme
        )
        per_bp_label_colors.append(label_colors)
        per_bp_disconnected.append(disconnected)
        per_bp_node_colors.append(node_colors)

        producers: dict[str, list[str]] = {}
        for atom in bp.blueprints:
            for stream in atom.streams:
                if stream.direction == "out":
                    topic = f"{stream.name}:{stream.type.__name__}"
                    producers.setdefault(topic, []).append(atom.module.__name__)
        conflicts: list[dict[str, object]] = [
            {
                "topic": topic,
                "topicColor": label_colors.get(topic, "#ccc"),
                "modules": [
                    {"name": module_name, "color": node_colors.get(module_name, "#ccc")}
                    for module_name in modules
                ],
            }
            for topic, modules in producers.items()
            if len(modules) > 1
        ]
        per_bp_conflicts.append(conflicts)

        outputs: dict[tuple[str, str], list[str]] = {}
        inputs: dict[tuple[str, str], list[str]] = {}
        for atom in bp.blueprints:
            for stream in atom.streams:
                key = (stream.name, stream.type.__name__)
                if stream.direction == "out":
                    outputs.setdefault(key, []).append(atom.module.__name__)
                else:
                    inputs.setdefault(key, []).append(atom.module.__name__)
        dangling_outs = {k: v for k, v in outputs.items() if k not in inputs}
        dangling_ins = {k: v for k, v in inputs.items() if k not in outputs}
        typos: list[dict[str, object]] = []
        for (out_name, out_type), out_modules in dangling_outs.items():
            for (in_name, in_type), in_modules in dangling_ins.items():
                if out_type != in_type:
                    continue
                distance = _levenshtein(out_name, in_name)
                if 0 < distance <= 2:
                    out_label = f"{out_name}:{out_type}"
                    in_label = f"{in_name}:{in_type}"
                    typos.append(
                        {
                            "outLabel": out_label,
                            "inLabel": in_label,
                            "outColor": label_colors.get(out_label, "#ccc"),
                            "inColor": label_colors.get(in_label, "#ccc"),
                            "outModules": [
                                {"name": m, "color": node_colors.get(m, "#ccc")}
                                for m in out_modules
                            ],
                            "inModules": [
                                {"name": m, "color": node_colors.get(m, "#ccc")} for m in in_modules
                            ],
                        }
                    )
        per_bp_typos.append(typos)

        tab_buttons.append({"name": name})
        tab_panels.append({"mermaid_code": mermaid_code})

    return _TEMPLATE.render(
        background=background,
        text_color=text_color,
        text_muted=text_muted,
        text_bright=text_bright,
        surface=surface,
        surface_hover=surface_hover,
        controls_bg=controls_bg,
        controls_btn=controls_btn,
        controls_border=controls_border,
        border_color=border_color,
        label_bg=label_bg,
        mermaid_theme=mermaid_theme,
        mermaid_js=_MERMAID_JS,
        tab_buttons=tab_buttons,
        tab_panels=tab_panels,
        all_label_colors_json=json.dumps(per_bp_label_colors),
        all_disconnected_json=json.dumps([sorted(d) for d in per_bp_disconnected]),
        all_conflicts_json=json.dumps(per_bp_conflicts),
        all_typos_json=json.dumps(per_bp_typos),
    )


def print_markdown(
    python_file: str, *, show_disconnected: bool, theme: str = DEFAULT_THEME
) -> None:
    blueprints = _load_blueprints(python_file)
    sections: list[str] = []
    for name, bp in blueprints:
        mermaid_code, _, _, _ = render_mermaid(bp, show_disconnected=show_disconnected, theme=theme)
        sections.append(f"## {name}\n\n```mermaid\n{mermaid_code}\n```")
    print("\n\n".join(sections))


def save_html(
    python_file: str,
    *,
    output_path: str,
    show_disconnected: bool,
    theme: str = DEFAULT_THEME,
) -> None:
    html = _build_html(python_file, show_disconnected=show_disconnected, theme=theme)
    with open(output_path, "w") as file:
        file.write(html)
    print(f"Wrote {output_path}", file=sys.stderr)


def serve_graph(
    python_file: str, *, show_disconnected: bool, port: int, theme: str = DEFAULT_THEME
) -> None:
    html = _build_html(python_file, show_disconnected=show_disconnected, theme=theme)
    html_bytes = html.encode("utf-8")

    favicon_svg = (
        b'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 32 32">'
        b'<circle cx="8" cy="16" r="5" fill="#3b82f6"/>'
        b'<circle cx="24" cy="6" r="4" fill="#60a5fa"/>'
        b'<circle cx="24" cy="26" r="4" fill="#60a5fa"/>'
        b'<line x1="12" y1="14" x2="20" y2="7" stroke="#60a5fa" stroke-width="2"/>'
        b'<line x1="12" y1="18" x2="20" y2="25" stroke="#60a5fa" stroke-width="2"/>'
        b"</svg>"
    )

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            if self.path == "/favicon.ico":
                self.send_response(200)
                self.send_header("Content-Type", "image/svg+xml")
                self.send_header("Content-Length", str(len(favicon_svg)))
                self.end_headers()
                self.wfile.write(favicon_svg)
                return
            if self.path not in ("/", ""):
                self.send_response(204)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html_bytes)))
            self.end_headers()
            self.wfile.write(html_bytes)

        def log_message(self, format: str, *args: object) -> None:
            pass

    server = HTTPServer(("127.0.0.1", port), Handler)
    actual_port = server.server_address[1]
    url = f"http://127.0.0.1:{actual_port}"
    print(f"Serving at {url}  (will exit after first request)")
    webbrowser.open(url)
    server.handle_request()
    print("Served. Exiting.")
