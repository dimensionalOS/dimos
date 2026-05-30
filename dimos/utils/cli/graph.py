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
import sys
import webbrowser

from dimos.core.coordination.blueprints import Blueprint
from dimos.core.introspection.blueprint.mermaid import DEFAULT_THEME, THEMES, render_mermaid


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
        if name.startswith("_"):
            continue
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
    per_bp_conflicts: list[list[dict[str, object]]] = []
    per_bp_typos: list[list[dict[str, object]]] = []

    tab_buttons = []
    tab_panels = []
    for idx, (name, bp) in enumerate(blueprints):
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
        conflicts = [
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

        active_cls = " active" if idx == 0 else ""
        tab_buttons.append(f'<button class="tab-btn{active_cls}" data-idx="{idx}">{name}</button>')
        tab_panels.append(
            f'<div class="tab-panel{active_cls}" data-idx="{idx}">'
            f'<div class="viewport"><div class="canvas">'
            f'<pre class="mermaid">\n{mermaid_code}\n</pre>'
            f"</div></div></div>"
        )

    all_label_colors_json = json.dumps(per_bp_label_colors)
    all_disconnected_json = json.dumps([sorted(d) for d in per_bp_disconnected])
    all_conflicts_json = json.dumps(per_bp_conflicts)
    all_typos_json = json.dumps(per_bp_typos)

    tab_bar_html = ""
    if len(blueprints) > 1:
        tab_bar_html = f'<div class="tab-bar">{"".join(tab_buttons)}</div>'

    return f"""\
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<title>Blueprint Diagrams</title>
<link rel="icon" type="image/svg+xml" href="/favicon.ico">
<style>
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{ background: {background}; color: {text_color}; font-family: sans-serif; overflow: hidden; height: 100vh; }}
.tab-bar {{
    display: flex; gap: 0; border-bottom: 1px solid {border_color}; background: {surface};
    position: relative; z-index: 2;
}}
.tab-btn {{
    background: transparent; color: {text_muted}; border: none; border-bottom: 2px solid transparent;
    padding: 0.6em 1.4em; font-size: 0.95em; cursor: pointer; white-space: nowrap;
}}
.tab-btn:hover {{ color: {text_color}; background: {surface_hover}; }}
.tab-btn.active {{ color: {text_bright}; border-bottom-color: #60a5fa; background: {background}; }}
.tab-panel.hidden {{ display: none; }}
.viewport {{
    width: 100%; height: calc(100vh - 2.6em);
    overflow: hidden; cursor: grab; position: relative;
}}
.viewport.grabbing {{ cursor: grabbing; }}
.canvas {{
    transform-origin: 0 0;
    position: absolute;
    padding: 2em;
}}
.controls {{
    position: fixed; bottom: 1.2em; right: 1.2em; z-index: 10;
    display: flex; gap: 0.4em; background: {controls_bg}; border-radius: 6px;
    padding: 0.3em; border: 1px solid {border_color};
}}
.controls button {{
    background: {controls_btn}; color: {text_color}; border: 1px solid {controls_border}; border-radius: 4px;
    width: 2.2em; height: 2.2em; font-size: 1em; cursor: pointer;
    display: flex; align-items: center; justify-content: center;
}}
.controls button:hover {{ background: {surface_hover}; }}
.edgeLabel rect, .edgeLabel polygon {{ fill: {label_bg} !important; stroke: none !important; rx: 6; ry: 6; }}
.edgeLabel .label-container {{ background: {label_bg} !important; border-radius: 6px; }}
.edgeLabel foreignObject div, .edgeLabel foreignObject span, .edgeLabel foreignObject p {{
    background: {label_bg} !important; background-color: {label_bg} !important;
    border-radius: 6px; padding: 2px 6px;
}}
.moduleNode .nodeLabel {{ font-size: 38px !important; font-weight: 600 !important; display: block !important; transform: scale(0.7) !important; }}
.streamNode .nodeLabel {{ font-size: 18px !important; }}
.warnings-container {{
    position: fixed; bottom: 1.2em; left: 1.2em; z-index: 10;
    display: flex; flex-direction: column; gap: 0.6em; max-width: 30em;
}}
.warnings-container:empty {{ display: none; }}
.warning-box {{
    background: {controls_bg}; border: 1px solid #e57373; border-radius: 6px;
    padding: 0.7em 1em; font-size: 0.85em; color: {text_color};
}}
.warning-title {{ color: #e57373; font-weight: 600; margin-bottom: 0.3em; }}
.warning-item {{ margin: 0.25em 0; padding-top: 0.4em; border-top: 1px solid {border_color}; }}
.warning-item:first-of-type {{ border-top: none; padding-top: 0; }}
.warning-module {{
    display: inline-block; padding: 3px 10px; border-radius: 10px;
    color: #eee; font-size: 0.92em; margin: 2px 2px;
}}
.warning-stream {{
    display: inline-block; padding: 3px 8px; border: 1px solid;
    border-radius: 3px; font-size: 0.92em; margin: 2px 2px;
}}
.typo-arrow {{ color: {text_muted}; margin: 0 2px; }}
</style>
</head><body>
{tab_bar_html}
{"".join(tab_panels)}
<div class="warnings-container" id="warningsContainer"></div>
<div class="controls">
    <button id="zoomIn" title="Zoom in">+</button>
    <button id="zoomOut" title="Zoom out">&minus;</button>
    <button id="resetView" title="Reset view">&#8634;</button>
</div>
<script type="module">
import mermaid from 'https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.esm.min.mjs';
mermaid.initialize({{
    startOnLoad: true,
    theme: '{mermaid_theme}',
    flowchart: {{
        curve: 'basis',
        padding: 8,
        nodeSpacing: 60,
        rankSpacing: 80,
    }},
}});

await mermaid.run();

const arrowScale = 2.3;
const arrowGap = 6;
document.querySelectorAll('marker').forEach(marker => {{
    const width = parseFloat(marker.getAttribute('markerWidth')) || 8;
    const height = parseFloat(marker.getAttribute('markerHeight')) || 8;
    marker.setAttribute('markerWidth', width * arrowScale);
    marker.setAttribute('markerHeight', height * arrowScale);
    const refX = parseFloat(marker.getAttribute('refX')) || 0;
    marker.setAttribute('refX', refX + arrowGap);
}});

const allLabelColors = {all_label_colors_json};
const allDisconnected = {all_disconnected_json};
const allConflicts = {all_conflicts_json};
const allTypos = {all_typos_json};

function renderWarnings(idx) {{
    const container = document.getElementById('warningsContainer');
    let html = '';
    const conflicts = allConflicts[idx] || [];
    if (conflicts.length > 0) {{
        html += '<div class="warning-box"><div class="warning-title">⚠ Possible Input Fighting</div>' +
            conflicts.map(c =>
                `<div class="warning-item">` +
                `<span class="warning-stream" style="border-color:${{c.topicColor}};color:${{c.topicColor}}">${{c.topic}}</span> ` +
                c.modules.map(m => `<span class="warning-module" style="background:${{m.color}}bf">${{m.name}}</span>`).join(' ') +
                `</div>`
            ).join('') + '</div>';
    }}
    const typos = allTypos[idx] || [];
    if (typos.length > 0) {{
        html += '<div class="warning-box"><div class="warning-title">⚠ Possible Typos</div>' +
            typos.map(t =>
                `<div class="warning-item">` +
                `<span class="warning-stream" style="border-color:${{t.outColor}};color:${{t.outColor}}">${{t.outLabel}}</span>` +
                `<span class="typo-arrow">≠</span>` +
                `<span class="warning-stream" style="border-color:${{t.inColor}};color:${{t.inColor}}">${{t.inLabel}}</span>` +
                `<div>` +
                t.outModules.map(m => `<span class="warning-module" style="background:${{m.color}}bf">${{m.name}}</span>`).join(' ') +
                `<span class="typo-arrow">→</span>` +
                t.inModules.map(m => `<span class="warning-module" style="background:${{m.color}}bf">${{m.name}}</span>`).join(' ') +
                `</div></div>`
            ).join('') + '</div>';
    }}
    container.innerHTML = html;
}}

function setupViewport(vp, labelColors, disconnectedList) {{
    const canvas = vp.querySelector('.canvas');
    const svg = canvas.querySelector('svg');
    if (!svg) return;
    let scale, panX, panY;
    let dragging = false, startX, startY;

    svg.querySelectorAll('.node').forEach(node => {{
        const rect = node.querySelector('rect');
        if (!rect) return;
        const w = parseFloat(rect.getAttribute('width'));
        const h = parseFloat(rect.getAttribute('height'));
        const x = parseFloat(rect.getAttribute('x'));
        const y = parseFloat(rect.getAttribute('y'));
        if (!w || !h) return;
        const isStream = rect.getAttribute('style')?.includes('fill: transparent') ||
                         rect.style.fill === 'transparent';
        if (isStream) {{
            const gx = 4, gy = 2;
            rect.setAttribute('width', w + gx * 2);
            rect.setAttribute('height', h + gy * 2);
            rect.setAttribute('x', x - gx);
            rect.setAttribute('y', y - gy);
            node.querySelectorAll('span, text, div').forEach(el => {{
                el.style.fontSize = '14px';
            }});
        }} else {{
            const gx = 30, gy = 18;
            rect.setAttribute('width', w + gx * 2);
            rect.setAttribute('height', h + gy * 2);
            rect.setAttribute('x', x - gx);
            rect.setAttribute('y', y - gy);
        }}
    }});

    svg.querySelectorAll('.edgeLabel').forEach(label => {{
        const fo = label.querySelector('foreignObject');
        if (fo) {{
            fo.setAttribute('height', '35');
            const div = fo.querySelector('div');
            if (div) {{
                const span = document.createElement('span');
                span.textContent = div.textContent;
                span.style.cssText = div.querySelector('span')?.style.cssText || '';
                span.style.display = 'inline-flex';
                span.style.alignItems = 'center';
                span.style.height = '100%';
                div.replaceWith(span);
            }}
        }}
        const rect = label.querySelector('rect');
        if (rect) {{ rect.setAttribute('rx', '6'); rect.setAttribute('ry', '6'); }}
    }});

    const disconnectedLabels = new Set(disconnectedList);
    svg.querySelectorAll('.edgeLabel').forEach(label => {{
        const text = (label.textContent || '').trim();
        const color = labelColors[text];
        if (!color) return;
        label.querySelectorAll('span, p, text').forEach(el => {{
            if (el.tagName === 'text') el.setAttribute('fill', color);
            else el.style.color = color;
        }});
        if (disconnectedLabels.has(text)) {{
            label.querySelectorAll('span').forEach(span => {{
                span.style.border = `dashed ${{color}} 1px`;
                span.style.borderRadius = '4px';
                span.style.padding = '2px 6px';
            }});
        }}
    }});

    function fitToView() {{
        const vpRect = vp.getBoundingClientRect();
        canvas.style.transform = 'none';
        const svgRect = svg.getBoundingClientRect();
        const svgW = svgRect.width;
        const svgH = svgRect.height;
        const pad = 40;
        scale = Math.min((vpRect.width - pad) / svgW, (vpRect.height - pad) / svgH);
        scale = Math.max(scale * 0.8, 0.2);
        panX = (vpRect.width - svgW * scale) / 2;
        panY = (vpRect.height - svgH * scale) / 2;
        apply();
    }}

    function apply() {{
        canvas.style.transform = `translate(${{panX}}px, ${{panY}}px) scale(${{scale}})`;
    }}

    fitToView();

    vp.addEventListener('wheel', e => {{
        e.preventDefault();
        const rect = vp.getBoundingClientRect();
        const mx = e.clientX - rect.left;
        const my = e.clientY - rect.top;
        const factor = e.deltaY < 0 ? 1.12 : 1 / 1.12;
        const newScale = Math.min(Math.max(scale * factor, 0.05), 50);
        panX = mx - (mx - panX) * (newScale / scale);
        panY = my - (my - panY) * (newScale / scale);
        scale = newScale;
        apply();
    }}, {{ passive: false }});

    vp.addEventListener('mousedown', e => {{
        if (e.button !== 0) return;
        dragging = true; startX = e.clientX - panX; startY = e.clientY - panY;
        vp.classList.add('grabbing');
    }});
    window.addEventListener('mousemove', e => {{
        if (!dragging) return;
        panX = e.clientX - startX; panY = e.clientY - startY;
        apply();
    }});
    window.addEventListener('mouseup', () => {{
        dragging = false;
        vp.classList.remove('grabbing');
    }});

    vp._fitToView = fitToView;
    vp._zoomBy = (factor) => {{
        const rect = vp.getBoundingClientRect();
        const cx = rect.width / 2, cy = rect.height / 2;
        const newScale = Math.min(Math.max(scale * factor, 0.05), 50);
        panX = cx - (cx - panX) * (newScale / scale);
        panY = cy - (cy - panY) * (newScale / scale);
        scale = newScale; apply();
    }};
}}

let activeViewport = null;
document.querySelectorAll('.tab-panel').forEach((panel, idx) => {{
    const vp = panel.querySelector('.viewport');
    if (vp) {{
        setupViewport(vp, allLabelColors[idx] || {{}}, allDisconnected[idx] || []);
        if (panel.classList.contains('active')) activeViewport = vp;
    }}
}});

document.getElementById('zoomIn').addEventListener('click', () => {{
    if (activeViewport?._zoomBy) activeViewport._zoomBy(1.3);
}});
document.getElementById('zoomOut').addEventListener('click', () => {{
    if (activeViewport?._zoomBy) activeViewport._zoomBy(1 / 1.3);
}});
document.getElementById('resetView').addEventListener('click', () => {{
    if (activeViewport?._fitToView) activeViewport._fitToView();
}});

renderWarnings(0);

document.querySelectorAll('.tab-panel:not(.active)').forEach(p => p.classList.add('hidden'));

document.querySelectorAll('.tab-btn').forEach(btn => {{
    btn.addEventListener('click', () => {{
        const idx = btn.dataset.idx;
        document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
        document.querySelectorAll('.tab-panel').forEach(p => {{
            p.classList.remove('active');
            p.classList.add('hidden');
        }});
        btn.classList.add('active');
        const panel = document.querySelector(`.tab-panel[data-idx="${{idx}}"]`);
        panel.classList.add('active');
        panel.classList.remove('hidden');
        const vp = panel.querySelector('.viewport');
        if (vp) {{
            activeViewport = vp;
            if (vp._fitToView) setTimeout(() => vp._fitToView(), 0);
        }}
        renderWarnings(parseInt(idx));
    }});
}});
</script>
</body></html>"""


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
    url = f"http://localhost:{actual_port}"
    print(f"Serving at {url}  (will exit after first request)")
    webbrowser.open(url)
    server.handle_request()
    print("Served. Exiting.")
