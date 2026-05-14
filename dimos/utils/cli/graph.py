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

"""Render Blueprint graphs from a Python file and open in the browser."""

from __future__ import annotations

import html as html_mod
import importlib.util
import os
import shutil
import sys
import tempfile
from typing import TYPE_CHECKING
import webbrowser

if TYPE_CHECKING:
    from dimos.core.blueprints import Blueprint


def _find_package_root(filepath: str) -> str | None:
    """Walk up from *filepath* looking for the outermost package directory.

    Returns the parent of that package (i.e. the directory that should be on
    ``sys.path``), or ``None`` if the file is not inside a package.
    """
    d = os.path.dirname(filepath)
    root = None
    while os.path.isfile(os.path.join(d, "__init__.py")):
        root = d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    if root is not None:
        return os.path.dirname(root)
    return None


def _load_blueprints(python_file: str) -> list[tuple[str, Blueprint]]:
    """Import *python_file* and return ``[(name, Blueprint), ...]``."""
    filepath = os.path.abspath(python_file)
    if not os.path.isfile(filepath):
        raise FileNotFoundError(filepath)

    # Ensure the file's package root is importable so that relative imports
    # like ``from smartnav.blueprints.foo import bar`` work.
    pkg_root = _find_package_root(filepath)
    if pkg_root and pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)

    spec = importlib.util.spec_from_file_location("_render_target", filepath)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load {filepath}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    from dimos.core.blueprints import Blueprint

    blueprints: list[tuple[str, Blueprint]] = []
    for name, obj in vars(mod).items():
        if name.startswith("_"):
            continue
        if isinstance(obj, Blueprint):
            blueprints.append((name, obj))

    if not blueprints:
        raise RuntimeError("No Blueprint instances found in module globals.")

    blueprints.reverse()
    print(f"Found {len(blueprints)} blueprint(s): {', '.join(n for n, _ in blueprints)}")
    return blueprints


def _build_html(python_file: str, *, show_disconnected: bool = True) -> str:
    """Build an HTML page that renders blueprints as Mermaid diagrams."""
    from dimos.core.introspection.blueprint.mermaid import render as mermaid_render

    blueprints = _load_blueprints(python_file)

    import json

    per_bp_label_colors: list[dict[str, str]] = []
    per_bp_disconnected: list[set[str]] = []

    tab_buttons = []
    tab_panels = []
    for idx, (name, bp) in enumerate(blueprints):
        mermaid_code, label_colors, disconnected = mermaid_render(
            bp, show_disconnected=show_disconnected
        )
        per_bp_label_colors.append(label_colors)
        per_bp_disconnected.append(disconnected)

        active_cls = " active" if idx == 0 else ""
        escaped_name = html_mod.escape(name)
        tab_buttons.append(
            f'<button class="tab-btn{active_cls}" data-idx="{idx}">{escaped_name}</button>'
        )
        tab_panels.append(
            f'<div class="tab-panel{active_cls}" data-idx="{idx}">'
            f'<div class="viewport"><div class="canvas">'
            f'<pre class="mermaid">\n{mermaid_code}\n</pre>'
            f"</div></div></div>"
        )

    all_label_colors_json = json.dumps(per_bp_label_colors)
    all_disconnected_json = json.dumps([sorted(d) for d in per_bp_disconnected])

    tab_bar_html = ""
    if len(blueprints) > 1:
        tab_bar_html = f'<div class="tab-bar">{"".join(tab_buttons)}</div>'

    return f"""\
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<title>Blueprint Diagrams</title>
<style>
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{ background: #1e1e1e; color: #ccc; font-family: sans-serif; overflow: hidden; height: 100vh; }}
.tab-bar {{
    display: flex; gap: 0; border-bottom: 1px solid #444; background: #252525;
    position: relative; z-index: 2;
}}
.tab-btn {{
    background: transparent; color: #888; border: none; border-bottom: 2px solid transparent;
    padding: 0.6em 1.4em; font-size: 0.95em; cursor: pointer; white-space: nowrap;
}}
.tab-btn:hover {{ color: #ccc; background: #2a2a2a; }}
.tab-btn.active {{ color: #eee; border-bottom-color: #60a5fa; background: #1e1e1e; }}
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
    display: flex; gap: 0.4em; background: #2a2a2a; border-radius: 6px;
    padding: 0.3em; border: 1px solid #444;
}}
.controls button {{
    background: #333; color: #ccc; border: 1px solid #555; border-radius: 4px;
    width: 2.2em; height: 2.2em; font-size: 1em; cursor: pointer;
    display: flex; align-items: center; justify-content: center;
}}
.controls button:hover {{ background: #444; }}
.edgeLabel rect, .edgeLabel polygon {{ fill: rgba(30,30,30,0.7) !important; stroke: none !important; rx: 6; ry: 6; }}
.edgeLabel .label-container {{ background: rgba(30,30,30,0.7) !important; border-radius: 6px; }}
.edgeLabel foreignObject div, .edgeLabel foreignObject span, .edgeLabel foreignObject p {{
    background: rgba(30,30,30,0.7) !important; background-color: rgba(30,30,30,0.7) !important;
    border-radius: 6px; padding: 2px 6px;
}}
.moduleNode .nodeLabel {{ font-size: 38px !important; font-weight: 600 !important; display: block !important; transform: scale(0.7) !important; }}
.streamNode .nodeLabel {{ font-size: 18px !important; }}
</style>
</head><body>
{tab_bar_html}
{"".join(tab_panels)}
<div class="controls">
    <button id="zoomIn" title="Zoom in">+</button>
    <button id="zoomOut" title="Zoom out">&minus;</button>
    <button id="resetView" title="Reset view">&#8634;</button>
</div>
<script type="module">
import mermaid from 'https://cdn.jsdelivr.net/npm/mermaid@11/dist/mermaid.esm.min.mjs';
mermaid.initialize({{
    startOnLoad: true,
    theme: 'dark',
    flowchart: {{
        curve: 'basis',
        padding: 8,
        nodeSpacing: 60,
        rankSpacing: 80,
    }},
}});

await mermaid.run();

const allLabelColors = {all_label_colors_json};
const allDisconnected = {all_disconnected_json};

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
        scale = Math.max(scale * 2, 0.2);
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

    document.getElementById('zoomIn').addEventListener('click', () => {{
        const rect = vp.getBoundingClientRect();
        const cx = rect.width / 2, cy = rect.height / 2;
        const newScale = Math.min(scale * 1.3, 50);
        panX = cx - (cx - panX) * (newScale / scale);
        panY = cy - (cy - panY) * (newScale / scale);
        scale = newScale; apply();
    }});
    document.getElementById('zoomOut').addEventListener('click', () => {{
        const rect = vp.getBoundingClientRect();
        const cx = rect.width / 2, cy = rect.height / 2;
        const newScale = Math.max(scale / 1.3, 0.05);
        panX = cx - (cx - panX) * (newScale / scale);
        panY = cy - (cy - panY) * (newScale / scale);
        scale = newScale; apply();
    }});
    document.getElementById('resetView').addEventListener('click', () => {{
        fitToView();
    }});
}}

// All panels are visible — set up every viewport while they have real dimensions
document.querySelectorAll('.tab-panel').forEach((panel, idx) => {{
    const vp = panel.querySelector('.viewport');
    if (vp) setupViewport(vp, allLabelColors[idx] || {{}}, allDisconnected[idx] || []);
}});

// Now hide the non-active panels
document.querySelectorAll('.tab-panel:not(.active)').forEach(p => p.classList.add('hidden'));

// Tab switching
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
        if (vp && vp._fitToView) setTimeout(() => vp._fitToView(), 0);
    }});
}});
</script>
</body></html>"""


def _build_html_graphviz(python_file: str, *, show_disconnected: bool = True) -> str:
    """Build an HTML page that renders blueprints as Graphviz SVGs (requires ``dot``)."""
    from dimos.core.introspection.svg import to_svg

    blueprints = _load_blueprints(python_file)

    if not shutil.which("dot"):
        raise RuntimeError(
            "graphviz is not installed (the 'dot' command was not found).\n"
            "Install it with:  brew install graphviz   (macOS)\n"
            "                  apt install graphviz    (Debian/Ubuntu)"
        )

    sections = []
    for name, bp in blueprints:
        fd, svg_path = tempfile.mkstemp(suffix=".svg", prefix=f"dimos_{name}_")
        os.close(fd)
        try:
            to_svg(bp, svg_path, show_disconnected=show_disconnected)
            with open(svg_path) as f:
                svg_content = f.read()
        finally:
            os.unlink(svg_path)
        escaped_name = html_mod.escape(name)
        sections.append(f'<h2>{escaped_name}</h2>\n<div class="diagram">{svg_content}</div>')

    return f"""\
<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<title>Blueprint Diagrams</title>
<style>
body {{ background: #1e1e1e; color: #ccc; font-family: sans-serif; margin: 2em; }}
h2 {{ border-bottom: 1px solid #444; padding-bottom: 0.3em; }}
.diagram {{ margin-bottom: 3em; }}
.diagram svg {{ max-width: 100%; height: auto; }}
</style>
</head><body>
{"".join(sections)}
</body></html>"""


def _build_markdown(python_file: str, *, show_disconnected: bool = True) -> str:
    """Return Mermaid markdown for all blueprints in *python_file*."""
    from dimos.core.introspection.blueprint.mermaid import render as mermaid_render

    blueprints = _load_blueprints(python_file)

    sections: list[str] = []
    for name, bp in blueprints:
        mermaid_code, _label_colors, _disconnected = mermaid_render(
            bp, show_disconnected=show_disconnected
        )
        sections.append(f"## {name}\n\n```mermaid\n{mermaid_code}\n```")

    return "\n\n".join(sections)


def main(
    python_file: str,
    *,
    show_disconnected: bool = True,
    port: int = 0,
    markdown: bool = False,
) -> None:
    """Render Blueprint SVG diagrams and display them via a one-shot HTTP server."""
    if markdown:
        print(_build_markdown(python_file, show_disconnected=show_disconnected))
        return

    from http.server import BaseHTTPRequestHandler, HTTPServer

    html = _build_html(python_file, show_disconnected=show_disconnected)
    html_bytes = html.encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
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
