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

import importlib.util
import os
import shutil
import tempfile
import webbrowser


def _build_html(python_file: str, *, show_disconnected: bool = True) -> str:
    """Import a Python file, find all Blueprint globals, and return rendered HTML."""
    filepath = os.path.abspath(python_file)
    if not os.path.isfile(filepath):
        raise FileNotFoundError(filepath)

    spec = importlib.util.spec_from_file_location("_render_target", filepath)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load {filepath}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    from dimos.core.blueprints import Blueprint
    from dimos.core.introspection.svg import to_svg

    blueprints: list[tuple[str, Blueprint]] = []
    for name, obj in vars(mod).items():
        if name.startswith("_"):
            continue
        if isinstance(obj, Blueprint):
            blueprints.append((name, obj))

    if not blueprints:
        raise RuntimeError("No Blueprint instances found in module globals.")

    print(f"Found {len(blueprints)} blueprint(s): {', '.join(n for n, _ in blueprints)}")

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
        to_svg(bp, svg_path, show_disconnected=show_disconnected)
        with open(svg_path) as f:
            svg_content = f.read()
        os.unlink(svg_path)
        sections.append(f'<h2>{name}</h2>\n<div class="diagram">{svg_content}</div>')

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


def main(python_file: str, *, show_disconnected: bool = True, port: int = 0) -> None:
    """Render Blueprint SVG diagrams and display them via a one-shot HTTP server."""
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

    server = HTTPServer(("0.0.0.0", port), Handler)
    actual_port = server.server_address[1]
    url = f"http://localhost:{actual_port}"
    print(f"Serving at {url}  (will exit after first request)")
    webbrowser.open(url)
    server.handle_request()
    print("Served. Exiting.")
