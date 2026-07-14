#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
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

"""Bake the skills-dashboard wrap-up into ONE self-contained report.html.

The live dashboard (``index.html``) fetches ``data/<item>/`` at runtime, which a
sandboxed/shareable context (e.g. a Claude artifact, strict CSP, ``file://``)
forbids. This packer takes the authored ``report.template.html`` and inlines
every figure it references so the result has zero external requests:

  - ``{{IMG:<slug>/<file>}}``  -> a ``data:`` URI (base64) for PNG/JPEG figures
  - ``{{SVG:<slug>/<file>}}``  -> the SVG markup inlined directly (prolog/DOCTYPE
                                  stripped, made width:100% responsive)

Both light/dark figure variants are embedded; the template's CSS shows the one
matching the viewer's theme. Paths in the template are relative to ``data/``.

Run:  python3 misc/skills_dashboard/scripts/build_report.py
Out:  misc/skills_dashboard/report.html   (git-ignored; regenerate on demand)
"""

from __future__ import annotations

import base64
from pathlib import Path
import re
import sys

DASHBOARD = Path(__file__).resolve().parent.parent  # misc/skills_dashboard
DATA = DASHBOARD / "data"
TEMPLATE = DASHBOARD / "report.template.html"
OUTPUT = DASHBOARD / "report.html"

_MIME = {"jpg": "image/jpeg", "jpeg": "image/jpeg", "png": "image/png", "webp": "image/webp"}


def _img_data_uri(rel: str) -> str:
    path = DATA / rel
    ext = path.suffix.lstrip(".").lower()
    if ext not in _MIME:
        raise ValueError(f"unsupported image type for inlining: {rel}")
    b64 = base64.b64encode(path.read_bytes()).decode("ascii")
    return f"data:{_MIME[ext]};base64,{b64}"


def _inline_svg(rel: str) -> str:
    svg = (DATA / rel).read_text()
    svg = re.sub(r"<\?xml.*?\?>", "", svg, flags=re.S)
    svg = re.sub(r"<!DOCTYPE.*?>", "", svg, flags=re.S)
    # Drop the root svg's fixed width/height so it scales to its figure card.
    svg = re.sub(r'(<svg[^>]*?)\swidth="[^"]*"', r"\1", svg, count=1)
    svg = re.sub(r'(<svg[^>]*?)\sheight="[^"]*"', r"\1", svg, count=1)
    head = svg.split(">", 1)[0]
    if "style=" not in head:
        svg = svg.replace("<svg", '<svg style="width:100%;height:auto;display:block"', 1)
    return svg.strip()


def build() -> int:
    if not TEMPLATE.exists():
        print(f"template not found: {TEMPLATE}", file=sys.stderr)
        return 1

    html = TEMPLATE.read_text()
    missing: list[str] = []

    def sub(fn):
        def _inner(m: re.Match[str]) -> str:
            rel = m.group(1)
            if not (DATA / rel).exists():
                missing.append(rel)
                return m.group(0)
            return fn(rel)

        return _inner

    html = re.sub(r"\{\{IMG:([^}]+)\}\}", sub(_img_data_uri), html)
    html = re.sub(r"\{\{SVG:([^}]+)\}\}", sub(_inline_svg), html)

    leftover = re.findall(r"\{\{[^}]+\}\}", html)
    if missing:
        print(f"ERROR: missing figures: {missing}", file=sys.stderr)
        return 2
    if leftover:
        print(f"ERROR: unresolved tokens: {leftover}", file=sys.stderr)
        return 2

    OUTPUT.write_text(html)
    print(f"wrote {OUTPUT.relative_to(DASHBOARD.parent.parent)}  ({len(html.encode()) / 1024:.0f} KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(build())
