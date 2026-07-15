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

"""Bake the LIVE skills dashboard into ONE self-contained report.html.

The dashboard (``index.html``) is data-driven: at runtime it ``fetch()``es
``data/items.json``, each ``data/<item>/item.json`` + ``results.xml`` + concept
sketch, and every figure / review-artifact image. A shareable context (a Claude
artifact, strict CSP, ``file://``) forbids those requests — so the artifact used
to be a hand-maintained *subset* of the dashboard and kept drifting behind it.

This packer removes that drift: it renders the artifact FROM THE SAME
``index.html`` + ``data/`` the dashboard uses, so the artifact has 100% of the
dashboard's depth by construction. It:

  - inlines every data file (items.json, item.json, results.xml, concept HTML,
    scenarios.json) into a ``window.__DATA__`` map and rewires the two fetch
    helpers to read from it;
  - inlines every figure and review-artifact image as a ``data:`` URI at
    near-native resolution (only genuinely large rasters are recompressed);
  - skips un-previewable binary exports (``.dxf``/``.glb``/``.rrd`` — nothing a
    browser can render), which fall back to a not-available badge;
  - prepends a "Live agent Q&A" landing tab from the master-VQA results, and
    grafts a digestible "At a glance" one-line-purpose table above the tabs;
  - strips the ``<!doctype>/<html>/<head>/<body>`` wrappers the Artifact host
    supplies, emitting ``<title> + <style> + body`` as a fragment.

Run:  uv run python misc/skills_dashboard/scripts/build_report.py
Out:  misc/skills_dashboard/report.html   (git-ignored; regenerate on demand)
"""

from __future__ import annotations

import base64
import io
import json
from pathlib import Path
import re
import sys

from PIL import Image

DASHBOARD = Path(__file__).resolve().parent.parent  # misc/skills_dashboard
DATA = DASHBOARD / "data"
REPO = DASHBOARD.parent.parent  # repo root
INDEX = DASHBOARD / "index.html"
OUTPUT = DASHBOARD / "report.html"

_MIME = {
    "jpg": "image/jpeg", "jpeg": "image/jpeg", "png": "image/png",
    "webp": "image/webp", "gif": "image/gif", "svg": "image/svg+xml",
    # dxf is NOT an image format a browser can decode — claiming "image/*" for
    # it is a false content-type/bytes mismatch (a plausible trip-wire for a
    # public-sharing content scanner). Generic binary is the honest type.
    "dxf": "application/octet-stream", "txt": "text/plain",
}
_RASTER = {"jpg", "jpeg", "png", "webp", "gif"}
# Binary CAD / 3D-model exports: a browser can't preview any of these, so
# inlining them only bloats the artifact for a "download" link that most
# viewers can't act on (glb needs a 3D viewer, rrd the Rerun viewer, dxf CAD
# software). They render as the not-available badge instead (see artifactsBlock).
_SKIP_INLINE_EXTS = {"dxf", "glb", "gltf", "rrd", "ply", "stl", "obj"}
# Non-image files up to this size are inlined as downloadable data URIs; bigger
# ones (e.g. multi-MB .rrd recordings) are left as (dead-in-artifact) links.
_MAX_FILE_INLINE = 1_600_000
# Review-artifact thumbnails render at ~150px tall in the grid, but the same
# data: URI is what opens on click — so this is really "max full-view
# resolution". Tuned for a locally-presented / bundled report (not public
# artifact sharing, which this content can't pass), so we keep near-native
# resolution: recompress only genuinely large rasters, and only downscale the
# rare one wider than the cap.
_THUMB_MAX_W = 1800
_THUMB_MIN_BYTES = 900_000  # only recompress/downscale files above this


def _data_uri(raw: bytes, ext: str) -> str:
    mime = _MIME.get(ext, "application/octet-stream")
    return f"data:{mime};base64," + base64.b64encode(raw).decode("ascii")


def _downscale_jpeg(raw: bytes, max_w: int) -> tuple[bytes, str]:
    """Re-encode a raster for a smaller artifact at (near-)full resolution.

    Downscales to ``max_w`` wide only if actually wider than that. Always
    recompresses through JPEG when there's no transparency to preserve — even
    at native resolution PNG is a poor format for photographic/rendered
    content, and this is what actually keeps a "higher resolution" embed from
    ballooning the artifact (a 1536x1024 render is ~3MB as PNG, ~400KB as
    quality-88 JPEG, same pixels). Falls back to the original bytes if PIL
    can't handle it.
    """
    try:
        im = Image.open(io.BytesIO(raw))
        im.load()
        if im.width > max_w:
            h = round(im.height * max_w / im.width)
            im = im.resize((max_w, h), Image.LANCZOS)
        out = io.BytesIO()
        if im.mode in ("RGBA", "P", "LA"):
            im.save(out, "PNG", optimize=True)
            return out.getvalue(), "png"
        im.convert("RGB").save(out, "JPEG", quality=88, optimize=True)
        return out.getvalue(), "jpg"
    except Exception:  # noqa: BLE001 - degrade to original on any PIL hiccup
        return raw, "png"


def _fs_path(ref: str, item_dir: Path) -> Path:
    """Resolve an asset reference to a filesystem path. Absolute refs (starting
    with '/') are repo-root-relative; bare names are relative to the item's data
    folder."""
    if ref.startswith("/"):
        return REPO / ref.lstrip("/")
    return item_dir / ref


def _inline_asset(ref: str, item_dir: Path, *, thumb: bool) -> str | None:
    """Return a data: URI for ``ref`` if it exists and is inlinable, else None."""
    path = _fs_path(ref, item_dir)
    if not path.is_file():
        return None
    ext = path.suffix.lstrip(".").lower()
    raw = path.read_bytes()
    if ext in _RASTER:
        if thumb and len(raw) > _THUMB_MIN_BYTES:
            raw, ext = _downscale_jpeg(raw, _THUMB_MAX_W)
        return _data_uri(raw, ext)
    if ext == "svg":
        return _data_uri(raw, "svg")
    # CAD/binary exports (dxf, ...): not previewable in a browser regardless of
    # size, so inlining them only bloats the artifact for a "download" link
    # nobody without CAD software can act on. Left un-inlined (renders as the
    # existing not-available badge, no dead link — see artifactsBlock).
    if ext in _SKIP_INLINE_EXTS:
        return None
    # Other small non-image files (e.g. .txt) still inline as-is.
    if len(raw) <= _MAX_FILE_INLINE:
        return _data_uri(raw, ext)
    return None


# ------------------------------------------------------------- VQA landing ----
# The opening tab: real results from the master-VQA suite — the agent answering
# plain-English questions by picking and calling the right perception skill over
# MCP. Rendered here (not from item.json) so it reuses the .vqa/.usage styling
# the dashboard already ships without touching the data pipeline or the KPIs.
VQA_RESULTS = REPO / "dimos" / "e2e_tests" / "vqa_results.ignore.json"


def _md_inline(text: str) -> str:
    """Escape, then render the little markdown the agent emits (**bold**).

    Newlines are preserved by the .vqa-a { white-space: pre-wrap } rule.
    """
    from html import escape

    return re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", escape(text or ""))


def _fmt_int(n: int) -> str:
    return f"{n:,}"


def _vqa_html(rows: list[dict]) -> tuple[str, str] | None:
    """Return (tab_button_html, panel_html) for the VQA landing tab, or None."""
    if not rows:
        return None
    from html import escape

    tools: set[str] = set()
    for v in rows:
        tools.update(v.get("tool_calls") or [])
    total_tok = sum((v.get("tokens") or {}).get("total_tokens", 0) for v in rows)
    durs = sorted(v.get("duration_s") or 0 for v in rows)
    median = durs[len(durs) // 2] if durs else 0.0

    def stat(k: str, val: str) -> str:
        return f'<div class="stat"><div class="k">{k}</div><div class="v">{val}</div></div>'

    strip = (
        '<div class="usage" style="margin:2px 0 20px">'
        + stat("Questions", str(len(rows)))
        + stat("Skills the agent chose", str(len(tools)))
        + stat("Median answer time", f"{median:.1f} s")
        + stat("Total tokens", _fmt_int(total_tok))
        + "</div>"
    )

    items = []
    for v in rows:
        toolspans = "".join(
            f'<span class="tool">{escape(t)}</span>' for t in (v.get("tool_calls") or [])
        )
        dur = f'<span class="dur">{v["duration_s"]}s</span>' if v.get("duration_s") is not None else ""
        tok = (v.get("tokens") or {}).get("total_tokens")
        tokspan = f'<span class="toks">{_fmt_int(tok)} tok</span>' if tok else ""
        meta = toolspans + dur + tokspan
        items.append(
            '<div class="vqa-item">'
            f'<p class="vqa-q">{escape(v.get("question", ""))}</p>'
            f'<p class="vqa-a">{_md_inline(v.get("answer", ""))}</p>'
            f'<p class="vqa-meta">{meta}</p>'
            "</div>"
        )
    transcript = '<div class="vqa">' + "\n".join(items) + "</div>"

    tab = (
        '<button class="tab" role="tab" id="tab-vqa" aria-controls="panel-vqa" '
        'aria-selected="false"><span class="dot impl"></span>'
        '<span class="n">&#9654;</span>Live agent Q&amp;A</button>'
    )
    panel = (
        '<section class="panel" role="tabpanel" id="panel-vqa" aria-labelledby="tab-vqa" hidden>'
        '<div class="panel-head"><h2>Live agent Q&amp;A</h2>'
        '<span class="pill impl">Working end-to-end</span></div>'
        '<p class="lede">One agent, asked plain-English questions about a space it just explored — '
        "answering each by choosing and calling the right perception skill over MCP. No question "
        "names a tool; the agent decides. These are verbatim results from the master-VQA suite, "
        "exercising the four shipped skills together.</p>"
        f"{strip}{transcript}"
        '<p class="code-cap" style="margin-top:16px">Source: '
        "<code>dimos/e2e_tests/master_vqa_test.py</code> → "
        "<code>vqa_results.ignore.json</code>. Run it yourself with the "
        "&ldquo;Run the VQA suite&rdquo; command above.</p>"
        "</section>"
    )
    return tab, panel


# ---------------------------------------------------------------- one-liners --
# The digestible "At a glance" intro grafted above the tabs. Purpose text mirrors
# each item's oneline; kept here so the intro stays a compact summary.
ONELINERS = [
    ("1", "Architectural floorplan generation",
     "Give it a recording of a walk-through; get back a multi-story CAD drawing set of the building.",
     "impl", "Implemented"),
    ("2", "Identify, list &amp; locate observed items",
     "Navigate a space, then ask &ldquo;what have you seen, and where is it?&rdquo; — no target object required.",
     "impl", "Implemented"),
    ("3", "Convert lidar to numpy signals",
     "Turn the live lidar stream into plain numpy — and into the handful of measurements skills actually ask for.",
     "impl", "Implemented"),
    ("4", "Temporally count items",
     "Count how many of an item there are over time — held until item 6 lets it count activities too.",
     "hold", "On hold"),
    ("5", "Contextual labeling (VLM educated guesses)",
     "When the detector shrugs (&ldquo;object&rdquo;, low confidence), ask a VLM — with the context the detector never used.",
     "impl", "Implemented"),
    ("6", "Identify specific activity (&ldquo;human does X&rdquo;)",
     "Recognize a specific human activity from a temporal window — held on a missing rigged human asset / dataset.",
     "hold", "On hold"),
    ("7", "Count occurrences of activity over time",
     "Aggregate activity events into rates over time — purely downstream of item 6.",
     "hold", "On hold"),
    ("8", "Full 3D scene model with cross-sections",
     "The shared 3D substrate the floorplan is drawn from — sliceable into plan cuts and elevations.",
     "proto", "Prototype"),
]


def _intro_html() -> str:
    rows = "\n".join(
        f'<tr><td class="num" style="text-align:left;color:var(--muted)">{n}</td>'
        f'<td>{name}</td><td style="color:var(--ink-2)">{purpose}</td>'
        f'<td class="num"><span class="pill {cls}">{label}</span></td></tr>'
        for n, name, purpose, cls, label in ONELINERS
    )
    return f"""
  <section class="intro" aria-label="At a glance">
    <h3 style="margin-top:26px">At a glance — one line each</h3>
    <div class="tbl-scroll"><table>
      <tr><th class="num" style="text-align:left">#</th><th>Skill / objective</th><th>Purpose</th><th class="num">Status</th></tr>
      {rows}
    </table></div>
    <p class="code-cap">Every tab below renders its skill's full detail — how it works, agent-skill
      signatures, test results by scenario (unit + real-data replays), evidence, examples, and review
      artifacts — inlined from the live <code>misc/skills_dashboard/</code> data.</p>
    <div class="testcmd" style="margin-top:6px">
      <span class="tc-label">&#9654; Run the VQA suite</span>
      <code class="tc-cmd">uv run pytest dimos/e2e_tests/master_vqa_test.py -m mujoco</code>
      <span class="tc-note">boots DimSim + the scene-memory-agentic graph and loops the question bank
        through the full MCP skill-calling loop, exercising the shipped skills end-to-end. Recorded
        china-office replay variants: <code>-m self_hosted</code> (needs <code>OPENAI_API_KEY</code>
        + the recording).</span>
    </div>
    <p class="code-cap" style="margin-top:-14px">View or edit the question bank at
      <code>dimos/e2e_tests/fixtures/vqa_questions.txt</code> (recorded-session variant:
      <code>vqa_questions_recorded.txt</code> in the same folder) — one question per line.</p>
  </section>"""


# ---------------------------------------------------------------- JS patches --
# Rewire the dashboard's two fetch helpers to read from the inlined __DATA__ map,
# and make bust()/asset resolution serve inlined data: URIs.
_FETCHJSON_OLD = """async function fetchJSON(url) {
  const r = await fetch(bust(url), { cache: 'no-store' });
  if (!r.ok) throw new Error(`${url}: HTTP ${r.status}`);
  return r.json();
}"""
_FETCHJSON_NEW = """async function fetchJSON(url) {
  const v = window.__DATA__[url];
  if (v === undefined) throw new Error(`${url}: not inlined`);
  return JSON.parse(v);
}"""

_FETCHTEXT_OLD = """async function fetchTextOrNull(url) {
  try {
    const r = await fetch(bust(url), { cache: 'no-store' });
    return r.ok ? r.text() : null;
  } catch { return null; }
}"""
_FETCHTEXT_NEW = """async function fetchTextOrNull(url) {
  const v = window.__DATA__[url];
  return v === undefined ? null : v;
}"""

_BUST_OLD = """const BUST = Date.now();
const bust = url => url + (url.includes('?') ? '&' : '?') + 'v=' + BUST;"""
_BUST_NEW = """const BUST = 0;
const bust = url => {
  if (typeof url === 'string' && url.slice(0, 5) === 'data:') return url;
  return (window.__ASSETS__ || {})[url] || url;
};"""

# Review artifacts: a.file is a repo-relative path that means nothing in a
# browser viewing the artifact — only a.src (an inlined data: URI) is ever
# openable here. Rewrite the whole function so every anchor is conditional on
# actually having something to open; otherwise render plain text/an ext badge.
_ART_OLD = """function artifactsBlock(arts, captionHtml) {
  if (!arts || !arts.length) return '';
  const isImg = f => /\\.(png|jpe?g|gif|webp|svg)$/i.test(f);
  const cells = arts.map(a => {
    const href = esc(a.file);
    const label = esc(a.label || a.file.split('/').pop());
    const note = a.note ? `<span class="art-note">${esc(a.note)}</span>` : '';
    const preview = isImg(a.file)
      ? `<a class="art-thumb" href="${href}" target="_blank" rel="noopener"><img src="${href}" alt="${label}" loading="lazy" onerror="this.closest('.art-thumb').classList.add('empty')"></a>`
      : `<a class="art-thumb art-file" href="${href}" target="_blank" rel="noopener"><span class="art-ext">${esc((a.file.split('.').pop() || 'file').toUpperCase())}</span></a>`;
    return `<figure class="art">${preview}<figcaption><a href="${href}" target="_blank" rel="noopener"><code>${label}</code></a>${note}</figcaption></figure>`;
  }).join('');
  return `${captionHtml ? `<p class="code-cap">${captionHtml}</p>` : ''}<div class="artifacts">${cells}</div>`;
}"""
_ART_NEW = """function artifactsBlock(arts, captionHtml) {
  if (!arts || !arts.length) return '';
  const isImg = f => /\\.(png|jpe?g|gif|webp|svg)$/i.test(f);
  const cells = arts.map(a => {
    const href = a.src || '';  // only an inlined data: URI is ever openable here
    const label = esc(a.label || a.file.split('/').pop());
    const note = a.note ? `<span class="art-note">${esc(a.note)}</span>` : '';
    const ext = esc((a.file.split('.').pop() || 'file').toUpperCase());
    let preview;
    if (isImg(a.file) && href) {
      preview = `<a class="art-thumb" href="${esc(href)}" target="_blank" rel="noopener"><img src="${esc(href)}" alt="${label}" loading="lazy"></a>`;
    } else if (isImg(a.file)) {
      preview = `<span class="art-thumb empty"></span>`;
    } else if (href) {
      preview = `<a class="art-thumb art-file" href="${esc(href)}" target="_blank" rel="noopener"><span class="art-ext">${ext}</span></a>`;
    } else {
      preview = `<span class="art-thumb art-file"><span class="art-ext">${ext}</span></span>`;
    }
    const capLabel = href ? `<a href="${esc(href)}" target="_blank" rel="noopener"><code>${label}</code></a>` : `<code>${label}</code>`;
    return `<figure class="art">${preview}<figcaption>${capLabel}${note}</figcaption></figure>`;
  }).join('');
  return `${captionHtml ? `<p class="code-cap">${captionHtml}</p>` : ''}<div class="artifacts">${cells}</div>`;
}"""

# Skills table "File" column: a repo-relative path, dead in the artifact.
_SKILLFILE_OLD = '<td class="mono"><a class="filelink" href="/${esc(s.file)}">${esc(s.file)}</a></td>'
_SKILLFILE_NEW = '<td class="mono"><code>${esc(s.file)}</code></td>'

# Panel footer "Source:" file list + "data:" folder link — both repo-relative.
_SOURCES_OLD = """  const sources = (item.sources || []).length
    ? `<p class="files">Source: ${item.sources.map(s =>
        `<a href="/${esc(s)}"><code>${esc(s)}</code></a>`).join(' · ')} ·
        data: <a href="${esc(dir)}/"><code>${esc(dir)}/</code></a></p>`
    : `<p class="files">data: <a href="${esc(dir)}/"><code>${esc(dir)}/</code></a></p>`;"""
_SOURCES_NEW = """  const sources = (item.sources || []).length
    ? `<p class="files">Source: ${item.sources.map(s =>
        `<code>${esc(s)}</code>`).join(' · ')} ·
        data: <code>${esc(dir)}/</code></p>`
    : `<p class="files">data: <code>${esc(dir)}/</code></p>`;"""

# Footer note is dashboard-maintainer instructions ("republish those... reload;
# this page needs no edits") that don't apply to a static artifact snapshot,
# plus two more dead repo-relative links — drop it entirely in the artifact.
_FOOTERNOTE_OLD = """    document.getElementById('footer-note').innerHTML =
      `Each tab renders <code>item.json</code> + <code>results.xml</code> + figures from its folder under
       <a class="filelink" href="data/">data/</a> — republish those (see the "Publishing to the dashboard"
       section of <a class="filelink" href="/dimos/skills/SKILLS_WISHLIST.md">SKILLS_WISHLIST.md</a>) and
       reload; this page needs no edits.`;"""
_FOOTERNOTE_NEW = "    // footer note omitted from the shared artifact (dashboard-maintainer instructions)"

# Prepend the Live-Q&A tab + panel (built in Python, injected as window vars) so
# it's the first tab and therefore the default-selected landing view. The
# dashboard's own tab logic (select(tabs[0]) at boot) then activates it, and
# the KPI math — which iterates `loaded` — never sees it.
_TABS_OLD = "document.getElementById('tabs').innerHTML = loaded.map((l, i) => {"
_TABS_NEW = "document.getElementById('tabs').innerHTML = (window.__VQA_TAB__ || '') + loaded.map((l, i) => {"
_PANELS_OLD = """    document.getElementById('panels').innerHTML =
      loaded.map((l, i) => panelHTML(i + 1, l.dir, l.item, l.junit, l.conceptHtml, SCENARIOS)).join('\\n');"""
_PANELS_NEW = """    document.getElementById('panels').innerHTML = (window.__VQA_PANEL__ || '') +
      loaded.map((l, i) => panelHTML(i + 1, l.dir, l.item, l.junit, l.conceptHtml, SCENARIOS)).join('\\n');"""

# Drop the "▶ Test it" command boxes. They point at a local `uv run …`
# invocation that's only actionable in the live dashboard — on the shared
# read-only artifact a viewer can't run it, so the box is noise. Neutralize
# the renderer rather than scrubbing every item's test_command from the data.
_TESTCMD_OLD = "function testCmdHTML(item) {\n  const t = item.test_command;"
_TESTCMD_NEW = (
    "function testCmdHTML(item) {\n"
    "  return '';  // 'Test it' boxes omitted from the shared artifact (not runnable there)\n"
    "  const t = item.test_command;"
)

# ------------------------------------------------------------- header swap ---
# The live dashboard is a generic backlog tool ("Skills Wishlist Dashboard").
# The artifact is a wrap-up deliverable, so give it the richer sprint framing —
# applied only here, not in index.html.
_HEAD_OLD = (
    '<p class="eyebrow">dimos · robot skills backlog · '
    '<span id="updated-note">loading…</span></p>\n'
    '      <h1>Skills Wishlist Dashboard</h1>'
)
_HEAD_NEW = (
    '<p class="eyebrow">dimos · robot perception &amp; scene understanding · '
    '<span id="updated-note">loading…</span></p>\n'
    '      <h1>Eight skills to enhance robot perception</h1>'
)
_SUB_OLD = (
    '<p class="sub">Candidate skills for reasoning about the environment the robot observes —\n'
    '  mapping, object &amp; scene understanding, and activity over time. Source of truth:\n'
    '  <a class="filelink" href="/dimos/skills/SKILLS_WISHLIST.md">dimos/skills/SKILLS_WISHLIST.md</a>;\n'
    '  this page renders whatever is published under\n'
    '  <a class="filelink" href="data/">misc/skills_dashboard/data/</a>.</p>'
)
_SUB_NEW = (
    '<p class="sub">Eight backlog items for reasoning about the space a robot observes —\n'
    '  reconstruct it, catalog what&rsquo;s in it, measure it, and reason about it over time.\n'
    '  Outcome so far: <b>four agent-callable skills</b> implemented and wired as MCP tools, a shared\n'
    '  <b>3D SceneModel</b> they slice and back-project against, and a <b>three-item activity track</b>\n'
    '  parked on a specific asset gap. Each skill&rsquo;s verification test drives the real code, not a\n'
    '  mock (broader test coverage still in progress); item&nbsp;2 is additionally replay-validated over\n'
    '  a recorded 87&nbsp;s Go2 session.</p>'
)

# ------------------------------------------------ group the on-hold items ----
# The live dashboard gives each backlog item its own tab. In the artifact the
# three parked items (4, 6, 7) collapse into one "On hold" tab so they don't eat
# the tab bar — the same grouping the earlier wrap-up had.
_SCEN_OLD = "function scenariosHTML(idx, item, junit, scenarios) {\n  const g = `st-${idx}`;"
_SCEN_NEW = ("function scenariosHTML(idx, item, junit, scenarios) {\n"
             "  if (item.conceptual) return '';\n  const g = `st-${idx}`;")

# Keep the "on hold" KPI at the true backlog count (3) even though the three
# items share one tab, and count "Items tracked" as the real backlog size.
_KPI_OLD = """const active = loaded.filter(l => ACTIVE_STATUSES.has(l.item.status)).length;
    const onHold = loaded.filter(l => l.item.status === 'hold').length;
    const ideas = loaded.length - active - onHold;"""
_KPI_NEW = """const active = loaded.filter(l => ACTIVE_STATUSES.has(l.item.status)).length;
    const onHold = loaded.reduce((a, l) => a + (l.item.hold_count || (l.item.status === 'hold' ? 1 : 0)), 0);
    const ideas = loaded.filter(l => l.item.status === 'idea').length;
    const tracked = active + onHold + ideas;"""
_KPI_TRACKED_OLD = '<div class="kpi"><div class="label">Items tracked</div><div class="value">${loaded.length}</div>'
_KPI_TRACKED_NEW = '<div class="kpi"><div class="label">Items tracked</div><div class="value">${tracked}</div>'


def _combined_hold_item(items_by_slug: dict) -> dict:
    """Fold the parked items (4, 6, 7) into one panel of cards."""
    def summ(slug: str) -> str:
        return (items_by_slug.get(slug) or {}).get("summary_html", "")

    it6 = items_by_slug.get("06-activity-recognition") or {}
    built = next((c["html"] for c in it6.get("cards", []) if "built" in c.get("title", "").lower()),
                 it6.get("hold_note_html", ""))
    return {
        "title": "The activity track — on hold",
        "tab_label": "On hold · 4·6·7",
        "status": "hold",
        "conceptual": True,
        "hold_count": 3,
        "summary_html": (
            "<b>Recognizing activities</b> — what a person is <em>doing</em>, not just what objects are "
            "present — is gated on a specific, fixable asset gap, so items 4, 6 and 7 are parked together "
            "rather than half-shipped. The data-generation harness that unblocks them is already built and "
            "passing."
        ),
        "cards_heading": "The three parked items",
        "cards": [
            {"title": "Item 4 — Temporally count items",
             "html": summ("04-temporal-item-counts")
             + " Held until item&nbsp;6 lets <em>activities</em>, not just items, be counted the same way."},
            {"title": "Item 6 — Identify specific activity",
             "html": summ("06-activity-recognition")
             + " Blocked on a <b>rigged, animated human asset</b> in DimSim (the repo has one unrigged model, "
               "so every activity renders as the same static pose) or a <b>real annotated activity dataset</b>."},
            {"title": "Item 7 — Count activity over time",
             "html": summ("07-activity-counts")},
            {"title": "What was built anyway (and passes)", "html": built},
        ],
        "dependencies": [
            "item 2 — detection + ObjectDB groundwork",
            "rigged human asset OR annotated activity dataset — the blocking gap",
        ],
        "hold_note_html": (
            "Resume once either (a) a rigged, animated human asset (or several) lands in DimSim, or (b) a real "
            "annotated activity dataset is on hand — whichever makes the harness&rsquo;s output useful for "
            "training/eval rather than a proof of plumbing."
        ),
    }


def build() -> int:
    if not INDEX.exists():
        print(f"index not found: {INDEX}", file=sys.stderr)
        return 1

    html = INDEX.read_text()
    index_json = json.loads((DATA / "items.json").read_text())
    slugs = index_json["items"]

    data_map: dict[str, str] = {"data/items.json": json.dumps(index_json)}
    assets: dict[str, str] = {}
    scen = DATA / "scenarios.json"
    if scen.exists():
        data_map["data/scenarios.json"] = scen.read_text()

    missing_assets: list[str] = []
    items_by_slug: dict[str, dict] = {}

    for slug in slugs:
        item_dir = DATA / slug
        dir_url = f"data/{slug}"
        item = json.loads((item_dir / "item.json").read_text())
        items_by_slug[slug] = item

        # Figures -> ASSETS keyed exactly as figureHTML builds the src.
        # thumb=False: figures are the page's hero images shown at full width,
        # so keep their original pixels (a locally-bundled report has no size
        # ceiling to worry about).
        for fig in item.get("figures", []):
            for key in ("light", "dark"):
                name = fig.get(key)
                if not name:
                    continue
                uri = _inline_asset(name, item_dir, thumb=False)
                if uri:
                    assets[f"{dir_url}/{name}"] = uri
                else:
                    missing_assets.append(f"{slug}:{name}")

        # Review artifacts (top-level + per-scenario): add an inlined `src`.
        def _do_arts(arts: list[dict]) -> None:
            for a in arts or []:
                ref = a.get("file", "")
                uri = _inline_asset(ref, item_dir, thumb=True)
                if uri:
                    a["src"] = uri
                elif _fs_path(ref, item_dir).suffix.lower() not in ("", ".rrd"):
                    missing_assets.append(f"{slug}:{ref}")

        _do_arts(item.get("review_artifacts", []))
        for sc in (item.get("scenarios") or {}).values():
            if isinstance(sc, dict):
                _do_arts(sc.get("review_artifacts", []))

        data_map[f"{dir_url}/item.json"] = json.dumps(item)

        results = item_dir / "results.xml"
        if results.exists():
            data_map[f"{dir_url}/results.xml"] = results.read_text()

        concept = item.get("concept_file")
        if concept and (item_dir / concept).exists():
            data_map[f"{dir_url}/{concept}"] = (item_dir / concept).read_text()

    # --- collapse the parked items (status "hold") into one "On hold" tab ---
    hold_slugs = [s for s in slugs if items_by_slug[s].get("status") == "hold"]
    if hold_slugs:
        kept = [s for s in slugs if s not in hold_slugs] + ["_onhold"]
        index_json["items"] = kept
        data_map["data/items.json"] = json.dumps(index_json)
        data_map["data/_onhold/item.json"] = json.dumps(_combined_hold_item(items_by_slug))

    # --- Live agent Q&A landing tab (from the master-VQA results) ---
    vqa_tab = vqa_panel = ""
    if VQA_RESULTS.exists():
        pair = _vqa_html(json.loads(VQA_RESULTS.read_text()))
        if pair:
            vqa_tab, vqa_panel = pair
    else:
        print(f"note: {VQA_RESULTS} not found — no Live Q&A tab", file=sys.stderr)

    # --- assemble the inlined-data script (escape </ to survive in <script>) ---
    def _emb(obj: object) -> str:
        return json.dumps(obj, ensure_ascii=False).replace("</", "<\\/")

    preload = (
        "<script>\n"
        f"window.__DATA__ = {_emb(data_map)};\n"
        f"window.__ASSETS__ = {_emb(assets)};\n"
        f"window.__VQA_TAB__ = {_emb(vqa_tab)};\n"
        f"window.__VQA_PANEL__ = {_emb(vqa_panel)};\n"
        "</script>\n"
    )

    # --- patch the dashboard markup + JS (artifact-only) ---
    for old, new in (
        (_FETCHJSON_OLD, _FETCHJSON_NEW),
        (_FETCHTEXT_OLD, _FETCHTEXT_NEW),
        (_BUST_OLD, _BUST_NEW),
        (_ART_OLD, _ART_NEW),
        (_SKILLFILE_OLD, _SKILLFILE_NEW),
        (_SOURCES_OLD, _SOURCES_NEW),
        (_FOOTERNOTE_OLD, _FOOTERNOTE_NEW),
        (_TABS_OLD, _TABS_NEW),
        (_PANELS_OLD, _PANELS_NEW),
        (_TESTCMD_OLD, _TESTCMD_NEW),
        (_SCEN_OLD, _SCEN_NEW),
        (_KPI_OLD, _KPI_NEW),
        (_KPI_TRACKED_OLD, _KPI_TRACKED_NEW),
        (_HEAD_OLD, _HEAD_NEW),
        (_SUB_OLD, _SUB_NEW),
    ):
        if old not in html:
            print(f"ERROR: anchor not found (index.html changed?):\n---\n{old}\n---",
                  file=sys.stderr)
            return 2
        html = html.replace(old, new, 1)

    # inject preload right before the main script
    html = html.replace('<script>\n"use strict";', preload + '<script>\n"use strict";', 1)

    # graft the digestible intro just before the tabs
    html = html.replace('<div class="tabs" role="tablist"',
                        _intro_html() + '\n  <div class="tabs" role="tablist"', 1)

    # --- strip the document wrappers the Artifact host supplies ---
    title = re.search(r"<title>.*?</title>", html, re.S)
    style = re.search(r"<style>.*?</style>", html, re.S)
    body = re.search(r"<body>(.*?)</body>", html, re.S)
    if not (title and style and body):
        print("ERROR: could not locate <title>/<style>/<body> in index.html", file=sys.stderr)
        return 2
    fragment = f"{title.group(0)}\n{style.group(0)}\n{body.group(1).strip()}\n"

    OUTPUT.write_text(fragment)
    size_kb = len(fragment.encode()) / 1024
    print(f"wrote {OUTPUT.relative_to(REPO)}  ({size_kb:.0f} KB, "
          f"{len(assets)} assets inlined, {len(data_map)} data files)")
    if missing_assets:
        print(f"note: {len(missing_assets)} asset(s) not inlined (shown as placeholders): "
              + ", ".join(missing_assets[:8]) + (" …" if len(missing_assets) > 8 else ""),
              file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(build())
