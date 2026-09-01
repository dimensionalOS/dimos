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

"""Build-time shims that let mkdocs render docs/ unmodified.

Two repo conventions are not markdown conventions, so we translate them while
building instead of rewriting 73 documents:

- links are repo-root absolute (`/docs/usage/modules.md`, `/dimos/core/module.py`),
  the form `bin/doclinks` maintains and github renders natively
- code fences carry md-babel parameters (```python skip session=rx)

The site also serves the markdown it was built from, for agents: every page
at its docs/ path, indexed by llms.txt (overrides/llms.txt).
"""

from pathlib import Path
import posixpath
import re

from mkdocs.utils import write_file

THEME_CSS = Path(__file__).with_name("theme.css")
REPO_ROOT = Path(__file__).resolve().parents[2]
# The home page's demo gifs, shared with the readme rather than copied.
README_ASSETS = REPO_ROOT / "assets" / "readme"

GITHUB_BLOB = "https://github.com/dimensionalOS/dimos/blob/main"

# Repo-root absolute links that are source, not docs: these leave the site.
SOURCE_ROOTS = (
    "dimos",
    "examples",
    "bin",
    "native",
    "misc",
    "docker",
    "scripts",
    "web",
    ".github",
)


_FENCE = re.compile(r"^(?P<indent>[ \t]*)(?P<ticks>```+|~~~+)(?P<info>[^\n]*)$")
_LINK = re.compile(r"\]\((?P<target>/[^)\s]+)(?P<title>\s+\"[^\"]*\")?\)")


def _normalize_fences(markdown: str) -> str:
    """Keep the language, drop md-babel params pygments cannot parse.

    Only opening fences are touched. Nested fences are left verbatim so
    codeblocks.md can keep quoting md-babel syntax at readers.
    """
    out: list[str] = []
    open_ticks: str | None = None
    for line in markdown.split("\n"):
        match = _FENCE.match(line)
        if not match:
            out.append(line)
            continue
        ticks, info = match.group("ticks"), match.group("info").strip()
        if open_ticks is not None:
            # Only a same-or-longer run of the same char can close the block.
            if not info and ticks[0] == open_ticks[0] and len(ticks) >= len(open_ticks):
                open_ticks = None
            out.append(line)
            continue
        open_ticks = ticks
        lang = info.split()[0] if info else ""
        if lang and re.fullmatch(r"[A-Za-z0-9_+-]+", lang):
            line = f"{match.group('indent')}{ticks}{lang}"
        out.append(line)
    return "\n".join(out)


# Github renders `> [!IMPORTANT]` blockquotes as coloured callouts. Material
# has the same thing under a different syntax, so translate rather than ask
# anyone to write one form for the repo and another for the site.
_ALERT_TYPES = {
    "NOTE": "note",
    "TIP": "tip",
    "IMPORTANT": "info",
    "WARNING": "warning",
    "CAUTION": "danger",
}
_ALERT = re.compile(
    r"^(?P<indent>[ \t]*)> \[!(?P<kind>[A-Z]+)\][ \t]*\n"
    r"(?P<body>(?:(?P=indent)>[^\n]*\n?)*)",
    re.M,
)


def _github_alerts(markdown: str) -> str:
    def convert(match: re.Match[str]) -> str:
        kind = _ALERT_TYPES.get(match.group("kind"))
        if kind is None:
            return match.group(0)
        indent = match.group("indent")
        lines = []
        for line in match.group("body").rstrip("\n").split("\n"):
            text = re.sub(r"^[ \t]*> ?", "", line)
            lines.append(f"{indent}    {text}" if text else "")
        # Material has no "important" type, and its default titles differ from
        # github's labels, so carry the label over explicitly.
        label = match.group("kind").capitalize()
        return f'{indent}!!! {kind} "{label}"\n\n' + "\n".join(lines) + "\n"

    return _ALERT.sub(convert, markdown)


def _rewrite_link(match: re.Match[str], src_uri: str) -> str:
    target = match.group("target")
    title = match.group("title") or ""
    anchor = ""
    if "#" in target:
        target, _, anchor = target.partition("#")
        anchor = "#" + anchor

    if target.startswith("/docs/"):
        page = target[len("/docs/") :]
        # A link to a directory means its index page; relpath would otherwise
        # collapse a link to the page's own directory down to ".".
        if page.endswith("/"):
            page += "index.md"
        rel = posixpath.relpath(page, posixpath.dirname(src_uri))
        return f"]({rel}{anchor}{title})"

    root = target.lstrip("/").split("/", 1)[0]
    if root in SOURCE_ROOTS:
        return f"]({GITHUB_BLOB}{target}{anchor}{title})"

    return match.group(0)


def on_files(files, config):
    """Ship the theme and the shared demo assets without adding files to docs/."""
    from mkdocs.structure.files import File

    files.append(File.generated(config, "assets/mkdocs-theme.css", content=THEME_CSS.read_text()))

    # The home page's screenshots live outside docs/, beside the readme's, so
    # pull them in by path rather than copying 43MB into the docs tree.
    for asset in sorted(README_ASSETS.glob("*")):
        if asset.is_file():
            files.append(
                File.generated(config, f"assets/readme/{asset.name}", abs_src_path=str(asset))
            )
    return files


def on_page_markdown(markdown, page, config, files):
    markdown = _github_alerts(markdown)
    markdown = _normalize_fences(markdown)
    return _LINK.sub(lambda m: _rewrite_link(m, page.file.src_uri), markdown)


def on_post_page(output, page, config):
    """Serve the page's markdown beside its html: usage/modules.md next to usage/modules/."""
    write_file(page.markdown.encode(), str(Path(config.site_dir, page.file.src_uri)))
    return output
