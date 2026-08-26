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
README = REPO_ROOT / "README.md"
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


def _page_url(path: str) -> str:
    """docs/usage/modules.md -> usage/modules/, the url mkdocs actually builds.

    Only needed for raw html attributes: mkdocs rewrites markdown links itself,
    but never looks inside an <a href> or an <img src>.
    """
    page = re.sub(r"^docs/", "", path).removesuffix(".md")
    page = re.sub(r"(^|/)index$", r"\1", page)
    return page.rstrip("/") + "/" if page else "."


def _readme_as_home() -> str:
    """The repo README, with its links pointed at the site instead of github."""
    text = README.read_text(encoding="utf-8")

    # Badges, star counts and the trendshift ribbon are furniture for a repo
    # landing page. Here they are a wall of images above the first sentence.
    text = re.sub(r"^\[!\[.*img\.shields\.io.*$", "", text, flags=re.M)
    text = re.sub(r"^!\[.*img\.shields\.io.*$", "", text, flags=re.M)
    text = re.sub(r"^<a href=\"https://trendshift\.io.*$", "", text, flags=re.M)

    # A 1px transparent gif forcing a minimum column width is a github table
    # hack. Here the cells already carry width="20%", and the spacer only adds
    # a line box, plus a lightbox anchor around a blank image.
    text = re.sub(r"\s*<img[^>]*spacer\.png[^>]*>", "", text)

    # <big> is deprecated, and being an inline tag it re-blocks markdown even
    # inside a div that asked for it. Material sizes the banner text anyway.
    text = text.replace("<big>", "").replace("</big>", "")

    # Github renders markdown inside a raw html block; python-markdown only
    # does so when the tag asks for it, which is what md_in_html reads.
    # Only on <div>: md_in_html does not treat a <td> as a markdown block, so
    # the attribute would survive into the output as a stray one.
    text = re.sub(r"<div\b(?![^>]*\bmarkdown=)([^>]*)>", r'<div\1 markdown="1">', text)

    # Raw html links into the docs tree, which mkdocs leaves alone.
    text = re.sub(
        r'(href=")docs/([^"]+\.md)"', lambda m: f'{m.group(1)}{_page_url(m.group(2))}"', text
    )
    # Raw html links to source, which have no page on the site at all.
    text = re.sub(
        r'(href=")((?:' + "|".join(SOURCE_ROOTS) + r')/[^"]*)"',
        lambda m: f'{m.group(1)}{GITHUB_BLOB}/{m.group(2)}"',
        text,
    )

    # docs/usage/modules.md -> usage/modules.md, since the site root is docs/.
    text = re.sub(r"\]\(docs/", "](", text)
    # Anything else in the repo is source, and lives on github.
    text = re.sub(
        r"\]\((AGENTS\.md|CONTRIBUTING\.md|LICENSE|(?:dimos|examples|bin|native|misc|scripts)/[^)#]*)",
        lambda m: f"]({GITHUB_BLOB}/{m.group(1)}",
        text,
    )
    # The readme opens with a centred banner rather than a heading, so give the
    # page a title for the nav and the browser tab.
    return f'---\ntitle: "Welcome to dimOS"\n---\n\n{text}'


def on_files(files, config):
    """Ship the theme and the readme-as-home without adding files to docs/."""
    from mkdocs.structure.files import File

    files.append(File.generated(config, "assets/mkdocs-theme.css", content=THEME_CSS.read_text()))
    files.append(File.generated(config, "index.md", content=_readme_as_home()))

    # The readme's screenshots live outside docs/, so pull them in by path
    # rather than copying 43MB into the docs tree.
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
