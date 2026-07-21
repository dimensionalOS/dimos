"""Sphinx configuration for the DimOS documentation.

The Markdown and MDX files remain the source of truth.  This file only adds
the Sphinx presentation layer and a small compatibility pass for the handful
of Mintlify components used by the introduction and quickstart pages.
"""

from __future__ import annotations

import os
from pathlib import Path
import re

from docutils.parsers.rst import directives
from myst_parser.parsers.sphinx_ import MystParser
from sphinx.directives.other import Include

project = "Dimensional"
copyright = "2026, Dimensional Team"
author = "Dimensional Team"
release = ""

extensions = [
    "myst_parser",
    "sphinx_design",
]

# Keep the existing Markdown/MDX source directly renderable, and register the
# small non-Markdown Sphinx entry point used by ``master_doc``.
source_suffix = {".md": "markdown", ".mdx": "markdown", ".rst": "restructuredtext"}
master_doc = "index"
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "node_modules"]

# The legacy Markdown corpus intentionally contains heading jumps.  Suppress
# only the known legacy-source diagnostics below. Strict builds still surface
# broken links, missing documents, and all unrelated Sphinx/MyST warnings.
suppress_warnings = [
    "myst.header",
    "myst.xref_missing",  # repository links intentionally point to GitHub
]

myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "fieldlist",
    "linkify",
    "strikethrough",
    "substitution",
    "tasklist",
]
myst_heading_anchors = 3
myst_title_to_header = True

html_theme = "furo"
html_title = "Dimensional · DimOS"
html_file_suffix = "/index.html"
html_link_suffix = "/"
html_logo = "assets/dimensional-logo-master-transparent.png"
html_favicon = "assets/favicon.png"
html_static_path = ["_static"]
html_css_files = ["dimensional.css"]
html_js_files = ["searchtools-root.js"]
templates_path = ["_templates"]
html_theme_options = {
    "announcement": "DimOS documentation — build physical applications in Python.",
    "source_repository": "https://github.com/dimensionalOS/dimos",
    "source_branch": "main",
    "source_directory": "docs/",
    "sidebar_hide_name": True,
    "light_css_variables": {
        "color-brand-primary": "#1682a3",
        "color-brand-content": "#08708f",
        "color-brand-visited": "#7358a6",
    },
    "dark_css_variables": {
        "color-brand-primary": "#50c5df",
        "color-brand-content": "#68d4e9",
        "color-brand-visited": "#c3a9ff",
    },
}

nitpicky = False
nitpick_ignore = [("py:class", "Dimos")]


_COMPONENT_OPEN = re.compile(
    r"^(?P<indent>\s*)<(?P<name>Columns|CardGroup|Card)\b(?P<attrs>[^>]*)>\s*$"
)
_COMPONENT_CLOSE = re.compile(r"^\s*</(?P<name>Columns|CardGroup|Card)>\s*$")
_ATTRIBUTE = re.compile(
    r"(?P<key>title|href|cols|icon)=[{]?(?:\"(?P<quoted>[^\"]*)\"|(?P<braced>[^}\s]+))[}]?"
)
_ICON = re.compile(r"<Icon\b[^>]*/>")
_LEGACY_LEXERS = re.compile(
    r"^(?P<indent>\s*)(?P<fence>`{3,}|~{3,})(?P<lexer>results|pikchr|node|diagon)(?P<rest>.*)$",
    re.MULTILINE,
)
_IMAGE = re.compile(r"!\[(?P<alt>[^]]*)\]\((?P<path>[^)]+)\)")
_MINTLIFY_DOC_LINK = re.compile(
    r"(?P<opening>\()(?P<path>/docs/[^)#\s]+)\.md(?P<anchor>#[^)\s]+)?(?P<closing>\))"
)
_ROOT_RELATIVE_ATTRIBUTE = re.compile(
    r"(?P<prefix>\b(?:href|src|action|poster)=['\"])(?P<relative>(?:\.\./)+)(?P<path>[^'\"]*)"
)
_CODING_AGENTS_LANDING = '---\ntitle: "For Agents"\n---\n'
_INCLUDED_SOURCES: dict[Path, str] = {}
_DOCS_DIR = Path(__file__).parent.resolve()
_SYNTHETIC_TITLES = {
    "capabilities/memory/algo_comparison.md": "Algorithm comparison",
    "capabilities/memory/plot.md": "Plot",
    "development/conventions.md": "Conventions",
}


def _attribute(attrs: str, key: str) -> str | None:
    for match in _ATTRIBUTE.finditer(attrs):
        if match.group("key") == key:
            return match.group("quoted") or match.group("braced")
    return None


def _mintlify_components(text: str) -> str:
    """Translate the small Mintlify JSX subset into sphinx-design directives.

    Only lines outside fenced code blocks are changed.  This keeps examples
    such as ``export ROBOT_IP=<YOUR_ROBOT_IP>`` completely untouched.
    """

    output: list[str] = []
    in_fence = False
    inside_card = False
    for line in text.splitlines():
        stripped = line.lstrip()
        if stripped.startswith("```") or stripped.startswith("~~~"):
            in_fence = not in_fence
            output.append(line)
            continue
        if in_fence:
            output.append(line)
            continue

        close = _COMPONENT_CLOSE.match(line)
        if close:
            if close.group("name") == "Card":
                inside_card = False
            output.append("::::" if close.group("name") in {"Columns", "CardGroup"} else ":::")
            continue

        open_match = _COMPONENT_OPEN.match(line)
        if open_match:
            name = open_match.group("name")
            attrs = open_match.group("attrs")
            if name in {"Columns", "CardGroup"}:
                cols = _attribute(attrs, "cols") or "2"
                output.append(f"::::{{grid}} 1 1 {cols} {cols}")
            else:
                title = _attribute(attrs, "title") or ""
                href = _attribute(attrs, "href")
                icon = _attribute(attrs, "icon") or "spark"
                output.append(f":::\u007bgrid-item-card\u007d {title}")
                output.append(
                    f":class-card: dimos-card dimos-icon-{re.sub(r'[^a-z0-9-]', '-', icon.lower())}"
                )
                if href:
                    output.append(f":link: {_canonical_card_route(href)}")
                output.append("")
                inside_card = True
            continue

        if inside_card:
            # Mintlify indents Card bodies for JSX readability. That
            # indentation becomes a literal code block in MyST; remove it so
            # prose, emphasis, and inline links are parsed normally.
            output.append(line.lstrip())
            continue

        output.append(_ICON.sub("", line))
    return "\n".join(output)


def _canonical_card_route(href: str) -> str:
    """Keep external Card links intact and make internal links site-rooted."""

    if not href.startswith("/") or href.startswith("//"):
        return href
    route = href
    if route.startswith("/docs/"):
        route = route.removeprefix("/docs")
    route = route.removesuffix(".md")
    if route == "/index":
        return "/"
    route = route.removesuffix("/index")
    return f"{route.rstrip('/')}/"


def _normalise_legacy_source(text: str, source_path: Path) -> str:
    """Make known legacy-only syntax safe for the Sphinx renderer."""

    # These fences are valid in the repository's Markdown tooling but are not
    # Pygments lexers. Rendering them as plain text preserves the examples and
    # avoids hiding highlighting failures for every other language.
    text = _LEGACY_LEXERS.sub(
        lambda match: f"{match.group('indent')}{match.group('fence')}text{match.group('rest')}",
        text,
    )

    # This one pip requirement is intentionally written in install syntax,
    # not TOML string syntax. Keep the displayed command unchanged while
    # asking Pygments to render the fence as plain text.
    if source_path.as_posix().endswith("capabilities/manipulation/a750.md"):
        text = re.sub(r"^(\s*)(`{3,}|~{3,})toml\s*$", r"\1\2text", text, flags=re.MULTILINE)

    # A few code-authoring examples intentionally reference generated assets
    # that are not checked into this repository. Replace only those missing
    # local images with an honest inline note; available assets still render.
    if "coding-agents/docs/" in source_path.as_posix():
        parent = source_path.parent

        def image_or_note(match: re.Match[str]) -> str:
            image = (parent / match.group("path")).resolve()
            if image.is_file():
                return match.group(0)
            return f"*Image unavailable in this checkout: {match.group('alt')}*"

        text = _IMAGE.sub(image_or_note, text)
    return text


def _normalise_mintlify_links(text: str) -> str:
    """Map Mintlify absolute Markdown links to Sphinx directory routes."""

    def route(match: re.Match[str]) -> str:
        path = match.group("path").removeprefix("/docs")
        anchor = match.group("anchor") or ""
        return f"{match.group('opening')}{path}/{anchor}{match.group('closing')}"

    return _MINTLIFY_DOC_LINK.sub(route, text)


def _strip_frontmatter_title(text: str) -> str:
    """Remove only a Markdown frontmatter title for an RST-owned wrapper."""

    lines = text.splitlines(keepends=True)
    if not lines or lines[0].strip() != "---":
        return text
    try:
        end = next(index for index, line in enumerate(lines[1:], 1) if line.strip() == "---")
    except StopIteration:
        return text
    lines = lines[: end + 1]
    body = text.splitlines(keepends=True)[end + 1 :]
    frontmatter = [line for line in lines[1:-1] if not re.match(r"^\s*title\s*:", line)]
    if not frontmatter:
        return "".join(body)
    return "".join([lines[0], *frontmatter, lines[-1], *body])


def _coding_agents_landing(text: str) -> str:
    """Replace only the legacy coding-agent tree with a navigable catalog."""

    if not text.startswith(_CODING_AGENTS_LANDING):
        return text
    return (
        _CODING_AGENTS_LANDING
        + """
The coding-agent notes are organized as a small toolkit: start with the
workflow guides, then use the documentation utilities when you are changing
the docs themselves.

## Agent toolkit

::::{grid} 1 1 2 2
:::{grid-item-card} Worktrees
:class-card: dimos-card dimos-icon-spark
:link: /coding-agents/worktrees/

Create and use provisioned worktrees with `bin/worktree`.
:::
:::{grid-item-card} Style guidelines
:class-card: dimos-card dimos-icon-spark
:link: /coding-agents/style/

Code style and conventions for DimOS contributions.
:::
:::{grid-item-card} Code quality rules
:class-card: dimos-card dimos-icon-spark
:link: /coding-agents/code-quality-rules/

Rules that agents scan and fix during coding work.
:::
:::{grid-item-card} Testing guidelines
:class-card: dimos-card dimos-icon-spark
:link: /coding-agents/testing/

How to write and run the project test suite.
:::
:::{grid-item-card} Documentation guides
:class-card: dimos-card dimos-icon-spark
:link: /coding-agents/docs/index/

Executable code blocks, doc links, and diagrams for writing docs.
:::
::::
"""
    )


def _normalise_markdown_source(text: str, source_path: Path) -> str:
    """Apply the Markdown compatibility passes exactly once per parse."""

    text = _normalise_mintlify_links(text)
    if source_path.as_posix().endswith("docs/coding-agents/index.md"):
        text = _coding_agents_landing(text)
    text = _normalise_legacy_source(text, source_path)
    if source_path.suffix == ".mdx":
        text = _mintlify_components(text)
    key = source_path.relative_to(_DOCS_DIR).as_posix()
    if key in _SYNTHETIC_TITLES and not re.match(r"^\s*#\s", text):
        title = _SYNTHETIC_TITLES[key]
        text = f"{title}\n{'=' * len(title)}\n\n{text}"
    return text


def _relativise_included_images(text: str, source_path: Path, parent_path: Path) -> str:
    """Resolve included Markdown images from the including RST document."""

    source_dir = source_path.parent
    parent_dir = parent_path.parent

    def image_path(match: re.Match[str]) -> str:
        path = match.group("path")
        if path.startswith(("/", "#", "data:", "http://", "https://")):
            return match.group(0)
        resolved = os.path.relpath(source_dir / path, parent_dir)
        return match.group(0).replace(path, resolved, 1)

    return _IMAGE.sub(image_path, text)


def _prepare_source(app, docname: str, source: list[str]) -> None:
    source_path = (Path(app.srcdir) / app.env.doc2path(docname)).resolve()
    # Mintlify links are rooted at /docs, while Sphinx serves docs at /. Only
    # normalize Markdown/MDX content; the RST root legitimately contains the
    # literal ``coding-agents/docs/...`` document names.
    if source_path.suffix in {".md", ".mdx"}:
        source[0] = _normalise_markdown_source(source[0], source_path)


def _prepare_included_source(app, relative_path, parent_docname: str, source: list[str]) -> None:
    """Apply Markdown compatibility passes to parser-qualified RST includes."""

    source_path = (Path(app.srcdir) / relative_path).resolve()
    if source_path.suffix in {".md", ".mdx"}:
        source[0] = _normalise_markdown_source(source[0], source_path)
        parent_path = (Path(app.srcdir) / app.env.doc2path(parent_docname)).resolve()
        source[0] = _relativise_included_images(source[0], source_path, parent_path)


class _IncludedMystParser(MystParser):
    """Feed include-read's transformed source to a parser-qualified include."""

    def parse(self, inputstring, document) -> None:
        source_path = Path(document["source"]).resolve()
        super().parse(_INCLUDED_SOURCES.get(source_path, inputstring), document)


class _IncludeWithReadEvent(Include):
    """Make Sphinx's parser-qualified include emit its normal include event."""

    def run(self):
        parser = self.options.get("parser")
        if parser is MystParser and self.arguments[0].lower().endswith((".md", ".mdx")):
            _relative_path, filename = self.env.relfn2path(self.arguments[0])
            source_path = Path(filename).resolve()
            text = source_path.read_text(encoding=self.state.document.settings.input_encoding)
            if "body-only" in self.options:
                text = _strip_frontmatter_title(text)
            arg = [text]
            relative_path = Path(os.path.relpath(source_path, self.env.srcdir))
            self.env.app.events.emit("include-read", relative_path, self.env.docname, arg)
            _INCLUDED_SOURCES[source_path] = arg[0]
            self.options["parser"] = _IncludedMystParser
            try:
                return super().run()
            finally:
                self.options["parser"] = parser
                _INCLUDED_SOURCES.pop(source_path, None)
        return super().run()


def _publish_root_index(app, exception) -> None:
    """Expose the master document at ``/`` as well as ``/index/``."""

    if exception is None:
        generated = Path(app.outdir) / "index" / "index.html"
        root = Path(app.outdir) / "index.html"
        if generated.is_file():
            rendered = generated.read_text(encoding="utf-8")
            # The master document is rendered in /index/, so Sphinx emits
            # ../-relative URLs. Once copied to /index.html those escape the
            # site root. Rewrite only URL-bearing HTML attributes; external
            # URLs and fragments are unaffected.
            rendered = _ROOT_RELATIVE_ATTRIBUTE.sub(
                lambda match: f"{match.group('prefix')}/{match.group('path')}",
                rendered,
            )
            root.write_text(rendered, encoding="utf-8")


def _trim_markdown_tocs(app, env) -> None:
    """Keep Markdown page titles, not their body headings, in Furo's tree."""

    for docname, toc in env.tocs.items():
        source_path = (Path(app.srcdir) / env.doc2path(docname)).resolve()
        if source_path.suffix in {".md", ".mdx"} and toc.children:
            toc.children[:] = toc.children[:1]


def setup(app):
    # The parser-qualified Docutils include uses Sphinx's outer RST directive;
    # expose MyST's include options there as well.
    Include.option_spec["relative-images"] = directives.flag
    Include.option_spec["relative-docs"] = directives.path
    Include.option_spec["body-only"] = directives.flag
    directives.register_directive("include", _IncludeWithReadEvent)
    app.connect("source-read", _prepare_source)
    app.connect("include-read", _prepare_included_source)
    app.connect("env-updated", _trim_markdown_tocs)
    app.connect("build-finished", _publish_root_index)
    return {"version": "1", "parallel_read_safe": True}
