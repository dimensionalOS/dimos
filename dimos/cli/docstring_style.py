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

"""Reject non-Google docstring sections without checking types or presence."""

import ast
from collections.abc import Iterator
from pathlib import Path
import re
import sys
import tokenize

_NUMPY_SECTIONS = {
    "Parameters",
    "Other Parameters",
    "Returns",
    "Yields",
    "Receives",
    "Raises",
    "Warns",
    "Warnings",
    "Attributes",
    "Methods",
    "See Also",
    "Notes",
    "Examples",
    "References",
}
_FIELD = re.compile(
    r"^(?::(?:param(?:eter)?|arg(?:ument)?|keyword|kwarg|type|ivar|cvar|var|vartype|raise[sd]?|exception)\s+[^:]+:"
    r"|:(?:returns?|rtype|yields?|ytype):"
    r"|[@\\](?:param|type|ivar|cvar|var|returns?|rtype|yields?|raises?)\b)",
    re.IGNORECASE,
)
_CODE_DIRECTIVE = re.compile(r"^\.\. (?:code(?:-block)?|sourcecode|doctest)::")


def non_google_sections(docstring: str) -> Iterator[str]:
    """Yield unsupported section markers outside examples and literal blocks."""
    lines = docstring.splitlines()
    fence = ""
    literal_indent: int | None = None
    pending_literal: int | None = None
    for index, line in enumerate(lines):
        text = line.strip()
        if not text:
            continue
        indent = len(line) - len(line.lstrip())
        if fence:
            if len(text) >= len(fence) and set(text) == {fence[0]}:
                fence = ""
            continue
        if literal_indent is not None:
            if indent > literal_indent:
                continue
            literal_indent = None
        if pending_literal is not None:
            if indent > pending_literal:
                literal_indent = pending_literal
                pending_literal = None
                continue
            pending_literal = None
        opening_fence = re.match(r"(`{3,}|~{3,})", text)
        if opening_fence:
            fence = opening_fence[0]
            continue
        if text in {"Examples:", "Example:"} or _CODE_DIRECTIVE.match(text) or text.endswith("::"):
            pending_literal = indent
            continue
        if indent:
            continue
        if _FIELD.match(text):
            yield f"field {text.split(':', 2)[1] if text.startswith(':') else text.split()[0]}"
        if text in _NUMPY_SECTIONS and index + 1 < len(lines):
            underline = lines[index + 1]
            if (
                re.fullmatch(r"-{3,}", underline.strip())
                and len(underline) - len(underline.lstrip()) == indent
            ):
                yield f"underlined section {text!r}"


def check_file(path: Path) -> list[str]:
    """Return diagnostics for actual module, class, and function docstrings."""
    try:
        with tokenize.open(path) as stream:
            tree = ast.parse(stream.read(), filename=str(path))
    except (SyntaxError, UnicodeError) as error:
        return [f"{path}:1: DOCSTYLE could not parse Python: {error}"]
    errors = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            docstring = ast.get_docstring(node)
            if docstring:
                for section in non_google_sections(docstring):
                    errors.append(
                        f"{path}:{node.body[0].lineno}: DOCSTYLE {section}; use Google sections "
                        "such as Args:, Returns:, or Attributes:"
                    )
    return errors


def main() -> int:
    """Check the Python files supplied by pre-commit."""
    errors = [error for name in sys.argv[1:] for error in check_file(Path(name))]
    for error in errors:
        print(error)
    return bool(errors)


if __name__ == "__main__":
    sys.exit(main())
