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

"""Build API fixtures with the site's real handler options and inspect the HTML."""

from html.parser import HTMLParser
import logging
from pathlib import Path
import tempfile

from mkdocs.commands.build import build
from mkdocs.config import load_config

ROOT = Path(__file__).resolve().parents[2]


class PageText(HTMLParser):
    """Collect signature text separately from prose and embedded source code."""

    def __init__(self) -> None:
        super().__init__()
        self.signatures: list[list[str]] = []
        self.signature_depth = 0
        self.table_parts: list[str] = []
        self.table_depth = 0

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        classes = dict(attrs).get("class") or ""
        if tag == "table":
            self.table_depth += 1
        if "doc-signature" in classes.split():
            self.signatures.append([])
            self.signature_depth = 1
        elif self.signature_depth and tag not in {"br", "hr", "img", "input", "meta", "link"}:
            self.signature_depth += 1

    def handle_endtag(self, tag: str) -> None:
        if tag == "table":
            self.table_depth -= 1
        if self.signature_depth:
            self.signature_depth -= 1

    def handle_data(self, data: str) -> None:
        if self.signature_depth:
            self.signatures[-1].append(data)
        if self.table_depth:
            self.table_parts.append(data)


def main() -> None:
    """Verify functions, a dataclass constructor, and Pydantic field declarations."""
    logging.basicConfig(level=logging.INFO)
    with tempfile.TemporaryDirectory() as destination:
        config = load_config(
            config_file=str(ROOT / "mkdocs.yml"),
            docs_dir=str(ROOT / "misc/mkdocs/api_fixture"),
            site_dir=destination,
            hooks=[],
            nav=None,
            strict=True,
        )
        build(config)
        page = PageText()
        page.feed((Path(destination) / "index.html").read_text())
        signatures = ["".join("".join(parts).split()) for parts in page.signatures]
        for expected in (
            "typed(value:int)->int",
            "untyped(value)",
            "joint_names:list[str]",
            "hold_duration:float=0.0",
            "extents_open:SweepExtents",
            "family:GripperFamily='parallel_2f'",
        ):
            assert any(expected in signature for signature in signatures), (
                f"Rendered API signatures are missing {expected!r}: {signatures}"
            )
        tables = "".join(page.table_parts)
        assert "Number of closing steps." in tables
        assert "Ordinary joints this task owns, in command order." in tables
    print("API HTML shows Google sections, typed declarations, and a dataclass constructor.")


if __name__ == "__main__":
    main()
