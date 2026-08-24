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

from pathlib import Path

from dimos.constants import DIMOS_PROJECT_ROOT

DOC_EXTENSIONS = {".md", ".mdx"}


def find_old_branding() -> list[tuple[Path, int, str]]:
    docs_dir = DIMOS_PROJECT_ROOT / "docs"
    hits: list[tuple[Path, int, str]] = []
    for path in sorted(docs_dir.rglob("*")):
        if path.suffix not in DOC_EXTENSIONS:
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        for lineno, line in enumerate(text.splitlines(), start=1):
            if "DimOS" in line:
                hits.append((path, lineno, line))
    return hits


def test_docs_use_current_branding() -> None:
    """Fail if any file under docs/ spells the brand "DimOS" instead of "dimOS"."""
    hits = find_old_branding()
    if hits:
        listing = "\n".join(
            f"  - {p.relative_to(DIMOS_PROJECT_ROOT)}:{lineno}: {line.strip()}"
            for p, lineno, line in hits
        )
        raise AssertionError(f'Found "DimOS" in docs/:\n{listing}\n\nThe brand is spelled "dimOS".')
