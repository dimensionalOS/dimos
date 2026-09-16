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
from dimos.deps.imports import ImportKind, is_test_file, iter_source_files, scan_file

DIMOS_DIR = DIMOS_PROJECT_ROOT / "dimos"

# Importing any of these at module level loads a heavyweight native library into
# every process that transitively imports the module - including worker
# processes that never touch it. Import them inside the function/method that
# uses them (or under `if TYPE_CHECKING:` for annotations only).
HEAVY_MODULES = ("cv2", "open3d", "rerun")

# Function-local imports are deferred and TYPE_CHECKING blocks never run;
# everything else (including try/except guards and __main__ blocks) executes
# at import time and counts as eager.
DEFERRED_KINDS = frozenset({ImportKind.LAZY, ImportKind.TYPE_ONLY})


def find_eager_heavy_imports(root: Path = DIMOS_DIR) -> dict[str, list[tuple[int, str]]]:
    """Map of dimos-relative file path -> [(line, module)] for eager heavy imports."""
    hits: dict[str, list[tuple[int, str]]] = {}
    for path in iter_source_files(root, exclude=is_test_file):
        scan = scan_file(path, project_root=root.parent)
        for site in scan.imports:
            if site.kind in DEFERRED_KINDS or site.top_level not in HEAVY_MODULES:
                continue
            hits.setdefault(str(path.relative_to(root)), []).append((site.lineno, site.module))
    return hits


def test_heavy_imports_are_inline() -> None:
    """Fail if any file imports cv2/open3d/rerun at module level."""
    hits = find_eager_heavy_imports()
    if hits:
        listing = "\n".join(
            f"  - dimos/{f}:{line}: `{module}`"
            for f, lines in sorted(hits.items())
            for line, module in lines
        )
        raise AssertionError(
            f"Found module-level import(s) of {'/'.join(HEAVY_MODULES)}:\n{listing}\n\n"
            "These libraries load large native extensions into every process that "
            "transitively imports the module. Import them inside the function or "
            "method that uses them; imports needed only for type annotations go "
            "under `if TYPE_CHECKING:`."
        )


def test_detection_rules(tmp_path: Path) -> None:
    """Guarded and __main__ imports are still violations; deferred ones are not."""
    root = tmp_path / "dimos"
    root.mkdir()
    (root / "mod.py").write_text(
        "from typing import TYPE_CHECKING\n"
        "try:\n"
        "    import cv2\n"
        "except ImportError:\n"
        "    pass\n"
        "if TYPE_CHECKING:\n"
        "    import open3d\n"
        "def f():\n"
        "    import rerun\n"
        'if __name__ == "__main__":\n'
        "    import open3d.core\n"
    )
    (root / "test_mod.py").write_text("import cv2\n")
    assert find_eager_heavy_imports(root) == {"mod.py": [(3, "cv2"), (11, "open3d.core")]}
