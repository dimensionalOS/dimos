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

import os

from dimos.constants import DIMOS_PROJECT_ROOT

PATTERN = "importorskip"

IGNORED_DIRS = {
    ".venv",
    "venv",
    "__pycache__",
    "node_modules",
    ".git",
    "dist",
    "build",
    ".egg-info",
    ".tox",
}


def _is_ignored_dir(dirpath: str) -> bool:
    parts = dirpath.split(os.sep)
    return bool(IGNORED_DIRS.intersection(parts))


def find_importorskip_usages() -> list[tuple[str, int, str]]:
    """Return a list of (rel_path, line_number, line_text) for every violation."""
    dimos_dir = DIMOS_PROJECT_ROOT / "dimos"
    violations: list[tuple[str, int, str]] = []
    # Skip this test file: it necessarily contains the forbidden pattern.
    self_path = os.path.realpath(__file__)

    for dirpath, dirnames, filenames in os.walk(dimos_dir):
        dirnames[:] = [d for d in dirnames if d not in IGNORED_DIRS]

        if _is_ignored_dir(dirpath):
            continue

        for fname in filenames:
            if not fname.endswith(".py"):
                continue

            full_path = os.path.join(dirpath, fname)
            if os.path.realpath(full_path) == self_path:
                continue
            rel_path = os.path.relpath(full_path, DIMOS_PROJECT_ROOT)

            try:
                with open(full_path, encoding="utf-8", errors="replace") as f:
                    for lineno, line in enumerate(f, start=1):
                        stripped = line.rstrip("\n")
                        if PATTERN not in stripped:
                            continue
                        violations.append((rel_path, lineno, stripped))
            except (OSError, UnicodeDecodeError):
                continue

    return violations


def test_no_importorskip():
    """
    Fail if any file uses `pytest.importorskip`.

    `importorskip` silently skips a test (or a whole module) when an optional
    dependency is missing. That means a broken test, or a dependency missing
    from the dev/CI environment, goes unnoticed because the test never runs.
    Install all dependencies locally so the tests actually execute. If a
    dependency genuinely cannot be installed in some environment, exclude those
    tests explicitly (e.g. a CI `--ignore`, or a platform-conditioned
    `collect_ignore` in a conftest) rather than silently skipping per import.
    """
    violations = find_importorskip_usages()
    if violations:
        report_lines = [
            f"Found {len(violations)} forbidden use(s) of `pytest.importorskip`. "
            "Don't silently skip tests when a dependency is missing -- install all "
            "dependencies so the tests run, or exclude the module explicitly "
            "(CI `--ignore` / a platform `collect_ignore`):",
            "",
        ]
        for path, lineno, text in violations:
            report_lines.append(f"  {path}:{lineno}: {text.strip()}")
        raise AssertionError("\n".join(report_lines))
