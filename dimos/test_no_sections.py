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
import re

from dimos.constants import DIMOS_PROJECT_ROOT

REPO_ROOT = str(DIMOS_PROJECT_ROOT)

# Matches lines that are purely separator characters (=== or ---) with optional
# whitespace, e.g.: # ============= or  # ---------------
SEPARATOR_LINE = re.compile(r"^\s*#\s*[-=]{10,}\s*$")

# Matches section headers wrapped in separators, e.g.:
#   # === My Section ===   or   # ===== My Section =====
INLINE_SECTION = re.compile(r"^\s*#\s*[-=]{3,}.+[-=]{3,}\s*$")

# VS Code-style region markers
REGION_MARKER = re.compile(r"^\s*#\s*(region|endregion)\b")

SCANNED_EXTENSIONS = {
    ".py",
    ".yml",
    ".yaml",
}

SCANNED_PREFIXES = {
    "Dockerfile",
}

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
    # third-party vendored code
    "gtsam",
    # auto-generated enhancement artifacts
    ".ignore.enhance",
}

# Lines that match section patterns but are actually programmatic / intentional.
# Each entry is (relative_path, line_substring) — if both match, the line is skipped.
WHITELIST = [
    # Sentinel marker used at runtime to detect already-converted Dockerfiles
    ("dimos/core/docker_module.py", "DIMOS_SENTINEL"),
]


def _should_scan(path: str) -> bool:
    basename = os.path.basename(path)
    _, ext = os.path.splitext(basename)
    if ext in SCANNED_EXTENSIONS:
        return True
    for prefix in SCANNED_PREFIXES:
        if basename.startswith(prefix):
            return True
    return False


def _is_ignored_dir(dirpath: str) -> bool:
    parts = dirpath.split(os.sep)
    return bool(IGNORED_DIRS.intersection(parts))


def _is_whitelisted(rel_path: str, line: str) -> bool:
    for allowed_path, allowed_substr in WHITELIST:
        if rel_path == allowed_path and allowed_substr in line:
            return True
    return False


def find_section_markers() -> list[tuple[str, int, str]]:
    """Return a list of (file, line_number, line_text) for every section marker."""
    violations: list[tuple[str, int, str]] = []

    for dirpath, dirnames, filenames in os.walk(REPO_ROOT):
        # Prune ignored directories in-place
        dirnames[:] = [d for d in dirnames if d not in IGNORED_DIRS]

        if _is_ignored_dir(dirpath):
            continue

        rel_dir = os.path.relpath(dirpath, REPO_ROOT)

        for fname in filenames:
            full_path = os.path.join(dirpath, fname)
            rel_path = os.path.join(rel_dir, fname)

            if not _should_scan(full_path):
                continue

            try:
                with open(full_path, encoding="utf-8", errors="replace") as f:
                    for lineno, line in enumerate(f, start=1):
                        stripped = line.rstrip("\n")
                        if _is_whitelisted(rel_path, stripped):
                            continue
                        if (
                            SEPARATOR_LINE.match(stripped)
                            or INLINE_SECTION.match(stripped)
                            or REGION_MARKER.match(stripped)
                        ):
                            violations.append((rel_path, lineno, stripped))
            except (OSError, UnicodeDecodeError):
                continue

    return violations


def test_no_section_markers():
    """
    Fail if any file contains section-style comment markers.

    If a file is too complicated to be understood without sections, then the
    sections should be files. We don't need "subfiles".
    """
    violations = find_section_markers()
    if violations:
        report_lines = [
            f"Found {len(violations)} section marker(s). "
            "If a file is too complicated to be understood without sections, "
            'then the sections should be files. We don\'t need "subfiles".',
            "",
        ]
        for path, lineno, text in violations:
            report_lines.append(f"  {path}:{lineno}: {text.strip()}")
        raise AssertionError("\n".join(report_lines))
