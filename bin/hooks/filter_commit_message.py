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

import os
from pathlib import Path
import subprocess
import sys

# Patterns that trigger truncation (everything from this line onwards is removed)
TRUNCATE_PATTERNS = [
    "Generated with",
    "Co-Authored-By",
]


def filter_text(text: str) -> tuple[str, str | None]:
    """Return (filtered_text, first_matched_pattern_or_None)."""
    lines = text.splitlines(keepends=True)
    filtered_lines: list[str] = []
    matched: str | None = None
    for line in lines:
        hit = next((p for p in TRUNCATE_PATTERNS if p in line), None)
        if hit is not None:
            matched = hit
            break
        filtered_lines.append(line)
    return "".join(filtered_lines), matched


def rewrite_file(path: Path) -> int:
    if not path.exists():
        return 0
    filtered, _ = filter_text(path.read_text())
    path.write_text(filtered)
    return 0


def check_commits() -> int:
    """Check every commit in the range pre-commit was invoked over.

    Locally on `git commit` no range is supplied, so we no-op rather than
    blocking commits on the state of HEAD — the commit-msg hook is in
    charge there. In CI, code-cleanup.yml passes `--from-ref/--to-ref` to
    pre-commit, which exports PRE_COMMIT_FROM_REF / PRE_COMMIT_TO_REF.
    """
    from_ref = os.environ.get("PRE_COMMIT_FROM_REF")
    to_ref = os.environ.get("PRE_COMMIT_TO_REF")
    if not (from_ref and to_ref):
        return 0

    try:
        rev_list = subprocess.run(
            ["git", "rev-list", "--reverse", f"{from_ref}..{to_ref}"],
            capture_output=True,
            text=True,
            check=True,
        )
    except subprocess.CalledProcessError as e:
        print(
            f"git rev-list {from_ref}..{to_ref} failed: {e.stderr.strip() or e}",
            file=sys.stderr,
        )
        return 1

    failures: list[tuple[str, str]] = []
    for sha in rev_list.stdout.split():
        try:
            msg = subprocess.run(
                ["git", "log", "-1", "--format=%B", sha],
                capture_output=True,
                text=True,
                check=True,
            ).stdout
        except subprocess.CalledProcessError as e:
            print(
                f"git log -1 {sha} failed: {e.stderr.strip() or e}",
                file=sys.stderr,
            )
            return 1
        _, matched = filter_text(msg)
        if matched is not None:
            failures.append((sha, matched))

    if failures:
        for sha, pattern in failures:
            print(
                f"{sha[:12]}: contains forbidden pattern: {pattern!r}",
                file=sys.stderr,
            )
        print(
            "\nInstall the commit-msg hook "
            "(`pre-commit install -t commit-msg`) or amend the offending "
            "commits to strip the trailer.",
            file=sys.stderr,
        )
        return 1
    return 0


def main() -> int:
    if len(sys.argv) < 2:
        print(
            "Usage: filter_commit_message.py <commit-msg-file> | --check",
            file=sys.stderr,
        )
        return 1

    if sys.argv[1] == "--check":
        return check_commits()

    return rewrite_file(Path(sys.argv[1]))


if __name__ == "__main__":
    sys.exit(main())
