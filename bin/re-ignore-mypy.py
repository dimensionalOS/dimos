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

from collections import defaultdict
from pathlib import Path
import re
import subprocess


def remove_type_ignore_comments(directory: Path) -> None:
    # Pattern matches "# type: ignore" with optional error codes in brackets.
    # Captures any trailing comment after `type: ignore`.
    type_ignore_pattern = re.compile(r"(\s*)#\s*type:\s*ignore(?:\[[^\]]*\])?(\s*#.*)?")

    for py_file in directory.rglob("*.py"):
        try:
            content = py_file.read_text()
        except Exception:
            continue

        new_lines = []
        modified = False

        for line in content.splitlines(keepends=True):
            match = type_ignore_pattern.search(line)
            if match:
                before = line[: match.start()]
                trailing_comment = match.group(2)

                if trailing_comment:
                    new_line = before + match.group(1) + trailing_comment.lstrip()
                else:
                    new_line = before

                if line.endswith("\n"):
                    new_line = new_line.rstrip() + "\n"
                else:
                    new_line = new_line.rstrip()
                new_lines.append(new_line)
                modified = True
            else:
                new_lines.append(line)

        if modified:
            try:
                py_file.write_text("".join(new_lines))
            except Exception:
                pass


def run_mypy(root: Path) -> str:
    result = subprocess.run(
        [str(root / "bin" / "mypy-ros")],
        capture_output=True,
        text=True,
        cwd=root,
    )
    return result.stdout + result.stderr


def parse_mypy_errors(output: str) -> dict[Path, dict[int, list[str]]]:
    error_pattern = re.compile(r"^(.+):(\d+): error: .+\[([^\]]+)\]\s*$")
    errors: dict[Path, dict[int, list[str]]] = defaultdict(lambda: defaultdict(list))

    for line in output.splitlines():
        match = error_pattern.match(line)
        if match:
            file_path = Path(match.group(1))
            line_num = int(match.group(2))
            error_code = match.group(3)
            if error_code not in errors[file_path][line_num]:
                errors[file_path][line_num].append(error_code)

    return errors


def add_type_ignore_comments(root: Path, errors: dict[Path, dict[int, list[str]]]) -> None:
    comment_pattern = re.compile(r"^([^#]*?)(  #.*)$")

    for file_path, line_errors in errors.items():
        full_path = root / file_path
        if not full_path.exists():
            continue

        try:
            content = full_path.read_text()
        except Exception:
            continue

        lines = content.splitlines(keepends=True)
        modified = False

        for line_num, error_codes in line_errors.items():
            if line_num < 1 or line_num > len(lines):
                continue

            idx = line_num - 1
            line = lines[idx]
            codes_str = ", ".join(sorted(error_codes))
            ignore_comment = f"  # type: ignore[{codes_str}]"

            has_newline = line.endswith("\n")
            line_content = line.rstrip("\n")

            comment_match = comment_pattern.match(line_content)
            if comment_match:
                code_part = comment_match.group(1)
                existing_comment = comment_match.group(2)
                new_line = code_part + ignore_comment + existing_comment
            else:
                new_line = line_content + ignore_comment

            if has_newline:
                new_line += "\n"

            lines[idx] = new_line
            modified = True

        if modified:
            try:
                full_path.write_text("".join(lines))
            except Exception:
                pass


def main() -> None:
    root = Path(__file__).parent.parent
    dimos_dir = root / "dimos"

    remove_type_ignore_comments(dimos_dir)
    mypy_output = run_mypy(root)
    errors = parse_mypy_errors(mypy_output)
    add_type_ignore_comments(root, errors)


if __name__ == "__main__":
    main()
