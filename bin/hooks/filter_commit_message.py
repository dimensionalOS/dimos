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

from pathlib import Path
import sys


def main() -> int:
    if len(sys.argv) < 2:
        print("Usage: filter_commit_message.py <commit-msg-file>", file=sys.stderr)
        return 1

    commit_msg_file = Path(sys.argv[1])
    if not commit_msg_file.exists():
        return 0

    lines = commit_msg_file.read_text().splitlines(keepends=True)

    # Patterns that trigger truncation (everything from this line onwards is removed)
    truncate_patterns = [
        "Generated with",
        "Co-Authored-By",
    ]

    # Find the first line containing any truncate pattern and truncate there
    filtered_lines = []
    for line in lines:
        if any(pattern in line for pattern in truncate_patterns):
            break
        filtered_lines.append(line)

    commit_msg_file.write_text("".join(filtered_lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
