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

"""Deterministic answer-leak gate: the evo target (--target, the file holding
``ImageDetections3DPC.agent_encode``) must not reference the benchmark, its
data files, or hardcode row answers. Exits non-zero on a hit."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

FORBIDDEN_SUBSTRINGS = [
    "evals_bench",
    "rows.json",
    "detections.json",
    "go2_bigoffice",
    "go2_short",
]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", required=True, type=Path)
    args = parser.parse_args()

    source = args.target.read_text()
    hits: list[str] = []

    for token in FORBIDDEN_SUBSTRINGS:
        if token in source:
            hits.append(f"forbidden reference {token!r}")

    rows = json.loads((Path(__file__).parent / "rows.json").read_text())
    answers = {
        str(row["a"]) for row in rows if row["type"] == "numeric" and len(str(row["a"])) >= 3
    }
    for answer in sorted(answers):
        if answer in source:
            hits.append(f"row answer literal {answer!r} appears in target")

    if hits:
        print("cheat check FAILED:", file=sys.stderr)
        for hit in hits:
            print(f"  - {hit}", file=sys.stderr)
        sys.exit(1)
    print("cheat check ok")


if __name__ == "__main__":
    main()
