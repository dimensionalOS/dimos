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

"""Integrity guard for the harness itself. Lives OUTSIDE the worktrees.

Every other gate is a tracked file inside the experiment worktree, which the
candidate agent can edit. `sys.exit(0)` at the top of check_rules.py, or a
relaxed assertion in invariants.rs, defeats that gate completely and nothing
else would notice -- the diff would show it, but no check would fail.

So this one file sits in `.evo/`, which is not part of any experiment
branch, and asserts the harness is byte-identical to the manifest pinned in
`.evo/harness.lock`. Optimizing the planner never requires editing the thing
that measures the planner. When the harness genuinely needs to change, a
human updates the lock deliberately.

  python3 .evo/bench_guard.py <worktree>

Stdlib only; runs under bare python3 like every other gate.
"""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOCK = HERE / "harness.lock"


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(f"usage: {Path(sys.argv[0]).name} <worktree>")
    worktree = Path(sys.argv[1]).resolve()
    if not LOCK.exists():
        sys.exit(f"harness lock missing at {LOCK}")
    lock = json.loads(LOCK.read_text(encoding="utf-8"))

    problems: list[str] = []

    # Verify the integrity files themselves against the copy tracked in the
    # experiment branch, before trusting anything they say. A one-sided edit
    # fails here; a matched edit shows up in the experiment diff.
    trust_path = worktree / "bench" / "trust.lock"
    if not trust_path.exists():
        problems.append("bench/trust.lock is missing -- it cross-pins the .evo integrity files")
    else:
        trust = json.loads(trust_path.read_text(encoding="utf-8"))
        for rel, want in sorted(trust["files"].items()):
            path = HERE.parent / rel
            got = (hashlib.sha256(path.read_bytes()).hexdigest()
                   if path.exists() else "<missing>")
            if got != want:
                problems.append(
                    f"{rel}: does not match the tracked bench/trust.lock\n"
                    f"      expected {want}\n      got      {got}"
                )

    for rel, want in sorted(lock["files"].items()):
        path = worktree / rel
        if not path.exists():
            problems.append(f"{rel}: deleted -- it is part of the measurement harness")
            continue
        got = hashlib.sha256(path.read_bytes()).hexdigest()
        if got != want:
            problems.append(
                f"{rel}: modified\n      expected {want}\n      got      {got}"
            )

    if problems:
        print(f"HARNESS TAMPERED ({len(problems)}):", file=sys.stderr)
        for p in problems:
            print(f"  - {p}", file=sys.stderr)
        print(
            "\n  The harness measures the candidate; it is not part of the candidate.\n"
            "  If a change here is genuinely needed, say so and get "
            f"{LOCK} updated on purpose.",
            file=sys.stderr,
        )
        sys.exit(1)
    print(f"harness intact: {len(lock['files'])} files match {LOCK.name}")


if __name__ == "__main__":
    main()
