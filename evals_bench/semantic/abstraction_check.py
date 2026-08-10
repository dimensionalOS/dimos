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

"""Abstraction-budget gate: deterministic AST diff against the fork point.

An experiment may add AT MOST one @skill function that touches memory2 APIs,
at most ONE ``agent_encode`` per class, and NO new store/stream wrapper
classes over memory2. More than that is abstraction laundering — moving the
benchmark's work into unpenalized scaffolding — and fails the gate (exit 1).

Usage: abstraction_check.py --worktree <path> [--fork 0ddf8b493]
"""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
import re
import subprocess
import sys

FORK_SHA = "0ddf8b493"
MEMORY2_API = re.compile(r"memory2|MemoryStore|SqliteStore|\.streams\b|\.stream\(")
WRAPPER_BASES = ("Store", "Stream")


def _git(worktree: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(worktree), *args], capture_output=True, text=True, check=False
    )
    return result.stdout


def changed_py_files(worktree: Path, fork: str) -> list[str]:
    tracked = _git(worktree, "diff", "--name-only", fork, "--", "*.py").splitlines()
    untracked = _git(
        worktree, "ls-files", "--others", "--exclude-standard", "--", "*.py"
    ).splitlines()
    return sorted({p for p in tracked + untracked if p})


def _decorator_is_skill(dec: ast.expr) -> bool:
    if isinstance(dec, ast.Call):
        dec = dec.func
    if isinstance(dec, ast.Attribute):
        return dec.attr == "skill"
    return isinstance(dec, ast.Name) and dec.id == "skill"


def _analyze(src: str) -> dict[str, object] | None:
    """AST facts for one file: skill fns, agent_encode counts, wrapper classes."""
    try:
        tree = ast.parse(src)
    except SyntaxError:
        return None
    imports_memory2 = "memory2" in src

    skills: dict[str, bool] = {}  # fn name -> touches memory2
    encodes: dict[str, int] = {}  # class name (or "<module>") -> agent_encode defs
    wrappers: set[str] = set()

    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            if any(_decorator_is_skill(d) for d in node.decorator_list):
                segment = ast.get_source_segment(src, node) or ""
                skills[node.name] = imports_memory2 or bool(MEMORY2_API.search(segment))
        if isinstance(node, ast.ClassDef):
            count = sum(
                isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
                and item.name == "agent_encode"
                for item in node.body
            )
            if count:
                encodes[node.name] = count
            base_names = [
                b.attr if isinstance(b, ast.Attribute) else getattr(b, "id", "") for b in node.bases
            ]
            if imports_memory2 and any(n.endswith(WRAPPER_BASES) for n in base_names if n):
                wrappers.add(node.name)
    for node in tree.body:  # module-level agent_encode defs
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and (
            node.name == "agent_encode"
        ):
            encodes["<module>"] = encodes.get("<module>", 0) + 1
    return {"skills": skills, "encodes": encodes, "wrappers": wrappers}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worktree", required=True, type=Path)
    parser.add_argument("--fork", default=FORK_SHA)
    args = parser.parse_args()

    new_memory2_skills: list[str] = []
    violations: list[str] = []

    for rel in changed_py_files(args.worktree, args.fork):
        path = args.worktree / rel
        if not path.exists():  # deleted file
            continue
        new = _analyze(path.read_text())
        old = _analyze(_git(args.worktree, "show", f"{args.fork}:{rel}"))
        if new is None:
            continue
        old = old or {"skills": {}, "encodes": {}, "wrappers": set()}

        for name, touches in new["skills"].items():  # type: ignore[union-attr]
            if name not in old["skills"] and touches:  # type: ignore[operator]
                new_memory2_skills.append(f"{rel}:{name}")
        for cls, count in new["encodes"].items():  # type: ignore[union-attr]
            added = count - old["encodes"].get(cls, 0)  # type: ignore[union-attr]
            if added > 1:
                violations.append(f"{rel}: {added} agent_encode defs added on {cls!r} (max 1)")
        for cls in new["wrappers"] - old["wrappers"]:  # type: ignore[operator]
            violations.append(f"{rel}: new class {cls!r} wraps a memory2 Store/Stream")

    if len(new_memory2_skills) > 1:
        violations.append(
            f"{len(new_memory2_skills)} @skill functions touching memory2 added (max 1): "
            + ", ".join(new_memory2_skills)
        )

    if violations:
        print("abstraction check FAILED:", file=sys.stderr)
        for v in violations:
            print(f"  - {v}", file=sys.stderr)
        sys.exit(1)
    print(f"abstraction check ok ({len(new_memory2_skills)} memory2 skill added)")


if __name__ == "__main__":
    main()
