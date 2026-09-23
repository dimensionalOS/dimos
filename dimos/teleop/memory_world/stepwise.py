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

"""Run an analysis script one step at a time. Standard library only: the sandbox imports it."""

from __future__ import annotations

import ast
from collections.abc import Callable
import time

# Short top-level statements are run together as one step up to this many lines.
STEP_GROUP_LINES = 8

_COMPOUND = (
    ast.For,
    ast.AsyncFor,
    ast.While,
    ast.If,
    ast.With,
    ast.AsyncWith,
    ast.Try,
    ast.FunctionDef,
    ast.AsyncFunctionDef,
    ast.ClassDef,
    ast.Match,
)


def analysis_steps(source: str) -> list[tuple[str, list[ast.stmt]]]:
    """Top-level statements of an analysis grouped into steps, each with its source.

    Loops, branches and definitions stand alone. Short statements between them run
    together, with the comment lines above them, so a script reads as a few steps
    rather than one per line.
    """
    lines = source.splitlines()
    steps: list[tuple[str, list[ast.stmt]]] = []
    group: list[ast.stmt] = []

    def start_line(node: ast.stmt) -> int:
        line = node.lineno - 1
        while line > 0 and lines[line - 1].lstrip().startswith("#"):
            line -= 1
        return line

    def flush() -> None:
        if group:
            first, last = start_line(group[0]), group[-1].end_lineno or group[-1].lineno
            steps.append(("\n".join(lines[first:last]), list(group)))
            group.clear()

    for node in ast.parse(source, "<analyze_memory>").body:
        span = (node.end_lineno or node.lineno) - node.lineno + 1
        compound = isinstance(node, _COMPOUND) or span > STEP_GROUP_LINES
        grouped = sum((n.end_lineno or n.lineno) - n.lineno + 1 for n in group)
        if compound or grouped + span > STEP_GROUP_LINES:
            flush()
        group.append(node)
        if compound:
            flush()
    flush()
    return steps


def run_stepwise(
    source: str, namespace: dict[str, object], report: Callable[[dict[str, object]], None]
) -> None:
    """Execute an analysis one step at a time, reporting each step's start, end and time."""
    steps = analysis_steps(source)
    for index, (text, nodes) in enumerate(steps):
        report({"index": index, "total": len(steps), "status": "start", "source": text})
        started = time.perf_counter()
        module = ast.Module(body=nodes, type_ignores=[])
        exec(compile(module, "<analyze_memory>", "exec"), namespace)
        report(
            {
                "index": index,
                "total": len(steps),
                "status": "done",
                "ms": round((time.perf_counter() - started) * 1000, 1),
            }
        )
