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

"""Dependency-light shell for the unified Evaluation CLI."""

from __future__ import annotations

from pathlib import Path
import threading
from typing import Any

import typer

app = typer.Typer(help="Run executable evaluations", no_args_is_help=True)

MAX_RENDERED_TOOL_RESULT_CHARS = 2_000


def execute_evaluation(*args: Any, **kwargs: Any) -> Any:
    """Import and dispatch the evaluation runtime only when ``eval run`` executes."""
    try:
        from dimos.benchmark.evaluation.runner import execute_evaluation as execute
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Evaluation dependencies are missing; run `uv sync --extra agents`"
        ) from exc

    return execute(*args, **kwargs)


@app.command("run")
def run(
    specification: Path = typer.Argument(..., exists=True, dir_okay=False, readable=True),
    api_key_env: str = typer.Option("OPENAI_API_KEY", "--api-key-env"),
    output: Path = typer.Option(..., "--output"),
    json_output: bool = typer.Option(False, "--json", help="Print compact JSON"),
    quiet: bool = typer.Option(False, "--quiet", help="Suppress live evaluation progress"),
) -> None:
    """Run one Evaluation Run Specification synchronously."""
    renderer = None if quiet else ProgressRenderer()
    try:
        result = execute_evaluation(
            specification,
            output=output,
            api_key_env=api_key_env,
            progress=renderer,
        )
    except Exception as exc:
        if renderer is not None:
            renderer.finish()
        typer.echo(f"Evaluation preflight failed: {type(exc).__name__}: {exc}", err=True)
        raise typer.Exit(2) from exc
    if renderer is not None:
        renderer.finish()
    typer.echo(result.model_dump_json() if json_output else format_result(result, output))
    if result.status == "cancelled":
        raise typer.Exit(130)
    if result.status == "failed":
        raise typer.Exit(1)


def format_result(result: Any, output: Path | None = None) -> str:
    """Render infrastructure status plus Evaluation-supplied summary rows."""
    heading = {
        "completed": "✓ Evaluation completed",
        "failed": "! Evaluation failed",
        "cancelled": "! Evaluation cancelled",
    }[result.status]
    rows = [
        ("Evaluation", result.evaluation.name),
        (
            "Agent",
            f"Pi · {result.runtime.model} · {result.runtime.thinking_level}",
        ),
    ]
    if result.report is not None:
        rows.extend(
            (item.label, "—" if item.value is None else str(item.value))
            for item in result.report.summary
        )
    if result.error is not None:
        rows.append(("Error", f"{result.error.error_type}: {result.error.message}"))
    rows.append(("Output", str((output / "run.json") if output is not None else "run.json")))
    width = max(len(label) for label, _ in rows)
    body = "\n".join(f"  {label:<{width}}  {value}" for label, value in rows)
    return f"{heading}\n\n{body}"


class ProgressRenderer:
    """Thread-safe concise terminal renderer for best-effort evaluation progress."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._assistant_open = False
        self._saw_assistant_text = False

    def __call__(self, event: Any) -> None:
        with self._lock:
            if event.kind == "assistant_text":
                if not self._assistant_open and not event.delta.strip():
                    return
                if not self._assistant_open:
                    typer.echo("[pi] ", err=True, nl=False)
                    self._assistant_open = True
                typer.echo(event.delta, err=True, nl=False)
                self._saw_assistant_text = True
                return
            self._end_assistant_line()
            if event.kind == "case_header":
                source = event.source
                if event.progress is not None:
                    source += f" @ {event.progress * 100:g}%"
                typer.echo("[eval] Session", err=True)
                typer.echo(f"  {'Case':<10} {event.case_id}", err=True)
                typer.echo(f"  {'Source':<10} {source}", err=True)
                typer.echo(f"  {'Question':<10} {event.question}", err=True)
                typer.echo(f"  {'Answer':<10} pending", err=True)
            elif event.kind == "status":
                typer.echo(f"[{event.channel}] {event.message}", err=True)
            elif event.kind == "tool_start":
                typer.echo("[python_exec] call", err=True)
                typer.echo(_indent(event.code), err=True)
            elif event.kind == "tool_end":
                status = "ok" if event.ok else "error"
                typer.echo(f"[python_exec] {status} ({event.duration_seconds:.1f}s)", err=True)
                if event.result:
                    typer.echo(_indent(_truncate_tool_result(event.result)), err=True)
            elif event.kind == "final_response" and not self._saw_assistant_text:
                typer.echo(f"[pi] {event.text}", err=True)

    def finish(self) -> None:
        with self._lock:
            self._end_assistant_line()

    def _end_assistant_line(self) -> None:
        if self._assistant_open:
            typer.echo("", err=True)
            self._assistant_open = False


def _indent(value: str) -> str:
    return "\n".join(f"  {line}" for line in value.splitlines())


def _truncate_tool_result(value: str) -> str:
    if len(value) <= MAX_RENDERED_TOOL_RESULT_CHARS:
        return value
    visible = value[:MAX_RENDERED_TOOL_RESULT_CHARS].rstrip()
    return f"{visible}\n... [terminal output truncated; full result retained]"
