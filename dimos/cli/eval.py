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

"""Dependency-light shell for the immutable single-case evaluation CLI."""

from __future__ import annotations

from pathlib import Path
import threading
from typing import Any, Literal

import typer

app = typer.Typer(help="Run immutable agent evaluation cases", no_args_is_help=True)

MAX_RENDERED_TOOL_RESULT_CHARS = 2_000


def execute_single_case(*args: Any, **kwargs: Any) -> Any:
    """Import and dispatch the evaluation runtime only when ``eval run`` executes."""
    try:
        from dimos.benchmark.agent_eval.single_case import execute_single_case as execute
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Evaluation dependencies are missing; run `uv sync --extra agents`"
        ) from exc

    return execute(*args, **kwargs)


@app.command("run")
def run(
    case: Path = typer.Argument(..., exists=True, dir_okay=False, readable=True),
    agent_backend: Literal["pi"] = typer.Option("pi", "--agent.backend"),
    agent_model: Literal["gpt-5.6-luna"] = typer.Option("gpt-5.6-luna", "--agent.model"),
    thinking_level: Literal["medium"] = typer.Option("medium", "--agent.thinking-level"),
    api_key_env: str = typer.Option("OPENAI_API_KEY", "--agent.api-key-env"),
    output: Path = typer.Option(..., "--output"),
    json_output: bool = typer.Option(False, "--json", help="Print compact JSON"),
    quiet: bool = typer.Option(False, "--quiet", help="Suppress live evaluation progress"),
) -> None:
    """Run one static evaluation case synchronously."""
    from dimos.benchmark.agent_eval.models import (
        EvalRunConfig,
        PiAgentConfig,
    )

    config = EvalRunConfig(
        agent=PiAgentConfig(
            backend=agent_backend,
            model=agent_model,
            thinking_level=thinking_level,
            api_key_env=api_key_env,
        )
    )
    renderer = None if quiet else ProgressRenderer()
    try:
        result = execute_single_case(
            case,
            config=config,
            output=output,
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
    if result.attempt_status == "failed":
        raise typer.Exit(1)


def format_result(result: Any, output: Path | None = None) -> str:
    """Render the compact typed result without exposing private oracle material."""
    if result.attempt_status == "failed":
        heading = "! Evaluation not evaluated"
    elif result.task_result == "passed":
        heading = "✓ Evaluation passed"
    else:
        heading = "✗ Evaluation failed"
    source = f"{result.recording} @ {result.progress * 100:g}%"
    answer = str(result.integer_answer) if result.integer_answer is not None else "—"
    rows = (
        ("Case", result.case_id),
        ("Source", source),
        ("Answer", answer),
        ("Result", result.task_result),
        ("Agent", f"Pi · {result.model} · {result.thinking_level}"),
        ("Tool calls", str(result.tool_call_count)),
        ("Duration", f"{result.duration_seconds:.1f}s"),
        ("Output", str((output / "result.json") if output is not None else "result.json")),
    )
    body = "\n".join(f"  {label:<10} {value}" for label, value in rows)
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
