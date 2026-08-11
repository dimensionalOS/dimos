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

"""Launch the pinned stock Pi CLI and parse its native JSON event stream."""

from __future__ import annotations

from dataclasses import dataclass
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time
from typing import Any

from dimos.benchmark.evaluation.progress import (
    AssistantTextProgress,
    FinalResponseProgress,
    ProgressSink,
    StatusProgress,
    ToolEndProgress,
    ToolStartProgress,
    emit_progress,
)

PI_VERSION = "0.80.10"
MAX_STDERR_BYTES = 64 * 1024
EXPORT_TIMEOUT_SECONDS = 30.0


@dataclass(frozen=True)
class PiRunResult:
    final_text: str
    tool_call_count: int
    duration_seconds: float
    transcript_path: Path | None
    stderr: str


class PiRunError(RuntimeError):
    def __init__(
        self,
        message: str,
        *,
        stderr: str = "",
        transcript_path: Path | None = None,
    ) -> None:
        super().__init__(message)
        self.stderr = stderr
        self.transcript_path = transcript_path


class PiExportError(RuntimeError):
    pass


class PiRunCancelledError(PiRunError):
    """The evaluator stopped an in-flight Pi session."""


class PiCliRunner:
    """Thin stock-CLI binding; the extension owns only the `python_exec` tool."""

    def __init__(
        self,
        *,
        cli: Path,
        extension: Path,
        model: str,
        thinking_level: str,
        timeout_s: float,
        progress: ProgressSink | None = None,
    ) -> None:
        if not cli.is_file():
            raise FileNotFoundError(f"Pi {PI_VERSION} CLI is not installed: {cli}")
        if not extension.is_file():
            raise FileNotFoundError(
                f"Pi CodePolicy extension is not built: {extension}; "
                "run `npm run build --prefix packages/pi-code-policy-extension`"
            )
        self.cli = cli
        self.extension = extension
        self.model = model
        self.thinking_level = thinking_level
        self.timeout_s = timeout_s
        self.progress = progress

    def run(
        self,
        *,
        prompt: str,
        system_prompt: str,
        mcp_url: str,
        api_key: str,
        run_dir: Path,
        cancel: threading.Event | None = None,
    ) -> PiRunResult:
        session_dir = run_dir / "pi-session"
        agent_dir = run_dir / ".pi-agent"
        system_prompt_path = run_dir / "system-prompt.txt"
        system_prompt_path.write_text(system_prompt, encoding="utf-8")
        command = (
            "node",
            str(self.cli),
            "--mode",
            "json",
            "--model",
            f"openai/{self.model}",
            "--thinking",
            self.thinking_level,
            "--session-dir",
            str(session_dir),
            "--name",
            "dimos-evaluation",
            "--no-builtin-tools",
            "--tools",
            "python_exec",
            "--no-extensions",
            "--extension",
            str(self.extension),
            "--no-skills",
            "--no-prompt-templates",
            "--no-themes",
            "--no-context-files",
            "--no-approve",
            "--system-prompt",
            str(system_prompt_path),
            prompt,
        )
        env = {
            "PATH": os.environ.get("PATH", ""),
            "OPENAI_API_KEY": api_key,
            "DIMOS_CODE_POLICY_MCP_URL": mcp_url,
            "PI_CODING_AGENT_DIR": str(agent_dir),
            "PI_SKIP_VERSION_CHECK": "1",
            "PI_TELEMETRY": "0",
        }
        started = time.monotonic()
        process = subprocess.Popen(
            command,
            cwd=run_dir,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        stdout = process.stdout
        stderr_stream = process.stderr
        assert stdout is not None
        assert stderr_stream is not None
        events = _PiEventAccumulator(self.progress)
        stderr_parts: list[str] = []
        stderr_bytes = 0

        def read_stdout() -> None:
            for line in stdout:
                events.feed(line)

        def read_stderr() -> None:
            nonlocal stderr_bytes
            for line in stderr_stream:
                line = line.replace(api_key, "[REDACTED]")
                encoded = line.encode()
                remaining = MAX_STDERR_BYTES - stderr_bytes
                if remaining > 0:
                    retained = encoded[:remaining].decode(errors="ignore")
                    stderr_parts.append(retained)
                    stderr_bytes += len(retained.encode())
                message = line.strip()
                if message:
                    emit_progress(
                        self.progress,
                        StatusProgress(channel="pi", message=_bounded_stderr(message)),
                    )

        readers = (
            threading.Thread(target=read_stdout, name="pi-stdout", daemon=True),
            threading.Thread(target=read_stderr, name="pi-stderr", daemon=True),
        )
        for reader in readers:
            reader.start()
        timed_out = False
        cancelled = False
        try:
            if cancel is None:
                try:
                    process.wait(timeout=self.timeout_s)
                except subprocess.TimeoutExpired:
                    timed_out = True
            else:
                deadline = time.monotonic() + self.timeout_s
                while process.poll() is None:
                    if cancel.wait(0.1):
                        cancelled = True
                        break
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        timed_out = True
                        break
                    try:
                        process.wait(timeout=min(remaining, 0.1))
                    except subprocess.TimeoutExpired:
                        pass
            if cancelled or timed_out:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        except BaseException:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            raise
        finally:
            for reader, stream in zip(
                readers,
                (stdout, stderr_stream),
                strict=True,
            ):
                reader.join(timeout=5)
                if reader.is_alive():
                    stream.close()
                    reader.join(timeout=1)

        stderr = "".join(stderr_parts)
        transcript_path = _latest_transcript(session_dir)
        if cancelled:
            raise PiRunCancelledError(
                "Pi was cancelled by the evaluator",
                stderr=stderr,
                transcript_path=transcript_path,
            )
        if timed_out:
            raise PiRunError(
                f"Pi timed out after {self.timeout_s:g}s",
                stderr=stderr,
                transcript_path=transcript_path,
            )
        duration = time.monotonic() - started
        if process.returncode != 0:
            raise PiRunError(
                f"Pi exited with status {process.returncode}",
                stderr=stderr,
                transcript_path=transcript_path,
            )
        if events.stop_error is not None:
            raise PiRunError(
                events.stop_error,
                stderr=stderr,
                transcript_path=transcript_path,
            )
        if events.final_text is None:
            raise PiRunError(
                "Pi produced no final assistant response",
                stderr=stderr,
                transcript_path=transcript_path,
            )
        return PiRunResult(
            final_text=events.final_text,
            tool_call_count=events.tool_count,
            duration_seconds=duration,
            transcript_path=transcript_path,
            stderr=stderr,
        )

    def export_transcript(
        self,
        *,
        transcript_path: Path,
        output_path: Path,
        mcp_url: str,
        api_key: str,
    ) -> None:
        """Export a saved session through Pi RPC so extension renderers are active."""
        request_id = "dimos-transcript-export"
        command = (
            "node",
            str(self.cli),
            "--mode",
            "rpc",
            "--model",
            f"openai/{self.model}",
            "--thinking",
            self.thinking_level,
            "--session",
            str(transcript_path),
            "--no-builtin-tools",
            "--tools",
            "python_exec",
            "--no-extensions",
            "--extension",
            str(self.extension),
            "--no-skills",
            "--no-prompt-templates",
            "--no-themes",
            "--no-context-files",
            "--no-approve",
            "--offline",
        )
        env = {
            "PATH": os.environ.get("PATH", ""),
            "OPENAI_API_KEY": api_key,
            "DIMOS_CODE_POLICY_MCP_URL": mcp_url,
            "PI_CODING_AGENT_DIR": str(transcript_path.parent / ".pi-agent-export"),
            "PI_SKIP_VERSION_CHECK": "1",
            "PI_TELEMETRY": "0",
        }
        process = subprocess.Popen(
            command,
            cwd=transcript_path.parent,
            env=env,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        stdin = process.stdin
        stdout = process.stdout
        stderr_stream = process.stderr
        assert stdin is not None
        assert stdout is not None
        assert stderr_stream is not None
        responses: queue.Queue[dict[str, Any]] = queue.Queue()
        stderr_parts: list[str] = []

        def read_stdout() -> None:
            for line in stdout:
                try:
                    response = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(response, dict) and response.get("id") == request_id:
                    responses.put(response)

        def read_stderr() -> None:
            retained = 0
            for line in stderr_stream:
                line = line.replace(api_key, "[REDACTED]")
                remaining = MAX_STDERR_BYTES - retained
                if remaining <= 0:
                    continue
                value = line.encode()[:remaining].decode(errors="ignore")
                stderr_parts.append(value)
                retained += len(value.encode())

        readers = (
            threading.Thread(target=read_stdout, name="pi-export-stdout", daemon=True),
            threading.Thread(target=read_stderr, name="pi-export-stderr", daemon=True),
        )
        for reader in readers:
            reader.start()
        try:
            stdin.write(
                json.dumps(
                    {
                        "id": request_id,
                        "type": "export_html",
                        "outputPath": str(output_path),
                    }
                )
                + "\n"
            )
            stdin.flush()
            deadline = time.monotonic() + EXPORT_TIMEOUT_SECONDS
            response = None
            while response is None and time.monotonic() < deadline:
                try:
                    response = responses.get(timeout=min(0.1, deadline - time.monotonic()))
                except queue.Empty:
                    if process.poll() is not None:
                        break
            if response is None:
                for reader in readers:
                    reader.join(timeout=1)
                detail = "".join(stderr_parts).strip()
                suffix = f": {detail}" if detail else ""
                if process.poll() is not None:
                    raise PiExportError(
                        f"Pi transcript exporter exited with status {process.returncode}{suffix}"
                    )
                raise PiExportError(
                    f"Pi transcript export timed out after {EXPORT_TIMEOUT_SECONDS:g}s{suffix}"
                )
            if not response.get("success"):
                message = str(response.get("error") or "Pi transcript export failed")
                raise PiExportError(message.replace(api_key, "[REDACTED]"))
            data = response.get("data")
            exported = data.get("path") if isinstance(data, dict) else None
            if not isinstance(exported, str) or Path(exported).resolve() != output_path.resolve():
                raise PiExportError("Pi transcript export returned an unexpected output path")
        finally:
            stdin.close()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            for reader, stream in zip(readers, (stdout, stderr_stream), strict=True):
                reader.join(timeout=1)
                if reader.is_alive():
                    stream.close()
                    reader.join(timeout=1)
        if process.returncode != 0:
            detail = "".join(stderr_parts).strip()
            suffix = f": {detail}" if detail else ""
            raise PiExportError(
                f"Pi transcript exporter exited with status {process.returncode}{suffix}"
            )
        if not output_path.is_file():
            raise PiExportError("Pi transcript exporter produced no HTML file")


def parse_pi_events(stream: str) -> tuple[str | None, int, str | None]:
    """Return the final assistant text, tool count, and terminal error."""
    events = _PiEventAccumulator()
    for line in stream.splitlines():
        events.feed(line)
    return events.final_text, events.tool_count, events.stop_error


class _PiEventAccumulator:
    def __init__(self, progress: ProgressSink | None = None) -> None:
        self.progress = progress
        self.final_text: str | None = None
        self.tool_count = 0
        self.stop_error: str | None = None
        self._tool_started: dict[str, float] = {}

    def feed(self, line: str) -> None:
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            return
        if not isinstance(event, dict):
            return
        event_type = event.get("type")
        if event_type == "message_update":
            update = event.get("assistantMessageEvent")
            if isinstance(update, dict) and update.get("type") == "text_delta":
                delta = update.get("delta")
                if isinstance(delta, str) and delta:
                    emit_progress(self.progress, AssistantTextProgress(delta=delta))
            return
        if event_type == "tool_execution_start":
            self.tool_count += 1
            call_id = str(event.get("toolCallId", ""))
            self._tool_started[call_id] = time.monotonic()
            args = event.get("args")
            code = args.get("code") if isinstance(args, dict) else None
            if event.get("toolName") == "python_exec" and isinstance(code, str) and code:
                emit_progress(self.progress, ToolStartProgress(code=code))
            return
        if event_type == "tool_execution_end":
            call_id = str(event.get("toolCallId", ""))
            started = self._tool_started.pop(call_id, time.monotonic())
            if event.get("toolName") == "python_exec":
                emit_progress(
                    self.progress,
                    ToolEndProgress(
                        ok=not bool(event.get("isError")),
                        result=_tool_result_text(event.get("result")),
                        duration_seconds=max(0.0, time.monotonic() - started),
                    ),
                )
            return
        if event_type != "message_end":
            return
        message = event.get("message")
        if not isinstance(message, dict) or message.get("role") != "assistant":
            return
        self.final_text = "".join(
            str(item.get("text", ""))
            for item in message.get("content", [])
            if isinstance(item, dict) and item.get("type") == "text"
        )
        emit_progress(self.progress, FinalResponseProgress(text=self.final_text))
        stop_reason = message.get("stopReason")
        if stop_reason in {"error", "aborted"}:
            self.stop_error = str(message.get("errorMessage") or f"Pi request {stop_reason}")


def _tool_result_text(result: Any) -> str:
    if isinstance(result, dict):
        content = result.get("content")
        if isinstance(content, list):
            return "\n".join(
                str(item.get("text", ""))
                for item in content
                if isinstance(item, dict) and item.get("type") == "text"
            )
    return str(result)


def _bounded_stderr(value: str) -> str:
    encoded = value.encode()
    if len(encoded) <= MAX_STDERR_BYTES:
        return value
    return encoded[:MAX_STDERR_BYTES].decode(errors="ignore")


def _latest_transcript(session_dir: Path) -> Path | None:
    transcripts = sorted(session_dir.rglob("*.jsonl")) if session_dir.exists() else []
    return transcripts[-1] if transcripts else None
