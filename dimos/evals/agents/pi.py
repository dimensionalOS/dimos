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

"""The Pi coding agent (pi.dev) as an eval agent."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import json
import os
from pathlib import Path
import shutil
import subprocess
import time
from typing import TYPE_CHECKING, Any

from dimos.agents.llm_trace import latest_pair
from dimos.evals.agents.lib.chat import DEFAULT_MODEL
from dimos.evals.agents.lib.proxy import RecordingProxy
from dimos.evals.agents.lib.recorder import StepDraft
from dimos.evals.types import EndedBy, Environment, RunningEnvironment, ToolCall, Trajectory

if TYPE_CHECKING:
    from dimos.memory.store.base import Store

OPENAI_URL = "https://api.openai.com/v1"
PI_TOOLS = ("read", "bash", "edit", "write")

PI_SYSTEM_PROMPT = (
    "You are evaluating a robot's perception and memory. Answer the question "
    "from the files and tools listed below and nothing else. Reply with the "
    "answer value only — a bare number or a short phrase. No explanation, no "
    "units unless asked."
)

RECORDING_HELP = (
    "The recording is a dimos memory store (sqlite). In Python:\n"
    "  from dimos.memory.store.sqlite import SqliteStore\n"
    "  store = SqliteStore(path=PATH, must_exist=True)\n"
    "store.streams.<name> is a stream, .last().data its latest message; iterating "
    "a stream yields observations with .ts and .data. Inspect with dir() and help()."
)

ROBOT_HELP = (
    "The robot is live. Call one of its tools from bash as\n"
    "  dimos mcp call <tool> --json-args '{\"arg\": value}'\n"
    "Tools:\n"
)


def render_tools(tools: list[dict[str, Any]]) -> str:
    """One line per MCP tool: name, argument names, first line of the description."""
    lines = []
    for t in tools:
        args = ", ".join((t.get("inputSchema") or {}).get("properties") or {})
        summary = str(t.get("description") or "").strip().partition("\n")[0]
        lines.append(f"- {t['name']}({args}): {summary}")
    return "\n".join(lines)


def tool_listing(mcp_url: str) -> str:
    from dimos.agents.mcp.mcp_adapter import McpAdapter

    return render_tools(McpAdapter(mcp_url).list_tools())


def recording_file(store: Store, path: Path) -> Path:
    """*store* as a file a subprocess can open: where it is when it is on disk,
    else written to *path*."""
    from dimos.memory.store.base import copy_streams
    from dimos.memory.store.sqlite import SqliteStore

    if isinstance(store, SqliteStore):
        return Path(store.config.path)
    copy_streams((s for _, s in store.streams.items()), SqliteStore(path=str(path))).stop()
    return path


def _result_text(result: Any) -> str:
    content = result.get("content") if isinstance(result, dict) else None
    if isinstance(content, list):
        return "\n".join(str(c.get("text", "")) for c in content if c.get("type") == "text")
    return str(result)


class _Events:
    """Pi's ``--mode json`` stream folded into steps: one per assistant
    ``message_end``, each paired with the request/response the proxy recorded
    for it; tool results are observations of the step that called them."""

    def __init__(self, raw_dir: Path, t0: float) -> None:
        self.raw_dir = raw_dir
        self.t0 = t0
        self.drafts: list[StepDraft] = []
        self.model = ""
        self.cached_tokens = 0
        self.error = ""
        self._next_seq = 0

    def feed(self, line: str) -> None:
        event = json.loads(line)
        if event.get("type") == "tool_execution_end" and self.drafts:
            self.drafts[-1].observations.append(_result_text(event.get("result")))
        elif event.get("type") == "message_end" and event["message"].get("role") == "assistant":
            self._step(event["message"])

    def _step(self, message: dict[str, Any]) -> None:
        pair = latest_pair(self.raw_dir, self._next_seq)
        if pair is None:
            raise RuntimeError(
                f"Pi made a model call that left no trace under {self.raw_dir}; "
                "every call must go through the recording proxy"
            )
        self._next_seq = pair[0] + 1
        usage = message.get("usage") or {}
        content = message.get("content") or []
        self.model = str(message.get("responseModel") or message.get("model") or self.model)
        self.cached_tokens += int(usage.get("cacheRead", 0))
        if message.get("stopReason") in ("error", "aborted"):
            self.error = str(message.get("errorMessage") or message["stopReason"])
        self.drafts.append(
            StepDraft(
                index=len(self.drafts),
                t=time.monotonic() - self.t0,
                message="".join(str(c.get("text", "")) for c in content if c.get("type") == "text"),
                tool_calls=[
                    ToolCall(name=str(c["name"]), args=dict(c.get("arguments") or {}))
                    for c in content
                    if c.get("type") == "toolCall"
                ],
                input_tokens=int(usage.get("input", 0)),
                output_tokens=int(usage.get("output", 0)),
                latency_s=float(json.loads(pair[2].read_text()).get("latency_s") or 0.0),
                request=pair[1],
                response=pair[2],
            )
        )


@dataclass
class Pi:
    """The Pi coding agent, headless (``pi --mode json``), with the run dir as
    its working directory. The case's artifacts and the recording are files
    named in the system prompt; a live robot is reached through ``dimos mcp
    call`` from Pi's ``bash`` tool — Pi has no MCP client, and its authors say
    to give it CLI tools instead, which dimos ships. Model traffic goes through
    a local :class:`RecordingProxy`, so every call is captured whole under
    ``run_dir/raw`` like any other agent's. OpenAI models only, for now.

    ``max_steps`` is enforced from outside: Pi is killed when it has made that
    many model calls and still wants to act. ``cli`` is the ``pi`` executable.
    ``skills`` are explicit skill files or directories for Pi's repeatable
    native ``--skill`` flag, resolved to absolute paths (Pi runs in the case
    dir); ambient skill discovery stays off — ``--no-skills`` disables
    discovery only, explicit paths still load.
    """

    model: str = DEFAULT_MODEL
    system_prompt: str = PI_SYSTEM_PROMPT
    thinking: str = "medium"
    max_steps: int | None = 40
    cli: str = "pi"
    modules: str = ""
    skills: Sequence[str] = ()

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        """Pi's fixed built-ins plus robot tools exposed through its bash tool."""
        return (*PI_TOOLS, *environment_tools)

    def preflight(self, environment: Environment) -> None:
        if isinstance(self.skills, str):
            raise RuntimeError("Pi.skills is a string; pass a list of paths (--set skills='[...]')")
        missing = [p for p in self.skills if not Path(p).expanduser().resolve().exists()]
        if missing:
            raise RuntimeError(f"Pi skill paths do not exist: {missing}")
        if shutil.which(self.cli) is None:
            raise RuntimeError(
                f"{self.cli!r} is not on PATH (npm install -g @earendil-works/pi-coding-agent)"
            )
        if "OPENAI_API_KEY" not in os.environ:
            raise RuntimeError("Pi needs OPENAI_API_KEY")
        if environment.has_robot and shutil.which("dimos") is None:
            raise RuntimeError("Pi reaches the robot through the dimos CLI, which is not on PATH")

    def run(self, inputs: str, env: RunningEnvironment, run_dir: Path) -> Trajectory:
        raw_dir = run_dir / "raw"
        t0 = time.monotonic()
        events = _Events(raw_dir, t0)
        with RecordingProxy(raw_dir, os.environ.get("OPENAI_BASE_URL", OPENAI_URL)) as proxy_url:
            self._write_agent_dir(run_dir, proxy_url)
            ended_by = self._drive(self._command(inputs, env, run_dir), run_dir, events)
        if events.error:
            raise RuntimeError(f"Pi stopped: {events.error}")
        steps = tuple(d.freeze() for d in events.drafts)
        return Trajectory(
            final_answer=next((s.message for s in reversed(steps) if not s.tool_calls), ""),
            steps=steps,
            model=events.model,
            input_tokens=sum(s.input_tokens for s in steps),
            output_tokens=sum(s.output_tokens for s in steps),
            cached_tokens=events.cached_tokens,
            duration_s=time.monotonic() - t0,
            ended_by=ended_by,
            raw_dir=raw_dir,
        )

    def _command(self, inputs: str, env: RunningEnvironment, run_dir: Path) -> list[str]:
        files = {
            **env.artifacts,
            "recording": recording_file(env.recording, run_dir / "recording.db"),
        }
        parts = [
            self.system_prompt,
            "Files:\n" + "\n".join(f"- {name}: {path}" for name, path in files.items()),
            RECORDING_HELP,
        ]
        if env.mcp_url:
            parts.append(ROBOT_HELP + tool_listing(env.mcp_url))
        prompt = "\n\n".join(parts)
        (run_dir / "system-prompt.txt").write_text(prompt)
        # Absolute before Pi changes to the run dir; --no-skills disables only
        # ambient discovery, explicit --skill paths still load.
        skills = [f for p in self.skills for f in ("--skill", str(Path(p).expanduser().resolve()))]
        return [
            self.cli, "--mode", "json", "--model", f"dimos/{self.model}", "--thinking", self.thinking,
            "--session-dir", str(run_dir / "pi-session"),
            "--tools", ",".join(PI_TOOLS),
            "--no-extensions", "--no-skills", "--no-prompt-templates", "--no-themes",
            "--no-context-files", "--no-approve", *skills,
            "--system-prompt", prompt, inputs,
        ]  # fmt: skip

    def _write_agent_dir(self, run_dir: Path, proxy_url: str) -> None:
        """A private Pi config dir: one provider, ``dimos``, which is the proxy."""
        agent_dir = run_dir / ".pi-agent"
        agent_dir.mkdir(parents=True, exist_ok=True)
        provider = {
            "baseUrl": proxy_url,
            "api": "openai-responses",
            "apiKey": "$OPENAI_API_KEY",
            "models": [{"id": self.model, "reasoning": True}],
        }
        (agent_dir / "models.json").write_text(
            json.dumps({"providers": {"dimos": provider}}, indent=2)
        )

    def _env(self, run_dir: Path) -> dict[str, str]:
        keep = ("PATH", "HOME", "XDG_STATE_HOME", "OPENAI_API_KEY")
        passed = {k: v for k, v in os.environ.items() if k in keep or k.startswith("DIMOS_")}
        return {
            **passed,
            "PI_CODING_AGENT_DIR": str(run_dir / ".pi-agent"),
            "PI_SKIP_VERSION_CHECK": "1",
            "PI_TELEMETRY": "0",
        }

    def _drive(self, command: list[str], run_dir: Path, events: _Events) -> EndedBy:
        """Run Pi to completion, or kill it at ``max_steps`` model calls."""
        with (
            (run_dir / "pi-stderr.txt").open("w") as stderr,
            subprocess.Popen(
                command,
                cwd=run_dir,
                env=self._env(run_dir),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=stderr,
                text=True,
            ) as proc,  # fmt: skip
        ):
            assert proc.stdout is not None
            for line in proc.stdout:
                events.feed(line)
                if self._over_budget(events):
                    proc.kill()
                    return "max_steps"
        return "answer"

    def _over_budget(self, events: _Events) -> bool:
        """``max_steps`` calls made and the last one still asks for a tool."""
        return (
            self.max_steps is not None
            and len(events.drafts) >= self.max_steps
            and bool(events.drafts[-1].tool_calls)
        )
