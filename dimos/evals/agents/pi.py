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
import queue
import shutil
import subprocess
import threading
import time
from typing import TYPE_CHECKING, Any

from dimos.agents.llm_trace import latest_pair
from dimos.evals.agents.lib.chat import DEFAULT_MODEL
from dimos.evals.agents.lib.proxy import RecordingProxy
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.types import (
    EndedBy,
    Environment,
    Metrics,
    RunningEnvironment,
    ToolCall,
    Trajectory,
)

if TYPE_CHECKING:
    from dimos.memory.stream import Stream

_PROVIDER = "dimos"  # the one provider in Pi's private config: the recording proxy


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


def recording_file(streams: Sequence[Stream[Any, Any]], path: Path) -> Path:
    """*streams* written to a memory store at *path*, under their own names,
    so a subprocess can open what the case selected and nothing more."""
    from dimos.memory.store.sqlite import SqliteStore

    store = SqliteStore(path=str(path))
    try:
        for stream in streams:
            if stream.name is None:
                raise ValueError("a stream must be bound to a store to be written out")
            target: Stream[Any, Any] = store.stream(stream.name, stream.data_type)
            for obs in stream:
                target.append(obs.data, ts=obs.ts, pose=obs.pose_tuple, tags=obs.tags)
    finally:
        store.stop()
    return path


def _registry_cost(cli: str, model: str) -> dict[str, Any] | None:
    """*model*'s pricing from the installed Pi's model registry, so Pi can
    price its own calls; None when the model is not in the registry."""
    exe = shutil.which(cli)
    for parent in Path(exe).resolve().parents if exe else ():
        data = parent / "node_modules" / "@earendil-works" / "pi-ai" / "dist" / "providers" / "data"
        if data.is_dir():
            for f in sorted(data.glob("*.json")):
                for models in json.loads(f.read_text()).values():
                    entry = models.get(model) if isinstance(models, dict) else None
                    if isinstance(entry, dict) and entry.get("provider") == "openai":
                        return dict(entry["cost"]) if entry.get("cost") else None
    return None


def _result_text(result: Any) -> str:
    content = result.get("content") if isinstance(result, dict) else None
    if isinstance(content, list):
        return "\n".join(str(c.get("text", "")) for c in content if c.get("type") == "text")
    return str(result)


def _pump(stream: Any, into: queue.Queue[str | None]) -> None:
    """Every line of *stream* onto the queue, then None at EOF."""
    for line in stream:
        into.put(line)
    into.put(None)


class _Events:
    """Pi's ``--mode json`` stream folded into a trajectory: one step per
    assistant ``message_end``, each paired with the request/response the proxy
    recorded for it; each tool result is attached to the call that produced it."""

    def __init__(self, raw_dir: Path, trajectory: TrajectoryBuilder) -> None:
        self.raw_dir = raw_dir
        self.trajectory = trajectory
        self.calls = 0  # model calls seen
        self.wants_tool = False  # the latest one asked for a tool
        self.error = ""
        self._next_seq = 0

    def feed(self, line: str) -> None:
        event = json.loads(line)
        if event.get("type") == "tool_execution_end" and self.calls:
            self.trajectory.observe(str(event["toolCallId"]), _result_text(event.get("result")))
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
        if message.get("stopReason") in ("error", "aborted"):
            self.error = str(message.get("errorMessage") or message["stopReason"])
        tool_calls = tuple(
            ToolCall(
                tool_call_id=str(c["id"]),
                function_name=str(c["name"]),
                arguments=dict(c.get("arguments") or {}),
            )
            for c in content
            if c.get("type") == "toolCall"
        )
        # Pi's ``input`` excludes cache traffic: what was sent is the three together.
        cached = int(usage.get("cacheRead", 0))
        self.trajectory.step(
            message="".join(str(c.get("text", "")) for c in content if c.get("type") == "text"),
            reasoning="\n\n".join(
                str(c.get("thinking", "")) for c in content if c.get("type") == "thinking"
            ),
            tool_calls=tool_calls,
            metrics=Metrics(
                prompt_tokens=int(usage.get("input", 0)) + int(usage.get("cacheWrite", 0)) + cached,
                completion_tokens=int(usage.get("output", 0)),
                cached_tokens=cached,
                cost_usd=float((usage.get("cost") or {}).get("total") or 0.0),
            ),
            model_name=str(message.get("responseModel") or message.get("model") or ""),
            latency_s=float(json.loads(pair[2].read_text()).get("latency_s") or 0.0),
            reasoning_tokens=int(usage.get("reasoning", 0)),
            request=pair[1],
            response=pair[2],
        )
        self.calls += 1
        self.wants_tool = bool(tool_calls)


@dataclass
class Pi:
    """The Pi coding agent, headless (``pi --mode json``), with the run dir as
    its working directory. The case's artifacts and the recording are files
    named in the system prompt; a live robot is reached through ``dimos mcp
    call`` from Pi's ``bash`` tool — Pi has no MCP client, and its authors say
    to give it CLI tools instead, which dimos ships. Model traffic goes through
    a local :class:`RecordingProxy`, so every call is captured whole under
    ``run_dir/raw`` like any other agent's. OpenAI models only, for now.

    The system prompt is composed from fields, in this order: ``system_prompt``
    (the agent's own contract), ``instructions`` (a suite's condition, e.g. an
    access contract; ``""`` for none), the file list, then — with
    ``builtin_guidance`` — how to open the recording and how to call the
    robot's tools, and the robot's tool listing whenever there is a robot.
    Turn ``builtin_guidance`` off to hand that teaching to ``skills`` instead:
    explicit skill files or directories for Pi's repeatable native ``--skill``
    flag, resolved to absolute paths (Pi runs in the case dir); ambient skill
    discovery stays off — ``--no-skills`` disables discovery only, explicit
    paths still load. ``tools`` is Pi's native tool allowlist.
    ``passthrough_env`` names the variables Pi's process inherits, on top of
    every ``DIMOS_*`` one (the dimos CLI it calls needs them).

    ``max_steps`` and the case's time budget are enforced from outside: Pi is
    killed when it has made that many model calls and still wants to act, or
    when the budget runs out, and the steps made so far are kept. ``cli`` is
    the ``pi`` executable.
    """

    model: str = DEFAULT_MODEL
    system_prompt: str = (
        "Answer the question from the files and tools listed below and nothing else."
    )
    instructions: str = ""
    builtin_guidance: bool = True
    tools: Sequence[str] = ("read", "bash", "edit", "write")
    thinking: str = "medium"
    max_steps: int | None = 40
    cli: str = "pi"
    modules: str = ""
    skills: Sequence[str] = ()
    passthrough_env: Sequence[str] = ("PATH", "HOME", "XDG_STATE_HOME", "OPENAI_API_KEY")

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        """Pi's native tools plus robot tools exposed through its bash tool."""
        return (*self.tools, *environment_tools)

    def preflight(self, environment: Environment) -> None:
        for name in ("skills", "tools", "passthrough_env"):
            if isinstance(getattr(self, name), str):
                raise RuntimeError(f"Pi.{name} is a string; pass a list (--set {name}='[...]')")
        missing = [p for p in self.skills if not Path(p).expanduser().resolve().exists()]
        if missing:
            raise RuntimeError(f"Pi skill paths do not exist: {missing}")
        if shutil.which(self.cli) is None:
            raise RuntimeError(
                f"{self.cli!r} is not on PATH (npm install -g @earendil-works/pi-coding-agent)"
            )
        if "OPENAI_API_KEY" not in os.environ:
            raise RuntimeError("Pi needs OPENAI_API_KEY")
        if environment.has_robot and "bash" not in self.tools:
            raise RuntimeError("Pi reaches the robot through its bash tool, which is not enabled")
        if environment.has_robot and shutil.which("dimos") is None:
            raise RuntimeError("Pi reaches the robot through the dimos CLI, which is not on PATH")

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        raw_dir = run_dir / "raw"
        events = _Events(
            raw_dir, TrajectoryBuilder(inputs, name=type(self).__name__, model=self.model)
        )
        upstream = os.environ.get("OPENAI_BASE_URL", "https://api.openai.com/v1")
        with RecordingProxy(raw_dir, upstream) as proxy_url:
            self._write_agent_dir(run_dir, proxy_url)
            ended_by = self._drive(self._command(inputs, env, run_dir), run_dir, events, timeout_s)
        if events.error:
            raise RuntimeError(f"Pi stopped: {events.error}")
        return events.trajectory.build(ended_by)

    def _command(self, inputs: str, env: RunningEnvironment, run_dir: Path) -> list[str]:
        files = dict(env.artifacts)
        if env.streams:  # the selection, not the whole source the artifact names
            files["recording"] = recording_file(env.streams, run_dir / "recording.db")
        parts = [self.system_prompt, self.instructions]
        parts.append("Files:\n" + "\n".join(f"- {name}: {path}" for name, path in files.items()))
        if self.builtin_guidance and "recording" in files:
            parts.append(
                "The recording is a dimos memory store (sqlite). In Python:\n"
                "  from dimos.memory.store.sqlite import SqliteStore\n"
                "  store = SqliteStore(path=PATH, must_exist=True)\n"
                "store.streams.<name> is a stream, .last().data its latest message; iterating "
                "a stream yields observations with .ts and .data. Inspect with dir() and help()."
            )
        if env.mcp_url:
            if self.builtin_guidance:
                parts.append(
                    "The robot is live. Call one of its tools from bash as\n"
                    "  dimos mcp call <tool> --json-args '{\"arg\": value}'"
                )
            parts.append("Tools:\n" + tool_listing(env.mcp_url))
        prompt = "\n\n".join(p for p in parts if p)
        (run_dir / "system-prompt.txt").write_text(prompt)
        # Absolute before Pi changes to the run dir; --no-skills disables only
        # ambient discovery, explicit --skill paths still load.
        skills = [f for p in self.skills for f in ("--skill", str(Path(p).expanduser().resolve()))]
        return [
            self.cli, "--mode", "json", "--model", f"{_PROVIDER}/{self.model}",
            "--thinking", self.thinking, "--session-dir", str(run_dir / "pi-session"),
            "--tools", ",".join(self.tools),
            "--no-extensions", "--no-skills", "--no-prompt-templates", "--no-themes",
            "--no-context-files", "--no-approve", *skills,
            "--system-prompt", prompt, inputs,
        ]  # fmt: skip

    def _write_agent_dir(self, run_dir: Path, proxy_url: str) -> None:
        """A private Pi config dir: one provider, ``dimos``, which is the proxy."""
        agent_dir = run_dir / ".pi-agent"
        agent_dir.mkdir(parents=True, exist_ok=True)
        model: dict[str, Any] = {"id": self.model, "reasoning": True}
        if cost := _registry_cost(self.cli, self.model):
            model["cost"] = cost
        provider = {
            "baseUrl": proxy_url,
            "api": "openai-responses",
            "apiKey": "$OPENAI_API_KEY",
            "models": [model],
        }
        (agent_dir / "models.json").write_text(
            json.dumps({"providers": {_PROVIDER: provider}}, indent=2)
        )

    def _env(self, run_dir: Path) -> dict[str, str]:
        keep = self.passthrough_env
        passed = {k: v for k, v in os.environ.items() if k in keep or k.startswith("DIMOS_")}
        return {
            **passed,
            "PI_CODING_AGENT_DIR": str(run_dir / ".pi-agent"),
            "PI_SKIP_VERSION_CHECK": "1",
            "PI_TELEMETRY": "0",
        }

    def _drive(
        self, command: list[str], run_dir: Path, events: _Events, timeout_s: float
    ) -> EndedBy:
        """Run Pi to completion, or kill it at ``max_steps`` model calls or
        when *timeout_s* runs out. Pi's events are read on a helper thread so
        the wait for the next one can carry the deadline. A Pi that exits on
        its own with a failure status (a rejected flag, say) is an error, not
        an answer."""
        deadline = time.monotonic() + timeout_s
        lines: queue.Queue[str | None] = queue.Queue()
        stderr_path = run_dir / "pi-stderr.txt"
        with (
            stderr_path.open("w") as stderr,
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
            reader = threading.Thread(target=_pump, args=(proc.stdout, lines), daemon=True)
            reader.start()
            try:
                while True:
                    try:
                        line = lines.get(timeout=max(0.0, deadline - time.monotonic()))
                    except queue.Empty:
                        return "timeout"
                    if line is None:
                        break
                    events.feed(line)
                    if self._over_budget(events):
                        return "max_steps"
            finally:
                proc.kill()  # a no-op once Pi has exited on its own
                reader.join()
        if proc.returncode and not events.error:
            events.error = f"exit status {proc.returncode}: {stderr_path.read_text().strip()}"
        return "answer"

    def _over_budget(self, events: _Events) -> bool:
        """``max_steps`` calls made and the last one still asks for a tool."""
        return self.max_steps is not None and events.calls >= self.max_steps and events.wants_tool
