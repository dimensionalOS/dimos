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

from collections.abc import Generator, Sequence
from contextlib import closing
import io
import json
import os
from pathlib import Path
import selectors
import shutil
import subprocess
import time
from typing import IO, TYPE_CHECKING, Any

from pydantic import Field

from dimos.core.coordination.process_lifecycle import kill_run_processes
from dimos.evals.agents.base import Agent, ModelAgentConfig
from dimos.evals.agents.lib.model_trace_proxy import model_trace_proxy
from dimos.evals.agents.lib.pi_to_atif import PiToAtif
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
from dimos.evals.environments.base import Environment
from dimos.evals.types import (
    EndedBy,
    RunningEnvironment,
    Trajectory,
)

if TYPE_CHECKING:
    from dimos.memory.stream import Stream


def tool_listing(mcp_url: str) -> str:
    """List each MCP tool's name, arguments, and first description line for Pi."""
    from dimos.agents.mcp.mcp_adapter import McpAdapter

    lines = []
    for t in McpAdapter(mcp_url).list_tools():
        args = ", ".join((t.get("inputSchema") or {}).get("properties") or {})
        summary = str(t.get("description") or "").strip().partition("\n")[0]
        lines.append(f"- {t['name']}({args}): {summary}")
    return "\n".join(lines)


def recording_file(streams: Sequence[Stream[Any, Any]], path: Path) -> Path:
    """*streams* written to a memory store at *path*, under their own names,
    so a subprocess can open what the case selected and nothing more."""
    from dimos.memory.store.sqlite import SqliteStore

    with SqliteStore(path=str(path)) as store:
        for stream in streams:
            if stream.name is None:
                raise ValueError("a stream must be bound to a store to be written out")
            target: Stream[Any, Any] = store.stream(stream.name, stream.data_type)
            for obs in stream:
                target.append(obs.data, ts=obs.ts, pose=obs.pose_tuple, tags=obs.tags)
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


def read_pi_events(stream: IO[bytes], deadline: float) -> Generator[dict[str, Any], None, None]:
    """Read Pi's JSON events until EOF, raising TimeoutError at the deadline."""
    pending = b""
    with selectors.DefaultSelector() as selector:
        selector.register(stream, selectors.EVENT_READ)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not selector.select(timeout=remaining):
                raise TimeoutError
            chunk = os.read(stream.fileno(), io.DEFAULT_BUFFER_SIZE)
            if not chunk:
                if pending:
                    yield json.loads(pending)
                return
            # Keep partial lines as bytes so split UTF-8 characters remain intact.
            *lines, pending = (pending + chunk).split(b"\n")
            for line in lines:
                if time.monotonic() >= deadline:
                    raise TimeoutError
                yield json.loads(line)


class PiAdapterConfig(ModelAgentConfig):
    """Settings for the headless Pi adapter."""

    # The first section of Pi's system prompt.
    system_prompt: str = (
        "Answer the question from the files and tools listed below and nothing else."
    )

    # Extra guidance appended after system_prompt.
    instructions: str = ""

    # Explain recording access and robot tools. Disable if a skill teaches these.
    builtin_guidance: bool = True

    # Pi's native tools to enable. bash is required for a robot.
    tools: tuple[str, ...] = ("read", "bash", "edit", "write")

    # Reasoning level passed to Pi's --thinking flag.
    thinking: str = "medium"

    # Stop after this many completed model calls if Pi still requests tools,
    # preserving recorded steps. None disables this limit; the timeout still applies.
    max_steps: int | None = 40

    # Grace period for Pi, then remaining tools, before forceful termination.
    shutdown_timeout_s: float = Field(default=2.0, ge=0.0, allow_inf_nan=False)

    # Pi executable name or path.
    cli: str = "pi"

    # Skill files or directories loaded with --skill. Relative paths resolve
    # against the caller's working directory before Pi starts in the case directory.
    skills: tuple[str, ...] = ()

    # Environment variables Pi inherits in addition to every DIMOS_* variable.
    passthrough_env: tuple[str, ...] = ("PATH", "HOME", "XDG_STATE_HOME", "OPENAI_API_KEY")


class PiAdapter(Agent):
    """Run headless Pi against case files and robot tools, recording an ATIF trajectory."""

    config: PiAdapterConfig

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        """Pi's native tools plus robot tools exposed through its bash tool."""
        return (*self.config.tools, *environment_tools)

    def preflight(self, environment: Environment) -> None:
        missing = [p for p in self.config.skills if not Path(p).expanduser().resolve().exists()]
        if missing:
            raise RuntimeError(f"Pi skill paths do not exist: {missing}")
        if shutil.which(self.config.cli) is None:
            raise RuntimeError(
                f"{self.config.cli!r} is not on PATH (npm install -g @earendil-works/pi-coding-agent)"
            )
        if "OPENAI_API_KEY" not in os.environ:
            raise RuntimeError("Pi needs OPENAI_API_KEY")
        if environment.has_robot and "bash" not in self.config.tools:
            raise RuntimeError("Pi reaches the robot through its bash tool, which is not enabled")
        if environment.has_robot and shutil.which("dimos") is None:
            raise RuntimeError("Pi reaches the robot through the dimos CLI, which is not on PATH")

    def run(
        self, inputs: str, env: RunningEnvironment, run_dir: Path, *, timeout_s: float
    ) -> Trajectory:
        raw_dir = run_dir / "raw"
        events = PiToAtif(
            raw_dir, TrajectoryBuilder(inputs, name=type(self).__name__, model=self.config.model)
        )
        upstream = os.environ.get("OPENAI_BASE_URL", "https://api.openai.com/v1")
        with model_trace_proxy(raw_dir, upstream) as proxy_url:
            self._write_model_config(run_dir, proxy_url)
            files = dict(env.artifacts)
            if env.streams:
                files["recording"] = recording_file(env.streams, run_dir / "recording.db")
            system_prompt = self._write_system_prompt(files, env.mcp_url, run_dir)
            command = self._build_pi_command(inputs, system_prompt, run_dir)
            ended_by = self._run_pi_process(command, run_dir, events, timeout_s)
        if events.error:
            raise RuntimeError(f"Pi stopped: {events.error}")
        return events.trajectory.build(ended_by)

    def _write_system_prompt(self, files: dict[str, Path], mcp_url: str, run_dir: Path) -> str:
        parts = [self.config.system_prompt, self.config.instructions]
        parts.append("Files:\n" + "\n".join(f"- {name}: {path}" for name, path in files.items()))
        if self.config.builtin_guidance and "recording" in files:
            parts.append(
                "The recording is a dimos memory store (sqlite). In Python:\n"
                "  from dimos.memory.store.sqlite import SqliteStore\n"
                "  store = SqliteStore(path=PATH, must_exist=True)\n"
                "store.streams.<name> is a stream, .last().data its latest message; iterating "
                "a stream yields observations with .ts and .data. Inspect with dir() and help()."
            )
        if mcp_url:
            if self.config.builtin_guidance:
                parts.append(
                    "The robot is live. Call one of its tools from bash as\n"
                    "  dimos mcp call <tool> --json-args '{\"arg\": value}'"
                )
            parts.append("Tools:\n" + tool_listing(mcp_url))
        prompt = "\n\n".join(p for p in parts if p)
        (run_dir / "system-prompt.txt").write_text(prompt)
        return prompt

    def _build_pi_command(self, inputs: str, system_prompt: str, run_dir: Path) -> list[str]:
        # Absolute before Pi changes to the run dir; --no-skills disables only
        # ambient discovery, explicit --skill paths still load.
        skills = [
            f for p in self.config.skills for f in ("--skill", str(Path(p).expanduser().resolve()))
        ]
        return [
            self.config.cli, "--mode", "json", "--model", f"dimos/{self.config.model}",
            "--thinking", self.config.thinking, "--session-dir", str(run_dir / "pi-session"),
            "--tools", ",".join(self.config.tools),
            "--no-extensions", "--no-skills", "--no-prompt-templates", "--no-themes",
            "--no-context-files", "--no-approve", *skills,
            "--system-prompt", system_prompt, inputs,
        ]  # fmt: skip

    def _write_model_config(self, run_dir: Path, proxy_url: str) -> None:
        """Route the dimos/<model> provider through the model trace proxy."""
        agent_dir = run_dir / ".pi-agent"
        agent_dir.mkdir(parents=True, exist_ok=True)
        model: dict[str, Any] = {"id": self.config.model, "reasoning": True}
        if cost := _registry_cost(self.config.cli, self.config.model):
            model["cost"] = cost
        provider = {
            "baseUrl": proxy_url,
            "api": "openai-responses",
            "apiKey": "$OPENAI_API_KEY",
            "models": [model],
        }
        (agent_dir / "models.json").write_text(
            json.dumps({"providers": {"dimos": provider}}, indent=2)
        )

    def _build_process_env(self, run_dir: Path) -> dict[str, str]:
        keep = self.config.passthrough_env
        passed = {k: v for k, v in os.environ.items() if k in keep or k.startswith("DIMOS_")}
        return {
            **passed,
            "PI_CODING_AGENT_DIR": str(run_dir / ".pi-agent"),
            "PI_SKIP_VERSION_CHECK": "1",
            "PI_TELEMETRY": "0",
        }

    def _run_pi_process(
        self, command: list[str], run_dir: Path, events: PiToAtif, timeout_s: float
    ) -> EndedBy:
        """Run Pi, consume its events, and stop its tools when the run ends."""
        deadline = time.monotonic() + timeout_s
        max_steps = self.config.max_steps
        stderr_path = run_dir / "pi-stderr.txt"
        process_env = self._build_process_env(run_dir)
        with (
            stderr_path.open("w") as stderr,
            subprocess.Popen(
                command,
                cwd=run_dir,
                env=process_env,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=stderr,
                start_new_session=True,
            ) as proc,  # fmt: skip
        ):
            assert proc.stdout is not None
            try:
                with closing(read_pi_events(proc.stdout, deadline)) as event_stream:
                    for event in event_stream:
                        events.append_event(event)
                        if (
                            max_steps is not None
                            and events.calls >= max_steps
                            and events.wants_tool
                        ):
                            return "max_steps"
                # EOF can precede process exit. Preserve Pi's own exit status.
                proc.wait(timeout=max(0.0, deadline - time.monotonic()))
            except (TimeoutError, subprocess.TimeoutExpired):
                return "timeout"
            finally:
                # Pi handles SIGTERM by stopping its active bash process groups.
                proc.terminate()  # a no-op once Pi has exited on its own
                try:
                    proc.wait(timeout=self.config.shutdown_timeout_s)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                # Background tools may already be orphaned or in separate sessions.
                # Their inherited Pi directory identifies this run after Pi exits.
                kill_run_processes(
                    process_env["PI_CODING_AGENT_DIR"],
                    env_var="PI_CODING_AGENT_DIR",
                    exclude_pids=(proc.pid,),
                    term_timeout=self.config.shutdown_timeout_s,
                )
        if proc.returncode and not events.error:
            events.error = f"exit status {proc.returncode}: {stderr_path.read_text().strip()}"
        return "answer"
