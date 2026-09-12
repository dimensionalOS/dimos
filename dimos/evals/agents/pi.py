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
from typing import IO, TYPE_CHECKING, Any, Literal

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

    provider: Literal["openai", "anthropic"] = "openai"
    max_output_tokens: int | None = Field(default=None, ge=1)

    # Shared case guidance appended to Pi's stock system prompt.
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

    # Maximum model HTTP requests, including retries. Zero skips Pi entirely;
    # None disables this limit. Completed steps are preserved when stopping.
    max_steps: int | None = Field(default=40, ge=0)

    # Grace period for Pi, then remaining tools, before forceful termination.
    shutdown_timeout_s: float = Field(default=2.0, ge=0.0, allow_inf_nan=False)

    # Pi executable name or path.
    cli: str = "pi"

    # Skill files or directories loaded with --skill. Relative paths resolve
    # against the caller's working directory before Pi starts in the case directory.
    skills: tuple[str, ...] = ()

    # Environment variables Pi inherits in addition to every DIMOS_* variable.
    passthrough_env: tuple[str, ...] = ("PATH",)


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
        if not os.environ.get(self._key_env):
            raise RuntimeError(f"{type(self).__name__} needs {self._key_env}")
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
        if self.config.max_steps == 0:
            return events.trajectory.build("max_steps")
        upstream = os.environ.get(
            f"{self.config.provider.upper()}_BASE_URL",
            "https://api.openai.com/v1"
            if self.config.provider == "openai"
            else "https://api.anthropic.com",
        )
        limit_reached = run_dir / "pi-request-limit-reached"
        with model_trace_proxy(
            raw_dir, upstream, max_requests=self.config.max_steps, limit_reached=limit_reached
        ) as proxy_url:
            self._write_model_config(run_dir, proxy_url)
            files = self._prepare_files(env, run_dir)
            system_prompt = self._write_system_prompt(files, env.mcp_url, run_dir)
            command = self._build_pi_command(inputs, system_prompt, run_dir)
            try:
                ended_by = self._run_pi_process(command, run_dir, events, timeout_s)
            except Exception as exc:
                ended_by = "error"
                events.error = str(exc)
        if limit_reached.exists():
            return events.trajectory.build("max_steps")
        if events.error and ended_by not in ("timeout", "max_steps"):
            return events.trajectory.build("error", error=events.error)
        return events.trajectory.build(ended_by)

    def _prepare_files(self, env: RunningEnvironment, run_dir: Path) -> dict[str, Path]:
        files = dict(env.artifacts)
        if env.streams:
            files["recording"] = recording_file(env.streams, run_dir / "recording.db")
        return files

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
            self.config.cli, "--mode", "json", "--model", f"{self.config.provider}/{self.config.model}",
            "--thinking", self.config.thinking, "--session-dir", str(run_dir / "pi-session"),
            "--tools", ",".join(self.config.tools),
            "--no-extensions", "--no-skills", "--no-prompt-templates", "--no-themes",
            "--no-context-files", "--no-approve", *skills,
            "--append-system-prompt", system_prompt, inputs,
        ]  # fmt: skip

    @property
    def _key_env(self) -> str:
        return f"{self.config.provider.upper()}_API_KEY"

    def _write_model_config(self, run_dir: Path, proxy_url: str) -> None:
        """Override routing only; Pi owns provider SDKs, capabilities and prices."""
        agent_dir = run_dir / ".pi-agent"
        agent_dir.mkdir(parents=True, exist_ok=True)
        provider: dict[str, Any] = {"baseUrl": proxy_url, "apiKey": f"${self._key_env}"}
        if self.config.max_output_tokens is not None:
            provider["modelOverrides"] = {
                self.config.model: {"maxTokens": self.config.max_output_tokens}
            }
        (agent_dir / "models.json").write_text(
            json.dumps({"providers": {self.config.provider: provider}}, indent=2)
        )

    def _build_process_env(self, run_dir: Path) -> dict[str, str]:
        keep = (*self.config.passthrough_env, self._key_env)
        passed = {k: v for k, v in os.environ.items() if k in keep or k.startswith("DIMOS_")}
        home = run_dir / "home"
        home.mkdir(exist_ok=True)
        return {
            **passed,
            "HOME": str(home),
            "XDG_CONFIG_HOME": str(home / "config"),
            "XDG_STATE_HOME": str(home / "state"),
            "XDG_CACHE_HOME": str(home / "cache"),
            "PI_CODING_AGENT_DIR": str(run_dir / ".pi-agent"),
            "PI_SKIP_VERSION_CHECK": "1",
            "PI_TELEMETRY": "0",
        }

    def _process_events(
        self, proc: subprocess.Popen[bytes], run_dir: Path, deadline: float
    ) -> Generator[dict[str, Any], None, None]:
        assert proc.stdout is not None
        yield from read_pi_events(proc.stdout, deadline)

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
                with closing(self._process_events(proc, run_dir, deadline)) as event_stream:
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
