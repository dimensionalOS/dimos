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
import os
from pathlib import Path
import selectors
import shutil
import subprocess
import tempfile
import time
from typing import IO, TYPE_CHECKING, Any, ClassVar

from pydantic import Field, JsonValue, TypeAdapter, model_validator

from dimos.core.coordination.process_lifecycle import kill_run_processes
from dimos.evals.agents.base import Agent, ModelAgentConfig
from dimos.evals.agents.lib.model_trace_proxy import model_trace_proxy
from dimos.evals.agents.lib.pi_config import (
    PROVIDERS,
    Provider,
    RunPaths,
    RuntimeConfig,
    Thinking,
    ToolPolicyState,
)
from dimos.evals.agents.lib.pi_to_atif import PiToAtif
from dimos.evals.agents.lib.plain_recording import plain_recording
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


_json_object = TypeAdapter(dict[str, JsonValue])


def read_pi_events(
    stream: IO[bytes] | io.RawIOBase, deadline: float
) -> Generator[dict[str, JsonValue], None, None]:
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
                    yield _json_object.validate_json(pending)
                return
            # Keep partial lines as bytes so split UTF-8 characters remain intact.
            *lines, pending = (pending + chunk).split(b"\n")
            if any(len(line) > 16 * 1024 * 1024 for line in (*lines, pending)):
                raise RuntimeError("Pi event exceeded the protocol buffer")
            for line in lines:
                if time.monotonic() >= deadline:
                    raise TimeoutError
                yield _json_object.validate_json(line)


NO_DIMOS_KEYWORDS = ("dimos", "dimensionalos")

NO_DIMOS_GUIDANCE = (
    "There is no robotics framework installed. You may use any public tool, library, SDK or "
    "web resource, and write whatever code you need under the current directory. Do not "
    "install, clone, import or run dimOS or anything from the dimensionalOS organisation; "
    "tool calls that mention it are denied."
)


def raw_robot_readme(endpoint: str) -> str:
    return f"""\
Robot interface: a Zenoh peer at {endpoint}. Connect to it directly (multicast scouting is off);
any Zenoh client works, e.g. `pip install eclipse-zenoh`.

  robot/camera/jpeg        JPEG bytes per frame; attachment is JSON {{"t": unix_seconds}}
  robot/lidar/xyz_f32      float32 little-endian (N,3) x y z in metres, lidar frame; attachment {{"t": ...}}
  robot/odom/json          {{"t","x","y","z","qx","qy","qz","qw"}}: base_link pose in the odom frame
  robot/camera_info/json   {{"width","height","K"}}: intrinsics, republished periodically
  robot/cmd_vel/json       publish {{"vx": m/s, "vy": m/s, "wz": rad/s, "t": seconds}}; the robot holds
                           that velocity for t seconds (max 2), then stops. Republish to keep moving.
                           Speeds are clamped to 1.0 m/s and 1.5 rad/s; non-finite values are ignored.

There is no other interface to this robot.
"""


def _ships_dimos(directory: str) -> bool:
    d = Path(directory)
    return (d / "dimos").exists() or "dimos" in d.parts


class PiAdapterConfig(ModelAgentConfig):
    """Settings for the headless Pi adapter."""

    provider: Provider = "openai"
    max_output_tokens: int | None = Field(default=None, ge=1)

    # Shared case guidance appended to Pi's stock system prompt.
    system_prompt: str = (
        "Answer the question from the files and tools listed below and nothing else."
    )

    # Extra guidance appended after system_prompt.
    instructions: str = ""

    # Explain recording access and robot tools. Disable if a skill teaches these.
    builtin_guidance: bool = True

    # The agent gets the robot as plain topics or files and no dimOS: PATH without dimOS,
    # no memory store or MCP guidance, and excluded_keywords defaulting to dimOS's names.
    no_dimos: bool = False
    # Host variables the Pi process may inherit; the provider key is added by name and
    # DIMOS_* variables pass through unless no_dimos. Everything else stays on the host.
    passthrough_env: tuple[str, ...] = (
        "PATH", "HOME", "LANG", "LC_ALL", "LC_CTYPE", "TERM", "TMPDIR",
        "XDG_CONFIG_HOME", "XDG_STATE_HOME", "XDG_CACHE_HOME", "SSL_CERT_FILE", "SSL_CERT_DIR",
    )  # fmt: skip

    # Reasoning level passed to Pi's --thinking flag.
    thinking: Thinking = "medium"

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

    @model_validator(mode="after")
    def _no_dimos_defaults(self) -> PiAdapterConfig:
        if self.no_dimos and not self.excluded_keywords:
            object.__setattr__(self, "excluded_keywords", NO_DIMOS_KEYWORDS)
        return self


class PiAdapter(Agent):
    """Run headless Pi against case files and robot tools, recording an ATIF trajectory."""

    config: PiAdapterConfig
    default_tools: ClassVar[tuple[str, ...]] = ("read", "bash", "edit", "write")
    tool_names: ClassVar[tuple[str, ...] | None] = (*default_tools, "grep", "find", "ls")
    robot_via_bash: ClassVar[bool] = True

    @property
    def selected_tools(self) -> tuple[str, ...]:
        return (
            self.default_tools if self.config.allowed_tools is None else self.config.allowed_tools
        )

    def validate_tools(self) -> None:
        if self.tool_names is not None:
            unknown = set(self.selected_tools) - set(self.tool_names)
            if unknown:
                raise ValueError(f"Unknown Pi tools: {sorted(unknown)}")
        if self.config.no_dimos and (self.config.modules or self.config.skills):
            raise ValueError("no_dimos cannot add dimOS modules or Pi skills")

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        """Pi's native tools plus robot tools exposed through its bash tool."""
        indirect = (
            environment_tools if "bash" in self.selected_tools and not self.config.no_dimos else ()
        )
        return (*self.selected_tools, *indirect)

    def preflight(self, environment: Environment) -> None:
        self.validate_tools()
        if self.config.no_dimos and environment.has_robot and not environment.provides_raw_robot:
            raise ValueError("no_dimos on a robot environment needs raw_bridge=True")
        missing = [p for p in self.config.skills if not Path(p).expanduser().resolve().exists()]
        if missing:
            raise RuntimeError(f"Pi skill paths do not exist: {missing}")
        if shutil.which(self.config.cli) is None:
            raise RuntimeError(
                f"{self.config.cli!r} is not on PATH (npm install -g @earendil-works/pi-coding-agent)"
            )
        if not os.environ.get(self._key_env):
            raise RuntimeError(f"{type(self).__name__} needs {self._key_env}")
        via_cli = environment.has_robot and self.robot_via_bash and not self.config.no_dimos
        if via_cli and "bash" not in self.selected_tools:
            raise RuntimeError("Pi reaches the robot through its bash tool, which is not enabled")
        if via_cli and shutil.which("dimos") is None:
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
        paths = RunPaths.for_run(run_dir)
        upstream = os.environ.get(f"{self.config.provider.upper()}_BASE_URL", self._base_url)
        limit_reached = run_dir / "pi-request-limit-reached"
        with model_trace_proxy(
            raw_dir, upstream, max_requests=self.config.max_steps, limit_reached=limit_reached
        ) as proxy_url:
            try:
                self._configure(env, paths, proxy_url)
                system_prompt = self._prepare_case(env, run_dir)
                command = self._build_pi_command(inputs, system_prompt, paths)
                ended_by = self._run_pi_process(command, paths, events, timeout_s)
            except Exception as exc:
                ended_by = "error"
                events.error = str(exc)
        blocked = self._blocked_calls(paths)
        if limit_reached.exists():
            return events.trajectory.build("max_steps", blocked_calls=blocked)
        if events.error and ended_by not in ("timeout", "max_steps"):
            return events.trajectory.build("error", error=events.error, blocked_calls=blocked)
        return events.trajectory.build(ended_by, blocked_calls=blocked)

    def _blocked_calls(self, paths: RunPaths) -> int:
        ready = paths.config / "extensions/runtime-ready.json"
        return (
            ToolPolicyState.model_validate_json(ready.read_text()).blocked if ready.is_file() else 0
        )

    def _prepare_case(self, env: RunningEnvironment, run_dir: Path) -> str:
        if self.config.no_dimos:
            files = self._no_dimos_files(env, run_dir)
        else:
            files = dict(env.artifacts)
            if env.streams:
                files["recording"] = recording_file(env.streams, run_dir / "recording.db")
        parts = [self.config.system_prompt, self.config.instructions]
        parts.append("Files:\n" + "\n".join(f"- {name}: {path}" for name, path in files.items()))
        if self.config.no_dimos:
            parts.append(NO_DIMOS_GUIDANCE)
        elif self.config.builtin_guidance and "recording" in files:
            parts.append(
                "The recording is a dimos memory store (sqlite). In Python:\n"
                "  from dimos.memory.store.sqlite import SqliteStore\n"
                "  store = SqliteStore(path=PATH, must_exist=True)\n"
                "store.streams.<name> is a stream, .last().data its latest message; iterating "
                "a stream yields observations with .ts and .data. Inspect with dir() and help()."
            )
        if env.mcp_url and not self.config.no_dimos:
            if self.config.builtin_guidance:
                parts.append(
                    "The robot is live. Call one of its tools from bash as\n"
                    "  dimos mcp call <tool> --json-args '{\"arg\": value}'"
                )
            parts.append("Tools:\n" + tool_listing(env.mcp_url))
        prompt = "\n\n".join(p for p in parts if p)
        (run_dir / "system-prompt.txt").write_text(prompt)
        return prompt

    def _no_dimos_files(self, env: RunningEnvironment, run_dir: Path) -> dict[str, Path]:
        """Robot cases: live topics only (ROBOT.md). Dataset cases: the selected observations
        exported as plain files, since the memory store needs dimOS to read."""
        files = dict(env.artifacts)
        files.pop("recording", None)  # a dimOS memory store; not readable without dimOS
        if env.raw_endpoint:
            readme = run_dir / "ROBOT.md"
            readme.write_text(raw_robot_readme(env.raw_endpoint))
            files["robot"] = readme
        elif env.mcp_url:
            raise ValueError("no_dimos on a robot environment needs raw_bridge=True")
        if env.streams:
            files["observations"] = plain_recording(env.streams, run_dir / "input")
        return files

    def _build_pi_command(self, inputs: str, system_prompt: str, paths: RunPaths) -> list[str]:
        # Absolute before Pi changes to the run dir; --no-skills disables only
        # ambient discovery, explicit --skill paths still load.
        skills = [
            f for p in self.config.skills for f in ("--skill", str(Path(p).expanduser().resolve()))
        ]
        tool_args = (
            ["--tools", ",".join(self.selected_tools)] if self.selected_tools else ["--no-tools"]
        )
        policy = ["--extension", str(paths.config / "extensions/runtime.js")]
        return [
            shutil.which(self.config.cli) or self.config.cli,  # no_dimos strips PATH dirs
            "--mode", "json", "--model", f"{self.config.provider}/{self.config.model}",
            "--thinking", self.config.thinking, "--session-dir", str(paths.workspace / "pi-session"),
            *tool_args, *policy,
            "--no-extensions", "--no-skills", "--no-prompt-templates", "--no-themes",
            "--no-context-files", "--no-approve", *skills,
            "--append-system-prompt", system_prompt, inputs,
        ]  # fmt: skip

    @property
    def _key_env(self) -> str:
        return PROVIDERS[self.config.provider][0]

    @property
    def _base_url(self) -> str:
        return PROVIDERS[self.config.provider][1]

    def _configure(self, env: RunningEnvironment, paths: RunPaths, proxy_url: str) -> None:
        config = RuntimeConfig(
            provider=self.config.provider,
            base_url=proxy_url,
            key_env=self._key_env,
            allowed_tools=self.config.allowed_tools,
            max_output_tokens=self.config.max_output_tokens,
            excluded_keywords=self.config.excluded_keywords,
            ignored_paths=(str(paths.workspace), str(paths.config)),
            max_tool_seconds=self.config.max_tool_seconds,
        )
        extensions = paths.config / "extensions"
        extensions.mkdir(parents=True, exist_ok=True)
        (extensions / "runtime-ready.json").unlink(missing_ok=True)
        shutil.copyfile(Path(__file__).parent / "lib/runtime.js", extensions / "runtime.js")
        (extensions / "runtime.json").write_text(config.model_dump_json())

    def _check_tools_ready(self, paths: RunPaths) -> None:
        ready = paths.config / "extensions/runtime-ready.json"
        if not ready.is_file():
            raise RuntimeError("Pi runtime extension failed to load")
        state = ToolPolicyState.model_validate_json(ready.read_text())
        if state.unknown or (
            self.config.allowed_tools is not None and state.tools != self.selected_tools
        ):
            raise ValueError(f"Tool allowlist was not applied: {state}")

    def _build_process_env(self, paths: RunPaths) -> dict[str, str]:
        keep = {*self.config.passthrough_env, self._key_env}
        dimos_vars = not self.config.no_dimos
        env = {
            k: v
            for k, v in os.environ.items()
            if k in keep or (dimos_vars and k.startswith("DIMOS_"))
        }
        if self.config.no_dimos:
            env["PATH"] = os.pathsep.join(
                d for d in env.get("PATH", "").split(os.pathsep) if not _ships_dimos(d)
            )
        env["DIMOS_EVAL_RUN_ID"] = str(paths.workspace)
        return env

    def _process_events(
        self, proc: subprocess.Popen[bytes], paths: RunPaths, deadline: float
    ) -> Generator[dict[str, JsonValue], None, None]:
        assert proc.stdout is not None
        yield from read_pi_events(proc.stdout, deadline)
        self._check_tools_ready(paths)

    def _run_pi_process(
        self, command: list[str], paths: RunPaths, events: PiToAtif, timeout_s: float
    ) -> EndedBy:
        """Run Pi, consume its events, and stop its tools when the run ends."""
        deadline = time.monotonic() + timeout_s
        max_steps = self.config.max_steps
        stderr_path = paths.workspace / "pi-stderr.txt"
        paths.cache.mkdir(parents=True, exist_ok=True)
        # The runtime dir hosts Unix sockets, whose paths are capped near 100 bytes; a
        # deeply nested cache (pytest tmp dirs on CI) falls back to the system temp dir.
        parent = (
            paths.cache if len(str(paths.cache / "run-XXXXXXXX" / "dimcode.sock")) < 96 else None
        )
        with tempfile.TemporaryDirectory(prefix="run-", dir=parent) as runtime:
            self._runtime_dir = Path(runtime)
            process_env = self._build_process_env(paths)
            process_env["XDG_RUNTIME_DIR"] = runtime
            with (
                stderr_path.open("w") as stderr,
                subprocess.Popen(
                    command,
                    cwd=paths.workspace,
                    env=process_env,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=stderr,
                    start_new_session=True,
                ) as proc,  # fmt: skip
            ):
                assert proc.stdout is not None
                try:
                    with closing(self._process_events(proc, paths, deadline)) as event_stream:
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
                    # The inherited run ID identifies them after Pi exits.
                    kill_run_processes(
                        str(paths.workspace),
                        env_var="DIMOS_EVAL_RUN_ID",
                        exclude_pids=(proc.pid,),
                        term_timeout=self.config.shutdown_timeout_s,
                    )
        if proc.returncode and not events.error:
            events.error = f"exit status {proc.returncode}: {stderr_path.read_text().strip()}"
        return "answer"
