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

"""The DimOS agent (`dimos agent run`) as an eval agent."""

from __future__ import annotations

import atexit
from collections.abc import Generator, Iterator
from contextlib import contextmanager
from dataclasses import replace
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
from typing import Annotated, Any, ClassVar, Literal
from urllib.parse import urlsplit

from pydantic import BaseModel, ConfigDict, Field, JsonValue, TypeAdapter
from pydantic.alias_generators import to_camel

from dimos.evals.active_learning import load_active_learning
from dimos.evals.agents.lib.model_trace_proxy import model_trace_proxy
from dimos.evals.agents.lib.pi_config import RunPaths
from dimos.evals.agents.pi import PiAdapter, PiAdapterConfig, read_pi_events
from dimos.evals.environments.base import Environment
from dimos.evals.types import RunningEnvironment
from dimos.memory.query import dump_caption_jsonl
from dimos.memory.store.sqlite import SqliteStore
from dimos.perception.identity.dump import TRACKS_FILE, sidecar_tracks_jsonl


class AssistantItem(BaseModel):
    kind: Literal["assistant"]
    message: dict[str, JsonValue]


class ToolResult(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel)

    tool_call_id: str
    content: list[JsonValue]


class ToolResultItem(BaseModel):
    kind: Literal["tool_result"]
    result: ToolResult


class OtherItem(BaseModel):
    kind: Literal["input", "user", "compaction", "summary", "reset"]


class ItemLine(BaseModel):
    item: Annotated[AssistantItem | ToolResultItem | OtherItem, Field(discriminator="kind")]


class Done(BaseModel):
    status: Literal["done"]


class Unanswered(BaseModel):
    status: Literal["unanswered"]
    reason: str


class OutcomeLine(BaseModel):
    outcome: Annotated[Done | Unanswered, Field(discriminator="status")]


_run_line: TypeAdapter[ItemLine | OutcomeLine] = TypeAdapter(ItemLine | OutcomeLine)


def _unserved_port() -> int:
    """A local TCP port that nothing listens on."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


# Case guidance for recording evals. `dimos agent run` has no --append-system-prompt, so
# `_build_pi_command` puts this in the user message (InputGate defaultMaxChars is 4000).
# AL stays a workspace file with an absolute-path pointer (consult-on-demand memory).
USER_PROMPT_MAX_CHARS = 4000

MEMORY_INDEX = """\
The recording is a DimOS memory store. Do not open the sqlite file from bash.
t is seconds from the first color_image frame, not a Unix timestamp.
1 Hz captions: captions.jsonl, or captions/YYYY-MM-DD.jsonl via captions/manifest.jsonl. Each line: {"t", "setting", "people_count", "people", "objects_and_text"}.
Tracks when present (tracks.jsonl or tracks/YYYY-MM-DD.jsonl): {"t", "tracks": [{"id", "name", "xyxy"}]}. Same id nearby is the same object.
Search with rg if on PATH, else grep -E. Filter on t. First/last = head/tail of grep. Do not cat a whole captions file. "people" is on every line — use "people_count": [1-9] or "people": [{.
"""

MEMORY_RULES = """\
Stay in the case working directory. Use the absolute caption / active-learning paths in this message. Do not find under $DIMOS_ROOT, Desktop/dimensional, /mnt/c, or historical runs. Do not search the repo for those files or for skills — skill paths are already in the system prompt.
Index first, several rounds: whole recording, then synonyms and at-t / first / last / between. Later hits are other candidates. A gap of several seconds is a new episode. If choices are timestamps, inspect those t values. Unique-count: distinct track ids; at most one inspect per id.
Must inspect (≥1) before answering when the question needs color, motion, barriers, door numbers, open/closed, or left/right spatial judgment. Pure counts and first/last timestamp choices may answer from the index alone.
When the question names a +Ns window, inspect only t in that window — not earlier/later segments. For first/last, take the head/tail grep hit's t only — do not scan neighbors. Prefer one multi-t inspect (up to four t → strip/2x2) over many calls; at most two inspect calls.
For multiple choice, match the inspected image, not the first grep line. Before the final answer, pause: restate the question and competing choices; name the index hits and inspected t; say why the rival choice loses. If the last thought named a different choice, inspect one more t in the same window or keep that choice. Then answer.
If the question lists no choices, answer in a short phrase and do not invent options.
inspect returns the color frame only, never the 1 Hz caption. Pass ask covering both competing answers: some direct, some that would refute the pick.
"""

MEMORY_GUIDANCE = f"{MEMORY_INDEX.rstrip()}\n{MEMORY_RULES}"


def _workspace_file(paths: RunPaths, name: str) -> Path | None:
    path = (paths.workspace / name).resolve()
    return path if path.is_file() else None


def recording_user_prompt(inputs: str, paths: RunPaths) -> str:
    """Question + absolute workspace pointers + memory protocol for `dimos agent run`."""
    parts = [inputs.rstrip()]
    captions = _workspace_file(paths, "captions.jsonl")
    manifest = paths.workspace / "captions" / "manifest.jsonl"
    if captions is not None:
        parts.append(f"Captions index: {captions}")
    elif manifest.is_file():
        parts.append(f"Captions manifest: {manifest.resolve()}")
    al = _workspace_file(paths, "active-learning.md")
    if al is not None:
        parts.append(f"Active-learning notes: {al}\nRead that file before answering.")
    parts.append(MEMORY_GUIDANCE.rstrip())
    prompt = "\n\n".join(parts) + "\n"
    if len(prompt) > USER_PROMPT_MAX_CHARS:
        raise ValueError(
            f"recording user prompt is {len(prompt)} chars; InputGate cuts at "
            f"{USER_PROMPT_MAX_CHARS}. Shorten MEMORY_GUIDANCE or the question."
        )
    return prompt


def _copy_sidecar_tracks(env: RunningEnvironment, run_dir: Path) -> None:
    """Copy precomputed tracks.jsonl from next to the source recording, if present."""
    src = env.artifacts.get("recording")
    if src is None:
        return
    sidecar = sidecar_tracks_jsonl(Path(str(src)))
    if sidecar.is_file():
        shutil.copy2(sidecar, run_dir / TRACKS_FILE)


def _path_with_user_local_bin(path: str) -> str:
    """Put ~/.local/bin first so eval bash sees a user-installed `rg`."""
    local_bin = Path.home() / ".local" / "bin"
    if not local_bin.is_dir():
        return path
    prefix = str(local_bin)
    parts = [p for p in path.split(os.pathsep) if p and p != prefix]
    return os.pathsep.join((prefix, *parts))


class DimosAgentAdapterConfig(PiAdapterConfig):
    cli: str = "dimos"
    """The `dimos` executable whose `dimos agent run` answers each case."""

    builtin_guidance: bool = False
    """Pi's SqliteStore snippet; this agent has the memory tool instead."""

    active_learning: str = ""
    """Named pack in ``dimos.evals.active_learning`` or a prompt-file path. Empty is off."""


class DimosAgentAdapter(PiAdapter):
    """One DimOS agent gateway per eval process, driven through `dimos agent run --json`.

    The gateway keeps its config, state, cache and socket in directories shared across
    cases so Node and skills stay warm. Each case is a new conversation (`--cwd` the case
    dir). Model requests go to one trace proxy through models.json; per-case traces are
    copied into the case `raw/` dir. The agent keeps its own system prompt, tools and
    skills. Case files dump to ``system-prompt.txt`` for debugging. ``dimos agent run``
    has no ``--append-system-prompt``; recording cases put memory guidance and absolute
    workspace pointers in the user message (must stay under ``USER_PROMPT_MAX_CHARS``).
    """

    config: DimosAgentAdapterConfig
    requires_mcp = True

    # The harness's tools without DimOS; inventory and agent_module join once the gateway links.
    default_tools: ClassVar[tuple[str, ...]] = (
        "read", "bash", "edit", "write", "stream_get", "memory", "job", "background",
    )  # fmt: skip
    tool_names: ClassVar[tuple[str, ...] | None] = None
    robot_via_bash: ClassVar[bool] = False

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        """The harness's tools. The gateway, not the case's McpServer, decides the rest, and the
        runner records them from the raw requests."""
        return self.default_tools

    def validate_tools(self) -> None:
        name = type(self).__name__
        if self.config.allowed_tools is not None:
            raise ValueError(f"{name} does not support allowed_tools: its tool set is fixed")
        if self.config.no_dimos:
            raise ValueError(f"{name} does not support no_dimos: the DimOS agent is dimOS")
        if self.config.skills:
            raise ValueError(f"{name} does not support skills: it reads its own skills directory")
        if self.config.excluded_keywords:
            raise ValueError(
                f"{name} does not support excluded_keywords: it has no live tool guard"
            )
        if "max_tool_seconds" in self.config.model_fields_set:
            raise ValueError(
                f"{name} does not support max_tool_seconds: its bash has no per-call time limit"
            )

    def preflight(self, environment: Environment) -> None:
        super().preflight(environment)
        # Checks Node and builds the agent when its inputs changed, before any case starts.
        version = subprocess.run(
            [shutil.which(self.config.cli) or self.config.cli, "agent", "version"],
            capture_output=True,
            text=True,
        )
        if version.returncode:
            tail = "\n".join(version.stderr.strip().splitlines()[-20:])
            raise RuntimeError(f"`{self.config.cli} agent version` failed:\n{tail}")

    def _prepare_case(self, env: RunningEnvironment, run_dir: Path) -> str:
        # The agent reaches the robot through its own tools and prompt, not an MCP tool listing.
        prompt = super()._prepare_case(replace(env, mcp_url=""), run_dir)
        recording = run_dir / "recording.db"
        if recording.is_file():
            with SqliteStore(path=str(recording), must_exist=True) as store:
                if "color_image_caption" in store.streams:
                    dump_caption_jsonl(store, run_dir)
            _copy_sidecar_tracks(env, run_dir)
            prompt = f"{prompt}\n\n{MEMORY_INDEX.rstrip()}"
            al = self.config.active_learning.strip()
            if al:
                text = load_active_learning(al).rstrip()
                (run_dir / "active-learning.md").write_text(text + "\n")
            prompt = f"{prompt}\n\n{MEMORY_RULES.rstrip()}"
            (run_dir / "system-prompt.txt").write_text(prompt)
        return prompt

    def _configure(self, env: RunningEnvironment, paths: RunPaths, proxy_url: str) -> None:
        _ = paths
        # Without a robot, a port nothing serves keeps the gateway off any other DimOS on this host.
        mcp = urlsplit(env.mcp_url or f"http://127.0.0.1:{_unserved_port()}/mcp")
        self._mcp_env = {"MCPSERVER__HOST": str(mcp.hostname), "MCPSERVER__PORT": str(mcp.port)}
        provider: dict[str, JsonValue] = {"baseUrl": proxy_url, "apiKey": "$DIMOS_EVAL_API_KEY"}
        if self.config.max_output_tokens is not None:
            provider["modelOverrides"] = {
                self.config.model: {"maxTokens": self.config.max_output_tokens}
            }
        agent_config = self._ensure_eval_runtime() / "config" / "dimos" / "agent"
        agent_config.mkdir(parents=True, exist_ok=True)
        (agent_config / "config.json").write_text(
            json.dumps(
                {
                    "model": {"provider": self.config.provider, "modelId": self.config.model},
                    "thinkingLevel": self.config.thinking,
                }
            )
        )
        (agent_config / "models.json").write_text(
            json.dumps({"providers": {self.config.provider: provider}})
        )

    def _build_pi_command(self, inputs: str, system_prompt: str, paths: RunPaths) -> list[str]:
        _ = system_prompt
        recording = (paths.workspace / "recording.db").is_file()
        prompt = recording_user_prompt(inputs, paths) if recording else inputs
        return [
            shutil.which(self.config.cli) or self.config.cli,
            "agent", "run", "--json", "--cwd", str(paths.workspace),
            prompt,
        ]  # fmt: skip

    def _build_process_env(self, paths: RunPaths) -> dict[str, str]:
        home = self._ensure_eval_runtime()
        env = {
            **super()._build_process_env(paths),
            **self._mcp_env,
            "XDG_CONFIG_HOME": str(home / "config"),
            "XDG_STATE_HOME": str(home / "state"),
            "XDG_CACHE_HOME": str(home / "cache"),
            "XDG_RUNTIME_DIR": str(home / "runtime"),
            "DIMOS_AGENT_SKIP_BUILD": "1",
            # Match InputGate / OutputLimiter headroom for stuffed recording user prompts.
            "DIMOS_AGENT_DEFAULT_MAX_CHARS": str(USER_PROMPT_MAX_CHARS),
            # `dimos agent` drops variables whose value it loaded from .env, the provider key included.
            "DIMOS_EVAL_API_KEY": os.environ[self._key_env],
        }
        path = env.get("PATH") or os.environ.get("PATH") or "/usr/bin:/bin"
        env["PATH"] = _path_with_user_local_bin(path)
        return env

    def _ensure_eval_runtime(self) -> Path:
        home = getattr(self, "_eval_home", None)
        if home is not None:
            return home
        home = Path(tempfile.mkdtemp(prefix="dimos-eval-agent-"))
        (home / "config").mkdir()
        (home / "state").mkdir()
        (home / "cache").mkdir()
        (home / "runtime").mkdir()
        self._eval_home = home
        self._proxy_cm: Any = None
        self._proxy_url: str | None = None
        self._shared_raw: Path | None = None
        atexit.register(self._shutdown_eval_runtime)
        return home

    def _event_raw_dir(self, raw_dir: Path) -> Path:
        """Read traces from the shared proxy dir; seed it with any files already in case raw/."""
        home = self._ensure_eval_runtime()
        shared_raw = home / "raw"
        shared_raw.mkdir(parents=True, exist_ok=True)
        self._shared_raw = shared_raw
        raw_dir.mkdir(parents=True, exist_ok=True)
        for trace in raw_dir.iterdir():
            if trace.is_file():
                shutil.copy2(trace, shared_raw / trace.name)
        return shared_raw

    def _shutdown_eval_runtime(self) -> None:
        home = getattr(self, "_eval_home", None)
        if home is None:
            return
        cli = shutil.which(self.config.cli) or self.config.cli
        env = {
            **os.environ,
            "XDG_CONFIG_HOME": str(home / "config"),
            "XDG_STATE_HOME": str(home / "state"),
            "XDG_CACHE_HOME": str(home / "cache"),
            "XDG_RUNTIME_DIR": str(home / "runtime"),
        }
        try:
            subprocess.run(
                [cli, "agent", "stop"], env=env, capture_output=True, timeout=30, check=False
            )
        except (OSError, subprocess.TimeoutExpired):
            pass
        proxy = getattr(self, "_proxy_cm", None)
        if proxy is not None:
            proxy.__exit__(None, None, None)
            self._proxy_cm = None
        shutil.rmtree(home, ignore_errors=True)
        self._eval_home = None

    @contextmanager
    def _trace_proxy(self, raw_dir: Path, upstream: str, limit_reached: Path) -> Iterator[str]:
        home = self._ensure_eval_runtime()
        if self._proxy_cm is None:
            shared_raw = home / "raw"
            shared_raw.mkdir(parents=True, exist_ok=True)
            self._shared_raw = shared_raw
            self._proxy_cm = model_trace_proxy(
                shared_raw, upstream, max_requests=None, limit_reached=home / "limit-reached"
            )
            self._proxy_url = self._proxy_cm.__enter__()
        assert self._proxy_url is not None and self._shared_raw is not None
        before = {p.name for p in self._shared_raw.iterdir()}
        try:
            yield self._proxy_url
        finally:
            raw_dir.mkdir(parents=True, exist_ok=True)
            for trace in self._shared_raw.iterdir():
                if trace.name not in before:
                    shutil.copy2(trace, raw_dir / trace.name)

    def _process_events(
        self, proc: subprocess.Popen[bytes], paths: RunPaths, deadline: float
    ) -> Generator[dict[str, JsonValue], None, None]:
        assert proc.stdout is not None
        for record in read_pi_events(proc.stdout, deadline):
            line = _run_line.validate_python(record)
            if isinstance(line, OutcomeLine):
                if isinstance(line.outcome, Unanswered):
                    raise RuntimeError(f"dimos agent run ended unanswered: {line.outcome.reason}")
            elif isinstance(line.item, AssistantItem):
                yield {"type": "message_end", "message": line.item.message}
            elif isinstance(line.item, ToolResultItem):
                result = line.item.result
                yield {
                    "type": "tool_execution_end",
                    "toolCallId": result.tool_call_id,
                    "result": {"content": result.content},
                }
